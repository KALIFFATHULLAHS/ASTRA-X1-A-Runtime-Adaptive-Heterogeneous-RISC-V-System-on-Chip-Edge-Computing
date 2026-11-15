`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 Edge-AI Accelerator: 4x4 INT8 Systolic Array
// Architecture:
//   - 16 Processing Elements in a 4x4 2D Mesh
//   - Mode: Output-Stationary (OS)
//   - Input Slices: 4 Activation inputs (A0..A3), 4 Weight inputs (B0..B3)
//   - Internal Skew Buffering: Automatically staggers rows/cols by 1, 2, 3, 4 cycles
//     ensuring exact data-valid alignment across all rows and columns.
//   - Outputs: 16 Signed 32-bit Accumulators C[row, col]
//   - Throughput: 16 Multiply-Accumulate operations (32 Ops) per clock cycle
//=============================================================================

module npu_systolic_array #(
    parameter ROWS = 4,
    parameter COLS = 4
)(
    input  wire        clk,
    input  wire        reset,

    // Execution Control
    input  wire        start,          // Pulse high to begin matrix multiplication
    input  wire [15:0] k_dim,          // Inner dimension K (number of MAC steps, e.g. 4, 8, 16...)
    output reg         busy,           // High while computation is in progress
    output reg         done,           // Pulsed high for 1 cycle when all PEs finish

    // Parallel Input Stream (Supplied simultaneously at each cycle k = 0 .. K-1)
    input  wire        data_valid,     // Valid qualifier for current input slice
    input  wire signed [7:0] in_a0,    // Row 0 activation
    input  wire signed [7:0] in_a1,    // Row 1 activation
    input  wire signed [7:0] in_a2,    // Row 2 activation
    input  wire signed [7:0] in_a3,    // Row 3 activation

    input  wire signed [7:0] in_b0,    // Col 0 weight
    input  wire signed [7:0] in_b1,    // Col 1 weight
    input  wire signed [7:0] in_b2,    // Col 2 weight
    input  wire signed [7:0] in_b3,    // Col 3 weight

    // Readout Interface (Indexed 32-bit register read)
    input  wire [1:0]  read_row,       // Select row 0..3
    input  wire [1:0]  read_col,       // Select column 0..3
    output wire signed [31:0] read_data,

    // Flattened 512-bit parallel output (all 16 32-bit accumulators)
    output wire [511:0] flat_c_matrix
);

    //=========================================================================
    // 1. SYMMETRIC SKEW BUFFERS (Data & Valid strictly matched)
    // Row 0 / Col 0: 1 cycle delay
    // Row 1 / Col 1: 2 cycles delay
    // Row 2 / Col 2: 3 cycles delay
    // Row 3 / Col 3: 4 cycles delay
    //=========================================================================

    // Row A Skew Registers
    reg signed [7:0] reg_a0;
    reg              reg_val_a0;

    reg signed [7:0] a1_q1, reg_a1;
    reg              val_a1_q1, reg_val_a1;

    reg signed [7:0] a2_q1, a2_q2, reg_a2;
    reg              val_a2_q1, val_a2_q2, reg_val_a2;

    reg signed [7:0] a3_q1, a3_q2, a3_q3, reg_a3;
    reg              val_a3_q1, val_a3_q2, val_a3_q3, reg_val_a3;

    // Col B Skew Registers
    reg signed [7:0] reg_b0;
    reg              reg_val_b0;

    reg signed [7:0] b1_q1, reg_b1;
    reg              val_b1_q1, reg_val_b1;

    reg signed [7:0] b2_q1, b2_q2, reg_b2;
    reg              val_b2_q1, val_b2_q2, reg_val_b2;

    reg signed [7:0] b3_q1, b3_q2, b3_q3, reg_b3;
    reg              val_b3_q1, val_b3_q2, val_b3_q3, reg_val_b3;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            reg_a0     <= 8'sd0; reg_val_a0 <= 1'b0;
            a1_q1      <= 8'sd0; reg_a1     <= 8'sd0; val_a1_q1 <= 1'b0; reg_val_a1 <= 1'b0;
            a2_q1      <= 8'sd0; a2_q2      <= 8'sd0; reg_a2    <= 8'sd0;
            val_a2_q1  <= 1'b0;  val_a2_q2  <= 1'b0;  reg_val_a2 <= 1'b0;
            a3_q1      <= 8'sd0; a3_q2      <= 8'sd0; a3_q3     <= 8'sd0; reg_a3 <= 8'sd0;
            val_a3_q1  <= 1'b0;  val_a3_q2  <= 1'b0;  val_a3_q3 <= 1'b0;  reg_val_a3 <= 1'b0;

            reg_b0     <= 8'sd0; reg_val_b0 <= 1'b0;
            b1_q1      <= 8'sd0; reg_b1     <= 8'sd0; val_b1_q1 <= 1'b0; reg_val_b1 <= 1'b0;
            b2_q1      <= 8'sd0; b2_q2      <= 8'sd0; reg_b2    <= 8'sd0;
            val_b2_q1  <= 1'b0;  val_b2_q2  <= 1'b0;  reg_val_b2 <= 1'b0;
            b3_q1      <= 8'sd0; b3_q2      <= 8'sd0; b3_q3     <= 8'sd0; reg_b3 <= 8'sd0;
            val_b3_q1  <= 1'b0;  val_b3_q2  <= 1'b0;  val_b3_q3 <= 1'b0;  reg_val_b3 <= 1'b0;
        end else begin
            // Row 0: 1 cycle
            reg_a0     <= in_a0;
            reg_val_a0 <= data_valid;

            // Row 1: 2 cycles
            a1_q1      <= in_a1;
            reg_a1     <= a1_q1;
            val_a1_q1  <= data_valid;
            reg_val_a1 <= val_a1_q1;

            // Row 2: 3 cycles
            a2_q1      <= in_a2;
            a2_q2      <= a2_q1;
            reg_a2     <= a2_q2;
            val_a2_q1  <= data_valid;
            val_a2_q2  <= val_a2_q1;
            reg_val_a2 <= val_a2_q2;

            // Row 3: 4 cycles
            a3_q1      <= in_a3;
            a3_q2      <= a3_q1;
            a3_q3      <= a3_q2;
            reg_a3     <= a3_q3;
            val_a3_q1  <= data_valid;
            val_a3_q2  <= val_a3_q1;
            val_a3_q3  <= val_a3_q2;
            reg_val_a3 <= val_a3_q3;

            // Col 0: 1 cycle
            reg_b0     <= in_b0;
            reg_val_b0 <= data_valid;

            // Col 1: 2 cycles
            b1_q1      <= in_b1;
            reg_b1     <= b1_q1;
            val_b1_q1  <= data_valid;
            reg_val_b1 <= val_b1_q1;

            // Col 2: 3 cycles
            b2_q1      <= in_b2;
            b2_q2      <= b2_q1;
            reg_b2     <= b2_q2;
            val_b2_q1  <= data_valid;
            val_b2_q2  <= val_b2_q1;
            reg_val_b2 <= val_b2_q2;

            // Col 3: 4 cycles
            b3_q1      <= in_b3;
            b3_q2      <= b3_q1;
            b3_q3      <= b3_q2;
            reg_b3     <= b3_q3;
            val_b3_q1  <= data_valid;
            val_b3_q2  <= val_b3_q1;
            val_b3_q3  <= val_b3_q2;
            reg_val_b3 <= val_b3_q3;
        end
    end

    //=========================================================================
    // 2. PE INTERCONNECT SIGNALS
    //=========================================================================

    wire signed [7:0]  pe_a [0:3][0:4];     // Horizontal activation wires (col 0 to 4)
    wire               pe_val_a [0:3][0:4];

    wire signed [7:0]  pe_b [0:4][0:3];     // Vertical weight wires (row 0 to 4)
    wire               pe_val_b [0:4][0:3];

    wire signed [31:0] pe_acc [0:3][0:3];   // 16 Accumulator outputs

    // Connect West boundaries to skewed inputs
    assign pe_a[0][0]     = reg_a0;
    assign pe_val_a[0][0] = reg_val_a0;
    assign pe_a[1][0]     = reg_a1;
    assign pe_val_a[1][0] = reg_val_a1;
    assign pe_a[2][0]     = reg_a2;
    assign pe_val_a[2][0] = reg_val_a2;
    assign pe_a[3][0]     = reg_a3;
    assign pe_val_a[3][0] = reg_val_a3;

    // Connect North boundaries to skewed inputs
    assign pe_b[0][0]     = reg_b0;
    assign pe_val_b[0][0] = reg_val_b0;
    assign pe_b[0][1]     = reg_b1;
    assign pe_val_b[0][1] = reg_val_b1;
    assign pe_b[0][2]     = reg_b2;
    assign pe_val_b[0][2] = reg_val_b2;
    assign pe_b[0][3]     = reg_b3;
    assign pe_val_b[0][3] = reg_val_b3;

    //=========================================================================
    // 3. 4x4 PROCESSING ELEMENT GRID INSTANTIATION
    //=========================================================================

    reg clr_accumulators;

    genvar r, c;
    generate
        for (r = 0; r < 4; r = r + 1) begin: gen_pe_rows
            for (c = 0; c < 4; c = c + 1) begin: gen_pe_cols
                npu_pe pe_inst (
                    .clk       (clk),
                    .reset     (reset),
                    .en        (1'b1),
                    .clr_acc   (clr_accumulators),

                    .in_a      (pe_a[r][c]),
                    .val_a     (pe_val_a[r][c]),

                    .in_b      (pe_b[r][c]),
                    .val_b     (pe_val_b[r][c]),

                    .out_a     (pe_a[r][c+1]),
                    .out_val_a (pe_val_a[r][c+1]),

                    .out_b     (pe_b[r+1][c]),
                    .out_val_b (pe_val_b[r+1][c]),

                    .acc_out   (pe_acc[r][c])
                );
            end
        end
    endgenerate

    // Flattened outputs for easy probing and bus binding
    generate
        for (r = 0; r < 4; r = r + 1) begin: gen_flat_rows
            for (c = 0; c < 4; c = c + 1) begin: gen_flat_cols
                assign flat_c_matrix[((r*4 + c)*32) +: 32] = pe_acc[r][c];
            end
        end
    endgenerate

    // Indexed multiplexer for reading specific C[row, col]
    assign read_data = pe_acc[read_row][read_col];

    //=========================================================================
    // 4. CONTROL STATE MACHINE & COMPLETION TIMER
    // Total execution latency = K steps + 4 (skew latency) + 6 (array traversal) + 2
    //=========================================================================

    localparam S_IDLE  = 2'd0;
    localparam S_RUN   = 2'd1;
    localparam S_DONE  = 2'd2;

    reg [1:0]  state;
    reg [15:0] cycle_cnt;
    reg [15:0] total_cycles;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state            <= S_IDLE;
            busy             <= 1'b0;
            done             <= 1'b0;
            clr_accumulators <= 1'b0;
            cycle_cnt        <= 16'd0;
            total_cycles     <= 16'd0;
        end else begin
            done <= 1'b0; // default pulse

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        clr_accumulators <= 1'b1; // Clear PE accumulators for new calculation
                        total_cycles     <= k_dim + 16'd12; // K cycles + 4 skew + 6 traversal + margin
                        cycle_cnt        <= 16'd0;
                        busy             <= 1'b1;
                        state            <= S_RUN;
                    end else begin
                        clr_accumulators <= 1'b0;
                    end
                end

                S_RUN: begin
                    clr_accumulators <= 1'b0;
                    cycle_cnt        <= cycle_cnt + 1'b1;

                    if (cycle_cnt >= total_cycles) begin
                        busy  <= 1'b0;
                        done  <= 1'b1; // Signal calculation complete
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    done  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
