`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: Full NPU Accelerator Wishbone Slave (Milestone 4)
// Architecture:
//   - 4x4 INT8 Systolic Array Core (16 PEs, 1.6 GOPS @ 50 MHz)
//   - Integrated Dual-Port Scratchpad Memory (Activations, Weights, Outputs)
//   - Autonomous Hardware Feed FSM (Zero CPU overhead during computation)
//   - Full backward compatibility with Milestone 2 Mock NPU register contract
//
// Register & Buffer Memory Map (Base Address: 0x4000_0000):
//   0x0000 [RO] NPU_REG_ID      : 32'h4153_5452 ("ASTR")
//   0x0004 [RO] NPU_REG_STATUS  : [0]=Ready, [1]=Done
//   0x0008 [RW] NPU_REG_SCRATCH : 32-bit General Scratchpad (100% M2 backward-compatible)
//   0x000C [RW] NPU_REG_CTRL    : Write 1 to bit 0 starts execution
//   0x0010 [RW] NPU_REG_K_DIM   : Inner reduction dimension K (Default = 4)
//   0x0014 [RO] NPU_REG_CYCLES  : Number of clock cycles taken by last execution
//
//   0x0040 - 0x007C [RO]: Output Matrix C (16 words = 4x4 INT32 accumulators)
//     Offset 0x0040 + (r*4 + c)*4 => C[r, c]
//
//   0x0100 - 0x01FF [RW]: Activation Buffer A (256 bytes)
//     Offset 0x0100 + (r*K + k) => A[r, k]
//
//   0x0200 - 0x02FF [RW]: Weight Buffer B (256 bytes)
//     Offset 0x0200 + (k*4 + c) => B[k, c]
//
//   0x0300 - 0x03FF [RW]: Output Scratchpad Buffer C (64 bytes = 16 words)
//=============================================================================

module wb_astra_npu (
    input  wire        clk,
    input  wire        reset,

    // Wishbone B4 Slave Interface
    input  wire [31:0] wb_adr_i,
    input  wire [31:0] wb_dat_i,
    input  wire [3:0]  wb_sel_i,
    input  wire        wb_we_i,
    input  wire        wb_stb_i,
    input  wire        wb_cyc_i,

    output reg  [31:0] wb_dat_o,
    output reg         wb_ack_o,

    // Interrupt & Activity Status outputs
    output reg         npu_irq,
    output wire        npu_busy_o
);

    wire _unused_sel = &{1'b0, wb_sel_i};

    // Address Decoding
    wire is_req = wb_cyc_i && wb_stb_i && !wb_ack_o;
    wire [15:0] offset = wb_adr_i[15:0];

    //-------------------------------------------------------------------------
    // Control Registers
    //-------------------------------------------------------------------------
    localparam [31:0] NPU_ID_CONST = 32'h4153_5452; // "ASTR"

    reg [31:0] reg_scratch;
    reg [15:0] reg_k_dim;
    reg [31:0] reg_cycles;
    reg        reg_busy;
    reg        reg_done;

    assign npu_busy_o = reg_busy;
    reg        start_pulse;

    //-------------------------------------------------------------------------
    // Internal Scratchpad Memories:
    // Buffer A: 256 bytes (Activations: up to 4 rows x 64 columns)
    // Buffer B: 256 bytes (Weights:     up to 64 rows x 4 columns)
    //-------------------------------------------------------------------------
    reg signed [7:0] buf_a [0:255];
    reg signed [7:0] buf_b [0:255];

    // Initialize buffers to 0
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            buf_a[i] = 8'sd0;
            buf_b[i] = 8'sd0;
        end
    end

    //-------------------------------------------------------------------------
    // 4x4 Systolic Array Instance
    //-------------------------------------------------------------------------
    reg        array_start;
    reg [15:0] array_k_dim;
    wire       array_busy;
    wire       array_done;

    reg        array_valid;
    reg signed [7:0] arr_a0, arr_a1, arr_a2, arr_a3;
    reg signed [7:0] arr_b0, arr_b1, arr_b2, arr_b3;

    reg  [1:0] read_row;
    reg  [1:0] read_col;
    wire signed [31:0] arr_read_data;
    wire [511:0] flat_c_matrix;

    npu_systolic_array #(
        .ROWS(4),
        .COLS(4)
    ) u_systolic (
        .clk           (clk),
        .reset         (reset),
        .start         (array_start),
        .k_dim         (array_k_dim),
        .busy          (array_busy),
        .done          (array_done),

        .data_valid    (array_valid),
        .in_a0         (arr_a0),
        .in_a1         (arr_a1),
        .in_a2         (arr_a2),
        .in_a3         (arr_a3),

        .in_b0         (arr_b0),
        .in_b1         (arr_b1),
        .in_b2         (arr_b2),
        .in_b3         (arr_b3),

        .read_row      (read_row),
        .read_col      (read_col),
        .read_data     (arr_read_data),
        .flat_c_matrix (flat_c_matrix)
    );

    // Latched Output C Matrix (16 words)
    reg signed [31:0] c_matrix [0:3][0:3];

    //-------------------------------------------------------------------------
    // Autonomous Hardware Feeder FSM
    // Streams slices A[:, k] and B[k, :] cycle-by-cycle into the systolic array
    //-------------------------------------------------------------------------
    localparam FSM_IDLE   = 2'd0;
    localparam FSM_FEED   = 2'd1;
    localparam FSM_WAIT   = 2'd2;
    localparam FSM_LATCH  = 2'd3;

    reg [1:0]  fsm_state;
    reg [15:0] k_cnt;
    reg [31:0] cycle_timer;

    // Row-major index generation for Matrix A [4 x K]: A[r, k] = r * K + k
    wire [7:0] a_stride = array_k_dim[7:0];
    wire [7:0] a_r0_idx = k_cnt[7:0];
    wire [7:0] a_r1_idx = a_stride + k_cnt[7:0];
    wire [7:0] a_r2_idx = {a_stride[6:0], 1'b0} + k_cnt[7:0];
    wire [7:0] a_r3_idx = {a_stride[6:0], 1'b0} + a_stride + k_cnt[7:0];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            fsm_state        <= FSM_IDLE;
            array_start      <= 1'b0;
            array_valid      <= 1'b0;
            array_k_dim      <= 16'd4;
            arr_a0           <= 8'sd0;
            arr_a1           <= 8'sd0;
            arr_a2           <= 8'sd0;
            arr_a3           <= 8'sd0;
            arr_b0           <= 8'sd0;
            arr_b1           <= 8'sd0;
            arr_b2           <= 8'sd0;
            arr_b3           <= 8'sd0;
            k_cnt            <= 16'd0;
            cycle_timer      <= 32'd0;
            reg_cycles       <= 32'd0;
            reg_busy         <= 1'b0;
            reg_done         <= 1'b0;
            npu_irq          <= 1'b0;
        end else begin
            array_start <= 1'b0;
            npu_irq     <= 1'b0;

            case (fsm_state)
                FSM_IDLE: begin
                    reg_busy <= 1'b0;
                    if (start_pulse) begin
                        reg_busy    <= 1'b1;
                        reg_done    <= 1'b0;
                        array_k_dim <= reg_k_dim;
                        array_start <= 1'b1; // Trigger array accumulator clear and run
                        k_cnt       <= 16'd0;
                        cycle_timer <= 32'd0;
                        fsm_state   <= FSM_FEED;
                    end
                end

                FSM_FEED: begin
                    cycle_timer <= cycle_timer + 1'b1;

                    if (k_cnt < array_k_dim) begin
                        array_valid <= 1'b1;
                        // Feed Column k of Matrix A: A[0, k], A[1, k], A[2, k], A[3, k]
                        // Matrix A layout in buf_a: contiguous row-major => A[r, k] = buf_a[r * K + k]
                        arr_a0 <= buf_a[a_r0_idx];
                        arr_a1 <= buf_a[a_r1_idx];
                        arr_a2 <= buf_a[a_r2_idx];
                        arr_a3 <= buf_a[a_r3_idx];

                        // Feed Row k of Matrix B: B[k, 0], B[k, 1], B[k, 2], B[k, 3]
                        // Matrix B layout in buf_b: contiguous row-major => B[k, c] = buf_b[k * 4 + c]
                        arr_b0 <= buf_b[{k_cnt[5:0], 2'd0}];
                        arr_b1 <= buf_b[{k_cnt[5:0], 2'd1}];
                        arr_b2 <= buf_b[{k_cnt[5:0], 2'd2}];
                        arr_b3 <= buf_b[{k_cnt[5:0], 2'd3}];

                        k_cnt  <= k_cnt + 1'b1;
                    end else begin
                        array_valid <= 1'b0;
                        arr_a0 <= 8'sd0; arr_a1 <= 8'sd0; arr_a2 <= 8'sd0; arr_a3 <= 8'sd0;
                        arr_b0 <= 8'sd0; arr_b1 <= 8'sd0; arr_b2 <= 8'sd0; arr_b3 <= 8'sd0;
                        fsm_state   <= FSM_WAIT;
                    end
                end

                FSM_WAIT: begin
                    cycle_timer <= cycle_timer + 1'b1;
                    if (array_done) begin
                        fsm_state <= FSM_LATCH;
                    end
                end

                FSM_LATCH: begin
                    // Latch all 16 accumulators into accessible registers
                    c_matrix[0][0] <= flat_c_matrix[31:0];
                    c_matrix[0][1] <= flat_c_matrix[63:32];
                    c_matrix[0][2] <= flat_c_matrix[95:64];
                    c_matrix[0][3] <= flat_c_matrix[127:96];

                    c_matrix[1][0] <= flat_c_matrix[159:128];
                    c_matrix[1][1] <= flat_c_matrix[191:160];
                    c_matrix[1][2] <= flat_c_matrix[223:192];
                    c_matrix[1][3] <= flat_c_matrix[255:224];

                    c_matrix[2][0] <= flat_c_matrix[287:256];
                    c_matrix[2][1] <= flat_c_matrix[319:288];
                    c_matrix[2][2] <= flat_c_matrix[351:320];
                    c_matrix[2][3] <= flat_c_matrix[383:352];

                    c_matrix[3][0] <= flat_c_matrix[415:384];
                    c_matrix[3][1] <= flat_c_matrix[447:416];
                    c_matrix[3][2] <= flat_c_matrix[479:448];
                    c_matrix[3][3] <= flat_c_matrix[511:480];

                    reg_cycles <= cycle_timer;
                    reg_busy   <= 1'b0;
                    reg_done   <= 1'b1;
                    npu_irq    <= 1'b1; // Trigger interrupt on completion
                    fsm_state  <= FSM_IDLE;
                end

                default: fsm_state <= FSM_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // Wishbone Bus Read & Write Handling
    //-------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            wb_ack_o    <= 1'b0;
            wb_dat_o    <= 32'd0;
            reg_scratch <= 32'h0000_0000;
            reg_k_dim   <= 16'd4;
            start_pulse <= 1'b0;
            read_row    <= 2'd0;
            read_col    <= 2'd0;
        end else begin
            start_pulse <= 1'b0; // default pulse

            if (is_req) begin
                wb_ack_o <= 1'b1;

                if (wb_we_i) begin
                    // --------------------------------------------------------
                    // WRITE TRANSACTION
                    // --------------------------------------------------------
                    if (offset < 16'h0040) begin
                        case (offset[5:2])
                            4'h0: ; // 0x00: ID is Read-Only
                            4'h1: ; // 0x04: STATUS is Read-Only
                            4'h2: reg_scratch <= wb_dat_i; // 0x08: SCRATCH REGISTER (Backward compatible with M2)
                            4'h3: begin                     // 0x0C: NPU_REG_CTRL
                                if (wb_dat_i[0]) start_pulse <= 1'b1;
                            end
                            4'h4: reg_k_dim   <= wb_dat_i[15:0]; // 0x10: NPU_REG_K_DIM
                            4'h5: ;                              // 0x14: CYCLES is Read-Only
                            default: ;
                        endcase
                    end else if (offset >= 16'h0100 && offset < 16'h0200) begin
                        // Activation Buffer A write (Offset 0x0100..0x01FF, word-packed 4 bytes)
                        if (wb_sel_i[0]) buf_a[{offset[7:2], 2'b00}] <= wb_dat_i[7:0];
                        if (wb_sel_i[1]) buf_a[{offset[7:2], 2'b01}] <= wb_dat_i[15:8];
                        if (wb_sel_i[2]) buf_a[{offset[7:2], 2'b10}] <= wb_dat_i[23:16];
                        if (wb_sel_i[3]) buf_a[{offset[7:2], 2'b11}] <= wb_dat_i[31:24];
                    end else if (offset >= 16'h0200 && offset < 16'h0300) begin
                        // Weight Buffer B write (Offset 0x0200..0x02FF, word-packed 4 bytes)
                        if (wb_sel_i[0]) buf_b[{offset[7:2], 2'b00}] <= wb_dat_i[7:0];
                        if (wb_sel_i[1]) buf_b[{offset[7:2], 2'b01}] <= wb_dat_i[15:8];
                        if (wb_sel_i[2]) buf_b[{offset[7:2], 2'b10}] <= wb_dat_i[23:16];
                        if (wb_sel_i[3]) buf_b[{offset[7:2], 2'b11}] <= wb_dat_i[31:24];
                    end
                end else begin
                    // --------------------------------------------------------
                    // READ TRANSACTION
                    // --------------------------------------------------------
                    if (offset < 16'h0040) begin
                        case (offset[5:2])
                            4'h0: wb_dat_o <= NPU_ID_CONST;              // 0x00: "ASTR"
                            4'h1: wb_dat_o <= {30'd0, reg_done, !reg_busy}; // 0x04: [0]=READY, [1]=DONE
                            4'h2: wb_dat_o <= reg_scratch;              // 0x08: SCRATCH REGISTER (M2 compatibility)
                            4'h3: wb_dat_o <= {31'd0, reg_busy};        // 0x0C: CTRL/BUSY
                            4'h4: wb_dat_o <= {16'd0, reg_k_dim};       // 0x10: K_DIM
                            4'h5: wb_dat_o <= reg_cycles;               // 0x14: CYCLES
                            default: wb_dat_o <= 32'd0;
                        endcase
                    end else if (offset >= 16'h0040 && offset < 16'h0080) begin
                        // Output Matrix C Direct Register Read (0x0040..0x007C)
                        // Offset: 0x40 + (r*4 + c)*4
                        wb_dat_o <= c_matrix[offset[5:4]][offset[3:2]];
                    end else if (offset >= 16'h0100 && offset < 16'h0200) begin
                        // Buffer A Read (packed 4 bytes)
                        wb_dat_o <= { buf_a[{offset[7:2], 2'b11}],
                                      buf_a[{offset[7:2], 2'b10}],
                                      buf_a[{offset[7:2], 2'b01}],
                                      buf_a[{offset[7:2], 2'b00}] };
                    end else if (offset >= 16'h0200 && offset < 16'h0300) begin
                        // Buffer B Read (packed 4 bytes)
                        wb_dat_o <= { buf_b[{offset[7:2], 2'b11}],
                                      buf_b[{offset[7:2], 2'b10}],
                                      buf_b[{offset[7:2], 2'b01}],
                                      buf_b[{offset[7:2], 2'b00}] };
                    end else begin
                        wb_dat_o <= 32'd0;
                    end
                end
            end else begin
                wb_ack_o <= 1'b0;
            end
        end
    end

endmodule
