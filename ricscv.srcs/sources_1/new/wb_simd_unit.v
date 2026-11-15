`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: 4-Lane 16-bit Packed SIMD Vector Accelerator (Milestone 6A)
// Base Address: 0x4000_3000 (Slave 5 on Wishbone B4 Interconnect)
//
// Features:
//   - 4 parallel 16-bit signed integer lanes (64-bit packed vector per cycle)
//   - Operations (OP_CODE in CTRL[5:2]):
//       4'd0 : ADD16  -> Y[i] = A[i] + B[i], ACC += sum(Y[0..3])
//       4'd1 : SUB16  -> Y[i] = A[i] - B[i], ACC += sum(Y[0..3])
//       4'd2 : MUL16  -> Y[i] = (A[i] * B[i])[15:0], ACC += sum(A[i]*B[i])
//       4'd3 : MAC16  -> Y[i] = (A[i] * B[i])[15:0], ACC += sum(A[i]*B[i])
//       4'd4 : ABS16  -> Y[i] = |A[i]|, ACC += sum(|A[0..3]|)
//       4'd5 : RELU16 -> Y[i] = max(0, A[i]), ACC += sum(Y[0..3])
//       4'd6 : MAX16  -> Y[i] = max(A[i], B[i]), ACC += sum(Y[0..3])
//       4'd7 : MIN16  -> Y[i] = min(A[i], B[i]), ACC += sum(Y[0..3])
//   - Dual execution modes:
//       VEC_MODE = 0 (CTRL[8]==0): Single 4-lane register operation on VEC_A/VEC_B (1 cycle)
//       VEC_MODE = 1 (CTRL[8]==1): Autonomous streaming over SIMD_LEN 4-lane vectors
//                                  from SIMD_BUF_A (0x100) & SIMD_BUF_B (0x200) -> SIMD_BUF_Y (0x300)
//
// Register Map (Offset from 0x4000_3000):
//   0x0000 [RO] SIMD_REG_ID     : 32'h5349_4D44 ("SIMD")
//   0x0004 [RO] SIMD_REG_STATUS : [0]=Ready (!busy), [1]=Done
//   0x0008 [RW] SIMD_REG_CTRL   : [0]=Start, [1]=Clear ACC/Status, [5:2]=OP_CODE, [8]=VEC_MODE
//   0x000C [RW] SIMD_REG_LEN    : Number of 4-lane (64-bit) vectors to stream (1..8)
//   0x0010 [RW] SIMD_VEC_A0     : {Lane1[15:0], Lane0[15:0]}
//   0x0014 [RW] SIMD_VEC_A1     : {Lane3[15:0], Lane2[15:0]}
//   0x0018 [RW] SIMD_VEC_B0     : {Lane1[15:0], Lane0[15:0]}
//   0x001C [RW] SIMD_VEC_B1     : {Lane3[15:0], Lane2[15:0]}
//   0x0020 [RO] SIMD_RES_Y0     : {OutLane1[15:0], OutLane0[15:0]}
//   0x0024 [RO] SIMD_RES_Y1     : {OutLane3[15:0], OutLane2[15:0]}
//   0x0028 [RO] SIMD_REG_ACC    : Signed 32-bit cross-lane reduction accumulator
//   0x002C [RO] SIMD_REG_CYCLES : Hardware execution cycles for last operation
//   0x0100..0x013C [RW] SIMD_BUF_A[0..15] : 32 x INT16 Vector A Scratchpad
//   0x0200..0x023C [RW] SIMD_BUF_B[0..15] : 32 x INT16 Vector B Scratchpad
//   0x0300..0x033C [RO] SIMD_BUF_Y[0..15] : 32 x INT16 Output Vector Y Scratchpad
//=============================================================================

module wb_simd_unit (
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

    // Activity Status for ASTRA-DAAQ Power Controller
    output wire        simd_busy_o
);

    wire _unused = &{1'b0, wb_adr_i[31:10], wb_adr_i[1:0], wb_sel_i};

    localparam [31:0] SIMD_ID_CONST = 32'h5349_4D44; // "SIMD"

    // Operation Codes
    localparam [3:0] OP_ADD16  = 4'd0;
    localparam [3:0] OP_SUB16  = 4'd1;
    localparam [3:0] OP_MUL16  = 4'd2;
    localparam [3:0] OP_MAC16  = 4'd3;
    localparam [3:0] OP_ABS16  = 4'd4;
    localparam [3:0] OP_RELU16 = 4'd5;
    localparam [3:0] OP_MAX16  = 4'd6;
    localparam [3:0] OP_MIN16  = 4'd7;

    // Registers
    reg [31:0] reg_vec_a0, reg_vec_a1;
    reg [31:0] reg_vec_b0, reg_vec_b1;
    reg [31:0] reg_res_y0, reg_res_y1;
    reg signed [31:0] reg_acc;
    reg [31:0] reg_cycles;
    reg [3:0]  reg_len;
    reg [3:0]  reg_opcode;
    reg        reg_vec_mode;
    reg        reg_busy;
    reg        reg_done;

    // Vector Scratchpads (16 words = 32 x INT16 = 8 x 64-bit vectors each)
    reg [31:0] buf_a [0:15];
    reg [31:0] buf_b [0:15];
    reg [31:0] buf_y [0:15];

    assign simd_busy_o = reg_busy;

    // Address Decoding
    wire is_req   = wb_cyc_i && wb_stb_i && !wb_ack_o;
    wire [1:0] region = wb_adr_i[9:8]; // 0=Ctrl, 1=BUF_A, 2=BUF_B, 3=BUF_Y
    wire [3:0] word_idx = wb_adr_i[5:2];

    // Streaming FSM State
    reg [3:0] vec_idx;
    reg       start_pulse;
    reg       clr_pulse;

    // Active 4-Lane Inputs (selected from registers or streaming buffer)
    wire [31:0] cur_a0 = reg_vec_mode ? buf_a[{vec_idx[2:0], 1'b0}] : reg_vec_a0;
    wire [31:0] cur_a1 = reg_vec_mode ? buf_a[{vec_idx[2:0], 1'b1}] : reg_vec_a1;
    wire [31:0] cur_b0 = reg_vec_mode ? buf_b[{vec_idx[2:0], 1'b0}] : reg_vec_b0;
    wire [31:0] cur_b1 = reg_vec_mode ? buf_b[{vec_idx[2:0], 1'b1}] : reg_vec_b1;

    wire signed [15:0] a_lane0 = cur_a0[15:0];
    wire signed [15:0] a_lane1 = cur_a0[31:16];
    wire signed [15:0] a_lane2 = cur_a1[15:0];
    wire signed [15:0] a_lane3 = cur_a1[31:16];

    wire signed [15:0] b_lane0 = cur_b0[15:0];
    wire signed [15:0] b_lane1 = cur_b0[31:16];
    wire signed [15:0] b_lane2 = cur_b1[15:0];
    wire signed [15:0] b_lane3 = cur_b1[31:16];

    // 4-Lane Parallel Arithmetic Datapath
    wire signed [31:0] prod0 = a_lane0 * b_lane0;
    wire signed [31:0] prod1 = a_lane1 * b_lane1;
    wire signed [31:0] prod2 = a_lane2 * b_lane2;
    wire signed [31:0] prod3 = a_lane3 * b_lane3;

    reg signed [15:0] y_lane0, y_lane1, y_lane2, y_lane3;
    reg signed [31:0] lane_sum;

    always @(*) begin
        case (reg_opcode)
            OP_ADD16: begin
                y_lane0  = a_lane0 + b_lane0;
                y_lane1  = a_lane1 + b_lane1;
                y_lane2  = a_lane2 + b_lane2;
                y_lane3  = a_lane3 + b_lane3;
                lane_sum = {{16{y_lane0[15]}}, y_lane0} + {{16{y_lane1[15]}}, y_lane1} +
                           {{16{y_lane2[15]}}, y_lane2} + {{16{y_lane3[15]}}, y_lane3};
            end
            OP_SUB16: begin
                y_lane0  = a_lane0 - b_lane0;
                y_lane1  = a_lane1 - b_lane1;
                y_lane2  = a_lane2 - b_lane2;
                y_lane3  = a_lane3 - b_lane3;
                lane_sum = {{16{y_lane0[15]}}, y_lane0} + {{16{y_lane1[15]}}, y_lane1} +
                           {{16{y_lane2[15]}}, y_lane2} + {{16{y_lane3[15]}}, y_lane3};
            end
            OP_MUL16, OP_MAC16: begin
                y_lane0  = prod0[15:0];
                y_lane1  = prod1[15:0];
                y_lane2  = prod2[15:0];
                y_lane3  = prod3[15:0];
                lane_sum = prod0 + prod1 + prod2 + prod3;
            end
            OP_ABS16: begin
                y_lane0  = (a_lane0 < 0) ? -a_lane0 : a_lane0;
                y_lane1  = (a_lane1 < 0) ? -a_lane1 : a_lane1;
                y_lane2  = (a_lane2 < 0) ? -a_lane2 : a_lane2;
                y_lane3  = (a_lane3 < 0) ? -a_lane3 : a_lane3;
                lane_sum = {{16{y_lane0[15]}}, y_lane0} + {{16{y_lane1[15]}}, y_lane1} +
                           {{16{y_lane2[15]}}, y_lane2} + {{16{y_lane3[15]}}, y_lane3};
            end
            OP_RELU16: begin
                y_lane0  = (a_lane0 < 0) ? 16'sd0 : a_lane0;
                y_lane1  = (a_lane1 < 0) ? 16'sd0 : a_lane1;
                y_lane2  = (a_lane2 < 0) ? 16'sd0 : a_lane2;
                y_lane3  = (a_lane3 < 0) ? 16'sd0 : a_lane3;
                lane_sum = {{16{y_lane0[15]}}, y_lane0} + {{16{y_lane1[15]}}, y_lane1} +
                           {{16{y_lane2[15]}}, y_lane2} + {{16{y_lane3[15]}}, y_lane3};
            end
            OP_MAX16: begin
                y_lane0  = (a_lane0 > b_lane0) ? a_lane0 : b_lane0;
                y_lane1  = (a_lane1 > b_lane1) ? a_lane1 : b_lane1;
                y_lane2  = (a_lane2 > b_lane2) ? a_lane2 : b_lane2;
                y_lane3  = (a_lane3 > b_lane3) ? a_lane3 : b_lane3;
                lane_sum = {{16{y_lane0[15]}}, y_lane0} + {{16{y_lane1[15]}}, y_lane1} +
                           {{16{y_lane2[15]}}, y_lane2} + {{16{y_lane3[15]}}, y_lane3};
            end
            OP_MIN16: begin
                y_lane0  = (a_lane0 < b_lane0) ? a_lane0 : b_lane0;
                y_lane1  = (a_lane1 < b_lane1) ? a_lane1 : b_lane1;
                y_lane2  = (a_lane2 < b_lane2) ? a_lane2 : b_lane2;
                y_lane3  = (a_lane3 < b_lane3) ? a_lane3 : b_lane3;
                lane_sum = {{16{y_lane0[15]}}, y_lane0} + {{16{y_lane1[15]}}, y_lane1} +
                           {{16{y_lane2[15]}}, y_lane2} + {{16{y_lane3[15]}}, y_lane3};
            end
            default: begin
                y_lane0  = 16'sd0;
                y_lane1  = 16'sd0;
                y_lane2  = 16'sd0;
                y_lane3  = 16'sd0;
                lane_sum = 32'sd0;
            end
        endcase
    end

    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            wb_ack_o     <= 1'b0;
            wb_dat_o     <= 32'd0;
            reg_vec_a0   <= 32'd0;
            reg_vec_a1   <= 32'd0;
            reg_vec_b0   <= 32'd0;
            reg_vec_b1   <= 32'd0;
            reg_res_y0   <= 32'd0;
            reg_res_y1   <= 32'd0;
            reg_acc      <= 32'sd0;
            reg_cycles   <= 32'd0;
            reg_len      <= 4'd1;
            reg_opcode   <= 4'd0;
            reg_vec_mode <= 1'b0;
            reg_busy     <= 1'b0;
            reg_done     <= 1'b0;
            vec_idx      <= 4'd0;
            start_pulse  <= 1'b0;
            clr_pulse    <= 1'b0;
            for (i = 0; i < 16; i = i + 1) begin
                buf_a[i] <= 32'd0;
                buf_b[i] <= 32'd0;
                buf_y[i] <= 32'd0;
            end
        end else begin
            start_pulse <= 1'b0;
            clr_pulse   <= 1'b0;

            if (clr_pulse && !reg_busy) begin
                reg_acc    <= 32'sd0;
                reg_done   <= 1'b0;
                reg_cycles <= 32'd0;
            end

            // Execution Engine
            if (start_pulse && !reg_busy) begin
                if (!reg_vec_mode) begin
                    // Single 4-lane 64-bit register operation completes in 1 cycle
                    reg_res_y0 <= {y_lane1, y_lane0};
                    reg_res_y1 <= {y_lane3, y_lane2};
                    if (reg_opcode == OP_MAC16)
                        reg_acc <= reg_acc + lane_sum;
                    else
                        reg_acc <= lane_sum;
                    reg_cycles <= 32'd1;
                    reg_done   <= 1'b1;
                    reg_busy   <= 1'b0;
                end else begin
                    // Start multi-vector autonomous streaming
                    vec_idx    <= 4'd0;
                    reg_cycles <= 32'd0;
                    reg_busy   <= 1'b1;
                    reg_done   <= 1'b0;
                end
            end else if (reg_busy) begin
                // Process 1 four-lane (64-bit) vector per clock cycle
                buf_y[{vec_idx[2:0], 1'b0}] <= {y_lane1, y_lane0};
                buf_y[{vec_idx[2:0], 1'b1}] <= {y_lane3, y_lane2};
                reg_res_y0                  <= {y_lane1, y_lane0};
                reg_res_y1                  <= {y_lane3, y_lane2};
                reg_acc                     <= reg_acc + lane_sum;
                reg_cycles                  <= reg_cycles + 1'b1;

                if (vec_idx + 4'd1 >= reg_len) begin
                    reg_busy <= 1'b0;
                    reg_done <= 1'b1;
                end else begin
                    vec_idx <= vec_idx + 4'd1;
                end
            end

            // Wishbone Slave Read / Write Interface
            if (is_req) begin
                wb_ack_o <= 1'b1;
                if (wb_we_i) begin
                    case (region)
                        2'b00: begin
                            case (word_idx)
                                4'h2: begin // 0x08: SIMD_REG_CTRL
                                    reg_opcode   <= wb_dat_i[5:2];
                                    reg_vec_mode <= wb_dat_i[8];
                                    if (wb_dat_i[0] && !reg_busy) start_pulse <= 1'b1;
                                    if (wb_dat_i[1])              clr_pulse   <= 1'b1;
                                end
                                4'h3: reg_len    <= (wb_dat_i[3:0] == 4'd0) ? 4'd1 : wb_dat_i[3:0]; // 0x0C
                                4'h4: reg_vec_a0 <= wb_dat_i; // 0x10
                                4'h5: reg_vec_a1 <= wb_dat_i; // 0x14
                                4'h6: reg_vec_b0 <= wb_dat_i; // 0x18
                                4'h7: reg_vec_b1 <= wb_dat_i; // 0x1C
                                default: ;
                            endcase
                        end
                        2'b01: buf_a[word_idx] <= wb_dat_i; // 0x100..0x13C
                        2'b10: buf_b[word_idx] <= wb_dat_i; // 0x200..0x23C
                        default: ;
                    endcase
                end else begin
                    case (region)
                        2'b00: begin
                            case (word_idx)
                                4'h0: wb_dat_o <= SIMD_ID_CONST;                          // 0x00: "SIMD"
                                4'h1: wb_dat_o <= {30'd0, reg_done, !reg_busy};           // 0x04: STATUS
                                4'h2: wb_dat_o <= {23'd0, reg_vec_mode, 2'd0, reg_opcode, 1'b0, reg_busy};
                                4'h3: wb_dat_o <= {28'd0, reg_len};                       // 0x0C: LEN
                                4'h4: wb_dat_o <= reg_vec_a0;                             // 0x10: VEC_A0
                                4'h5: wb_dat_o <= reg_vec_a1;                             // 0x14: VEC_A1
                                4'h6: wb_dat_o <= reg_vec_b0;                             // 0x18: VEC_B0
                                4'h7: wb_dat_o <= reg_vec_b1;                             // 0x1C: VEC_B1
                                4'h8: wb_dat_o <= reg_res_y0;                             // 0x20: RES_Y0
                                4'h9: wb_dat_o <= reg_res_y1;                             // 0x24: RES_Y1
                                4'hA: wb_dat_o <= reg_acc;                                // 0x28: ACC
                                4'hB: wb_dat_o <= reg_cycles;                             // 0x2C: CYCLES
                                default: wb_dat_o <= 32'd0;
                            endcase
                        end
                        2'b01: wb_dat_o <= buf_a[word_idx];
                        2'b10: wb_dat_o <= buf_b[word_idx];
                        2'b11: wb_dat_o <= buf_y[word_idx];
                    endcase
                end
            end else begin
                wb_ack_o <= 1'b0;
            end
        end
    end

endmodule
