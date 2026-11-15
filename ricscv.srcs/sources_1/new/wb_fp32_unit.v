`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: IEEE-754 Single-Precision (Binary32) FP32 Engine & Timer (Milestone 6B)
// Base Address: 0x4000_4000 (Slave 6 on Wishbone B4 Interconnect)
//
// Supported IEEE-754 Operations (FPU_OPCODE in CTRL[5:2]):
//   4'd0 : FADD.S   -> RESULT = OP_A + OP_B (IEEE-754 binary32)
//   4'd1 : FSUB.S   -> RESULT = OP_A - OP_B (IEEE-754 binary32)
//   4'd2 : FMUL.S   -> RESULT = OP_A * OP_B (IEEE-754 binary32)
//   4'd3 : FMADD.S  -> RESULT = (OP_A * OP_B) + OP_C (also updates OP_C <= RESULT)
//   4'd4 : FMIN.S   -> RESULT = min(OP_A, OP_B)
//   4'd5 : FMAX.S   -> RESULT = max(OP_A, OP_B)
//   4'd6 : FCVT.W.S -> RESULT = signed_int32(OP_A) (truncates toward zero)
//   4'd7 : FCVT.S.W -> RESULT = float32(signed_int32(OP_A))
//   4'd8 : FABS.S   -> RESULT = |OP_A|
//   4'd9 : FNEG.S   -> RESULT = -OP_A
//
// Register Map (Offset from 0x4000_4000):
//   0x0000 [RO] FPU_REG_ID     : 32'h4650_5531 ("FPU1")
//   0x0004 [RO] FPU_REG_STATUS : [0]=Ready (!busy), [1]=Done
//   0x0008 [RW] FPU_REG_OP_A   : 32-bit IEEE-754 Operand A (or INT32 for FCVT.S.W)
//   0x000C [RW] FPU_REG_OP_B   : 32-bit IEEE-754 Operand B
//   0x0010 [RW] FPU_REG_OP_C   : 32-bit IEEE-754 Operand C / FMA Accumulator
//   0x0014 [RW] FPU_REG_CTRL   : [0]=Start, [1]=Clear Status/C, [5:2]=FPU_OPCODE
//   0x0018 [RO] FPU_REG_RESULT : 32-bit IEEE-754 Result (or INT32 for FCVT.W.S)
//   0x001C [RO] FPU_REG_CYCLES : Hardware execution cycles for last operation
//   0x0020 [RO] MTIME_LO       : Lower 32 bits of 64-bit real-time counter
//   0x0024 [RO] MTIME_HI       : Upper 32 bits of 64-bit real-time counter
//   0x0040..0x007C [RW] FPU_RF[0..15] : 16-entry IEEE-754 FP32 Register File (f0..f15)
//=============================================================================

module wb_fp32_unit (
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

    // Activity Status & Interrupts for ASTRA-DAAQ / CPU CSR Unit
    input  wire        npu_irq_i,
    input  wire        dma_irq_i,
    input  wire        uart_rx_irq_i,
    output wire        fpu_busy_o,
    output wire        timer_irq_o,
    output wire        soc_irq_o
);

    wire _unused = &{1'b0, wb_adr_i[31:8], wb_adr_i[1:0], wb_sel_i};

    localparam [31:0] FPU_ID_CONST = 32'h4650_5531; // "FPU1"

    // Operation Codes
    localparam [3:0] FOP_ADD    = 4'd0;
    localparam [3:0] FOP_SUB    = 4'd1;
    localparam [3:0] FOP_MUL    = 4'd2;
    localparam [3:0] FOP_MADD   = 4'd3;
    localparam [3:0] FOP_MIN    = 4'd4;
    localparam [3:0] FOP_MAX    = 4'd5;
    localparam [3:0] FOP_CVT_WS = 4'd6;
    localparam [3:0] FOP_CVT_SW = 4'd7;
    localparam [3:0] FOP_ABS    = 4'd8;
    localparam [3:0] FOP_NEG    = 4'd9;

    // Registers
    reg [31:0] reg_op_a;
    reg [31:0] reg_op_b;
    reg [31:0] reg_op_c;
    reg [31:0] reg_result;
    reg [31:0] reg_cycles;
    reg [3:0]  reg_opcode;
    reg        reg_busy;
    reg        reg_done;
    reg [63:0] mtime_cnt;
    reg [63:0] mtimecmp_reg;

    // SoC Interrupt Controller Registers
    reg        irq_npu_latched;
    reg        irq_dma_latched;
    reg [3:0]  irq_enable_reg;
    reg [31:0] irq_total_cnt;

    wire timer_irq_w = (mtime_cnt >= mtimecmp_reg);
    wire [3:0] irq_pending_vec = {uart_rx_irq_i, timer_irq_w, irq_dma_latched, irq_npu_latched};

    assign fpu_busy_o  = reg_busy;
    assign timer_irq_o = timer_irq_w;
    assign soc_irq_o   = |(irq_pending_vec & irq_enable_reg);

    // Wishbone Request Decoding
    wire is_req     = wb_cyc_i && wb_stb_i && !wb_ack_o;
    wire is_rf_space = wb_adr_i[7:6] == 2'b01; // 0x40..0x7C
    wire [3:0] word_idx = wb_adr_i[5:2];

    // ========================================================================
    // IEEE-754 Single-Precision Helper Functions (Combinational Synthesizable)
    // ========================================================================

    // 1. IEEE-754 Binary32 Multiplier: fp32_mul(a, b)
    function [31:0] fp32_mul;
        input [31:0] a;
        input [31:0] b;
        reg        s_a, s_b, s_res;
        reg [7:0]  e_a, e_b;
        reg [23:0] m_a, m_b;
        reg [47:0] prod;
        reg signed [9:0] e_res;
        reg [22:0] f_res;
        begin
            s_a = a[31]; e_a = a[30:23];
            s_b = b[31]; e_b = b[30:23];
            if ((e_a == 8'd0) || (e_b == 8'd0)) begin
                fp32_mul = 32'h0000_0000;
            end else begin
                s_res = s_a ^ s_b;
                m_a   = {1'b1, a[22:0]};
                m_b   = {1'b1, b[22:0]};
                prod  = m_a * m_b;
                if (prod[47]) begin
                    f_res = prod[46:24];
                    e_res = {2'b00, e_a} + {2'b00, e_b} - 10'sd126;
                end else begin
                    f_res = prod[45:23];
                    e_res = {2'b00, e_a} + {2'b00, e_b} - 10'sd127;
                end
                if (e_res <= 10'sd0)
                    fp32_mul = 32'h0000_0000;
                else if (e_res >= 10'sd255)
                    fp32_mul = {s_res, 8'hFF, 23'd0};
                else
                    fp32_mul = {s_res, e_res[7:0], f_res};
            end
        end
    endfunction

    // 2. IEEE-754 Binary32 Adder/Subtractor: fp32_addsub(a, b, is_sub)
    function [31:0] fp32_addsub;
        input [31:0] a;
        input [31:0] b;
        input        is_sub;
        reg        sa, sb;
        reg [7:0]  ea, eb;
        reg [22:0] fa, fb;
        reg        a_ge_b;
        reg        slg, ssm;
        reg [7:0]  elg, esm, ediff;
        reg [26:0] mlg, msm;
        reg [27:0] msum, mdiff;
        reg [7:0]  eres;
        reg [22:0] fres;
        reg [4:0]  lzc;
        reg [26:0] norm_m;
        integer    k;
        begin
            sa = a[31]; ea = a[30:23]; fa = a[22:0];
            sb = b[31] ^ is_sub; eb = b[30:23]; fb = b[22:0];

            if (ea == 8'd0 && fa == 23'd0) begin
                fp32_addsub = (eb == 8'd0 && fb == 23'd0) ? 32'd0 : {sb, eb, fb};
            end else if (eb == 8'd0 && fb == 23'd0) begin
                fp32_addsub = {sa, ea, fa};
            end else begin
                a_ge_b = ({ea, fa} >= {eb, fb});
                slg    = a_ge_b ? sa : sb;
                ssm    = a_ge_b ? sb : sa;
                elg    = a_ge_b ? ea : eb;
                esm    = a_ge_b ? eb : ea;
                mlg    = {1'b1, (a_ge_b ? fa : fb), 3'b000};
                ediff  = elg - esm;
                msm    = (ediff >= 8'd27) ? 27'd0 : ({1'b1, (a_ge_b ? fb : fa), 3'b000} >> ediff);

                if (slg == ssm) begin
                    msum = {1'b0, mlg} + {1'b0, msm};
                    if (msum[27]) begin
                        eres = elg + 8'd1;
                        fres = msum[26:4];
                    end else begin
                        eres = elg;
                        fres = msum[25:3];
                    end
                    fp32_addsub = {slg, eres, fres};
                end else begin
                    mdiff = {1'b0, mlg} - {1'b0, msm};
                    if (mdiff[26:0] == 27'd0) begin
                        fp32_addsub = 32'h0000_0000;
                    end else begin
                        lzc = 5'd0;
                        for (k = 0; k <= 26; k = k + 1) begin
                            if (mdiff[k]) lzc = 5'd26 - k[4:0];
                        end
                        norm_m = mdiff[26:0] << lzc;
                        if (elg <= {3'b000, lzc}) begin
                            fp32_addsub = 32'h0000_0000;
                        end else begin
                            eres = elg - {3'b000, lzc};
                            fres = norm_m[25:3];
                            fp32_addsub = {slg, eres, fres};
                        end
                    end
                end
            end
        end
    endfunction

    // 3. IEEE-754 Float32 -> Signed INT32 (FCVT.W.S)
    function [31:0] fp32_to_i32;
        input [31:0] a;
        reg        s;
        reg [7:0]  e;
        reg [54:0] m_ext;
        reg [54:0] shifted;
        reg [31:0] mag;
        begin
            s = a[31];
            e = a[30:23];
            if (e < 8'd127) begin
                fp32_to_i32 = 32'd0;
            end else if (e >= 8'd158) begin
                fp32_to_i32 = s ? 32'h8000_0000 : 32'h7FFF_FFFF;
            end else begin
                m_ext = {31'd0, 1'b1, a[22:0]};
                if (e >= 8'd150)
                    shifted = m_ext << (e - 8'd150);
                else
                    shifted = m_ext >> (8'd150 - e);
                mag = shifted[31:0];
                fp32_to_i32 = s ? (~mag + 32'd1) : mag;
            end
        end
    endfunction

    // 4. Signed INT32 -> IEEE-754 Float32 (FCVT.S.W)
    function [31:0] i32_to_fp32;
        input [31:0] val;
        reg        s;
        reg [31:0] mag;
        reg [4:0]  msb_pos;
        reg [31:0] norm;
        reg [7:0]  e;
        integer    k;
        begin
            if (val == 32'd0) begin
                i32_to_fp32 = 32'h0000_0000;
            end else begin
                s   = val[31];
                mag = s ? (~val + 32'd1) : val;
                msb_pos = 5'd0;
                for (k = 0; k < 32; k = k + 1) begin
                    if (mag[k]) msb_pos = k[4:0];
                end
                norm = mag << (5'd31 - msb_pos);
                e    = 8'd127 + {3'b000, msb_pos};
                i32_to_fp32 = {s, e, norm[30:8]};
            end
        end
    endfunction

    // 5. IEEE-754 Float32 Comparison: returns 1 if a < b
    function fp32_lt;
        input [31:0] a;
        input [31:0] b;
        reg sa, sb;
        begin
            sa = a[31];
            sb = b[31];
            if (a[30:0] == 31'd0 && b[30:0] == 31'd0)
                fp32_lt = 1'b0;
            else if (sa != sb)
                fp32_lt = sa; // negative < positive
            else if (sa == 1'b0)
                fp32_lt = (a[30:0] < b[30:0]);
            else
                fp32_lt = (a[30:0] > b[30:0]);
        end
    endfunction

    // ========================================================================
    // Multi-Cycle Execution FSM (2 cycles for standard ops, 3 cycles for FMADD)
    // ========================================================================
    localparam [1:0] FSM_IDLE   = 2'd0;
    localparam [1:0] FSM_STAGE1 = 2'd1;
    localparam [1:0] FSM_STAGE2 = 2'd2;

    // 16-entry IEEE-754 Register File (f0..f15)
    reg [31:0] fpu_rf [0:15];

    reg [1:0]  fsm_state;
    reg [31:0] stage1_val;
    reg        start_pulse;
    reg        clr_pulse;

    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            wb_ack_o        <= 1'b0;
            wb_dat_o        <= 32'd0;
            reg_op_a        <= 32'd0;
            reg_op_b        <= 32'd0;
            reg_op_c        <= 32'd0;
            reg_result      <= 32'd0;
            reg_cycles      <= 32'd0;
            reg_opcode      <= 4'd0;
            reg_busy        <= 1'b0;
            reg_done        <= 1'b0;
            mtime_cnt       <= 64'd0;
            mtimecmp_reg    <= 64'hFFFF_FFFF_FFFF_FFFF;
            irq_npu_latched <= 1'b0;
            irq_dma_latched <= 1'b0;
            irq_enable_reg  <= 4'hF;
            irq_total_cnt   <= 32'd0;
            fsm_state       <= FSM_IDLE;
            stage1_val      <= 32'd0;
            start_pulse     <= 1'b0;
            clr_pulse       <= 1'b0;
            for (i = 0; i < 16; i = i + 1) begin
                fpu_rf[i] <= 32'd0;
            end
        end else begin
            mtime_cnt   <= mtime_cnt + 64'd1;
            start_pulse <= 1'b0;
            clr_pulse   <= 1'b0;

            if (npu_irq_i) begin
                irq_npu_latched <= 1'b1;
                irq_total_cnt   <= irq_total_cnt + 32'd1;
            end
            if (dma_irq_i) begin
                irq_dma_latched <= 1'b1;
                irq_total_cnt   <= irq_total_cnt + 32'd1;
            end

            if (clr_pulse && !reg_busy) begin
                reg_op_c   <= 32'd0;
                reg_done   <= 1'b0;
                reg_cycles <= 32'd0;
            end

            case (fsm_state)
                FSM_IDLE: begin
                    if (start_pulse) begin
                        reg_busy   <= 1'b1;
                        reg_done   <= 1'b0;
                        reg_cycles <= 32'd1;
                        fsm_state  <= FSM_STAGE1;
                    end
                end

                FSM_STAGE1: begin
                    reg_cycles <= reg_cycles + 32'd1;
                    case (reg_opcode)
                        FOP_ADD: begin
                            reg_result <= fp32_addsub(reg_op_a, reg_op_b, 1'b0);
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_SUB: begin
                            reg_result <= fp32_addsub(reg_op_a, reg_op_b, 1'b1);
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_MUL: begin
                            reg_result <= fp32_mul(reg_op_a, reg_op_b);
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_MADD: begin
                            // Stage 1: Compute A * B, then proceed to Stage 2 (+ C)
                            stage1_val <= fp32_mul(reg_op_a, reg_op_b);
                            fsm_state  <= FSM_STAGE2;
                        end
                        FOP_MIN: begin
                            reg_result <= fp32_lt(reg_op_a, reg_op_b) ? reg_op_a : reg_op_b;
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_MAX: begin
                            reg_result <= fp32_lt(reg_op_a, reg_op_b) ? reg_op_b : reg_op_a;
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_CVT_WS: begin
                            reg_result <= fp32_to_i32(reg_op_a);
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_CVT_SW: begin
                            reg_result <= i32_to_fp32(reg_op_a);
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_ABS: begin
                            reg_result <= {1'b0, reg_op_a[30:0]};
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        FOP_NEG: begin
                            reg_result <= {~reg_op_a[31], reg_op_a[30:0]};
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                        default: begin
                            reg_result <= 32'd0;
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end
                    endcase
                end

                FSM_STAGE2: begin
                    // Complete FMADD: (A * B) + C
                    reg_cycles <= reg_cycles + 32'd1;
                    reg_result <= fp32_addsub(stage1_val, reg_op_c, 1'b0);
                    reg_op_c   <= fp32_addsub(stage1_val, reg_op_c, 1'b0);
                    reg_busy   <= 1'b0;
                    reg_done   <= 1'b1;
                    fsm_state  <= FSM_IDLE;
                end

                default: fsm_state <= FSM_IDLE;
            endcase

            // Wishbone Slave Read / Write Interface
            if (is_req) begin
                wb_ack_o <= 1'b1;
                if (wb_we_i) begin
                    if (is_rf_space) begin
                        fpu_rf[word_idx] <= wb_dat_i;
                    end else begin
                        case (word_idx)
                            4'h2: reg_op_a <= wb_dat_i; // 0x08: OP_A
                            4'h3: reg_op_b <= wb_dat_i; // 0x0C: OP_B
                            4'h4: reg_op_c <= wb_dat_i; // 0x10: OP_C
                            4'h5: begin                 // 0x14: CTRL
                                reg_opcode <= wb_dat_i[5:2];
                                if (wb_dat_i[0] && !reg_busy) start_pulse <= 1'b1;
                                if (wb_dat_i[1])              clr_pulse   <= 1'b1;
                            end
                            4'h8: mtime_cnt[31:0]     <= wb_dat_i; // 0x20: MTIME_LO
                            4'h9: mtime_cnt[63:32]    <= wb_dat_i; // 0x24: MTIME_HI
                            4'hA: mtimecmp_reg[31:0]  <= wb_dat_i; // 0x28: MTIMECMP_LO
                            4'hB: mtimecmp_reg[63:32] <= wb_dat_i; // 0x2C: MTIMECMP_HI
                            4'hD: irq_enable_reg      <= wb_dat_i[3:0]; // 0x34: SOC_IRQ_ENABLE
                            4'hE: begin                                 // 0x38: SOC_IRQ_CLEAR
                                if (wb_dat_i[0]) irq_npu_latched <= 1'b0;
                                if (wb_dat_i[1]) irq_dma_latched <= 1'b0;
                            end
                            default: ;
                        endcase
                    end
                end else begin
                    if (is_rf_space) begin
                        wb_dat_o <= fpu_rf[word_idx];
                    end else begin
                        case (word_idx)
                            4'h0: wb_dat_o <= FPU_ID_CONST;                  // 0x00: "FPU1"
                            4'h1: wb_dat_o <= {30'd0, reg_done, !reg_busy};   // 0x04: STATUS
                            4'h2: wb_dat_o <= reg_op_a;                      // 0x08: OP_A
                            4'h3: wb_dat_o <= reg_op_b;                      // 0x0C: OP_B
                            4'h4: wb_dat_o <= reg_op_c;                      // 0x10: OP_C
                            4'h5: wb_dat_o <= {26'd0, reg_opcode, 1'b0, reg_busy};
                            4'h6: wb_dat_o <= reg_result;                    // 0x18: RESULT
                            4'h7: wb_dat_o <= reg_cycles;                    // 0x1C: CYCLES
                            4'h8: wb_dat_o <= mtime_cnt[31:0];               // 0x20: MTIME_LO
                            4'h9: wb_dat_o <= mtime_cnt[63:32];              // 0x24: MTIME_HI
                            4'hA: wb_dat_o <= mtimecmp_reg[31:0];            // 0x28: MTIMECMP_LO
                            4'hB: wb_dat_o <= mtimecmp_reg[63:32];           // 0x2C: MTIMECMP_HI
                            4'hC: wb_dat_o <= {28'd0, irq_pending_vec};      // 0x30: SOC_IRQ_PENDING
                            4'hD: wb_dat_o <= {28'd0, irq_enable_reg};       // 0x34: SOC_IRQ_ENABLE
                            4'hF: wb_dat_o <= irq_total_cnt;                 // 0x3C: SOC_IRQ_COUNT
                            default: wb_dat_o <= 32'd0;
                        endcase
                    end
                end
            end else begin
                wb_ack_o <= 1'b0;
            end
        end
    end

endmodule
