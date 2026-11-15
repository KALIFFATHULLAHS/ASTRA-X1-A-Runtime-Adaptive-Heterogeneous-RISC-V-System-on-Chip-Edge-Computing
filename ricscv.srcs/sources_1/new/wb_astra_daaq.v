`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: ASTRA-DAAQ Runtime Dependency-Aware Power & Activity Controller
// (Milestone 7 Research Contribution)
// Base Address: 0x4000_2000 (Slave 4 on Wishbone B4 Interconnect)
//
// Evaluates Cross-Resource Dependencies and Break-Even Quiescence Policy:
//   E_benefit = (Delta_P * T_idle) - E_transition
//
// Supports 4 Controlled Ablation Modes (DAAQ_REG_MODE[1:0]):
//   2'd0 : Mode A (Baseline Always-On)       -> Zero gating, all engines active
//   2'd1 : Mode B (Naive Local Gating)       -> Immediate local idle gating (ignores dependencies)
//   2'd2 : Mode C (Fixed-Threshold DAAQ)     -> Dependency-safe gating after T_thresh idle cycles
//   2'd3 : Mode D (Adaptive Phase-Hint DAAQ) -> Dependency-safe + software phase-guided gating
//
// Software Workload Phase Hints (DAAQ_REG_PHASE[2:0]):
//   3'd0 : PHASE_GENERAL      (Default balanced thresholds)
//   3'd1 : PHASE_DMA_XFER     (Pre-warm DMA + NPU/SIMD targets; quiesce FPU)
//   3'd2 : PHASE_SIMD_PREPROC (Pre-warm SIMD; quiesce NPU, DMA, FPU immediately when idle)
//   3'd3 : PHASE_NPU_GEMM     (Pre-warm NPU; quiesce SIMD, DMA, FPU immediately when idle)
//   3'd4 : PHASE_FPU_POST     (Pre-warm FPU; quiesce NPU, SIMD, DMA immediately when idle)
//   3'd5 : PHASE_IDLE         (Aggressively quiesce all idle accelerators)
//
// Register Map (Offset from 0x4000_2000):
//   0x0000 [RO] DAAQ_REG_ID           : 32'h4441_4151 ("DAAQ")
//   0x0004 [RW] DAAQ_REG_MODE         : [1:0]=Ablation Mode (0..3), [4]=Reset telemetry counters
//   0x0008 [RW] DAAQ_REG_PHASE        : [2:0]=Software Phase Hint (0..5)
//   0x000C [RW] DAAQ_REG_THRESH       : [15:0]=Break-even idle cycle threshold (default 8)
//   0x0010 [RO] DAAQ_REG_STATUS       : [11:8]=dep_{fpu,simd,dma,npu}, [3:0]=ce_{fpu,simd,dma,npu}
//   0x0014 [RO] DAAQ_REG_TOTAL_CYC    : Total elapsed clock cycles in current measurement window
//   0x0018 [RO] DAAQ_REG_GATED_NPU    : Cycles NPU was quiesced
//   0x001C [RO] DAAQ_REG_GATED_DMA    : Cycles DMA was quiesced
//   0x0020 [RO] DAAQ_REG_GATED_SIMD   : Cycles SIMD was quiesced
//   0x0024 [RO] DAAQ_REG_GATED_FPU    : Cycles FPU was quiesced
//   0x0028 [RO] DAAQ_REG_ENERGY_SAVED : Weighted dynamic switching energy units saved
//   0x002C [RO] DAAQ_REG_WAKE_STALLS  : Cumulative wake-up/transition penalty events
//=============================================================================

module wb_astra_daaq (
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

    // Live Hardware Activity & Bus Dependency Inputs
    input  wire        npu_busy_i,
    input  wire        dma_busy_i,
    input  wire        simd_busy_i,
    input  wire        fpu_busy_i,
    input  wire        npu_bus_req_i,
    input  wire        dma_bus_req_i,
    input  wire        simd_bus_req_i,
    input  wire        fpu_bus_req_i,

    // Safe Quiescence / Clock-Enable Outputs
    output reg         ce_npu_o,
    output reg         ce_dma_o,
    output reg         ce_simd_o,
    output reg         ce_fpu_o
);

    wire _unused = &{1'b0, wb_adr_i[31:6], wb_adr_i[1:0], wb_sel_i};

    localparam [31:0] DAAQ_ID_CONST = 32'h4441_4151; // "DAAQ"

    // Ablation Modes
    localparam [1:0] MODE_A_BASELINE = 2'd0;
    localparam [1:0] MODE_B_LOCAL    = 2'd1;
    localparam [1:0] MODE_C_FIXED    = 2'd2;
    localparam [1:0] MODE_D_ADAPTIVE = 2'd3;

    // Software Phase Hints
    localparam [2:0] PHASE_GENERAL      = 3'd0;
    localparam [2:0] PHASE_DMA_XFER     = 3'd1;
    localparam [2:0] PHASE_SIMD_PREPROC = 3'd2;
    localparam [2:0] PHASE_NPU_GEMM     = 3'd3;
    localparam [2:0] PHASE_FPU_POST     = 3'd4;
    localparam [2:0] PHASE_IDLE         = 3'd5;

    // Control Registers
    reg [1:0]  reg_mode;
    reg [2:0]  reg_phase;
    reg [15:0] reg_thresh;
    reg        clr_stats;

    // Telemetry Counters
    reg [31:0] cnt_total_cyc;
    reg [31:0] cnt_gated_npu;
    reg [31:0] cnt_gated_dma;
    reg [31:0] cnt_gated_simd;
    reg [31:0] cnt_gated_fpu;
    reg [31:0] cnt_energy_saved;
    reg [31:0] cnt_wake_stalls;

    // Per-Unit Idle Duration Timers
    reg [15:0] idle_npu;
    reg [15:0] idle_dma;
    reg [15:0] idle_simd;
    reg [15:0] idle_fpu;

    // ========================================================================
    // 1. Cross-Resource Dependency Tracker (daaq_tracker)
    // An accelerator has an active dependency if:
    //   - It is currently executing (busy_i), OR
    //   - A Wishbone bus request is targeting its address range (bus_req_i), OR
    //   - The autonomous DMA engine is active and transferring to/from shared slaves
    // ========================================================================
    wire dep_npu  = npu_busy_i  || npu_bus_req_i  || dma_busy_i;
    wire dep_dma  = dma_busy_i  || dma_bus_req_i;
    wire dep_simd = simd_busy_i || simd_bus_req_i || dma_busy_i;
    wire dep_fpu  = fpu_busy_i  || fpu_bus_req_i;

    // ========================================================================
    // 2. Adaptive Break-Even Threshold Calculation (daaq_policy)
    // ========================================================================
    reg [15:0] eff_thresh_npu;
    reg [15:0] eff_thresh_dma;
    reg [15:0] eff_thresh_simd;
    reg [15:0] eff_thresh_fpu;

    always @(*) begin
        if (reg_mode == MODE_C_FIXED) begin
            eff_thresh_npu  = reg_thresh;
            eff_thresh_dma  = reg_thresh;
            eff_thresh_simd = reg_thresh;
            eff_thresh_fpu  = reg_thresh;
        end else begin
            // MODE_D_ADAPTIVE: Adjust break-even threshold based on software phase hint
            case (reg_phase)
                PHASE_DMA_XFER: begin
                    eff_thresh_dma  = 16'd4; // Pre-warm DMA & NPU with short window
                    eff_thresh_npu  = 16'd4;
                    eff_thresh_simd = 16'd1; // Quiesce SIMD immediately
                    eff_thresh_fpu  = 16'd1; // Quiesce FPU immediately
                end
                PHASE_SIMD_PREPROC: begin
                    eff_thresh_simd = 16'd4; // Keep SIMD pre-warmed
                    eff_thresh_npu  = 16'd1; // Quiesce NPU immediately
                    eff_thresh_dma  = 16'd1; // Quiesce DMA immediately
                    eff_thresh_fpu  = 16'd1; // Quiesce FPU immediately
                end
                PHASE_NPU_GEMM: begin
                    eff_thresh_npu  = 16'd4; // Keep NPU pre-warmed
                    eff_thresh_simd = 16'd1; // Quiesce SIMD immediately
                    eff_thresh_dma  = 16'd1; // Quiesce DMA immediately
                    eff_thresh_fpu  = 16'd1; // Quiesce FPU immediately
                end
                PHASE_FPU_POST: begin
                    eff_thresh_fpu  = 16'd4; // Keep FPU pre-warmed
                    eff_thresh_npu  = 16'd1; // Quiesce NPU immediately
                    eff_thresh_simd = 16'd1; // Quiesce SIMD immediately
                    eff_thresh_dma  = 16'd1; // Quiesce DMA immediately
                end
                PHASE_IDLE: begin
                    eff_thresh_npu  = 16'd1;
                    eff_thresh_dma  = 16'd1;
                    eff_thresh_simd = 16'd1;
                    eff_thresh_fpu  = 16'd1;
                end
                default: begin
                    eff_thresh_npu  = 16'd2;
                    eff_thresh_dma  = 16'd2;
                    eff_thresh_simd = 16'd2;
                    eff_thresh_fpu  = 16'd2;
                end
            endcase
        end
    end

    // ========================================================================
    // 3. Quiescence Decision Logic (daaq_quiescence)
    // ========================================================================
    always @(*) begin
        case (reg_mode)
            MODE_A_BASELINE: begin
                ce_npu_o  = 1'b1;
                ce_dma_o  = 1'b1;
                ce_simd_o = 1'b1;
                ce_fpu_o  = 1'b1;
            end
            MODE_B_LOCAL: begin
                // Naive local gating: gates whenever local busy==0 (ignores bus/DMA dependencies)
                ce_npu_o  = npu_busy_i;
                ce_dma_o  = dma_busy_i;
                ce_simd_o = simd_busy_i;
                ce_fpu_o  = fpu_busy_i;
            end
            MODE_C_FIXED, MODE_D_ADAPTIVE: begin
                // Dependency-aware adaptive quiescence
                ce_npu_o  = dep_npu  || (idle_npu  < eff_thresh_npu);
                ce_dma_o  = dep_dma  || (idle_dma  < eff_thresh_dma);
                ce_simd_o = dep_simd || (idle_simd < eff_thresh_simd);
                ce_fpu_o  = dep_fpu  || (idle_fpu  < eff_thresh_fpu);
            end
            default: begin
                ce_npu_o  = 1'b1;
                ce_dma_o  = 1'b1;
                ce_simd_o = 1'b1;
                ce_fpu_o  = 1'b1;
            end
        endcase
    end

    // Detect cold-wake hazard events (bus/DMA hits an engine while its clock enable was gated)
    wire cold_hit_npu  = !ce_npu_o  && (npu_bus_req_i  || dma_busy_i);
    wire cold_hit_dma  = !ce_dma_o  && dma_bus_req_i;
    wire cold_hit_simd = !ce_simd_o && (simd_bus_req_i || dma_busy_i);
    wire cold_hit_fpu  = !ce_fpu_o  && fpu_bus_req_i;
    wire [2:0] cold_hits_this_cyc = {2'b00, cold_hit_npu} + {2'b00, cold_hit_dma} +
                                    {2'b00, cold_hit_simd} + {2'b00, cold_hit_fpu};

    // Weighted dynamic energy savings per clock cycle:
    // NPU (16 PEs) = 4 units/cyc, SIMD (4 lanes) = 2 units/cyc, FPU = 2 units/cyc, DMA = 1 unit/cyc
    wire [3:0] energy_saved_this_cyc = (!ce_npu_o  ? 4'd4 : 4'd0) +
                                       (!ce_simd_o ? 4'd2 : 4'd0) +
                                       (!ce_fpu_o  ? 4'd2 : 4'd0) +
                                       (!ce_dma_o  ? 4'd1 : 4'd0);

    // Wishbone Slave Request Decoding
    wire is_req = wb_cyc_i && wb_stb_i && !wb_ack_o;
    wire [3:0] reg_idx = wb_adr_i[5:2];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            wb_ack_o         <= 1'b0;
            wb_dat_o         <= 32'd0;
            reg_mode         <= MODE_A_BASELINE;
            reg_phase        <= PHASE_GENERAL;
            reg_thresh       <= 16'd8;
            clr_stats        <= 1'b0;
            cnt_total_cyc    <= 32'd0;
            cnt_gated_npu    <= 32'd0;
            cnt_gated_dma    <= 32'd0;
            cnt_gated_simd   <= 32'd0;
            cnt_gated_fpu    <= 32'd0;
            cnt_energy_saved <= 32'd0;
            cnt_wake_stalls  <= 32'd0;
            idle_npu         <= 16'd0;
            idle_dma         <= 16'd0;
            idle_simd        <= 16'd0;
            idle_fpu         <= 16'd0;
        end else begin
            clr_stats <= 1'b0;

            // Update per-engine idle timers
            if (dep_npu)  idle_npu  <= 16'd0;
            else if (idle_npu != 16'hFFFF)  idle_npu  <= idle_npu  + 16'd1;

            if (dep_dma)  idle_dma  <= 16'd0;
            else if (idle_dma != 16'hFFFF)  idle_dma  <= idle_dma  + 16'd1;

            if (dep_simd) idle_simd <= 16'd0;
            else if (idle_simd != 16'hFFFF) idle_simd <= idle_simd + 16'd1;

            if (dep_fpu)  idle_fpu  <= 16'd0;
            else if (idle_fpu != 16'hFFFF)  idle_fpu  <= idle_fpu  + 16'd1;

            // Update Telemetry Counters
            if (clr_stats) begin
                cnt_total_cyc    <= 32'd0;
                cnt_gated_npu    <= 32'd0;
                cnt_gated_dma    <= 32'd0;
                cnt_gated_simd   <= 32'd0;
                cnt_gated_fpu    <= 32'd0;
                cnt_energy_saved <= 32'd0;
                cnt_wake_stalls  <= 32'd0;
            end else begin
                cnt_total_cyc    <= cnt_total_cyc + 32'd1;
                if (!ce_npu_o)  cnt_gated_npu  <= cnt_gated_npu  + 32'd1;
                if (!ce_dma_o)  cnt_gated_dma  <= cnt_gated_dma  + 32'd1;
                if (!ce_simd_o) cnt_gated_simd <= cnt_gated_simd + 32'd1;
                if (!ce_fpu_o)  cnt_gated_fpu  <= cnt_gated_fpu  + 32'd1;
                cnt_energy_saved <= cnt_energy_saved + {28'd0, energy_saved_this_cyc};
                cnt_wake_stalls  <= cnt_wake_stalls  + {29'd0, cold_hits_this_cyc};
            end

            // Wishbone Slave Read / Write Interface
            if (is_req) begin
                wb_ack_o <= 1'b1;
                if (wb_we_i) begin
                    case (reg_idx)
                        4'h1: begin // 0x04: DAAQ_REG_MODE
                            reg_mode <= wb_dat_i[1:0];
                            if (wb_dat_i[4]) clr_stats <= 1'b1;
                        end
                        4'h2: reg_phase  <= wb_dat_i[2:0];  // 0x08: DAAQ_REG_PHASE
                        4'h3: reg_thresh <= wb_dat_i[15:0]; // 0x0C: DAAQ_REG_THRESH
                        default: ;
                    endcase
                end else begin
                    case (reg_idx)
                        4'h0: wb_dat_o <= DAAQ_ID_CONST;                             // 0x00: "DAAQ"
                        4'h1: wb_dat_o <= {30'd0, reg_mode};                         // 0x04: MODE
                        4'h2: wb_dat_o <= {29'd0, reg_phase};                        // 0x08: PHASE
                        4'h3: wb_dat_o <= {16'd0, reg_thresh};                       // 0x0C: THRESH
                        4'h4: wb_dat_o <= {20'd0, dep_fpu, dep_simd, dep_dma, dep_npu,
                                           4'd0,  ce_fpu_o, ce_simd_o, ce_dma_o, ce_npu_o}; // 0x10: STATUS
                        4'h5: wb_dat_o <= cnt_total_cyc;                             // 0x14: TOTAL_CYC
                        4'h6: wb_dat_o <= cnt_gated_npu;                             // 0x18: GATED_NPU
                        4'h7: wb_dat_o <= cnt_gated_dma;                             // 0x1C: GATED_DMA
                        4'h8: wb_dat_o <= cnt_gated_simd;                            // 0x20: GATED_SIMD
                        4'h9: wb_dat_o <= cnt_gated_fpu;                             // 0x24: GATED_FPU
                        4'hA: wb_dat_o <= cnt_energy_saved;                          // 0x28: ENERGY_SAVED
                        4'hB: wb_dat_o <= cnt_wake_stalls;                           // 0x2C: WAKE_STALLS
                        default: wb_dat_o <= 32'd0;
                    endcase
                end
            end else begin
                wb_ack_o <= 1'b0;
            end
        end
    end

endmodule
