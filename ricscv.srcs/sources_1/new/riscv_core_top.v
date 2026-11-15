`timescale 1ns/1ps

//=============================================================
//  RISC-V 7-Stage RV32IM Processor Top-Level
//  Power-Gated + Approximate Arithmetic + WFI Idle + UART Loader
//  PART 1 / 4
//=============================================================

module riscv_core_top (
    input  wire        clk_in,
    input  wire        reset,

    input  wire        uart_rx,
    output wire        uart_tx,

    output wire       dbg_wfi_active,
    output wire [7:0] dbg_led,        // connect to board LEDs [7:0]
    output wire [7:0] dbg_led_hi,     // connect to board LEDs [15:8]
    input  wire [15:0] sw_i,
    input  wire [3:0]  pb_i,
    output wire [3:0]  seven_seg_digit_o,
    output wire [7:0]  seven_seg_seg_o,
    output wire        spi_sck_o,
    output wire        spi_cs_n_o,
    output wire        spi_mosi_o,
    output wire        i2c_scl_o,
    output wire        i2c_sda_o
);

//=============================================================
// CLOCK MANAGER + GATED CLOCKS
//=============================================================

wire ce_if1, ce_if2, ce_id, ce_ex1, ce_ex2, ce_mem, ce_wb;
wire ce_mul, ce_div, ce_approx, ce_uart, ce_csr;

wire clk_sys;
wire clk_if1, clk_if2, clk_id, clk_ex1, clk_ex2, clk_mem, clk_wb;
wire clk_mul, clk_div, clk_approx, clk_uart, clk_csr;

clock_manager clk_mgr (
    .clk_in(clk_in),
    .reset(reset),

    .ce_if1(ce_if1), .ce_if2(ce_if2), .ce_id(ce_id),
    .ce_ex1(ce_ex1), .ce_ex2(ce_ex2),
    .ce_mem(ce_mem), .ce_wb(ce_wb),

    .ce_mul(ce_mul), .ce_div(ce_div), .ce_approx(ce_approx),
    .ce_uart(ce_uart), .ce_csr(ce_csr),

    .clk_sys(clk_sys),
    .clk_if1(clk_if1), .clk_if2(clk_if2), .clk_id(clk_id),
    .clk_ex1(clk_ex1), .clk_ex2(clk_ex2),
    .clk_mem(clk_mem), .clk_wb(clk_wb),
    .clk_mul(clk_mul), .clk_div(clk_div), .clk_approx(clk_approx),
    .clk_uart(clk_uart), .clk_csr(clk_csr)
);

//=============================================================
// PIPELINE WIRES DECLARATION
//=============================================================

//---------- PREDICTED PC & BRANCH TARGET WIRES ----------
wire [31:0] id_pred_pc;
wire [31:0] ex1_pred_pc;
wire [31:0] ex2_pred_pc;
wire        ex2_branch_taken;
wire [31:0] ex2_branch_target;

//---------- HAZARD / STALL / FLUSH ----------
wire stall_if, stall_id, stall_ex1, stall_ex2;
wire flush_id, flush_ex1;
wire mul_busy, div_busy;

//---------- IF1 → IF2 ----------
wire [31:0] if1_pc;
wire        if1_valid;
wire [31:0] imem_rdata;
reg  [31:0] if2_pc;
wire        if2_valid;
wire [31:0] if1_pred_pc;
reg  [31:0] if2_pred_pc;

always @(posedge clk_if1 or posedge reset) begin
    if (reset) begin
        if2_pc      <= 32'h0;
        if2_pred_pc <= 32'h0;
    end else if (!stall_if) begin
        if2_pc      <= if1_pc;
        if2_pred_pc <= if1_pred_pc;
    end
end

//---------- IF2 → ID ----------
wire [31:0] id_pc;
wire [31:0] id_instr;
wire        id_valid;

//---------- ID → EX1 ----------
wire [31:0] ex1_pc;
wire [31:0] ex1_imm;
wire [31:0] ex1_instr;
wire [4:0]  ex1_rd;
wire [23:0] ex1_ctrl_bus;
wire        ex1_valid;

//---------- EX1 → EX2 ----------
wire [31:0] ex2_pc;
wire [31:0] ex2_op_a;
wire [31:0] ex2_op_b;
wire [31:0] ex2_imm;
wire [31:0] ex2_instr;
wire [4:0]  ex2_rd;
wire [23:0] ex2_ctrl_bus;
wire        ex2_valid;
wire [31:0] ex2_comb_result;

//---------- EX2 → MEM ----------
wire [31:0] mem_result;

wire [31:0] mem_pc;
wire [4:0]  mem_rd;
wire [23:0] mem_ctrl_bus;
wire        mem_valid;
wire [31:0] ex2_store_data;
wire [31:0] mem_store_data;
//---------- MEM → WB ----------
wire [31:0] wb_data;
wire [31:0] wb_pc;
wire [23:0] wb_ctrl_bus;
wire [4:0]  wb_rd;
wire        wb_valid;
wire [31:0] mem_instr;
wire [31:0] wb_instr;
//=============================================================
// INSTRUCTION MEMORY (IMEM) + UART LOADER
//=============================================================
// You may use Xilinx BRAM IP or handwritten BRAM module.

wire [31:0] imem_addr;

bram_imem imem (
    .clk(clk_if1),
    .addr(imem_addr[15:2]),
    .rdata(imem_rdata),
    .reset(reset),
    .uart_clk(clk_uart),
    .uart_rx(uart_rx),
    .uart_tx() // Disconnected to prevent multi-driver contention
);


//=============================================================
// DATA MEMORY (DMEM)
//=============================================================
//=============================================================
// DATA MEMORY (DMEM)
//=============================================================

//=============================================================
// 32-BIT WISHBONE B4 SoC INTERCONNECT & SLAVES
//=============================================================

// Master Interface (CPU Memory Stage)
wire [31:0] wb_m_adr;
wire [31:0] wb_m_dat_w;
wire [3:0]  wb_m_sel;
wire        wb_m_we;
wire        wb_m_cyc;
wire        wb_m_stb;
wire [31:0] wb_m_dat_r;
wire        wb_m_ack;
wire        wb_m_err;
wire        stall_mem;

// Slave 0: Data SRAM (BRAM)
wire [31:0] wb_s0_adr;
wire [31:0] wb_s0_dat_w;
wire [3:0]  wb_s0_sel;
wire        wb_s0_we;
wire        wb_s0_cyc;
wire        wb_s0_stb;
wire [31:0] wb_s0_dat_r;
wire        wb_s0_ack;

// Slave 1: Mock NPU
wire [31:0] wb_s1_adr;
wire [31:0] wb_s1_dat_w;
wire [3:0]  wb_s1_sel;
wire        wb_s1_we;
wire        wb_s1_cyc;
wire        wb_s1_stb;
wire [31:0] wb_s1_dat_r;
wire        wb_s1_ack;

// Slave 2: Peripherals (UART TX + GPIO LEDs)
wire [31:0] wb_s2_adr;
wire [31:0] wb_s2_dat_w;
wire [3:0]  wb_s2_sel;
wire        wb_s2_we;
wire        wb_s2_cyc;
wire        wb_s2_stb;
wire [31:0] wb_s2_dat_r;
wire        wb_s2_ack;

// Slave 3: DMA Controller MMIO (0x4000_1000)
wire [31:0] wb_s3_adr;
wire [31:0] wb_s3_dat_w;
wire [3:0]  wb_s3_sel;
wire        wb_s3_we;
wire        wb_s3_cyc;
wire        wb_s3_stb;
wire [31:0] wb_s3_dat_r;
wire        wb_s3_ack;

// Slave 4: ASTRA-DAAQ Power & Activity Controller MMIO (0x4000_2000)
wire [31:0] wb_s4_adr;
wire [31:0] wb_s4_dat_w;
wire [3:0]  wb_s4_sel;
wire        wb_s4_we;
wire        wb_s4_cyc;
wire        wb_s4_stb;
wire [31:0] wb_s4_dat_r;
wire        wb_s4_ack;

// Slave 5: 4-Lane 16-bit Packed SIMD Accelerator (0x4000_3000)
wire [31:0] wb_s5_adr;
wire [31:0] wb_s5_dat_w;
wire [3:0]  wb_s5_sel;
wire        wb_s5_we;
wire        wb_s5_cyc;
wire        wb_s5_stb;
wire [31:0] wb_s5_dat_r;
wire        wb_s5_ack;

// Slave 6: IEEE-754 Single-Precision FP32 Engine & Timer (0x4000_4000)
wire [31:0] wb_s6_adr;
wire [31:0] wb_s6_dat_w;
wire [3:0]  wb_s6_sel;
wire        wb_s6_we;
wire        wb_s6_cyc;
wire        wb_s6_stb;
wire [31:0] wb_s6_dat_r;
wire        wb_s6_ack;

// Master 1: Autonomous DMA Controller Master Interface
wire [31:0] wb_m1_adr;
wire [31:0] wb_m1_dat_w;
wire [3:0]  wb_m1_sel;
wire        wb_m1_we;
wire        wb_m1_cyc;
wire        wb_m1_stb;
wire [31:0] wb_m1_dat_r;
wire        wb_m1_ack;
wire        wb_m1_err;

// Live Accelerator Busy, Interrupt & Clock Enable Wires
wire npu_busy_w;
wire dma_busy_w;
wire simd_busy_w;
wire fpu_busy_w;
wire npu_irq_w;
wire dma_irq_w;
wire uart_rx_irq_w;
wire timer_irq_w;
wire soc_irq_w;
wire daaq_ce_npu;
wire daaq_ce_dma;
wire daaq_ce_simd;
wire daaq_ce_fpu;

// Wishbone Crossbar / Address Decoder
wb_interconnect bus_intercon (
    .clk      (clk_sys),
    .reset    (reset),

    // Master 0 port (CPU)
    .m_adr_i  (wb_m_adr),
    .m_dat_i  (wb_m_dat_w),
    .m_sel_i  (wb_m_sel),
    .m_we_i   (wb_m_we),
    .m_cyc_i  (wb_m_cyc),
    .m_stb_i  (wb_m_stb),
    .m_dat_o  (wb_m_dat_r),
    .m_ack_o  (wb_m_ack),
    .m_err_o  (wb_m_err),

    // Master 1 port (DMA)
    .m1_adr_i (wb_m1_adr),
    .m1_dat_i (wb_m1_dat_w),
    .m1_sel_i (wb_m1_sel),
    .m1_we_i  (wb_m1_we),
    .m1_cyc_i (wb_m1_cyc),
    .m1_stb_i (wb_m1_stb),
    .m1_dat_o (wb_m1_dat_r),
    .m1_ack_o (wb_m1_ack),
    .m1_err_o (wb_m1_err),

    // Slave 0: Data SRAM
    .s0_adr_o (wb_s0_adr),
    .s0_dat_o (wb_s0_dat_w),
    .s0_sel_o (wb_s0_sel),
    .s0_we_o  (wb_s0_we),
    .s0_cyc_o (wb_s0_cyc),
    .s0_stb_o (wb_s0_stb),
    .s0_dat_i (wb_s0_dat_r),
    .s0_ack_i (wb_s0_ack),

    // Slave 1: ASTRA-X1 NPU
    .s1_adr_o (wb_s1_adr),
    .s1_dat_o (wb_s1_dat_w),
    .s1_sel_o (wb_s1_sel),
    .s1_we_o  (wb_s1_we),
    .s1_cyc_o (wb_s1_cyc),
    .s1_stb_o (wb_s1_stb),
    .s1_dat_i (wb_s1_dat_r),
    .s1_ack_i (wb_s1_ack),

    // Slave 2: Peripherals
    .s2_adr_o (wb_s2_adr),
    .s2_dat_o (wb_s2_dat_w),
    .s2_sel_o (wb_s2_sel),
    .s2_we_o  (wb_s2_we),
    .s2_cyc_o (wb_s2_cyc),
    .s2_stb_o (wb_s2_stb),
    .s2_dat_i (wb_s2_dat_r),
    .s2_ack_i (wb_s2_ack),

    // Slave 3: DMA Controller MMIO
    .s3_adr_o (wb_s3_adr),
    .s3_dat_o (wb_s3_dat_w),
    .s3_sel_o (wb_s3_sel),
    .s3_we_o  (wb_s3_we),
    .s3_cyc_o (wb_s3_cyc),
    .s3_stb_o (wb_s3_stb),
    .s3_dat_i (wb_s3_dat_r),
    .s3_ack_i (wb_s3_ack),

    // Slave 4: ASTRA-DAAQ Power Controller
    .s4_adr_o (wb_s4_adr),
    .s4_dat_o (wb_s4_dat_w),
    .s4_sel_o (wb_s4_sel),
    .s4_we_o  (wb_s4_we),
    .s4_cyc_o (wb_s4_cyc),
    .s4_stb_o (wb_s4_stb),
    .s4_dat_i (wb_s4_dat_r),
    .s4_ack_i (wb_s4_ack),

    // Slave 5: 4-Lane INT16 SIMD Accelerator
    .s5_adr_o (wb_s5_adr),
    .s5_dat_o (wb_s5_dat_w),
    .s5_sel_o (wb_s5_sel),
    .s5_we_o  (wb_s5_we),
    .s5_cyc_o (wb_s5_cyc),
    .s5_stb_o (wb_s5_stb),
    .s5_dat_i (wb_s5_dat_r),
    .s5_ack_i (wb_s5_ack),

    // Slave 6: IEEE-754 FP32 Engine & Timer
    .s6_adr_o (wb_s6_adr),
    .s6_dat_o (wb_s6_dat_w),
    .s6_sel_o (wb_s6_sel),
    .s6_we_o  (wb_s6_we),
    .s6_cyc_o (wb_s6_cyc),
    .s6_stb_o (wb_s6_stb),
    .s6_dat_i (wb_s6_dat_r),
    .s6_ack_i (wb_s6_ack)
);

// Slave 0 Instance: Data SRAM (16KB BRAM)
wb_bram_dmem u_wb_dmem (
    .clk      (clk_sys),
    .reset    (reset),
    .wb_adr_i (wb_s0_adr),
    .wb_dat_i (wb_s0_dat_w),
    .wb_sel_i (wb_s0_sel),
    .wb_we_i  (wb_s0_we),
    .wb_stb_i (wb_s0_stb),
    .wb_cyc_i (wb_s0_cyc),
    .wb_dat_o (wb_s0_dat_r),
    .wb_ack_o (wb_s0_ack)
);

// Slave 1 Instance: ASTRA-X1 NPU Accelerator (0x4000_0000)
wb_astra_npu u_astra_npu (
    .clk        (clk_sys),
    .reset      (reset),
    .wb_adr_i   (wb_s1_adr),
    .wb_dat_i   (wb_s1_dat_w),
    .wb_sel_i   (wb_s1_sel),
    .wb_we_i    (wb_s1_we),
    .wb_stb_i   (wb_s1_stb),
    .wb_cyc_i   (wb_s1_cyc),
    .wb_dat_o   (wb_s1_dat_r),
    .wb_ack_o   (wb_s1_ack),
    .npu_irq    (npu_irq_w),
    .npu_busy_o (npu_busy_w)
);

// L1 Instruction Cache & L1 Data Cache / MMIO Hardware Engine (Slide 12)
wire [31:0] icache_hits_w;
wire [31:0] icache_misses_w;
wire [31:0] dcache_hits_w;
wire [31:0] dcache_misses_w;
wire [31:0] dcache_snoops_w;
wire [31:0] mmio_accesses_w;
wire        cache_flush_w;

astra_l1_caches u_l1_caches (
    .clk             (clk_sys),
    .reset           (reset),
    .cache_flush_i   (cache_flush_w),
    // L1 I-Cache Port
    .if_valid_i      (if1_valid),
    .if_stall_i      (stall_if),
    .if_pc_i         (if1_pc),
    .imem_rdata_i    (imem_rdata),
    // L1 D-Cache / MMIO Port (CPU Master 0)
    .cpu_cyc_i       (wb_m_cyc),
    .cpu_stb_i       (wb_m_stb),
    .cpu_we_i        (wb_m_we),
    .cpu_sel_i       (wb_m_sel),
    .cpu_adr_i       (wb_m_adr),
    .cpu_dat_w_i     (wb_m_dat_w),
    .bus_dat_r_i     (wb_m_dat_r),
    .bus_ack_i       (wb_m_ack),
    // Hardware DMA Write-Snoop Coherency Port (Master 1)
    .dma_cyc_i       (wb_m1_cyc),
    .dma_stb_i       (wb_m1_stb),
    .dma_we_i        (wb_m1_we),
    .dma_sel_i       (wb_m1_sel),
    .dma_adr_i       (wb_m1_adr),
    .dma_dat_w_i     (wb_m1_dat_w),
    .dma_ack_i       (wb_m1_ack),
    // Telemetry Outputs
    .icache_hits_o   (icache_hits_w),
    .icache_misses_o (icache_misses_w),
    .dcache_hits_o   (dcache_hits_w),
    .dcache_misses_o (dcache_misses_w),
    .dcache_snoops_o (dcache_snoops_w),
    .mmio_accesses_o (mmio_accesses_w)
);

// Slave 2 Instance: Peripherals (GPIO / UART / SPI / I2C + L1 Cache Telemetry at 0xFFFF_0000)
wb_peripherals #(
    .CLK_FREQ(50_000_000),
    .BAUD    (115200)
) u_peripherals (
    .clk               (clk_sys),
    .reset             (reset),
    .wb_adr_i          (wb_s2_adr),
    .wb_dat_i          (wb_s2_dat_w),
    .wb_sel_i          (wb_s2_sel),
    .wb_we_i           (wb_s2_we),
    .wb_stb_i          (wb_s2_stb),
    .wb_cyc_i          (wb_s2_cyc),
    .wb_dat_o          (wb_s2_dat_r),
    .wb_ack_o          (wb_s2_ack),
    .icache_hits_i     (icache_hits_w),
    .icache_misses_i   (icache_misses_w),
    .dcache_hits_i     (dcache_hits_w),
    .dcache_misses_i   (dcache_misses_w),
    .dcache_snoops_i   (dcache_snoops_w),
    .mmio_accesses_i   (mmio_accesses_w),
    .cache_flush_o     (cache_flush_w),
    .uart_rx           (uart_rx),
    .uart_tx           (uart_tx),
    .dbg_led           (dbg_led),
    .dbg_led_hi        (dbg_led_hi),
    .sw_i              (sw_i),
    .pb_i              (pb_i),
    .seven_seg_digit_o (seven_seg_digit_o),
    .seven_seg_seg_o   (seven_seg_seg_o),
    .spi_sck_o         (spi_sck_o),
    .spi_cs_n_o        (spi_cs_n_o),
    .spi_mosi_o        (spi_mosi_o),
    .spi_miso_i        (spi_mosi_o),
    .i2c_scl_o         (i2c_scl_o),
    .i2c_sda_o         (i2c_sda_o),
    .uart_rx_irq_o     (uart_rx_irq_w)
);

// Slave 3 & Master 1 Instance: Autonomous Wishbone DMA Controller (0x4000_1000)
wb_dma_controller u_dma_ctrl (
    .clk        (clk_sys),
    .reset      (reset),

    // Slave 3 Port (CPU MMIO Control/Status)
    .s_wb_adr_i (wb_s3_adr),
    .s_wb_dat_i (wb_s3_dat_w),
    .s_wb_sel_i (wb_s3_sel),
    .s_wb_we_i  (wb_s3_we),
    .s_wb_stb_i (wb_s3_stb),
    .s_wb_cyc_i (wb_s3_cyc),
    .s_wb_dat_o (wb_s3_dat_r),
    .s_wb_ack_o (wb_s3_ack),

    // Master 1 Port (Autonomous Bus Transfers)
    .m_wb_adr_o (wb_m1_adr),
    .m_wb_dat_o (wb_m1_dat_w),
    .m_wb_sel_o (wb_m1_sel),
    .m_wb_we_o  (wb_m1_we),
    .m_wb_stb_o (wb_m1_stb),
    .m_wb_cyc_o (wb_m1_cyc),
    .m_wb_dat_i (wb_m1_dat_r),
    .m_wb_ack_i (wb_m1_ack),
    .m_wb_err_i (wb_m1_err),

    .dma_irq    (dma_irq_w),
    .dma_busy_o (dma_busy_w)
);

// Slave 4 Instance: ASTRA-DAAQ Runtime Dependency-Aware Power Controller (0x4000_2000)
wb_astra_daaq u_astra_daaq (
    .clk            (clk_sys),
    .reset          (reset),
    .wb_adr_i       (wb_s4_adr),
    .wb_dat_i       (wb_s4_dat_w),
    .wb_sel_i       (wb_s4_sel),
    .wb_we_i        (wb_s4_we),
    .wb_stb_i       (wb_s4_stb),
    .wb_cyc_i       (wb_s4_cyc),
    .wb_dat_o       (wb_s4_dat_r),
    .wb_ack_o       (wb_s4_ack),
    .npu_busy_i     (npu_busy_w),
    .dma_busy_i     (dma_busy_w),
    .simd_busy_i    (simd_busy_w),
    .fpu_busy_i     (fpu_busy_w),
    .npu_bus_req_i  (wb_s1_cyc && wb_s1_stb),
    .dma_bus_req_i  (wb_s3_cyc && wb_s3_stb),
    .simd_bus_req_i (wb_s5_cyc && wb_s5_stb),
    .fpu_bus_req_i  (wb_s6_cyc && wb_s6_stb),
    .ce_npu_o       (daaq_ce_npu),
    .ce_dma_o       (daaq_ce_dma),
    .ce_simd_o      (daaq_ce_simd),
    .ce_fpu_o       (daaq_ce_fpu)
);

// Slave 5 Instance: 4-Lane 16-bit Packed SIMD Vector Accelerator (0x4000_3000)
wb_simd_unit u_simd_unit (
    .clk         (clk_sys),
    .reset       (reset),
    .wb_adr_i    (wb_s5_adr),
    .wb_dat_i    (wb_s5_dat_w),
    .wb_sel_i    (wb_s5_sel),
    .wb_we_i     (wb_s5_we),
    .wb_stb_i    (wb_s5_stb),
    .wb_cyc_i    (wb_s5_cyc),
    .wb_dat_o    (wb_s5_dat_r),
    .wb_ack_o    (wb_s5_ack),
    .simd_busy_o (simd_busy_w)
);

// Slave 6 Instance: IEEE-754 Single-Precision FP32 Engine, Timer & Interrupt Unit (0x4000_4000)
wb_fp32_unit u_fp32_unit (
    .clk           (clk_sys),
    .reset         (reset),
    .wb_adr_i      (wb_s6_adr),
    .wb_dat_i      (wb_s6_dat_w),
    .wb_sel_i      (wb_s6_sel),
    .wb_we_i       (wb_s6_we),
    .wb_stb_i      (wb_s6_stb),
    .wb_cyc_i      (wb_s6_cyc),
    .wb_dat_o      (wb_s6_dat_r),
    .wb_ack_o      (wb_s6_ack),
    .npu_irq_i     (npu_irq_w),
    .dma_irq_i     (dma_irq_w),
    .uart_rx_irq_i (uart_rx_irq_w),
    .fpu_busy_o    (fpu_busy_w),
    .timer_irq_o   (timer_irq_w),
    .soc_irq_o     (soc_irq_w)
);

// Backward compatibility alias for testbenches monitoring UART TX state
wire uart_tx_busy = u_peripherals.uart_tx_busy;

//=============================================================
// PIPELINE PREDICTED PC PROPAGATION
//=============================================================
wire [31:0] branch_recovery_pc = ex2_branch_taken ? ex2_branch_target : (ex2_pc + 32'd4);
wire ex2_is_control_flow   = ex2_valid && (ex2_ctrl_bus[12] || ex2_ctrl_bus[11] || ex2_ctrl_bus[10]);
wire branch_mispredict     = ex2_is_control_flow && (branch_recovery_pc != ex2_pred_pc);

//=============================================================
// BRANCH PREDICTOR
//=============================================================
wire [31:0] bpu_predicted_pc;
wire        predicted_valid;

wire ex2_is_branch_instr = ex2_valid && (ex2_ctrl_bus[12] || ex2_ctrl_bus[11] || ex2_ctrl_bus[10]);

branch_predictor bpu (
    .clk(clk_sys),
    .reset(reset),

    .if_pc(if1_pc),
    .predicted_pc(bpu_predicted_pc),
    .predicted_valid(predicted_valid),

    .is_branch_instr(ex2_is_branch_instr),
    .branch_taken(ex2_branch_taken),
    .branch_pc   (ex2_pc), 
    .branch_target(ex2_branch_target)
);

//=============================================================
// IF1 STAGE
//=============================================================

wire flush_if = flush_id | flush_ex1;

if1 if1_stage (
    .clk_if1(clk_if1),
    .reset(reset),

    .stall_if(stall_if),
    .flush_if(flush_if),

    .branch_mispredict(branch_mispredict),
    .branch_recovery_pc(branch_recovery_pc),

    .predicted_pc(bpu_predicted_pc),
    .predicted_valid(predicted_valid),

    .pc_out(if1_pc),
    .if1_valid(if1_valid),
    .if1_pred_pc(if1_pred_pc),
    .imem_addr(imem_addr)
);

//=============================================================
// IF2 STAGE
//=============================================================

if2 if2_stage (
    .clk_if2(clk_if2),
    .reset(reset),

    .if1_pc(if1_pc),
    .if1_instr(imem_rdata),
    .if1_valid(if1_valid),
    .if1_pred_pc(if1_pred_pc),

    .stall_if2(stall_id),
    .flush_if2(flush_id),

    .id_pc(id_pc),
    .id_instr(id_instr),
    .id_valid(id_valid),
    .id_pred_pc(id_pred_pc)
);

//=============================================================
//  RISC-V 7-Stage RV32IM Processor Top-Level
//  PART 2 / 4
//=============================================================


//=============================================================
// REGISTER FILE
//=============================================================
wire        rf_we;
wire [4:0]  rf_waddr;
wire [31:0] rf_wdata;
wire [4:0]  id_rs1_addr;
wire [4:0]  id_rs2_addr;
wire [31:0] id_rs1_data;
wire [31:0] id_rs2_data;

register_file rf (
    .clk(clk_wb),           // Writes occur in WB stage
    .we(rf_we),
    .waddr(rf_waddr),
    .wdata(rf_wdata),

    .raddr1(id_rs1_addr),
    .raddr2(id_rs2_addr),
    .rdata1(id_rs1_data),
    .rdata2(id_rs2_data)
);


//=============================================================
// HAZARD UNIT (RAW + load-use stalls)
//=============================================================

hazard_unit hazard (
    .id_valid(id_valid),
    .id_rs1(id_instr[19:15]),
    .id_rs2(id_instr[24:20]),

    .ex1_valid(ex1_valid),
    .ex1_rd(ex1_rd),
    .ex1_mem_read(ex1_ctrl_bus[14]),

    .ex2_valid(ex2_valid),
    .ex2_rd(ex2_rd),
    .ex2_mem_read(ex2_ctrl_bus[14]),

    .mem_valid(mem_valid),
    .mem_rd(mem_rd),
    .mem_mem_read(mem_ctrl_bus[14]),

    .stall_if(stall_if),
    .stall_id(stall_id),
    .stall_ex1(stall_ex1),
    .flush_id(flush_id),
    .flush_ex1(flush_ex1),
    .mul_busy(mul_busy),
    .div_busy(div_busy),
    .stall_ex2(stall_ex2),
    .stall_mem(stall_mem),
    .branch_mispredict(branch_mispredict)
);


//=============================================================
// FORWARDING UNIT
//=============================================================

wire [31:0] fwd_rs1;
wire [31:0] fwd_rs2;
wire [31:0] ex1_rs1;
wire [31:0] ex1_rs2;

forwarding_unit fwd_unit (
    .id_rs1(ex1_instr[19:15]),
    .id_rs2(ex1_instr[24:20]),

    // EX2 stage forwarding
    .ex2_valid(ex2_valid && ex2_ctrl_bus[15]),
    .ex2_rd(ex2_rd),
    .ex2_result(ex2_comb_result),

    // MEM stage forwarding
    .mem_valid(mem_valid && mem_ctrl_bus[15]),
    .mem_rd(mem_rd),
    .mem_result(mem_result),

    // WB stage forwarding
    .wb_valid(wb_valid && wb_ctrl_bus[15]),
    .wb_rd(wb_rd),
    .wb_result(wb_data),

    // RF Write (post-WB) forwarding
    .rf_we(rf_we),
    .rf_waddr(rf_waddr),
    .rf_wdata(rf_wdata),

    .fwd_rs1_val(fwd_rs1),
    .fwd_rs2_val(fwd_rs2),

    .original_rs1(ex1_rs1),
    .original_rs2(ex1_rs2)
);


//=============================================================
// ID STAGE
//=============================================================

wire is_branch_id;
wire is_load_id;
wire is_store_id;
wire is_mul_id;
wire is_div_id;
wire is_approx_id;

id id_stage (
    .clk_id(clk_id),
    .reset(reset),

    .id_pc_in(id_pc),
    .id_instr_in(id_instr),
    .id_valid_in(id_valid),
    .id_pred_pc(id_pred_pc),

    .stall_id(stall_id),
    .stall_ex1(stall_ex1),
    .flush_id(flush_id),

    .rs1_addr(id_rs1_addr),
    .rs2_addr(id_rs2_addr),
    .rs1_data(id_rs1_data),
    .rs2_data(id_rs2_data),

    .ex1_pc(ex1_pc),
    .ex1_rs1(ex1_rs1),
    .ex1_rs2(ex1_rs2),
    .ex1_rd(ex1_rd),
    .ex1_imm(ex1_imm),
    .ex1_instr(ex1_instr),
    .ex1_ctrl_bus(ex1_ctrl_bus),
    .ex1_valid(ex1_valid),
    .ex1_pred_pc(ex1_pred_pc),

    .is_branch(is_branch_id),
    .is_load(is_load_id),
    .is_store(is_store_id),
    .is_mul(is_mul_id),
    .is_div(is_div_id),
    .is_approx(is_approx_id)
);


//=============================================================
// CSR + POWER MODE + APPROX MODE REGISTERS
//=============================================================

wire        wfi_active;

// Debug output

//=============================================================
//  RISC-V 7-Stage RV32IM Processor Top-Level
//  PART 3 / 4
//=============================================================


//=============================================================
// EX1 STAGE
//=============================================================

wire mul_start_ex1;
wire div_start_ex1;

ex1 ex1_stage (
    .clk_ex1(clk_ex1),
    .reset(reset),

    .ex1_valid_in(ex1_valid),
    .id_pc(ex1_pc),
    .id_instr(ex1_instr),
    .id_rs1(ex1_rs1),
    .id_rs2(ex1_rs2),
    .id_imm(ex1_imm),
    .id_rd(ex1_rd),
    .id_ctrl_bus(ex1_ctrl_bus),
    .ex1_pred_pc(ex1_pred_pc),

    .fwd_rs1_val(fwd_rs1),
    .fwd_rs2_val(fwd_rs2),

    .stall_ex1(stall_ex1),
    .flush_ex1(flush_ex1),

    // DIV state for start pulse gating
    .div_busy(div_busy),

    // EX1 → EX2 outputs
    .ex2_valid(ex2_valid),
    .ex2_pc(ex2_pc),
    .ex2_op_a(ex2_op_a),
    .ex2_op_b(ex2_op_b),
    .ex2_imm(ex2_imm),
    .ex2_instr(ex2_instr),
    .ex2_rd(ex2_rd),
    .ex2_ctrl_bus(ex2_ctrl_bus),
    .ex2_store_data(ex2_store_data),
    .ex2_pred_pc(ex2_pred_pc),

    // Branch resolution
    .ex2_branch_taken(ex2_branch_taken),
    .ex2_branch_target(ex2_branch_target),

    // MUL/DIV start
    .mul_start(mul_start_ex1),
    .div_start(div_start_ex1)
);


//=============================================================
// CSR UNIT
//=============================================================

wire [31:0] csr_rdata;
wire        csr_approx_mode;
wire [7:0]  csr_power_mode;

csr_unit core_csr (
    .clk(clk_sys),
    .reset(reset),

    .csr_op(ex2_ctrl_bus[9]),
    .csr_addr(ex2_instr[31:20]),
    .csr_funct3(ex2_instr[14:12]),
    .csr_wdata(ex2_op_a),
    .csr_rdata(csr_rdata),

    .inst_retired(wb_valid),
    .timer_irq_i(timer_irq_w),
    .ext_irq_i(soc_irq_w),
    
    .approx_mode(csr_approx_mode),
    .power_mode(csr_power_mode)
);

//=============================================================
// MULTIPLIER UNIT (DSP-Optimized)
//=============================================================

wire [31:0] mul_result;

mul_unit mul_u (
    .clk(clk_sys),
    .reset(reset),

    .start(mul_start_ex1),
    .op_type(ex2_ctrl_bus[18:16]),
    .op_a(ex2_op_a),
    .op_b(ex2_op_b),

    .result(mul_result),
    .busy(mul_busy)
);


//=============================================================
// DIVIDER UNIT (Iterative 32-Cycle Restoring Divider)
//=============================================================

wire [31:0] div_result;

div_unit div_u (
    .clk(clk_ex2),
    .reset(reset),

    .start(div_start_ex1),
    .op_type(ex2_ctrl_bus[18:16]),
    .op_a(ex2_op_a),
    .op_b(ex2_op_b),

    .result(div_result),
    .busy(div_busy)
);


//=============================================================
// EX2 STAGE
//=============================================================

ex2 ex2_stage (
    .clk_ex2(clk_ex2),
    .reset(reset),

    .ex2_valid_in(ex2_valid),
    .ex2_pc_in(ex2_pc),
    .ex2_op_a_in(ex2_op_a),
    .ex2_op_b_in(ex2_op_b),
    .ex2_instr_in(ex2_instr),
    .ex2_imm_in(ex2_imm),
    .ex2_rd_in(ex2_rd),
    .ex2_ctrl_bus_in(ex2_ctrl_bus),
    .ex2_store_data_in(ex2_store_data),
    .mem_instr(mem_instr),

    .mul_start(mul_start_ex1),
    .div_start(div_start_ex1),

    .approx_enable(csr_approx_mode),

    .mul_result(mul_result),
    .mul_busy(mul_busy),
    .div_result(div_result),
    .div_busy(div_busy),
    .uart_tx_busy(1'b0),

    .csr_rdata(csr_rdata),
    .stall_mem(stall_mem),

    // Outputs to MEM
    .mem_valid(mem_valid),
    .mem_result(mem_result),
    .mem_pc(mem_pc),
    .mem_rd(mem_rd),
    .mem_ctrl_bus(mem_ctrl_bus),
    .mem_store_data(mem_store_data),

    .stall_ex2(stall_ex2),
    .ex2_comb_result(ex2_comb_result)
);


//=============================================================
// MEMORY STAGE (LOAD/STORE) with Wishbone Master
//=============================================================

mem_stage mem_stg (
    .clk_mem(clk_mem),
    .reset(reset),

    .mem_valid_in(mem_valid),

    .ex2_result(mem_result),
    .ex2_op_b(mem_store_data),
    .ex2_pc(mem_pc),
    .ex2_rd(mem_rd),
    .ex2_ctrl_bus(mem_ctrl_bus),
    .mem_instr_in(mem_instr),

    // Wishbone Master Interface
    .wb_adr_o(wb_m_adr),
    .wb_dat_o(wb_m_dat_w),
    .wb_sel_o(wb_m_sel),
    .wb_we_o(wb_m_we),
    .wb_cyc_o(wb_m_cyc),
    .wb_stb_o(wb_m_stb),
    .wb_dat_i(wb_m_dat_r),
    .wb_ack_i(wb_m_ack),
    .wb_err_i(wb_m_err),

    .stall_mem(stall_mem),

    // Outputs to WB
    .wb_instr(wb_instr),
    .wb_valid(wb_valid),
    .wb_result(wb_data),
    .wb_pc(wb_pc),
    .wb_rd(wb_rd),
    .wb_ctrl_bus(wb_ctrl_bus)
);

//=============================================================
//  RISC-V 7-Stage RV32IM Processor Top-Level
//  PART 4 / 4 - FINAL
//=============================================================


//=============================================================
// WRITEBACK (WB) STAGE
//=============================================================

wb_stage wb (
    .clk_wb(clk_wb),
    .reset(reset),

    .wb_valid_in(wb_valid),
    .wb_pc_in(wb_pc),
    .wb_data_in(wb_data),
    .wb_ctrl_bus_in(wb_ctrl_bus),
    .wb_rd_in(wb_rd),
    .wb_instr_in(wb_instr),  // instruction passes through EX2/MEM path

    // Register file write port
    .rf_we(rf_we),
    .rf_waddr(rf_waddr),
    .rf_wdata(rf_wdata),

    // WFI / Halt
    .wfi_active(wfi_active)
);



//=============================================================
// STAGE GATING CONTROLLER
//=============================================================

stage_gating_controller sg_ctrl (
    .clk(clk_sys),
    .reset(reset),

    // From ID (instruction class)
    .is_branch(is_branch_id),
    .is_load(is_load_id),
    .is_store(is_store_id),
    .is_mul(is_mul_id),
    .is_div(is_div_id),
    .is_approx(is_approx_id),

    // Stall + flush
    .stall_if(stall_if),
    .stall_id(stall_id),
    .stall_ex(stall_ex1 | stall_ex2),
    .flush_pipeline(flush_id | flush_ex1),

    // CSR power modes
    .pmode(csr_power_mode),
    .wfi_active(wfi_active),

    // MUL/DIV busy signals
    .mul_busy(mul_busy),
    .div_busy(div_busy),

    // CLOCK ENABLE OUTPUTS
    .ce_if1(ce_if1),
    .ce_if2(ce_if2),
    .ce_id(ce_id),
    .ce_ex1(ce_ex1),
    .ce_ex2(ce_ex2),
    .ce_mem(ce_mem),
    .ce_wb(ce_wb),

    .ce_mul(ce_mul),
    .ce_div(ce_div),
    .ce_approx(ce_approx),

    .ce_uart(ce_uart),
    .ce_csr(ce_csr)
);


//=============================================================
// DEBUG OUTPUTS
//=============================================================

//assign dbg_pc = if1_pc;
assign dbg_wfi_active = wfi_active;

// Simple LED debug - show lower 8 bits of writeback data
//assign dbg_led = rf_wdata[7:0];


//=============================================================
// END OF TOP-LEVEL
//=============================================================

endmodule
