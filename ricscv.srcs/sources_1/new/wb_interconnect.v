`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: 32-bit Multi-Master Wishbone B4 Crossbar / Interconnect
// Connects 2 Masters to 7 Slaves (Full Heterogeneous SoC Map):
//   Master 0: 7-Stage RV32IM CPU Memory Stage
//   Master 1: Autonomous DMA Controller (wb_dma_controller)
//
//   Slave 0: Data SRAM (BRAM)       -> 0x0000_0000..0x0000_3FFF / 0x2000_0000..0x2000_FFFF
//   Slave 1: ASTRA-X1 NPU           -> 0x4000_0000..0x4000_0FFF ("ASTR")
//   Slave 2: Peripherals (UART/LED) -> 0xFFFF_0000..0xFFFF_000F
//   Slave 3: DMA Controller MMIO    -> 0x4000_1000..0x4000_1FFF ("DMA1")
//   Slave 4: ASTRA-DAAQ Power Ctrl  -> 0x4000_2000..0x4000_2FFF ("DAAQ")
//   Slave 5: 4-Lane INT16 SIMD Unit -> 0x4000_3000..0x4000_3FFF ("SIMD")
//   Slave 6: IEEE-754 FP32 & Timer  -> 0x4000_4000..0x4000_4FFF ("FPU1")
//
// Crossbar Concurrency:
//   - Slave 0 (SRAM), Slave 1 (NPU), Slave 5 (SIMD), and Slave 6 (FP32) are
//     accessible by both Master 0 (CPU) and Master 1 (Autonomous DMA).
//   - Slave 2 (Peripherals), Slave 3 (DMA MMIO), and Slave 4 (DAAQ MMIO) have
//     a dedicated non-blocking path to Master 0 (CPU).
//=============================================================================

module wb_interconnect (
    input  wire        clk,
    input  wire        reset,

    // ========================================================
    // Master 0 Interface (from CPU Memory Stage)
    // ========================================================
    input  wire [31:0] m_adr_i,
    input  wire [31:0] m_dat_i,
    input  wire [3:0]  m_sel_i,
    input  wire        m_we_i,
    input  wire        m_cyc_i,
    input  wire        m_stb_i,
    output reg  [31:0] m_dat_o,
    output reg         m_ack_o,
    output reg         m_err_o,

    // ========================================================
    // Master 1 Interface (from Autonomous DMA Controller)
    // ========================================================
    input  wire [31:0] m1_adr_i,
    input  wire [31:0] m1_dat_i,
    input  wire [3:0]  m1_sel_i,
    input  wire        m1_we_i,
    input  wire        m1_cyc_i,
    input  wire        m1_stb_i,
    output reg  [31:0] m1_dat_o,
    output reg         m1_ack_o,
    output reg         m1_err_o,

    // ========================================================
    // Slave 0 Interface: Data SRAM (0x0000_xxxx / 0x2000_xxxx)
    // ========================================================
    output wire [31:0] s0_adr_o,
    output wire [31:0] s0_dat_o,
    output wire [3:0]  s0_sel_o,
    output wire        s0_we_o,
    output wire        s0_cyc_o,
    output wire        s0_stb_o,
    input  wire [31:0] s0_dat_i,
    input  wire        s0_ack_i,

    // ========================================================
    // Slave 1 Interface: ASTRA-X1 NPU (0x4000_0xxx)
    // ========================================================
    output wire [31:0] s1_adr_o,
    output wire [31:0] s1_dat_o,
    output wire [3:0]  s1_sel_o,
    output wire        s1_we_o,
    output wire        s1_cyc_o,
    output wire        s1_stb_o,
    input  wire [31:0] s1_dat_i,
    input  wire        s1_ack_i,

    // ========================================================
    // Slave 2 Interface: Peripherals (0xFFFF_xxxx)
    // ========================================================
    output wire [31:0] s2_adr_o,
    output wire [31:0] s2_dat_o,
    output wire [3:0]  s2_sel_o,
    output wire        s2_we_o,
    output wire        s2_cyc_o,
    output wire        s2_stb_o,
    input  wire [31:0] s2_dat_i,
    input  wire        s2_ack_i,

    // ========================================================
    // Slave 3 Interface: DMA Controller MMIO (0x4000_1xxx)
    // ========================================================
    output wire [31:0] s3_adr_o,
    output wire [31:0] s3_dat_o,
    output wire [3:0]  s3_sel_o,
    output wire        s3_we_o,
    output wire        s3_cyc_o,
    output wire        s3_stb_o,
    input  wire [31:0] s3_dat_i,
    input  wire        s3_ack_i,

    // ========================================================
    // Slave 4 Interface: ASTRA-DAAQ Power Controller (0x4000_2xxx)
    // ========================================================
    output wire [31:0] s4_adr_o,
    output wire [31:0] s4_dat_o,
    output wire [3:0]  s4_sel_o,
    output wire        s4_we_o,
    output wire        s4_cyc_o,
    output wire        s4_stb_o,
    input  wire [31:0] s4_dat_i,
    input  wire        s4_ack_i,

    // ========================================================
    // Slave 5 Interface: 4-Lane INT16 SIMD Engine (0x4000_3xxx)
    // ========================================================
    output wire [31:0] s5_adr_o,
    output wire [31:0] s5_dat_o,
    output wire [3:0]  s5_sel_o,
    output wire        s5_we_o,
    output wire        s5_cyc_o,
    output wire        s5_stb_o,
    input  wire [31:0] s5_dat_i,
    input  wire        s5_ack_i,

    // ========================================================
    // Slave 6 Interface: IEEE-754 FP32 Engine & Timer (0x4000_4xxx)
    // ========================================================
    output wire [31:0] s6_adr_o,
    output wire [31:0] s6_dat_o,
    output wire [3:0]  s6_sel_o,
    output wire        s6_we_o,
    output wire        s6_cyc_o,
    output wire        s6_stb_o,
    input  wire [31:0] s6_dat_i,
    input  wire        s6_ack_i
);

    // --------------------------------------------------------
    // Master 0 (CPU) Address Decoding
    // --------------------------------------------------------
    wire m0_sel_s0 = (m_adr_i[31:28] == 4'h0) || (m_adr_i[31:28] == 4'h2);
    wire m0_sel_s1 = (m_adr_i[31:12] == 20'h40000);
    wire m0_sel_s2 = (m_adr_i[31:16] == 16'hFFFF);
    wire m0_sel_s3 = (m_adr_i[31:12] == 20'h40001);
    wire m0_sel_s4 = (m_adr_i[31:12] == 20'h40002);
    wire m0_sel_s5 = (m_adr_i[31:12] == 20'h40003);
    wire m0_sel_s6 = (m_adr_i[31:12] == 20'h40004);
    wire m0_sel_unmapped = m_cyc_i && m_stb_i &&
                           !m0_sel_s0 && !m0_sel_s1 && !m0_sel_s2 &&
                           !m0_sel_s3 && !m0_sel_s4 && !m0_sel_s5 && !m0_sel_s6;

    // --------------------------------------------------------
    // Master 1 (DMA) Address Decoding
    // --------------------------------------------------------
    wire m1_sel_s0 = (m1_adr_i[31:28] == 4'h0) || (m1_adr_i[31:28] == 4'h2);
    wire m1_sel_s1 = (m1_adr_i[31:12] == 20'h40000);
    wire m1_sel_s5 = (m1_adr_i[31:12] == 20'h40003);
    wire m1_sel_s6 = (m1_adr_i[31:12] == 20'h40004);
    wire m1_sel_unmapped = m1_cyc_i && m1_stb_i &&
                           !m1_sel_s0 && !m1_sel_s1 && !m1_sel_s5 && !m1_sel_s6;

    // --------------------------------------------------------
    // Shared Bus Arbitration (Slave 0, Slave 1, Slave 5, Slave 6)
    // Transaction-locked so a 2-cycle (req -> ack) operation is never split
    // --------------------------------------------------------
    wire m0_req_shared = m_cyc_i  && m_stb_i  && (m0_sel_s0 || m0_sel_s1 || m0_sel_s5 || m0_sel_s6);
    wire m1_req_shared = m1_cyc_i && m1_stb_i && (m1_sel_s0 || m1_sel_s1 || m1_sel_s5 || m1_sel_s6);

    reg m0_inflight;
    reg m1_inflight;

    wire grant_m1 = m1_inflight ? 1'b1 :
                    m0_inflight ? 1'b0 :
                    (m1_req_shared && !m0_req_shared);

    wire shared_ack = s0_ack_i || s1_ack_i || s5_ack_i || s6_ack_i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            m0_inflight <= 1'b0;
            m1_inflight <= 1'b0;
        end else begin
            if (shared_ack) begin
                m0_inflight <= 1'b0;
                m1_inflight <= 1'b0;
            end else if (grant_m1 && m1_req_shared) begin
                m1_inflight <= 1'b1;
            end else if (!grant_m1 && m0_req_shared) begin
                m0_inflight <= 1'b1;
            end
        end
    end

    // Shared Bus Multiplexed Signals (driving Slave 0, 1, 5, 6)
    wire [31:0] sh_adr = grant_m1 ? m1_adr_i : m_adr_i;
    wire [31:0] sh_dat = grant_m1 ? m1_dat_i : m_dat_i;
    wire [3:0]  sh_sel = grant_m1 ? m1_sel_i : m_sel_i;
    wire        sh_we  = grant_m1 ? m1_we_i  : m_we_i;
    wire        sh_cyc = grant_m1 ? m1_cyc_i : m_cyc_i;
    wire        sh_stb = grant_m1 ? m1_stb_i : m_stb_i;
    wire        sh_s0  = grant_m1 ? m1_sel_s0 : m0_sel_s0;
    wire        sh_s1  = grant_m1 ? m1_sel_s1 : m0_sel_s1;
    wire        sh_s5  = grant_m1 ? m1_sel_s5 : m0_sel_s5;
    wire        sh_s6  = grant_m1 ? m1_sel_s6 : m0_sel_s6;

    // --------------------------------------------------------
    // Drive Shared Slaves: Slave 0 (SRAM), Slave 1 (NPU), Slave 5 (SIMD), Slave 6 (FP32)
    // --------------------------------------------------------
    assign s0_adr_o = sh_adr;
    assign s0_dat_o = sh_dat;
    assign s0_sel_o = sh_sel;
    assign s0_we_o  = sh_we;
    assign s0_cyc_o = sh_cyc && sh_s0;
    assign s0_stb_o = sh_stb && sh_s0;

    assign s1_adr_o = sh_adr;
    assign s1_dat_o = sh_dat;
    assign s1_sel_o = sh_sel;
    assign s1_we_o  = sh_we;
    assign s1_cyc_o = sh_cyc && sh_s1;
    assign s1_stb_o = sh_stb && sh_s1;

    assign s5_adr_o = sh_adr;
    assign s5_dat_o = sh_dat;
    assign s5_sel_o = sh_sel;
    assign s5_we_o  = sh_we;
    assign s5_cyc_o = sh_cyc && sh_s5;
    assign s5_stb_o = sh_stb && sh_s5;

    assign s6_adr_o = sh_adr;
    assign s6_dat_o = sh_dat;
    assign s6_sel_o = sh_sel;
    assign s6_we_o  = sh_we;
    assign s6_cyc_o = sh_cyc && sh_s6;
    assign s6_stb_o = sh_stb && sh_s6;

    // --------------------------------------------------------
    // Drive Dedicated CPU Slaves: Slave 2 (Peripherals), Slave 3 (DMA MMIO), Slave 4 (DAAQ MMIO)
    // --------------------------------------------------------
    assign s2_adr_o = m_adr_i;
    assign s2_dat_o = m_dat_i;
    assign s2_sel_o = m_sel_i;
    assign s2_we_o  = m_we_i;
    assign s2_cyc_o = m_cyc_i && m0_sel_s2;
    assign s2_stb_o = m_stb_i && m0_sel_s2;

    assign s3_adr_o = m_adr_i;
    assign s3_dat_o = m_dat_i;
    assign s3_sel_o = m_sel_i;
    assign s3_we_o  = m_we_i;
    assign s3_cyc_o = m_cyc_i && m0_sel_s3;
    assign s3_stb_o = m_stb_i && m0_sel_s3;

    assign s4_adr_o = m_adr_i;
    assign s4_dat_o = m_dat_i;
    assign s4_sel_o = m_sel_i;
    assign s4_we_o  = m_we_i;
    assign s4_cyc_o = m_cyc_i && m0_sel_s4;
    assign s4_stb_o = m_stb_i && m0_sel_s4;

    // --------------------------------------------------------
    // Response Multiplexing to Master 0 (CPU)
    // --------------------------------------------------------
    always @(*) begin
        if (m0_sel_s0) begin
            m_dat_o = s0_dat_i;
            m_ack_o = !grant_m1 && s0_ack_i;
            m_err_o = 1'b0;
        end else if (m0_sel_s1) begin
            m_dat_o = s1_dat_i;
            m_ack_o = !grant_m1 && s1_ack_i;
            m_err_o = 1'b0;
        end else if (m0_sel_s2) begin
            m_dat_o = s2_dat_i;
            m_ack_o = s2_ack_i;
            m_err_o = 1'b0;
        end else if (m0_sel_s3) begin
            m_dat_o = s3_dat_i;
            m_ack_o = s3_ack_i;
            m_err_o = 1'b0;
        end else if (m0_sel_s4) begin
            m_dat_o = s4_dat_i;
            m_ack_o = s4_ack_i;
            m_err_o = 1'b0;
        end else if (m0_sel_s5) begin
            m_dat_o = s5_dat_i;
            m_ack_o = !grant_m1 && s5_ack_i;
            m_err_o = 1'b0;
        end else if (m0_sel_s6) begin
            m_dat_o = s6_dat_i;
            m_ack_o = !grant_m1 && s6_ack_i;
            m_err_o = 1'b0;
        end else if (m0_sel_unmapped) begin
            m_dat_o = 32'hDEAD_BEEF;
            m_ack_o = 1'b0;
            m_err_o = 1'b1;
        end else begin
            m_dat_o = 32'h0000_0000;
            m_ack_o = 1'b0;
            m_err_o = 1'b0;
        end
    end

    // --------------------------------------------------------
    // Response Multiplexing to Master 1 (DMA)
    // --------------------------------------------------------
    always @(*) begin
        if (m1_sel_s0) begin
            m1_dat_o = s0_dat_i;
            m1_ack_o = grant_m1 && s0_ack_i;
            m1_err_o = 1'b0;
        end else if (m1_sel_s1) begin
            m1_dat_o = s1_dat_i;
            m1_ack_o = grant_m1 && s1_ack_i;
            m1_err_o = 1'b0;
        end else if (m1_sel_s5) begin
            m1_dat_o = s5_dat_i;
            m1_ack_o = grant_m1 && s5_ack_i;
            m1_err_o = 1'b0;
        end else if (m1_sel_s6) begin
            m1_dat_o = s6_dat_i;
            m1_ack_o = grant_m1 && s6_ack_i;
            m1_err_o = 1'b0;
        end else if (m1_sel_unmapped) begin
            m1_dat_o = 32'hDEAD_BEEF;
            m1_ack_o = 1'b0;
            m1_err_o = 1'b1;
        end else begin
            m1_dat_o = 32'h0000_0000;
            m1_ack_o = 1'b0;
            m1_err_o = 1'b0;
        end
    end

endmodule
