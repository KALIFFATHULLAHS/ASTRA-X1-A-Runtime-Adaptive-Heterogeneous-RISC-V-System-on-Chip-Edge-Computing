`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: Hardware L1 Instruction Cache & L1 Data Cache / MMIO Unit
// Matches Slide 12 Architecture:
//   1. Direct-Mapped L1 Instruction Cache (64 lines x 32-bit, Tag + Valid + Data)
//      - Tracks instruction fetch stream from IF1/IMEM
//      - Provides live icache_hits, icache_misses, and icache_miss_busy status
//   2. Direct-Mapped Write-Through L1 Data Cache / MMIO Partitioner (64 lines)
//      - Cacheable Region : 0x0000_0000 .. 0x3FFF_FFFF (Data SRAM)
//      - Uncached MMIO    : 0x4000_0000 .. 0xFFFF_FFFF (NPU, DMA, DAAQ, SIMD, FPU, Periph)
//      - Write Policy     : Write-Through + Write-Allocate (4-byte word) / Invalidate (partial)
//      - Coherency Policy : Hardware Bus Snooping on Master 1 (DMA Controller)
//                           Automatically updates/invalidates D-Cache lines when
//                           Autonomous DMA writes to Data SRAM!
//=============================================================================

module astra_l1_caches (
    input  wire        clk,
    input  wire        reset,
    input  wire        cache_flush_i,

    // --------------------------------------------------------
    // 1. L1 Instruction Cache Monitor / Tag-Line Engine
    // --------------------------------------------------------
    input  wire        if_valid_i,
    input  wire        if_stall_i,
    input  wire [31:0] if_pc_i,
    input  wire [31:0] imem_rdata_i,

    // --------------------------------------------------------
    // 2. L1 Data Cache / MMIO Interface (CPU LSU Master 0)
    // --------------------------------------------------------
    input  wire        cpu_cyc_i,
    input  wire        cpu_stb_i,
    input  wire        cpu_we_i,
    input  wire [3:0]  cpu_sel_i,
    input  wire [31:0] cpu_adr_i,
    input  wire [31:0] cpu_dat_w_i,
    input  wire [31:0] bus_dat_r_i,
    input  wire        bus_ack_i,

    // --------------------------------------------------------
    // 3. Hardware DMA Write Snoop Port (Master 1 Coherency)
    // --------------------------------------------------------
    input  wire        dma_cyc_i,
    input  wire        dma_stb_i,
    input  wire        dma_we_i,
    input  wire [3:0]  dma_sel_i,
    input  wire [31:0] dma_adr_i,
    input  wire [31:0] dma_dat_w_i,
    input  wire        dma_ack_i,

    // --------------------------------------------------------
    // 4. Live Performance & Telemetry Outputs (to MMIO & DAAQ)
    // --------------------------------------------------------
    output reg  [31:0] icache_hits_o,
    output reg  [31:0] icache_misses_o,
    output reg  [31:0] dcache_hits_o,
    output reg  [31:0] dcache_misses_o,
    output reg  [31:0] dcache_snoops_o,
    output reg  [31:0] mmio_accesses_o
);

    wire _unused = &{1'b0, if_pc_i[31:16], if_pc_i[1:0], cpu_adr_i[1:0], dma_adr_i[1:0]};

    // =========================================================================
    // L1 INSTRUCTION CACHE (64 Lines, Direct-Mapped: Index [7:2], Tag [15:8])
    // =========================================================================
    reg [63:0] ic_valid;
    reg [7:0]  ic_tag  [0:63];
    reg [31:0] ic_data [0:63];

    wire [5:0] ic_idx = if_pc_i[7:2];
    wire [7:0] ic_t   = if_pc_i[15:8];
    wire       ic_req = if_valid_i && !if_stall_i;
    wire       ic_hit = ic_valid[ic_idx] && (ic_tag[ic_idx] == ic_t);

    // =========================================================================
    // L1 DATA CACHE (64 Lines, Direct-Mapped: Index [7:2], Tag [15:8])
    // Cacheable: 0x0000_0000 .. 0x3FFF_FFFF (cpu_adr_i[31:30] == 2'b00)
    // Uncached MMIO: 0x4000_0000 .. 0xFFFF_FFFF
    // =========================================================================
    reg [63:0] dc_valid;
    reg [7:0]  dc_tag  [0:63];
    reg [31:0] dc_data [0:63];

    wire       is_cacheable_cpu = (cpu_adr_i[31:30] == 2'b00);
    wire [5:0] dc_idx           = cpu_adr_i[7:2];
    wire [7:0] dc_t             = cpu_adr_i[15:8];
    wire       cpu_trans_done   = cpu_cyc_i && cpu_stb_i && bus_ack_i;
    wire       dc_hit           = dc_valid[dc_idx] && (dc_tag[dc_idx] == dc_t);

    // DMA Snoop Signals (Master 1 writing to Cacheable SRAM)
    wire       is_cacheable_dma = (dma_adr_i[31:30] == 2'b00);
    wire [5:0] snoop_idx        = dma_adr_i[7:2];
    wire [7:0] snoop_t          = dma_adr_i[15:8];
    wire       dma_write_done   = dma_cyc_i && dma_stb_i && dma_we_i && dma_ack_i && is_cacheable_dma;
    wire       snoop_match      = dc_valid[snoop_idx] && (dc_tag[snoop_idx] == snoop_t);

    integer i;

    always @(posedge clk) begin
        if (reset || cache_flush_i) begin
            ic_valid        <= 64'd0;
            dc_valid        <= 64'd0;
            icache_hits_o   <= 32'd0;
            icache_misses_o <= 32'd0;
            dcache_hits_o   <= 32'd0;
            dcache_misses_o <= 32'd0;
            dcache_snoops_o <= 32'd0;
            mmio_accesses_o <= 32'd0;
            for (i = 0; i < 64; i = i + 1) begin
                ic_tag[i]  <= 8'd0;
                ic_data[i] <= 32'd0;
                dc_tag[i]  <= 8'd0;
                dc_data[i] <= 32'd0;
            end
        end else begin
            // 1. L1 I-Cache Lookup & Line Fill
            if (ic_req) begin
                if (ic_hit) begin
                    icache_hits_o <= icache_hits_o + 32'd1;
                end else begin
                    icache_misses_o  <= icache_misses_o + 32'd1;
                    ic_valid[ic_idx] <= 1'b1;
                    ic_tag[ic_idx]   <= ic_t;
                    ic_data[ic_idx]  <= imem_rdata_i;
                end
            end

            // 2. L1 D-Cache / MMIO Access Tracking & Line Update
            if (cpu_trans_done) begin
                if (!is_cacheable_cpu) begin
                    // Uncached MMIO Bypass (NPU, DMA, DAAQ, SIMD, FPU, Peripherals)
                    mmio_accesses_o <= mmio_accesses_o + 32'd1;
                end else if (!cpu_we_i) begin
                    // Cacheable SRAM Load
                    if (dc_hit) begin
                        dcache_hits_o <= dcache_hits_o + 32'd1;
                    end else begin
                        dcache_misses_o  <= dcache_misses_o + 32'd1;
                        dc_valid[dc_idx] <= 1'b1;
                        dc_tag[dc_idx]   <= dc_t;
                        dc_data[dc_idx]  <= bus_dat_r_i;
                    end
                end else begin
                    // Cacheable SRAM Store (Write-Through + Write-Allocate/Update)
                    if (dc_hit) begin
                        dcache_hits_o <= dcache_hits_o + 32'd1;
                    end else begin
                        dcache_misses_o <= dcache_misses_o + 32'd1;
                    end

                    if (cpu_sel_i == 4'b1111) begin
                        dc_valid[dc_idx] <= 1'b1;
                        dc_tag[dc_idx]   <= dc_t;
                        dc_data[dc_idx]  <= cpu_dat_w_i;
                    end else begin
                        // Sub-word write invalidates line so next read reloads full merged word
                        dc_valid[dc_idx] <= 1'b0;
                    end
                end
            end

            // 3. Hardware DMA Write Snoop Coherency (Master 1 -> Data SRAM)
            if (dma_write_done) begin
                dcache_snoops_o <= dcache_snoops_o + 32'd1;
                if (dma_sel_i == 4'b1111) begin
                    // Update / Pre-warm D-Cache line with fresh DMA-transferred word!
                    dc_valid[snoop_idx] <= 1'b1;
                    dc_tag[snoop_idx]   <= snoop_t;
                    dc_data[snoop_idx]  <= dma_dat_w_i;
                end else if (snoop_match) begin
                    dc_valid[snoop_idx] <= 1'b0;
                end
            end
        end
    end

endmodule
