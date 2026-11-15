`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: Wishbone B4 Data BRAM Memory Slave
// Wraps 16KB (4096 x 32-bit words) synchronous dual-port/single-port BRAM
// Provides 1-cycle latency synchronous read and write with exact byte strobes.
//=============================================================================

module wb_bram_dmem (
    input  wire        clk,
    input  wire        reset,

    // Wishbone B4 Slave Interface
    input  wire [31:0] wb_adr_i,
    input  wire [31:0] wb_dat_i,
    input  wire [3:0]  wb_sel_i,
    input  wire        wb_we_i,
    input  wire        wb_stb_i,
    input  wire        wb_cyc_i,

    output wire [31:0] wb_dat_o,
    output reg         wb_ack_o
);

    wire is_req = wb_cyc_i && wb_stb_i && !wb_ack_o;

    // Instantiate existing bram_dmem module
    wire [13:0] bram_addr = wb_adr_i[15:2];
    wire [3:0]  bram_wstrb = (is_req && wb_we_i) ? wb_sel_i : 4'b0000;

    bram_dmem dmem_core (
        .clk   (clk),
        .addr  (bram_addr),
        .wdata (wb_dat_i),
        .wstrb (bram_wstrb),
        .rdata (wb_dat_o)
    );

    // 1-cycle synchronous acknowledge (matching BRAM synchronous read)
    always @(posedge clk) begin
        if (reset) begin
            wb_ack_o <= 1'b0;
        end else if (is_req) begin
            wb_ack_o <= 1'b1;
        end else begin
            wb_ack_o <= 1'b0;
        end
    end

endmodule
