`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: Mock NPU Register Slave
// Wishbone B4 Classic Slave Interface
// Base Address: 0x4000_0000
//
// Register Map:
//   Offset 0x00 (0x4000_0000) [RO]: NPU_REG_ID      = 32'h4153_5452 ("ASTR")
//   Offset 0x04 (0x4000_0004) [RO]: NPU_REG_STATUS  = 32'h0000_0003 (READY | IDLE)
//   Offset 0x08 (0x4000_0008) [RW]: NPU_REG_SCRATCH = Read/Write 32-bit test reg
//=============================================================================

module wb_mock_npu #(
    parameter [31:0] NPU_ID_VAL = 32'h4153_5452 // ASCII "ASTR"
)(
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
    output reg         wb_ack_o
);

    // 32-bit Scratchpad register for bus write/readback testing
    reg [31:0] scratch_reg;

    wire is_valid_req = wb_cyc_i && wb_stb_i && !wb_ack_o;

    always @(posedge clk) begin
        if (reset) begin
            wb_ack_o    <= 1'b0;
            wb_dat_o    <= 32'h0000_0000;
            scratch_reg <= 32'h0000_0000;
        end else if (is_valid_req) begin
            wb_ack_o <= 1'b1; // Synchronous 1-cycle acknowledge

            if (wb_we_i) begin
                // Write transaction
                case (wb_adr_i[5:2])
                    4'h2: begin // Offset 0x08: SCRATCH REGISTER
                        if (wb_sel_i[0]) scratch_reg[7:0]   <= wb_dat_i[7:0];
                        if (wb_sel_i[1]) scratch_reg[15:8]  <= wb_dat_i[15:8];
                        if (wb_sel_i[2]) scratch_reg[23:16] <= wb_dat_i[23:16];
                        if (wb_sel_i[3]) scratch_reg[31:24] <= wb_dat_i[31:24];
                    end
                    default: begin
                        // Read-only registers ignore writes
                    end
                endcase
            end else begin
                // Read transaction
                case (wb_adr_i[5:2])
                    4'h0: wb_dat_o <= NPU_ID_VAL;          // Offset 0x00: NPU ID ("ASTR")
                    4'h1: wb_dat_o <= 32'h0000_0003;       // Offset 0x04: STATUS (Bit 0: READY, Bit 1: IDLE)
                    4'h2: wb_dat_o <= scratch_reg;         // Offset 0x08: SCRATCH REGISTER
                    default: wb_dat_o <= 32'h0000_0000;
                endcase
            end
        end else begin
            wb_ack_o <= 1'b0;
        end
    end

endmodule
