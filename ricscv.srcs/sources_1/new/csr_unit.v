`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: Machine-Mode & Floating-Point Control/Status Register (CSR) Unit
// Matches Slide 12 ("Integer RF + FP RF / fcsr" & "Interrupts / timers (IRQ to core)")
// Implements:
//   - Standard RISC-V Floating-Point CSRs:
//       0x001 fflags, 0x002 frm, 0x003 fcsr
//   - Standard RISC-V Machine Information CSRs:
//       0xF11 mvendorid = 32'h4153_5452 ("ASTR")
//       0xF12 marchid   = 32'h0000_0001 (ASTRA-X1 v1)
//       0xF14 mhartid   = 32'h0000_0000 (Hart 0)
//   - Standard RISC-V Trap & Interrupt CSRs:
//       0x300 mstatus, 0x301 misa (RV32IMFX = 0x40001125), 0x304 mie, 0x305 mtvec,
//       0x340 mscratch, 0x341 mepc, 0x342 mcause, 0x344 mip (live timer & ext IRQs)
//   - Standard Hardware Performance Counters:
//       0xC00/0xC80 mcycle / mcycleh
//       0xC02/0xC82 minstret / minstreth
//   - Custom ASTRA-X1 Power & Approximate Arithmetic CSRs:
//       0x800 mapprox, 0x801 mpower
//=============================================================================

module csr_unit (
    input  wire        clk,
    input  wire        reset,

    // CSR access interface (from EX2)
    input  wire        csr_op,           // Enable CSR operation
    input  wire [11:0] csr_addr,
    input  wire [2:0]  csr_funct3,       // RW, RS, RC
    input  wire [31:0] csr_wdata,
    output reg  [31:0] csr_rdata,

    // Performance & Interrupt inputs
    input  wire        inst_retired,     // 1 if an instruction finished WB this cycle
    input  wire        timer_irq_i,      // From 64-bit mtime >= mtimecmp
    input  wire        ext_irq_i,        // From SoC Interrupt Controller (NPU/DMA/UART)

    // Special control outputs
    output reg         approx_mode,
    output reg [7:0]   power_mode
);

    wire _unused = csr_funct3[2];

    // Standard Performance Counters
    reg [63:0] mcycle;
    reg [63:0] minstret;

    // Floating-Point CSRs (fcsr = {24'd0, frm[2:0], fflags[4:0]})
    reg [4:0]  fflags;
    reg [2:0]  frm;

    // Standard Machine-Mode Trap / Interrupt CSRs
    reg [31:0] mstatus;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mscratch;
    reg [31:0] mepc;
    reg [31:0] mcause;

    // Custom CSRs
    reg [31:0] mapprox;
    reg [31:0] mpower;

    wire [31:0] mip_val = {20'd0, ext_irq_i, 3'd0, timer_irq_i, 7'd0};

    // Cycle counter
    always @(posedge clk) begin
        if (reset) mcycle <= 64'd0;
        else       mcycle <= mcycle + 64'd1;
    end

    // Instruction counter
    always @(posedge clk) begin
        if (reset) minstret <= 64'd0;
        else if (inst_retired) minstret <= minstret + 64'd1;
    end

    // CSR Read
    always @(*) begin
        case (csr_addr)
            12'h001: csr_rdata = {27'd0, fflags};
            12'h002: csr_rdata = {29'd0, frm};
            12'h003: csr_rdata = {24'd0, frm, fflags};
            12'h300: csr_rdata = mstatus;
            12'h301: csr_rdata = 32'h4000_1125; // RV32IMFX (I + M + F + X custom extensions)
            12'h304: csr_rdata = mie;
            12'h305: csr_rdata = mtvec;
            12'h340: csr_rdata = mscratch;
            12'h341: csr_rdata = mepc;
            12'h342: csr_rdata = mcause;
            12'h344: csr_rdata = mip_val;
            12'h800: csr_rdata = mapprox;
            12'h801: csr_rdata = mpower;
            12'hC00: csr_rdata = mcycle[31:0];
            12'hC80: csr_rdata = mcycle[63:32];
            12'hC02: csr_rdata = minstret[31:0];
            12'hC82: csr_rdata = minstret[63:32];
            12'hF11: csr_rdata = 32'h4153_5452; // mvendorid = "ASTR"
            12'hF12: csr_rdata = 32'h0000_0001; // marchid   = ASTRA-X1 v1
            12'hF14: csr_rdata = 32'h0000_0000; // mhartid   = 0
            default: csr_rdata = 32'd0;
        endcase
    end

    // CSR Write & Hardware Interrupt Cause Auto-Update
    always @(posedge clk) begin
        if (reset) begin
            fflags      <= 5'd0;
            frm         <= 3'b000;        // RNE (Round to Nearest, ties to Even)
            mstatus     <= 32'h0000_1800; // MPP = M-mode
            mie         <= 32'd0;
            mtvec       <= 32'd0;
            mscratch    <= 32'd0;
            mepc        <= 32'd0;
            mcause      <= 32'd0;
            mapprox     <= 32'd0;
            mpower      <= 32'hFF;        // All stages enabled by default
            approx_mode <= 1'b0;
            power_mode  <= 8'hFF;
        end else begin
            // Hardware Interrupt Cause Recording when enabled in mie
            if (ext_irq_i && mie[11]) begin
                mcause <= 32'h8000_000B; // Machine External Interrupt (NPU/DMA/UART)
            end else if (timer_irq_i && mie[7]) begin
                mcause <= 32'h8000_0007; // Machine Timer Interrupt (mtime >= mtimecmp)
            end

            if (csr_op) begin
                case (csr_funct3[1:0])
                    2'b01: begin // CSRRW
                        case (csr_addr)
                            12'h001: fflags   <= csr_wdata[4:0];
                            12'h002: frm      <= csr_wdata[2:0];
                            12'h003: begin frm <= csr_wdata[7:5]; fflags <= csr_wdata[4:0]; end
                            12'h300: mstatus  <= csr_wdata;
                            12'h304: mie      <= csr_wdata;
                            12'h305: mtvec    <= csr_wdata;
                            12'h340: mscratch <= csr_wdata;
                            12'h341: mepc     <= csr_wdata;
                            12'h342: mcause   <= csr_wdata;
                            12'h800: mapprox  <= csr_wdata;
                            12'h801: mpower   <= csr_wdata;
                            default: ;
                        endcase
                    end
                    2'b10: begin // CSRRS
                        case (csr_addr)
                            12'h001: fflags   <= fflags   | csr_wdata[4:0];
                            12'h002: frm      <= frm      | csr_wdata[2:0];
                            12'h003: begin frm <= frm | csr_wdata[7:5]; fflags <= fflags | csr_wdata[4:0]; end
                            12'h300: mstatus  <= mstatus  | csr_wdata;
                            12'h304: mie      <= mie      | csr_wdata;
                            12'h305: mtvec    <= mtvec    | csr_wdata;
                            12'h340: mscratch <= mscratch | csr_wdata;
                            12'h341: mepc     <= mepc     | csr_wdata;
                            12'h342: mcause   <= mcause   | csr_wdata;
                            12'h800: mapprox  <= mapprox  | csr_wdata;
                            12'h801: mpower   <= mpower   | csr_wdata;
                            default: ;
                        endcase
                    end
                    2'b11: begin // CSRRC
                        case (csr_addr)
                            12'h001: fflags   <= fflags   & ~csr_wdata[4:0];
                            12'h002: frm      <= frm      & ~csr_wdata[2:0];
                            12'h003: begin frm <= frm & ~csr_wdata[7:5]; fflags <= fflags & ~csr_wdata[4:0]; end
                            12'h300: mstatus  <= mstatus  & ~csr_wdata;
                            12'h304: mie      <= mie      & ~csr_wdata;
                            12'h305: mtvec    <= mtvec    & ~csr_wdata;
                            12'h340: mscratch <= mscratch & ~csr_wdata;
                            12'h341: mepc     <= mepc     & ~csr_wdata;
                            12'h342: mcause   <= mcause   & ~csr_wdata;
                            12'h800: mapprox  <= mapprox  & ~csr_wdata;
                            12'h801: mpower   <= mpower   & ~csr_wdata;
                            default: ;
                        endcase
                    end
                    default: ;
                endcase

                approx_mode <= mapprox[0];
                power_mode  <= mpower[7:0];
            end
        end
    end

endmodule
