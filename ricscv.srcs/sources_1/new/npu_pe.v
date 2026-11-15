`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 Edge-AI Accelerator: Processing Element (PE)
// Architecture: Output-Stationary (OS) MAC Engine
// Precision:
//   - Activation Input: Signed 8-bit (INT8) [-128 .. +127]
//   - Weight Input:     Signed 8-bit (INT8) [-128 .. +127]
//   - Product:          Signed 16-bit (INT16)
//   - Accumulator:      Signed 32-bit (INT32) with synchronous clear
// Forwarding:
//   - Activation Out:   Registered 1-cycle horizontal pass to East neighbor
//   - Weight Out:       Registered 1-cycle vertical pass to South neighbor
//=============================================================================

module npu_pe (
    input  wire        clk,
    input  wire        reset,
    input  wire        en,          // Global compute enable
    input  wire        clr_acc,     // Clear accumulator to 0 (start of new GEMM)

    // Systolic Inputs & Valids
    input  wire signed [7:0] in_a,  // Activation stream (from West)
    input  wire              val_a, // Activation valid flag

    input  wire signed [7:0] in_b,  // Weight stream (from North)
    input  wire              val_b, // Weight valid flag

    // Systolic Forwarding Outputs (Registered 1-cycle delay)
    output reg  signed [7:0] out_a, // Forward to East neighbor
    output reg               out_val_a,

    output reg  signed [7:0] out_b, // Forward to South neighbor
    output reg               out_val_b,

    // Stationary Accumulated Result
    output reg  signed [31:0] acc_out
);

    // Multiplier: Signed 8x8 -> Signed 16-bit
    wire signed [15:0] mult_product = in_a * in_b;
    wire compute_active = val_a && val_b;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            out_a      <= 8'sd0;
            out_val_a  <= 1'b0;
            out_b      <= 8'sd0;
            out_val_b  <= 1'b0;
            acc_out    <= 32'sd0;
        end else if (en) begin
            // 1. Forward horizontal activation and valid
            out_a      <= in_a;
            out_val_a  <= val_a;

            // 2. Forward vertical weight and valid
            out_b      <= in_b;
            out_val_b  <= val_b;

            // 3. Output-Stationary Accumulation
            if (clr_acc) begin
                if (compute_active) begin
                    acc_out <= {{16{mult_product[15]}}, mult_product};
                end else begin
                    acc_out <= 32'sd0;
                end
            end else if (compute_active) begin
                acc_out <= acc_out + {{16{mult_product[15]}}, mult_product};
            end
        end
    end

endmodule
