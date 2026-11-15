`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 Milestone 3: 4x4 INT8 Systolic Array Verification Testbench
// Validates 16-PE matrix acceleration against NumPy golden model.
//=============================================================================

module tb_npu_systolic;

    localparam CLK_PERIOD = 20; // 50 MHz (20 ns)

    reg        clk;
    reg        reset;

    // Array Controls
    reg        start;
    reg [15:0] k_dim;
    wire       busy;
    wire       done;

    // Parallel Stream Inputs
    reg        data_valid;
    reg signed [7:0] in_a0, in_a1, in_a2, in_a3;
    reg signed [7:0] in_b0, in_b1, in_b2, in_b3;

    // Readout Interface
    reg  [1:0]  read_row;
    reg  [1:0]  read_col;
    wire signed [31:0] read_data;
    wire [511:0] flat_c_matrix;

    // Error tracking
    integer errors;

    // Instantiate 4x4 Systolic Array Under Test
    npu_systolic_array #(
        .ROWS(4),
        .COLS(4)
    ) dut (
        .clk           (clk),
        .reset         (reset),
        .start         (start),
        .k_dim         (k_dim),
        .busy          (busy),
        .done          (done),

        .data_valid    (data_valid),
        .in_a0         (in_a0),
        .in_a1         (in_a1),
        .in_a2         (in_a2),
        .in_a3         (in_a3),

        .in_b0         (in_b0),
        .in_b1         (in_b1),
        .in_b2         (in_b2),
        .in_b3         (in_b3),

        .read_row      (read_row),
        .read_col      (read_col),
        .read_data     (read_data),
        .flat_c_matrix (flat_c_matrix)
    );

    // Clock Generator: 50 MHz
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Include auto-generated Python golden test tasks
    `include "npu_test_cases.vh"

    // Simulation Main
    initial begin
        errors     = 0;
        reset      = 1;
        start      = 0;
        k_dim      = 0;
        data_valid = 0;
        in_a0 = 0; in_a1 = 0; in_a2 = 0; in_a3 = 0;
        in_b0 = 0; in_b1 = 0; in_b2 = 0; in_b3 = 0;
        read_row   = 0;
        read_col   = 0;

        $display("=========================================================");
        $display(" ASTRA-X1 MILESTONE 3: 4x4 INT8 SYSTOLIC ARRAY TESTBENCH");
        $display(" Target Clock: 50 MHz | 16 MACs (32 Integer Ops)/Cycle");
        $display("=========================================================");

        #(CLK_PERIOD * 10);
        reset = 0;
        #(CLK_PERIOD * 5);

        // Run all generated matrix test tasks
        run_test_0;
        #(CLK_PERIOD * 5);

        run_test_1;
        #(CLK_PERIOD * 5);

        run_test_2;
        #(CLK_PERIOD * 5);

        run_test_3;
        #(CLK_PERIOD * 5);

        run_test_4;
        #(CLK_PERIOD * 5);

        run_test_5;
        #(CLK_PERIOD * 5);

        run_test_6;
        #(CLK_PERIOD * 5);

        run_test_7;
        #(CLK_PERIOD * 10);

        // Summary
        $display("\n=========================================================");
        $display(" ASTRA-X1 MILESTONE 3: VERIFICATION SUMMARY");
        $display("=========================================================");
        if (errors == 0) begin
            $display(">>> [SUCCESS] ALL NPU MATRIX TESTS PASSED (100%% BIT-EXACT MATCH)!");
            $display(">>> 16-PE INT8 Output-Stationary Systolic Array is Fully Operational.");
        end else begin
            $display(">>> [FAILURE] Encountered %0d bit-level mismatches.", errors);
        end
        $display("=========================================================\n");

        $finish;
    end

endmodule
