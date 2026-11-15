`timescale 1ns/1ps

module riscv_core_tb;

    // Parameters
    parameter CLK_PERIOD = 20; // 50 MHz (matches Artix-7 50 MHz oscillator)

    // Signals
    reg        clk_in;
    reg        reset;
    reg        uart_rx;
    wire       uart_tx;
    wire       dbg_wfi_active;
    wire [7:0] dbg_led;

    // Instantiate Top Module
    riscv_core_top dut (
        .clk_in           (clk_in),
        .reset            (reset),
        .uart_rx          (uart_rx),
        .uart_tx          (uart_tx),
        .dbg_wfi_active   (dbg_wfi_active),
        .dbg_led          (dbg_led),
        .dbg_led_hi       (),
        .sw_i             (16'hA55A),
        .pb_i             (4'h0),
        .seven_seg_digit_o(),
        .seven_seg_seg_o  (),
        .spi_sck_o        (),
        .spi_cs_n_o       (),
        .spi_mosi_o       (),
        .i2c_scl_o        (),
        .i2c_sda_o        ()
    );

    // Clock Generation
    initial begin
        clk_in = 0;
        forever #(CLK_PERIOD/2) clk_in = ~clk_in;
    end

    // =========================================================================
    // UART Monitor - Frame-timed sequential receiver
    // 115200 baud @ 50 MHz -> 434 cycles/bit -> 434 * 20 ns = 8680 ns per bit
    // =========================================================================
    localparam integer BIT_PERIOD_NS = 8680;

    reg [7:0] uart_rx_byte;
    integer   uart_byte_cnt;
    integer   k;

    initial begin
        uart_rx_byte  = 8'h00;
        uart_byte_cnt = 0;

        wait (!reset);
        #(CLK_PERIOD * 2);

        forever begin
            // Wait for start bit (falling edge of uart_tx)
            @(negedge uart_tx);

            // Skip to middle of bit 0 (1.5 bit periods past start falling edge)
            #(BIT_PERIOD_NS + (BIT_PERIOD_NS / 2));

            // Sample 8 data bits (LSB first)
            for (k = 0; k < 8; k = k + 1) begin
                uart_rx_byte[k] = uart_tx;
                if (k < 7) #(BIT_PERIOD_NS);
            end

            $display("[UART] Byte %0d: dec=%0d hex=0x%02h char='%c' time=%0t ns",
                     uart_byte_cnt, uart_rx_byte, uart_rx_byte, uart_rx_byte, $time);
            uart_byte_cnt = uart_byte_cnt + 1;

            // Wait through the stop bit into idle before waiting for next start bit
            #(BIT_PERIOD_NS * 1.5);
        end
    end

    // =========================================================================
    // WFI Halt Detection
    // When WFI is reached, allow in-flight UART transmission to finish, then exit.
    // =========================================================================
    always @(posedge clk_in) begin
        if (dbg_wfi_active && !reset) begin
            $display("[M2] WFI reached at time=%0t ns. Board LED output = 0x%02h", $time, dbg_led);
            // Allow all trailing UART bytes ('P', 'A', 'S', 'S', '\n') to complete
            #(BIT_PERIOD_NS * 65);
            $display("[M2] Simulation completed successfully.");
            $finish;
        end
    end

    // Trace register writes
    always @(posedge clk_in) begin
        if (!reset && dut.rf_we && dut.rf_waddr != 7) begin
            $display("[WB] Register x%0d <= 0x%08h at time %0t ns", dut.rf_waddr, dut.rf_wdata, $time);
        end
    end

    // =========================================================================
    // Simulation Control & Timeout Guard
    // =========================================================================
    initial begin
        reset   = 1;
        uart_rx = 1;

        $display("=========================================================");
        $display(" ASTRA-X1 Milestone 2: Wishbone + Mock NPU Simulation");
        $display(" Expecting: PASS (NPU ID=0x41535452, Scratchpad RW OK)");
        $display("=========================================================");

        #(CLK_PERIOD * 20);
        reset = 0;
        $display("[TB] Reset released at %0t ns", $time);

        // Generous timeout budget: 60000 cycles (1.2 milliseconds)
        #(CLK_PERIOD * 60000);

        $display("[TB] Timeout: 60000 cycles elapsed — WFI never asserted.");
        $display("[TB] LED state = 0x%02h", dbg_led);
        $finish;
    end

endmodule
