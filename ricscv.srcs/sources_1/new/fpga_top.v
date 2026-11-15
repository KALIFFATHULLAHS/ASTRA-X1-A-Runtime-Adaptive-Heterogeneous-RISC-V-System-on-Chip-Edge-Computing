`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 Heterogeneous SoC: EDGE Artix-7 100T Physical FPGA Top-Level
// Connects:
//   - 50 MHz System Oscillator (Pin N11) & Reset Button (Pin M14)
//   - USB-UART Bootloader & Runtime Console (Pins D4 / C4 @ 115200 baud)
//   - 16 Onboard LEDs (led[7:0] = Status/Pass 0x5A, led[15:8] = Upper GPIO)
//   - 16 Slide Switches (sw[15:0]) & 4 Directional Push Buttons (pb[3:0])
//   - 4-Digit Multiplexed 7-Segment Display (digit[3:0], Seven_Seg[7:0])
//   - Onboard MCP4921 12-bit SPI DAC (SCK=E5, CS=F5, MOSI=F4)
//   - Onboard I2C Master Bus (i2c_scl=M16, i2c_sda=N16)
//=============================================================================

module fpga_top (
    input  wire        clk_in,
    input  wire        reset,       // Center button M14 (press = reset=1)
    input  wire        uart_rx,     // USB-UART RX (D4)
    output wire        uart_tx,     // USB-UART TX (C4)
    output wire [15:0] led,         // 16 Onboard LEDs
    input  wire [15:0] sw,          // 16 Slide Switches
    input  wire [3:0]  pb,          // 4 Directional Push Buttons
    output wire [3:0]  digit,       // 4-Digit 7-Segment Anodes
    output wire [7:0]  Seven_Seg,   // 7-Segment Cathodes (A..G, DP)
    output wire        SCK,         // MCP4921 SPI DAC SCK (E5)
    output wire        CS,          // MCP4921 SPI DAC CS_N (F5)
    output wire        MOSI,        // MCP4921 SPI DAC MOSI (F4)
    output wire        i2c_scl,     // I2C SCL (M16)
    output wire        i2c_sda      // I2C SDA (N16)
);

    wire       dbg_wfi_active;
    wire [7:0] dbg_led;
    wire [7:0] dbg_led_hi;

    riscv_core_top core (
        .clk_in           (clk_in),
        .reset            (reset),
        .uart_rx          (uart_rx),
        .uart_tx          (uart_tx),
        .dbg_wfi_active   (dbg_wfi_active),
        .dbg_led          (dbg_led),
        .dbg_led_hi       (dbg_led_hi),
        .sw_i             (sw),
        .pb_i             (pb),
        .seven_seg_digit_o(digit),
        .seven_seg_seg_o  (Seven_Seg),
        .spi_sck_o        (SCK),
        .spi_cs_n_o       (CS),
        .spi_mosi_o       (MOSI),
        .i2c_scl_o        (i2c_scl),
        .i2c_sda_o        (i2c_sda)
    );

    assign led[7:0]  = dbg_led;
    assign led[15:8] = (dbg_led_hi != 8'd0) ? dbg_led_hi : {7'd0, dbg_wfi_active};

endmodule
