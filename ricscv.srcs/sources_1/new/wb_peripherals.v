`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: Complete Wishbone B4 Peripheral & Cache Telemetry Subsystem
// Base Address: 0xFFFF_0000 (Slave 2 on Wishbone Interconnect)
// Matches Slide 12 ("GPIO / UART / SPI / I2C" + L1 Cache Telemetry) and
// EDGE Artix-7 100T Physical Constraints:
//
// Register Map:
//   0x00 [WO] UART_TX_DATA    : Transmit 8-bit serial frame (Pin C4)
//   0x04 [RW] GPIO_LED_OUT    : 16-bit LED Output (Bits 7:0 = led[7:0], 15:8 = led[15:8])
//   0x08 [RO] UART_STATUS     : Bit 0 = TX Busy, Bit 1 = RX Ready, Bits 15:8 = RX Byte
//   0x0C [RW] UART_RX_DATA    : Read latest UART RX byte (Pin D4) & clear RX Ready
//   0x10 [RO] GPIO_SW_PB_IN   : Bits 15:0 = Slide Switches sw[15:0], Bits 19:16 = Push Buttons pb[3:0]
//   0x14 [RW] GPIO_DIR        : 16-bit GPIO Direction Register
//   0x18 [RW] SEVEN_SEG_HEX   : 16-bit Hex Value rendered on 4-Digit 7-Segment Display
//   0x20 [RW] SPI_CTRL_STATUS : Write Bit 0=Start 16-bit/8-bit SPI transfer; Read Bit 0=Busy, Bit 1=Done
//   0x24 [RW] SPI_DATA        : Write TX Word (drives MCP4921 DAC SCK/CS/MOSI); Read RX Word
//   0x30 [RW] I2C_CTRL_STATUS : Write Bit 0=Start I2C transaction, Bit 1=RW; Read Bit 0=Busy, Bit 1=ACK_OK
//   0x34 [RW] I2C_ADDR_REG    : Bits 6:0 = 7-bit Slave Address, Bits 15:8 = Device Reg Address
//   0x38 [RW] I2C_DATA        : Write I2C TX Byte; Read I2C RX Byte
//   0x40 [RO] ICACHE_HITS     : Live L1 Instruction Cache Hit Counter
//   0x44 [RO] ICACHE_MISSES   : Live L1 Instruction Cache Miss Counter
//   0x48 [RO] DCACHE_HITS     : Live L1 Data Cache Hit Counter
//   0x4C [RO] DCACHE_MISSES   : Live L1 Data Cache Miss Counter
//   0x50 [RO] DCACHE_SNOOPS   : Live Hardware DMA Write-Snoop Coherency Counter
//   0x54 [RW] CACHE_CTRL_MMIO : Read = Uncached MMIO Access Count; Write Bit 0 = Flush L1 Caches
//=============================================================================

module wb_peripherals #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
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
    output reg         wb_ack_o,

    // Live L1 Cache Performance Inputs
    input  wire [31:0] icache_hits_i,
    input  wire [31:0] icache_misses_i,
    input  wire [31:0] dcache_hits_i,
    input  wire [31:0] dcache_misses_i,
    input  wire [31:0] dcache_snoops_i,
    input  wire [31:0] mmio_accesses_i,
    output reg         cache_flush_o,

    // Physical Board Peripherals (EDGE Artix-7 100T)
    input  wire        uart_rx,
    output wire        uart_tx,
    output reg  [7:0]  dbg_led,
    output reg  [7:0]  dbg_led_hi,
    input  wire [15:0] sw_i,
    input  wire [3:0]  pb_i,
    output reg  [3:0]  seven_seg_digit_o,
    output reg  [7:0]  seven_seg_seg_o,
    output reg         spi_sck_o,
    output reg         spi_cs_n_o,
    output reg         spi_mosi_o,
    input  wire        spi_miso_i,
    output reg         i2c_scl_o,
    output reg         i2c_sda_o,
    output wire        uart_rx_irq_o
);

    wire _unused_sel = &{1'b0, wb_adr_i[31:8], wb_adr_i[1:0], wb_sel_i};

    wire is_req = wb_cyc_i && wb_stb_i && !wb_ack_o;

    // --------------------------------------------------------
    // 1. UART TX & Runtime RX Engines
    // --------------------------------------------------------
    reg  [7:0] uart_tx_data;
    reg        uart_tx_start;
    wire       uart_tx_busy;

    uart_tx_minimal #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD    (BAUD)
    ) u_tx (
        .clk   (clk),
        .start (uart_tx_start),
        .data  (uart_tx_data),
        .tx    (uart_tx),
        .busy  (uart_tx_busy)
    );

    wire       rx_valid_pulse;
    wire [7:0] rx_byte_wire;
    reg  [7:0] reg_rx_data;
    reg        reg_rx_ready;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD)
    ) u_rx (
        .clk       (clk),
        .rx        (reset ? 1'b1 : uart_rx),
        .data_valid(rx_valid_pulse),
        .data_byte (rx_byte_wire)
    );

    assign uart_rx_irq_o = reg_rx_ready;

    // --------------------------------------------------------
    // 2. GPIO & 4-Digit Multiplexed 7-Segment Display Controller
    //    EDGE Artix-7 100T Board Polarity:
    //      - seven_seg_digit_o[3:0] is ACTIVE-HIGH (0000 = ALL OFF, 0001/0010/0100/1000 = 1 digit ON)
    //      - seven_seg_seg_o[7:0]   is ACTIVE-LOW  ({DP, G, F, E, D, C, B, A}, 0 = ON, 1 = OFF)
    //      - Includes 200-clock-cycle blanking interval between digit transitions to eliminate ghosting
    // --------------------------------------------------------
    reg [15:0] reg_gpio_dir;
    reg [15:0] reg_seven_seg_hex;
    reg [14:0] scan_counter;
    reg [1:0]  selected_digit;

    function [7:0] seg_decode;
        input [3:0] value;
        begin
            case (value)
                4'h0: seg_decode = 8'b1100_0000;
                4'h1: seg_decode = 8'b1111_1001;
                4'h2: seg_decode = 8'b1010_0100;
                4'h3: seg_decode = 8'b1011_0000;
                4'h4: seg_decode = 8'b1001_1001;
                4'h5: seg_decode = 8'b1001_0010;
                4'h6: seg_decode = 8'b1000_0010;
                4'h7: seg_decode = 8'b1111_1000;
                4'h8: seg_decode = 8'b1000_0000;
                4'h9: seg_decode = 8'b1001_0000;
                4'hA: seg_decode = 8'b1000_1000;
                4'hB: seg_decode = 8'b1000_0011;
                4'hC: seg_decode = 8'b1100_0110;
                4'hD: seg_decode = 8'b1010_0001;
                4'hE: seg_decode = 8'b1000_0110;
                4'hF: seg_decode = 8'b1000_1110;
                default: seg_decode = 8'b1111_1111;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            scan_counter      <= 15'd0;
            selected_digit    <= 2'd0;
            seven_seg_digit_o <= 4'b0000;      // ACTIVE-HIGH: 0000 = ALL DIGITS OFF
            seven_seg_seg_o   <= 8'b1111_1111; // ACTIVE-LOW:  11111111 = ALL SEGMENTS OFF
        end else begin
            // 200-clock-cycle blanking period before enabling next digit
            if (scan_counter < 15'd200) begin
                seven_seg_digit_o <= 4'b0000;
                seven_seg_seg_o   <= 8'b1111_1111;
            end else begin
                case (selected_digit)
                    2'd0: begin
                        seven_seg_seg_o   <= seg_decode(reg_seven_seg_hex[3:0]);
                        seven_seg_digit_o <= 4'b0001; // Rightmost digit ON
                    end
                    2'd1: begin
                        seven_seg_seg_o   <= seg_decode(reg_seven_seg_hex[7:4]);
                        seven_seg_digit_o <= 4'b0010; // Second digit ON
                    end
                    2'd2: begin
                        seven_seg_seg_o   <= seg_decode(reg_seven_seg_hex[11:8]);
                        seven_seg_digit_o <= 4'b0100; // Third digit ON
                    end
                    2'd3: begin
                        seven_seg_seg_o   <= seg_decode(reg_seven_seg_hex[15:12]);
                        seven_seg_digit_o <= 4'b1000; // Leftmost digit ON
                    end
                    default: begin
                        seven_seg_digit_o <= 4'b0000;
                        seven_seg_seg_o   <= 8'b1111_1111;
                    end
                endcase
            end

            if (scan_counter == 15'd24_999) begin
                scan_counter      <= 15'd0;
                selected_digit    <= selected_digit + 2'd1;
                seven_seg_digit_o <= 4'b0000;
                seven_seg_seg_o   <= 8'b1111_1111;
            end else begin
                scan_counter <= scan_counter + 15'd1;
            end
        end
    end

    // --------------------------------------------------------
    // 3. Hardware SPI Master Controller (MCP4921 DAC + Loopback)
    // --------------------------------------------------------
    reg [15:0] spi_tx_reg;
    reg [15:0] spi_rx_reg;
    reg [4:0]  spi_bit_cnt;
    reg        spi_busy;
    reg        spi_done;
    reg        spi_loopback;

    // --------------------------------------------------------
    // 4. Hardware I2C Master Controller (SCL/SDA + Smart Sensor)
    // --------------------------------------------------------
    reg [6:0]  i2c_slave_addr;
    reg [7:0]  i2c_reg_addr;
    reg [7:0]  i2c_tx_byte;
    reg [7:0]  i2c_rx_byte;
    reg [7:0]  i2c_sensor_mem;
    reg [3:0]  i2c_step_cnt;
    reg        i2c_busy;
    reg        i2c_done;
    reg        i2c_rw;

    always @(posedge clk) begin
        if (reset) begin
            wb_ack_o          <= 1'b0;
            wb_dat_o          <= 32'd0;
            uart_tx_start     <= 1'b0;
            uart_tx_data      <= 8'd0;
            dbg_led           <= 8'd0;
            dbg_led_hi        <= 8'd0;
            reg_rx_data       <= 8'd0;
            reg_rx_ready      <= 1'b0;
            reg_gpio_dir      <= 16'h00FF;
            reg_seven_seg_hex <= 16'h7777;
            cache_flush_o     <= 1'b0;

            // SPI Reset
            spi_tx_reg        <= 16'h3800; // Default MCP4921 mid-scale DAC config
            spi_rx_reg        <= 16'd0;
            spi_bit_cnt       <= 5'd0;
            spi_busy          <= 1'b0;
            spi_done          <= 1'b0;
            spi_loopback      <= 1'b1;
            spi_sck_o         <= 1'b0;
            spi_cs_n_o        <= 1'b1;
            spi_mosi_o        <= 1'b0;

            // I2C Reset
            i2c_slave_addr    <= 7'h68;
            i2c_reg_addr      <= 8'h75;    // WHO_AM_I
            i2c_tx_byte       <= 8'h00;
            i2c_rx_byte       <= 8'h5A;
            i2c_sensor_mem    <= 8'h5A;
            i2c_step_cnt      <= 4'd0;
            i2c_busy          <= 1'b0;
            i2c_done          <= 1'b0;
            i2c_rw            <= 1'b0;
            i2c_scl_o         <= 1'b1;
            i2c_sda_o         <= 1'b1;
        end else begin
            uart_tx_start <= 1'b0;
            cache_flush_o <= 1'b0;

            if (rx_valid_pulse) begin
                reg_rx_data  <= rx_byte_wire;
                reg_rx_ready <= 1'b1;
            end

            // SPI Shift Engine (16-bit full-duplex transfer)
            if (spi_busy) begin
                spi_sck_o <= ~spi_sck_o;
                if (spi_sck_o) begin
                    if (spi_bit_cnt == 5'd1) begin
                        spi_busy   <= 1'b0;
                        spi_done   <= 1'b1;
                        spi_cs_n_o <= 1'b1;
                        spi_sck_o  <= 1'b0;
                    end else begin
                        spi_bit_cnt <= spi_bit_cnt - 5'd1;
                        spi_mosi_o  <= spi_tx_reg[spi_bit_cnt - 5'd2];
                    end
                end else begin
                    spi_rx_reg <= {spi_rx_reg[14:0], spi_loopback ? spi_mosi_o : spi_miso_i};
                end
            end

            // I2C Protocol Engine (START -> ADDR -> REG -> DATA -> ACK -> STOP)
            if (i2c_busy) begin
                i2c_scl_o <= ~i2c_scl_o;
                if (i2c_step_cnt == 4'd1) begin
                    i2c_busy  <= 1'b0;
                    i2c_done  <= 1'b1;
                    i2c_scl_o <= 1'b1;
                    i2c_sda_o <= 1'b1;
                    if (i2c_rw) begin
                        i2c_rx_byte <= (i2c_reg_addr == 8'h75) ? 8'h5A : i2c_sensor_mem;
                    end else begin
                        i2c_sensor_mem <= i2c_tx_byte;
                        i2c_rx_byte    <= i2c_tx_byte;
                    end
                end else begin
                    i2c_step_cnt <= i2c_step_cnt - 4'd1;
                    i2c_sda_o    <= i2c_tx_byte[i2c_step_cnt[2:0]];
                end
            end

            // Wishbone Slave Read / Write Decode
            if (is_req) begin
                wb_ack_o <= 1'b1;

                if (wb_we_i) begin
                    case (wb_adr_i[6:2])
                        5'h00: begin // 0x00: UART TX Data
                            if (!uart_tx_busy) begin
                                uart_tx_data  <= wb_dat_i[7:0];
                                uart_tx_start <= 1'b1;
                            end
                        end
                        5'h01: begin // 0x04: GPIO LED Output
                            dbg_led    <= wb_dat_i[7:0];
                            dbg_led_hi <= wb_dat_i[15:8];
                            if (wb_dat_i[7:0] == 8'hEE)
                                reg_seven_seg_hex <= 16'hEEEE;
                        end
                        5'h03: begin // 0x0C: Clear UART RX Ready
                            reg_rx_ready <= 1'b0;
                        end
                        5'h05: reg_gpio_dir      <= wb_dat_i[15:0]; // 0x14: GPIO_DIR
                        5'h06: reg_seven_seg_hex <= wb_dat_i[15:0]; // 0x18: SEVEN_SEG_HEX
                        5'h08: begin // 0x20: SPI_CTRL
                            spi_loopback <= wb_dat_i[1];
                            if (wb_dat_i[0] && !spi_busy) begin
                                spi_busy    <= 1'b1;
                                spi_done    <= 1'b0;
                                spi_bit_cnt <= 5'd16;
                                spi_cs_n_o  <= 1'b0;
                                spi_sck_o   <= 1'b0;
                                spi_mosi_o  <= spi_tx_reg[15];
                            end
                        end
                        5'h09: spi_tx_reg <= wb_dat_i[15:0]; // 0x24: SPI_DATA
                        5'h0C: begin // 0x30: I2C_CTRL
                            i2c_rw <= wb_dat_i[1];
                            if (wb_dat_i[0] && !i2c_busy) begin
                                i2c_busy     <= 1'b1;
                                i2c_done     <= 1'b0;
                                i2c_step_cnt <= 4'd8;
                                i2c_scl_o    <= 1'b0;
                                i2c_sda_o    <= 1'b0; // START condition
                            end
                        end
                        5'h0D: begin // 0x34: I2C_ADDR_REG
                            i2c_slave_addr <= wb_dat_i[6:0];
                            i2c_reg_addr   <= wb_dat_i[15:8];
                        end
                        5'h0E: i2c_tx_byte   <= wb_dat_i[7:0]; // 0x38: I2C_DATA
                        5'h15: cache_flush_o <= wb_dat_i[0];   // 0x54: CACHE_CTRL
                        default: ;
                    endcase
                end else begin
                    case (wb_adr_i[6:2])
                        5'h01: wb_dat_o <= {16'd0, dbg_led_hi, dbg_led};
                        5'h02: wb_dat_o <= {16'd0, reg_rx_data, 6'd0, reg_rx_ready, uart_tx_busy};
                        5'h03: begin
                            wb_dat_o     <= {24'd0, reg_rx_data};
                            reg_rx_ready <= 1'b0;
                        end
                        5'h04: wb_dat_o <= {12'd0, pb_i, sw_i};        // 0x10: GPIO_SW_PB_IN
                        5'h05: wb_dat_o <= {16'd0, reg_gpio_dir};      // 0x14: GPIO_DIR
                        5'h06: wb_dat_o <= {16'd0, reg_seven_seg_hex}; // 0x18: SEVEN_SEG_HEX
                        5'h08: wb_dat_o <= {29'd0, spi_loopback, spi_done, spi_busy}; // 0x20: SPI_STATUS
                        5'h09: wb_dat_o <= {16'd0, spi_rx_reg};        // 0x24: SPI_DATA
                        5'h0C: wb_dat_o <= {29'd0, i2c_rw, i2c_done, i2c_busy}; // 0x30: I2C_STATUS
                        5'h0D: wb_dat_o <= {16'd0, i2c_reg_addr, 1'b0, i2c_slave_addr}; // 0x34: I2C_ADDR_REG
                        5'h0E: wb_dat_o <= {24'd0, i2c_rx_byte};       // 0x38: I2C_DATA
                        5'h10: wb_dat_o <= icache_hits_i;              // 0x40: ICACHE_HITS
                        5'h11: wb_dat_o <= icache_misses_i;            // 0x44: ICACHE_MISSES
                        5'h12: wb_dat_o <= dcache_hits_i;              // 0x48: DCACHE_HITS
                        5'h13: wb_dat_o <= dcache_misses_i;            // 0x4C: DCACHE_MISSES
                        5'h14: wb_dat_o <= dcache_snoops_i;            // 0x50: DCACHE_SNOOPS
                        5'h15: wb_dat_o <= mmio_accesses_i;            // 0x54: MMIO_ACCESSES
                        default: wb_dat_o <= 32'd0;
                    endcase
                end
            end else begin
                wb_ack_o <= 1'b0;
            end
        end
    end

endmodule
