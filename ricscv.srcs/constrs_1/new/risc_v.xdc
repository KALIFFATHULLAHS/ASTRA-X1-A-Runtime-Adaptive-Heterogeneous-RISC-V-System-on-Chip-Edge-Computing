##=============================================================================
## ASTRA-X1 Heterogeneous RISC-V SoC — EDGE Artix-7 100T (xc7a100t-2ftg256c)
## Complete Physical Pin & Timing Constraints (Slide 12 SoC Peripherals)
##=============================================================================

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

# 50 MHz System Clock (Pin N11)
set_property -dict { PACKAGE_PIN N11 IOSTANDARD LVCMOS33 } [get_ports { clk_in }];
create_clock -period 20.000 -name clk_in [get_ports clk_in]

# Reset Button (Center Push Button pb[4] — Pin M14)
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { reset }];

# USB-UART (115200 Baud to PC — Pins C4 / D4)
set_property -dict { PACKAGE_PIN C4  IOSTANDARD LVCMOS33 } [get_ports { uart_tx }];
set_property -dict { PACKAGE_PIN D4  IOSTANDARD LVCMOS33 } [get_ports { uart_rx }];

# 16 Slide Switches (GPIO_SW_PB_IN[15:0])
set_property -dict { PACKAGE_PIN L5  IOSTANDARD LVCMOS33 } [get_ports { sw[0] }];
set_property -dict { PACKAGE_PIN L4  IOSTANDARD LVCMOS33 } [get_ports { sw[1] }];
set_property -dict { PACKAGE_PIN M4  IOSTANDARD LVCMOS33 } [get_ports { sw[2] }];
set_property -dict { PACKAGE_PIN M2  IOSTANDARD LVCMOS33 } [get_ports { sw[3] }];
set_property -dict { PACKAGE_PIN M1  IOSTANDARD LVCMOS33 } [get_ports { sw[4] }];
set_property -dict { PACKAGE_PIN N3  IOSTANDARD LVCMOS33 } [get_ports { sw[5] }];
set_property -dict { PACKAGE_PIN N2  IOSTANDARD LVCMOS33 } [get_ports { sw[6] }];
set_property -dict { PACKAGE_PIN N1  IOSTANDARD LVCMOS33 } [get_ports { sw[7] }];
set_property -dict { PACKAGE_PIN P1  IOSTANDARD LVCMOS33 } [get_ports { sw[8] }];
set_property -dict { PACKAGE_PIN P4  IOSTANDARD LVCMOS33 } [get_ports { sw[9] }];
set_property -dict { PACKAGE_PIN T8  IOSTANDARD LVCMOS33 } [get_ports { sw[10] }];
set_property -dict { PACKAGE_PIN R8  IOSTANDARD LVCMOS33 } [get_ports { sw[11] }];
set_property -dict { PACKAGE_PIN N6  IOSTANDARD LVCMOS33 } [get_ports { sw[12] }];
set_property -dict { PACKAGE_PIN T7  IOSTANDARD LVCMOS33 } [get_ports { sw[13] }];
set_property -dict { PACKAGE_PIN P8  IOSTANDARD LVCMOS33 } [get_ports { sw[14] }];
set_property -dict { PACKAGE_PIN M6  IOSTANDARD LVCMOS33 } [get_ports { sw[15] }];

# 16 Onboard LEDs (led[7:0] = Status/Pass 0x5A, led[15:8] = Upper GPIO / WFI)
set_property -dict { PACKAGE_PIN J3  IOSTANDARD LVCMOS33 } [get_ports { led[0] }];
set_property -dict { PACKAGE_PIN H3  IOSTANDARD LVCMOS33 } [get_ports { led[1] }];
set_property -dict { PACKAGE_PIN J1  IOSTANDARD LVCMOS33 } [get_ports { led[2] }];
set_property -dict { PACKAGE_PIN K1  IOSTANDARD LVCMOS33 } [get_ports { led[3] }];
set_property -dict { PACKAGE_PIN L3  IOSTANDARD LVCMOS33 } [get_ports { led[4] }];
set_property -dict { PACKAGE_PIN L2  IOSTANDARD LVCMOS33 } [get_ports { led[5] }];
set_property -dict { PACKAGE_PIN K3  IOSTANDARD LVCMOS33 } [get_ports { led[6] }];
set_property -dict { PACKAGE_PIN K2  IOSTANDARD LVCMOS33 } [get_ports { led[7] }];
set_property -dict { PACKAGE_PIN K5  IOSTANDARD LVCMOS33 } [get_ports { led[8] }];
set_property -dict { PACKAGE_PIN P6  IOSTANDARD LVCMOS33 } [get_ports { led[9] }];
set_property -dict { PACKAGE_PIN R7  IOSTANDARD LVCMOS33 } [get_ports { led[10] }];
set_property -dict { PACKAGE_PIN R6  IOSTANDARD LVCMOS33 } [get_ports { led[11] }];
set_property -dict { PACKAGE_PIN T5  IOSTANDARD LVCMOS33 } [get_ports { led[12] }];
set_property -dict { PACKAGE_PIN R5  IOSTANDARD LVCMOS33 } [get_ports { led[13] }];
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports { led[14] }];
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33 } [get_ports { led[15] }];

# 4 Directional Push Buttons (GPIO_SW_PB_IN[19:16])
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { pb[0] }];
set_property -dict { PACKAGE_PIN L14 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { pb[1] }];
set_property -dict { PACKAGE_PIN M12 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { pb[2] }];
set_property -dict { PACKAGE_PIN L13 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { pb[3] }];

# 4-Digit 7-Segment Display (Multiplexed Hex Output @ 0xFFFF_0018)
set_property -dict { PACKAGE_PIN F2  IOSTANDARD LVCMOS33 } [get_ports { digit[0] }];
set_property -dict { PACKAGE_PIN E1  IOSTANDARD LVCMOS33 } [get_ports { digit[1] }];
set_property -dict { PACKAGE_PIN G5  IOSTANDARD LVCMOS33 } [get_ports { digit[2] }];
set_property -dict { PACKAGE_PIN G4  IOSTANDARD LVCMOS33 } [get_ports { digit[3] }];

set_property -dict { PACKAGE_PIN G2  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[0] }];
set_property -dict { PACKAGE_PIN G1  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[1] }];
set_property -dict { PACKAGE_PIN H5  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[2] }];
set_property -dict { PACKAGE_PIN H4  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[3] }];
set_property -dict { PACKAGE_PIN J5  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[4] }];
set_property -dict { PACKAGE_PIN J4  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[5] }];
set_property -dict { PACKAGE_PIN H2  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[6] }];
set_property -dict { PACKAGE_PIN H1  IOSTANDARD LVCMOS33 } [get_ports { Seven_Seg[7] }];

# Onboard SPI DAC (MCP4921 — Controlled via 0xFFFF_0020 / 0xFFFF_0024)
set_property -dict { PACKAGE_PIN E5  IOSTANDARD LVCMOS33 } [get_ports { SCK }];
set_property -dict { PACKAGE_PIN F5  IOSTANDARD LVCMOS33 } [get_ports { CS }];
set_property -dict { PACKAGE_PIN F4  IOSTANDARD LVCMOS33 } [get_ports { MOSI }];

# Onboard I2C Master Bus (Controlled via 0xFFFF_0030 .. 0xFFFF_0038)
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { i2c_scl }];
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports { i2c_sda }];
