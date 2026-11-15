# ASTRA-X1: Developer Platform Release, Silicon Validation & Patent Novelty Dossier

**Platform Name:** ASTRA-X1 Heterogeneous RISC-V Edge-Computing System-on-Chip (SoC)  
**Target Silicon:** Xilinx / AMD Artix-7 FPGA (`xc7a100t-2ftg256c` on EDGE Artix-7 100T Development Board)  
**System Clock:** 50 MHz Synchronous System Domain (`clk_in` = Pin `N11`, `WNS > 0 ns`)  
**Interconnect:** 32-Bit Multi-Master × 7-Slave Wishbone B4 Crossbar (`CPU Master 0` + `DMA Master 1`)  

---

## 1. Dual-Mode Boot Architecture: How the Product OS and Milestone Scripts (`m1..m8`) Work Together

A key architectural feature of **ASTRA-X1** ([bram_imem.v](file:///d:/FINAL YEAR PROJECT/ricscv/ricscv.srcs/sources_1/new/bram_imem.v)) is its **Dual-Path Instruction Memory Boot Architecture**, which gives you **both** a standalone power-on product **and** 100% compatibility with every Python verification script (`m1` through `m8`):

1. **Mode A — Standalone Power-On Boot Image (Baked into FPGA Bitstream):**
   - During Vivado synthesis, `bram_imem.v` executes `$readmemh("program.mem", mem);` (confirmed in `synth_1/runme.log`: `INFO: [Synth 8-3876] $readmem data file 'program.mem' is read successfully`).
   - The moment the FPGA configures from JTAG or onboard SPI Flash (or whenever you press and release Reset `M14` without sending UART bytes), the CPU immediately boots the built-in image in Block RAM.
2. **Mode B — Dynamic Hardware UART Bootloader Override (Run Any `m1..m8` `.py` Script Anytime):**
   - While you hold down the physical **Reset Button (`Pin M14`)**, the hardware UART receiver inside `bram_imem.v` listens on `COM3 @ 115200 baud` and writes incoming 32-bit instruction words directly into `mem[0..N]` starting at `0x0000_0000`.
   - When you release Reset (`M14`), the CPU executes the newly uploaded `.py` program immediately!
   - **Result:** You can run `upload_m2_npu_test.py`, `upload_m4_npu_gemm.py`, `upload_m5a_tiled_gemm.py`, `upload_m5b_dma_npu.py`, `upload_m6a_simd_bench.py`, `upload_m6b_fp32_bench.py`, `upload_m7_daaq_power.py`, `upload_m8_periph_cache_irq.py`, or `astra_x1_product_os.py` back-to-back at any time without ever re-synthesizing the bitstream!

---

## 2. Core Research Novelty & Patentable Invention Disclosure

### 2.1 Prior Art vs. ASTRA-X1's Original Contribution
In recent literature (including 2026 IEEE VLSID RISC-V accelerator papers), integrating an in-order RISC-V core with a $4 \times 4$ systolic MAC array and a DMA engine is established system architecture. **ASTRA-X1 does not claim the systolic array or Wishbone bus alone as its invention.**

Instead, ASTRA-X1 introduces **Cross-Domain Dependency-Aware Adaptive Quiescence with Snoop-Coherent DMA Handshaking (`ASTRA-DAAQ`)**, implemented in [wb_astra_daaq.v](file:///d:/FINAL YEAR PROJECT/ricscv/ricscv.srcs/sources_1/new/wb_astra_daaq.v) and [astra_l1_caches.v](file:///d:/FINAL YEAR PROJECT/ricscv/ricscv.srcs/sources_1/new/astra_l1_caches.v).

### 2.2 Three Patentable Hardware Claims

1. **Claim 1 — Producer-Consumer Cross-Engine Dependency Matrix Inhibiting Premature Quiescence:**
   - **Problem Solved:** In heterogeneous pipelines (`DMA -> SIMD -> NPU -> FP32`), conventional local idle-timeout clock gating (**Mode B**) gates downstream accelerators while upstream producers are still preparing or transferring operands, causing severe **cold-wake pipeline hazards (`74` stall cycles measured in silicon)**.
   - **Hardware Mechanism:** `wb_astra_daaq.v` maintains a programmable cross-engine dependency bitmask (`DEP_MASK`) and taps live hardware busy and Wishbone strobe wires (`dma_busy`, `simd_busy`, `npu_busy`, `fpu_busy`, `s1_stb..s6_stb`). A downstream engine's idle counter is hardware-clamped to `ACTIVE/READY` whenever its designated upstream producer is active, achieving **zero cold-wake stall cycles (`WakeStall = 0`)** in silicon.

2. **Claim 2 — Break-Even Energy Evaluator with Phase-Hinted Adaptive Hysteresis:**
   - **Problem Solved:** Fixed idle thresholds (**Mode C**) waste energy during known non-compute phases because they wait for a conservative static timeout ($T_{\text{fixed}} = 16$ cycles) before gating unused engines.
   - **Hardware Mechanism:** `wb_astra_daaq.v` evaluates the break-even energy condition ($E_{\text{benefit}} = \Delta P \cdot T_{\text{idle}} - E_{\text{wake}} > 0$) and dynamically modulates per-engine quiescence thresholds (`T_idle = 2` vs. `12` cycles) using lightweight architectural phase hints (`DAAQ_REG_PHASE`: `DMA_XFER`, `SIMD_PREPROC`, `NPU_GEMM`, `FPU_POST`, `IDLE`) while preserving hardware safety overrides.
   - **Measured Silicon Result:** Increases gated accelerator cycles and achieves **`+12.6%` higher energy savings (`2363` vs. `2098` energy units)** over Fixed-Threshold DAAQ while maintaining **`0` wake stalls**.

3. **Claim 3 — Snoop-Coherent L1 Data Cache & Autonomous DMA Quiescence Handshake:**
   - **Problem Solved:** When an autonomous DMA engine writes accelerator output tiles into system SRAM, traditional cached SoCs require software cache invalidation loops or CPU intervention before the accelerator and DMA domain can safely quiesce.
   - **Hardware Mechanism:** [astra_l1_caches.v](file:///d:/FINAL YEAR PROJECT/ricscv/ricscv.srcs/sources_1/new/astra_l1_caches.v) implements a **Single-Cycle Wishbone Master-1 Write-Snoop Port** that monitors `dma_cyc && dma_stb && dma_we && dma_ack` on the shared SRAM bus. When DMA writes a 32-bit word to cacheable SRAM (`0x0000_0000..0x3FFF_FFFF`), the L1 Data Cache automatically updates/pre-warms the matching cache line in the same clock cycle (`DCACHE_SNOOPS`), allowing immediate zero-miss CPU reads (`DCACHE_HITS`) and immediate producer quiescence.

---

## 3. Complete Silicon Validation Matrix (Measured on EDGE Artix-7 100T @ 50 MHz)

| Milestone | Verification Script | Hardware Subsystem | Measured Silicon Metrics | Speedup / Result |
| :--- | :--- | :--- | :--- | :--- |
| **M1–M3** | [upload_m2_npu_test.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m2_npu_test.py) | 7-Stage RV32IMFX CPU + Wishbone B4 + $4 \times 4$ Systolic Array | `NPU_ID = 0x41535452` (`"ASTR"`), 10,000 NumPy matrices | Bit-exact (`0x5A`) |
| **M4** | [upload_m4_npu_gemm.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m4_npu_gemm.py) | $4 \times 4$ INT8 Systolic NPU GEMM (`0x4000_0000`) | `CPU SW: 3482 cyc` vs. `NPU SoC: 43 cyc` / `MAC: 19 cyc` | **`80x` SoC / `183x` MAC** |
| **M5A** | [upload_m5a_tiled_gemm.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m5a_tiled_gemm.py) | $8 \times 8$ Block-Tiled NPU GEMM ($K=8$, 4 tiles) | `64/64` INT32 output cells matched in `224` SoC / `92` MAC cycles | **Dynamic $K=8$ Tiling** |
| **M5B** | [upload_m5b_dma_npu.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m5b_dma_npu.py) | Autonomous DMA Controller & 2-Master Bus Arbiter (`0x4000_1000`) | `CPU 16W Copy: 331 cyc` vs. `DMA 16W Burst: 64 cyc` | **`5.17x` DMA Speedup** |
| **M6A** | [upload_m6a_simd_bench.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m6a_simd_bench.py) | 4-Lane 16-Bit Packed SIMD DSP (`0x4000_3000`) | `CPU 32-Elem: 747 cyc` vs. `SIMD Core: 8 cyc` (`DotProd = -2832`) | **`93x` Core Speedup** |
| **M6B** | [upload_m6b_fp32_bench.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m6b_fp32_bench.py) | IEEE-754 Binary32 FP32 Engine (`0x4000_4000`) | `CPU SW Emul: 290 cyc` vs. `FPU Core: 12 cyc` (`FMA = 12.0`) | **`24x` FPU Speedup** |
| **M7** | [upload_m7_daaq_power.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m7_daaq_power.py) | 4-Mode ASTRA-DAAQ Power Ablation + Full `DMA->SIMD->NPU->FP32` | `Mode B: 74 stalls`, `Mode C: E=2098 (0 stalls)`, `Mode D: E=2363 (0 stalls)` | **`+12.6%` DAAQ Energy, `0` Stalls** |
| **M8** | [upload_m8_periph_cache_irq.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/upload_m8_periph_cache_irq.py) | L1 I/D Caches + DMA Snoops + Timer/IRQs + SPI DAC + I²C + GPIO/7-Seg | `D$ Hits=8, Snoops=4`, `IRQs=7`, `SPI=2748`, `I2C=123`, `fcsr=69` | **100% SoC Verified** |
| **OS** | [astra_x1_product_os.py](file:///d:/FINAL YEAR PROJECT/ricscv/scripts/astra_x1_product_os.py) | Interactive On-Chip Product OS & Live Hardware Shell | Live `sw[15:0]` $\rightarrow$ 7-Seg Hex + Runtime UART RX Commands `1/2/3/q` | **Turn-Key Product** |

---

## 4. Developer Platform SDK & File Inventory

### 4.1 Bare-Metal C SDK (`firmware/`)
- [firmware/include/astra_soc.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_soc.h) — Global Memory Map, CSR Macros (`mcycle`, `minstret`, `mstatus`, `mie`, `mip`, `mcause`, `fcsr`), and Interrupt Bit Definitions.
- [firmware/include/astra_npu.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_npu.h) — $4 \times 4$ & Block-Tiled INT8 Systolic NPU Driver API.
- [firmware/include/astra_dma.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_dma.h) — Autonomous Wishbone B4 Burst DMA Driver API.
- [firmware/include/astra_simd.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_simd.h) — 4-Lane INT16 Packed SIMD Vector Driver API.
- [firmware/include/astra_fp32.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_fp32.h) — IEEE-754 Binary32 FPU & 64-Bit `mtime`/`mtimecmp` Real-Time Timer Driver API.
- [firmware/include/astra_daaq.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_daaq.h) — ASTRA-DAAQ 4-Mode Power & Dependency Policy Driver API.
- [firmware/include/astra_periph_cache.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_periph_cache.h) — SPI Master (`MCP4921` DAC), I²C Master, GPIO Switches/Buttons, 4-Digit 7-Segment Display, and L1 Cache Telemetry API.
- [firmware/include/astra_uart.h](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/include/astra_uart.h) — Full-Duplex UART TX/RX Console API.
- [firmware/linker.ld](file:///d:/FINAL YEAR PROJECT/ricscv/firmware/linker.ld) — Bare-Metal RISC-V Linker Script.

### 4.2 Arduino-Style High-Level Python-to-RISC-V Developer SDK (`tools/astra_sdk.py`)
Just as **Arduino IDE** lets users write `setup()` / `loop()` for an ESP32 and click "Upload", **ASTRA-X1** provides a turn-key Python-to-RISC-V SDK ([tools/astra_sdk.py](file:///d:/FINAL YEAR PROJECT/ricscv/tools/astra_sdk.py)) and starter application template ([applications/my_first_astra_app.py](file:///d:/FINAL YEAR PROJECT/ricscv/applications/my_first_astra_app.py)):
- Developers write high-level Python calls (`app.npu_matmul_4x4(...)`, `app.simd_vector_op(...)`, `app.fpu_compute(...)`, `app.seven_seg_hex(...)`, `app.daaq_set_mode(...)`, `app.print_board_telemetry()`).
- Running `python applications/my_first_astra_app.py` automatically compiles the user sketch into native 32-bit `RV32IMFX` machine code (`program.mem`), streams it over the hardware USB-UART bootloader (`COM3 @ 115200 baud`) while Reset (`Pin M14`) is held, and opens a live serial monitor — **zero Vivado re-synthesis required**.

---

## 5. Vivado Placed & Routed Silicon Implementation Reports (`xc7a100t-2ftg256c` @ 50 MHz)

All values below are extracted directly from the completed Vivado 2025.1 physical implementation run in `ricscv.runs/impl_1/`:

### 5.1 Post-Placement Silicon Resource Utilization ([fpga_top_utilization_placed.rpt](file:///d:/FINAL YEAR PROJECT/ricscv/ricscv.runs/impl_1/fpga_top_utilization_placed.rpt))

| FPGA Resource Category | Used | Available (`xc7a100t`) | Utilization (%) | Architectural Notes |
| :--- | ---: | ---: | ---: | :--- |
| **Slice LUTs (Total)** | **19,670** | 63,400 | **31.03%** | 19,618 Logic LUTs + 52 Distributed RAM LUTs |
| **Slice Registers (Flip-Flops)** | **16,561** | 126,800 | **13.06%** | **0 Latches** (100% synchronous edge-triggered design) |
| **Slices** | **6,343** | 15,850 | **40.02%** | Full 7-Slave SoC + L1 I$/D$ + NPU + SIMD + FPU + DMA + DAAQ |
| **Block RAM (`RAMB36E1`)** | **8** | 135 | **5.93%** | 4 × `RAMB36E1` (16 KB IMEM) + 4 × `RAMB36E1` (16 KB DMEM) |
| **DSP48E1 Slices** | **18** | 240 | **7.50%** | 12 (RV32M 32-bit MUL) + 4 (4-Lane SIMD) + 2 (IEEE-754 FPU) |
| **Bonded IOBs (Physical Pins)** | **59** | 170 | **34.71%** | Clock, Reset, UART, 16 LEDs, 16 Switches, 4 Buttons, 7-Seg, SPI, I²C |

### 5.2 Post-Routing Timing Closure Summary ([fpga_top_timing_summary_routed.rpt](file:///d:/FINAL YEAR PROJECT/ricscv/ricscv.runs/impl_1/fpga_top_timing_summary_routed.rpt))

| Timing Metric | Measured Value | Target Constraint | Status |
| :--- | ---: | ---: | :--- |
| **System Clock Period (`sys_clk_pin`)** | **20.000 ns (50.00 MHz)** | 20.000 ns (50.00 MHz) | **MET** |
| **Worst Negative Slack (`WNS` - Setup)** | **+1.612 ns** | $\ge 0.000\text{ ns}$ | **0 Failing Endpoints (out of 34,417)** |
| **Total Negative Slack (`TNS` - Setup)** | **0.000 ns** | $0.000\text{ ns}$ | **MET** |
| **Worst Hold Slack (`WHS` - Hold)** | **+0.060 ns** | $\ge 0.000\text{ ns}$ | **0 Failing Endpoints (out of 34,417)** |
| **Worst Pulse Width Slack (`WPWS`)** | **+8.750 ns** | $\ge 0.000\text{ ns}$ | **0 Failing Endpoints (out of 16,690)** |
| **Maximum Achievable Frequency ($F_{\max}$)** | **54.38 MHz** | $\ge 50.00\text{ MHz}$ | **$+8.8\%$ Frequency Headroom** |

### 5.3 Post-Routing On-Chip Power Breakdown ([fpga_top_power_routed.rpt](file:///d:/FINAL YEAR PROJECT/ricscv/ricscv.runs/impl_1/fpga_top_power_routed.rpt))

| Hierarchy / Subsystem Instance | RTL Module | Power (mW) | % of Dynamic Power |
| :--- | :--- | ---: | ---: |
| **Total On-Chip Power** | `fpga_top` (Dynamic + Static) | **302 mW (0.302 W)** | — (`204 mW` Dynamic + `98 mW` Static) |
| Clock Management & Distribution | `clk_mgr` (`MMCME2_ADV` + Global `BUFG`) | **106 mW** | 51.9% |
| 7-Stage RV32IMFX CPU Core | `IF/ID/EX/MEM1/MEM2/WB` + `regfile` + `muldiv` + `csr` | **39 mW** | 19.1% |
| $4 \times 4$ INT8 Systolic NPU | `u_astra_npu` (`u_systolic` = `10 mW` + buffers = `10 mW`) | **20 mW** | 9.8% |
| On-Chip Block RAM (32 KB Total) | `imem` (`6 mW`) + `u_wb_sram` (`4 mW`) | **10 mW** | 4.9% |
| Autonomous Burst DMA Controller | `u_dma_ctrl` | **8 mW** | 3.9% |
| IEEE-754 Binary32 FP32 Engine | `u_fp32_unit` + `mtime`/`mtimecmp` | **7 mW** | 3.4% |
| Peripherals & I/O Ring | `u_wb_periph` (UART, SPI, I²C, 7-Seg, GPIO) + I/O pads | **8 mW** | 3.9% |
| 4-Lane INT16 Packed SIMD Engine | `u_simd_unit` | **2 mW** | 1.0% |
| Snoop-Coherent L1 I$ & D$ Caches | `u_l1_caches` | **2 mW** | 1.0% |
| ASTRA-DAAQ Power Controller & Bus | `u_daaq_ctrl` + `u_wb_interconnect` | **2 mW** | 1.0% |

