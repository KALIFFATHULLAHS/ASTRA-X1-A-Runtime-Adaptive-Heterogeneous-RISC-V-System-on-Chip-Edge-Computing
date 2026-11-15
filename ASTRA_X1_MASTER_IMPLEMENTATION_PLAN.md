# ASTRA-X1 System-on-Chip: Master Implementation & Engineering Roadmap

**Document Type:** Engineering Specification & Milestone Implementation Plan  
**Project Title:** ASTRA-X1: Design and Implementation of a Runtime-Adaptive Heterogeneous RISC-V System-on-Chip with Floating-Point, SIMD Acceleration, and Power-Aware Execution for Edge Computing  
**Target Hardware:** Digilent / EDGE Artix-7 XC7A100T-2FTG256C FPGA  
**Target Clock:** 50 MHz Synchronous System Domain  
**Interconnect:** 32-bit Synchronous Wishbone B4  
**Primary Application:** Real-time Edge-AI Vibration-Anomaly Classification  
**Date:** November 2026  

---

## 1. Executive Summary & Product Vision

**ASTRA-X1** is a complete, software-programmable, heterogeneous edge-computing System-on-Chip (SoC) designed and verified for Xilinx Artix-7 FPGAs. 

Rather than treating the FPGA as a simple microcontroller, ASTRA-X1 is an integrated heterogeneous acceleration platform:
1. **Host Control Brain:** A verified 32-bit, 7-stage in-order single-issue RISC-V CPU (RV32IM baseline, expanding to RV32IMF with scalar IEEE-754 binary32 floating point).
2. **Dedicated AI Core:** A 16-PE, $4 \times 4$ signed INT8 systolic MAC array with signed INT32 stationary accumulation and dedicated scratchpad buffers, tailored for quantized neural network inferences.
3. **Pre-Processing DSP Engine:** A 4-lane 16-bit packed SIMD engine for high-throughput time-domain vibration sensor filtering.
4. **Data Mover:** A 1-channel block Direct Memory Access (DMA) engine with multi-master bus arbitration.
5. **Research Contribution (ASTRA-DAAQ):** Cross-resource Dependency-Aware Adaptive Quiescence that evaluates break-even transition energies ($E_{\text{benefit}} \approx \Delta P \times T_{\text{idle}} - E_{\text{transition}}$) and monitors execution dependencies to eliminate idle switching without dropping in-flight transactions or incurring wake-up stalls.
6. **End-to-End Product:** Bare-metal C SDK, drivers, hardware bootloader, and an industrial vibration anomaly classification demonstration.

---

## 2. Global SoC Architecture

```text
                             ASTRA-X1 FPGA SoC
 ┌────────────────────────────────────────────────────────────────────────┐
 │                                                                        │
 │   ┌────────────────────────┐                                           │
 │   │  7-STAGE RISC-V CPU    │  <-- Milestone 1 (Verified Working)       │
 │   │  (RV32IM Core)         │                                           │
 │   │  IF1-IF2-ID-EX1-EX2-   │                                           │
 │   │  MEM-WB                │                                           │
 │   │  DSP Multiplier & Div  │                                           │
 │   └───────────┬────────────┘                                           │
 │               │                                                        │
 │               ▼ [CPU Memory Port]                                      │
 │   ┌────────────────────────┐                                           │
 │   │ Wishbone Master Adapter│  <-- Milestone 2                          │
 │   └───────────┬────────────┘                                           │
 │               │                                                        │
 │               ▼                                                        │
 │   ══════════════════════════════════════════════════════════════════   │
 │              32-BIT WISHBONE B4 SoC INTERCONNECT (THE BUS)              │
 │   ══════════════════════════════════════════════════════════════════   │
 │         │                  │                    │            │         │
 │         ▼                  ▼                    ▼            ▼         │
 │   ┌───────────┐      ┌───────────┐        ┌───────────┐ ┌──────────┐   │
 │   │ Data SRAM │      │  NPU      │        │ UART / LED│ │ DMA      │   │
 │   │ (64 KB    │      │Controller │        │Peripherals│ │Controller│   │
 │   │  BRAM)    │      └─────┬─────┘        └───────────┘ └──────────┘   │
 │   └───────────┘            │                                           │
 │                            ▼                                           │
 │                     ┌─────────────┐                                    │
 │                     │NPU Buffers &│                                    │
 │                     │ Scratchpad  │                                    │
 │                     └──────┬──────┘                                    │
 │                            │                                           │
 │                            ▼                                           │
 │                     ┌─────────────┐                                    │
 │                     │ 4 × 4 INT8  │                                    │
 │                     │Systolic MAC │ (Executes 16 Multiplications       │
 │                     │    Array    │  every clock cycle in parallel!)   │
 │                     └─────────────┘                                    │
 │                                                                        │
 │   ┌────────────────────────────────────────────────────────────────┐   │
 │   │               ASTRA-DAAQ ACTIVITY CONTROLLER                   │   │
 │   │  (Monitors CPU, NPU, DMA & Memory to dynamically manage power) │   │
 │   └────────────────────────────────────────────────────────────────┘   │
 └────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Physical Address & Memory Map Contract

All addresses on the 32-bit Wishbone interconnect follow this strict memory contract:

| Physical Address Range | Size | Region Name | Access Privileges | Description / Function |
| :--- | :--- | :--- | :--- | :--- |
| `0x0000_0000 - 0x0000_0FFF` | 4 KB | **Boot BRAM / ROM** | Read / Exec (CPU) | Reset vector `0x0000_0000`. Boot code & initialization. |
| `0x0000_1000 - 0x0000_7FFF` | 28 KB | **Executable App BRAM** | Read / Write / Exec | Application text space loaded via bootloader. |
| `0x2000_0000 - 0x2000_FFFF` | 64 KB | **Data SRAM** | Read / Write (Byte/Word) | General system stack, heap, and static variables. |
| `0x2001_0000 - 0x2001_3FFF` | 16 KB | **NPU Scratchpad RAM** | Read / Write | Dual-port buffer for neural weights, activations, and outputs. |
| `0x4000_0000 - 0x4000_003F` | 64 B | **NPU Control Registers** | Read / Write (MMIO) | NPU ID (`0x41535452`), Status, Control, Matrix Dimensions. |
| `0x4000_1000 - 0x4000_101F` | 32 B | **DMA Control Registers** | Read / Write (MMIO) | Source address, Destination address, Length, Trigger, IRQ. |
| `0x4000_2000 - 0x4000_203F` | 64 B | **ASTRA-DAAQ Registers** | Read / Write (MMIO) | Activity mode select, idle thresholds, software phase hints. |
| `0x4000_3000 - 0x4000_301F` | 32 B | **SIMD Control Registers**| Read / Write (MMIO) | 4-lane packed 16-bit operation triggers and accumulator. |
| `0x4000_4000 - 0x4000_401F` | 32 B | **Timer / Interrupt Unit** | Read / Write (MMIO) | 64-bit real-time counter (`mtime`, `mtimecmp`). |
| `0xFFFF_0000 - 0xFFFF_0003` | 4 B | **Legacy UART TX Data** | Write-only (MMIO) | Preserved: Writing byte triggers serial transmit. |
| `0xFFFF_0004 - 0xFFFF_0007` | 4 B | **Board LEDs / GPIO** | Read / Write (MMIO) | Preserved: Lower 8 bits drive physical board LEDs. |
| `0xFFFF_0008 - 0xFFFF_000B` | 4 B | **UART RX Data / Status** | Read-only (MMIO) | New: Software-readable received byte and RX-ready flag. |

---

## 4. Master Milestone Roadmap (Milestones 1 to 8)

```text
[M1: CPU Baseline] ──> [M2: SoC Wishbone Bus] ──> [M3: 4x4 INT8 Systolic Core]
                                                          │
[M6: SIMD & FPU]   <── [M5: DMA & Edge AI Model] <── [M4: Full SoC NPU Integration]
       │
[M7: ASTRA-DAAQ Power Research] ──> [M8: Final Product, SDK & Demo]
```

---

### Milestone 1: 7-Stage RV32IM CPU Core Baseline
* **Status:** Verified Complete
* **Delivered RTL:** `if1.v`, `if2.v`, `id.v`, `ex1.v`, `ex2.v`, `mem.v`, `wb.v`, `hazard_unit.v`, `forwarding_unit.v`, `branch_predictor.v`, `mul_unit.v`, `div_unit.v`, `csr_unit.v`, `clock_manager.v`, `stage_gating_controller.v`, `fpga_top.v`.
* **Verified Capabilities:**
  - 50 MHz timing closure on Artix-7 XC7A100T.
  - DSP48E1 hardware multiplier and 32-cycle restoring iterative divider.
  - 64-entry BHT + BTB dynamic branch prediction.
  - Hardware UART byte-stream bootloader for instruction memory.
  - Verified on hardware via `upload_master_suite.py` ($12 \times 7 = 84$, $168 / 4 = 42$, subroutines).

---

### Milestone 2: SoC Wishbone Interconnect & Mock NPU
* **Status:** Immediate Next Step
* **Objective:** Connect the CPU memory stage to a 32-bit Wishbone B4 interconnect, resolve synchronous BRAM load wait-states, map a Mock NPU slave, and execute a live CPU program that reads the NPU ID and outputs `PASS` via UART.

#### Detailed Engineering Tasks:
1. **CPU Wishbone Master Adapter (`mem.v` & `hazard_unit.v`):**
   - Implement Wishbone B4 Master ports: `wb_adr_o`, `wb_dat_o`, `wb_we_o`, `wb_sel_o`, `wb_stb_o`, `wb_cyc_o`, `wb_dat_i`, `wb_ack_i`, `wb_err_i`.
   - Implement `stall_mem` in `hazard_unit.v`. On any memory instruction (`mem_read` or `mem_write`), stall stages `IF`, `ID`, `EX1`, `EX2` until `wb_ack_i` or `wb_err_i` is returned.
   - Latch `wb_dat_i` directly when `wb_ack_i` arrives, fixing the synchronous BRAM read latency issue cleanly.
2. **Wishbone Crossbar & Address Decoder (`wb_interconnect.v`):**
   - Address decoding for Data SRAM, Mock NPU, and UART/GPIO peripherals.
   - Slave response multiplexing for read data and acknowledge lines.
   - Terminate unmapped addresses with `wb_err_i = 1` in 1 clock cycle to prevent CPU deadlock.
3. **Mock NPU Register Slave (`wb_mock_npu.v`):**
   - `0x4000_0000` (`NPU_REG_ID`): Read-only register returning `32'h4153_5452` (`"ASTR"`).
   - `0x4000_0004` (`NPU_REG_STATUS`): Read-only register returning `32'h0000_0003` (`READY | IDLE`).
   - `0x4000_0008` (`NPU_REG_SCRATCH`): Read/Write 32-bit register for bus read/write testing.
4. **Integration & Test Program:**
   - Write assembly/C program loading `0x4000_0000`. If ID equals `0x41535452`, transmit `"PASS\n"` over UART.
   - Verify via XSim simulation, synthesize bitstream, and run physical FPGA test.

* **Exit Criteria:** CPU reads Mock NPU ID over the Wishbone bus and prints `"PASS"` to the host serial terminal.

---

### Milestone 3: 4×4 INT8 Systolic Array Accelerator Core
* **Objective:** Design, simulate, and numerically verify a standalone 16-PE matrix multiplication hardware engine.

#### Detailed Engineering Tasks:
1. **Processing Element (`npu_pe.v`):**
   - Inputs: Signed 8-bit activation ($A$), signed 8-bit weight ($B$).
   - Multiplier: $8 \times 8 \rightarrow \text{signed 16-bit}$ product.
   - Accumulator: Signed 32-bit accumulator with saturation:
     $$C_{\text{acc}} \leftarrow C_{\text{acc}} + (A \times B)$$
   - Pipeline registers for horizontal activation forwarding and vertical weight forwarding.
2. **Systolic Array Mesh (`npu_systolic_array.v`):**
   - $4 \times 4$ grid of 16 PEs operating in output-stationary mode.
   - Skew-buffering logic to stagger matrix input flows by 1 cycle per row/column.
3. **Controller & Verification (`tb_npu_systolic.v` & `tools/npu_matrix_gen.py`):**
   - Python reference script generates randomized matrices with edge cases ($-128, +127, 0$).
   - Validate hardware outputs against Python NumPy golden results.

* **Exit Criteria:** 10,000 matrix multiplications achieve 100% bit-exact match against NumPy reference.

---

### Milestone 4: Full SoC NPU Integration & Scratchpad Memory
* **Objective:** Attach the systolic array to the Wishbone bus with dual-port scratchpad memory, enabling bare-metal C programs to execute hardware-accelerated matrix multiplication.

#### Detailed Engineering Tasks:
1. **NPU Scratchpad (`npu_scratchpad.v`):**
   - 16 KB Dual-Port BRAM:
     - Port A: Connected to Wishbone bus (CPU reads/writes inputs, weights, outputs).
     - Port B: Connected to NPU array controller (wide parallel feed).
   - Partitions: 4 KB Activations, 4 KB Weights, 8 KB Output Accumulators.
2. **Complete NPU Wishbone Slave (`wb_astra_npu.v`):**
   - Control registers: `NPU_CTRL` (Start, Soft Reset), `NPU_STATUS` (Busy, Done), `NPU_K_DIM`, `NPU_CYCLES`.
3. **Bare-Metal C Driver (`firmware/drivers/astra_npu.h`):**
   - Functions: `npu_write_weights()`, `npu_write_inputs()`, `npu_start()`, `npu_wait_done()`, `npu_read_outputs()`.
4. **Hardware Verification:**
   - CPU writes test matrices into scratchpad, starts NPU, polls completion, reads results, and validates output over UART.

* **Exit Criteria:** CPU successfully offloads matrix operations to NPU and confirms matching mathematical results over serial output.

---

### Milestone 5: DMA Engine & Real Edge-AI Inference Application
* **Objective:** Implement a 1-channel block DMA engine and execute complete INT8 neural network inference for industrial vibration anomaly detection.

#### Detailed Engineering Tasks:
1. **DMA Controller (`wb_dma.v` & `wb_arbiter.v`):**
   - Bus arbiter arbitrates between CPU Master and DMA Master.
   - Autonomous memory block transfers: System SRAM $\longleftrightarrow$ NPU Scratchpad.
2. **Vibration Anomaly Detection Model:**
   - Neural Network: Multi-Layer Perceptron (MLP) (16 Features $\rightarrow$ 16 Hidden $\rightarrow$ 2 Output Classes: *Normal* vs *Fault*).
   - Post-Training Quantization (PTQ) to INT8 weights and activations.
   - Export weights and calibration parameters to C header files.
3. **Inference Pipeline Execution:**
   - DMA transfers input features and weights to NPU scratchpad.
   - NPU executes layer computations: $\mathbf{Y} = \text{ReLU}(\mathbf{W} \cdot \mathbf{X} + \mathbf{B})$.
   - CPU computes classification result and streams anomaly score to host over UART.

* **Exit Criteria:** FPGA accurately classifies vibration test vectors with $<100\,\mu\text{s}$ latency, bit-exact with the host quantization model.

---

### Milestone 6: Heterogeneous Compute Expansion (SIMD & FPU)
* **Objective:** Add a 4-lane 16-bit packed SIMD engine and a scalar FP32 FPU to complete the heterogeneous architecture.

Milestone 6 — Extend the processor and SoC
Target: January–February 2027

Work: Complete trap/interrupt support, integrate FP32, SIMD, DMA, peripherals and the planned memory hierarchy.

Completion evidence: Independent module tests plus integrated FPGA applications.

#### Detailed Engineering Tasks:
1. **4-Lane 16-bit SIMD Accelerator (`wb_simd.v`):**
   - Memory-mapped at `0x4000_3000`.
   - Two 64-bit operand vectors holding four 16-bit signed integers each.
   - Operations: Parallel ADD16, SUB16, MUL16, MAC16, ABS16, RELU16.
   - Accelerates pre-processing (FFT windowing, mean removal, RMS calculation).
2. **IEEE-754 FP32 Floating-Point Unit (`fpu/`):**
   - Add FP register file ($32 \times 32$-bit `f0-f31`).
   - Implement scalar operations: `FADD.S`, `FSUB.S`, `FMUL.S`, `FDIV.S`, `FCVT`, `FMIN/FMAX`.
   - Integrate with core pipeline decode and hazard logic.

* **Exit Criteria:** SIMD accelerates sensor normalization by $>3\times$ over scalar code; FPU passes standard floating-point compliance test vectors.

---

### Milestone 7: ASTRA-DAAQ Research Power Architecture
* **Objective:** Implement the Dependency-Aware Adaptive Quiescence controller, measure actual power savings on the Artix-7, and prove transaction safety.

#### Detailed Engineering Tasks:
1. **Activity Monitor & Dependency Tracker (`daaq_tracker.v`):**
   - Taps CPU pipeline valid/stall flags, NPU busy/start/done flags, DMA in-flight signals, and memory wait states.
   - Maintains an exact count of in-flight dependencies so units never quiesce when work is pending.
2. **Policy Engine (`daaq_policy.v`):**
   - Evaluates:
     $$E_{\text{benefit}} \approx \Delta P \times T_{\text{idle}} - E_{\text{transition}}$$
   - Software Phase Hints register (`0x4000_2004`): Firmware indicates upcoming workload phases (`CONTROL`, `PREPROCESS`, `INFERENCE`, `TRANSFER`, `IDLE`).
3. **Safe Quiescence Controller (`daaq_quiescence.v`):**
   - Glitch-free synchronous clock enables and operand isolation.
   - Preserves all architectural state and wake-up responsiveness.
4. **4-Way Controlled Ablation Experimentation:**
   - **Mode A (Baseline):** No activity management.
   - **Mode B (Local Gating):** Naive idle gating without dependency tracking.
   - **Mode C (Fixed DAAQ):** Dependency-aware quiescence with fixed thresholds.
   - **Mode D (Adaptive DAAQ):** Full adaptive thresholding with software phase hints.
5. **Power & Energy Quantification:**
   - Record exact cycle latency ($T_{\text{total}}$).
   - Generate SAIF switching activity files from real workloads.
   - Use Vivado Power Analyzer to measure on-chip power ($\text{mW}$) and calculate energy per task:
     $$E_{\text{task}} = P_{\text{avg}} \times T_{\text{task}}$$

* **Exit Criteria:** Zero lost transactions demonstrated in hardware; measurable dynamic power savings proven against baseline without performance loss.

---

### Milestone 8: Turn-Key Productization, Bare-Metal SDK & Demo
* **Objective:** Deliver a complete, reproducible FPGA platform with software toolchain, documentation, and live demonstration.

#### Detailed Engineering Tasks:
1. **Software Bootloader:**
   - Resides in initial 4 KB BRAM. Receives application `.bin` files over UART RX and programs application memory without holding physical reset buttons.
2. **ASTRA Bare-Metal SDK:**
   - Headers: `astra_soc.h`, `astra_npu.h`, `astra_dma.h`, `astra_daaq.h`, `astra_uart.h`.
   - GCC Linker script (`linker.ld`) mapping text, data, and stack to exact physical memory locations.
   - Host Python upload script (`astra_uploader.py`).
3. **Live Demonstration:**
   - Host serial terminal displays real-time inference results, latency metrics, and DAAQ energy savings.

* **Exit Criteria:** Standalone FPGA boot, fast application upload via UART ($<5\,\text{s}$), and live anomaly inference demonstration.

---

## 5. Architectural Design Rationale

### Why Wishbone B4 Classic?
* **Simplicity vs. AXI4 Overhead:** AXI4-Full requires 5 independent channels (AW, W, B, AR, R) with complex handshake logic. On an Artix-7 XC7A100T, AXI4 crossbars consume thousands of LUTs and complicate pipeline backpressure.
* **Deterministic Handshake:** Wishbone B4 Classic provides an elegant `cyc/stb/ack` handshake that integrates directly into the CPU's memory stage, supporting zero-wait-state BRAM accesses and multi-cycle peripheral stalls.

### Why a 4×4 INT8 Systolic Array?
* **FPGA DSP Optimization:** An Artix-7 100T has 240 DSP48E1 slices. A $4 \times 4$ array uses only 16 DSPs (or can even be mapped to LUTs). This leaves ample DSP resources available for the CPU multiplier, divider, SIMD, and FPU.
* **Compute-to-Bandwidth Ratio:** Matrix multiplication has high data reuse. Storing intermediate sums in stationary accumulators reduces memory bandwidth bottlenecks over the 32-bit SoC bus.

### Why INT8 Inputs with INT32 Accumulation?
* 8-bit integer weights reduce memory storage by $4\times$ compared to 32-bit floats.
* Summing products of two 8-bit numbers ($-128 \times -128 = 16384$) across deep matrices will overflow 16 bits rapidly. 32-bit accumulators eliminate overflow risk during matrix reductions.

---

## 6. Deliverables & Verification Checklist

| Deliverable Artifact | Description / Location | Target Milestone |
| :--- | :--- | :---: |
| **Synthesizable Bus RTL** | `soc/wb_interconnect.v`, `soc/wb_master_adapter.v` | **M2** |
| **Mock NPU Register Slave** | `npu/wb_mock_npu.v` (ID: `0x41535452`) | **M2** |
| **M2 Hardware Test Suite** | `scripts/upload_m2_npu_test.py`, `applications/m2_test/` | **M2** |
| **Systolic Array RTL** | `npu/npu_pe.v`, `npu/npu_systolic_array.v` | **M3** |
| **Python Golden Model** | `tools/npu_matrix_gen.py` | **M3** |
| **NPU Scratchpad & Wrapper** | `npu/npu_scratchpad.v`, `npu/wb_astra_npu.v` | **M4** |
| **DMA Controller** | `soc/wb_dma.v`, `soc/wb_arbiter.v` | **M5** |
| **Trained Vibration Model** | `firmware/vibration_model/weights.h` | **M5** |
| **SIMD & FPU Modules** | `simd/wb_simd.v`, `fpu/` | **M6** |
| **ASTRA-DAAQ Controller** | `daaq/daaq_tracker.v`, `daaq/daaq_policy.v` | **M7** |
| **Power Ablation Report** | `docs/results/daaq_power_comparison.pdf` | **M7** |
| **Bare-Metal C SDK** | `firmware/include/astra_*.h`, `firmware/linker.ld` | **M8** |
| **Final Artix-7 Bitstream** | `fpga/bitstream/astra_x1_final.bit` | **M8** |
