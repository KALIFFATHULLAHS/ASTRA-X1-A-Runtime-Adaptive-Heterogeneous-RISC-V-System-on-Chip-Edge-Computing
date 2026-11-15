#!/usr/bin/env python3
r"""
ASTRA-X1 Master Final Project Report PDF Generator
==================================================
Generates a publication- and thesis-grade PDF dossier (`ASTRA_X1_FINAL_PROJECT_REPORT.pdf`)
complete with cover page, numbered headers/footers, architecture tables, memory maps,
patent claims, Vivado 2025.1 Placed & Routed Silicon Utilization/Timing/Power tables,
Milestone 1-8 hardware benchmark tables, and all 10 embedded physical EDGE Artix-7 100T
hardware verification captures.
"""

import os
import sys
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, KeepTogether, HRFlowable, Image, Preformatted
)
from reportlab.pdfgen import canvas


class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        if self._pageNumber == 1:
            return
        self.saveState()
        self.setFont("Helvetica-Bold", 8)
        self.setFillColor(colors.HexColor("#1E3A8A"))
        self.drawString(48, 11 * 72 - 34, "ASTRA-X1 HETEROGENEOUS RISC-V SoC — FINAL YEAR PROJECT REPORT & SILICON DOSSIER")
        self.setStrokeColor(colors.HexColor("#CBD5E1"))
        self.setLineWidth(0.75)
        self.line(48, 11 * 72 - 40, 8.5 * 72 - 48, 11 * 72 - 40)

        self.setFont("Helvetica", 8)
        self.setFillColor(colors.HexColor("#475569"))
        self.line(48, 44, 8.5 * 72 - 48, 44)
        self.drawString(48, 30, "Target Silicon: Xilinx Artix-7 XC7A100T-2FTG256C @ 50 MHz | WNS = +1.612 ns | Power = 302 mW")
        self.drawRightString(8.5 * 72 - 48, 30, f"Page {self._pageNumber} of {page_count}")
        self.restoreState()


def make_table(data, col_widths, header_bg="#0F2942"):
    t = Table(data, colWidths=col_widths, repeatRows=1)
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor(header_bg)),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, 0), 8.5),
        ("BOTTOMPADDING", (0, 0), (-1, 0), 6),
        ("TOPPADDING", (0, 0), (-1, 0), 6),
        ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
        ("FONTSIZE", (0, 1), (-1, -1), 8),
        ("TEXTCOLOR", (0, 1), (-1, -1), colors.HexColor("#1E293B")),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F8FAFC")]),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E1")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 1), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 1), (-1, -1), 4),
    ]))
    return t


def build_pdf(out_path, img_dir):
    doc = SimpleDocTemplate(
        out_path,
        pagesize=letter,
        leftMargin=48,
        rightMargin=48,
        topMargin=52,
        bottomMargin=52
    )

    styles = getSampleStyleSheet()
    PRIMARY = colors.HexColor("#0F2942")
    SECONDARY = colors.HexColor("#1D4ED8")
    DARK_TEXT = colors.HexColor("#1E293B")

    title_style = ParagraphStyle(
        "CoverTitle", parent=styles["Title"], fontName="Helvetica-Bold",
        fontSize=21, leading=26, textColor=PRIMARY, alignment=0, spaceAfter=10
    )
    subtitle_style = ParagraphStyle(
        "CoverSub", parent=styles["Normal"], fontName="Helvetica-Bold",
        fontSize=11.5, leading=16, textColor=SECONDARY, spaceAfter=14
    )
    h1_style = ParagraphStyle(
        "H1", parent=styles["Heading1"], fontName="Helvetica-Bold",
        fontSize=14, leading=18, textColor=PRIMARY, spaceBefore=14, spaceAfter=6
    )
    h2_style = ParagraphStyle(
        "H2", parent=styles["Heading2"], fontName="Helvetica-Bold",
        fontSize=11, leading=15, textColor=SECONDARY, spaceBefore=10, spaceAfter=4
    )
    body_style = ParagraphStyle(
        "Body", parent=styles["Normal"], fontName="Helvetica",
        fontSize=9.2, leading=13.5, textColor=DARK_TEXT, spaceAfter=6
    )
    bullet_style = ParagraphStyle(
        "BulletItem", parent=body_style, leftIndent=14, bulletIndent=4, spaceAfter=4
    )
    code_style = ParagraphStyle(
        "CodeBlock", fontName="Courier", fontSize=7.8, leading=10.2,
        textColor=colors.HexColor("#0F172A"), backColor=colors.HexColor("#F1F5F9"),
        borderPadding=6, leftIndent=6, rightIndent=6, spaceAfter=8
    )
    caption_style = ParagraphStyle(
        "Caption", parent=styles["Normal"], fontName="Helvetica-Oblique",
        fontSize=8.2, leading=11, textColor=colors.HexColor("#475569"),
        alignment=1, spaceBefore=3, spaceAfter=10
    )

    story = []

    # =========================================================================
    # PAGE 1: COVER PAGE & EXECUTIVE SUMMARY
    # =========================================================================
    story.append(Spacer(1, 14))
    story.append(Paragraph(
        "DESIGN AND IMPLEMENTATION OF A RUNTIME-ADAPTIVE HETEROGENEOUS RISC-V SYSTEM-ON-CHIP WITH FLOATING-POINT, SIMD ACCELERATION, AND POWER-AWARE EXECUTION FOR EDGE COMPUTING",
        title_style
    ))
    story.append(Paragraph(
        "Architecture Codename: ASTRA-X1 | Target Silicon: Xilinx Artix-7 XC7A100T-2FTG256C (EDGE Artix-7 100T Board)",
        subtitle_style
    ))
    story.append(HRFlowable(width="100%", thickness=1.5, color=PRIMARY, spaceAfter=12))

    meta_data = [
        ["Parameter", "Verified Physical Silicon Specification"],
        ["Processor Core", "7-Stage In-Order Pipelined RV32IMFX Core (IF1, IF2, ID, EX1, EX2, MEM, WB) + 64-Entry BHT Predictor"],
        ["Cache Hierarchy", "64-Line Direct-Mapped L1 I-Cache + Write-Through L1 D-Cache with Single-Cycle DMA Write Snooping"],
        ["System Interconnect", "32-Bit 2-Master (CPU Master 0, DMA Master 1) x 7-Slave Wishbone B4 Crossbar & Priority Arbiter"],
        ["Heterogeneous Engines", "Slave 1: 4x4 INT8 Systolic NPU | Slave 3: Burst DMA | Slave 4: ASTRA-DAAQ | Slave 5: 4-Lane SIMD | Slave 6: IEEE-754 FP32"],
        ["Board Peripherals", "Full-Duplex UART (115200), 16 LEDs, 16 Slide Switches, 4 Push Buttons, Active-High 4-Digit 7-Seg, SPI DAC, I2C Master"],
        ["Silicon Utilization", "19,670 Slice LUTs (31.03%) | 16,561 Flip-Flops (13.06%, 0 Latches) | 8 RAMB36E1 (32 KB) | 18 DSP48E1 Slices (7.50%)"],
        ["Timing & Power", "50.00 MHz System Clock | WNS = +1.612 ns (Fmax = 54.38 MHz) | WHS = +0.060 ns | Total Power = 302 mW (204 mW Dynamic)"],
        ["Developer SDK", "3-Tier Platform: (1) Python-to-RISC-V SDK, (2) Bare-Metal C SDK (firmware/include/*.h), (3) Interactive On-Chip Product OS"]
    ]
    story.append(make_table(meta_data, [1.6 * inch, 5.5 * inch]))
    story.append(Spacer(1, 10))

    story.append(Paragraph("1. Executive Summary & Architectural Problem Statement", h1_style))
    story.append(Paragraph(
        "Modern edge-AI and cyber-physical embedded systems require executing diverse computational workloads — combining scalar control flow (RISC-V CPU), parallel signal preprocessing (Packed INT16 SIMD), dense deep-learning matrix multiplication (INT8 Systolic Array NPU), autonomous block data movement (Burst DMA), and high-dynamic-range post-processing (IEEE-754 Binary32 FP32) — within a tight milliwatt power budget.",
        body_style
    ))
    story.append(Paragraph(
        "Conventional homogeneous microcontrollers suffer from severe latency bottlenecks (80x-183x slower on matrix/vector kernels), while conventional heterogeneous SoCs face two critical architectural challenges when applying fine-grained clock gating:",
        body_style
    ))
    story.append(Paragraph(
        "• <b>Cold-Wake Pipeline Hazards:</b> Independent local idle-timeout clock gating (Mode B) prematurely gates downstream accelerators while upstream producers (DMA or SIMD) are still preparing operands, causing <b>74 wake-up stall cycles</b> per pipeline pass.",
        bullet_style
    ))
    story.append(Paragraph(
        "• <b>Cache-Coherency Quiescence Overhead:</b> When an autonomous DMA engine writes accelerator output tiles into system SRAM, traditional cached SoCs require software cache invalidation loops before the CPU can read results or before the accelerator domain can enter quiescence.",
        bullet_style
    ))
    story.append(Paragraph(
        "<b>ASTRA-X1</b> resolves both bottlenecks via a unified <b>2-Master x 7-Slave Wishbone B4 Heterogeneous SoC</b> governed by the novel <b>ASTRA-DAAQ</b> (<i>Cross-Domain Dependency-Aware Adaptive Quiescence</i>) hardware controller and a <b>Single-Cycle DMA Write-Snoop Coherent L1 Data Cache</b>.",
        body_style
    ))

    # =========================================================================
    # SECTION 2: GLOBAL MEMORY MAP & PATENT NOVELTY CLAIMS
    # =========================================================================
    story.append(Paragraph("2. Global Memory Map & Wishbone B4 Address Space", h1_style))
    mem_data = [
        ["Bus Entity", "Address Range", "ID (0x00)", "Architectural Description"],
        ["Instruction BRAM", "0x0000_0000 - 0x0000_3FFF", "—", "16 KB Dual-Mode Boot BRAM ($readmemh) + UART Bootloader + L1 I-Cache"],
        ["Slave 0: Data SRAM", "0x2000_0000 - 0x2000_3FFF", "—", "16 KB Wishbone Data SRAM + Snoop-Coherent L1 D-Cache"],
        ["Slave 1: Systolic NPU", "0x4000_0000 - 0x4000_0FFF", "'ASTR'", "4x4 Signed INT8 Systolic Array (16 PEs, 32b Acc, Dynamic K=1..64 Tiling)"],
        ["Slave 3: Burst DMA", "0x4000_1000 - 0x4000_1FFF", "'DMA1'", "Autonomous Wishbone Master-1 Block Transfer Engine + Completion IRQ"],
        ["Slave 4: ASTRA-DAAQ", "0x4000_2000 - 0x4000_2FFF", "'DAAQ'", "Cross-Domain Dependency-Aware Adaptive Quiescence Power Controller"],
        ["Slave 5: Packed SIMD", "0x4000_3000 - 0x4000_3FFF", "'SIMD'", "4-Lane 16-Bit Signed Vector DSP (ADD16, SUB16, MUL16, MAC16, ABS16, RELU16, MAX16, MIN16)"],
        ["Slave 6: FP32 & Timer", "0x4000_4000 - 0x4000_4FFF", "'FPU1'", "IEEE-754 Binary32 FPU (FADD.S..FMADD.S), 16x32b FP RF, 64b mtime/mtimecmp, IRQ Ctrl"],
        ["Slave 2: Peripherals", "0xFFFF_0000 - 0xFFFF_00FF", "—", "UART TX/RX, 16 LEDs, 16 Switches, 4 Buttons, Active-High 7-Seg, SPI DAC, I2C, L1$ Telemetry"]
    ]
    story.append(make_table(mem_data, [1.35 * inch, 1.55 * inch, 0.65 * inch, 3.55 * inch]))

    story.append(Paragraph("3. Core Research Novelty & Patentable Claims (ASTRA-DAAQ)", h1_style))
    story.append(Paragraph(
        "• <b>Claim 1 — Producer-Consumer Cross-Engine Dependency Matrix Inhibiting Premature Quiescence:</b> Taps live accelerator busy and Wishbone strobe wires (dma_busy, simd_busy, npu_busy, fpu_busy) and clamps a downstream consumer engine's idle counter to ACTIVE while its upstream producer is active. Eliminates all <b>74 cold-wake stall cycles</b> measured in Mode B down to <b>0 wake stalls</b> in Modes C & D.",
        bullet_style
    ))
    story.append(Paragraph(
        "• <b>Claim 2 — Break-Even Energy Evaluator with Phase-Hinted Adaptive Hysteresis:</b> Evaluates break-even quiescence (E_benefit = dP * T_idle - E_wake > 0) and dynamically modulates idle thresholds (T_idle = 2 vs. 12 cycles) using architectural phase hints. Achieves <b>+12.6% higher energy savings (2363 vs. 2098 units)</b> over Fixed-Threshold DAAQ with <b>0 wake stalls</b>.",
        bullet_style
    ))
    story.append(Paragraph(
        "• <b>Claim 3 — Single-Cycle Wishbone Master-1 DMA Write-Snoop L1 Data Cache Coherency:</b> Monitors autonomous DMA Master-1 writes on the shared SRAM bus and updates/pre-warms matching L1 Data Cache lines in a single clock cycle (DCACHE_SNOOPS = 4), enabling immediate zero-miss CPU reads (DCACHE_HITS = 8) without software cache invalidation.",
        bullet_style
    ))

    story.append(PageBreak())

    # =========================================================================
    # PAGE 2: VIVADO PLACED & ROUTED SILICON IMPLEMENTATION REPORTS
    # =========================================================================
    story.append(Paragraph("4. Vivado 2025.1 Placed & Routed Silicon Implementation Reports", h1_style))
    story.append(Paragraph(
        "All metrics below are extracted directly from the completed Vivado 2025.1 physical implementation run (ricscv.runs/impl_1/) targeting the Xilinx Artix-7 XC7A100T-2FTG256C device on the EDGE Artix-7 100T board:",
        body_style
    ))

    story.append(Paragraph("4.1 Post-Placement Silicon Resource Utilization (fpga_top_utilization_placed.rpt)", h2_style))
    util_data = [
        ["FPGA Resource Category", "Used", "Available", "Utilization (%)", "Architectural Notes"],
        ["Slice LUTs (Total)", "19,670", "63,400", "31.03%", "19,618 Logic LUTs + 52 Distributed RAM LUTs"],
        ["Slice Registers (Flip-Flops)", "16,561", "126,800", "13.06%", "0 Latches (100% synchronous edge-triggered RTL)"],
        ["Occupied Slices", "6,343", "15,850", "40.02%", "Full 7-Slave SoC + L1 I$/D$ + NPU + SIMD + FPU + DMA + DAAQ"],
        ["Block RAM Tile (RAMB36E1)", "8", "135", "5.93%", "4 (16 KB Instruction BRAM) + 4 (16 KB Data SRAM)"],
        ["DSP48E1 Slices", "18", "240", "7.50%", "12 (RV32M 32b MUL) + 4 (4-Lane SIMD) + 2 (IEEE-754 FPU)"],
        ["Bonded IOBs (Physical Pins)", "59", "170", "34.71%", "Clock, Reset, UART, 16 LEDs, 16 Switches, 4 Buttons, 7-Seg, SPI, I2C"]
    ]
    story.append(make_table(util_data, [1.65 * inch, 0.7 * inch, 0.75 * inch, 0.9 * inch, 3.1 * inch]))

    story.append(Paragraph("4.2 Post-Routing Timing Closure Summary @ 50 MHz (fpga_top_timing_summary_routed.rpt)", h2_style))
    timing_data = [
        ["Timing Metric", "Measured Silicon Value", "Target Constraint", "Status / Margin"],
        ["System Clock Period (sys_clk_pin)", "20.000 ns (50.00 MHz)", "20.000 ns (50.00 MHz)", "MET"],
        ["Worst Negative Slack (WNS - Setup)", "+1.612 ns", ">= 0.000 ns", "0 Failing Endpoints (out of 34,417)"],
        ["Total Negative Slack (TNS - Setup)", "0.000 ns", "0.000 ns", "MET"],
        ["Worst Hold Slack (WHS - Hold)", "+0.060 ns", ">= 0.000 ns", "0 Failing Endpoints (out of 34,417)"],
        ["Worst Pulse Width Slack (WPWS)", "+8.750 ns", ">= 0.000 ns", "0 Failing Endpoints (out of 16,690)"],
        ["Maximum Achievable Frequency (Fmax)", "54.38 MHz", ">= 50.00 MHz", "+8.8% Frequency Headroom"]
    ]
    story.append(make_table(timing_data, [2.1 * inch, 1.5 * inch, 1.4 * inch, 2.1 * inch]))

    story.append(Paragraph("4.3 Post-Routing On-Chip Power Breakdown (fpga_top_power_routed.rpt)", h2_style))
    power_data = [
        ["Subsystem Hierarchy", "RTL Module Instance", "Power (mW)", "Share of Dynamic Power"],
        ["Total On-Chip Power", "fpga_top (Dynamic + Static)", "302 mW (0.302 W)", "204 mW Dynamic + 98 mW Static"],
        ["Clock Tree & MMCM", "clk_mgr (MMCME2_ADV + BUFG)", "106 mW", "51.9%"],
        ["7-Stage RV32IMFX CPU Core", "IF/ID/EX/MEM/WB + regfile + muldiv + csr", "39 mW", "19.1%"],
        ["4x4 INT8 Systolic Array NPU", "u_astra_npu (u_systolic = 10 mW + buffers = 10 mW)", "20 mW", "9.8%"],
        ["32 KB On-Chip Block RAM", "imem (6 mW) + u_wb_sram (4 mW)", "10 mW", "4.9%"],
        ["Autonomous Burst DMA Engine", "u_dma_ctrl", "8 mW", "3.9%"],
        ["Peripherals & I/O Ring", "u_wb_periph (UART, SPI, I2C, 7-Seg, GPIO) + IOBs", "8 mW", "3.9%"],
        ["IEEE-754 FP32 Engine & Timer", "u_fp32_unit", "7 mW", "3.4%"],
        ["4-Lane INT16 Packed SIMD", "u_simd_unit", "2 mW", "1.0%"],
        ["Snoop-Coherent L1 I$ & D$", "u_l1_caches", "2 mW", "1.0%"],
        ["ASTRA-DAAQ Power Ctrl & Bus", "u_daaq_ctrl + u_wb_interconnect", "2 mW", "1.0%"]
    ]
    story.append(make_table(power_data, [1.85 * inch, 2.55 * inch, 1.1 * inch, 1.6 * inch]))

    story.append(Paragraph("5. Master Hardware Benchmark Matrix (Milestones 1–8)", h1_style))
    bench_data = [
        ["Milestone", "Subsystem Verified on EDGE Artix-7 100T", "CPU Baseline", "Hardware Engine", "Measured Speedup / Result"],
        ["M1 - M3", "7-Stage RV32IMFX Core + Wishbone B4 + Systolic Array", "—", "—", "NPU_ID='ASTR', 10,000 NumPy matrices PASS"],
        ["M4", "4x4 Signed INT8 Systolic NPU GEMM (0x4000_0000)", "3,482 cyc", "43 SoC / 19 MAC cyc", "80x SoC Speedup / 183x MAC Speedup"],
        ["M5A", "8x8 Block-Tiled NPU GEMM (K=8, 4 Tiles, 64 Cells)", "27,856 cyc", "224 SoC / 92 MAC cyc", "64/64 INT32 Cells Matched (124x SoC)"],
        ["M5B", "Autonomous DMA Controller & 2-Master Arbiter", "331 cyc", "64 DMA burst cyc", "5.17x Data-Movement Speedup"],
        ["M6A", "4-Lane 16-Bit Packed SIMD Vector DSP (32-Elem Dot)", "747 cyc", "40 SoC / 8 SIMD cyc", "93x Core Speedup (DotProd = -2832)"],
        ["M6B", "IEEE-754 Single-Precision FP32 Engine (FMA DotProd)", "290 cyc", "12 FPU core cyc", "24x FPU Speedup (FMA = 12.0)"],
        ["M7", "4-Mode ASTRA-DAAQ Power Ablation + Full Hetero Pipe", "74 Wake Stalls", "0 Wake Stalls", "E_Saved = 2363 (+12.6% vs Mode C)"],
        ["M8", "L1 I$/D$ + DMA Snoops + Timer/IRQs + SPI + I2C + 7-Seg", "—", "—", "I$=90, D$=8, Snoops=4, IRQs=7, SPI=2748"],
        ["SDK / OS", "Python SDK + Bare-Metal C SDK + Interactive Product OS", "—", "—", "All 3 Programming Flows Verified on COM3"]
    ]
    story.append(make_table(bench_data, [0.75 * inch, 2.55 * inch, 1.0 * inch, 1.15 * inch, 1.65 * inch]))

    story.append(PageBreak())

    # =========================================================================
    # PAGES 3+: PHYSICAL HARDWARE VERIFICATION CAPTURES (ALL 10 SCREENSHOTS)
    # =========================================================================
    story.append(Paragraph("6. Physical EDGE Artix-7 100T Silicon Verification Captures", h1_style))
    story.append(Paragraph(
        "The following live terminal and IDE captures document physical execution on the EDGE Artix-7 100T (xc7a100t-2ftg256c) board over USB-UART (COM3 @ 115200 baud):",
        body_style
    ))

    captures = [
        ("m4_hardware_benchmark_capture.jpg",
         "Figure 1: Milestone 4 — 4x4 Signed INT8 Systolic Array NPU GEMM on Physical Hardware (CPU: 3482 cycles vs. NPU SoC: 43 cycles [80x] / MAC Core: 19 cycles [183x], PASS)."),
        ("m5a_tiled_8x8_hardware_capture.jpg",
         "Figure 2: Milestone 5A — 8x8 Block-Tiled NPU Matrix Multiplication (K=8, 4 Tiles, 64/64 INT32 Output Cells Verified in 224 SoC / 92 MAC Cycles, PASS)."),
        ("m5b_dma_hardware_capture.jpg",
         "Figure 3: Milestone 5B — Autonomous Burst DMA Controller & 2-Master Bus Arbiter (CPU 16-Word Copy: 331 cycles vs. DMA Burst: 64 cycles [5.17x Speedup], PASS)."),
        ("m6a_simd_hardware_capture.jpg",
         "Figure 4: Milestone 6A — 4-Lane 16-Bit Signed Packed SIMD Vector Accelerator (ADD16, SUB16, MUL16, ABS16, RELU16, MAX16 & 32-Element Dot Product = -2832 in 8 Core Cycles [93x Speedup], PASS)."),
        ("m6b_fp32_hardware_capture.jpg",
         "Figure 5: Milestone 6B — IEEE-754 Single-Precision FP32 Engine (FADD.S, FSUB.S, FMUL.S, FMADD.S, FMIN.S, FMAX.S, FABS.S, FCVT & 4-Element FMA Dot Product = 12.0 in 12 Cycles [24x Speedup], PASS)."),
        ("m7_m8_full_soc_hardware_capture.jpg",
         "Figure 6: Milestone 7 — 4-Mode ASTRA-DAAQ Runtime Power & Dependency Ablation (Mode B = 74 Wake Stalls vs. Mode D Adaptive DAAQ = 0 Wake Stalls & 2363 Energy Units Saved [+12.6%], PASS)."),
        ("m8_periph_cache_irq_hardware_capture.jpg",
         "Figure 7: Milestone 8 — Complete Peripherals, L1 I$/D$ Caches (8 Hits, 4 DMA Snoops), 64-Bit Timer, SoC Interrupts (Mask = 7), SPI DAC (2748), I2C Master (123) & Live Slide Switches (sw[0] = 1, PASS)."),
        ("sdk_app_sw32_hardware_capture.jpg",
         "Figure 8: Developer Platform Way 1 — Arduino-Style High-Level Python-to-RISC-V SDK Sketch (my_first_astra_app.py) Running NPU+DMA, SIMD, FP32 & Reading Live Switch sw[5] = 32 on Hardware."),
        ("c_sdk_hardware_capture.jpg",
         "Figure 9: Developer Platform Way 2 — Bare-Metal Embedded C Application (applications/my_first_c_app.c) Compiled via tools/build_c_app.py and Verified on Physical Silicon (FP32 FMADD.S = 50.0, PASS)."),
        ("product_os_hardware_capture.jpg",
         "Figure 10: Developer Platform Way 3 — Standalone On-Chip Product OS v1.0 (scripts/astra_x1_product_os.py) Booting from BRAM and Executing Live Interactive Serial Command [2] on Physical Silicon.")
    ]

    for idx, (fname, cap) in enumerate(captures):
        fpath = os.path.join(img_dir, fname)
        if os.path.exists(fpath):
            img = Image(fpath, width=6.6 * inch, height=3.45 * inch)
            block = KeepTogether([
                img,
                Paragraph(cap, caption_style)
            ])
            story.append(block)
            if idx % 2 == 1 and idx < len(captures) - 1:
                story.append(PageBreak())

    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"Successfully generated PDF report: {out_path}")


if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_pdf = os.path.join(base_dir, "ASTRA_X1_FINAL_PROJECT_REPORT.pdf")
    artifact_dir = r"C:\Users\skali\.gemini\antigravity\brain\5c286792-f9d5-49d3-9208-a1e454c10cff"
    build_pdf(out_pdf, artifact_dir)
