#ifndef ASTRA_SOC_H
#define ASTRA_SOC_H

#include <stdint.h>

/* ============================================================================
 * ASTRA-X1 Heterogeneous RISC-V SoC — Master Hardware Address & Register SDK
 * Target: EDGE Artix-7 XC7A100T-2FTG256C @ 50 MHz (32-bit Wishbone B4 Bus)
 * ============================================================================ */

#define ASTRA_MMIO32(addr)        (*(volatile uint32_t *)(uintptr_t)(addr))

/* 1. Physical Memory & Accelerator Base Addresses */
#define ASTRA_IMEM_BASE           0x00000000UL
#define ASTRA_SRAM_BASE           0x20000000UL
#define ASTRA_NPU_BASE            0x40000000UL  /* Slave 1: 4x4 INT8 Systolic NPU ("ASTR")  */
#define ASTRA_DMA_BASE            0x40001000UL  /* Slave 3: Autonomous Block DMA ("DMA1")   */
#define ASTRA_DAAQ_BASE           0x40002000UL  /* Slave 4: ASTRA-DAAQ Power Ctrl ("DAAQ")  */
#define ASTRA_SIMD_BASE           0x40003000UL  /* Slave 5: 4-Lane INT16 SIMD Unit ("SIMD") */
#define ASTRA_FPU_BASE            0x40004000UL  /* Slave 6: IEEE-754 FP32 & Timer ("FPU1")  */
#define ASTRA_PERIPH_BASE         0xFFFF0000UL  /* Slave 2: UART TX/RX & GPIO LEDs          */

/* 2. Expected Hardware Identification Signatures */
#define ASTRA_NPU_ID_MAGIC        0x41535452UL  /* "ASTR" */
#define ASTRA_DMA_ID_MAGIC        0x444D4131UL  /* "DMA1" */
#define ASTRA_DAAQ_ID_MAGIC       0x44414151UL  /* "DAAQ" */
#define ASTRA_SIMD_ID_MAGIC       0x53494D44UL  /* "SIMD" */
#define ASTRA_FPU_ID_MAGIC        0x46505531UL  /* "FPU1" */

/* 3. Slave 1: ASTRA-X1 4x4 INT8 Systolic NPU Registers (0x4000_0000) */
#define NPU_REG_ID                ASTRA_MMIO32(ASTRA_NPU_BASE + 0x000UL)
#define NPU_REG_STATUS            ASTRA_MMIO32(ASTRA_NPU_BASE + 0x004UL)
#define NPU_REG_SCRATCH           ASTRA_MMIO32(ASTRA_NPU_BASE + 0x008UL)
#define NPU_REG_CTRL              ASTRA_MMIO32(ASTRA_NPU_BASE + 0x00CUL)
#define NPU_REG_K_DIM             ASTRA_MMIO32(ASTRA_NPU_BASE + 0x010UL)
#define NPU_REG_CYCLES            ASTRA_MMIO32(ASTRA_NPU_BASE + 0x014UL)
#define NPU_MAT_C(idx)            ASTRA_MMIO32(ASTRA_NPU_BASE + 0x040UL + ((idx) << 2))
#define NPU_BUF_A_WORD(idx)       ASTRA_MMIO32(ASTRA_NPU_BASE + 0x100UL + ((idx) << 2))
#define NPU_BUF_B_WORD(idx)       ASTRA_MMIO32(ASTRA_NPU_BASE + 0x200UL + ((idx) << 2))
#define NPU_BUF_A(idx)            NPU_BUF_A_WORD(idx)
#define NPU_BUF_B(idx)            NPU_BUF_B_WORD(idx)

/* 4. Slave 3: Autonomous Wishbone B4 DMA Controller Registers (0x4000_1000) */
#define DMA_REG_ID                ASTRA_MMIO32(ASTRA_DMA_BASE + 0x000UL)
#define DMA_REG_STATUS            ASTRA_MMIO32(ASTRA_DMA_BASE + 0x004UL)
#define DMA_REG_SRC_ADDR          ASTRA_MMIO32(ASTRA_DMA_BASE + 0x008UL)
#define DMA_REG_DST_ADDR          ASTRA_MMIO32(ASTRA_DMA_BASE + 0x00CUL)
#define DMA_REG_LEN_WORDS         ASTRA_MMIO32(ASTRA_DMA_BASE + 0x010UL)
#define DMA_REG_CTRL              ASTRA_MMIO32(ASTRA_DMA_BASE + 0x014UL)
#define DMA_REG_CYCLES            ASTRA_MMIO32(ASTRA_DMA_BASE + 0x018UL)
#define DMA_REG_WORDS_DONE        ASTRA_MMIO32(ASTRA_DMA_BASE + 0x01CUL)

/* 5. Slave 4: ASTRA-DAAQ Power & Activity Controller Registers (0x4000_2000) */
#define DAAQ_REG_ID               ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x000UL)
#define DAAQ_REG_MODE             ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x004UL)
#define DAAQ_REG_PHASE            ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x008UL)
#define DAAQ_REG_THRESH           ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x00CUL)
#define DAAQ_REG_STATUS           ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x010UL)
#define DAAQ_REG_TOTAL_CYC        ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x014UL)
#define DAAQ_REG_GATED_NPU        ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x018UL)
#define DAAQ_REG_GATED_DMA        ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x01CUL)
#define DAAQ_REG_GATED_SIMD       ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x020UL)
#define DAAQ_REG_GATED_FPU        ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x024UL)
#define DAAQ_REG_ENERGY_SAVED     ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x028UL)
#define DAAQ_REG_WAKE_STALLS      ASTRA_MMIO32(ASTRA_DAAQ_BASE + 0x02CUL)

/* 6. Slave 5: 4-Lane 16-bit Packed SIMD Vector Engine Registers (0x4000_3000) */
#define SIMD_REG_ID               ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x000UL)
#define SIMD_REG_STATUS           ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x004UL)
#define SIMD_REG_CTRL             ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x008UL)
#define SIMD_REG_LEN              ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x00CUL)
#define SIMD_VEC_A0               ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x010UL)
#define SIMD_VEC_A1               ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x014UL)
#define SIMD_VEC_B0               ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x018UL)
#define SIMD_VEC_B1               ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x01CUL)
#define SIMD_RES_Y0               ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x020UL)
#define SIMD_RES_Y1               ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x024UL)
#define SIMD_REG_ACC              ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x028UL)
#define SIMD_REG_CYCLES           ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x02CUL)
#define SIMD_BUF_A(idx)           ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x100UL + ((idx) << 2))
#define SIMD_BUF_B(idx)           ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x200UL + ((idx) << 2))
#define SIMD_BUF_Y(idx)           ASTRA_MMIO32(ASTRA_SIMD_BASE + 0x300UL + ((idx) << 2))

/* 7. Slave 6: IEEE-754 Single-Precision FP32 Engine, 64-Bit Timer & Interrupt Controller (0x4000_4000) */
#define FPU_REG_ID                ASTRA_MMIO32(ASTRA_FPU_BASE + 0x000UL)
#define FPU_REG_STATUS            ASTRA_MMIO32(ASTRA_FPU_BASE + 0x004UL)
#define FPU_REG_OP_A              ASTRA_MMIO32(ASTRA_FPU_BASE + 0x008UL)
#define FPU_REG_OP_B              ASTRA_MMIO32(ASTRA_FPU_BASE + 0x00CUL)
#define FPU_REG_OP_C              ASTRA_MMIO32(ASTRA_FPU_BASE + 0x010UL)
#define FPU_REG_CTRL              ASTRA_MMIO32(ASTRA_FPU_BASE + 0x014UL)
#define FPU_REG_RESULT            ASTRA_MMIO32(ASTRA_FPU_BASE + 0x018UL)
#define FPU_REG_CYCLES            ASTRA_MMIO32(ASTRA_FPU_BASE + 0x01CUL)
#define FPU_MTIME_LO              ASTRA_MMIO32(ASTRA_FPU_BASE + 0x020UL)
#define FPU_MTIME_HI              ASTRA_MMIO32(ASTRA_FPU_BASE + 0x024UL)
#define FPU_MTIMECMP_LO           ASTRA_MMIO32(ASTRA_FPU_BASE + 0x028UL)
#define FPU_MTIMECMP_HI           ASTRA_MMIO32(ASTRA_FPU_BASE + 0x02CUL)
#define SOC_IRQ_PENDING           ASTRA_MMIO32(ASTRA_FPU_BASE + 0x030UL)
#define SOC_IRQ_ENABLE            ASTRA_MMIO32(ASTRA_FPU_BASE + 0x034UL)
#define SOC_IRQ_CLEAR             ASTRA_MMIO32(ASTRA_FPU_BASE + 0x038UL)
#define SOC_IRQ_COUNT             ASTRA_MMIO32(ASTRA_FPU_BASE + 0x03CUL)
#define FPU_RF(idx)               ASTRA_MMIO32(ASTRA_FPU_BASE + 0x040UL + ((idx) << 2))

/* 8. Slave 2: Peripherals — UART TX/RX & GPIO LEDs (0xFFFF_0000) */
#define PERIPH_UART_TX            ASTRA_MMIO32(ASTRA_PERIPH_BASE + 0x000UL)
#define PERIPH_GPIO_LEDS          ASTRA_MMIO32(ASTRA_PERIPH_BASE + 0x004UL)
#define PERIPH_UART_STATUS        ASTRA_MMIO32(ASTRA_PERIPH_BASE + 0x008UL)
#define PERIPH_UART_RX_DATA       ASTRA_MMIO32(ASTRA_PERIPH_BASE + 0x00CUL)

#endif /* ASTRA_SOC_H */
