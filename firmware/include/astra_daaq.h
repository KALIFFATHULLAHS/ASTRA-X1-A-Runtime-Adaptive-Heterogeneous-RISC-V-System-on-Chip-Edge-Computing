#ifndef ASTRA_DAAQ_H
#define ASTRA_DAAQ_H

#include "astra_soc.h"

/* ============================================================================
 * ASTRA-DAAQ Runtime Dependency-Aware Power & Activity Controller (0x4000_2000)
 * ============================================================================ */

#define DAAQ_MODE_A_BASELINE    0UL
#define DAAQ_MODE_B_LOCAL       1UL
#define DAAQ_MODE_C_FIXED       2UL
#define DAAQ_MODE_D_ADAPTIVE    3UL
#define DAAQ_MODE_ADAPTIVE      3UL

#define DAAQ_PHASE_GENERAL      0UL
#define DAAQ_PHASE_DMA_XFER     1UL
#define DAAQ_PHASE_SIMD_PREPROC 2UL
#define DAAQ_PHASE_NPU_GEMM     3UL
#define DAAQ_PHASE_FPU_POST     4UL
#define DAAQ_PHASE_IDLE         5UL

static inline void astra_daaq_set_mode(uint32_t mode, int reset_counters) {
    DAAQ_REG_MODE = (mode & 0x3UL) | (reset_counters ? 0x10UL : 0x0UL);
}

static inline void astra_daaq_set_phase_hint(uint32_t phase) {
    DAAQ_REG_PHASE = phase & 0x7UL;
}

#endif /* ASTRA_DAAQ_H */
