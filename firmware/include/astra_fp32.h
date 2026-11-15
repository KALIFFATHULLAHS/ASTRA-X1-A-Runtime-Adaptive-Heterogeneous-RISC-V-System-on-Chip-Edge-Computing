#ifndef ASTRA_FP32_H
#define ASTRA_FP32_H

#include "astra_soc.h"

/* ============================================================================
 * ASTRA-X1 IEEE-754 Single-Precision FP32 Engine & Timer Driver (0x4000_4000)
 * ============================================================================ */

#define FPU_OP_ADD      0UL
#define FPU_OP_SUB      1UL
#define FPU_OP_MUL      2UL
#define FPU_OP_MADD     3UL
#define FPU_OP_MIN      4UL
#define FPU_OP_MAX      5UL
#define FPU_OP_CVT_WS   6UL
#define FPU_OP_CVT_SW   7UL
#define FPU_OP_ABS      8UL
#define FPU_OP_NEG      9UL

static inline uint32_t astra_fpu_exec_raw(uint32_t op, uint32_t a_bits, uint32_t b_bits) {
    FPU_REG_OP_A = a_bits;
    FPU_REG_OP_B = b_bits;
    FPU_REG_CTRL = ((op & 0xFUL) << 2) | 0x1UL;
    while ((FPU_REG_STATUS & 0x2UL) == 0) { }
    return FPU_REG_RESULT;
}

static inline float astra_fpu_fmadd(float a, float b, float c) {
    union { float f; uint32_t u; } ua = { a }, ub = { b }, uc = { c }, ur;
    FPU_REG_OP_A = ua.u;
    FPU_REG_OP_B = ub.u;
    FPU_REG_OP_C = uc.u;
    FPU_REG_CTRL = ((FPU_OP_MADD & 0xFUL) << 2) | 0x1UL;
    while ((FPU_REG_STATUS & 0x2UL) == 0) { }
    ur.u = FPU_REG_RESULT;
    return ur.f;
}

static inline uint64_t astra_timer_get_mtime(void) {
    uint32_t lo = FPU_MTIME_LO;
    uint32_t hi = FPU_MTIME_HI;
    return (((uint64_t)hi) << 32) | lo;
}

#endif /* ASTRA_FP32_H */
