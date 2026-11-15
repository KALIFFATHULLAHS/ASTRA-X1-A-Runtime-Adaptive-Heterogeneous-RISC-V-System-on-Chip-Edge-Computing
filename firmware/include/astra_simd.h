#ifndef ASTRA_SIMD_H
#define ASTRA_SIMD_H

#include "astra_soc.h"

/* ============================================================================
 * ASTRA-X1 4-Lane 16-bit Packed SIMD Accelerator Driver (0x4000_3000)
 * ============================================================================ */

#define SIMD_OP_ADD16   0UL
#define SIMD_OP_SUB16   1UL
#define SIMD_OP_MUL16   2UL
#define SIMD_OP_MAC16   3UL
#define SIMD_OP_ABS16   4UL
#define SIMD_OP_RELU16  5UL
#define SIMD_OP_MAX16   6UL
#define SIMD_OP_MIN16   7UL

static inline uint32_t astra_simd_pack2_i16(int16_t lane0, int16_t lane1) {
    return (((uint32_t)(uint16_t)lane1) << 16) | ((uint32_t)(uint16_t)lane0);
}

static inline int32_t astra_simd_exec_4lane(uint32_t op, int16_t a0, int16_t a1, int16_t a2, int16_t a3,
                                            int16_t b0, int16_t b1, int16_t b2, int16_t b3) {
    SIMD_VEC_A0   = astra_simd_pack2_i16(a0, a1);
    SIMD_VEC_A1   = astra_simd_pack2_i16(a2, a3);
    SIMD_VEC_B0   = astra_simd_pack2_i16(b0, b1);
    SIMD_VEC_B1   = astra_simd_pack2_i16(b2, b3);
    SIMD_REG_CTRL = ((op & 0xFUL) << 2) | 0x3UL;
    return (int32_t)SIMD_REG_ACC;
}

static inline int32_t astra_simd_stream_vectors(uint32_t op, uint32_t num_vec4) {
    SIMD_REG_CTRL = 0x2UL; /* Clear accumulator */
    SIMD_REG_LEN  = num_vec4;
    SIMD_REG_CTRL = (1UL << 8) | ((op & 0xFUL) << 2) | 0x1UL;
    while ((SIMD_REG_STATUS & 0x2UL) == 0) { }
    return (int32_t)SIMD_REG_ACC;
}

#endif /* ASTRA_SIMD_H */
