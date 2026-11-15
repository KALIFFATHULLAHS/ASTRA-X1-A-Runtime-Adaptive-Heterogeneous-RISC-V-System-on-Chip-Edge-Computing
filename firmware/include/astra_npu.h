#ifndef ASTRA_NPU_H
#define ASTRA_NPU_H

#include "astra_soc.h"

/* ============================================================================
 * ASTRA-X1 4x4 INT8 Systolic Array NPU Driver (0x4000_0000)
 * ============================================================================ */

static inline int astra_npu_verify_id(void) {
    return (NPU_REG_ID == ASTRA_NPU_ID_MAGIC);
}

static inline void astra_npu_set_k_dim(uint32_t k_dim) {
    NPU_REG_K_DIM = k_dim;
}

static inline void astra_npu_start(void) {
    NPU_REG_CTRL = 0x1UL;
}

static inline void astra_npu_wait_done(void) {
    while ((NPU_REG_STATUS & 0x2UL) == 0) { }
}

static inline uint32_t astra_npu_get_cycles(void) {
    return NPU_REG_CYCLES;
}

static inline void astra_npu_read_matrix_c(int32_t out_c[16]) {
    for (int i = 0; i < 16; i++) {
        out_c[i] = (int32_t)NPU_MAT_C(i);
    }
}

static inline void astra_npu_matmul_4x4(const int8_t mat_a[4][4], const int8_t mat_b[4][4]) {
    for (int r = 0; r < 4; r++) {
        uint32_t w = ((uint32_t)(uint8_t)mat_a[r][0]) |
                     (((uint32_t)(uint8_t)mat_a[r][1]) << 8) |
                     (((uint32_t)(uint8_t)mat_a[r][2]) << 16) |
                     (((uint32_t)(uint8_t)mat_a[r][3]) << 24);
        NPU_BUF_A(r) = w;
    }
    for (int k = 0; k < 4; k++) {
        uint32_t w = ((uint32_t)(uint8_t)mat_b[k][0]) |
                     (((uint32_t)(uint8_t)mat_b[k][1]) << 8) |
                     (((uint32_t)(uint8_t)mat_b[k][2]) << 16) |
                     (((uint32_t)(uint8_t)mat_b[k][3]) << 24);
        NPU_BUF_B(k) = w;
    }
    astra_npu_set_k_dim(4);
    astra_npu_start();
    astra_npu_wait_done();
}

#endif /* ASTRA_NPU_H */
