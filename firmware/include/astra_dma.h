#ifndef ASTRA_DMA_H
#define ASTRA_DMA_H

#include "astra_soc.h"

/* ============================================================================
 * ASTRA-X1 Autonomous Wishbone B4 DMA Controller Driver (0x4000_1000)
 * ============================================================================ */

static inline int astra_dma_verify_id(void) {
    return (DMA_REG_ID == ASTRA_DMA_ID_MAGIC);
}

static inline void astra_dma_transfer_async(uint32_t src_addr, uint32_t dst_addr, uint32_t len_words) {
    DMA_REG_SRC_ADDR  = src_addr;
    DMA_REG_DST_ADDR  = dst_addr;
    DMA_REG_LEN_WORDS = len_words;
    DMA_REG_CTRL      = 0x1UL;
}

static inline void astra_dma_wait_done(void) {
    while ((DMA_REG_STATUS & 0x2UL) == 0) { }
}

static inline uint32_t astra_dma_transfer_blocking(uint32_t src_addr, uint32_t dst_addr, uint32_t len_words) {
    astra_dma_transfer_async(src_addr, dst_addr, len_words);
    astra_dma_wait_done();
    return DMA_REG_CYCLES;
}

#endif /* ASTRA_DMA_H */
