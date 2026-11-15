#ifndef ASTRA_PERIPH_CACHE_H
#define ASTRA_PERIPH_CACHE_H

#include "astra_soc.h"

/* =========================================================================
 * ASTRA-X1 Peripheral & L1 Cache Register Offsets (Base: 0xFFFF_0000)
 * ========================================================================= */
#define PERIPH_UART_TX        (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x00))
#define PERIPH_GPIO_LED       (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x04))
#define PERIPH_UART_STATUS    (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x08))
#define PERIPH_UART_RX        (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x0C))
#define PERIPH_GPIO_SW_PB     (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x10))
#define PERIPH_GPIO_DIR       (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x14))
#define PERIPH_SEVEN_SEG      (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x18))
#define PERIPH_SPI_CTRL       (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x20))
#define PERIPH_SPI_DATA       (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x24))
#define PERIPH_I2C_CTRL       (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x30))
#define PERIPH_I2C_ADDR       (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x34))
#define PERIPH_I2C_DATA       (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x38))
#define PERIPH_ICACHE_HITS    (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x40))
#define PERIPH_ICACHE_MISSES  (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x44))
#define PERIPH_DCACHE_HITS    (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x48))
#define PERIPH_DCACHE_MISSES  (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x4C))
#define PERIPH_DCACHE_SNOOPS  (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x50))
#define PERIPH_CACHE_CTRL     (*(volatile uint32_t *)(ASTRA_PERIPH_BASE + 0x54))

static inline uint16_t astra_spi_transfer16(uint16_t tx_word, int loopback) {
    PERIPH_SPI_DATA = tx_word;
    PERIPH_SPI_CTRL = (loopback ? 2u : 0u) | 1u;
    while ((PERIPH_SPI_CTRL & 2u) == 0u) {}
    return (uint16_t)(PERIPH_SPI_DATA & 0xFFFFu);
}

static inline uint8_t astra_i2c_read_reg(uint8_t slave_addr, uint8_t reg_addr) {
    PERIPH_I2C_ADDR = ((uint32_t)reg_addr << 8) | (slave_addr & 0x7Fu);
    PERIPH_I2C_CTRL = 3u; /* Start=1, RW=1 (Read) */
    while ((PERIPH_I2C_CTRL & 2u) == 0u) {}
    return (uint8_t)(PERIPH_I2C_DATA & 0xFFu);
}

static inline void astra_l1_cache_flush(void) {
    PERIPH_CACHE_CTRL = 1u;
}

#define ASTRA_SEVEN_SEG_WRITE(val)  (PERIPH_SEVEN_SEG = (uint32_t)(val))
#define ASTRA_LED_WRITE(val)        (PERIPH_GPIO_LED  = (uint32_t)(val))
#ifdef __riscv
#define ASTRA_WFI()                 __asm__ volatile ("wfi")
#else
#define ASTRA_WFI()                 do {} while (0)
#endif

#endif /* ASTRA_PERIPH_CACHE_H */
