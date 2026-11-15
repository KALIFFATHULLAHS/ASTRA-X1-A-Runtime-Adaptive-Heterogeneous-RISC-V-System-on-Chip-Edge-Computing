#ifndef ASTRA_UART_H
#define ASTRA_UART_H

#include "astra_soc.h"

/* ============================================================================
 * ASTRA-X1 UART TX/RX & GPIO LED Driver (0xFFFF_0000)
 * ============================================================================ */

static inline void astra_gpio_set_leds(uint8_t mask) {
    PERIPH_GPIO_LEDS = (uint32_t)mask;
}

static inline void astra_uart_putc(char c) {
    while (PERIPH_UART_STATUS & 0x1UL) { }
    PERIPH_UART_TX = (uint32_t)(uint8_t)c;
}

static inline void astra_uart_puts(const char *s) {
    while (*s) {
        astra_uart_putc(*s++);
    }
}

static inline int astra_uart_rx_ready(void) {
    return (PERIPH_UART_STATUS & 0x2UL) != 0;
}

static inline uint8_t astra_uart_getc_blocking(void) {
    while (!astra_uart_rx_ready()) { }
    return (uint8_t)(ASTRA_MMIO32(ASTRA_PERIPH_BASE + 0x00CUL) & 0xFFUL);
}

#endif /* ASTRA_UART_H */
