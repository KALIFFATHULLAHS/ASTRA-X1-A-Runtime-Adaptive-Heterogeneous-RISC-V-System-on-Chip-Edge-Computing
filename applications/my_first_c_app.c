/**
 * ============================================================================
 * ASTRA-X1 Bare-Metal C Application ("STM32Cube / ESP-IDF" Style)
 * File: applications/my_first_c_app.c
 *
 * How to compile & run on your EDGE Artix-7 100T FPGA board:
 *   & C:/Users/skali/AppData/Local/Microsoft/WindowsApps/python3.13.exe tools\build_c_app.py applications\my_first_c_app.c
 * ============================================================================
 */

#include "../firmware/include/astra_soc.h"
#include "../firmware/include/astra_uart.h"
#include "../firmware/include/astra_npu.h"
#include "../firmware/include/astra_dma.h"
#include "../firmware/include/astra_simd.h"
#include "../firmware/include/astra_fp32.h"
#include "../firmware/include/astra_daaq.h"
#include "../firmware/include/astra_periph_cache.h"

int main(void) {
    // 1. Configure 4-Digit 7-Segment Display to show "7777" and enable Adaptive DAAQ
    ASTRA_SEVEN_SEG_WRITE(0x7777);
    astra_daaq_set_mode(DAAQ_MODE_ADAPTIVE, 1);

    astra_uart_puts("\r\n=== HELLO FROM ASTRA-X1 BARE-METAL C SDK! ===\r\n");

    // 2. Offload 4x4 INT8 Matrix Multiply to Systolic NPU (0x4000_0000) via DMA (0x4000_1000)
    const int8_t mat_a[4][4] = {
        {  2,  1,  0, -1 },
        {  4,  3,  2,  1 },
        { -2, -1,  0,  1 },
        {  5,  4,  3,  2 }
    };
    const int8_t mat_b[4][4] = {
        {  3,  0,  1,  2 },
        {  1,  2,  0, -1 },
        {  0,  1,  2,  1 },
        { -1,  2,  1,  0 }
    };
    astra_npu_matmul_4x4(mat_a, mat_b);

    // 3. Execute 4-Lane 16-bit Signed SIMD Vector MUL16 on Slave 5 (0x4000_3000)
    astra_simd_exec_4lane(SIMD_OP_MUL16, 15, -10, 25, 8, 4, 3, -2, 5);

    // 4. Execute IEEE-754 Single-Precision FP32 FMADD.S on Slave 6 (0x4000_4000)
    //    Result = (22.5f * 2.0f) + 5.0f = 50.0f
    float fp_out = astra_fpu_fmadd(22.5f, 2.0f, 5.0f);

    // 5. Light up 0x5A on LEDs and halt cleanly with WFI
    ASTRA_LED_WRITE(0x5A);
    astra_uart_puts("PASS\r\n");
    ASTRA_WFI();
    return 0;
}
