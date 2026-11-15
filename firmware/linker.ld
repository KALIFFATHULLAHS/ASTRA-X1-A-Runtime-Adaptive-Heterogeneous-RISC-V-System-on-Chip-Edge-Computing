/* ============================================================================
 * ASTRA-X1 Heterogeneous RISC-V SoC Bare-Metal Linker Script (linker.ld)
 * Memory Layout:
 *   - IMEM (Instruction BRAM) : 0x0000_0000 .. 0x0000_7FFF (32 KB)
 *   - DMEM (Data SRAM)        : 0x2000_0000 .. 0x2000_3FFF (16 KB)
 * ============================================================================ */

OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY
{
    IMEM (rx)  : ORIGIN = 0x00000000, LENGTH = 32K
    DMEM (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

SECTIONS
{
    .text : {
        KEEP(*(.init))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > IMEM

    .data : {
        __data_start = .;
        *(.data*)
        *(.sdata*)
        . = ALIGN(4);
        __data_end = .;
    } > DMEM AT > IMEM

    .bss : {
        __bss_start = .;
        *(.bss*)
        *(.sbss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > DMEM

    _stack_top = ORIGIN(DMEM) + LENGTH(DMEM);
}
