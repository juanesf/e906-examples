/* E906 firmware linker script for Allwinner A523/T527 (Avaota-A1).
 *
 * The remoteproc driver (sun55i_e906_rproc) loads ELF segments whose
 * p_paddr lies inside one of the reserved memory regions (carveouts)
 * exposed in the DT:
 *
 *   carveout        physical       device addr (DA)     size
 *   riscvsram0     0x07280000  ->  0x3ffc0000           0x40000
 *   riscvsram1     0x072c0000  ->  0x40000000           0x40000
 *   vdev0buffer    0x4ae00000  ->  0x4ae00000           0x40000
 *   vdev0vring0    0x4ae40000  ->  0x4ae40000           0x2000
 *   vdev0vring1    0x4ae42000  ->  0x4ae42000           0x2000
 *   e906_dram      0x60000000  ->  0x60000000           0xa00000
 *
 * The E906 boots from the address written to E906_STA_ADD_REG (0x204 of
 * the e906-cfg block), taken from the ELF entry point.  We link at DA
 * 0x3ffc0000 (SRAM0) and place the stack at the top of SRAM0.  A mailbox
 * is mapped at DDR 0x60000000 (DA == PA) so the ARM host can observe it
 * with devmem.
 */

ENTRY(_start)

MEMORY
{
    SRAM0 (RWX) : ORIGIN = 0x3ffc0000, LENGTH = 0x00040000
}

SECTIONS
{
    .text :
    {
        KEEP(*(.text.entry))
        *(.text*)
        *(.text.*)
        . = ALIGN(16);
        __etext = .;
    } > SRAM0

    .rodata :
    {
        *(.srodata*)
        *(.rodata*)
        . = ALIGN(16);
    } > SRAM0

    .data :
    {
        *(.sdata*)
        *(.data*)
        . = ALIGN(16);
        __edata = .;
    } > SRAM0

    .bss :
    {
        . = ALIGN(16);
        __bss_start = .;
        *(.sbss*)
        *(.bss*)
        *(COMMON)
        . = ALIGN(16);
        __bss_end = .;
    } > SRAM0

    __stack_top = ORIGIN(SRAM0) + LENGTH(SRAM0);
}
