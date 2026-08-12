/* E906 firmware entry point.  The remoteproc driver releases reset, the
 * E906 fetches its first instruction from the start address programmed in
 * E906_STA_ADD_REG (0x204) -- the ELF entry point, i.e. _start.
 *
 * ROOT CAUSE of "data loads return 0" (A523 E906):
 *   With the D-cache DISABLED (mhcr DE=bit1 = 0, the reset default), every
 *   data load (SRAM0, DDR, even read-back of an own store) returns 0, while
 *   stores and instruction fetches work.  Enabling the D-cache (DE=1) makes
 *   loads return real data.  This was isolated by replicating the vendor
 *   firmware init (mstatus bit13, mxstatus 0x408000, mhcr 0x103F, full PMP)
 *   which fixed it, then bisecting down to the two csrsi below as the only
 *   necessary change (mhcr reads back 0x3).
 *   Note: the old crt0 wrote mcor (0x7C2) and cleared DE via csrc; MCOR is
 *   not implemented on this core (reads 0) and DE was already 0 at reset,
 *   so the old init left both caches disabled.
 */

#define MHCR 0x7C1

.section .text.entry
.align 2
.globl _start
.type _start, @function
_start:
    /* hal_icache_init/DE sequence from the vendor: enable I-cache (bit0)
     * and D-cache (bit1).  Without DE=1 all data loads return 0. */
    csrsi   MHCR, 1
    csrsi   MHCR, 2
    fence

    /* stack pointer at the top of SRAM0 (0x40000000), grows down */
    la      sp, __stack_top

    /* zero .bss */
    la      t0, __bss_start
    la      t1, __bss_end
1:
    bgeu    t0, t1, 2f
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       1b
2:
    call    main
3:
    wfi
    j       3b
.size _start, . - _start
