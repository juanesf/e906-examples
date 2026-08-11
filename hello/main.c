/*
 * E906 "hello world" example for the Allwinner A523/T527 RISC-V
 * co-processor (Avaota-A1).
 *
 * Cross-compiled for rv32imac/ilp32.  Booted by the ARM-side remoteproc
 * driver (sun55i_e906_rproc) from the reserved DDR carveout.
 *
 * The firmware writes a mailbox into the reserved E906 DDR region
 * (physical 0x60000000, DA == PA) and bumps a counter forever.  From the
 * ARM Linux side you can watch it with:
 *
 *   sudo devmem 0x60000000      # magic    = 0xE9061B0B
 *   sudo devmem 0x60000004      # version  = 0x00000001
 *   sudo devmem 0x60000008      # flags    = 0x0000B00B
 *   sudo devmem 0x6000000C      # counter  (keeps incrementing while E906 runs)
 *   sudo devmem 0x60000040 16   # "HELLO WORLD FROM E906 CO-PROCESSOR!"
 *
 * It also prints the message over S-UART0 @ 115200 8N1 (Avaota-A1 40-pin
 * header, pin 37 = TX / SoC PM0).
 */

#define MBOX_BASE   0x60000000UL
#define MBOX_MAGIC  0xE9061B0BUL
#define MBOX_VERSION 0x00000001UL
#define MBOX_FLAGS  0x0000B00BUL

static volatile unsigned int *const mbox =
    (volatile unsigned int *)MBOX_BASE;

#define S_GPIO_BASE  0x07022000UL
#define S_UART0_BASE 0x07080000UL
#define STBY_PRCM    0x07010000UL

#define PM_CFG0      (S_GPIO_BASE + 0x30UL)
#define S_UART_BGR   (STBY_PRCM + 0x18CUL)

static void uart_init(void)
{
	volatile unsigned int *lcr =
	    (volatile unsigned int *)(S_UART0_BASE + 0x0CUL);
	volatile unsigned int *bgr =
	    (volatile unsigned int *)S_UART_BGR;

	*bgr |= (1UL << 16) | (1UL << 0);	/* de-assert reset, gate on */

	*(volatile unsigned int *)PM_CFG0 = 0x22UL;	/* PM0 = S-UART0-TX */

	*lcr = 0x80 | 0x03;
	*(volatile unsigned int *)(S_UART0_BASE + 0x00UL) = 13;
	*(volatile unsigned int *)(S_UART0_BASE + 0x04UL) = 0;
	*lcr = 0x03;

	*(volatile unsigned int *)(S_UART0_BASE + 0x08UL) = 0x07;
}

static void uart_putc(char c)
{
	volatile unsigned int *usr =
	    (volatile unsigned int *)(S_UART0_BASE + 0x7CUL);
	volatile unsigned int *thr =
	    (volatile unsigned int *)(S_UART0_BASE + 0x00UL);

	while (!(*usr & 0x2UL))
		;
	*thr = (unsigned int)c;
}

static void uart_puts(const char *s)
{
	while (*s)
		uart_putc(*s++);
}

static void uart_hex(unsigned int v)
{
	const char *hex = "0123456789ABCDEF";
	int i;

	for (i = 7; i >= 0; i--)
		uart_putc(hex[(v >> (4 * i)) & 0xF]);
}

static void delay_loop(unsigned int n)
{
	while (n--)
		__asm__ volatile("nop");
}

int main(void)
{
	/* "HELLO WORLD FROM E906 CO-PROCESSOR!" as 4-byte words at +0x40 */
	static const char msg[] = "HELLO WORLD FROM E906 CO-PROCESSOR!";
	unsigned int counter = 0;
	unsigned int i;

	mbox[0] = MBOX_MAGIC;
	mbox[1] = MBOX_VERSION;
	mbox[2] = MBOX_FLAGS;
	mbox[3] = 0;

	for (i = 0; i < (sizeof(msg) + 3) / 4; i++) {
		unsigned int w = 0;
		unsigned int j;

		for (j = 0; j < 4; j++) {
			unsigned int k = i * 4 + j;

			if (k < sizeof(msg) - 1)
				w |= (unsigned int)(unsigned char)msg[k]
				     << (8 * j);
		}
		mbox[16 + i] = w;
	}

	uart_init();
	uart_puts("\r\n[E906] HELLO WORLD FROM E906 CO-PROCESSOR!\r\n");

	for (;;) {
		mbox[3] = counter++;
		delay_loop(2400000UL);
	}
	/* unreachable */
}
