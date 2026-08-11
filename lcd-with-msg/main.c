/*
 * E906 "LCD gradient + ARM message panel" example for the Allwinner
 * A523/T527 RISC-V co-processor (Avaota-A1).
 *
 * Cross-compiled for rv32imac/ilp32.  Drives the on-board ST7789V LCD
 * (240x135) directly from the E906 via S_SPI0: a moving white bar over an
 * animated RGB gradient, a frame counter (bottom-right), AND a text panel
 * (top-left) whose content comes from the ARM host over the shared DDR
 * mailbox.  This demonstrates ARM -> E906 messaging.
 *
 * Mailbox layout in the reserved DDR carveout (0x60000000, DA == PA):
 *   E906 -> ARM:
 *     +0x00 magic 0xE9061B0B, +0x04 version, +0x08 flags,
 *     +0x0C frame counter, +0x10 result (0 = ok), +0x14 error step
 *   ARM -> E906 (E906 polls, never blocks on it):
 *     +0x40 seq      (incremented by the ARM on every text update)
 *     +0x44 text[4][32] (4 lines x 32 ASCII chars)
 *
 * From the ARM host:
 *   sudo devmem 0x60000000      # magic   = 0xE9061B0B
 *   sudo devmem 0x6000000C      # frame counter
 *   and run mbox_info.py (included) to push the text panel every 2 s.
 *
 * The LCD is wired to the RISC-V domain:
 *   - PL8  = backlight LED (GPIO out, active high)
 *   - PL9  = LCD reset     (GPIO out, active low)
 *   - PL13 = DC            (GPIO out, 0=command 1=data)
 *   - PL10 = S-SPI0-CS0    (mux 6)
 *   - PL11 = S-SPI0-CLK    (mux 6)
 *   - PL12 = S-SPI0-MOSI   (mux 6)
 *
 * Controller: S_SPI0 @ 0x07092000 (sun6i-style):
 *   - clocks: STBY_PRCM S_SPI0_CLK_REG @ 0x07010150 (bit31 gate on, source
 *     CLK24M, N=3 -> SPI_CLK = 8 MHz) and S_SPI0_BGR_REG @ 0x0701015C
 *     (bit16 RST de-assert, bit0 GATING pass).
 *   - GCR @ 0x04: SRST=bit31 (write-1), MODE=bit1 (master), EN=bit0.
 *   - TCR @ 0x08: XCH=bit31 (write-1 start), SDM=bit13, DHB=bit8,
 *     CS_LEVEL=bit7 (1=high/idle, 0=low/selected), CS_MANUAL=bit6,
 *     SPOL=bit2 (active-low CS), CPOL/CPHA=0 (SPI mode 0).
 *   - FCR @ 0x18: TF_RST=bit31, RF_RST=bit15 (write-1, auto-clear).
 *   - ISR @ 0x14: TC=bit12 (transfer complete, write-1 clear).
 *   - FSR @ 0x1C: TF_CNT [23:16] bytes in TX FIFO (64-byte FIFO).
 *   - MBC @ 0x30 = total bursts, MTC @ 0x34 = TX count,
 *     BCC @ 0x38 STC [23:0] = single-mode TX count.
 *   - TXD FIFO @ 0x200 (byte writes).
 */

#define MBOX_BASE 0x60000000UL

#define MBOX_MAGIC   0xE9061B0BUL
#define MBOX_VERSION 0x00000001UL
#define MBOX_FLAGS   0x0000B00BUL

static volatile unsigned int *const mbox =
    (volatile unsigned int *)MBOX_BASE;

#define MBOX_ARM_SEQ (MBOX_BASE + 0x40UL)
#define MBOX_TEXT    (MBOX_BASE + 0x44UL)
#define TEXT_LINES   4
#define TEXT_LINE_LEN 32

#define S_GPIO_BASE  0x07022000UL
#define S_UART0_BASE 0x07080000UL
#define STBY_PRCM    0x07010000UL
#define S_SPI0_BASE  0x07092000UL

#define PL_CFG1       (S_GPIO_BASE + 0x04UL)
#define PL_DAT        (S_GPIO_BASE + 0x10UL)
#define PM_CFG0       (S_GPIO_BASE + 0x30UL)

#define S_SPI0_CLK_REG (STBY_PRCM + 0x150UL)
#define S_SPI0_BGR_REG (STBY_PRCM + 0x15CUL)
#define S_UART_BGR     (STBY_PRCM + 0x18CUL)

#define SPI_GCR   0x04UL
#define SPI_TCR   0x08UL
#define SPI_IER   0x10UL
#define SPI_ISR   0x14UL
#define SPI_FCR   0x18UL
#define SPI_FSR   0x1CUL
#define SPI_MBC   0x30UL
#define SPI_MTC   0x34UL
#define SPI_BCC   0x38UL
#define SPI_TXD   0x200UL

#define GCR_SRST  (1UL << 31)
#define GCR_TP    (1UL << 7)
#define GCR_MASTER (1UL << 1)
#define GCR_EN    (1UL << 0)

#define TCR_XCH   (1UL << 31)
#define TCR_SDM   (1UL << 13)
#define TCR_DHB   (1UL << 8)
#define TCR_CS_LEVEL (1UL << 7)
#define TCR_CS_MANUAL (1UL << 6)
#define TCR_SPOL  (1UL << 2)

#define ISR_TC    (1UL << 12)

#define FCR_TF_RST (1UL << 31)
#define FCR_RF_RST (1UL << 15)

#define PIN_BLK  (1UL << 8)	/* PL8 backlight */
#define PIN_RST  (1UL << 9)	/* PL9 reset */
#define PIN_DC   (1UL << 13)	/* PL13 data/command */

#define LCD_W 240
#define LCD_H 135

static unsigned char fb[LCD_W * LCD_H * 2];	/* ~65 KB in SRAM0 */

static unsigned int lcd_err_step;
static unsigned int last_isr;

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

static inline unsigned int rdtime(void)
{
	unsigned int t;

	__asm__ volatile("rdtime %0" : "=r"(t));
	return t;
}

static void delay_ms(unsigned int ms)
{
	unsigned int start = rdtime();

	while ((rdtime() - start) < ms * 24000UL)
		;
}

static void gpio_init(void)
{
	volatile unsigned int *plcfg =
	    (volatile unsigned int *)PL_CFG1;
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;

	/*
	 * PL8[3:0]=1 out, PL9[7:4]=1 out, PL10[11:8]=6 S-SPI0-CS0,
	 * PL11[15:12]=6 S-SPI0-CLK, PL12[19:16]=6 S-SPI0-MOSI,
	 * PL13[23:20]=1 out (DC).
	 */
	*plcfg = 0x00166611UL;

	/* reset low, backlight off, DC=command */
	*pldat &= ~(PIN_RST | PIN_BLK | PIN_DC);
	delay_ms(10);
	/* reset high (run), backlight on */
	*pldat = (*pldat | PIN_RST | PIN_BLK) & ~PIN_DC;
	delay_ms(150);
}

static void spi_init(void)
{
	volatile unsigned int *bgr =
	    (volatile unsigned int *)S_SPI0_BGR_REG;
	volatile unsigned int *clk =
	    (volatile unsigned int *)S_SPI0_CLK_REG;
	volatile unsigned int *gcr =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_GCR);
	volatile unsigned int *tcr =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_TCR);
	unsigned int n;

	/* de-assert reset + un-gate bus clock */
	*bgr = (1UL << 16) | (1UL << 0);
	/* module clock: gate on, source CLK24M, N=3, M=1 -> 8 MHz */
	*clk = (1UL << 31) | (2UL << 8);

	/* soft-reset the controller, wait for auto-clear */
	*gcr = GCR_SRST;
	for (n = 0; n < 200000UL && (*gcr & GCR_SRST); n++)
		;
	if (*gcr & GCR_SRST) {
		lcd_err_step = 0x12;	/* SRST never auto-cleared */
		return;
	}

	/* master, normal pause behaviour, bus enabled */
	*gcr = GCR_MASTER | GCR_TP | GCR_EN;

	/* SPI mode 0, active-low manual CS idle high, TX-only */
	*tcr = TCR_SDM | TCR_DHB | TCR_CS_LEVEL | TCR_CS_MANUAL | TCR_SPOL;
}

static void spi_cs(int on)
{
	volatile unsigned int *tcr =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_TCR);

	if (on)
		*tcr &= ~TCR_CS_LEVEL;	/* CS low = selected */
	else
		*tcr |= TCR_CS_LEVEL;	/* CS high = idle */
}

static void spi_write(const unsigned char *data, unsigned int n)
{
	volatile unsigned int *isr =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_ISR);
	volatile unsigned int *fcr =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_FCR);
	volatile unsigned int *fsr =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_FSR);
	volatile unsigned int *mbc =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_MBC);
	volatile unsigned int *mtc =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_MTC);
	volatile unsigned int *bcc =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_BCC);
	volatile unsigned int *tcr =
	    (volatile unsigned int *)(S_SPI0_BASE + SPI_TCR);
	volatile unsigned char *txd =
	    (volatile unsigned char *)(S_SPI0_BASE + SPI_TXD);
	unsigned int sent = 0;
	unsigned int timeout = 2000000UL;

	*isr = ~0UL;			/* clear stale interrupts */
	*fcr = FCR_TF_RST | FCR_RF_RST;	/* reset both FIFOs */
	while (*fcr & (FCR_TF_RST | FCR_RF_RST))
		;

	*bcc = n & 0xFFFFFFUL;		/* STC = single-mode TX count */
	*mbc = n & 0xFFFFFFUL;		/* total bursts */
	*mtc = n & 0xFFFFFFUL;		/* TX count */

	*tcr |= TCR_XCH;		/* start exchange */

	for (;;) {
		unsigned int st;

		if (!--timeout) {
			lcd_err_step = 0xEEUL;
			break;
		}
		st = *isr;
		last_isr = st;
		if (st & ISR_TC)
			break;
		/* refill TX FIFO whenever it has room */
		while (sent < n) {
			unsigned int cnt = (*fsr >> 16) & 0xFFUL;

			if (cnt >= 64)
				break;
			*txd = data[sent++];
		}
	}
	*isr = ISR_TC;
}

static void lcd_cmd(unsigned char cmd)
{
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;

	spi_cs(1);
	*pldat &= ~PIN_DC;		/* command */
	spi_write(&cmd, 1);
	spi_cs(0);
}

static void lcd_write_reg(unsigned char cmd, const unsigned char *d,
			  unsigned int n)
{
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;

	spi_cs(1);
	*pldat &= ~PIN_DC;
	spi_write(&cmd, 1);
	*pldat |= PIN_DC;
	if (n)
		spi_write(d, n);
	spi_cs(0);
}

static const unsigned char gamma_pos[15] = {
	0xD0, 0x05, 0x0A, 0x09, 0x08, 0x05, 0x2E, 0x44,
	0x45, 0x0F, 0x17, 0x16, 0x2B, 0x33
};
static const unsigned char gamma_neg[15] = {
	0xD0, 0x05, 0x0A, 0x09, 0x08, 0x05, 0x2E, 0x43,
	0x45, 0x0F, 0x16, 0x16, 0x2B, 0x33
};

static void lcd_init(void)
{
	unsigned char v;

	v = 0x68;			/* MADCTL: BGR | MV | MX (rotate 90 + 180 flip) */
	lcd_write_reg(0x36, &v, 1);

	lcd_cmd(0x11);			/* SLPOUT */
	delay_ms(120);

	v = 0x05;			/* COLMOD: RGB-565 */
	lcd_write_reg(0x3A, &v, 1);

	{
		const unsigned char d[5] = { 0x05, 0x05, 0x00, 0x33, 0x33 };
		lcd_write_reg(0xB2, d, 5);	/* PORCTRL */
	}
	{
		const unsigned char d[1] = { 0x75 };
		lcd_write_reg(0xB7, d, 1);	/* GCTRL */
	}
	{
		const unsigned char d[2] = { 0x01, 0xFF };
		lcd_write_reg(0xC2, d, 2);	/* VDVVRHEN */
	}
	{
		const unsigned char d[1] = { 0x13 };
		lcd_write_reg(0xC3, d, 1);	/* VRHS */
	}
	{
		const unsigned char d[1] = { 0x20 };
		lcd_write_reg(0xC4, d, 1);	/* VDVS */
	}
	{
		const unsigned char d[1] = { 0x22 };
		lcd_write_reg(0xBB, d, 1);	/* VCOMS */
	}
	{
		const unsigned char d[1] = { 0x20 };
		lcd_write_reg(0xC5, d, 1);	/* VCMOFSET */
	}
	{
		const unsigned char d[2] = { 0xA4, 0xA1 };
		lcd_write_reg(0xD0, d, 2);	/* PWCTRL1 */
	}
	lcd_write_reg(0xE0, gamma_pos, 14);	/* PVGAMCTRL */
	lcd_write_reg(0xE1, gamma_neg, 14);	/* NVGAMCTRL */

	lcd_cmd(0x21);			/* INVON */
	lcd_cmd(0x29);			/* DISPON */
}

static void lcd_fill(const unsigned char *buf)
{
	const unsigned char win[4] = { 0x00, 0x28, 0x01, 0x17 };	/* CASET 40..279 */
	const unsigned char row[4] = { 0x00, 0x34, 0x00, 0xBA };	/* RASET 52..186 */
	const unsigned char ramwr = 0x2C;
	volatile unsigned int *dc = (volatile unsigned int *)PL_DAT;

	lcd_write_reg(0x2A, win, 4);	/* CASET */
	lcd_write_reg(0x2B, row, 4);	/* RASET */

	/* RAMWR + pixel stream with CS held low the whole time */
	spi_cs(1);
	*dc &= ~PIN_DC;
	spi_write(&ramwr, 1);
	*dc |= PIN_DC;
	spi_write(buf, LCD_W * LCD_H * 2);
	spi_cs(0);
}

/* 5x7 font, ASCII 32..126, 5 bytes/glyph (column-major, bit0 = top row). */
static const unsigned char font5x7[95][5] = {
	{0x00,0x00,0x00,0x00,0x00},	/*   */
	{0x00,0x00,0x5F,0x00,0x00},	/* ! */
	{0x00,0x07,0x00,0x07,0x00},	/* " */
	{0x14,0x7F,0x14,0x7F,0x14},	/* # */
	{0x24,0x2A,0x7F,0x2A,0x12},	/* $ */
	{0x23,0x13,0x08,0x64,0x62},	/* % */
	{0x36,0x49,0x55,0x22,0x50},	/* & */
	{0x00,0x05,0x03,0x00,0x00},	/* ' */
	{0x00,0x1C,0x22,0x41,0x00},	/* ( */
	{0x00,0x41,0x22,0x1C,0x00},	/* ) */
	{0x14,0x08,0x3E,0x08,0x14},	/* * */
	{0x08,0x08,0x3E,0x08,0x08},	/* + */
	{0x00,0x50,0x30,0x00,0x00},	/* , */
	{0x08,0x08,0x08,0x08,0x08},	/* - */
	{0x00,0x60,0x60,0x00,0x00},	/* . */
	{0x20,0x10,0x08,0x04,0x02},	/* / */
	{0x3E,0x51,0x49,0x45,0x3E},	/* 0 */
	{0x00,0x42,0x7F,0x40,0x00},	/* 1 */
	{0x42,0x61,0x51,0x49,0x46},	/* 2 */
	{0x21,0x41,0x45,0x4B,0x31},	/* 3 */
	{0x18,0x14,0x12,0x7F,0x10},	/* 4 */
	{0x27,0x45,0x45,0x45,0x39},	/* 5 */
	{0x3C,0x4A,0x49,0x49,0x30},	/* 6 */
	{0x01,0x71,0x09,0x05,0x03},	/* 7 */
	{0x36,0x49,0x49,0x49,0x36},	/* 8 */
	{0x06,0x49,0x49,0x29,0x1E},	/* 9 */
	{0x00,0x36,0x36,0x00,0x00},	/* : */
	{0x00,0x56,0x36,0x00,0x00},	/* ; */
	{0x08,0x14,0x22,0x41,0x00},	/* < */
	{0x14,0x14,0x14,0x14,0x14},	/* = */
	{0x00,0x41,0x22,0x14,0x08},	/* > */
	{0x02,0x01,0x51,0x09,0x06},	/* ? */
	{0x32,0x49,0x79,0x41,0x3E},	/* @ */
	{0x7E,0x11,0x11,0x11,0x7E},	/* A */
	{0x7F,0x49,0x49,0x49,0x36},	/* B */
	{0x3E,0x41,0x41,0x41,0x22},	/* C */
	{0x7F,0x41,0x41,0x22,0x1C},	/* D */
	{0x7F,0x49,0x49,0x49,0x41},	/* E */
	{0x7F,0x09,0x09,0x09,0x01},	/* F */
	{0x3E,0x41,0x49,0x49,0x7A},	/* G */
	{0x7F,0x08,0x08,0x08,0x7F},	/* H */
	{0x00,0x41,0x7F,0x41,0x00},	/* I */
	{0x20,0x40,0x41,0x3F,0x01},	/* J */
	{0x7F,0x08,0x14,0x22,0x41},	/* K */
	{0x7F,0x40,0x40,0x40,0x40},	/* L */
	{0x7F,0x02,0x0C,0x02,0x7F},	/* M */
	{0x7F,0x04,0x08,0x10,0x7F},	/* N */
	{0x3E,0x41,0x41,0x41,0x3E},	/* O */
	{0x7F,0x09,0x09,0x09,0x06},	/* P */
	{0x3E,0x41,0x51,0x21,0x5E},	/* Q */
	{0x7F,0x09,0x19,0x29,0x46},	/* R */
	{0x46,0x49,0x49,0x49,0x31},	/* S */
	{0x01,0x01,0x7F,0x01,0x01},	/* T */
	{0x3F,0x40,0x40,0x40,0x3F},	/* U */
	{0x1F,0x20,0x40,0x20,0x1F},	/* V */
	{0x3F,0x40,0x38,0x40,0x3F},	/* W */
	{0x63,0x14,0x08,0x14,0x63},	/* X */
	{0x07,0x08,0x70,0x08,0x07},	/* Y */
	{0x61,0x51,0x49,0x45,0x43},	/* Z */
	{0x00,0x7F,0x41,0x41,0x00},	/* [ */
	{0x02,0x04,0x08,0x10,0x20},	/* \ */
	{0x00,0x41,0x41,0x7F,0x00},	/* ] */
	{0x04,0x02,0x01,0x02,0x04},	/* ^ */
	{0x40,0x40,0x40,0x40,0x40},	/* _ */
	{0x00,0x01,0x02,0x04,0x00},	/* ` */
	{0x20,0x54,0x54,0x54,0x78},	/* a */
	{0x7F,0x48,0x44,0x44,0x38},	/* b */
	{0x38,0x44,0x44,0x44,0x20},	/* c */
	{0x38,0x44,0x44,0x48,0x7F},	/* d */
	{0x38,0x54,0x54,0x54,0x18},	/* e */
	{0x08,0x7E,0x09,0x01,0x02},	/* f */
	{0x0C,0x52,0x52,0x52,0x3E},	/* g */
	{0x7F,0x08,0x04,0x04,0x78},	/* h */
	{0x00,0x44,0x7D,0x40,0x00},	/* i */
	{0x20,0x40,0x44,0x3D,0x00},	/* j */
	{0x7F,0x10,0x28,0x44,0x00},	/* k */
	{0x00,0x41,0x7F,0x40,0x00},	/* l */
	{0x7C,0x04,0x18,0x04,0x78},	/* m */
	{0x7C,0x08,0x04,0x04,0x78},	/* n */
	{0x38,0x44,0x44,0x44,0x38},	/* o */
	{0x7C,0x14,0x14,0x14,0x08},	/* p */
	{0x08,0x14,0x14,0x18,0x7C},	/* q */
	{0x7C,0x08,0x04,0x04,0x08},	/* r */
	{0x48,0x54,0x54,0x54,0x20},	/* s */
	{0x04,0x3F,0x44,0x40,0x20},	/* t */
	{0x3C,0x40,0x40,0x20,0x7C},	/* u */
	{0x1C,0x20,0x40,0x20,0x1C},	/* v */
	{0x3C,0x40,0x30,0x40,0x3C},	/* w */
	{0x44,0x28,0x10,0x28,0x44},	/* x */
	{0x0C,0x50,0x50,0x50,0x3C},	/* y */
	{0x44,0x64,0x54,0x4C,0x44},	/* z */
	{0x00,0x08,0x36,0x41,0x00},	/* { */
	{0x00,0x00,0x7F,0x00,0x00},	/* | */
	{0x00,0x41,0x36,0x08,0x00},	/* } */
	{0x10,0x08,0x08,0x10,0x08},	/* ~ */
};

static void fb_px(unsigned int x, unsigned int y, unsigned short c)
{
	fb[(y * LCD_W + x) * 2] = (unsigned char)(c >> 8);
	fb[(y * LCD_W + x) * 2 + 1] = (unsigned char)c;
}

#define TEXT_SCALE 2

static void draw_text(int x, int y, const char *s, unsigned int maxlen,
		      unsigned short fg, unsigned short bg, unsigned int scale)
{
	while (*s && maxlen--) {
		unsigned int g = (unsigned char)*s++;
		int col, row, sx, sy;

		if (g < 32 || g > 126)
			g = 32;
		for (col = 0; col < 5; col++) {
			unsigned char bits = font5x7[g - 32][col];

			for (row = 0; row < 7; row++) {
				unsigned short c =
				    (bits & (1 << row)) ? fg : bg;

				for (sy = 0; sy < scale; sy++)
					for (sx = 0; sx < scale; sx++)
						fb_px(x + col * scale + sx,
						      y + row * scale + sy, c);
			}
		}
		x += 6 * scale;
	}
}

static void draw_frame_counter(unsigned int frame)
{
	char s[10];
	unsigned int f = frame;
	int i;

	s[0] = 'F';
	for (i = 8; i > 0; i--) {
		s[i] = "0123456789ABCDEF"[f & 0xF];
		f >>= 4;
	}
	s[9] = 0;
	draw_text(LCD_W - 9 * 6 * TEXT_SCALE - 2, LCD_H - 7 * TEXT_SCALE - 2,
		  s, 9, 0xFFFF, 0x0000, TEXT_SCALE);
}

static void arm_read_line(unsigned int line, char *out)
{
	volatile const unsigned int *p =
	    (volatile const unsigned int *)(MBOX_TEXT + line * TEXT_LINE_LEN);
	int w;

	for (w = 0; w < TEXT_LINE_LEN / 4; w++) {
		unsigned int v = p[w];

		out[w * 4 + 0] = (char)(v & 0xFF);
		out[w * 4 + 1] = (char)((v >> 8) & 0xFF);
		out[w * 4 + 2] = (char)((v >> 16) & 0xFF);
		out[w * 4 + 3] = (char)((v >> 24) & 0xFF);
	}
}

static char panel_text[TEXT_LINES][TEXT_LINE_LEN];
static unsigned int panel_valid;
static unsigned int last_seq;

static void poll_arm_text(void)
{
	unsigned int seq = *(volatile unsigned int *)MBOX_ARM_SEQ;
	int l;

	if (seq == last_seq)
		return;
	last_seq = seq;
	for (l = 0; l < TEXT_LINES; l++)
		arm_read_line(l, panel_text[l]);
	panel_valid = 1;
}

static void draw_arm_text(void)
{
	int l;

	if (!panel_valid)
		return;
	for (l = 0; l < TEXT_LINES; l++)
		draw_text(4, 4 + l * (7 * TEXT_SCALE + 4),
			  panel_text[l], (LCD_W - 8) / (6 * TEXT_SCALE),
			  0xFFFF, 0x0000, TEXT_SCALE);
}

static void build_pattern(unsigned int frame)
{
	unsigned int x, y;
	unsigned int bar = (frame * 2) % (LCD_W + 48);

	for (y = 0; y < LCD_H; y++) {
		for (x = 0; x < LCD_W; x++) {
			unsigned int r = (x * 31) / (LCD_W - 1);
			unsigned int g = (y * 63) / (LCD_H - 1);
			unsigned int b = ((LCD_W - 1 - x) * 31) / (LCD_W - 1);
			unsigned short c =
			    (unsigned short)((r << 11) | (g << 5) | b);

			if (x >= bar && x < bar + 24)
				c = 0xFFFF;	/* moving white bar */
			else if (y >= 16 && y < LCD_H - 16 &&
				 x >= 32 && x < LCD_W - 32 &&
				 ((x / 16 + y / 16) & 1))
				c = (unsigned short)(c >> 1);	/* checkerboard */
			fb[(y * LCD_W + x) * 2] = (unsigned char)(c >> 8);
			fb[(y * LCD_W + x) * 2 + 1] = (unsigned char)c;
		}
	}
}

int main(void)
{
	unsigned int frame = 0;

	mbox[0] = MBOX_MAGIC;
	mbox[1] = MBOX_VERSION;
	mbox[2] = MBOX_FLAGS;
	mbox[3] = 0;
	mbox[4] = 0xFFFFFFFF;		/* result (0 = ok) */
	mbox[5] = 0;			/* error step */

	uart_init();
	uart_puts("\r\n[E906] ST7789V LCD gradient via S_SPI0\r\n");

	gpio_init();
	spi_init();
	if (lcd_err_step == 0)
		lcd_init();

	uart_puts("[E906] LCD initialised, drawing\r\n");

	/* don't draw the ARM panel until the first real update */
	last_seq = *(volatile unsigned int *)MBOX_ARM_SEQ;

	for (;;) {
		build_pattern(frame);

		/*
		 * Poll the ARM->E906 mailbox every 16 frames (~1 s).  The
		 * text panel is opaque so old pixels are overwritten.  If
		 * the ARM is busy, hung or panicked the seq simply stops
		 * changing and the last panel stays; the draw loop itself
		 * never depends on these reads.
		 */
		if ((frame & 15) == 0)
			poll_arm_text();

		draw_arm_text();	/* opaque panel redrawn from SRAM every frame */
		draw_frame_counter(frame);
		lcd_fill(fb);	/* send composite (pattern + text) to the LCD */

		mbox[3] = frame;
		mbox[4] = (lcd_err_step != 0) ? 0xFFFFFFFFUL : 0;
		mbox[5] = lcd_err_step;

		if (lcd_err_step != 0) {
			uart_puts("[E906] SPI error step=0x");
			uart_hex(lcd_err_step);
			uart_puts(" ISR=");
			uart_hex(last_isr);
			uart_puts("\r\n");
		}

		frame++;
		delay_ms(40);
	}
}
