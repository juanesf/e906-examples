/*
 * E906 "System Monitor" example for the Allwinner A523/T527 RISC-V
 * co-processor (Avaota-A1).
 *
 * Cross-compiled for rv32imac/ilp32.  The E906 drives the on-board ST7789V
 * LCD (240x135) via S_SPI0 and renders a live system-monitor dashboard fed
 * by the ARM host through a mailbox in the reserved DDR carveout
 * (0x60000000, DA == PA).
 *
 * Mailbox layout (v3, system monitor):
 *   +0x00 magic   0xE9061B0B      (written by E906)
 *   +0x04 version 3               (written by E906)
 *   +0x08 flags   0x0000B00B      (written by E906)
 *   +0x0C e906_cnt                (written by E906, frame counter)
 *   +0x10 arm_cnt                 (ARM -> E906, bumped each push)
 *   +0x14 status                  (ARM -> E906, bit0 = data valid)
 *   +0x18 load1x1000              (ARM)
 *   +0x1C load5x1000              (ARM)
 *   +0x20 load15x1000             (ARM)
 *   +0x24 mem_total_KB            (ARM)
 *   +0x28 mem_used_KB             (ARM)
 *   +0x2C temp_mC                 (ARM, millidegrees)
 *   +0x30 uptime_s                (ARM)
 *   +0x34 clock_packed            (ARM: (hh<<16)|(mm<<8)|ss)
 *   +0x38 spare
 *   +0x3C spare
 *   +0x40 arm_seq                 (ARM, bumped with the text lines)
 *   +0x44 text[4][32]             (ARM: hostname / IP / info / status)
 *
 * Pin usage (verified against the Avaota-A1 schematic/header):
 *   - LCD:  PL8 = backlight, PL9 = reset, PL13 = DC,
 *           PL10/PL11/PL12 = S-SPI0 CS0/CLK/MOSI (mux 6)
 *   - Debug: S-UART0 (PM0/PM1), Avaota header pins 37 TX / 40 RX @115200
 */

#define MBOX_BASE 0x60000000UL

#define MBOX_MAGIC   0xE9061B0BUL
#define MBOX_VERSION 0x00000003UL
#define MBOX_FLAGS   0x0000B00BUL

static volatile unsigned int *const mbox =
    (volatile unsigned int *)MBOX_BASE;

/* T-Head dcache.ipa: invalidate ONE D-cache line by physical address.  The
 * ARM writes the mailbox through an uncached /dev/mem mapping, but this
 * core's write-through D-cache can keep a stale copy of the ARM-written
 * fields (seq, metrics), so they would never refresh.  Invalidate only the
 * DDR mailbox lines -- never dcache.iall: it drops dirty SRAM0 lines and
 * crashes the core (found on the board). */
static inline void dcache_ipa(unsigned int pa)
{
	__asm__ volatile(".insn i 0x0B, 0, x0, %0, 0x02A" :: "r"(pa) : "memory");
}

/* invalidate the lines covering +0x00..+0x1F, +0x20..+0x3F and +0x40..+0x5F
 * (covers every ARM-written field regardless of the line size) */
static inline void dcache_inval_mbox(void)
{
	dcache_ipa(MBOX_BASE + 0x00UL);
	dcache_ipa(MBOX_BASE + 0x20UL);
	dcache_ipa(MBOX_BASE + 0x40UL);
}

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

#define TEXT_LINES 4
#define TEXT_LINE_LEN 32

static unsigned char fb[LCD_W * LCD_H * 2];	/* ~65 KB in SRAM0 */

static unsigned int lcd_err_step;
static unsigned int last_isr;

/* ---- palette (RGB565) ---- */
#define C_BLACK  0x0000UL
#define C_NAVY   0x0010UL	/* deep blue */
#define C_PANEL  0x0841UL	/* panel fill */
#define C_EDGE   0x3186UL	/* panel border */
#define C_TRACK  0x18C3UL	/* empty bar track */
#define C_CYAN   0x07FFUL
#define C_ORANGE 0xFB40UL
#define C_GREEN  0x07E0UL
#define C_YELLOW 0xFFE0UL
#define C_RED    0xF800UL
#define C_WHITE  0xFFFFUL
#define C_DIM    0x7BEFUL	/* dim secondary text */
#define C_BG     0x0000UL	/* main canvas background */

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
	if (x >= LCD_W || y >= LCD_H)
		return;
	fb[(y * LCD_W + x) * 2] = (unsigned char)(c >> 8);
	fb[(y * LCD_W + x) * 2 + 1] = (unsigned char)c;
}

static void fb_rect(unsigned int x, unsigned int y, unsigned int w,
		    unsigned int h, unsigned short c)
{
	unsigned int i, j;

	for (j = 0; j < h; j++)
		for (i = 0; i < w; i++)
			fb_px(x + i, y + j, c);
}

static void draw_text(int x, int y, const char *s, unsigned int maxlen,
		      unsigned short fg, unsigned short bg, unsigned int scale)
{
	while (*s && maxlen--) {
		unsigned int g = (unsigned char)*s++;
		unsigned int col, row, sx, sy;

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

static unsigned short lighter(unsigned short c)
{
	unsigned int r = (c >> 11) & 31;
	unsigned int g = (c >> 5) & 63;
	unsigned int b = c & 31;

	r += (31 - r) >> 1;
	g += (63 - g) >> 1;
	b += (31 - b) >> 1;
	return (unsigned short)((r << 11) | (g << 5) | b);
}

/* horizontal progress bar with 1px dark outline + top highlight */
static void draw_hbar(int x, int y, int w, int h, unsigned int pct,
		      unsigned short fill, unsigned short track)
{
	fb_rect(x, y, w, h, C_BLACK);
	fb_rect(x + 1, y + 1, w - 2, h - 2, track);
	if (pct > 100)
		pct = 100;
	if (pct) {
		unsigned int fw = (unsigned int)((w - 2) * pct) / 100;

		fb_rect(x + 1, y + 1, fw, h - 2, fill);
		fb_rect(x + 1, y + 1, fw, 1, lighter(fill));
	}
}

/* framed dashboard panel */
static void panel(int x, int y, int w, int h)
{
	fb_rect(x, y, w, h, C_PANEL);
	fb_rect(x, y, w, 1, C_EDGE);
	fb_rect(x, y + h - 1, w, 1, C_EDGE);
	fb_rect(x, y, 1, h, C_EDGE);
	fb_rect(x + w - 1, y, 1, h, C_EDGE);
}

/* traffic-light colour for a percentage */
static unsigned short level_col(unsigned int pct)
{
	if (pct < 50)
		return (unsigned short)C_GREEN;
	if (pct < 80)
		return (unsigned short)C_YELLOW;
	return (unsigned short)C_RED;
}

/* ---- mailbox v3 (read side) ---- */

static unsigned int last_seq, last_arm_cnt, stale_frames;
static unsigned int stats_valid;
static unsigned int load1x, load5x, load15x;
static unsigned int mem_total_kb, mem_used_kb, temp_mc, uptime_s;
static unsigned int clock_packed;
static char hostname[33], ip[33];

static void arm_read_line(unsigned int line, char *out)
{
	volatile const unsigned int *p =
	    (volatile const unsigned int *)(MBOX_BASE + 0x44UL +
					    line * TEXT_LINE_LEN);
	int w;

	for (w = 0; w < TEXT_LINE_LEN / 4; w++) {
		unsigned int v = p[w];

		out[w * 4 + 0] = (char)(v & 0xFF);
		out[w * 4 + 1] = (char)((v >> 8) & 0xFF);
		out[w * 4 + 2] = (char)((v >> 16) & 0xFF);
		out[w * 4 + 3] = (char)((v >> 24) & 0xFF);
	}
}

static void read_stats(void)
{
	unsigned int seq;

	dcache_inval_mbox();	/* drop stale ARM-written fields */
	seq = mbox[16];		/* +0x40 */

	load1x = mbox[6];		/* +0x18 */
	load5x = mbox[7];		/* +0x1C */
	load15x = mbox[8];		/* +0x20 */
	mem_total_kb = mbox[9];		/* +0x24 */
	mem_used_kb = mbox[10];		/* +0x28 */
	temp_mc = mbox[11];		/* +0x2C */
	uptime_s = mbox[12];		/* +0x30 */
	clock_packed = mbox[13];	/* +0x34 */

	if (seq != last_seq) {
		last_seq = seq;
		arm_read_line(0, hostname);
		arm_read_line(1, ip);
		stats_valid = 1;
		stale_frames = 0;
	} else if (stale_frames < 0xFFFFFFFUL) {
		stale_frames++;
	}
	last_arm_cnt = mbox[4];		/* +0x10 */
}

/* ---- formatting helpers ---- */

static void fmt_load(char *out, unsigned int x1000)
{
	unsigned int ip = x1000 / 1000;

	if (ip >= 10) {
		out[0] = '9';
		out[1] = '.';
		out[2] = '9';
		out[3] = '9';
		out[4] = 0;
	} else {
		out[0] = (char)('0' + ip);
		out[1] = '.';
		out[2] = (char)('0' + (x1000 / 100) % 10);
		out[3] = (char)('0' + (x1000 / 10) % 10);
		out[4] = 0;
	}
}

static void fmt_temp(char *out, unsigned int mc)
{
	unsigned int deg = mc / 1000;
	unsigned int dec = (mc / 100) % 10;

	if (deg >= 100) {
		out[0] = (char)('0' + deg / 100);
		out[1] = (char)('0' + (deg / 10) % 10);
		out[2] = (char)('0' + deg % 10);
		out[3] = '.';
		out[4] = (char)('0' + dec);
		out[5] = 0;
	} else {
		out[0] = (char)('0' + deg / 10);
		out[1] = (char)('0' + deg % 10);
		out[2] = '.';
		out[3] = (char)('0' + dec);
		out[4] = 0;
	}
}

static void fmt_clock(char *out, unsigned int packed)
{
	unsigned int hh = (packed >> 16) & 0xFF;
	unsigned int mm = (packed >> 8) & 0xFF;
	unsigned int ss = packed & 0xFF;

	out[0] = (char)('0' + hh / 10);
	out[1] = (char)('0' + hh % 10);
	out[2] = ':';
	out[3] = (char)('0' + mm / 10);
	out[4] = (char)('0' + mm % 10);
	out[5] = ':';
	out[6] = (char)('0' + ss / 10);
	out[7] = (char)('0' + ss % 10);
	out[8] = 0;
}

/* HH:MM:SS (8 chars, <100h) or DDd HH:MM (9 chars, >=100h). */
static unsigned int fmt_uptime(char *out, unsigned int up)
{
	unsigned int hh = up / 3600;
	unsigned int mm = (up % 3600) / 60;
	unsigned int ss = up % 60;

	if (hh < 100) {
		out[0] = (char)('0' + hh / 10);
		out[1] = (char)('0' + hh % 10);
		out[2] = ':';
		out[3] = (char)('0' + mm / 10);
		out[4] = (char)('0' + mm % 10);
		out[5] = ':';
		out[6] = (char)('0' + ss / 10);
		out[7] = (char)('0' + ss % 10);
		out[8] = 0;
		return 8;
	} else {
		unsigned int d = hh / 24;

		if (d >= 100)
			d = 99;
		out[0] = (char)('0' + d / 10);
		out[1] = (char)('0' + d % 10);
		out[2] = 'd';
		out[3] = ' ';
		out[4] = (char)('0' + (hh % 24) / 10);
		out[5] = (char)('0' + (hh % 24) % 10);
		out[6] = ':';
		out[7] = (char)('0' + mm / 10);
		out[8] = (char)('0' + mm % 10);
		out[9] = 0;
		return 9;
	}
}

static void fmt_mem(char *out, unsigned int used_kb, unsigned int tot_kb)
{
	unsigned int u = used_kb / 1024;
	unsigned int t = tot_kb / 1024;
	unsigned int t0 = t;

	if (t0 >= 10000)	/* cap at 9999 to keep it short */
		t0 = 9999;
	out[0] = (char)('0' + (u / 1000) % 10);
	out[1] = (char)('0' + (u / 100) % 10);
	out[2] = (char)('0' + (u / 10) % 10);
	out[3] = (char)('0' + u % 10);
	out[4] = '/';
	out[5] = (char)('0' + (t0 / 1000) % 10);
	out[6] = (char)('0' + (t0 / 100) % 10);
	out[7] = (char)('0' + (t0 / 10) % 10);
	out[8] = (char)('0' + t0 % 10);
	out[9] = 'M';
	out[10] = 'B';
	out[11] = 0;
}

static unsigned int text_w(const char *s, unsigned int scale)
{
	unsigned int n = 0;

	while (*s)
		n++, s++;
	return n * 6 * scale;
}

/* ---- dashboard ---- */

static void draw_header(unsigned int frame)
{
	char clk[10];

	fb_rect(0, 0, LCD_W, 14, C_NAVY);
	fb_rect(0, 14, LCD_W, 1, C_ORANGE);
	draw_text(4, 1, "ARMBIAN MONITOR", 15, C_WHITE, C_NAVY, 1);

	if (stats_valid) {
		fmt_clock(clk, clock_packed);
		draw_text(240 - 4 - text_w(clk, 2), 1, clk, 9, C_CYAN,
			  C_NAVY, 2);
	} else {
		draw_text(240 - 4 - 8 * 12, 1, "--:--:--", 8, C_DIM, C_NAVY, 2);
	}
	/* blinking status LED (between title and clock) */
	fb_rect(98, 3, 8, 8,
		(stale_frames < 8) ? C_GREEN : ((frame & 1) ? C_RED : C_BLACK));
}

static void draw_load(void)
{
	static const char *tag[3] = { "1m", "5m", "15m" };
	unsigned int v[3] = { load1x, load5x, load15x };
	int i;

	panel(2, 16, LCD_W - 4, 27);
	draw_text(6, 19, "LOAD", 4, C_CYAN, C_PANEL, 1);

	for (i = 0; i < 3; i++) {
		int y = 19 + i * 7;
		char b[8];
		unsigned int pct = v[i] * 100 / 4000;	/* full scale = 4.00 */
		unsigned short col;

		if (pct > 100)
			pct = 100;
		col = level_col(pct);
		fmt_load(b, v[i]);
		draw_text(34, y, tag[i], 2, C_DIM, C_PANEL, 1);
		draw_text(52, y, b, 5, C_WHITE, C_PANEL, 1);
		draw_hbar(84, y, 142, 7, pct, col, C_TRACK);
	}
}

static void draw_mem(void)
{
	char b[16], s[16];
	unsigned int pct;

	panel(2, 45, LCD_W - 4, 23);
	draw_text(6, 49, "MEM", 3, C_CYAN, C_PANEL, 1);

	if (stats_valid && mem_total_kb) {
		pct = mem_used_kb * 100 / mem_total_kb;
		b[0] = (char)('0' + (pct / 100) % 10);
		b[1] = (char)('0' + (pct / 10) % 10);
		b[2] = '%';
		b[3] = 0;
		draw_text(40, 47, b, 4, C_WHITE, C_PANEL, 2);
		draw_hbar(86, 50, 140, 9, pct, level_col(pct), C_TRACK);
		fmt_mem(s, mem_used_kb, mem_total_kb);
		draw_text(86, 60, s, 14, C_DIM, C_PANEL, 1);
	} else {
		draw_text(40, 50, "n/a", 3, C_DIM, C_PANEL, 2);
	}
}

static void draw_temp(void)
{
	char b[8];
	unsigned int deg;
	unsigned short col;
	unsigned int pct;

	panel(2, 70, LCD_W - 4, 18);
	draw_text(6, 74, "TEMP", 4, C_CYAN, C_PANEL, 1);

	if (stats_valid && temp_mc) {
		unsigned int i;

		deg = temp_mc / 1000;
		col = (deg < 55) ? C_GREEN : ((deg < 75) ? C_YELLOW : C_RED);
		fmt_temp(b, temp_mc);
		for (i = 0; b[i]; i++)
			;
		b[i] = 'C';		/* append unit wherever the value ends */
		b[i + 1] = 0;
		draw_text(40, 72, b, 6, col, C_PANEL, 2);
		pct = (deg * 100) / 100;	/* 100 C full scale */
		if (pct > 100)
			pct = 100;
		draw_hbar(110, 75, 96, 9, pct, col, C_TRACK);
		draw_text(212, 72, deg < 55 ? "OK" : "HOT", 3,
			  col, C_PANEL, 1);
	} else {
		draw_text(40, 74, "n/a", 3, C_DIM, C_PANEL, 2);
	}
}

static void draw_uptime(void)
{
	char b[16];

	panel(2, 90, LCD_W - 4, 16);
	draw_text(6, 93, "UPTIME", 6, C_DIM, C_PANEL, 1);

	if (stats_valid) {
		fmt_uptime(b, uptime_s);
		draw_text(LCD_W - 4 - text_w(b, 2), 91, b, 15, C_WHITE,
			  C_PANEL, 2);
	} else {
		draw_text(LCD_W - 4 - text_w("--:--:--", 2), 91, "--:--:--", 8,
			  C_DIM, C_PANEL, 2);
	}
}

static void draw_footer(void)
{
	/* Armbian mini logo badge (small, left of the text block) */
	fb_rect(6, 108, 9, 9, C_ORANGE);
	draw_text(8, 109, "A", 1, C_NAVY, C_ORANGE, 1);

	draw_text(22, 107, hostname, 24, C_WHITE, C_BG, 2);
	draw_text(22, 121, ip, 24, C_CYAN, C_BG, 2);
}

static void draw_monitor(unsigned int frame)
{
	fb_rect(0, 0, LCD_W, LCD_H, C_BG);
	draw_header(frame);
	draw_load();
	draw_mem();
	draw_temp();
	draw_uptime();
	draw_footer();
}

int main(void)
{
	unsigned int cnt = 0;

	mbox[0] = MBOX_MAGIC;
	mbox[1] = MBOX_VERSION;
	mbox[2] = MBOX_FLAGS;
	mbox[3] = 0;

	uart_init();
	uart_puts("\r\n[E906] System monitor\r\n");

	gpio_init();
	spi_init();
	if (lcd_err_step == 0)
		lcd_init();

	last_seq = mbox[16];
	last_arm_cnt = mbox[4];

	for (;;) {
		cnt++;
		read_stats();
		draw_monitor(cnt);
		lcd_fill(fb);
		mbox[3] = cnt;
		delay_ms(250);
	}
}
