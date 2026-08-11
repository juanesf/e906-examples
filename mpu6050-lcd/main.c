/*
 * E906 "MPU6050 + LCD" example for the Allwinner A523/T527 RISC-V
 * co-processor (Avaota-A1).
 *
 * Cross-compiled for rv32imac/ilp32.  The E906 reads an MPU6050 IMU over
 * S_TWI1 (acting as I2C master, no ARM help) and shows the live accel /
 * gyro / temperature on the on-board ST7789V LCD (240x135) driven via
 * S_SPI0.  The same values are published through a mailbox in the reserved
 * DDR carveout (0x60000000, DA == PA) so the ARM host can consume them.
 *
 * Mailbox layout (v2, MPU6050):
 *   +0x00 magic 0xE9061B0B
 *   +0x04 version 2
 *   +0x08 flags  0x0000B00B
 *   +0x0C counter (sample count)
 *   +0x10 result (0 = ok, else error)
 *   +0x14 last error step
 *   +0x18 mpu_addr used (0x68 / 0x69 / 0xFF = none)
 *   +0x1C WHO_AM_I read back
 *   +0x20 AX  (int16, sign-extended)
 *   +0x24 AY
 *   +0x28 AZ
 *   +0x2C GX
 *   +0x30 GY
 *   +0x34 GZ
 *   +0x38 TEMP raw (int16)
 *   +0x3C TEMP_C x100 (int32)
 *   +0x40 ARM_SEQ  (ARM -> E906, increment = panel update)
 *   +0x44 text[4][32] (ARM -> E906, 4 x 32 ASCII)
 *
 * Pin usage (verified against the Avaota-A1 schematic/header):
 *   - LCD:  PL8 = backlight, PL9 = reset, PL13 = DC,
 *           PL10/PL11/PL12 = S-SPI0 CS0/CLK/MOSI (mux 6)
 *   - IMU:  S_TWI1 on the P-port header: pin 28 = SCK (PM2),
 *           pin 29 = SDA (PM3)  -> PM_CFG0 = 0x2222
 *           (PM0 = S-UART0-TX also set by this value)
 *   - Debug: S-UART0 (PM0/PM1), Avaota header pins 37 TX / 40 RX @115200
 *
 * S_TWI1 @ 0x07081800 recipe (cross-checked with vendor amp_rv0.bin):
 *   - gate/reset in STBY_PRCM S_TWI_BGR @ 0x0701019C: bit17 RST de-assert,
 *     bit1 GATING pass
 *   - classic master regs: 0x00 ADDR, 0x08 DATA, 0x0C CNTR, 0x10 STAT,
 *     0x14 CCR, 0x18 SRST
 *   - CCR for 100 kHz: Fscl = Fin/(2^CLK_N*(CLK_M+1)*10), Fin=24MHz,
 *     CLK_N=1, CLK_M=11 -> (11<<3)|1 = 0x59
 *   - STAT codes: 0x08 START, 0x18 addr+W ACK, 0x28 data Tx ACK,
 *     0x40 addr+R ACK, 0x50 data Rx ACK, 0x58 data Rx NACK
 */

#define MBOX_BASE 0x60000000UL

#define MBOX_MAGIC   0xE9061B0BUL
#define MBOX_VERSION 0x00000002UL
#define MBOX_FLAGS   0x0000B00BUL

#define MBOX_ARM_SEQ (MBOX_BASE + 0x40UL)
#define MBOX_TEXT    (MBOX_BASE + 0x44UL)

static volatile unsigned int *const mbox =
    (volatile unsigned int *)MBOX_BASE;

#define S_GPIO_BASE  0x07022000UL
#define S_UART0_BASE 0x07080000UL
#define STBY_PRCM    0x07010000UL
#define S_SPI0_BASE  0x07092000UL
#define S_TWI1_BASE  0x07081800UL

#define PL_CFG1       (S_GPIO_BASE + 0x04UL)
#define PL_DAT        (S_GPIO_BASE + 0x10UL)
#define PM_CFG0       (S_GPIO_BASE + 0x30UL)
#define PM_PUL0       (S_GPIO_BASE + 0x54UL)

#define S_SPI0_CLK_REG (STBY_PRCM + 0x150UL)
#define S_SPI0_BGR_REG (STBY_PRCM + 0x15CUL)
#define S_UART_BGR     (STBY_PRCM + 0x18CUL)
#define S_TWI_BGR      (STBY_PRCM + 0x19CUL)

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

#define TWI_DATA   0x00UL
#define TWI_CNTR   0x0CUL
#define TWI_STAT   0x10UL
#define TWI_CCR    0x14UL
#define TWI_SRST   0x18UL

#define TW_INT_EN  0x80UL
#define TW_BUS_EN  0x40UL
#define TW_M_STA   0x20UL
#define TW_M_STP   0x10UL
#define TW_INT_FLG 0x08UL
#define TW_A_ACK   0x04UL

#define MPU_ADDR0   0x68UL
#define MPU_ADDR1   0x69UL
#define MPU_PWR1    0x6BUL
#define MPU_WHO     0x75UL
#define MPU_ACCEL   0x3BUL
#define MPU_TEMP    0x41UL
#define MPU_GYRO    0x43UL

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
static unsigned int last_stat;
static unsigned int last_step;
static unsigned char mpu_addr = MPU_ADDR0;

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

static void uart_dec(int v)
{
	char buf[12];
	int i = 0, neg = 0;

	if (v < 0) {
		neg = 1;
		v = -v;
	}
	do {
		buf[i++] = '0' + (char)(v % 10);
		v /= 10;
	} while (v);
	if (neg)
		buf[i++] = '-';
	while (i)
		uart_putc(buf[--i]);
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

/* fixed-width signed decimal: sign + up to 6 digits + space */
static void fmt_sdec(char *out, int v)
{
	int i, len;

	*out++ = (v < 0) ? '-' : '+';
	if (v < 0)
		v = -v;
	len = 1;
	{
		int t = v;

		while (t >= 10) {
			t /= 10;
			len++;
		}
	}
	for (i = 6 - len; i > 0; i--)
		*out++ = ' ';
	i = 0;
	do {
		out[5 - i] = (char)('0' + (v % 10));
		v /= 10;
		i++;
	} while (v);
	out += 6;
	*out = 0;
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

/* ---- S_TWI1 (I2C) master ---- */

static volatile unsigned int *twi_reg(unsigned int off)
{
	return (volatile unsigned int *)(S_TWI1_BASE + off);
}

static void twi_init(void)
{
	volatile unsigned int *bgr =
	    (volatile unsigned int *)S_TWI_BGR;

	/* S_TWI1 gated/reset by default: bit17 RST de-assert, bit1 GATING */
	*bgr |= (1UL << 17) | (1UL << 1);

	/* PM2/PM3 -> S-TWI1-SCK/SDA (mux 2), PM0 -> S-UART0-TX: 0x2222 */
	*(volatile unsigned int *)PM_CFG0 = 0x2222UL;
	*(volatile unsigned int *)PM_PUL0 = 0x50UL;	/* weak pull-ups */

	*twi_reg(TWI_SRST) = 1;		/* write-1 soft reset, auto-clears */
	while (*twi_reg(TWI_SRST) & 1)
		;
	*twi_reg(TWI_CCR) = 0x59UL;	/* 100 kHz from 24 MHz */
	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN;
}

static unsigned int twi_wait(void)
{
	unsigned int n, st;

	for (n = 0; n < 2000000UL; n++) {
		if (*twi_reg(TWI_CNTR) & TW_INT_FLG)
			break;
	}
	st = *twi_reg(TWI_STAT) & 0xFFUL;
	last_stat = st;
	return st;
}

static int twi_byte(unsigned char b)
{
	*twi_reg(TWI_DATA) = b;
	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN | TW_INT_FLG;

	if (twi_wait() == 0xFFUL)
		last_step = 9;
	return 0;
}

static int twi_start(void)
{
	unsigned int st;

	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN | TW_INT_FLG;	/* clear stale flag */
	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN | TW_M_STA;	/* request START */
	st = twi_wait();
	if (st != 0x08UL) {
		last_step = 1;
		return -1;
	}
	twi_byte((unsigned int)(mpu_addr << 1) | 0);
	return 0;
}

static int twi_rep_start(void)
{
	unsigned int st;

	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN | TW_INT_FLG;	/* clear stale flag */
	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN | TW_M_STA;	/* repeated START */
	st = twi_wait();
	if (st != 0x10UL && st != 0x08UL) {
		last_step = 1;
		return -1;
	}
	twi_byte((unsigned int)(mpu_addr << 1) | 1);
	return 0;
}

static int twi_read(unsigned char *out, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		int last = (i == n - 1);
		unsigned int st;
		unsigned int cntr = TW_INT_EN | TW_BUS_EN | TW_INT_FLG;

		if (!last)
			cntr |= TW_A_ACK;

		*twi_reg(TWI_CNTR) = cntr;
		st = twi_wait();
		if ((!last && st != 0x50UL) || (last && st != 0x58UL)) {
			last_step = 4;
			return -1;
		}

		out[i] = (unsigned char)*twi_reg(TWI_DATA);
	}

	return 0;
}

static void twi_stop(void)
{
	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN | TW_M_STP | TW_INT_FLG;
	twi_wait();
	*twi_reg(TWI_CNTR) = TW_INT_EN | TW_BUS_EN;
}

static int mpu_write_reg(unsigned char reg, unsigned char val)
{
	unsigned int st;

	if (twi_start() != 0) {
		last_step = 1;
		return -1;
	}
	if ((st = twi_wait()) != 0x18UL) {
		last_step = 2;
		return -1;
	}
	twi_byte(reg);
	if ((st = twi_wait()) != 0x28UL) {
		last_step = 3;
		return -1;
	}
	twi_byte(val);
	if ((st = twi_wait()) != 0x28UL) {
		last_step = 4;
		return -1;
	}
	twi_stop();
	return 0;
}

static int mpu_read_regs(unsigned char reg, unsigned char *out, int n)
{
	unsigned int st;

	if (twi_start() != 0) {
		last_step = 1;
		return -1;
	}
	if ((st = twi_wait()) != 0x18UL) {
		last_step = 2;
		return -1;
	}
	twi_byte(reg);
	if ((st = twi_wait()) != 0x28UL) {
		last_step = 3;
		return -1;
	}
	if (twi_rep_start() != 0) {
		last_step = 4;
		return -1;
	}
	if ((st = twi_wait()) != 0x40UL) {
		last_step = 5;
		return -1;
	}
	if (twi_read(out, n) != 0) {
		last_step = 6;
		return -1;
	}
	twi_stop();
	return 0;
}

/* ---- screen ---- */

static void draw_screen(unsigned int cnt, int result, int ax, int ay, int az,
			int gx, int gy, int gz, int temp, unsigned char who)
{
	char b[40];

	draw_text(2, 2, "MPU6050 S-TWI1", 20, 0x07FF, 0x0000, 2);

	draw_text(2, 24, "ACCEL   X      Y      Z", 30, 0x4FB7, 0x0000, 1);
	fmt_sdec(b, ax);
	draw_text(2, 33, "AX", 2, 0xFFFF, 0x0000, 1);
	draw_text(32, 33, b, 8, 0x07FF, 0x0000, 1);
	fmt_sdec(b, ay);
	draw_text(94, 33, "AY", 2, 0xFFFF, 0x0000, 1);
	draw_text(124, 33, b, 8, 0x07FF, 0x0000, 1);
	fmt_sdec(b, az);
	draw_text(186, 33, "AZ", 2, 0xFFFF, 0x0000, 1);
	draw_text(216, 33, b, 8, 0x07FF, 0x0000, 1);

	draw_text(2, 44, "GYRO   X      Y      Z", 30, 0x4FB7, 0x0000, 1);
	fmt_sdec(b, gx);
	draw_text(32, 53, b, 8, 0xFDE0, 0x0000, 1);
	fmt_sdec(b, gy);
	draw_text(124, 53, b, 8, 0xFDE0, 0x0000, 1);
	fmt_sdec(b, gz);
	draw_text(216, 53, b, 8, 0xFDE0, 0x0000, 1);

	{
		int c100 = (int)(((long)temp * 100) / 340) + 3653;
		int ipart = c100 / 100;
		int dpart = (c100 % 100) / 10;

		if (ipart < 10)
			draw_text(2, 66, "TEMP  +0", 8, 0xAFE5, 0x0000, 1);
		else
			draw_text(2, 66, "TEMP  +", 7, 0xAFE5, 0x0000, 1);
		{
			char t[5];

			t[0] = (char)('0' + (ipart % 10));
			t[1] = '.';
			t[2] = (char)('0' + dpart);
			t[3] = 'C';
			t[4] = 0;
			draw_text(58, 66, t, 4, 0xAFE5, 0x0000, 1);
		}
	}

	draw_text(2, 77, "CNT", 3, 0xFFFF, 0x0000, 1);
	{
		char h[9];
		int i;

		for (i = 7; i >= 0; i--) {
			h[i] = "0123456789ABCDEF"[cnt & 0xF];
			cnt >>= 4;
		}
		h[8] = 0;
		draw_text(32, 77, h, 8, 0xFFFF, 0x0000, 1);
	}

	if (result == 0) {
		char w[16];

		draw_text(2, 88, "WHO=0x", 6, 0x4FE0, 0x0000, 1);
		w[0] = "0123456789ABCDEF"[who >> 4];
		w[1] = "0123456789ABCDEF"[who & 0xF];
		w[2] = 0;
		draw_text(38, 88, w, 2, 0x4FE0, 0x0000, 1);
		draw_text(56, 88, "I2C OK", 6, 0x4FE0, 0x0000, 1);
	} else {
		draw_text(2, 88, "I2C ERR step=", 13, 0xF800, 0x0000, 1);
		b[0] = "0123456789ABCDEF"[last_step >> 4];
		b[1] = "0123456789ABCDEF"[last_step & 0xF];
		b[2] = 0;
		draw_text(94, 88, b, 2, 0xF800, 0x0000, 1);
	}

	/* ARM panel: 4 opaque rows at the bottom */
	if (panel_valid)
		for (int l = 0; l < TEXT_LINES; l++)
			draw_text(2, 100 + l * 9, panel_text[l], 38,
				  0xFFFF, 0x0000, 1);
}

int main(void)
{
	unsigned char who = 0xFF, acc[6], gyr[6], tbuf[2];
	unsigned int cnt = 0;
	int result = -1;
	int ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, temp = 0;

	mbox[0] = MBOX_MAGIC;
	mbox[1] = MBOX_VERSION;
	mbox[2] = MBOX_FLAGS;
	mbox[3] = 0;			/* counter */
	mbox[4] = 0xFFFFFFFF;		/* result (0 = ok) */
	mbox[5] = 0;			/* error step */
	mbox[6] = 0x68;			/* addr to use */
	mbox[7] = 0xFF;			/* WHO_AM_I */
	mbox[8] = 0;			/* AX .. TEMP_C x100 */
	mbox[9] = 0;
	mbox[10] = 0;
	mbox[11] = 0;
	mbox[12] = 0;
	mbox[13] = 0;
	mbox[14] = 0;
	mbox[15] = 0;

	uart_init();
	uart_puts("\r\n[E906] MPU6050 + ST7789V LCD\r\n");

	gpio_init();
	spi_init();
	if (lcd_err_step == 0)
		lcd_init();
	twi_init();

	uart_puts("[E906] LCD ready, probing MPU6050 on S_TWI1...\r\n");

	if (mpu_write_reg(MPU_PWR1, 0) != 0 ||
	    mpu_read_regs(MPU_WHO, &who, 1) != 0) {
		mpu_addr = MPU_ADDR1;
		mpu_write_reg(MPU_PWR1, 0);
		if (mpu_read_regs(MPU_WHO, &who, 1) != 0)
			who = 0xFF;
	}
	if (who != 0xFF)
		mpu_write_reg(MPU_PWR1, 0);	/* wake from sleep */

	mbox[6] = mpu_addr;
	mbox[7] = who;

	uart_puts("WHO_AM_I=0x");
	uart_hex(who);
	uart_puts(" addr=0x");
	uart_hex(mpu_addr);
	uart_puts("\r\n");

	/* don't draw the ARM panel until the first real update */
	last_seq = *(volatile unsigned int *)MBOX_ARM_SEQ;

	for (;;) {
		cnt++;
		result = -1;

		if (who != 0xFF &&
		    mpu_read_regs(MPU_ACCEL, acc, 6) == 0 &&
		    mpu_read_regs(MPU_GYRO, gyr, 6) == 0 &&
		    mpu_read_regs(MPU_TEMP, tbuf, 2) == 0) {
			ax = (int)(short)((acc[0] << 8) | acc[1]);
			ay = (int)(short)((acc[2] << 8) | acc[3]);
			az = (int)(short)((acc[4] << 8) | acc[5]);
			gx = (int)(short)((gyr[0] << 8) | gyr[1]);
			gy = (int)(short)((gyr[2] << 8) | gyr[3]);
			gz = (int)(short)((gyr[4] << 8) | gyr[5]);
			temp = (int)(short)((tbuf[0] << 8) | tbuf[1]);

			mbox[8] = (unsigned int)ax;
			mbox[9] = (unsigned int)ay;
			mbox[10] = (unsigned int)az;
			mbox[11] = (unsigned int)gx;
			mbox[12] = (unsigned int)gy;
			mbox[13] = (unsigned int)gz;
			mbox[14] = (unsigned int)temp;
			mbox[15] = (unsigned int)(((long)temp * 100) / 340) + 3653;
			result = 0;
		} else {
			mbox[8] = 0;
			mbox[9] = 0;
			mbox[10] = 0;
			mbox[11] = 0;
			mbox[12] = 0;
			mbox[13] = 0;
			mbox[14] = 0;
			mbox[15] = 0;
		}
		mbox[3] = cnt;
		mbox[4] = (unsigned int)result;
		mbox[5] = (unsigned int)result ? last_step : 0;

		/* poll the ARM->E906 mailbox every 16 samples (~4 s) */
		if ((cnt & 15) == 0)
			poll_arm_text();

		draw_screen(cnt, result, ax, ay, az, gx, gy, gz, temp, who);
		lcd_fill(fb);

		uart_puts("\r\ncnt=");
		uart_hex(cnt);
		uart_puts(" AX=");
		uart_dec(ax);
		uart_puts(" AY=");
		uart_dec(ay);
		uart_puts(" AZ=");
		uart_dec(az);
		uart_puts(" GX=");
		uart_dec(gx);
		uart_puts(" T=");
		uart_dec(temp);
		uart_puts(" r=");
		uart_hex((unsigned int)result);

		if (lcd_err_step != 0) {
			uart_puts(" SPI_err=0x");
			uart_hex(lcd_err_step);
		}

		delay_ms(250);
	}
}
