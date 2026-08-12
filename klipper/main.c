/*
 * E906 "klipper" example for the Allwinner A523/T527 RISC-V co-processor
 * (Avaota-A1): stepper step/dir driver + software-PWM heater/fan on the
 * E906, commanded by the ARM host over the DDR mailbox.
 *
 * Cross-compiled for rv32imac/ilp32.  Booted by the ARM-side remoteproc
 * driver (sun55i_e906_rproc) from the reserved DDR carveout.
 *
 * Block 2: timer-based stepping + move queue.
 *   - Precise step timing with rdtime (24 MHz), period measured from
 *     rising edge to rising edge (no accumulated drift).
 *   - Move queue of up to QSIZE=16 moves held in E906 SRAM.  The ARM
 *     enqueues one move per QADD command (single command slot, the
 *     mailbox protocol proven reliable in v4); the E906 executes the
 *     queued moves back-to-back while still servicing the mailbox and
 *     the UART.  Each move is a signed step delta (sign = direction)
 *     and carries its own step rate.
 * Block 3: endstops + homing.
 *   - X/Y endstops on PL14/PL15 (active-low, internal pull-up).  The
 * Block 3: endstops + homing.
 *   - The endstop is read from the mailbox virtual input at +0x60 (line
 *     0x60, ARM -> E906, never written by the E906 so it is safe to
 *     invalidate); the host drives bit0 = X hit, bit1 = Y hit.  This is
 *     the test channel AND the current interface: on this part S_PIO is
 *     RISC-V-exclusive (ARM writes to PL14/15 mux bits are dropped by
 *     the bus) and its PUL writes do not stick, so physical pins cannot
 *     be pulled up or driven.  Physical PL14/PL15 (active-high, external
 *     pull-down) are OR-ed in as a reserved real-endstop hook.
 *   - HOME(axis, dir, rate): a long timer-stepped move toward min/max;
 *     every step boundary checks the endstop, and on trigger the move
 *     stops and the axis position is zeroed (state 2 while homing).
 *   - The MCU does NOT drive the LCD (the display node stays for Linux).
 *
 * Mailbox v8 (reserved DDR @ 0x60000000, DA == PA):
 *   +0x00 magic 0xE9061B0B   +0x04 version = 8
 *   +0x08 flags 0x0000B00B   +0x0C ms tick
 *   +0x10 result             +0x14 error step
 *   +0x18 pos_x (signed)     +0x1C pos_y (signed)
 *   +0x20 rate (last used)   +0x24 state (0 idle / 1 moving / 2 homing)
 *   +0x28 mxstatus (diag)    +0x2C endstops (bit0 X / bit1 Y, 1=hit)
 *   +0x34 uart rxbuf length (diag)  +0x78 uart RATE cmds executed (diag)
 *   +0x60 virtual endstop (ARM -> E906; bit0 X / bit1 Y, 1=hit)
 *
 * Command slot (ARM -> E906, polled by the E906; v4-proven protocol):
 *   +0x40 cmd  1 FLUSH, 2 SET, 3 RATE, 4 QADD, 5 QFREE, 6 ENDSTOP, 7 HOME,
 *              8 MUTE (UART responses 0/1)
 *   +0x44 arg0 QADD: axis / SET: ch / RATE: steps/s / HOME: axis / MUTE: 0|1
 *   +0x48 arg1 QADD: steps / SET: duty / HOME: 0=min 1=max
 *   +0x4C arg2 QADD|HOME: rate (0 = default)
 *   +0x50 ack  incremented once consumed
 *   +0x54 result 0 ok, 1 busy/full, 2 bad args; QFREE: free slots;
 *               ENDSTOP: endstop bits
 *
 * Serial channel: S-UART0 @ 115200 8N1 (TX on PM0 = 40-pin header pin 37).
 * ASCII commands, CR/LF terminated:
 *
 *   MOVE X <steps> [rate]     enqueue steps on X (signed: sign = dir)
 *   MOVE Y <steps> [rate]     enqueue steps on Y
 *   SET H <0-255>             heater PWM duty
 *   SET F <0-255>             fan PWM duty
 *   RATE <50-20000>           default step rate in steps/s
 *   HOME X|Y <0|1> [rate]     homing run toward min/max (0=min, 1=max)
 *   ENDSTOP                   report X/Y endstop state (0/1)
 *   FLUSH                     stop current move and drop the queue
 *   STATUS                    one-line state report
 *   HELP                      command list
 *
 *   NOTE: on this board the ARM and the E906 share S-UART0, so a TX/RX
 *   jumper on the header (pin 37 <-> pin 40) loops the E906's own replies
 *   back into its RX.  The MUTE mailbox command lets the ARM run the
 *   serial channel as a self-injection test bed without that feedback.
 *
 * Pins (R-domain S_PIO):
 *   PL2 = STEP_X   PL3 = DIR_X
 *   PL4 = STEP_Y   PL5 = DIR_Y
 *   PL6 = HEATER   PL7 = FAN        (software PWM @ ~1 kHz)
 *   PL14/PL15 = reserved X/Y endstop hook (active-high, external pull-down)
 *   PL0/PL1 are the PMIC r_i2c0 (left untouched); PL8..PL13 are the
 *   LCD/DSI pins owned by Linux (never touched by the E906).
 *
 * Cache note: the ARM writes the command slot through an uncached
 * /dev/mem mapping but the E906's D-cache can hold a stale copy of the
 * line (cmd==0 forever).  mbox_poll() starts with a T-Head dcache.ipa
 * (invalidate D-cache line by physical address) before reading the slot.
 * The E906 only ever invalidates ARM->E906 lines (0x40 cmd slot, 0x60
 * virtual endstop); its own writes go to the header lines 0x00/0x20 and
 * the ack/result line 0x50 (never invalidated).  Verified on the board.
 */

#define MBOX_BASE  0x60000000UL
#define MBOX_MAGIC 0xE9061B0BUL
#define MBOX_VERSION 0x00000008UL
#define MBOX_FLAGS 0x0000B00BUL

static volatile unsigned int *const mbox =
    (volatile unsigned int *)MBOX_BASE;

#define S_GPIO_BASE  0x07022000UL
#define S_UART0_BASE 0x07080000UL
#define STBY_PRCM    0x07010000UL

#define PL_CFG0      (S_GPIO_BASE + 0x00UL)	/* PL0..PL7 mux nibbles */
#define PL_DAT       (S_GPIO_BASE + 0x10UL)	/* PL0..PL15 levels */
#define PM_CFG0      (S_GPIO_BASE + 0x30UL)	/* PM0..PM7 mux nibbles */
#define S_UART_BGR   (STBY_PRCM + 0x18CUL)

#define PIN_STEP_X   (1UL << 2)
#define PIN_DIR_X    (1UL << 3)
#define PIN_STEP_Y   (1UL << 4)
#define PIN_DIR_Y    (1UL << 5)
#define PIN_HEATER   (1UL << 6)
#define PIN_FAN      (1UL << 7)
#define PIN_XMIN     (1UL << 14)
#define PIN_YMIN     (1UL << 15)

#define TICKS_PER_MS     24000UL	/* rdtime runs at 24 MHz */
#define PWM_PERIOD_TICKS (TICKS_PER_MS * 1UL)	/* ~1 kHz */
#define STEP_PULSE_TICKS 1200UL		/* ~50 us max pulse high */

#define MIN_RATE  50UL
#define MAX_RATE  20000UL

/* homing safety cap: if the endstop never trips, abort after this many
 * steps instead of running to the mechanical limit */
#define MAX_HOME_STEPS  2000000UL

#define QSIZE     16

static int pos_x;
static int pos_y;

static int moving;
static int cur_step_pin;
static int cur_dir_pin;
static int *cur_pos;
static unsigned int remaining;
static int dir;
static unsigned int rate = 1000;
static unsigned int period_ticks = TICKS_PER_MS;	/* 1000 steps/s */
static unsigned int pulse_ticks;
static unsigned int next_step;
static unsigned int pulse_until;
static int pulse_active;

/* move queue, in E906 SRAM (not DDR): one entry per QADD command */
struct qent {
	unsigned int axis;
	int steps;
	unsigned int rate;
};

static struct qent queue[QSIZE];
static unsigned int q_head;
static unsigned int q_tail;

static int pwm_h;
static int pwm_f;
static unsigned int pwm_phase_h;
static unsigned int pwm_phase_f;

static int homing;		/* nonzero while a homing run is active */
static int homing_axis;		/* 0 = X, 1 = Y */
static int hold_next;		/* skip one queue_poll after homing */
static unsigned int es_state;	/* bit0 X hit, bit1 Y hit (active-low) */
static char rxbuf[64];

static unsigned int rxlen;

static unsigned int uart_mute;	/* MUTE cmd: parse UART commands but stay silent */

static unsigned int diag_rxlen;	/* diagnostic: current rxbuf length (+0x34) */
static unsigned int diag_rate_set;	/* diagnostic: uart RATE cmds executed (+0x78) */

static inline unsigned int rdtime(void)
{
	unsigned int t;

	__asm__ volatile("rdtime %0" : "=r"(t));
	return t;
}

static inline unsigned int csr_mxstatus(void)
{
	unsigned int v;

	__asm__ volatile("csrr %0, 0x7C0" : "=r"(v));
	return v;
}

/* T-Head dcache.ipa: invalidate the D-cache line containing physical
 * address `pa`.  Only used on the ARM->E906 command-slot line, which the
 * E906 itself never writes. */
static inline void dcache_ipa(unsigned int pa)
{
	__asm__ volatile(".insn i 0x0B, 0, x0, %0, 0x02A"
			 :: "r"(pa) : "memory");
}

/* wrap-safe: returns 1 when now >= a */
static inline int tick_ge(unsigned int now, unsigned int a)
{
	return (int)(now - a) >= 0;
}

static void uart_init(void)
{
	volatile unsigned int *lcr =
	    (volatile unsigned int *)(S_UART0_BASE + 0x0CUL);
	volatile unsigned int *bgr =
	    (volatile unsigned int *)S_UART_BGR;

	*bgr |= (1UL << 16) | (1UL << 0);	/* de-assert reset, gate on */

	*(volatile unsigned int *)PM_CFG0 = 0x22UL;	/* PM0 TX, PM1 RX */

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

	if (uart_mute)
		return;
	while (!(*usr & 0x2UL))
		;
	*thr = (unsigned int)c;
}

static void uart_puts(const char *s)
{
	if (uart_mute)
		return;
	while (*s)
		uart_putc(*s++);
}

static void uart_dec(int v)
{
	char tmp[12];
	int i = 0, neg = 0;

	if (v < 0) {
		neg = 1;
		v = -v;
	}
	do {
		tmp[i++] = "0123456789"[v % 10];
		v /= 10;
	} while (v);
	if (neg)
		uart_putc('-');
	while (i)
		uart_putc(tmp[--i]);
}

static void gpio_init(void)
{
	volatile unsigned int *plcfg0 =
	    (volatile unsigned int *)PL_CFG0;
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;
	unsigned int v;

	/* PL2..PL7 -> gpio_out (1); keep PL0/PL1 (PMIC r_i2c0) as-is.
	 * PL8..PL13 (LCD/DSI) and PL14/PL15 (endstop hook) are left as the
	 * boot state: we cannot rely on writing their CFG/PUL registers on
	 * this part (writes from both masters are dropped/do not stick). */
	v = *plcfg0;
	v &= ~(0xFFFFFFUL << 8);
	v |= 0x111111UL << 8;
	*plcfg0 = v;

	/* all outputs low: STEP/DIR low, heater/fan off */
	*pldat &= ~(PIN_STEP_X | PIN_DIR_X | PIN_STEP_Y | PIN_DIR_Y |
		    PIN_HEATER | PIN_FAN);
}

/* endstop state = virtual input (+0x60, ARM->E906, the test channel) OR
 * the physical PL14/15 hook (active-high, external pull-down required).
 * The virtual word lives in cache line 0x60, which the E906 never writes,
 * so invalidating it before each read cannot drop our own stores. */
static unsigned int es_read(void)
{
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;
	unsigned int v = *pldat;
	unsigned int phys;

	phys = ((v & PIN_XMIN) ? 1 : 0) | ((v & PIN_YMIN) ? 2 : 0);
	dcache_ipa(MBOX_BASE + 0x60UL);
	es_state = phys | (mbox[24] & 3);
	return es_state;
}

/* per-axis endstop check, used at each step boundary while homing */
static inline int es_hit(int axis)
{
	return (es_state >> axis) & 1;
}

/* ---------- move queue (E906 SRAM) ---------- */

static int queue_add(unsigned int axis, int steps, unsigned int mrate)
{
	if ((q_tail + 1) % QSIZE == q_head)
		return 1;		/* full */
	if (steps == 0)
		return 2;		/* bad args */
	queue[q_tail].axis = axis;
	queue[q_tail].steps = steps;
	queue[q_tail].rate = mrate;
	q_tail = (q_tail + 1) % QSIZE;
	return 0;			/* ok */
}

static void queue_clear(void)
{
	q_head = 0;
	q_tail = 0;
}

static int queue_free(void)
{
	int used = (int)((q_tail + QSIZE - q_head) % QSIZE);

	return QSIZE - 1 - used;
}

/* ---------- motion: timer-based stepping ---------- */

static void load_move(unsigned int axis, int steps, unsigned int mrate)
{
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;
	unsigned int n;
	int d;

	if (mrate == 0)
		mrate = rate;
	if (mrate < MIN_RATE)
		mrate = MIN_RATE;
	if (mrate > MAX_RATE)
		mrate = MAX_RATE;

	n = (unsigned int)(steps < 0 ? -steps : steps);
	if (n == 0) {
		moving = 0;
		return;
	}
	d = steps > 0;

	cur_step_pin = (axis == 0) ? 2 : 4;
	cur_dir_pin = (axis == 0) ? 3 : 5;
	cur_pos = (axis == 0) ? &pos_x : &pos_y;

	/* set direction before the first step pulse */
	if (d)
		*pldat |= (1UL << cur_dir_pin);
	else
		*pldat &= ~(1UL << cur_dir_pin);

	/* period start-to-start, so no drift accumulates */
	period_ticks = (TICKS_PER_MS * 1000UL) / mrate;
	pulse_ticks = period_ticks / 2UL;
	if (pulse_ticks > STEP_PULSE_TICKS)
		pulse_ticks = STEP_PULSE_TICKS;

	remaining = n;
	dir = d;
	rate = mrate;
	moving = 1;
	pulse_active = 0;
	next_step = rdtime();
}

/* start a homing run: a long timer-stepped move toward min/max; step_poll
 * aborts it when the endstop trips and zeroes the axis position. */
static void do_home(unsigned int axis, unsigned int hdir, unsigned int mrate)
{
	int steps = (int)MAX_HOME_STEPS;

	if (hdir == 0)
		steps = -steps;
	homing = 1;
	homing_axis = (int)axis;
	load_move(axis, steps, mrate);
}

/* dequeue the next queued move when idle (back-to-back execution).
 * hold_next is set when homing just finished so the next move does not
 * start on the same poll (the endstop is still pressed). */
static void queue_poll(void)
{
	if (moving)
		return;
	if (hold_next) {
		hold_next = 0;
		return;
	}
	if (q_head == q_tail)
		return;
	load_move(queue[q_head].axis, queue[q_head].steps,
		  queue[q_head].rate);
	q_head = (q_head + 1) % QSIZE;
}

static void step_poll(void)
{
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;
	unsigned int now = rdtime();

	if (!moving)
		return;

	if (pulse_active) {
		if (tick_ge(now, pulse_until)) {
			*pldat &= ~(1UL << cur_step_pin);
			pulse_active = 0;
			remaining--;
			if (dir > 0)
				(*cur_pos)++;
			else
				(*cur_pos)--;
			if (remaining == 0) {
				moving = 0;
				if (homing) {
					/* safety cap reached: endstop
					 * never tripped */
					homing = 0;
					mbox[21] = 2;
				}
			}
		}
	} else if (tick_ge(now, next_step)) {
		if (homing && es_hit(homing_axis)) {
			/* endstop tripped: stop, zero the axis, hold the
			 * next queued move (the switch is still pressed) */
			moving = 0;
			*cur_pos = 0;
			homing = 0;
			hold_next = 1;
			mbox[21] = 0;	/* result: homed ok */
			return;
		}
		*pldat |= (1UL << cur_step_pin);
		pulse_active = 1;
		pulse_until = now + pulse_ticks;
		next_step += period_ticks;
	}
}

/* ---------- software PWM ---------- */

static void pwm_poll(void)
{
	volatile unsigned int *pldat =
	    (volatile unsigned int *)PL_DAT;
	unsigned int now = rdtime();

	if (now - pwm_phase_h >= PWM_PERIOD_TICKS) {
		pwm_phase_h = now;
		*pldat |= PIN_HEATER;
	} else if (now - pwm_phase_h >=
		   ((unsigned int)pwm_h * PWM_PERIOD_TICKS) / 256UL) {
		*pldat &= ~PIN_HEATER;
	}

	if (now - pwm_phase_f >= PWM_PERIOD_TICKS) {
		pwm_phase_f = now;
		*pldat |= PIN_FAN;
	} else if (now - pwm_phase_f >=
		   ((unsigned int)pwm_f * PWM_PERIOD_TICKS) / 256UL) {
		*pldat &= ~PIN_FAN;
	}
}

/* ---------- command handling ---------- */

static const char *next_tok(const char **pp)
{
	const char *p = *pp;
	const char *s;

	while (*p == ' ' || *p == '\t')
		p++;
	if (*p == 0) {
		*pp = p;
		return 0;
	}
	s = p;
	while (*p && *p != ' ' && *p != '\t')
		p++;
	*pp = p;
	return s;
}

static int tok_len(const char *a, const char *b)
{
	return (int)(b - a);
}

static int tok_cmp(const char *a, const char *b, int n)
{
	int i;

	for (i = 0; i < n; i++)
		if (a[i] != b[i])
			return a[i] - b[i];
	return 0;
}

static long tok_atoi(const char *a, const char *b)
{
	long v = 0;
	int neg = 0;

	if (a < b && (*a == '-' || *a == '+')) {
		neg = (*a == '-');
		a++;
	}
	while (a < b && *a >= '0' && *a <= '9') {
		v = v * 10 + (*a - '0');
		a++;
	}
	return neg ? -v : v;
}

static const char *tok_end(const char *s)
{
	while (*s && *s != ' ' && *s != '\t')
		s++;
	return s;
}

static void uart_cmd(const char *line)
{
	const char *cmd, *t1, *e1, *t2, *e2, *t3, *e3, *ec;

	cmd = next_tok(&line);
	if (!cmd)
		return;
	ec = line;		/* end of the cmd token */

	t1 = next_tok(&line);
	if (t1)
		e1 = tok_end(t1);
	else
		e1 = line;
	t2 = next_tok(&line);
	if (t2)
		e2 = tok_end(t2);
	else
		e2 = line;
	t3 = next_tok(&line);
	if (t3)
		e3 = tok_end(t3);
	else
		e3 = line;

	if (tok_len(cmd, ec) == 4 && tok_cmp(cmd, "MOVE", 4) == 0) {
		long steps;
		unsigned int mrate = 0;
		unsigned int axis;
		int r;

		if (!t1 || !t2) {
			uart_puts("ERR MOVE axis steps [rate]\r\n");
			return;
		}
		steps = tok_atoi(t2, e2);
		if (steps == 0 || steps > 1000000L || steps < -1000000L) {
			uart_puts("ERR MOVE steps\r\n");
			return;
		}
		if (t3)
			mrate = (unsigned int)tok_atoi(t3, e3);
		if (tok_len(t1, e1) == 1 && t1[0] == 'X')
			axis = 0;
		else if (tok_len(t1, e1) == 1 && t1[0] == 'Y')
			axis = 1;
		else {
			uart_puts("ERR MOVE axis\r\n");
			return;
		}
		r = queue_add(axis, (int)steps, mrate);
		if (r == 1)
			uart_puts("ERR queue full\r\n");
		else if (r == 2)
			uart_puts("ERR MOVE steps\r\n");
		else
			uart_puts("OK MOVE\r\n");
		return;
	}

	if (tok_len(cmd, ec) == 4 && tok_cmp(cmd, "HOME", 4) == 0) {
		long d;
		unsigned int hrate = 0;
		unsigned int axis;

		if (!t1 || !t2) {
			uart_puts("ERR HOME axis dir [rate]\r\n");
			return;
		}
		d = tok_atoi(t2, e2);
		if (d != 0 && d != 1) {
			uart_puts("ERR HOME dir\r\n");
			return;
		}
		if (t3)
			hrate = (unsigned int)tok_atoi(t3, e3);
		if (tok_len(t1, e1) == 1 && t1[0] == 'X')
			axis = 0;
		else if (tok_len(t1, e1) == 1 && t1[0] == 'Y')
			axis = 1;
		else {
			uart_puts("ERR HOME axis\r\n");
			return;
		}
		if (moving) {
			uart_puts("ERR busy\r\n");
			return;
		}
		do_home(axis, (unsigned int)d, hrate);
		uart_puts("OK HOME\r\n");
		return;
	}

	if (tok_len(cmd, ec) == 7 && tok_cmp(cmd, "ENDSTOP", 7) == 0) {
		unsigned int es = es_read();

		uart_puts("X=");
		uart_dec(es & 1);
		uart_puts(" Y=");
		uart_dec((es >> 1) & 1);
		uart_puts("\r\n");
		return;
	}

	if (tok_len(cmd, ec) == 3 && tok_cmp(cmd, "SET", 3) == 0) {
		long v;

		if (!t1 || !t2) {
			uart_puts("ERR SET ch duty\r\n");
			return;
		}
		v = tok_atoi(t2, e2);
		if (v < 0 || v > 255) {
			uart_puts("ERR SET duty\r\n");
			return;
		}
		if (tok_len(t1, e1) == 1 && t1[0] == 'H') {
			pwm_h = (int)v;
			uart_puts("OK SET H\r\n");
		} else if (tok_len(t1, e1) == 1 && t1[0] == 'F') {
			pwm_f = (int)v;
			uart_puts("OK SET F\r\n");
		} else {
			uart_puts("ERR SET ch\r\n");
		}
		return;
	}

	if (tok_len(cmd, ec) == 4 && tok_cmp(cmd, "RATE", 4) == 0) {
		long v;

		if (!t1) {
			uart_puts("ERR RATE hz\r\n");
			return;
		}
		v = tok_atoi(t1, e1);
		if (v < (long)MIN_RATE || v > (long)MAX_RATE) {
			uart_puts("ERR RATE hz\r\n");
			return;
		}
		rate = (unsigned int)v;
		diag_rate_set++;
		uart_puts("OK RATE\r\n");
		return;
	}

	if (tok_len(cmd, ec) == 5 && tok_cmp(cmd, "FLUSH", 5) == 0) {
		moving = 0;
		pulse_active = 0;
		remaining = 0;
		queue_clear();
		uart_puts("OK FLUSH\r\n");
		return;
	}

	if (tok_len(cmd, ec) == 6 && tok_cmp(cmd, "STATUS", 6) == 0) {
		unsigned int es = es_read();

		uart_puts("X=");
		uart_dec(pos_x);
		uart_puts(" Y=");
		uart_dec(pos_y);
		uart_puts(" BUSY=");
		uart_dec(moving);
		uart_puts(" HOME=");
		uart_dec(homing);
		uart_puts(" ES=");
		uart_dec((int)es);
		uart_puts(" Q=");
		uart_dec(queue_free());
		uart_puts(" H=");
		uart_dec(pwm_h);
		uart_puts(" F=");
		uart_dec(pwm_f);
		uart_puts(" RATE=");
		uart_dec((int)rate);
		uart_puts("\r\n");
		return;
	}

	if (tok_len(cmd, ec) == 4 && tok_cmp(cmd, "HELP", 4) == 0) {
		uart_puts("MOVE X|Y steps [rate] | HOME X|Y 0|1 [rate] | "
			  "ENDSTOP | SET H|F 0-255 | RATE hz | FLUSH | "
			  "STATUS | HELP\r\n");
		return;
	}

	uart_puts("ERR unknown\r\n");
}

static void uart_poll(void)
{
	volatile unsigned int *lsr =
	    (volatile unsigned int *)(S_UART0_BASE + 0x14UL);
	volatile unsigned int *rbr =
	    (volatile unsigned int *)(S_UART0_BASE + 0x00UL);
	int c;

	while (*lsr & 0x1UL) {
		c = (int)(*rbr & 0xFFUL);
		/* The RX FIFO of this SoC reads back 0x00 once empty, while
		 * LSR.DR can still be set for a few cycles after the last
		 * real pop. 0x00 is never a real command byte: stop. */
		if (c == 0)
			break;
		if (c == '\r' || c == '\n') {
			if (rxlen) {
				rxbuf[rxlen] = 0;
				uart_cmd(rxbuf);
				rxlen = 0;
				diag_rxlen = 0;
			}
		} else if (rxlen < sizeof(rxbuf) - 1) {
			rxbuf[rxlen++] = (char)c;
			diag_rxlen = rxlen;
		}
	}
}

static void mbox_poll(void)
{
	unsigned int cmd;

	/* The ARM writes the command slot through an uncached /dev/mem
	 * mapping; our D-cache can hold a stale copy of the line.  The
	 * E906 never writes this line itself, so invalidating it cannot
	 * drop our own stores. */
	dcache_ipa(MBOX_BASE + 0x40UL);
	cmd = mbox[16];		/* +0x40 */

	if (cmd == 0)
		return;

	switch (cmd) {
	case 1:		/* FLUSH */
		moving = 0;
		pulse_active = 0;
		remaining = 0;
		queue_clear();
		mbox[21] = 0;
		break;
	case 2: {	/* SET */
		unsigned int ch = mbox[17];
		unsigned int duty = mbox[18];

		if (duty > 255)
			mbox[21] = 2;
		else if (ch == 0) {
			pwm_h = (int)duty;
			mbox[21] = 0;
		} else if (ch == 1) {
			pwm_f = (int)duty;
			mbox[21] = 0;
		} else
			mbox[21] = 2;
		break;
	}
	case 3: {	/* RATE */
		unsigned int r = mbox[17];

		if (r < MIN_RATE || r > MAX_RATE)
			mbox[21] = 2;
		else {
			rate = r;
			mbox[21] = 0;
		}
		break;
	}
	case 4:		/* QADD */
		mbox[21] = (unsigned int)queue_add(mbox[17],
						   (int)mbox[18],
						   mbox[19]);
		break;
	case 5:		/* QFREE */
		mbox[21] = (unsigned int)queue_free();
		break;
	case 6:		/* ENDSTOP */
		mbox[21] = es_read();
		break;
	case 7: {	/* HOME axis dir [rate] */
		unsigned int haxis = mbox[17];
		unsigned int hdir = mbox[18];
		unsigned int hrate = mbox[19];

		if (haxis > 1 || hdir > 1 || hrate > MAX_RATE)
			mbox[21] = 2;	/* bad args */
		else if (moving)
			mbox[21] = 1;	/* busy */
		else {
			do_home(haxis, hdir, hrate);
			mbox[21] = 0;
		}
		break;
	}
	case 8: {	/* MUTE uart on/off (0 = talk, 1 = silent) */
		unsigned int on = mbox[17];

		if (on > 1)
			mbox[21] = 2;	/* bad args */
		else {
			uart_mute = on;
			if (on) {
				rxlen = 0;	/* drop the stale feedback line */
				diag_rxlen = 0;	/* keep mbox +0x34 in sync */
			}
			mbox[21] = 0;
		}
		break;
	}
	default:
		mbox[21] = 2;
		break;
	}

	mbox[20]++;		/* +0x50 ack */
	mbox[16] = 0;		/* consume the command */
}

int main(void)
{
	unsigned int tick_ms = 0;

	mbox[0] = MBOX_MAGIC;
	mbox[1] = MBOX_VERSION;
	mbox[2] = MBOX_FLAGS;
	mbox[3] = 0;
	mbox[4] = 0;		/* +0x10 result */
	mbox[5] = 0;		/* +0x14 error step */
	mbox[10] = csr_mxstatus();	/* +0x28 diagnostic */

	uart_init();
	uart_puts("\r\n[E906] klipper stepper/PWM/home firmware v8 "
		  "(queue in SRAM)\r\n");
	uart_puts("[E906] cmd: MOVE X|Y steps [rate] | HOME X|Y 0|1 [rate] | "
		  "ENDSTOP | SET H|F 0-255 | RATE hz | FLUSH | STATUS | "
		  "HELP\r\n");

	gpio_init();

	for (;;) {
		unsigned int now = rdtime();

		uart_poll();
		mbox_poll();
		queue_poll();
		pwm_poll();
		step_poll();

		if (now - tick_ms >= TICKS_PER_MS) {
			tick_ms = now;
			mbox[3]++;		/* ms tick */
		}
		mbox[6] = (unsigned int)pos_x;
		mbox[7] = (unsigned int)pos_y;
		mbox[8] = rate;
		mbox[9] = (unsigned int)(moving ? (homing ? 2 : 1) : 0);
		mbox[11] = es_read();		/* +0x2C endstops */
		mbox[13] = diag_rxlen;		/* +0x34 diag: rxbuf length */
		mbox[30] = diag_rate_set;	/* +0x78 diag: uart RATE executed */
	}
	/* unreachable */
}
