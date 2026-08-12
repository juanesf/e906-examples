#!/usr/bin/env python3
"""
ARM-side verifier for the E906 "klipper" example (Avaota-A1), mailbox v8.

Run on the board with root while the e906-klipper firmware is loaded:

    sudo python3 step_check.py                              # 1 MOVE X 2000
    sudo python3 step_check.py --axis Y --steps 500 --dir 0
    sudo python3 step_check.py --queue 3 --steps 1000 --rate 1000
    sudo python3 step_check.py --home [--axis Y]            # homing + endstop
    sudo python3 step_check.py --pwm                        # heater/fan PWM
    sudo python3 step_check.py --uart          # force injection via S-UART0
    sudo python3 step_check.py --observe       # do not inject, only sample

Moves: FLUSH + enqueue one per QADD on the DDR mailbox command slot
(0x60000000, cmd @ +0x40, ack @ +0x50; the move queue is held inside the
E906's SRAM).  If the mailbox does not accept the moves it falls back to
S-UART0 (0x07080000).  It then samples the physical PL2/PL4 step pin
(S_PIO data reg 0x07022010), counts rising edges, measures the real
execution time and compares with the E906's own position counters.

--home: drives the virtual endstop input (+0x60: the ARM writes bit0/bit1
for X/Y hit; S_PIO is RISC-V-exclusive on this SoC and its PUL writes do
not stick, so pins cannot be pulled up or driven), issues HOME over the
mailbox, trips the switch mid-move and verifies the E906 stops and zeroes
the axis position.

--uart: ARM -> E906 self-injection over S-UART0.  The ARM and the E906
share the same UART module, so this needs a TX<->RX jumper on the header
(pin 37 / PM0 -> pin 40 / PM1).  The jumper also loops the E906's own
replies back into its RX (self-feedback flood), so the script MUTEs the
E906 via the MUTE mailbox command, drains the stale feedback and then
injects cleanly.
"""

import argparse
import mmap
import os
import struct
import sys
import threading
import time

MBOX_BASE = 0x60000000
S_GPIO_BASE = 0x07022000
S_UART_BASE = 0x07080000

MBOX_MAGIC = 0xE9061B0B
MBOX_VERSION = 8

# mailbox command slot (v4-proven layout)
CMD_OFF = 0x40
ARG0_OFF = 0x44
ARG1_OFF = 0x48
ARG2_OFF = 0x4C
ACK_OFF = 0x50
RESULT_OFF = 0x54

CMD_FLUSH = 1
CMD_SET = 2
CMD_RATE = 3
CMD_QADD = 4
CMD_QFREE = 5
CMD_ENDSTOP = 6
CMD_HOME = 7
CMD_MUTE = 8          # arg0 1 = UART output silenced, 0 = talking

ES_OVR = 0x60          # ARM -> E906 virtual endstop (bit0 X / bit1 Y hit)
UART_RXLEN = 0x34      # E906 diag: current UART rxbuf length
RATE_OFF = 0x20        # E906 diag: current rate
DIAG_RATE_SET = 0x78   # E906 diag: uart RATE commands executed

RESULT_OK = 0
RESULT_BUSY = 1
RESULT_BAD = 2

PL_CFG1 = 0x04          # S_PIO mux register, PL8..PL15 nibbles
PL_DAT = 0x10           # S_PIO data register (bit n = PLn)
UART_USR = 0x7C         # DW-APB UART status register
UART_THR = 0x00         # transmit holding register (write)
UART_RBR = 0x00         # receive buffer register (read)
UART_LSR = 0x14         # line status register (bit0 = RX data ready)

STEP_PIN = {"X": 2, "Y": 4}
DIR_PIN = {"X": 3, "Y": 5}
AXIS_IDX = {"X": 0, "Y": 1}
POS_OFF = {"X": 0x18, "Y": 0x1C}

HOME_TRIGGER_STEPS = 300   # trip the fake switch after this many steps


def map_region(base, size=4096):
    """mmap /dev/mem; returns (mm, rd, wr) for 4-byte aligned accesses."""
    page = base & ~0xFFF
    off = base - page
    fd = os.open("/dev/mem", os.O_RDWR)
    try:
        m = mmap.mmap(fd, size, mmap.MAP_SHARED,
                      mmap.PROT_READ | mmap.PROT_WRITE, offset=page)
    finally:
        os.close(fd)
    rd = lambda o: struct.unpack_from("<I", m, off + o)[0]
    wr = lambda o, v: struct.pack_into("<I", m, off + o, v & 0xFFFFFFFF)
    return m, rd, wr


def safe_map(base, what):
    try:
        return map_region(base)
    except Exception as e:
        print(f"[!] cannot map {what} @ 0x{base:X}: {e}")
        return None, None, None


def mbox_send(mrd, mwr, code, a0, a1, a2):
    """Write a mailbox command and wait for the E906 to consume it.

    Protocol proven in v4: write args, write cmd last; the E906 executes
    and zeroes cmd (+0x40) and writes ack/result."""
    try:
        mwr(ARG0_OFF, a0)
        mwr(ARG1_OFF, a1)
        mwr(ARG2_OFF, a2)
        mwr(CMD_OFF, code)          # go last
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if mrd(CMD_OFF) == 0:   # consumed
                return mrd(RESULT_OFF)
            time.sleep(0.0005)
        return -1                   # timeout: E906 did not consume it
    except Exception as e:
        print(f"[!] mailbox write failed: {e}")
        return -1


def mbox_init(mrd, mwr):
    """Zero the command slot AND the virtual-endstop word (so stale DDR
    values from a previous session cannot be read as phantom commands or
    phantom endstops) and FLUSH the queue."""
    for off in (CMD_OFF, ARG0_OFF, ARG1_OFF, ARG2_OFF, ACK_OFF, RESULT_OFF,
                ES_OVR):
        mwr(off, 0)
    time.sleep(0.05)
    r = mbox_send(mrd, mwr, CMD_FLUSH, 0, 0, 0)
    print(f"[*] INIT mailbox: slot zeroed, FLUSH: {result_str(r)}")


def uart_send(rd, wr, text, gap=0.001):
    """Transmit `text` byte-by-byte on S-UART0 THR (polling USR/TFNF).

    A gap between chars is required: the E906 polls the RX FIFO (depth 1)
    in its main loop and drops back-to-back chars on a FIFO overrun.  1ms
    per char is a safe margin over the 87us/frame at 115200 baud."""
    try:
        for ch in text.encode("ascii"):
            for _ in range(100000):
                if rd(UART_USR) & 0x2:      # TFNF: TX FIFO not full
                    break
            wr(UART_THR, ch)
            time.sleep(gap)
        return True
    except Exception as e:
        print(f"[!] S-UART0 injection failed: {e}")
        return False


def result_str(res):
    return {RESULT_OK: "ok", RESULT_BUSY: "busy",
            RESULT_BAD: "bad args", -1: "timeout"}.get(res, str(res))


def uart_loopback_ok(mrd, mwr, urd, uwr):
    """True if the S-UART0 module's TX loops back to its RX.

    The ARM and the E906 share the same S-UART0 module: the ARM's THR
    drives the TX pad (PM0, 40-pin header pin 37) and the E906's RX comes
    from the RX pad (PM1, header pin 40).  Without an external jumper
    tying pin 37 to pin 40 there is no loopback, so ARM->E906 UART
    self-injection can never reach the E906.

    The probe MUTEs the E906 first (killing the self-feedback flood) and
    writes 'U', then confirms it reached the E906 by watching its rxbuf
    length (mbox +0x34).  The ARM's own LSR.DR is not a reliable probe
    here: the E906 polls the shared RX FIFO in a tight loop and usually
    pops the echo before the ARM samples it."""
    mbox_send(mrd, mwr, CMD_MUTE, 1, 0, 0)
    time.sleep(0.5)               # let the loopback flood die down
    for _ in range(100000):
        if urd(UART_USR) & 0x2:   # TFNF: TX FIFO not full
            break
    uwr(UART_THR, 0x55)
    t = time.monotonic() + 0.2
    while time.monotonic() < t:
        if mrd(UART_RXLEN):
            return True
    return False


def send_queue(mrd, mwr, axis, steps_signed, rate, count):
    """FLUSH then enqueue `count` moves via QADD; returns # queued."""
    r = mbox_send(mrd, mwr, CMD_FLUSH, 0, 0, 0)
    print(f"[*] FLUSH via mailbox: {result_str(r)}")
    queued = 0
    for i in range(count):
        res = mbox_send(mrd, mwr, CMD_QADD, axis, steps_signed, rate)
        if res == RESULT_OK:
            queued += 1
        elif res == RESULT_BUSY:
            print(f"[!] queue full at move {i + 1}")
            break
        else:
            print(f"[!] QADD rejected (res {result_str(res)}) at move {i + 1}")
            break
    print(f"[*] queued {queued}/{count} MOVE {axis} {steps_signed} "
          f"@ {rate or 'default'} (E906 SRAM queue)")
    return queued


def send_uart(urd, uwr, args, steps_signed):
    """Send RATE + the MOVE(s) as ASCII over the R-domain S-UART0."""
    if args.rate:
        uart_send(urd, uwr, f"RATE {args.rate}\r\n")
        time.sleep(0.05)          # let the RATE line settle before the MOVE(s)
    for _ in range(args.queue):
        if args.rate:
            cmd = f"MOVE {args.axis} {steps_signed} {args.rate}\r\n"
        else:
            cmd = f"MOVE {args.axis} {steps_signed}\r\n"
        uart_send(urd, uwr, cmd)
        time.sleep(0.02)          # don't overrun the depth-1 RX FIFO
    print(f"[*] injected {args.queue}x via S-UART0: "
          f"{cmd.strip()!r}")
    return args.queue


def mbox_fallback_uart(uart, urd, uwr, args, steps_signed):
    if uart is None:
        print("[!] S-UART0 fallback unavailable; no injection")
        return 0
    print("[!] mailbox rejected the MOVE; trying S-UART0 fallback")
    return send_uart(urd, uwr, args, steps_signed)


def uart_flush_clean(mrd, mwr, urd, uwr):
    """Quiet the E906's UART and leave its rxbuf empty.

    With a TX<->RX jumper the E906's own boot replies loop back into its
    RX; MUTE stops new ones but the already-transmitted backlog keeps
    dribbling in for a while and a truncated reply can sit half-parsed in
    rxbuf forever.  Wait for that backlog to drain, then push CRs until
    the E906 reports rxlen 0 (mbox +0x34).  Returns True if clean."""
    mbox_send(mrd, mwr, CMD_MUTE, 1, 0, 0)
    time.sleep(5.0)               # let the boot feedback backlog drain
    for _ in range(8):
        uart_send(urd, uwr, "\r")   # force-fire any stuck fragment
        time.sleep(0.3)
        if mrd(UART_RXLEN) == 0:
            time.sleep(0.3)
            if mrd(UART_RXLEN) == 0:
                return True
    return False


def wait_and_count(grd, mrd, pin, pos_off, target, timeout):
    """Tight loop: count rising edges on `pin` until pos reaches `target`
    or `timeout` s elapse.  Returns (edges, elapsed, t_first).

    `elapsed` is anchored to the first observed rising edge when any edge
    is seen, so moves that start before the sampler begins (the UART
    injection runs the moves while still sending them) are timed from the
    real step start; otherwise it is anchored to the loop start."""
    prev = (grd(PL_DAT) >> pin) & 1
    count = 0
    t_first = None
    start = time.monotonic()
    end = start + timeout
    while True:
        v = (grd(PL_DAT) >> pin) & 1
        if v and not prev:
            count += 1
            if t_first is None:
                t_first = time.monotonic()
        prev = v
        if mrd(pos_off) == target:
            break
        if time.monotonic() >= end:
            break
    done = time.monotonic()
    elapsed = done - (t_first if t_first is not None else start)
    return count, elapsed, t_first


def es_override(mwr, bits):
    """Set the virtual endstop input (+0x60).  Written with retries: the
    DDR mailbox region occasionally drops writes, and the E906 re-reads
    the word every loop iteration."""
    for _ in range(3):
        mwr(ES_OVR, bits)
        time.sleep(0.001)
    time.sleep(0.01)


def home_run(mrd, mwr, grd, gwr, axis, rate=1000):
    """Full homing cycle on the virtual endstop (+0x60).

    Position reads over the mailbox are unreliable (the DDR region drops
    them intermittently), so this counts real rising edges on the STEP pin
    and samples the DIR pin instead; the E906's own zeroing is verified
    from the final position read (which is written once and stable).
    """
    axis_i = AXIS_IDX[axis]
    step_pin = STEP_PIN[axis]
    dir_pin = DIR_PIN[axis]
    ok = True

    print(f"[*] home {axis}: virtual endstop at +0x60 (PL14/15 hook)")
    es_override(mwr, 0)                 # released

    es0 = mbox_send(mrd, mwr, CMD_ENDSTOP, 0, 0, 0)
    print(f"[*] ENDSTOP before home: 0x{es0:02X} (expect 0x00 = "
          "both free)")
    if es0 & (1 << axis_i):
        print("[!] endstop already triggered before start")
        ok = False

    r = mbox_send(mrd, mwr, CMD_HOME, axis_i, 0, rate)   # 0 = min
    print(f"[*] HOME {axis} min @ {rate}: {result_str(r)}")

    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0 and mrd(0x24) != 2:
        time.sleep(0.001)
    if mrd(0x24) != 2:
        print("[!] E906 did not enter homing state (2)")
        es_override(mwr, 0)
        return False

    dir_low = (grd(PL_DAT) >> dir_pin) & 1
    print(f"[*] homing running (DIR_{axis} = {dir_low}, expected 0 = "
          f"min); counting {HOME_TRIGGER_STEPS} steps...")
    if dir_low != 0:
        print(f"[!] DIR_{axis} high: homing toward max, not min")
        ok = False

    prev = (grd(PL_DAT) >> step_pin) & 1
    count = 0
    triggered = False
    trig_time = 0.0
    done = False
    while time.monotonic() - t0 < 15.0:
        v = (grd(PL_DAT) >> step_pin) & 1
        if v and not prev:
            count += 1
        prev = v
        if not triggered and count >= HOME_TRIGGER_STEPS:
            es_override(mwr, 1 << axis_i)   # trip the switch
            triggered = True
            trig_time = time.monotonic()
        if triggered and mrd(0x24) == 0:
            done = True
            break

    pos1 = mrd(POS_OFF[axis])
    es1 = mbox_send(mrd, mwr, CMD_ENDSTOP, 0, 0, 0)

    print(f"[*] home finished: state={mrd(0x24)}, pos={pos1}, "
          f"steps={count}, endstop=0x{es1:02X}")

    if not done:
        print("[!] timeout waiting for homing to finish")
        ok = False
    if count < HOME_TRIGGER_STEPS:
        print(f"[!] did not reach the expected {HOME_TRIGGER_STEPS} steps "
              f"({count})")
        ok = False
    if pos1 != 0:
        print(f"[!] pos not zeroed after home ({pos1})")
        ok = False
    if not (es1 & (1 << axis_i)):
        print(f"[!] endstop {axis} not triggered after homing")
        ok = False
    if ok:
        print(f"[OK] homing {axis}: {count} steps toward min, endstop "
              f"tripped at {trig_time - t0:.3f}s, pos zeroed")

    es_override(mwr, 0)                 # release the switch
    es2 = mbox_send(mrd, mwr, CMD_ENDSTOP, 0, 0, 0)
    print(f"[*] endstop after release: 0x{es2:02X} (expect 0x00)")
    if es2 & (1 << axis_i):
        print("[!] endstop still triggered after release")
        ok = False
    return ok


def pwm_check(mrd, mwr, grd):
    """Verify the software PWM on PL6 (heater) / PL7 (fan).

    Sets each duty over the mailbox (cmd SET, ch 0 = H / 1 = F) and samples
    the pin for ~10 ms: the high-time ratio must track duty/256 at ~1 kHz.
    All duties and both channels must pass for the block to be OK."""
    channels = [("H", 6, 0), ("F", 7, 1)]
    duties = [64, 128, 192, 255, 0]
    ok = True
    for name, pin, ch in channels:
        for duty in duties:
            r = mbox_send(mrd, mwr, CMD_SET, ch, duty, 0)
            if r != 0:
                print(f"[!] SET {name} {duty} via mailbox: result {r}")
                ok = False
                continue
            high = 0
            samples = 0
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.01:
                if (grd(PL_DAT) >> pin) & 1:
                    high += 1
                samples += 1
            ratio = high / max(samples, 1)
            exp = duty / 256.0
            err = abs(ratio - exp)
            good = err <= 0.08
            ok = ok and good
            print(f"[{'OK' if good else '~'}] PWM {name} duty {duty:3d}: "
                  f"ratio {ratio:.3f} expected {exp:.3f} "
                  f"({'within' if good else f'out ({err:.3f})'})")
    mbox_send(mrd, mwr, CMD_SET, 0, 0, 0)   # heater off
    mbox_send(mrd, mwr, CMD_SET, 1, 0, 0)   # fan off
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--axis", default="X", choices=["X", "Y"])
    ap.add_argument("--steps", type=int, default=2000)
    ap.add_argument("--dir", type=int, default=1)
    ap.add_argument("--rate", type=int, default=None,
                    help="steps/s per move (50-20000)")
    ap.add_argument("--queue", type=int, default=1,
                    help="number of moves to queue")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--home", action="store_true",
                    help="homing + endstop (virtual switch on PL14/15)")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--mbox", action="store_true",
                      help="mailbox only (no serial fallback)")
    mode.add_argument("--uart", action="store_true",
                      help="force injection via S-UART0")
    mode.add_argument("--pwm", action="store_true",
                      help="verify heater/fan PWM (SET H/F via mailbox)")
    mode.add_argument("--observe", action="store_true",
                      help="do not inject a command, only sample")
    args = ap.parse_args()

    if os.geteuid() != 0:
        print("Run with sudo.")
        return 1

    mbox, mrd, mwr = safe_map(MBOX_BASE, "mailbox DDR")
    gpio, grd, gwr = safe_map(S_GPIO_BASE, "S_PIO")
    uart, urd, uwr = safe_map(S_UART_BASE, "S-UART0")

    if mbox is None or gpio is None:
        return 1

    if mrd(0x00) != MBOX_MAGIC:
        print(f"[!] mailbox has no magic 0x{MBOX_MAGIC:08X} (read "
              f"0x{mrd(0x00):08X}); e906-klipper firmware is not loaded.")
        print("    sudo ./deploy-klipper.sh")
        return 1

    if mrd(0x04) != MBOX_VERSION:
        print(f"[!] mailbox version {mrd(0x04)} != {MBOX_VERSION} "
              "(expected e906-klipper v8)")
        return 1

    mbox_init(mrd, mwr)

    if args.pwm:
        ok = pwm_check(mrd, mwr, grd)
        return 0 if ok else 1

    if args.home:
        ok = home_run(mrd, mwr, grd, gwr, args.axis, args.rate or 1000)
        return 0 if ok else 1

    pin = STEP_PIN[args.axis]
    pos_off = POS_OFF[args.axis]
    pos0 = mrd(pos_off)
    rate_now = mrd(0x20)
    mst = mrd(0x28)

    print(f"[*] E906 klipper alive (v{mrd(0x04)}), current rate {rate_now} Hz, "
          f"mxstatus 0x{mst:08X} (theadisaee="
          f"{'YES' if mst & (1 << 22) else 'NO'})")

    steps_signed = args.steps if args.dir else -args.steps
    total_steps = abs(steps_signed) * args.queue
    target = (pos0 + steps_signed * args.queue) & 0xFFFFFFFF

    if args.observe:
        print("[*] observe mode: nothing injected; sampling "
              f"for {args.seconds:.1f}s")
        edges = 0
        prev = (grd(PL_DAT) >> pin) & 1
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            v = (grd(PL_DAT) >> pin) & 1
            if v and not prev:
                edges += 1
            prev = v
        elapsed = args.seconds
    elif args.uart:
        if uart is None:
            print("[!] S-UART0 not accessible (use --observe)")
            return 1
        if not uart_loopback_ok(mrd, mwr, urd, uwr):
            print("[~] SKIP --uart: no TX<->RX loopback on S-UART0 "
                  "(ARM THR drives PM0 = pin 37, the E906 RX comes from "
                  "PM1 = pin 40; bridge both on the header "
                  "to use this channel)")
            return 0
        # The ARM and the E906 share S-UART0: with a TX<->RX jumper the
        # E906's own boot replies loop back into its RX and flood the
        # parser, and a truncated reply can wedge rxbuf forever.  Quiet
        # it, drain the backlog and leave a clean rxbuf, then inject.
        if not uart_flush_clean(mrd, mwr, urd, uwr):
            print("[!] could not leave the E906 rxbuf clean; "
                  "use --mbox or --observe")
            return 2
        rate_before = mrd(RATE_OFF)
        timeout = max(args.seconds, total_steps /
                      max(args.rate or rate_now, 1) * 1.5 + 0.5)
        # Sample in a thread started *before* the injection: the E906
        # executes each MOVE as soon as it is parsed, so moves overlap
        # the TX.  wait_and_count anchors elapsed to the first edge.
        box = {}

        def _sampler():
            box["edges"], box["elapsed"], box["t_first"] = wait_and_count(
                grd, mrd, pin, pos_off, target, timeout)

        th = threading.Thread(target=_sampler, daemon=True)
        th.start()
        send_uart(urd, uwr, args, steps_signed)
        th.join(timeout + 2.0)
        if th.is_alive():
            print("[!] sampler did not finish in time")
            return 2
        edges, elapsed = box["edges"], box["elapsed"]
        if args.rate and mrd(DIAG_RATE_SET) == 0:
            print("[!] E906 did not execute the RATE over UART "
                  "(corrupted line?)")
        if args.rate:
            print(f"[*] rate via UART: {mrd(RATE_OFF)} "
                  f"(was {rate_before})")
        print(f"[*] waiting for the move to finish ({timeout:.1f}s max)...")
        mbox_send(mrd, mwr, CMD_MUTE, 0, 0, 0)   # back to talking
    else:
        n = send_queue(mrd, mwr, AXIS_IDX[args.axis], steps_signed,
                       args.rate or 0, args.queue)
        sent_via = "mbox"
        if n < args.queue:
            if not args.mbox:
                n2 = mbox_fallback_uart(uart, urd, uwr, args, steps_signed)
                n = n2 if n2 else n
                sent_via = "uart+mbox" if n2 else "mbox"
            else:
                print("[!] could not queue all moves (no fallback)")
        print(f"[*] channel used: {sent_via}")
        timeout = max(args.seconds, total_steps /
                      max(args.rate or rate_now, 1) * 1.5 + 0.5)
        print(f"[*] waiting for the move to finish ({timeout:.1f}s max)...")
        edges, elapsed, _ = wait_and_count(grd, mrd, pin, pos_off, target,
                                           timeout)

    pos1 = mrd(pos_off)
    state = mrd(0x24)

    delta = (pos1 - pos0) & 0xFFFFFFFF
    if delta >= 0x80000000:
        delta -= 0x100000000

    print(f"[*] PL{args.axis} (pin PL{2 if args.axis=='X' else 4}): "
          f"{edges} rising edges in {elapsed:.2f}s")
    print(f"[*] mailbox pos_{args.axis.lower()}: {pos0} -> {pos1} "
          f"(delta {delta}), state={state}")

    if args.observe:
        return 0

    # 1) position / pulses
    if delta == 0:
        print("[!] no movement: the command did not reach the E906")
        return 1
    ratio = edges / abs(delta)
    if ratio >= 0.9:
        print(f"[OK] {edges} pin pulses vs {abs(delta)} steps "
              f"counted by the E906 ({ratio:.0%})")
    else:
        print(f"[~] there was movement ({abs(delta)} steps) but I only saw "
              f"{edges} edges ({ratio:.0%})")

    # 2) timing: expected total = N*steps/rate
    rate_used = args.rate or rate_now
    expected = total_steps / rate_used
    err_pct = (elapsed / expected - 1) * 100.0
    print(f"[*] timing: {elapsed:.3f}s actual vs {expected:.3f}s expected "
          f"({err_pct:+.1f}%)")
    if abs(err_pct) <= 8.0:
        print(f"[OK] timing correct (no gaps in queue: {err_pct:+.1f}%)")
    else:
        print(f"[~] timing out of tolerance ({err_pct:+.1f}%), "
              "possible jitter or gaps between moves")

    # 3) queue drained (QFREE: free slots; 15 = empty)
    if not args.uart and args.queue > 1:
        free = mbox_send(mrd, mwr, CMD_QFREE, 0, 0, 0)
        if free == 15:
            print(f"[OK] queue drained ({args.queue} moves consumed, "
                  "QFREE=15)")
        else:
            print(f"[~] queue not drained: QFREE={free} (15=empty)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
