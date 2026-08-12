#!/usr/bin/env python3
"""
E906 MCU bridge for Klipper (Avaota-A1 / Allwinner A523/T527).

This daemon runs on the ARM Linux host (as root, for /dev/mem) and makes the
RISC-V E906 co-processor look like a regular Klipper mcu over the standard
Klipper serial pipe protocol.

  * Klipper talks to this bridge over a PTY symlinked at /tmp/klipper_host_e906
  * The bridge talks to the E906 over the reserved DDR mailbox:
        0x60000000, command slot @ +0x40, ack/result @ +0x50
    using the v8 protocol implemented in e906-klipper.elf.

Features mapped to the E906:
  - stepper X/Y queue_step -> mailbox QADD (signed steps + rate)
  - endstop + trsync -> mailbox ENDSTOP polling and FLUSH on trigger
  - heater/fan PWM -> mailbox SET (ch 0/1, duty 0-255)
  - virtual 50 MHz clock for Klipper clocksync

Run directly for debugging:

    sudo python3 /home/avaota/e906-examples/klipper/e906_mcu_bridge.py

To stop: Ctrl-C or kill.  On fatal error the E906 queue is FLUSHed and the
firmware left in a safe state.
"""

import errno
import fcntl
import json
import mmap
import os
import signal
import struct
import sys
import termios
import threading
import time
import tty
import zlib

# ---------------------------------------------------------------------------
# Paths and constants
# ---------------------------------------------------------------------------
FIFO = "/tmp/klipper_host_e906"

MBOX_BASE = 0x60000000
MBOX_PAGE = MBOX_BASE & ~0xFFF
MBOX_OFF = MBOX_BASE - MBOX_PAGE

S_GPIO_BASE = 0x07022000
S_GPIO_PAGE = S_GPIO_BASE & ~0xFFF
S_GPIO_OFF = S_GPIO_BASE - S_GPIO_PAGE

# Command slot offsets (mailbox v8)
CMD_OFF = 0x40
ARG0_OFF = 0x44
ARG1_OFF = 0x48
ARG2_OFF = 0x4C
ACK_OFF = 0x50
RESULT_OFF = 0x54
ES_OVR = 0x60

# Diagnostic/status offsets from E906
POS_X_OFF = 0x18
POS_Y_OFF = 0x1C
RATE_OFF = 0x20
STATE_OFF = 0x24
ENDSTOP_OFF = 0x2C

CMD_FLUSH = 1
CMD_SET = 2
CMD_RATE = 3
CMD_QADD = 4
CMD_QFREE = 5
CMD_ENDSTOP = 6
CMD_HOME = 7

RESULT_OK = 0
RESULT_BUSY = 1
RESULT_BAD = 2

# Pin enumeration seen by Klipper
PIN_STEP_X = "PL2"
PIN_DIR_X = "PL3"
PIN_STEP_Y = "PL4"
PIN_DIR_Y = "PL5"
PIN_HEATER = "PL6"
PIN_FAN = "PL7"
PIN_ES_X = "ES_X"
PIN_ES_Y = "ES_Y"

# Clock / timing
CLOCK_FREQ = 50_000_000
MIN_STEP_RATE = 50            # E906 min rate, steps/s
MAX_STEP_RATE = 20000         # E906 max rate, steps/s
BATCH_TIME = 0.020            # ~20 ms of motion per E906 MOVE
MAX_BATCH_STEPS = 400         # do not exceed 16-bit-ish reasonable timing
HOMING_POLL_US = 1000         # endstop poll interval during homing
TRSYNC_REPORT_US = 30000      # periodic trsync_state while active
WORKER_PERIOD_US = 500        # worker thread loop

REASON_ENDSTOP_HIT = 1
REASON_HOST_REQUEST = 2
REASON_PAST_END_TIME = 3
REASON_COMMS_TIMEOUT = 4

# ---------------------------------------------------------------------------
# Klipper wire protocol helpers (VLQ, frame, CRC16)
# ---------------------------------------------------------------------------
MESSAGE_MIN = 5
MESSAGE_MAX = 64
MESSAGE_HEADER_SIZE = 2
MESSAGE_TRAILER_SIZE = 3
MESSAGE_POS_LEN = 0
MESSAGE_POS_SEQ = 1
MESSAGE_TRAILER_CRC = 3
MESSAGE_TRAILER_SYNC = 1
MESSAGE_SYNC = 0x7e
MESSAGE_DEST = 0x10


def crc16_ccitt(buf):
    """Standard CCITT-FALSE CRC used by the Klipper serial protocol."""
    crc = 0xFFFF
    for b in buf:
        data = b ^ (crc & 0xFF)
        data = (data ^ (data << 4)) & 0xFF
        crc = (((data << 8) | (crc >> 8)) ^ (data >> 4) ^ (data << 3)) & 0xFFFF
    return crc


def encode_vlq(value, out=None):
    """Encode a signed/unsigned 32-bit integer in Klipper VLQ format."""
    if out is None:
        out = []
    v = value
    if v >= 0xC000000 or v < -0x4000000:
        out.append(((v >> 28) & 0x7F) | 0x80)
    if v >= 0x180000 or v < -0x80000:
        out.append(((v >> 21) & 0x7F) | 0x80)
    if v >= 0x3000 or v < -0x1000:
        out.append(((v >> 14) & 0x7F) | 0x80)
    if v >= 0x60 or v < -0x20:
        out.append(((v >> 7) & 0x7F) | 0x80)
    out.append(v & 0x7F)
    return out


def decode_vlq(data, pos):
    """Decode one VLQ integer from data, return (value, new_pos)."""
    c = data[pos]
    pos += 1
    v = c & 0x7F
    if (c & 0x60) == 0x60:
        v |= -0x20
    while c & 0x80:
        c = data[pos]
        pos += 1
        v = (v << 7) | (c & 0x7F)
    return v, pos


def encode_message(msgid, args, param_types, msgid_bytes=None):
    """Encode a payload message (msgid + args) into a list of byte values."""
    _ = msgid
    out = []
    if msgid_bytes is None:
        raise RuntimeError("msgid_bytes required")
    out.extend(msgid_bytes)
    for (name, typ), value in zip(param_types, args):
        _ = name
        if isinstance(typ, Enumeration):
            typ.encode(out, value)
        elif typ == 'u':
            encode_vlq(value, out)
        elif typ == 'i':
            encode_vlq(value, out)
        elif typ == 'hu':
            encode_vlq(value & 0xFFFF, out)
        elif typ == 'hi':
            v = value & 0xFFFF
            if v & 0x8000:
                v |= -0x10000
            encode_vlq(v, out)
        elif typ == 'c':
            out.append(value & 0xFF)
        elif typ == 's':
            if isinstance(value, str):
                value = value.encode()
            out.append(len(value))
            out.extend(value)
        elif typ == '*s':
            if isinstance(value, str):
                value = value.encode()
            out.append(len(value))
            out.extend(value)
        else:
            raise RuntimeError("unknown type %r" % (typ,))
    return out


class Enumeration:
    def __init__(self, name, mapping):
        self.name = name
        self.mapping = mapping
        self.reverse = {v: k for k, v in mapping.items()}

    def encode(self, out, value):
        if value not in self.mapping:
            raise RuntimeError("unknown enum %s value %r" % (self.name, value))
        encode_vlq(self.mapping[value], out)

    def decode(self, data, pos):
        v, pos = decode_vlq(data, pos)
        return self.reverse.get(v, "?%d" % v), pos


# Parse a Klipper message format string into (name, list of (argname, type/str))
def parse_format(fmt):
    parts = fmt.split()
    name = parts[0]
    params = []
    for part in parts[1:]:
        argname, fstr = part.split('=')
        params.append((argname, fstr))
    return name, params


TYPE_MAP = {'%u': 'u', '%i': 'i', '%hu': 'hu', '%hi': 'hi',
            '%c': 'c', '%s': 's', '%*s': '*s', '%.*s': '*s'}


def format_lookup(fmt, enums):
    name, params = parse_format(fmt)
    out = []
    for argname, fstr in params:
        enum = None
        for ename, emap in enums.items():
            if argname == ename or argname.endswith('_' + ename):
                enum = Enumeration(ename, emap)
                break
        out.append((argname, enum if enum else TYPE_MAP[fstr]))
    return name, out


def decode_args(data, pos, param_types, end):
    out = {}
    for field_name, typ in param_types:
        if pos > end:
            raise RuntimeError("decode overrun")
        if isinstance(typ, Enumeration):
            out[field_name], pos = typ.decode(data, pos)
        elif typ == 'u':
            out[field_name], pos = decode_vlq(data, pos)
            out[field_name] &= 0xFFFFFFFF
        elif typ == 'i':
            out[field_name], pos = decode_vlq(data, pos)
        elif typ == 'hu':
            out[field_name], pos = decode_vlq(data, pos)
            out[field_name] &= 0xFFFF
        elif typ == 'hi':
            out[field_name], pos = decode_vlq(data, pos)
            # VLQ is already signed
        elif typ == 'c':
            out[field_name] = data[pos]
            pos += 1
        elif typ == 's':
            l = data[pos]
            pos += 1
            out[field_name] = bytes(data[pos:pos + l]).decode('utf-8', errors='replace')
            pos += l
        elif typ == '*s':
            l = data[pos]
            pos += 1
            out[field_name] = bytes(data[pos:pos + l])
            pos += l
        else:
            raise RuntimeError("unknown type %r" % (typ,))
    return out, pos


def sign_extend(v, bits):
    if v & (1 << (bits - 1)):
        v -= (1 << bits)
    return v


def frame_message(payload, seq):
    """Return a complete framed message as bytes."""
    # Note: buf contains header + payload only; trailer isn't appended yet,
    # so the CRC must be over the full buf.
    buf = [0] * MESSAGE_HEADER_SIZE
    buf.extend(payload)
    msglen = len(buf) + MESSAGE_TRAILER_SIZE
    buf[MESSAGE_POS_LEN] = msglen
    buf[MESSAGE_POS_SEQ] = seq
    crc = crc16_ccitt(buf)
    crc_hi = crc >> 8
    crc_lo = crc & 0xFF
    buf.extend([crc_hi, crc_lo, MESSAGE_SYNC])
    return bytes(buf)


# Default messages are hardcoded with msgid 0/1 on both sides.
DEFAULT_COMMANDS = {
    "identify offset=%u count=%c": 1,
}
DEFAULT_RESPONSES = {
    "identify_response offset=%u data=%*s": 0,
}


def build_message_dict():
    """Return (commands, responses, enumerations, config) for identify data."""
    enumerations = {
        "pin": {
            PIN_STEP_X: 2, PIN_DIR_X: 3,
            PIN_STEP_Y: 4, PIN_DIR_Y: 5,
            PIN_HEATER: 6, PIN_FAN: 7,
            PIN_ES_X: 16, PIN_ES_Y: 17,
        }
    }

    command_formats = [
        "get_uptime",
        "get_clock",
        "get_config",
        "allocate_oids count=%c",
        "finalize_config crc=%u",
        "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u",
        "reset_step_clock oid=%c clock=%u",
        "set_next_step_dir oid=%c dir=%c",
        "queue_step oid=%c interval=%u count=%hu add=%hi",
        "stepper_get_position oid=%c",
        "config_endstop oid=%c pin=%c pull_up=%c",
        "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c",
        "endstop_query_state oid=%c",
        "config_trsync oid=%c",
        "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c",
        "trsync_set_timeout oid=%c clock=%u",
        "trsync_trigger oid=%c reason=%c",
        "stepper_stop_on_trigger oid=%c trsync_oid=%c",
        "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u",
        "update_digital_out oid=%c value=%c",
        "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u",
        "queue_digital_out oid=%c clock=%u on_ticks=%u",
        "emergency_stop",
        "reset",
        "config_reset",
    ]

    response_formats = [
        "uptime high=%u clock=%u",
        "clock clock=%u",
        "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
        "stepper_position oid=%c pos=%i",
        "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
        "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
    ]

    start_id = 2
    commands = {}
    for i, fmt in enumerate(command_formats):
        commands[fmt] = start_id + i
    start_id += len(command_formats)
    responses = {}
    for i, fmt in enumerate(response_formats):
        responses[fmt] = start_id + i

    # merge defaults so the dictionary is self-contained
    full_commands = dict(DEFAULT_COMMANDS)
    full_commands.update(commands)
    full_responses = dict(DEFAULT_RESPONSES)
    full_responses.update(responses)

    config_constants = {
        "CLOCK_FREQ": CLOCK_FREQ,
        "SERIAL_BAUD": 250000,
        "RECEIVE_WINDOW": 192,
        "STEPPER_BOTH_EDGE": 1,
        "STEPPER_STEP_BOTH_EDGE": 1,
        "STEPPER_OPTIMIZED_UNSTEP": 0,
        "PWM_MAX": 255,
        "ADC_MAX": 1023,
        "STATS_SUMSQ_BASE": 256.0,
    }

    return full_commands, full_responses, enumerations, config_constants


# ---------------------------------------------------------------------------
# /dev/mem mailbox helpers (reuse the step_check.py proven pattern)
# ---------------------------------------------------------------------------
def map_region(base, size=4096):
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


class E906Mailbox:
    """Low-level access to the E906 mailbox command slot."""
    def __init__(self):
        self.mbox, self.mrd, self.mwr = map_region(MBOX_BASE)
        self.gpio, self.grd, self.gwr = map_region(S_GPIO_BASE)
        self.lock = threading.Lock()
        self.last_ack = 0

    def check_magic(self):
        magic = self.mrd(0x00)
        version = self.mrd(0x04)
        if magic != 0xE9061B0B:
            raise RuntimeError("E906 firmware magic not found @ 0x60000000 "
                               "(got 0x%08X)" % magic)
        if version != 8:
            raise RuntimeError("E906 mailbox version %d != 8" % version)
        return True

    def sync_slot(self):
        """Poll until any previous command is consumed."""
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if self.mrd(CMD_OFF) == 0:
                return True
            time.sleep(0.0005)
        return False

    def send_cmd(self, code, a0=0, a1=0, a2=0, timeout=1.0):
        with self.lock:
            if not self.sync_slot():
                raise RuntimeError("mailbox command slot busy (timeout)")
            self.mwr(ARG0_OFF, a0)
            self.mwr(ARG1_OFF, a1)
            self.mwr(ARG2_OFF, a2)
            self.mwr(CMD_OFF, code)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self.mrd(CMD_OFF) == 0:
                    return self.mrd(RESULT_OFF)
                time.sleep(0.0005)
            raise RuntimeError("mailbox command %d timed out" % code)

    def pos_x(self):
        v = self.mrd(POS_X_OFF)
        if v & 0x80000000:
            v -= 0x100000000
        return v

    def pos_y(self):
        v = self.mrd(POS_Y_OFF)
        if v & 0x80000000:
            v -= 0x100000000
        return v

    def endstop_state(self):
        return self.mrd(ENDSTOP_OFF) & 0x3

    def es_override(self, bits):
        for _ in range(3):
            self.mwr(ES_OVR, bits)
            time.sleep(0.001)


# ---------------------------------------------------------------------------
# Bridge state / protocol translator
# ---------------------------------------------------------------------------
class E906Bridge:
    def __init__(self, fifo_path=FIFO):
        self.fifo_path = fifo_path
        self.e906 = E906Mailbox()
        self.lock = threading.Lock()

        self.commands, self.responses, self.enums, self.config_consts = build_message_dict()
        self._msg_formats = {}
        self._format_by_id = {}
        self._param_types = {}
        self._msgid_bytes = {}

        def add_format(fmt, mid, is_cmd):
            name, params = format_lookup(fmt, self.enums)
            self._msg_formats[name] = (fmt, mid, params, is_cmd)
            self._format_by_id[mid] = (fmt, name, params, is_cmd)
            self._param_types[fmt] = params
            b = []
            encode_vlq(mid, b)
            self._msgid_bytes[fmt] = b

        for is_cmd, d in [(True, self.commands), (False, self.responses)]:
            for fmt, mid in d.items():
                add_format(fmt, mid, is_cmd)

        # The identify command/response are hardcoded at msgid 1/0 by Klipper.
        add_format("identify offset=%u count=%c", 1, True)
        add_format("identify_response offset=%u data=%*s", 0, False)

        # Binarize the dictionary once.  Keep it compact so it fits in a
        # handful of 64-byte identify_response chunks.
        dict_obj = {
            "version": "v0.13.0-e906",  # arbitrary but informative
            "build_versions": "e906-bridge",
            "config": self.config_consts,
            "commands": self.commands,
            "responses": self.responses,
            "output": {},
            "enumerations": self.enums,
        }
        raw_json = json.dumps(dict_obj, separators=(',', ':')).encode()
        self.identify_data = zlib.compress(raw_json, 9)
        self.identify_len = len(self.identify_data)
        # Klipper cannot accept a single message larger than 64 bytes, so we
        # limit identify_response chunks.  With a 40-byte payload we are well
        # below the 59-byte payload maximum.
        self.identify_chunk = 40

        self.fd = None
        self.fd_write = None
        self.slave_fd = None
        self.next_sequence = MESSAGE_DEST
        self.last_sent_seq = None
        self.rx_synced = False
        self.running = True
        self.rx_buf = bytearray()

        # State
        self.oid_count = 0
        self.config_crc = 0
        self.config_finalized = False
        self.is_shutdown = False

        # Steppers: oid -> {'axis': 0/1, 'dir': 0/1, ...}
        self.steppers = {}

        # Pending motion buckets per axis: sum of steps and ticks
        self.pending = {
            0: {'steps': 0, 'ticks': 0, 'dir': 0, 'empty': True},
            1: {'steps': 0, 'ticks': 0, 'dir': 0, 'empty': True},
        }

        # Endstops: oid -> {'pin': 16/17, 'invert': 0/1, ...}
        self.endstops = {}

        # Trsyncs: oid -> active dict with report_clock/report_ticks/expire_clock
        self.trsyncs = {}

        # Active homing: list of {'endstop_oid': ..., 'trsync_oid': ..., 'pin_value': 0/1,
        #                         'trigger_reason': ..., 'axis': 0/1? derived from endstop}
        self.homing = []

        # Digital outputs / PWM cache: oid -> {'pin': ..., 'invert': bool,
        #                                      'duty': 0..255, 'pwm_max': cycle_ticks}
        self.digital_outs = {}

        # Virtual clock
        self.t0_monotonic = time.monotonic()
        self.last_report_clock = {0: 0, 1: 0}

    # ------------------------------------------------------------------
    # Clock helpers (50 MHz, starting at 0 at bridge start)
    # ------------------------------------------------------------------
    def clock(self):
        return int((time.monotonic() - self.t0_monotonic) * CLOCK_FREQ)

    def uptime(self):
        c = self.clock()
        hi = (c >> 32) & 0xFFFFFFFF
        lo = c & 0xFFFFFFFF
        return hi, lo

    # ------------------------------------------------------------------
    # PTY I/O
    # ------------------------------------------------------------------
    def open_pty(self):
        """Create a PTY and expose it through /tmp/klipper_host_e906.

        We keep the slave fd open in the bridge so the PTY never reports EIO
        when Klipper closes and reopens its side; the bridge never reads from
        the slave, so it does not see its own writes echoed back.
        """
        if os.path.islink(self.fifo_path) or os.path.exists(self.fifo_path):
            os.unlink(self.fifo_path)
        master, slave = os.openpty()
        slave_path = os.ttyname(slave)
        # Disable all line discipline processing on the PTY.  This is required
        # so that raw Klipper frames (which may contain XON/XOFF and other
        # control bytes) pass through unchanged in both directions.
        try:
            tty.setraw(slave, termios.TCSANOW)
        except termios.error:
            pass
        # Make the pts device accessible to whatever user runs Klipper.
        try:
            os.chmod(slave_path, 0o666)
        except OSError:
            pass
        os.symlink(slave_path, self.fifo_path)
        self.fd = master
        self.fd_write = master
        self.slave_fd = slave
        self.slave_path = slave_path

    def close_fifo(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
            self.fd_write = None
        if getattr(self, 'slave_fd', None) is not None:
            os.close(self.slave_fd)
            self.slave_fd = None
        if os.path.islink(self.fifo_path):
            try:
                os.unlink(self.fifo_path)
            except OSError:
                pass

    def write_frame(self, payload, seq=None):
        """Thread-safe write of one framed message."""
        if seq is None:
            seq = self.next_sequence
        frame = frame_message(payload, seq)
        with self.lock:
            if self.fd is not None:
                try:
                    os.write(self.fd_write, frame)
                    self.last_sent_seq = seq
                except BrokenPipeError:
                    pass

    def send_ack(self):
        self.write_frame([], self.next_sequence)

    def send_response(self, fmt, args):
        params = self._param_types[fmt]
        payload = encode_message(self._format_by_id,
                                 args, params,
                                 self._msgid_bytes[fmt])
        self.write_frame(payload)

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------
    def handle_command(self, fmt, args):
        if fmt == "identify offset=%u count=%c":
            return self.cmd_identify(args['offset'], args['count'])
        if fmt == "get_uptime":
            hi, lo = self.uptime()
            return self.send_response("uptime high=%u clock=%u", [hi, lo])
        if fmt == "get_clock":
            return self.send_response("clock clock=%u", [self.clock() & 0xFFFFFFFF])
        if fmt == "get_config":
            return self.send_response(
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
                [1 if self.config_finalized else 0,
                 self.config_crc & 0xFFFFFFFF,
                 1 if self.is_shutdown else 0,
                 500])
        if fmt == "allocate_oids count=%c":
            self.oid_count = args['count']
            return
        if fmt == "finalize_config crc=%u":
            self.config_crc = args['crc']
            self.config_finalized = True
            return
        if fmt == "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u":
            self.steppers[args['oid']] = {
                'axis': 0 if args['step_pin'] == PIN_STEP_X else 1,
                'dir': 0,
                'invert_step': args['invert_step'],
            }
            return
        if fmt == "reset_step_clock oid=%c clock=%u":
            return
        if fmt == "set_next_step_dir oid=%c dir=%c":
            s = self.steppers.get(args['oid'])
            if s is None:
                return
            axis = s['axis']
            # flush any pending steps in the old direction first
            self.flush_axis(axis)
            s['dir'] = args['dir']
            return
        if fmt == "queue_step oid=%c interval=%u count=%hu add=%hi":
            self.cmd_queue_step(args['oid'], args['interval'], args['count'], args['add'])
            return
        if fmt == "stepper_get_position oid=%c":
            pos = self.e906.pos_x() if self.steppers.get(args['oid'], {}).get('axis') == 0 else self.e906.pos_y()
            # Klipper position uses a 0x40000000 bias internally; we mirror
            # the E906 counters as raw so the relative step count is correct.
            return self.send_response("stepper_position oid=%c pos=%i", [args['oid'], pos])
        if fmt == "config_endstop oid=%c pin=%c pull_up=%c":
            pin_val = self.enums['pin'].get(args['pin'])
            axis = 0 if pin_val in (2, 16) else 1
            self.endstops[args['oid']] = {
                'pin': args['pin'],
                'axis': axis,
                'pull_up': args['pull_up'],
            }
            return
        if fmt == "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c":
            st = self.endstops.get(args['oid'])
            if st is None:
                return
            # Clear any pending virtual endstop state before homing
            self.e906.es_override(0)
            # Stop stepping for this axis when trsync fires
            for stepper in self.steppers.values():
                if stepper['axis'] == st['axis']:
                    # flush prior moves so the new homing move runs cleanly
                    self.flush_axis(st['axis'])
                    self.e906.send_cmd(CMD_FLUSH, 0, 0, 0)
                    break
            entry = {
                'endstop_oid': args['oid'],
                'trsync_oid': args['trsync_oid'],
                'pin_value': args['pin_value'],
                'trigger_reason': args['trigger_reason'],
                'axis': st['axis'],
                'clock': args['clock'],
                'sample_time': args['sample_ticks'],
                'sample_count': args['sample_count'],
                'rest_ticks': args['rest_ticks'],
                'samples': 0,
            }
            # replace any previous homing for same endstop
            self.homing = [h for h in self.homing if h['endstop_oid'] != args['oid']]
            if args['sample_count'] != 0 or args['trsync_oid'] != 0:
                self.homing.append(entry)
            return
        if fmt == "endstop_query_state oid=%c":
            st = self.endstops.get(args['oid'])
            es = self.e906.endstop_state()
            val = 0
            if st is not None:
                val = 1 if (es & (1 << st['axis'])) else 0
            return self.send_response(
                "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
                [args['oid'], 0, self.clock() & 0xFFFFFFFF, val])
        if fmt == "config_trsync oid=%c":
            self.trsyncs[args['oid']] = {
                'active': False,
                'report_clock': 0,
                'report_ticks': 0,
                'expire_reason': 0,
                'expire_clock': 0,
                'oid': args['oid'],
                'trigger_reason': 0,
                'triggered': False,
            }
            return
        if fmt == "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c":
            ts = self.trsyncs.get(args['oid'])
            if ts:
                ts['active'] = True
                ts['report_clock'] = args['report_clock']
                ts['report_ticks'] = args['report_ticks']
                ts['expire_reason'] = args['expire_reason']
                ts['expire_clock'] = args['report_clock'] + 10 * CLOCK_FREQ  # generous
                ts['triggered'] = False
                ts['trigger_reason'] = 0
            return
        if fmt == "trsync_set_timeout oid=%c clock=%u":
            ts = self.trsyncs.get(args['oid'])
            if ts:
                ts['expire_clock'] = args['clock']
            return
        if fmt == "trsync_trigger oid=%c reason=%c":
            return self.cmd_trsync_trigger(args['oid'], args['reason'])
        if fmt == "stepper_stop_on_trigger oid=%c trsync_oid=%c":
            # Tracked implicitly when the trsync fires and we FLUSH the E906.
            return
        if fmt == "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u":
            self.digital_outs[args['oid']] = {
                'pin': args['pin'],
                'value': args['value'],
                'invert': False,
                'cycle_ticks': None,
                'duty': 0 if args['value'] == 0 else 255,
            }
            # Apply initial value immediately
            self.update_digital_out(args['oid'])
            return
        if fmt == "update_digital_out oid=%c value=%c":
            d = self.digital_outs.get(args['oid'])
            if d:
                d['duty'] = 0 if args['value'] == 0 else 255
                self.update_digital_out(args['oid'])
            return
        if fmt == "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u":
            d = self.digital_outs.get(args['oid'])
            if d:
                d['cycle_ticks'] = args['cycle_ticks']
            return
        if fmt == "queue_digital_out oid=%c clock=%u on_ticks=%u":
            d = self.digital_outs.get(args['oid'])
            if d and d.get('cycle_ticks'):
                max_ticks = d['cycle_ticks']
                if max_ticks:
                    duty255 = int(min(255, (args['on_ticks'] * 255) // max_ticks))
                    d['duty'] = duty255
                    self.update_digital_out(args['oid'])
            elif d:
                d['duty'] = 255 if args['on_ticks'] else 0
                self.update_digital_out(args['oid'])
            return
        if fmt == "emergency_stop":
            self.is_shutdown = True
            self.e906.send_cmd(CMD_FLUSH, 0, 0, 0)
            return
        if fmt == "reset":
            self.reset_state()
            self.e906.send_cmd(CMD_FLUSH, 0, 0, 0)
            return
        if fmt == "config_reset":
            self.reset_state()
            self.e906.send_cmd(CMD_FLUSH, 0, 0, 0)
            return

    def cmd_identify(self, offset, count):
        chunk = self.identify_data[offset:offset + min(count, self.identify_chunk)]
        self.send_response("identify_response offset=%u data=%*s",
                           [offset, chunk])

    def cmd_queue_step(self, oid, interval, count, add):
        s = self.steppers.get(oid)
        if s is None or count == 0:
            return
        # Total ticks for this burst per stepcompress semantics:
        # sum_{k=0..count-1} (interval + k*add)
        ticks_total = count * interval + add * (count * (count - 1) // 2)
        if ticks_total <= 0:
            # Klipper may send zero-interval moves at start-up; treat as minimum
            ticks_total = count
        axis = s['axis']
        direction = 1 if s['dir'] else -1
        bucket = self.pending[axis]
        if bucket['empty']:
            bucket['steps'] = 0
            bucket['ticks'] = 0
            bucket['empty'] = False
        bucket['steps'] += direction * count
        bucket['ticks'] += ticks_total
        if abs(bucket['steps']) >= MAX_BATCH_STEPS or bucket['ticks'] >= int(BATCH_TIME * CLOCK_FREQ):
            self.flush_axis(axis)

    def flush_axis(self, axis):
        bucket = self.pending[axis]
        if bucket['empty'] or bucket['steps'] == 0 or bucket['ticks'] == 0:
            bucket['empty'] = True
            bucket['steps'] = 0
            bucket['ticks'] = 0
            return
        steps = bucket['steps']
        ticks = bucket['ticks']
        bucket['empty'] = True
        bucket['steps'] = 0
        bucket['ticks'] = 0
        avg_interval = ticks / abs(steps)
        if avg_interval <= 0:
            return
        rate = int(CLOCK_FREQ / avg_interval)
        rate = max(MIN_STEP_RATE, min(MAX_STEP_RATE, rate))
        # Round steps to int; preserve sign
        isteps = steps
        # Push to E906 command slot; the worker loop will retry on busy.
        self.qadd(axis, isteps, rate)

    def qadd(self, axis, steps, rate):
        """Enqueue one MOVE on the E906, blocking-retry on busy/full."""
        try:
            res = self.e906.send_cmd(CMD_QADD, axis, steps, rate, timeout=0.05)
            if res == RESULT_BUSY:
                # Retry once after a short pause
                time.sleep(0.001)
                res = self.e906.send_cmd(CMD_QADD, axis, steps, rate, timeout=0.05)
            if res != RESULT_OK:
                sys.stderr.write("QADD failed: axis=%d steps=%d rate=%d res=%d\n"
                                 % (axis, steps, rate, res))
        except Exception as e:
            sys.stderr.write("QADD exception: %s\n" % e)

    def update_digital_out(self, oid):
        d = self.digital_outs.get(oid)
        if d is None:
            return
        ch = 0 if d['pin'] == PIN_HEATER else 1
        duty = max(0, min(255, d['duty']))
        try:
            self.e906.send_cmd(CMD_SET, ch, duty, 0, timeout=0.05)
        except Exception as e:
            sys.stderr.write("SET failed: %s\n" % e)

    def cmd_trsync_trigger(self, oid, reason):
        ts = self.trsyncs.get(oid)
        if ts is None:
            return self.send_response(
                "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
                [oid, 0, reason, self.clock() & 0xFFFFFFFF])
        ts['triggered'] = True
        ts['active'] = False
        ts['trigger_reason'] = reason
        # For HOST_REQUEST Klipper expects the endstop-hit reason if we
        # actually homed; otherwise report the requested reason.
        reported_reason = reason
        if reason == REASON_HOST_REQUEST and ts.get('endstop_hit_reason'):
            reported_reason = ts['endstop_hit_reason']
        return self.send_response(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            [oid, 0, reported_reason, self.clock() & 0xFFFFFFFF])

    def reset_state(self):
        self.config_crc = 0
        self.config_finalized = False
        self.is_shutdown = False
        self.steppers.clear()
        self.endstops.clear()
        self.trsyncs.clear()
        self.homing = []
        self.digital_outs.clear()
        for b in self.pending.values():
            b['steps'] = 0
            b['ticks'] = 0
            b['empty'] = True

    # ------------------------------------------------------------------
    # Worker / periodic tasks
    # ------------------------------------------------------------------
    def do_homing_poll(self):
        if not self.homing:
            return
        es = self.e906.endstop_state()
        clock32 = self.clock() & 0xFFFFFFFF
        for h in list(self.homing):
            val = 1 if (es & (1 << h['axis'])) else 0
            if val == h['pin_value']:
                h['samples'] += 1
                # require only 1 sample for now (E906 already debounces in main loop)
                if h['samples'] >= max(1, h.get('sample_count', 1)):
                    # Endstop hit: stop all E906 motion and report trigger
                    try:
                        self.e906.send_cmd(CMD_FLUSH, 0, 0, 0, timeout=0.05)
                    except Exception:
                        pass
                    # Update trsync state
                    ts = self.trsyncs.get(h['trsync_oid'])
                    if ts:
                        ts['triggered'] = True
                        ts['active'] = False
                        ts['trigger_reason'] = h['trigger_reason']
                        ts['endstop_hit_reason'] = h['trigger_reason']
                        self.send_response(
                            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
                            [h['trsync_oid'], 0, h['trigger_reason'], clock32])
                    self.homing = [hh for hh in self.homing if hh['endstop_oid'] != h['endstop_oid']]
            else:
                h['samples'] = 0

    def do_trsync_reports(self):
        clock32 = self.clock() & 0xFFFFFFFF
        for ts in self.trsyncs.values():
            if not ts.get('active') or ts.get('triggered'):
                continue
            # optional timeout handling
            if ts['expire_clock'] != 0 and ((clock32 - ts['expire_clock']) & 0xFFFFFFFF) < 0x80000000:
                # Expired
                ts['triggered'] = True
                ts['active'] = False
                ts['trigger_reason'] = ts['expire_reason']
                self.send_response(
                    "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
                    [ts['oid'], 0, ts['expire_reason'], clock32])
                continue
            if ts['report_ticks']:
                last = self.last_report_clock.get(ts['oid'], 0)
                delta = (clock32 - last) & 0xFFFFFFFF
                if delta >= ts['report_ticks']:
                    self.last_report_clock[ts['oid']] = clock32
                    self.send_response(
                        "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
                        [ts['oid'], 1, 0, clock32])

    def worker(self):
        while self.running:
            # Flush any stale bucket at the end of a move stream
            for axis in (0, 1):
                self.flush_axis(axis)
            self.do_homing_poll()
            self.do_trsync_reports()
            time.sleep(WORKER_PERIOD_US / 1_000_000.0)

    # ------------------------------------------------------------------
    # Main reader loop
    # ------------------------------------------------------------------
    def read_bytes(self):
        try:
            data = os.read(self.fd, MESSAGE_MAX * 4)
            if not data:
                return False
            self.rx_buf.extend(data)
            return True
        except OSError as e:
            if e.errno != errno.EAGAIN:
                sys.stderr.write("read error: %s\n" % e)
                return False
            return True

    def find_block(self):
        """Return (block_bytes, msgseq) or (None, None) if not enough data."""
        while True:
            if len(self.rx_buf) < MESSAGE_MIN:
                return None, None
            msglen = self.rx_buf[MESSAGE_POS_LEN]
            if msglen < MESSAGE_MIN or msglen > MESSAGE_MAX:
                # resync to next SYNCH
                try:
                    idx = self.rx_buf.index(MESSAGE_SYNC, 1)
                except ValueError:
                    self.rx_buf.clear()
                    return None, None
                del self.rx_buf[:idx + 1]
                continue
            if len(self.rx_buf) < msglen:
                return None, None
            if self.rx_buf[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC:
                del self.rx_buf[:1]
                continue
            msgcrc = ((self.rx_buf[msglen - MESSAGE_TRAILER_CRC] << 8)
                      | self.rx_buf[msglen - MESSAGE_TRAILER_CRC + 1])
            crc = crc16_ccitt(self.rx_buf[:msglen - MESSAGE_TRAILER_SIZE])
            if crc != msgcrc:
                del self.rx_buf[:1]
                continue
            block = bytes(self.rx_buf[:msglen])
            msgseq = self.rx_buf[MESSAGE_POS_SEQ]
            del self.rx_buf[:msglen]
            return block, msgseq

    def process_block(self, block, msgseq):
        if (msgseq & ~0x0F) != MESSAGE_DEST:
            return
        if not self.rx_synced:
            self.rx_synced = True
            self.next_sequence = ((msgseq + 1) & 0x0F) | MESSAGE_DEST
            self.last_sent_seq = None
        elif msgseq != self.next_sequence:
            # NAK out-of-order
            self.write_frame([], self.next_sequence)
            return
        else:
            self.next_sequence = ((msgseq + 1) & 0x0F) | MESSAGE_DEST
            self.last_sent_seq = None
        pos = MESSAGE_HEADER_SIZE
        payload_end = len(block) - MESSAGE_TRAILER_SIZE
        while pos < payload_end:
            msgid, pos = decode_vlq(block, pos)
            entry = self._format_by_id.get(msgid)
            if entry is None:
                break
            fmt, name, params, is_cmd = entry
            args, pos = decode_args(block, pos, params, payload_end)
            try:
                self.handle_command(fmt, args)
            except Exception as e:
                sys.stderr.write("Exception handling %s: %s\n" % (fmt, e))
                import traceback
                traceback.print_exc()
        # Always emit an empty ACK at the current expected sequence number.
        # Klipper's serialqueue uses the empty ACK to complete the
        # notification in raw_send_wait_ack(); a data response alone does not
        # wake the waiting caller.  If we already sent a data response at this
        # sequence number the empty frame is a harmless duplicate ACK.
        self.send_ack()
        self.last_sent_seq = None

    def run(self):
        try:
            self.e906.check_magic()
        except RuntimeError as e:
            sys.stderr.write("%s\n" % e)
            sys.exit(1)
        # Leave virtual endstop clean
        self.e906.es_override(0)
        self.open_pty()
        try:
            worker_thread = threading.Thread(target=self.worker, daemon=True)
            worker_thread.start()
            sys.stderr.write("E906 bridge listening on %s -> %s\n" % (self.fifo_path, self.slave_path))
            while self.running:
                if not self.read_bytes():
                    time.sleep(0.001)
                    continue
                while True:
                    block, msgseq = self.find_block()
                    if block is None:
                        break
                    self.process_block(block, msgseq)
        finally:
            self.close_fifo()


def main():
    if os.geteuid() != 0:
        sys.stderr.write("Run as root (needs /dev/mem)\n")
        sys.exit(1)
    bridge = E906Bridge()

    def on_sig(signum, frame):
        bridge.running = False

    signal.signal(signal.SIGTERM, on_sig)
    signal.signal(signal.SIGINT, on_sig)
    try:
        bridge.run()
    finally:
        bridge.close_fifo()


if __name__ == "__main__":
    main()
