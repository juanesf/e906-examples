#!/usr/bin/env python3
"""ARM -> E906 system-monitor daemon (mailbox v3).

Pushes structured metrics into the E906 mailbox (reserved DDR @ 0x60000000)
every second.  The E906 renders a dashboard on the ST7789V LCD.

Layout (all offsets are 4-byte aligned; /dev/mem is device memory so only
aligned 32-bit writes are allowed, anything else SIGBUSes):
  +0x00  magic      0xE9061B0B   (written by E906)
  +0x04  version    3            (written by E906)
  +0x08  flags      0x0000B00B   (written by E906)
  +0x0C  e906_cnt                (written by E906)
  +0x10  arm_cnt                 (bumped each push)
  +0x14  status                  (bit0 = data valid)
  +0x18  load1_x1000
  +0x1C  load5_x1000
  +0x20  load15_x1000
  +0x24  mem_total_KB
  +0x28  mem_used_KB
  +0x2C  temp_mC                 (millidegrees Celsius)
  +0x30  uptime_s
  +0x34  clock_packed            ((hh<<16)|(mm<<8)|ss)
  +0x38  spare
  +0x3C  spare
  +0x40  arm_seq                 (bumped with the text lines)
  +0x44  text[4][32]             (hostname / IP / build / status)
"""
import mmap
import os
import struct
import subprocess
import time

BASE = 0x60000000
MAP_LEN = 0x200
OFF = {
    "arm_cnt": 0x10,
    "status": 0x14,
    "load1": 0x18,
    "load5": 0x1C,
    "load15": 0x20,
    "mem_total": 0x24,
    "mem_used": 0x28,
    "temp": 0x2C,
    "uptime": 0x30,
    "clock": 0x34,
    "seq": 0x40,
    "text": 0x44,
}
TEXT_LINES = 4
LLEN = 32
BANNER = "E906 SYS MON v3"


def _val(path, f=float):
    try:
        with open(path) as fh:
            return f(fh.read().split()[0])
    except Exception:
        return 0.0


def read_load():
    parts = open("/proc/loadavg").read().split()[:3]
    return [int(float(p) * 1000) for p in parts]


def read_mem():
    info = {}
    with open("/proc/meminfo") as fh:
        for line in fh:
            if line.startswith("MemTotal:"):
                info["total"] = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                info["avail"] = int(line.split()[1])
    total = info.get("total", 0)
    avail = info.get("avail", 0)
    return total, max(0, total - avail)


def read_temp_mc():
    """Return the CPU temperature in millidegrees Celsius.

    Prefers the CPU thermal zones (cpu4/cpu0), reporting the hottest one.
    Falls back to the hottest reading across all thermal zones, then to
    the hwmon temp inputs.
    """
    base = "/sys/class/thermal"
    cpu = []
    allz = []
    try:
        for z in sorted(os.listdir(base)):
            tz = os.path.join(base, z, "temp")
            if not os.path.exists(tz):
                continue
            v = _val(tz, int)
            if not v:
                continue
            allz.append(v)
            try:
                typ = open(os.path.join(base, z, "type")).read().strip()
            except OSError:
                typ = ""
            if typ.startswith("cpu"):
                cpu.append(v)
    except OSError:
        pass
    if cpu:
        return max(cpu)
    if allz:
        return max(allz)
    best = None
    try:
        for h in sorted(os.listdir("/sys/class/hwmon")):
            inp = os.path.join("/sys/class/hwmon", h, "temp1_input")
            if not os.path.exists(inp):
                continue
            v = _val(inp, int)
            if v and (best is None or v > best):
                best = v
    except OSError:
        pass
    return best or 0


def get_clock_packed():
    lt = time.localtime()
    return (lt.tm_hour << 16) | (lt.tm_min << 8) | lt.tm_sec


def get_text_lines():
    try:
        host = open("/etc/hostname").read().strip() or "armbian"
    except OSError:
        host = "armbian"
    try:
        ip = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True,
            timeout=2).stdout.split()[0]
    except Exception:
        ip = "?.?.?.?"
    return [host, ip, BANNER, time.strftime("%a %d %b %Y")]


with open("/dev/mem", "r+b") as f:
    m = mmap.mmap(f.fileno(), MAP_LEN, offset=BASE)
    arm_cnt = 0
    seq = 0
    while True:
        try:
            l1, l5, l15 = read_load()
            mem_total, mem_used = read_mem()
            temp = read_temp_mc()
            up = int(_val("/proc/uptime"))
            clk = get_clock_packed()

            struct.pack_into("<I", m, OFF["status"], 1)
            struct.pack_into("<I", m, OFF["arm_cnt"], arm_cnt)
            struct.pack_into("<III", m, OFF["load1"], l1, l5, l15)
            struct.pack_into("<II", m, OFF["mem_total"], mem_total, mem_used)
            struct.pack_into("<II", m, OFF["temp"], temp, up)
            struct.pack_into("<I", m, OFF["clock"], clk)

            seq += 1
            arm_cnt += 1
            struct.pack_into("<I", m, OFF["seq"], seq)
            for i, s in enumerate(get_text_lines()):
                b = s.encode()[:LLEN].ljust(LLEN, b" ")
                for w in range(LLEN // 4):
                    v = int.from_bytes(b[w * 4:w * 4 + 4], "little")
                    struct.pack_into("<I", m, OFF["text"] + i * LLEN + w * 4, v)
        except Exception as exc:
            pass  # keep the loop alive across transient read errors
        time.sleep(1)
