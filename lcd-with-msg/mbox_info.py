#!/usr/bin/env python3
"""ARM -> E906 info daemon.

Writes load/IP/uptime text into the E906 mailbox (reserved DDR @ 0x60000000)
every 2 seconds.  Layout:
  +0x40  seq      (incremented each update)
  +0x44  text[4][32]  (ASCII, line per row)
"""
import mmap
import struct
import subprocess
import time

BASE = 0x60000000
SEQ_OFF = 0x40
TXT_OFF = 0x44
LINES = 4
LLEN = 32


def get_lines():
    load = open("/proc/loadavg").read().split()[:3]
    l0 = ("LOAD  " + " ".join(load)).ljust(LLEN, " ")
    try:
        ip = subprocess.run(["hostname", "-I"],
                            capture_output=True, text=True).stdout.split()[0]
    except Exception:
        ip = "?.?.?.?"
    l1 = ("IP    " + ip).ljust(LLEN, " ")
    up = float(open("/proc/uptime").read().split()[0])
    h, m = int(up // 3600), int((up % 3600) // 60)
    l2 = ("UP    %02d:%02d" % (h, m)).ljust(LLEN, " ")
    l3 = "E906-LCD MAILBOX v2".ljust(LLEN, " ")
    return [l0, l1, l2, l3]


with open("/dev/mem", "r+b") as f:
    m = mmap.mmap(f.fileno(), 0x200, offset=BASE)
    seq = 0
    while True:
        lines = get_lines()
        seq += 1
        struct.pack_into("<I", m, SEQ_OFF, seq)
        for i, s in enumerate(lines):
            b = s.encode()[:LLEN].ljust(LLEN, b" ")
            # /dev/mem is mapped as device memory: wide or misaligned
            # accesses to this carveout cause SIGBUS, so write only
            # 4-byte-aligned words.
            for w in range(LLEN // 4):
                v = int.from_bytes(b[w * 4:w * 4 + 4], "little")
                struct.pack_into("<I", m, TXT_OFF + i * LLEN + w * 4, v)
        time.sleep(2)
