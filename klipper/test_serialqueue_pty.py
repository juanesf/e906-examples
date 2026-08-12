#!/home/avaota/klippy-env/bin/python
"""Test real Klipper serialqueue against our bridge over a PTY.

Creates a PTY, launches a tiny firmware thread that只 responds to identify,
and then lets Klipper's C serialqueue connect to the slave side.  The goal is
to reproduce exactly the same topology as production:

   serialqueue (host)  <->  PTY slave symlink  <->  bridge thread (master)
"""
import fcntl
import json
import os
import socket
import struct
import sys
import termios
import threading
import time
import tty
import zlib

sys.path.insert(0, '/home/avaota/klipper/klippy')
import chelper

sys.path.insert(0, os.path.dirname(__file__))
from e906_mcu_bridge import (
    build_message_dict, encode_message, format_lookup, frame_message,
    MESSAGE_DEST, MESSAGE_MIN,
)


def make_msgid(v):
    out = []
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


def encode_cmd(fmt, args, msgid):
    name, params = format_lookup(fmt, {})
    return encode_message(None, args, params, make_msgid(msgid))


def firmware_thread(master_fd, stop_event):
    """Minimal firmware: read blocks on master, reply to identify."""
    commands, responses, enums, config_consts = build_message_dict()
    identify_id = commands["identify offset=%u count=%c"]
    ident_dict = {
        "version": "v0.13.0-e906",
        "build_versions": "e906-bridge",
        "config": config_consts,
        "commands": commands,
        "responses": responses,
        "output": {},
    }
    ident_compressed = zlib.compress(
        json.dumps(ident_dict, separators=(',', ':')).encode(), 9)

    rx_buf = bytearray()
    next_seq = MESSAGE_DEST
    while not stop_event.is_set():
        try:
            chunk = os.read(master_fd, 256)
        except OSError as e:
            if e.errno != os.errno.EAGAIN:
                sys.stderr.write("firmware read error: %s\n" % e)
            time.sleep(0.001)
            continue
        if not chunk:
            time.sleep(0.001)
            continue
        rx_buf.extend(chunk)
        # Minimal block parse: find 0x7e aligned frames.
        while True:
            if len(rx_buf) < MESSAGE_MIN:
                break
            if rx_buf[-1] != 0x7e:
                # discard until last sync
                try:
                    idx = rx_buf.rindex(0x7e)
                    del rx_buf[:idx]
                    continue
                except ValueError:
                    rx_buf.clear()
                    break
            # find previous sync
            try:
                start = rx_buf[:-1].rindex(0x7e) + 1
            except ValueError:
                start = 0
            block = bytes(rx_buf[start:])
            if len(block) < MESSAGE_MIN or block[0] > 64:
                rx_buf.clear()
                break
            # Validate length/CRC roughly
            msglen = block[0]
            if msglen <= len(block):
                # Respond: identify_response with same seq as ack
                seq = block[1]
                if seq == 0x10 or seq == 0x11 or seq == 0x12:
                    resp_seq = ((seq + 1) & 0x0F) | MESSAGE_DEST
                    payload = encode_cmd(
                        "identify_response offset=%u data=%*s",
                        [0, ident_compressed[:40]], 0)
                    resp = frame_message(payload, resp_seq)
                    os.write(master_fd, resp)
            del rx_buf[start:]


def main():
    path = "/tmp/klipper_host_e906_test"
    if os.path.islink(path) or os.path.exists(path):
        os.unlink(path)

    master, slave = os.openpty()
    slave_path = os.ttyname(slave)
    tty.setraw(slave, termios.TCSANOW)
    os.chmod(slave_path, 0o666)
    os.symlink(slave_path, path)
    print("PTY master=%d slave=%s link=%s" % (master, slave_path, path))

    stop_event = threading.Event()
    t = threading.Thread(target=firmware_thread, args=(master, stop_event),
                         daemon=True)
    t.start()

    ffi_main, ffi_lib = chelper.get_ffi()
    sq = ffi_lib.serialqueue_alloc(master, b'u', 0, b'sqtest')
    ffi_lib.serialqueue_set_wire_frequency(sq, 250000)
    cq = ffi_lib.serialqueue_alloc_commandqueue()

    cmd = frame_message(
        encode_cmd("identify offset=%u count=%c", [0, 40],
                   build_message_dict()[0]["identify offset=%u count=%c"]),
        MESSAGE_DEST | 1)

    buf = ffi_main.new('uint8_t[]', len(cmd))
    for i, b in enumerate(cmd):
        buf[i] = b
    ffi_lib.serialqueue_send(sq, cq, buf, len(cmd), 0, 0, 1)

    pqm = ffi_main.new('struct pull_queue_message *')
    deadline = time.monotonic() + 3
    got_data = False
    got_notify = False
    while time.monotonic() < deadline:
        ffi_lib.serialqueue_pull(sq, pqm)
        if pqm.len < 0:
            print("pull break (len<0)")
            break
        if pqm.len > 0 or pqm.notify_id:
            print("pull len=%d notify_id=%d sent=%f recv=%f" % (
                pqm.len, pqm.notify_id, pqm.sent_time, pqm.receive_time))
        if pqm.len > 0:
            data = bytes(ffi_main.buffer(pqm.msg, pqm.len))
            print("  data", data.hex())
            got_data = True
        if pqm.notify_id:
            got_notify = True
        if got_data and got_notify:
            break
        time.sleep(0.01)

    print("RESULT got_data=%s got_notify=%s" % (got_data, got_notify))

    stop_event.set()
    os.close(master)
    os.close(slave)
    os.unlink(path)
    ffi_lib.serialqueue_free(sq)
    ffi_lib.serialqueue_free_commandqueue(cq)


if __name__ == "__main__":
    main()
