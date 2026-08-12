#!/usr/bin/env python3
"""Minimal host-side Klipper serial client for the E906 bridge PTY.

Opens /tmp/klipper_host_e906, performs the identify handshake, decompresses the
identity dictionary, and reports success or the exact byte stream that failed.
"""
import os
import select
import sys
import termios
import time
import tty
import zlib

sys.path.insert(0, os.path.dirname(__file__))
from e906_mcu_bridge import (
    build_message_dict, crc16_ccitt, decode_args, decode_vlq,
    encode_message, frame_message, format_lookup,
    MESSAGE_DEST, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN,
    MESSAGE_POS_LEN, MESSAGE_POS_SEQ, MESSAGE_SYNC, MESSAGE_TRAILER_CRC,
    MESSAGE_TRAILER_SIZE, MESSAGE_TRAILER_SYNC,
)


def set_raw(fd):
    tty.setraw(fd, termios.TCSANOW)


def encode_msgid(v, out):
    if v >= 0xC000000 or v < -0x4000000:
        out.append(((v >> 28) & 0x7F) | 0x80)
    if v >= 0x180000 or v < -0x80000:
        out.append(((v >> 21) & 0x7F) | 0x80)
    if v >= 0x3000 or v < -0x1000:
        out.append(((v >> 14) & 0x7F) | 0x80)
    if v >= 0x60 or v < -0x20:
        out.append(((v >> 7) & 0x7F) | 0x80)
    out.append(v & 0x7F)


def make_command(fmt, args, msgid):
    name, params = format_lookup(fmt, {})
    mid = []
    encode_msgid(msgid, mid)
    payload = encode_message(None, args, params, mid)
    return payload


def parse_block(data):
    """Return (block_bytes, seq, msglen) and remove it from data, or None."""
    if len(data) < MESSAGE_MIN:
        return None
    msglen = data[MESSAGE_POS_LEN]
    if msglen < MESSAGE_MIN or msglen > MESSAGE_MAX:
        return None
    if len(data) < msglen:
        return None
    if data[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC:
        return None
    got_crc = (data[msglen - MESSAGE_TRAILER_CRC] << 8) | data[msglen - MESSAGE_TRAILER_CRC + 1]
    if got_crc != crc16_ccitt(data[:msglen - MESSAGE_TRAILER_SIZE]):
        return None
    seq = data[MESSAGE_POS_SEQ]
    block = bytes(data[:msglen])
    del data[:msglen]
    return block, seq, msglen


def expect_frame(ser, rx_buf, timeout=5.0):
    deadline = time.monotonic() + timeout
    while True:
        res = parse_block(rx_buf)
        if res is not None:
            return res
        if time.monotonic() > deadline:
            raise RuntimeError("timeout waiting for frame")
        ready, _, _ = select.select([ser.fileno()], [], [], 0.1)
        if ready:
            chunk = os.read(ser.fileno(), 256)
            if not chunk:
                raise RuntimeError("PTY closed")
            rx_buf.extend(chunk)


def decode_response(fmt, block):
    name, params = format_lookup(fmt, {})
    pos = MESSAGE_HEADER_SIZE
    msgid, pos = decode_vlq(block, pos)
    payload_end = len(block) - MESSAGE_TRAILER_SIZE
    args, pos = decode_args(block, pos, params, payload_end)
    return args


def main():
    path = "/tmp/klipper_host_e906"
    if not os.path.exists(path):
        sys.stderr.write("%s does not exist; bridge not running?\n" % path)
        sys.exit(1)
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    set_raw(fd)
    ser = os.fdopen(fd, "rb+", 0)

    commands, responses, enums, config = build_message_dict()
    identify_id = commands["identify offset=%u count=%c"]

    identify_data = b""
    offset = 0
    send_seq = 1
    rx_buf = bytearray()

    chunk_size = 40
    while True:
        seq = MESSAGE_DEST | (send_seq & 0x0F)
        cmd = make_command("identify offset=%u count=%c", [offset, chunk_size], identify_id)
        frame = frame_message(cmd, seq)
        ser.write(frame)
        print("sent identify offset=%d seq=%02x frame=%s" % (offset, seq, frame.hex()))
        send_seq += 1

        block, rseq, _ = expect_frame(ser, rx_buf)
        print("recv block seq=%02x len=%d" % (rseq, len(block)))

        args = decode_response("identify_response offset=%u data=%*s", block)
        chunk_offset = args["offset"]
        chunk = args["data"]
        print("  identify_response offset=%d len=%d" % (chunk_offset, len(chunk)))
        if chunk_offset != offset:
            raise RuntimeError("unexpected offset %d != %d" % (chunk_offset, offset))
        identify_data += chunk
        if not chunk or len(chunk) < chunk_size:
            break
        offset += len(chunk)

    raw = bytes(identify_data)
    print("\nTotal identify bytes: %d" % len(raw))
    try:
        text = zlib.decompress(raw)
        data = text.decode()
        print("Dictionary JSON length: %d" % len(data))
        print("First 200 chars: %s" % data[:200])
    except Exception as e:
        print("Failed to decompress identify data: %s" % e)
        print("Raw hex (first 200 bytes): %s" % raw[:200].hex())
        sys.exit(1)

    print("\nIdentify handshake OK")


if __name__ == "__main__":
    main()
