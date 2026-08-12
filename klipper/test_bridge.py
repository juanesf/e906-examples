#!/usr/bin/env python3
"""Unit tests for the E906 MCU bridge protocol translator.

Run without root; this mocks /dev/mem access and directly exercises the
message decoder / dispatcher to verify the identify handshake, get_config,
queue_step batching, and PWM translation.
"""

import importlib.util
import os
import sys
import time
import zlib
import json

# ---------------------------------------------------------------------------
# Mocks that must be injected into the bridge module namespace.
# ---------------------------------------------------------------------------
class MockMailbox:
    QSIZE = 16

    def __init__(self):
        self.queue = []
        self.duty = [0, 0]
        self.es_state = 0

    def check_magic(self):
        return True

    def sync_slot(self):
        return True

    def send_cmd(self, code, a0=0, a1=0, a2=0, timeout=1.0):
        if code == 4:  # CMD_QADD
            if len(self.queue) >= self.QSIZE - 1:
                return 1
            self.queue.append((a0, a1, a2))
            return 0
        if code == 2:  # CMD_SET
            if a0 in (0, 1):
                self.duty[a0] = a1
            return 0
        return 0

    def pos_x(self):
        return 0

    def pos_y(self):
        return 0

    def endstop_state(self):
        return self.es_state & 0x3

    def es_override(self, bits):
        self.es_state = bits & 0x3


bridge_path = os.path.join(os.path.dirname(__file__), "e906_mcu_bridge.py")
with open(bridge_path, "r") as f:
    bridge_source = f.read()

mod_ns = {}
for modname in ("errno", "json", "mmap", "os", "signal", "struct", "sys",
                "threading", "time", "zlib"):
    mod_ns[modname] = __import__(modname)
mod_ns["map_region"] = lambda base, size=4096: (None, lambda o: 0, lambda o, v: None)

exec(compile(bridge_source, bridge_path, "exec"), mod_ns)

mod_ns["E906Mailbox"] = MockMailbox
mod_ns["map_region"] = lambda base, size=4096: (None, lambda o: 0, lambda o, v: None)

mod = type(sys)("e906_mcu_bridge")
mod.__dict__.update(mod_ns)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def make_block(payload, seq):
    return mod.frame_message(payload, seq)


def write_command(bridge, msgid, args, params, seq):
    midb = []
    mod.encode_vlq(msgid, midb)
    payload = mod.encode_message(msgid, args, params, midb)
    return make_block(payload, seq)


def decode_response(payload, fmt, bridge):
    msgid, pos = mod.decode_vlq(payload, 0)
    params = bridge._param_types[fmt]
    args, pos = mod.decode_args(payload, pos, params, len(payload))
    return args


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_roundtrip():
    fmt = "queue_step oid=%c interval=%u count=%hu add=%hi"
    params = mod.format_lookup(fmt, mod.build_message_dict()[2])[1]
    mid = 99
    midb = []
    mod.encode_vlq(mid, midb)
    payload = mod.encode_message(mid, [3, 2400, 50, -2], params, midb)
    msgid, pos = mod.decode_vlq(payload, 0)
    assert msgid == mid
    args, pos = mod.decode_args(payload, pos, params, len(payload))
    assert args == {'oid': 3, 'interval': 2400, 'count': 50, 'add': -2}, args
    print("[OK] encode/decode roundtrip")


def test_identify_and_get_config():
    bridge = mod.E906Bridge.__new__(mod.E906Bridge)
    mod.E906Bridge.__init__(bridge, fifo_path="/dev/null")
    bridge.e906 = MockMailbox()
    bridge.write_frame = lambda payload, seq=None: None

    captured = []
    def capture(payload, seq=None):
        captured.append((bytes(payload), seq if seq is not None else bridge.next_sequence))
    bridge.write_frame = capture

    # Identify request
    midb = []
    mod.encode_vlq(1, midb)
    payload = mod.encode_message(1, [0, 40], bridge._param_types["identify offset=%u count=%c"], midb)
    block = make_block(payload, 0x10)
    bridge.process_block(block, 0x10)

    # Should have sent identify_response(s) and an ack.  The first response is what we care about.
    id_resp = None
    for p, s in captured:
        if p:
            msgid, pos = mod.decode_vlq(p, 0)
            if msgid == 0:
                id_resp = decode_response(p, "identify_response offset=%u data=%*s", bridge)
                break
    assert id_resp is not None, captured
    assert id_resp['offset'] == 0
    assert len(id_resp['data']) <= 40
    received = id_resp['data']
    if len(received) < 40:
        assert received == bridge.identify_data
    dictionary = json.loads(zlib.decompress(bridge.identify_data))
    assert dictionary['config']['CLOCK_FREQ'] == mod.CLOCK_FREQ
    print("[OK] identify response offset=%d len=%d" % (id_resp['offset'], len(received)))

    # get_config query
    captured.clear()
    gcid = bridge.commands["get_config"]
    payload = mod.encode_message(gcid, [], bridge._param_types["get_config"], bridge._msgid_bytes["get_config"])
    block = make_block(payload, 0x11)
    bridge.process_block(block, 0x11)

    resp = None
    for p, s in captured:
        if p:
            msgid, pos = mod.decode_vlq(p, 0)
            if msgid == bridge.responses["config is_config=%c crc=%u is_shutdown=%c move_count=%hu"]:
                resp = decode_response(p, "config is_config=%c crc=%u is_shutdown=%c move_count=%hu", bridge)
                break
    assert resp is not None, captured
    assert resp['is_shutdown'] == 0
    assert resp['move_count'] == 500
    print("[OK] get_config response", resp)


def test_queue_step():
    bridge = mod.E906Bridge.__new__(mod.E906Bridge)
    mod.E906Bridge.__init__(bridge, fifo_path="/dev/null")
    bridge.e906 = MockMailbox()
    captured = []
    bridge.write_frame = lambda payload, seq=None: captured.append(payload)

    cmds = bridge.commands
    def issue(fmt, args_list, seq):
        msgid = cmds[fmt]
        params = bridge._param_types[fmt]
        payload = mod.encode_message(msgid, args_list, params, bridge._msgid_bytes[fmt])
        bridge.process_block(make_block(payload, seq), seq)

    issue("allocate_oids count=%c", [8], 0x10)
    issue("config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u",
          [0, "PL2", "PL3", 0, 100], 0x11)
    issue("reset_step_clock oid=%c clock=%u", [0, 0], 0x12)
    issue("set_next_step_dir oid=%c dir=%c", [0, 1], 0x13)
    issue("queue_step oid=%c interval=%u count=%hu add=%hi",
          [0, 2500, 200, 0], 0x14)
    issue("queue_step oid=%c interval=%u count=%hu add=%hi",
          [0, 2500, 200, 0], 0x15)
    time.sleep(0.05)
    bridge.flush_axis(0)

    assert len(bridge.e906.queue) >= 1, bridge.e906.queue
    axis, steps, rate = bridge.e906.queue[0]
    assert axis == 0
    assert steps == 400, steps
    assert mod.MIN_STEP_RATE <= rate <= mod.MAX_STEP_RATE
    print("[OK] queue_step batched to QADD", bridge.e906.queue[0])


def test_pwm_mapping():
    bridge = mod.E906Bridge.__new__(mod.E906Bridge)
    mod.E906Bridge.__init__(bridge, fifo_path="/dev/null")
    bridge.e906 = MockMailbox()
    bridge.write_frame = lambda payload, seq=None: None
    cmds = bridge.commands

    def issue(fmt, args_list, seq):
        msgid = cmds[fmt]
        params = bridge._param_types[fmt]
        payload = mod.encode_message(msgid, args_list, params, bridge._msgid_bytes[fmt])
        bridge.process_block(make_block(payload, seq), seq)

    issue("config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u",
          [2, "PL6", 0, 0, 0], 0x10)
    issue("set_digital_out_pwm_cycle oid=%c cycle_ticks=%u", [2, 5000000], 0x11)
    issue("queue_digital_out oid=%c clock=%u on_ticks=%u",
          [2, 100000, 2500000], 0x12)

    assert bridge.e906.duty[0] == 127, bridge.e906.duty
    print("[OK] PWM half-duty mapping", bridge.e906.duty)


if __name__ == "__main__":
    test_roundtrip()
    test_identify_and_get_config()
    test_queue_step()
    test_pwm_mapping()
    print("\nAll unit tests passed.")
