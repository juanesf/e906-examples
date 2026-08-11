#!/usr/bin/env python3
"""ARM-side daemon for the E906 "MPU6050 + LCD" example.

Reads the E906 mailbox (reserved DDR @ 0x60000000, DA == PA) every 100 ms,
prints the MPU6050 accel / gyro / temperature and, when /dev/uinput is
available, injects the values into the Linux kernel input subsystem
(as ABS_X/Y/Z = accel, ABS_RX/RY/RZ = gyro on an "E906 MPU6050" device) so
evtest and other input apps can see the IMU.

Mailbox layout (written by the E906 firmware):
  +0x00 magic 0xE9061B0B, +0x04 version 2, +0x08 flags
  +0x0C counter, +0x10 result (0 = ok), +0x14 error step
  +0x18 mpu_addr, +0x1C WHO_AM_I
  +0x20 AX  +0x24 AY  +0x28 AZ   (int16, sign-extended to int32)
  +0x2C GX  +0x30 GY  +0x34 GZ
  +0x38 TEMP raw +0x3C TEMP_C x100
"""
import fcntl
import mmap
import os
import struct
import time

BASE = 0x60000000
MAGIC = 0xE9061B0B

# uinput ioctls (Linux 64-bit: dir<<30 | size<<16 | type<<8 | nr)
def _iow(t, n, size):
    return (1 << 30) | (size << 16) | (t << 8) | n

def _io(t, n):
    return (t << 8) | n

UINPUT_BASE = ord("U")

# Modern uinput API (this kernel, see include/uapi/linux/uinput.h):
#   struct uinput_setup = { struct input_id id; char name[80]; __u32 ff } = 92 B
#   struct uinput_abs_setup = { __u16 code; __u16 pad; input_absinfo(24) } = 28 B
UINPUT_SETUP_SIZE = 92
UINPUT_ABS_SETUP_SIZE = 28

UI_SET_EVBIT = _iow(UINPUT_BASE, 100, 4)
UI_SET_ABSBIT = _iow(UINPUT_BASE, 103, 4)
UI_DEV_SETUP = _iow(UINPUT_BASE, 3, UINPUT_SETUP_SIZE)
UI_ABS_SETUP = _iow(UINPUT_BASE, 4, UINPUT_ABS_SETUP_SIZE)
UI_DEV_CREATE = _io(UINPUT_BASE, 1)
UI_DEV_DESTROY = _io(UINPUT_BASE, 2)

EV_SYN, EV_ABS = 0, 3
SYN_REPORT = 0
ABS_X, ABS_Y, ABS_Z = 0, 1, 2
ABS_RX, ABS_RY, ABS_RZ = 3, 4, 5

INPUT_EVENT = struct.Struct("<qqHHi")   # timeval(16) + type + code + value
ABS_SETUP = struct.Struct("<HHiiiiii")  # code + pad + input_absinfo (28 B)


class UInput:
    def __init__(self):
        self.fd = None
        if not os.path.exists("/dev/uinput"):
            return
        try:
            self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)

            fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_ABS)
            for code in (ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ):
                fcntl.ioctl(self.fd, UI_SET_ABSBIT, code)

            setup = bytearray(UINPUT_SETUP_SIZE)
            struct.pack_into("HHHH", setup, 0, 0x0013, 0x1, 0xE906, 1)
            struct.pack_into("80s", setup, 8, b"E906 MPU6050")
            fcntl.ioctl(self.fd, UI_DEV_SETUP, setup)

            for code in (ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ):
                abs_setup = ABS_SETUP.pack(code, 0, 0, -32768, 32767, 0, 0, 0)
                fcntl.ioctl(self.fd, UI_ABS_SETUP, abs_setup)

            fcntl.ioctl(self.fd, UI_DEV_CREATE)
        except OSError:
            if self.fd is not None:
                os.close(self.fd)
                self.fd = None

    def event(self, code, value):
        if self.fd is None:
            return
        t = time.time()
        sec, usec = int(t), int((t - int(t)) * 1e6)
        os.write(self.fd, INPUT_EVENT.pack(sec, usec, EV_ABS, code, value))

    def sync(self):
        if self.fd is None:
            return
        t = time.time()
        sec, usec = int(t), int((t - int(t)) * 1e6)
        os.write(self.fd, INPUT_EVENT.pack(sec, usec, EV_SYN, SYN_REPORT, 0))

    def close(self):
        if self.fd is not None:
            try:
                fcntl.ioctl(self.fd, UI_DEV_DESTROY)
            except OSError:
                pass
            os.close(self.fd)
            self.fd = None


def rd32(m, off):
    return struct.unpack_from("<I", m, off)[0]


def s16(v):
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v & 0x80000000 else v


def main():
    try:
        os.system("modprobe uinput 2>/dev/null")
    except Exception:
        pass

    ui = UInput()
    if ui.fd is None:
        print("[mpu_info] /dev/uinput not available - printing only", flush=True)

    with open("/dev/mem", "r+b") as f:
        m = mmap.mmap(f.fileno(), 0x40, offset=BASE)
        last_cnt = None
        try:
            while True:
                magic = rd32(m, 0x00)
                cnt = rd32(m, 0x0C)
                result = rd32(m, 0x10)
                step = rd32(m, 0x14)
                addr = rd32(m, 0x18)
                who = rd32(m, 0x1C)
                ax, ay, az = s16(rd32(m, 0x20)), s16(rd32(m, 0x24)), s16(rd32(m, 0x28))
                gx, gy, gz = s16(rd32(m, 0x2C)), s16(rd32(m, 0x30)), s16(rd32(m, 0x34))
                temp = s16(rd32(m, 0x38))
                tc100 = rd32(m, 0x3C)

                ok = magic == MAGIC
                line = ("magic=%s " % ("OK" if ok else "BAD"))
                if ok:
                    line += "addr=0x%02x who=0x%02x cnt=0x%08x r=%d/%d " % (
                        addr, who, cnt, result, step)
                    line += "AX=%-7d AY=%-7d AZ=%-7d " % (ax, ay, az)
                    line += "GX=%-7d GY=%-7d GZ=%-7d " % (gx, gy, gz)
                    line += "T=%.2f C" % (tc100 / 100.0)
                print("[mpu_info] " + line, flush=True)

                if ok and magic == MAGIC and cnt != last_cnt:
                    last_cnt = cnt
                    if result == 0:
                        ui.event(ABS_X, ax)
                        ui.event(ABS_Y, ay)
                        ui.event(ABS_Z, az)
                        ui.event(ABS_RX, gx)
                        ui.event(ABS_RY, gy)
                        ui.event(ABS_RZ, gz)
                        ui.sync()
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    ui.close()


if __name__ == "__main__":
    main()
