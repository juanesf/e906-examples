# E906 co-processor examples (Allwinner A523/T527 / Avaota-A1)

Bare-metal examples for the RISC-V E906 co-processor, cross-compiled for
`rv32imac`/`ilp32` with `riscv64-unknown-elf-gcc` and deployed as remoteproc
firmware for the `sun55i_e906_rproc` driver.

## Build

Each example has a `compile-*` script on `~/bin` (added to `PATH` in
`~/.bashrc`); you can also run them from this folder as `./compile-hello`, etc.

```
compile-hello           # hello/           -> e906-hello.elf
compile-lcd             # lcd/             -> e906-lcd.elf
compile-lcd-with-msg    # lcd-with-msg/    -> e906-lcd-msg.elf
compile-mpu6050-lcd     # mpu6050-lcd/     -> e906-mpu6050-lcd.elf
compile-sysmon          # sysmon/          -> e906-sysmon.elf
```

## The important crt0.S fix

All examples share `crt0.S`, which enables the I-cache and D-cache
(`csrsi mhcr,1` / `csrsi mhcr,2`, i.e. `mhcr = 0x3`) before `main()`.

Without the D-cache enabled (the reset default), the A523 E906 returns **0 for
every data load** (SRAM0, DDR, even read-back of an own store) while stores and
instruction fetches still work.  The vendor firmware enables the caches, which
is why it works; keeping them disabled made our first firmware appear to
"receive nothing".

## Deploy (ARM side)

Copy the ELF to the board firmware path and bounce the remoteproc:

```sh
scp e906-hello.elf avaota@<ip>:/lib/firmware/rproc-7130000.e906_rproc-fw
ssh avaota@<ip> 'echo stop  | sudo tee /sys/class/remoteproc/remoteproc0/state'
ssh avaota@<ip> 'echo start | sudo tee /sys/class/remoteproc/remoteproc0/state'
```

The E906 boots from the ELF entry point (`0x3ffc0000`).  The DT reserves:
`riscvsram0` `0x3ffc0000 -> 0x07280000` (0x40000) and the DDR mailbox carveout
at `0x60000000`.

## Autostart at boot

Whatever ELF is stored at `/lib/firmware/rproc-7130000.e906_rproc-fw` is the
firmware loaded at boot: the `e906-rproc.service` unit (oneshot) waits for
`/sys/class/remoteproc/remoteproc0/state` to appear and then writes `start`,
which makes the kernel load that ELF into the E906.

Only one firmware can be the boot firmware. To switch to another example:

```sh
sudo cp /lib/firmware/rproc-7130000.e906_rproc-fw{,.prev}   # keep current
sudo cp e906-sysmon.elf /lib/firmware/rproc-7130000.e906_rproc-fw
echo stop  | sudo tee /sys/class/remoteproc/remoteproc0/state   # reload now
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
```

(or just reboot). Examples whose UI depends on the ARM host need a matching
systemd service that feeds the mailbox, e.g. `e906-sysmon.service` runs
`mbox_sysmon.py` for the sysmon dashboard; feed `lcd-with-msg` the same way
with `mbox_info.py`. Only enable one feeder at a time.

## Examples

### hello — "hello world" from the E906 co-processor

Writes a mailbox in the reserved DDR (`0x60000000`, DA == PA) and bumps a
counter forever.  Watch it from the ARM:

```sh
sudo devmem 0x60000000     # magic    = 0xE9061B0B
sudo devmem 0x6000000C     # counter  (keeps incrementing)
sudo devmem 0x60000040 16  # "HELLO WORLD FROM E906 CO-PROCESSOR!"
```

It also prints the message over S-UART0 @ 115200 8N1 (header pin 37 =
TX / SoC PM0).  Photos of the serial setup:

![Serial print](hello/serial-print.jpeg)
![USB-serial adapter](hello/serial-usb-adapter.jpeg)
![Serial port / GPIO header](hello/serial-port-gpio.jpeg)

### lcd — gradient on the ST7789V

Drives the on-board 240x135 ST7789V LCD via S_SPI0 with no ARM help: animated
RGB gradient, moving white bar, frame counter.  Also keeps the mailbox
magic/counter for the ARM to read.

![Gradient](lcd/gradient.jpeg)

### lcd-with-msg — gradient + ARM message panel

Same as `lcd`, plus a text panel (top-left) that the **ARM pushes** over the
DDR mailbox every 2 s:

- `+0x40` `seq` (incremented per update)
- `+0x44` 4 x 32 ASCII text lines

Run `mbox_info.py` on the ARM to feed it:

```sh
sudo python3 mbox_info.py    # writes LOAD / IP / UP / version lines
```

![Gradient + message panel](lcd-with-msg/gradient-with-msg.mp4)

The E906 polls `seq` every ~16 frames and redraws the panel.  Mailbox header
(E906 -> ARM): `+0x00` magic `0xE9061B0B`, `+0x04` version, `+0x08` flags,
`+0x0C` frame counter, `+0x10` result, `+0x14` error step.

### mpu6050-lcd — MPU6050 + LCD

Same LCD wiring as `lcd`, reading a GY-521 (MPU6050) IMU over TWI/SMBus and
rendering the measured orientation on the display.

![MPU6050 + LCD](mpu6050-lcd/mpu6050-lcd.jpeg)

### sysmon — system monitor dashboard (mailbox v3)

Live dashboard on the ST7789V fed by the ARM host via a v3 mailbox:
LOAD (1m/5m/15m with bars), MEM usage, CPU temperature (THS), uptime, clock
and a footer with hostname/IP.  The ARM side runs `mbox_sysmon.py` (a systemd
service `e906-sysmon.service`), which pushes metrics every second:

```sh
sudo python3 sysmon/mbox_sysmon.py   # daemon; reads /sys/class/thermal
```

![sysmon dashboard](sysmon/sysmon-dashboard.jpeg)

Mailbox v3 layout (reserved DDR @ 0x60000000, all offsets 4-byte aligned):

| offset | field                 | writer |
|--------|-----------------------|--------|
| +0x00  | magic 0xE9061B0B      | E906   |
| +0x04  | version = 3           | E906   |
| +0x08  | flags 0x0000B00B      | E906   |
| +0x0C  | e906_cnt (frame)      | E906   |
| +0x10  | arm_cnt               | ARM    |
| +0x14  | status (bit0 valid)   | ARM    |
| +0x18..| load1/5/15 x1000      | ARM    |
| +0x24  | mem_total_KB          | ARM    |
| +0x28  | mem_used_KB           | ARM    |
| +0x2C  | temp_mC               | ARM    |
| +0x30  | uptime_s              | ARM    |
| +0x34  | clock_packed          | ARM    |
| +0x40  | seq + text[4][32]     | ARM    |

## Layout notes

- Link base: DA `0x3ffc0000` (SRAM0), stack at the top of SRAM0
  (`0x40000000`).
- The E906's `crt0.S` also zeroes `.bss` and sets `sp` before `main`.
- `/dev/mem` on the board only allows **aligned 32-bit accesses**; wider or
  unaligned reads/writes SIGBUS (mailbox readers must use word-at-a-time
  `struct.unpack_from`).

## Acknowledgements

Investigation and on-board testing for these examples (the E906 cache root
cause, the mailbox protocol, the ST7789V wiring and the sysmon daemon) were
carried out with the assistance of opencode.

## License

MIT — see [LICENSE](LICENSE).
