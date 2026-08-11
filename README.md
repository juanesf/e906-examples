# E906 co-processor examples (Allwinner A523/T527 / Avaota-A1)

Three bare-metal examples for the RISC-V E906 co-processor, cross-compiled
for `rv32imac`/`ilp32` with `riscv64-unknown-elf-gcc`.

## Build

```
compile-hello           # hello/           -> e906-hello.elf
compile-lcd             # lcd/             -> e906-lcd.elf
compile-lcd-with-msg    # lcd-with-msg/    -> e906-lcd-msg.elf
```

The scripts are on `~/bin` (added to `PATH` in `~/.bashrc`); you can also
run them from this folder as `./compile-hello`, etc.

## The important crt0.S fix

All examples share `crt0.S`, which enables the I-cache and D-cache
(`csrsi mhcr,1` / `csrsi mhcr,2`, i.e. `mhcr = 0x3`) before `main()`.

Without the D-cache enabled (the reset default), the A523 E906 returns **0
for every data load** (SRAM0, DDR, even read-back of an own store) while
stores and instruction fetches still work.  The vendor firmware enables the
caches, which is why it works; keeping them disabled made our first
firmware appear to "receive nothing".

## Deploy (ARM side)

Copy the ELF to the board firmware path and bounce the remoteproc:

```sh
scp e906-hello.elf avaota@<ip>:/lib/firmware/rproc-7130000.e906_rproc-fw
ssh avaota@<ip> 'echo stop  | sudo tee /sys/class/remoteproc/remoteproc0/state'
ssh avaota@<ip> 'echo start | sudo tee /sys/class/remoteproc/remoteproc0/state'
```

The E906 boots from the ELF entry point (`0x3ffc0000`).  The DT reserves:
`riscvsram0` `0x3ffc0000 -> 0x07280000` (0x40000) and the DDR mailbox
carveout at `0x60000000`.

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
TX / SoC PM0).

### lcd — gradient on the ST7789V

Drives the on-board 240x135 ST7789V LCD via S_SPI0 with no ARM help:
animated RGB gradient, moving white bar, frame counter.  Also keeps the
mailbox magic/counter for the ARM to read.

### lcd-with-msg — gradient + ARM message panel

Same as `lcd`, plus a text panel (top-left) that the **ARM pushes** over
the DDR mailbox every 2 s:

- `+0x40` `seq` (incremented per update)
- `+0x44` 4 x 32 ASCII text lines

Run `mbox_info.py` on the ARM to feed it:

```sh
sudo python3 mbox_info.py    # writes LOAD / IP / UP / version lines
```

The E906 polls `seq` every ~16 frames and redraws the panel.  Mailbox
header (E906 -> ARM): `+0x00` magic `0xE9061B0B`, `+0x04` version,
`+0x08` flags, `+0x0C` frame counter, `+0x10` result, `+0x14` error step.

## Layout notes

- Link base: DA `0x3ffc0000` (SRAM0), stack at the top of SRAM0
  (`0x40000000`).
- The E906's `crt0.S` also zeroes `.bss` and sets `sp` before `main`.
- `mbox_info.py` uses `/dev/mem`; run it with sudo on the board.
