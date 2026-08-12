#!/bin/bash
# Install the E906 "klipper" stepper/PWM firmware as the boot firmware and
# wire up the Klipper host integration: the MCU bridge daemon and the
# minimal printer config/service.
#
# Run on the board (needs root) from a checkout of this repo:
#
#     git clone https://github.com/juanesf/e906-examples.git
#     cd e906-examples && sudo ./deploy-klipper.sh
#
# Installs:
#   * klipper/e906-klipper.elf as the boot firmware (reloaded immediately)
#   * klipper/e906_mcu_bridge.py  -> /usr/local/bin/e906_mcu_bridge.py
#   * klipper/printer.cfg         -> /usr/local/etc/e906-klipper/printer.cfg
#   * systemd units e906-mcu-bridge.service and klipper-printer.service
#
# If a standard Klipper host is already present (klipper.service, e.g. the
# kiauh layout), the printer unit is installed but NOT enabled so that the
# existing host keeps owning the PTY; only the bridge daemon is started.
#
# The primary command channel is the DDR mailbox (no serial cable needed);
# S-UART0 @ 115200 (header pin 37 / PM0) is the debug serial.
#
# Verify afterwards:
#   sudo systemctl status e906-mcu-bridge
#   sudo python3 klipper/test_host.py          # identify handshake over the PTY
#
# To switch back to another firmware, copy its ELF to the same path and
# bounce the remoteproc (or just reboot).
set -e

cd "$(dirname "$(readlink -f "$0")")"

FW=/lib/firmware/rproc-7130000.e906_rproc-fw
ELF=klipper/e906-klipper.elf

[ "$(id -u)" -eq 0 ] || { echo "Run with sudo." >&2; exit 1; }
[ -f "$ELF" ] || { echo "Missing $ELF (run ./compile-klipper on the host)." >&2; exit 1; }

install -m 0644 "$ELF" "$FW"
install -m 0755 klipper/e906_mcu_bridge.py /usr/local/bin/e906_mcu_bridge.py
install -d /usr/local/etc/e906-klipper
install -m 0644 klipper/printer.cfg /usr/local/etc/e906-klipper/printer.cfg
install -m 0644 klipper/e906-mcu-bridge.service /etc/systemd/system/
install -m 0644 klipper/klipper-printer.service /etc/systemd/system/

echo stop  | tee /sys/class/remoteproc/remoteproc0/state >/dev/null || true
echo start | tee /sys/class/remoteproc/remoteproc0/state >/dev/null

systemctl daemon-reload
systemctl enable --now e906-mcu-bridge
if [ -e /etc/systemd/system/klipper.service ]; then
    echo "Existing klipper.service detected: keeping it as the Klipper host."
else
    systemctl enable --now klipper-printer
fi

echo "OK: Klipper firmware + bridge deployed."
echo "    sudo systemctl status e906-mcu-bridge"
echo "    sudo python3 klipper/test_host.py"
