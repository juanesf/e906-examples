#!/bin/bash
# Install the sysmon dashboard as the E906 boot firmware and enable the
# ARM-side mailbox daemon so the dashboard starts at every boot.
#
# Run on the board (needs root) from a checkout of this repo:
#
#     git clone https://github.com/juanesf/e906-examples.git
#     cd e906-examples && sudo ./deploy-sysmon.sh
#
# The prebuilt sysmon/e906-sysmon.elf is included in the repo; rebuild it on
# a host with the RISC-V toolchain via ./compile-sysmon if you prefer.
set -e

cd "$(dirname "$(readlink -f "$0")")"

FW=/lib/firmware/rproc-7130000.e906_rproc-fw
ELF=sysmon/e906-sysmon.elf

[ "$(id -u)" -eq 0 ] || { echo "Ejecutar con sudo." >&2; exit 1; }
[ -f "$ELF" ] || { echo "Falta $ELF (./compile-sysmon en el host)." >&2; exit 1; }

install -m 0644 "$ELF" "$FW"                          # boot firmware
install -m 0755 deploy/e906-rproc-start.sh /usr/local/bin/
install -m 0644 deploy/e906-rproc.service  /etc/systemd/system/
install -m 0644 deploy/e906-sysmon.service /etc/systemd/system/
install -m 0644 sysmon/mbox_sysmon.py       /usr/local/bin/mbox_sysmon.py

systemctl daemon-reload
systemctl enable --now e906-rproc e906-sysmon

echo "OK: dashboard E906 desplegado. Se inicia en el próximo boot."
