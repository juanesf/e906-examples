#!/bin/sh
# Wait for the E906 remoteproc to appear, then start it if not running.
state=/sys/class/remoteproc/remoteproc0/state
for i in $(seq 1 60); do
    if [ -r "$state" ]; then
        cur=$(cat "$state" 2>/dev/null)
        [ "$cur" = running ] && exit 0
        echo start > "$state" 2>/dev/null && exit 0
    fi
    sleep 1
done
exit 1
