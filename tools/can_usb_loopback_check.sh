#!/usr/bin/env bash
# USB-CAN self-test. Uses the adapter's advertised controller loopback mode.
# The final capture distinguishes RX from the local TX echo. Restores normal
# 500 kbit/s mode on exit. Run on the Linux host that owns can0.
set -u

iface=${1:-can0}
for utility in ip timeout candump cansend; do
    if ! command -v "$utility" >/dev/null 2>&1; then
        printf 'Required command missing: %s\n' "$utility" >&2
        exit 2
    fi
done
admin=()
if (( EUID != 0 )); then
    if ! command -v sudo >/dev/null 2>&1; then
        printf 'Run as root or install sudo.\n' >&2
        exit 2
    fi
    sudo -v || exit 2
    admin=(sudo)
fi

ip -details -statistics link show dev "$iface" || exit 2
capture_pid=''
restore_normal() {
    local result=$?
    trap - EXIT INT TERM
    if [[ -n "$capture_pid" ]]; then
        kill "$capture_pid" 2>/dev/null || true
        wait "$capture_pid" 2>/dev/null || true
    fi
    printf '\nRestoring normal CAN mode at 500000 bit/s...\n'
    if "${admin[@]}" ip link set dev "$iface" down &&
       "${admin[@]}" ip link set dev "$iface" type can bitrate 500000 loopback off listen-only off &&
       "${admin[@]}" ip link set dev "$iface" up; then
        printf 'Normal mode restored.\n'
    else
        printf 'Restore failed; check the ip link error above.\n' >&2
        result=1
    fi
    exit "$result"
}
trap restore_normal EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"${admin[@]}" ip link set dev "$iface" down || exit 1
if ! "${admin[@]}" ip link set dev "$iface" type can bitrate 500000 loopback on listen-only off; then
    printf 'Loopback setup failed. Unsupported mode is not a failed hardware test.\n' >&2
    exit 1
fi
"${admin[@]}" ip link set dev "$iface" up || exit 1
printf '\nAdapter loopback enabled. Expect ID 321, data 1122334455667788.\n'
ip -details -statistics link show dev "$iface" || exit 1
timeout 6s candump -t d -e -x "$iface,0:0,#FFFFFFFF" &
capture_pid=$!
sleep 1
if ! cansend "$iface" 321#1122334455667788; then
    printf 'cansend failed.\n' >&2
    exit 1
fi
wait "$capture_pid"
capture_result=$?
capture_pid=''
if (( capture_result != 0 && capture_result != 124 )); then
    printf 'Capture failed with exit code %s.\n' "$capture_result" >&2
    exit 1
fi
printf '\nLoopback results (compare RX and TX counters with the snapshot above):\n'
ip -details -statistics link show dev "$iface" || exit 1
printf '\nA local TX echo alone does not establish adapter receive success.\n'
