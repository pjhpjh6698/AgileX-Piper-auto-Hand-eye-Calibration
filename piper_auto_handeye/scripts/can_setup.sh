#!/usr/bin/env bash
# Bring up the Piper CAN interface(s) for the hand-eye calibration stack.
#
#   ./can_setup.sh                  # bring up every gs_usb CAN iface at 1 Mbit/s
#   ./can_setup.sh can0             # bring up just this one
#   ./can_setup.sh can0 1000000
#
# Idempotent: an interface that is already UP at the right bitrate is left
# alone (bouncing it would drop the SDK's rx thread on a running node).
#
# This only prepares the socketcan link. To check that the ARM actually
# answers on it, run the SDK-level probe:
#   ros2 run piper_auto_handeye agx_arm_check --can-port can0
set -uo pipefail

BITRATE="${2:-1000000}"
WANT_IFACE="${1:-}"

red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }

# --- prerequisites --------------------------------------------------------
missing=()
command -v ip      >/dev/null || missing+=("iproute2")
command -v ethtool >/dev/null || missing+=("ethtool")
command -v candump >/dev/null || missing+=("can-utils")
if ((${#missing[@]})); then
    red "Missing tools: ${missing[*]}"
    echo "  sudo apt update && sudo apt install -y ${missing[*]}"
    exit 1
fi

# The USB-CAN dongles are gs_usb (OpenMoko 1d50:606f). The kernel normally
# autoloads it; do it explicitly so a fresh boot doesn't surprise us.
if ! lsmod | grep -q '^gs_usb'; then
    sudo modprobe gs_usb 2>/dev/null || ylw "note: could not modprobe gs_usb (may be built in)"
fi

# --- discover -------------------------------------------------------------
mapfile -t IFACES < <(ip -br link show type can 2>/dev/null | awk '{print $1}')
if ((${#IFACES[@]} == 0)); then
    red "No CAN interface found."
    echo "  - Is the USB-CAN dongle plugged in?  Check with: lsusb | grep -i 'CAN adapter'"
    echo "  - Expected device: 1d50:606f (Geschwister Schneider / gs_usb)"
    exit 1
fi

if [[ -n "$WANT_IFACE" ]]; then
    if ! printf '%s\n' "${IFACES[@]}" | grep -qx "$WANT_IFACE"; then
        red "Interface '$WANT_IFACE' not found. Present: ${IFACES[*]}"
        exit 1
    fi
    IFACES=("$WANT_IFACE")
fi

# --- bring up -------------------------------------------------------------
rc=0
for iface in "${IFACES[@]}"; do
    state=$(ip -br link show "$iface" | awk '{print $2}')
    cur_bitrate=$(ip -d link show "$iface" | awk '/bitrate/ {for(i=1;i<=NF;i++) if($i=="bitrate") print $(i+1)}' | head -1)
    businfo=$(sudo ethtool -i "$iface" 2>/dev/null | awk '/bus-info/ {print $2}')

    if [[ "$state" == "UP" && "$cur_bitrate" == "$BITRATE" ]]; then
        grn "$iface: already UP at ${BITRATE} bps  (usb ${businfo:-?})"
        continue
    fi

    ylw "$iface: configuring -> ${BITRATE} bps  (usb ${businfo:-?})"
    sudo ip link set "$iface" down 2>/dev/null
    if ! sudo ip link set "$iface" type can bitrate "$BITRATE"; then
        red "$iface: failed to set bitrate"; rc=1; continue
    fi
    if ! sudo ip link set "$iface" up; then
        red "$iface: failed to bring up"; rc=1; continue
    fi
    grn "$iface: UP at ${BITRATE} bps"
done

# --- confirm the bus is actually carrying arm frames ----------------------
echo
for iface in "${IFACES[@]}"; do
    frames=$(timeout 1 candump -n 20 "$iface" 2>/dev/null | wc -l)
    if ((frames > 0)); then
        grn "$iface: ${frames} frames in 1 s -- an arm is powered and talking"
    else
        ylw "$iface: NO traffic. Link is up but nothing is transmitting."
        echo "         Check that the arm is powered on and the CAN cable is seated."
        rc=1
    fi
done

exit $rc
