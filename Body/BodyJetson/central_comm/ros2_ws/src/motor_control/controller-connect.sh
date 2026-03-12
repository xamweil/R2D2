#!/usr/bin/env bash
# pair-ps4.sh — Pair a PS4 controller via Bluetooth
# Hold SHARE + PS until light double-flashes before running

set -euo pipefail

echo "Scanning... Put controller in pairing mode now."

bluetoothctl <<EOF
power on
agent on
default-agent
scan on
EOF

sleep 15

MAC=$(bluetoothctl devices | grep -i "wireless controller" | awk '{print $2}' | head -1)

if [[ -z "$MAC" ]]; then
    echo "ERROR: No PS4 controller found." >&2
    exit 1
fi

echo "Found controller: $MAC"

bluetoothctl <<EOF
pair $MAC
trust $MAC
connect $MAC
EOF

echo "Done! Controller $MAC is connected."
