#!/bin/sh
set -eu

for device in /dev/ttyAMA0 /dev/ttyAMA1; do
    if [ ! -c "$device" ]; then
        echo "AeroMind preflight: missing serial device $device" >&2
        exit 1
    fi
    if [ ! -r "$device" ] || [ ! -w "$device" ]; then
        echo "AeroMind preflight: service user cannot access $device" >&2
        exit 1
    fi
done

if pgrep -f '[s]warmXian.*\.py' >/dev/null 2>&1; then
    echo "AeroMind preflight: legacy swarmXian process still owns the flight UARTs" >&2
    exit 1
fi

if [ ! -r /etc/aeromind-apm-lite/fleet.yaml ]; then
    echo "AeroMind preflight: fleet configuration is missing" >&2
    exit 1
fi

echo "AeroMind preflight: UARTs and configuration are ready"
