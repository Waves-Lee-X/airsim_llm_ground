#!/bin/sh
set -eu

usage() {
    echo "Usage: sudo $0 [--activate] [--online]" >&2
}

activate=false
online=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        --activate) activate=true ;;
        --online) online=true ;;
        *) usage; exit 2 ;;
    esac
    shift
done

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this installer with sudo" >&2
    exit 1
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source_root=$(CDPATH= cd -- "$script_dir/../.." && pwd)
install_root=/opt/aeromind-apm-lite
config_root=/etc/aeromind-apm-lite
runtime_user=aeromind

if ! id "$runtime_user" >/dev/null 2>&1; then
    useradd --system --home-dir /nonexistent --shell /usr/sbin/nologin "$runtime_user"
fi
usermod -a -G dialout "$runtime_user"

install -d -m 0755 "$install_root" "$config_root"
if [ "$source_root" != "$install_root" ]; then
    cp -a "$source_root/src" "$source_root/configs" "$source_root/deploy" \
        "$source_root/pyproject.toml" "$source_root/setup.py" \
        "$source_root/setup.cfg" "$install_root/"
fi

if [ ! -x "$install_root/venv/bin/python" ]; then
    echo "Missing $install_root/venv/bin/python; install the offline Python environment first" >&2
    exit 1
fi

if [ "$online" = true ]; then
    "$install_root/venv/bin/python" -m pip install --no-cache-dir \
        -r "$install_root/deploy/raspberry-pi/requirements-onboard.txt"
fi
PYTHONPATH="$install_root/src" "$install_root/venv/bin/python" -c \
    'import pydantic, serial, yaml, pymavlink, aeromind_apm_lite'
install -m 0644 "$source_root/deploy/systemd/aeromind-apm-lite@.service" \
    /etc/systemd/system/aeromind-apm-lite@.service
install -m 0755 "$source_root/deploy/raspberry-pi/preflight-onboard.sh" \
    "$install_root/deploy/raspberry-pi/preflight-onboard.sh"

if [ ! -f "$config_root/fleet.yaml" ]; then
    install -m 0644 "$source_root/configs/real/fleet.uav3.yaml" \
        "$config_root/fleet.yaml"
fi
if [ ! -s "$config_root/uav3.env" ]; then
    echo "Missing $config_root/uav3.env; refusing to start without the local credential" >&2
    exit 1
fi
chown root:"$runtime_user" "$config_root/uav3.env"
chmod 0640 "$config_root/uav3.env"

systemctl daemon-reload

if [ "$activate" = true ]; then
    if grep -q '^[^#].*swarmXian_group_fix\.py' /etc/rc.local; then
        backup="/etc/rc.local.aeromind-backup-$(date +%Y%m%d%H%M%S)"
        cp -p /etc/rc.local "$backup"
        sed -i '/^[^#].*swarmXian_group_fix\.py/s/^/# disabled by AeroMind APM Lite: /' /etc/rc.local
        echo "Legacy rc.local entry disabled; backup: $backup"
    fi
    systemctl stop aeromind-apm-lite-bench.service 2>/dev/null || true
    pkill -TERM -f '[s]warmXian_group_fix\.py' 2>/dev/null || true
    wait_count=0
    while pgrep -f '[s]warmXian_group_fix\.py' >/dev/null 2>&1; do
        wait_count=$((wait_count + 1))
        if [ "$wait_count" -ge 5 ]; then
            echo "Legacy flight process did not stop; Lite service was not started" >&2
            exit 1
        fi
        sleep 1
    done
    if [ -f /home/lenovo3/stream/ed_rtsp.py ]; then
        cp -p /home/lenovo3/stream/ed_rtsp.py \
            "/home/lenovo3/stream/ed_rtsp.py.aeromind-backup-$(date +%Y%m%d%H%M%S)"
    fi
    install -m 0755 -o lenovo3 -g lenovo3 \
        "$source_root/deploy/raspberry-pi/ed_rtsp.py" \
        /home/lenovo3/stream/ed_rtsp.py
    systemctl restart ed_rtsp.service
    systemctl is-active --quiet ed_rtsp.service
    systemctl enable --now aeromind-apm-lite@3.service
    systemctl --no-pager --full status aeromind-apm-lite@3.service
else
    echo "Installed without switching UART ownership."
    echo "Run again with --activate when the propellers are removed and the ground P9 is ready."
fi
