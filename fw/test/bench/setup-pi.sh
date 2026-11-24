#!/bin/bash
# One-time setup of the can-workbench Pi (RPi 4, Debian 13) for GX5s bench tests:
# CAN tester hat (MCP25625 on SPI0, INA219 on I2C, VCC/ACC relays) + ESP32 Workbench.
# Run on the Pi:  sudo bash setup-pi.sh   then reboot.
set -euo pipefail
U=${SUDO_USER:?run with sudo}
H=$(getent passwd "$U" | cut -d: -f6)

apt-get update
apt-get install -y git nodejs npm build-essential can-utils i2c-tools mosquitto-clients \
    python3-serial python3-can python3-smbus2 python3-pytest

# Hat wiring (schematic "rPI CAN Device Tester" rev 1.0):
#   MCP25625: SPI0 CE0, 16 MHz crystal, INT GPIO25, #RESET GPIO24 (hold high)
#   Relays:   GPIO4 = device VCC, GPIO17 = device ACC (ignition). Off at boot.
#   Serial:   GPIO14/15 (/dev/serial0) = device external UART (SAT-RX / SAT-TX)
CFG=/boot/firmware/config.txt
grep -q gx5s-bench "$CFG" || cat >>"$CFG" <<'EOF'

# gx5s-bench: CAN tester hat
[all]
dtparam=spi=on
dtparam=i2c_arm=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
enable_uart=1
gpio=24=op,dh
gpio=4,17=op,dl
EOF

# The hat's serial header (GPIO14/15) goes to the board's external UART: no login console on it.
sed -i 's/console=serial0,[0-9]* //' /boot/firmware/cmdline.txt

echo i2c-dev >/etc/modules-load.d/i2c-dev.conf # /dev/i2c-1 for the INA219
modprobe i2c-dev

cat >/etc/systemd/system/can0.service <<'EOF'
[Unit]
Description=can0 at 500 kbit/s (GX5s firmware default)
BindsTo=sys-subsystem-net-devices-can0.device
After=sys-subsystem-net-devices-can0.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/ip link set can0 up type can bitrate 500000 restart-ms 100

[Install]
WantedBy=sys-subsystem-net-devices-can0.device
EOF

# ESP32 Workbench, as the login user (serial console + RFC2217 bridge on :4001+).
sudo -u "$U" -H bash -euo pipefail <<'EOF'
cd ~
[ -x .deno/bin/deno ] || curl -fsSL https://deno.land/install.sh | sh -s -- -y --no-modify-path
[ -d esp32-workbench ] || git clone https://github.com/react-iot/esp32-workbench
cd esp32-workbench && ~/.deno/bin/deno install
EOF

cat >/etc/systemd/system/esp32-workbench.service <<EOF
[Unit]
Description=ESP32 Workbench
After=network-online.target

[Service]
User=$U
SupplementaryGroups=dialout plugdev
WorkingDirectory=$H/esp32-workbench
Environment=HOME=$H PATH=$H/.deno/bin:/usr/local/bin:/usr/bin:/bin
EnvironmentFile=-$H/esp32-workbench/.env
ExecStart=$H/.deno/bin/deno run --allow-net --allow-read --allow-write --allow-env --allow-run --allow-sys --node-modules-dir ./bin/server.ts
Restart=on-failure
RestartSec=3
KillMode=mixed

[Install]
WantedBy=multi-user.target
EOF

# Soak logger (soak.py, copied to ~/gx5s-bench with test_bench.py). Not enabled:
# start it for a long run, stop it before running pytest (both drive the hat).
cat >/etc/systemd/system/gx5s-soak.service <<EOF
[Unit]
Description=GX5s soak logger (supply current, wake and sleep events)
After=esp32-workbench.service

[Service]
User=$U
WorkingDirectory=$H/gx5s-bench
ExecStart=/usr/bin/python3 -u soak.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# MQTT capture: every message the broker lets this login see, one line each (time,
# topic, payload), appended to ~/mqtt-capture/messages.log for later analysis. The
# login is ~/.config/reactiot/mqtt-monitor.env (MQTT_HOST, MQTT_PORT, MQTT_USER,
# MQTT_PASS; keep it mode 600); the service does nothing until that file exists. The
# capture can hold provisioning messages, which carry broker credentials: it is
# written owner-only.
cat >/etc/systemd/system/gx5s-mqtt-capture.service <<EOF
[Unit]
Description=GX5s MQTT capture (all device messages, for later analysis)
After=network-online.target
Wants=network-online.target
ConditionPathExists=$H/.config/reactiot/mqtt-monitor.env

[Service]
User=$U
UMask=0077
EnvironmentFile=$H/.config/reactiot/mqtt-monitor.env
ExecStartPre=/bin/mkdir -p $H/mqtt-capture
ExecStart=/bin/sh -c 'exec mosquitto_sub -h "\$\$MQTT_HOST" -p "\$\$MQTT_PORT" -u "\$\$MQTT_USER" -P "\$\$MQTT_PASS" -i bench-monitor -t "device/#" -t "discovery/#" -t "location/#" -F "%%I %%t %%p" >>$H/mqtt-capture/messages.log'
Restart=always
RestartSec=30

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable can0.service esp32-workbench.service gx5s-mqtt-capture.service
echo "Done. Reboot, then check: ip -br link show can0; i2cdetect -y 1; curl localhost:4000/api/ports"
