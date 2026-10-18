# ESP32 MAC monitor

`esp32_mac_monitor.py` waits for an ESP32 connected through the board's CP2102
USB-UART bridge, reads its factory MAC using `esptool`, and prints a
label-friendly line:

```text
ESP32 MAC: C4:DD:57:E0:9F:AC    PORT: /dev/cu.SLAB_USBtoUART
```

## Setup

Create the local environment and install Espressif's tool once:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install esptool
```

The monitor automatically uses `.venv/bin/esptool` when it exists.

## Use

From this directory, start the monitor before plugging in a board:

```sh
python3 esp32_mac_monitor.py
```

Read one board and exit:

```sh
python3 esp32_mac_monitor.py --once
```

For a known port, which is useful when testing one board repeatedly:

```sh
python3 esp32_mac_monitor.py --port /dev/cu.SLAB_USBtoUART --once
```

For label software or another script, request one JSON record per board:

```sh
python3 esp32_mac_monitor.py --json
```

The script only runs esptool's `read-mac` operation; it does not flash or erase
the device. The ESP32 may reset briefly while esptool connects.
