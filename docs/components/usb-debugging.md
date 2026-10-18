# GX5s Logger Board - USB Debugging

**References:** USB1 USB-C connector, U8 `CP2102-GMR`  
**Function:** USB service, serial logging, reset, and boot-mode entry  

## Overview

The USB-C connector provides a USB service connection to a Silicon Labs CP2102 USB-to-UART bridge. The bridge connects to the ESP32-S UART and also provides the DTR/RTS control signals used for automatic reset and boot-mode entry.

## USB reprogramming and automatic reset

The GX5s Logger Board can be **reprogrammed over USB**. The CP2102 exposes the ESP32 UART through a virtual COM port, while its DTR and RTS outputs drive the reset and `MODE` circuitry. This allows supported programming tools to enter the ESP32 bootloader, transfer firmware, and reset the device automatically without manually operating the board switches.

## Typical USB serial port names

| Platform | Typical CP2102 device name | GX5s example |
|---|---|---|
| macOS | `/dev/cu.SLAB_USBtoUART` or `/dev/cu.usbserial-*` | `/dev/cu.usbserial-0001` |
| Linux | `/dev/ttyUSB0` or `/dev/ttyUSB*` | Usually `/dev/ttyUSB0` |

On macOS, list serial devices with `ls /dev/cu.*`. On Linux, list USB serial devices with `ls /dev/ttyUSB*` or inspect `dmesg` after connecting the board. The exact suffix depends on the CP2102 driver and connected-device history.

## USB-C connector: USB1

| USB-C function | GX5s connection |
|---|---|
| VBUS | `VBUS-USB` |
| USB D+ | `USB-D+` |
| USB D- | `USB-D-` |
| CC1 | R45, 5.1 kΩ pull-down |
| CC2 | R46, 5.1 kΩ pull-down |
| SBU1/SBU2 | No functional connection shown |
| Shield/shell | GND |

R45 and R46 are the USB-C device/sink configuration resistors. The VBUS path includes C16, 1 µF, and C12, 100 nF in the local USB input area.

## USB power behavior

- The **CP2102 is powered only via the USB port** through `VBUS-USB`.
- The **GX5s Logger Board can be powered via the USB-C port**.
- Confirm the board-level USB power barriers and current limits against the assembled hardware.

## Enclosure Integration Note

When the GX5s Logger Board is installed in its enclosure, a **long-reach USB-C connector** is required so the plug can extend through the enclosure wall and fully insert into USB-C Connector. 


## CP2102-GMR specifications

| Property | Specification |
|---|---|
| Device | Silicon Labs CP2102-GMR |
| Function | USB 2.0 full-speed USB-to-UART bridge |
| USB speed | Full-speed, 12 Mbps |
| UART data rate | 300 bps to 1 Mbps |
| Package | 28-pin QFN, 5 mm x 5 mm |
| UART format | 5-8 data bits, 1/1.5/2 stop bits, configurable parity |
| CP2102 supply | USB-port powered only through `VBUS-USB` |
| Local interface | UART plus modem-control signals |

## CP2102 pin and board mapping

| U8 pin | CP2102 signal | GX5s connection | Function |
|---:|---|---|---|
| 1 | DCD | No functional net shown | Modem-control input |
| 2 | RI | No functional net shown | Modem-control input |
| 3 | GND | GND | Ground |
| 4 | D+ | `USB-D+` | USB data positive |
| 5 | D- | `USB-D-` | USB data negative |
| 6 | VDD | Local bridge supply | Digital supply |
| 7 | REGIN | `VBUS-USB` | Regulator input |
| 8 | VBUS | `VBUS-USB` | USB bus voltage sense/supply |
| 9 | `/RST` | Reset support | CP2102 reset |
| 10 | NC | No connection shown | Leave unconnected |
| 11 | SUSPEND | No functional net shown | Suspend output |
| 12 | SUSPEND | No functional net shown | Inverted suspend output |
| 13-22 | NC | No connection shown | Leave unconnected |
| 23 | CTS | No functional net shown | UART modem-control input |
| 24 | RTS | `MODE` through Q13/R35 | Boot-mode control path |
| 25 | RXD | `USB-TX` | Data from ESP32 U0TXD into CP2102 |
| 26 | TXD | `USB-RX` | Data from CP2102 into ESP32 U0RXD |
| 27 | DSR | No functional net shown | Modem-control input |
| 28 | DTR | `RESET` through Q1/R31 | Automatic reset path |

## ESP32 serial and programming path

| ESP32 signal | GX5s net | CP2102 signal |
|---|---|---|
| U0TXD | `USB-TX` | RXD, U8 pin 25 |
| U0RXD | `USB-RX` | TXD, U8 pin 26 |
| EN / RESET | `RESET` | DTR through Q1/R31 |
| GPIO0 / MODE | `MODE` | RTS through Q13/R35 |

The schematic labels the DTR network **Auto Reset** and the RTS network **MODE**. This is the standard arrangement for entering the ESP32 serial bootloader from a USB programming tool and performing an automatic reset after programming.

## Debugging workflow

1. Connect a USB-C cable to USB1.
2. Confirm the host enumerates a CP210x virtual COM port.
3. Use the UART connected to `USB-TX`/`USB-RX` for ESP32 logs and programming.
4. Assert `MODE` during reset to enter the ESP32 bootloader.
5. Release `MODE` and reset again for normal application startup.

### Using esptool

`esptool` can use the CP2102 DTR/RTS control lines to reset the ESP32 and enter the serial bootloader automatically. A read-only chip check is:

```sh
esptool --port /dev/cu.usbserial-0001 chip-id       # macOS example
esptool --port /dev/ttyUSB0 chip-id                 # Linux example
```

The same automatic reset behavior is used by supported flash and erase commands. Confirm the target port before running any command that writes or erases flash.

## Source

[Silicon Labs CP2102 product page and documentation](https://www.silabs.com/interface/usb-bridges/classic/device.cp2102)

---

<table align="center"><tr><td align="center"><a href="https://reactiot.com.au"><img src="../../../design/handoff_reactiot_website/assets/logo.svg" alt="React IoT" width="96" height="42"><br>reactiot.com.au</a></td><td width="32"></td><td align="center"><a href="https://www.srmiles.com"><img src="https://srmiles.com/assets/srm-logo.svg" alt="srmiles" width="168" height="42"><br>srmiles.com</a></td></tr></table>
