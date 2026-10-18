# GX5s Logger Board - ESP32 Module

**Role:** Main controller, communications coordinator, sensor interface, and power-state controller.  
**Schematic identifier:** `ESP32-S`  
**Source schematic:** `SCH_GX5s-v210620_2026-09-19.pdf`, Sheet 1/5

## Module 

| Property | Observed value |
|---|---|
| Chip | ESP32-D0WD-V3 |
| Silicon revision | 3.0 |
| CPU | Dual-core, 240 MHz |
| Wireless features | Wi-Fi and Bluetooth |
| Crystal | 40 MHz |
| Flash manufacturer ID | `0x20` |
| Flash device ID | `0x4018` |
| Detected flash capacity | 16 MB |
| Flash voltage | 3.3 V |
| PSRAM | None |

The board also reported MAC address `c4:dd:57:e0:9f:ac`. This device-specific identifier is recorded here for the hardware readout but is not a board-wide specification.

## Identification status

The schematic identifies the device only as `ESP32-S`; it does not show a complete Espressif module ordering code such as an ESP32-WROOM or ESP32-WROVER variant. The exact module memory size, antenna variant, package pin numbering, and fitted flash/PSRAM configuration therefore require confirmation from the physical module marking or approved BOM.

Espressif documents different ESP32 module variants with different flash and PSRAM configurations. Use the fitted module marking and the corresponding module datasheet before publishing fixed memory numbers.

## Memory and storage

| Storage/resource | Evidence in GX5s schematic | Status |
|---|---|---|
| ESP32 internal ROM/RAM | ESP32-D0WD-V3 observed | Confirm exact usable regions from the chip datasheet |
| ESP32 module boot flash | Read from connected board | 16 MB detected; manufacturer/device IDs `0x20` / `0x4018` |
| Optional SD NAND, U3 | `XTSD08GLGEAG`, labelled **Flash Storage - Optional** | 8 Gbit / 1 GByte nominal; see [1 GB Flash Storage](flash-1g.md) |
| PSRAM | No PSRAM device or dedicated PSRAM wiring shown | Do not assume PSRAM is fitted |
| RTC timekeeping | U17 `PCF8563T/5,518` | Separate real-time clock, not MCU program storage |
| eSIM data | G1 MFF2 eSIM | Modem identity storage, not MCU storage |

## Optional external flash pinout

| U3 signal | GX5s net | ESP32-S GPIO |
|---|---|---:|
| CS | `SD_CS` | GPIO5 |
| CLK | `SD_CLK` | GPIO18 |
| DO / MISO | `SD_MISO` | GPIO16 |
| DI / MOSI | `SD_MOSI` | GPIO4 |
| VDD | `SD-3V3` | 3.3 V rail |
| VSS | GND | GND |

The external flash block is explicitly marked optional in the schematic. The connected-board inspection detected 16 MB of ESP32 boot flash; that query does not identify the separate U3 SD NAND device.

## Functional pinout

This is the board-level signal map shown by the schematic. It is a functional net map rather than a physical module pin-number table because the exact ESP32-S module variant is not identified.

| MCU function | GX5s net / connected function |
|---|---|
| EN / reset | `RESET`, reset switch |
| GPIO36 | `ADC-VCC` |
| GPIO39 | `VCC-INT` |
| GPIO34 | `ACC-INT` |
| GPIO35 | `BAT-ADC` |
| GPIO32 | `CAN-RX` |
| GPIO33 | `CAN-TX` |
| GPIO25 | `GSM-INT` |
| GPIO26 | `INT` |
| GPIO27 | `I2C-SCL` |
| GPIO14 | `I2C-SDA` |
| GPIO12 | `APS-INT` |
| U0TXD | `USB-TX` |
| U0RXD | `USB-RX` |
| GPIO21 | `SAT-RX` |
| GPIO19 | `SAT-TX` |
| GPIO5 | `SD_CS` |
| GPIO18 | `SD_CLK` |
| GPIO17 | `GSM-RST` |
| GPIO16 | `SD_MISO` |
| GPIO4 | `SD_MOSI` |
| GPIO0 | `MODE` / boot-mode switch |
| GPIO2 | `SD-3V3` / optional flash power control |
| GPIO15 | `3V3-PSM` / power-management control, high = on |
| GPIO22 | `GSM-TX` |
| GPIO23 | No functional net shown |

## Power, reset, and boot controls

| Function | Board connection | Schematic note |
|---|---|---|
| 3.3 V supply | `3V3` | Main MCU supply rail |
| Ground | GND | Multiple ground connections shown |
| Enable/reset | `RESET` / EN | Reset switch and USB auto-reset circuitry |
| Boot mode | GPIO0 / `MODE` | GPIO0 must be high at boot |
| Boot strap | GPIO12 | GPIO12 must be low to boot |
| Boot messages | GPIO15 | Low disables boot messages |

GPIO2 is also labelled `SD-3V3` on the schematic and feeds the optional U3 flash supply. GPIO15 is labelled `3V3-PSM` and is marked high-active for power management. Verify the external switching/current path before treating GPIO2 as a direct power source.

The schematic also notes these wake conditions:

- `VCC-INT` must go high to wake the MCU.
- `ACC-INT` must go high to wake the MCU.
- `APS-INT` must go high to wake the MCU.
- `INT` must go high to wake the MCU.

## Verification items

- Confirm the exact ESP32 module ordering code.
- Confirm the physical module flash arrangement and whether PSRAM is fitted.
- Confirm the physical module package pinout.
- Confirm whether optional U3 is populated on the assembled board.
- Confirm the external flash capacity and manufacturer from the BOM.
- Confirm GPIO strap behavior on the production revision.

## Reference

[Espressif ESP32 Modules and Boards documentation](https://docs.espressif.com/projects/esp-idf/en/v3.3/hw-reference/modules-and-boards.html)

---

<table align="center"><tr><td align="center"><a href="https://reactiot.com.au"><img src="../../../design/handoff_reactiot_website/assets/logo.svg" alt="React IoT" width="96" height="42"><br>reactiot.com.au</a></td><td width="32"></td><td align="center"><a href="https://www.srmiles.com"><img src="https://srmiles.com/assets/srm-logo.svg" alt="srmiles" width="168" height="42"><br>srmiles.com</a></td></tr></table>
