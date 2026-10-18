# GX5s Logger Board - TCA9555 GPIO Expander

**Part:** Texas Instruments `TCA9555RTWR`  
**Function:** 16-bit I2C/SMBus GPIO expander  

## Overview

U12 provides sixteen remotely controlled digital I/O lines over the shared I2C bus. The GX5s uses the device for charger controls, status LEDs, modem sleep control, and auxiliary signals.

## Key specifications

| Property | Specification |
|---|---|
| Device | TI TCA9555RTWR |
| Function | 16-bit I2C/SMBus I/O expander |
| Package | 24-pin WQFN, RTW, 4 mm x 4 mm |
| Supply range | 1.65 V to 5.5 V |
| GX5s supply | `3V3` |
| I2C speed | Standard and Fast mode, up to 400 kHz |
| Address selection | A0, A1, A2; up to eight devices on one bus |
| Port structure | Push-pull I/O; inputs after power-on reset |
| Interrupt | Open-drain interrupt output |
| Operating temperature | -40 °C to +85 °C |
| GPIO low-level sink capability | Up to 8 mA at the specified test condition |

## I2C and control pin mapping

| U12 pin | Signal | GX5s connection |
|---:|---|---|
| 18 | A0 | GND in schematic |
| 23 | A1 | GND in schematic |
| 24 | A2 | GND in schematic |
| 22 | INT# | No clearly routed MCU connection shown; verify PCB/BOM |
| 21 | VCC | `3V3` |
| 20 | SDA | `I2C-SDA` |
| 19 | SCL | `I2C-SCL` |
| 9 | GND | GND |
| 25 | EP | Exposed pad; verify grounding requirement for the RTW package |

With A0, A1, and A2 all low, the expected 7-bit I2C address is `0x20`. Confirm this with a live bus scan because the schematic symbol wiring should be checked against the assembled board.

## Port mapping

| TCA9555 port | U12 pin | GX5s net/function | Direction/use |
|---|---:|---|---|
| P00 | 1 | `IO-40` | Auxiliary I/O |
| P01 | 2 | `IO-41` | Auxiliary I/O |
| P02 | 3 | `IO-42` | Auxiliary I/O |
| P03 | 4 | `IO-43` | Auxiliary I/O |
| P04 | 5 | `BAT-CHRG` | Battery-charge status/control |
| P05 | 6 | `CHRG-EN` | Charger enable |
| P06 | 7 | No connection shown | Unused |
| P07 | 8 | No connection shown | Unused |
| P10 | 10 | No connection shown | Unused |
| P11 | 11 | No connection shown | Unused |
| P12 | 12 | `LED-SVR` | Green connection-status LED, high-active |
| P13 | 13 | `GSM-SLP` | Modem sleep control |
| P14 | 14 | `LED-IG` | Yellow ACC/ignition-status LED, high-active |
| P15 | 15 | No connection shown | Unused |
| P16 | 16 | No connection shown | Unused |
| P17 | 17 | `APS-RST` | Motion-sensor reset |

The TCA9555 port names above follow TI's nomenclature: P00-P07 are port 0 and P10-P17 are port 1. The schematic's `IO-40` through `IO-43` labels are board-level net names, not TCA9555 register addresses.

## Local support

| Reference | Value/function |
|---|---|
| C23 | 100 nF supply decoupling capacitor |

## Firmware integration

- Configure the device at I2C address `0x20` if the address straps are confirmed low.
- Set the intended outputs before enabling charger, modem-sleep, reset, or LED functions.
- Remember that ports power up as inputs.
- Treat `LED-SVR` and `LED-IG` as high-active according to the schematic annotations.
- Confirm whether `INT#` is intentionally unused; the schematic does not show a clear MCU interrupt net for U12.

## Verification items

- Confirm the physical RTW package and exposed-pad connection.
- Confirm A0/A1/A2 strap levels on the PCB.
- Scan the live I2C bus for address `0x20`.
- Confirm the exact P17/P15/P16 wiring against the assembled PCB, especially the `APS-RST` route.
- Confirm output-current limits for any externally loaded GPIO.

## Source

[Local Texas Instruments TCA9555 datasheet](../references/TI-TCA9555-datasheet.pdf) ([online source](https://www.ti.com/lit/ds/symlink/tca9555.pdf))

---

<table align="center"><tr><td align="center"><a href="https://reactiot.com.au"><img src="../../../design/handoff_reactiot_website/assets/logo.svg" alt="React IoT" width="96" height="42"><br>reactiot.com.au</a></td><td width="32"></td><td align="center"><a href="https://www.srmiles.com"><img src="https://srmiles.com/assets/srm-logo.svg" alt="srmiles" width="168" height="42"><br>srmiles.com</a></td></tr></table>
