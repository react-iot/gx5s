# GX5s Logger Board - Real-Time Clock

**Part marking:** `PCF8563T/5,518`  
**Function:** Real-time clock and calendar  

## Overview

U17 is an NXP PCF8563 low-power real-time clock/calendar connected to the shared I2C bus. It uses an external 32.768 kHz crystal at X1 and provides time/calendar registers, alarm and timer functions, a programmable clock output, and a voltage-low detector.

## Key specifications

| Property | Specification |
|---|---|
| Device | NXP PCF8563T/5 |
| Package | SO8, 8 leads |
| Supply range | 1.0 V to 5.5 V at room temperature; 1.8 V to 5.5 V with active 400 kHz I2C |
| Time base | External 32.768 kHz quartz crystal |
| I2C speed | Up to 400 kbit/s |
| I2C address | Write `0x51` / `0xA2`; read `0x51` / `0xA3` |
| Backup current | Typical 0.25 µA at 3.0 V, 25 °C |
| Clock output | Programmable 32.768 kHz, 1.024 kHz, 32 Hz, or 1 Hz |
| Interrupt output | Open-drain, active-low |
| Functions | Calendar, alarm, timer, clock output, voltage-low detection, POR |

## Pin mapping

| U17 pin | PCF8563 pin | Function | GX5s connection |
|---:|---|---|---|
| 1 | OSCI | Oscillator input | X1 crystal |
| 2 | OSCO | Oscillator output | X1 crystal |
| 3 | INT | Open-drain interrupt output | No external functional net shown |
| 4 | VSS | Ground | GND |
| 5 | SDA | I2C data | `I2C-SDA` to ESP32 GPIO14 |
| 6 | SCL | I2C clock | `I2C-SCL` to ESP32 GPIO27 |
| 7 | CLKOUT | Programmable clock output | No external functional net shown |
| 8 | VDD | Supply | `VBUS` / RTC supply rail |

## Supporting components

| Reference | Value/function |
|---|---|
| X1 | 32.768 kHz RTC crystal |
| C14 | 100 nF local supply decoupling capacitor |

The RTC supply can be provided through either of the board's intended power sources:

- the external wire harness supply; or
- the internal connected battery, when the optional battery is fitted.

The schematic labels the U17 supply net `VBUS`; the exact source-selection and isolation circuit should be confirmed against the PCB/BOM. The internal battery path is optional and should not be assumed on every board build.

## Firmware 

- Use the shared I2C bus on `I2C-SDA` and `I2C-SCL`.
- The 7-bit target address is `0x51`.
- Treat the interrupt as active-low and open-drain if it is later routed to the MCU.
- Initialise and validate the oscillator-stop/voltage-low status before trusting the time value.
- Use the external crystal configuration represented by X1.

## Verification items

- Confirm the assembled crystal frequency and load characteristics.
- Confirm the `VBUS` voltage at U17 VDD for both external-harness and battery-powered operation.
- Confirm whether the optional internal battery and its power-selection/isolation circuit are fitted.
- Confirm whether U17 INT or CLKOUT is intentionally left unconnected.
- Scan the live I2C bus for target address `0x51`.

## Source

[Local NXP PCF8563 product data sheet](../references/NXP-PCF8563-datasheet.pdf) ([online source](https://www.nxp.com/docs/en/data-sheet/PCF8563.pdf))

---

<table align="center"><tr><td align="center"><a href="https://reactiot.com.au"><img src="../../../design/handoff_reactiot_website/assets/logo.svg" alt="React IoT" width="96" height="42"><br>reactiot.com.au</a></td><td width="32"></td><td align="center"><a href="https://www.srmiles.com"><img src="https://srmiles.com/assets/srm-logo.svg" alt="srmiles" width="168" height="42"><br>srmiles.com</a></td></tr></table>
