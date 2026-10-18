# GX5s Logger Board - Modem Interface and eSIM

**References:** X2 Mini PCIe connector  
**Function:** Cellular modem interface and subscriber identity interface  

## Overview

The GX5s Logger Board exposes a cellular modem through an X2 Mini PCIe connector and provides an MFF2 eSIM footprint.

## Mini PCIe connector

| Group | Board nets |
|---|---|
| Power | `+3V3`, `+3V3AUX`, `+1V5`, GND |
| Host UART | `GSM-TX`, `GSM-RX` |
| Control/status | `GSM-WAKE`, `GSM-INT`, `GSM-RI`, `GSM-SLP`, `GSM-RST` |
| UIM/eSIM | `USIM_VCC`, `USIM_DATA`, `USIM_CLK`, `USIM_RST`, `USIM_VPP` |
| USB | `USB-D+`, `USB-D-` |
| Modem indicators | `GSM-LED` / blue GSM LED |

### Host-side modem signal mapping

| Modem function | GX5s net | Connection/behavior |
|---|---|---|
| Wake | `GSM-WAKE` | Low active |
| Interrupt | `GSM-INT` | High active after the schematic's inversion stage |
| Ring indicator | `GSM-RI` | Low active |
| Sleep control | `GSM-SLP` | Routed to GPIO expander U12 |
| Reset | `GSM-RST` | Driven from ESP32-S |
| UART transmit | `GSM-TX` | MCU-to-modem serial path |
| UART receive | `GSM-RX` | Modem-to-MCU serial path |

### Modem status LED

`GSM-LED` drives a blue GSM connection LED through R16, This LED is only controlled by the Modem

## MFF2 eSIM: G1 (Not Installed)

G1 is an MFF2 eSIM footprint. Its signal mapping is:

| eSIM pin/function | GX5s net |
|---|---|
| VCC | `USIM_VCC` |
| VPP | `USIM_VPP` |
| I/O | `USIM_DATA` |
| RST | `USIM_RST` |
| CLK | `USIM_CLK` |
| GND | GND |

---

<table align="center"><tr><td align="center"><a href="https://reactiot.com.au"><img src="../../../design/handoff_reactiot_website/assets/logo.svg" alt="React IoT" width="96" height="42"><br>reactiot.com.au</a></td><td width="32"></td><td align="center"><a href="https://www.srmiles.com"><img src="https://srmiles.com/assets/srm-logo.svg" alt="srmiles" width="168" height="42"><br>srmiles.com</a></td></tr></table>
