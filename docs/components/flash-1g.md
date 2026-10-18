# GX5s Logger Board - 1 GB Flash Storage

**Reference:** U3  
**Part:** XTX `XTSD08GLGEAG`  
**Storage type:** SD NAND with integrated controller  

## Overview

U3 is an XTX SD NAND device. The `08G` density code identifies an **8 Gbit device, equivalent to 1 GByte nominal capacity**. This is separate from the ESP32 module's boot flash.

The device supports SD 2.0-style operation and SPI mode. It includes an internal NAND controller, hardware ECC, and bad-block management, so firmware should treat it as managed flash storage rather than raw NAND.

## Key specifications

| Property | Specification |
|---|---|
| Nominal capacity | 8 Gbit / 1 GByte |
| Interface | SD 2.0 or SPI mode |
| Package | LGA8, 8 mm x 6 mm |
| Supply voltage | 2.7 V to 3.6 V |
| GX5s supply net | `SD-3V3` |
| Maximum clock | 50 MHz |
| Temperature range | -30 °C to +85 °C operating |
| Storage temperature | -40 °C to +85 °C |
| Controller features | Hardware ECC and bad-block management |
| Maximum read/write operating current | 30 mA at datasheet test conditions |
| Maximum standby current | 20 mA at datasheet test conditions |
| Population status | Optional according to schematic |

## Pin assignment

| U3 pin | SD mode | SPI mode | GX5s connection |
|---:|---|---|---|
| 1 | SD2 / I/O | NC | No connection shown |
| 2 | SD3 / I/O | CS | `SD_CS` from ESP32 GPIO5 |
| 3 | CLK | CLK | `SD_CLK` from ESP32 GPIO18 |
| 4 | VSS | VSS | GND |
| 5 | CMD | DI | `SD_MOSI` from ESP32 GPIO4 |
| 6 | SD0 / I/O | DO | `SD_MISO` to ESP32 GPIO16 |
| 7 | SD1 / I/O | NC | No connection shown |
| 8 | VDD | VDD | `SD-3V3` / 3.3 V |

## ESP32-to-U3 SPI mapping

| ESP32 function | ESP32 GPIO | GX5s net | U3 pin |
|---|---:|---|---:|
| SPI chip select | GPIO5 | `SD_CS` | 2 |
| SPI clock | GPIO18 | `SD_CLK` | 3 |
| SPI MOSI / data in | GPIO4 | `SD_MOSI` | 5 |
| SPI MISO / data out | GPIO16 | `SD_MISO` | 6 |
| Supply | — | `SD-3V3` | 8 |
| Ground | — | GND | 4 |

## Integration notes

- The schematic names the signals `SD_*`, but the connections are the SPI-mode mapping shown above.

- The 1 GB U3 device should not be confused with the ESP32 module boot flash. 

  

## Source

[Local XTX XTSD01G/XTSD02G/XTSD04G/XTSD08G datasheet](../references/XTX-XTSD08G-SD-NAND-datasheet.pdf) ([online source](https://datasheet.lcsc.com/datasheet/pdf/c17c51abcda5dcbe5d6bc064f785e2dc.pdf))

---

<table align="center"><tr><td align="center"><a href="https://reactiot.com.au"><img src="../../../design/handoff_reactiot_website/assets/logo.svg" alt="React IoT" width="96" height="42"><br>reactiot.com.au</a></td><td width="32"></td><td align="center"><a href="https://www.srmiles.com"><img src="https://srmiles.com/assets/srm-logo.svg" alt="srmiles" width="168" height="42"><br>srmiles.com</a></td></tr></table>
