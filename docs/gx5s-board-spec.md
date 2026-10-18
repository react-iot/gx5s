# GX5s Development Board Specification

**Board identifier:** GX5s  
**Board Version:** `v2106200`  
**Schematic date:** 2021-06-20  

Reference PDFs: [component datasheets](references/README.md)

---

## 1. Board Capabilites

The GX5s Logger Development Board is built around an ESP32-S MCU.

- ESP32-S MCU (16mb Flash)
- 1G OnBoard SD Storage Additional API Flash
- 9 ~ 36 V nominal-class external Voltage
- External Interupt/Wake Detection
- External Harness Connector (Power, Voltage Detection, UART, I2C, CAN 2.0B)
- Mini PCIe connector Modem Interface
- USB-C service/power input through a CP2102 USB-to-TTL bridge.
- An MPU-6050 motion sensor on I2C.
- A PCF8563 real-time clock on I2C.
- A TCA9555 16-bit I/O expander for auxiliary controls, charger signals, modem sleep, and status LEDs.
- Lipo Battery Charging
- Reset, boot-mode, power, ignition, and server-connection indicators.

### Functional block diagram

```text
External wiring
        │
        ├── 36 V → 5 V switching supply → 3V3 regulator → ESP32-S MCU
        ├── ACC / VCC detection ──────────────────────────┘
        ├── CANH/CANL → SN65HVD230 CAN transceiver → ESP32-S
        └── SAT-RX/SAT-TX + I2C → harness connector

USB-C → CP2102 USB-to-TTL → ESP32-S UART
ESP32-S UART/controls → Mini PCIe modem → MFF2 eSIM
ESP32-S I2C → MPU-6050, PCF8563 RTC, LTC4311 active terminator
ESP32-S I2C → TCA9555 GPIO expander → LEDs, charger, modem sleep/reset
ESP32-S SPI → optional flash
```



## 3. External interfaces

### 3.1 External wire harness: U7

The harness is labelled **Molex 2x6 Micro-fit 3.0**. The schematic labels the connector pins as follows:

| Harness pin | Signal | Direction/use |
|---:|---|---|
| 1 | GND (INPUT) | External ground |
| 2 | VCC/PWR (INPUT) | External 9~36V Power |
| 3 | GND | 5V GND Reference |
| 4 | 5V0 | 5V Power Rail |
| 5 | INT | MCU Interupt (3V3) |
| 6 | Voltage Detection | 5~36V Power Detection |
| 7 | CAN -Low | CAN Bus 2.0B Low |
| 8 |  | CAN Bus 2.0B HIGH |
| 9 | UART RX | UART RX (3V3 |
| 10 | Unlabelled in schematic | Requires hardware review |
| 11 | UART-TX | Serial output to satellite/GNSS device |
| 12 | I2C-SDA | I2C data |

The power sheet also labels the incoming external-input nets `VCC-IN` and `GND-IN`; the exact connector-to-harness pin mapping should be confirmed from the assembled connector and wiring harness drawing.

### 3.2 USB-C service connector: USB1

USB1 provides USB VBUS and USB data. `R45` and `R46` are both 5.1 kΩ CC pull-down resistors, consistent with a USB-C sink/device configuration. The schematic labels the data nets `USB-D+` and `USB-D-`, and the input power net `VBUS-USB`.

### 3.3 Mini PCIe modem connector: X2

X2 is labelled `MPCIE-JAE`. The schematic exposes:

- Modem power: `+3V3`, `+3V3AUX`, `+1V5`, and ground.
- SIM/UIM: `USIM_VCC`, `USIM_DATA`, `USIM_CLK`, `USIM_RST`, `USIM_VPP`.
- Host control/status: `GSM-WAKE`, `GSM-INT`, `GSM-RI`, `GSM-SLP`, `GSM-RST`.
- Host UART: `GSM-TX`, `GSM-RX`.
- USB differential pair: `USB-D+`, `USB-D-`.
- Modem status LED: `GSM-LED` drives a blue LED through `R16` (1 kΩ).

The modem’s RF connector, antenna matching, and antenna path are not represented on the supplied sheets and should not be inferred from this report.

### 3.4 Embedded SIM: G1

G1 is an MFF2 eSIM footprint. The schematic shows:

| eSIM signal | Board net |
|---|---|
| VCC | `USIM_VCC` |
| VPP | `USIM_VPP` |
| I/O | `USIM_DATA` |
| RST | `USIM_RST` |
| CLK | `USIM_CLK` |
| GND | GND |

`R40` is 15 kΩ on the SIM support circuitry. Capacitors shown around the eSIM include 33 pF and 100 nF parts.

## 4. Major component pages

Each section below is intended to become one report page in a rendered document.

### Page A - ESP32-S module: memory, storage, and pinout

Standalone component page: [ESP32 Module](components/esp32-module.md)

Related storage page: [1 GB Flash Storage](components/flash-1g.md)

Related RTC page: [PCF8563 Real-Time Clock](components/rtc-pcf8563.md)

Related GPIO page: [TCA9555 GPIO Expander](components/gpio-expander-tca9555.md)

Related USB page: [USB Debugging](components/usb-debugging.md)

**Reference/designator:** ESP32-S  
**Role:** Main controller, communications coordinator, sensor interface, and power-state controller.

#### Verified from connected board

The connected board identifies itself as an **ESP32-D0WD-V3, silicon revision 3.0**, with dual-core 240 MHz processing, Wi-Fi/Bluetooth, a 40 MHz crystal, and **16 MB of ESP32 boot flash**. The flash voltage was reported as 3.3 V. PSRAM was not reported by the read-only inspection query and should not be assumed. This is separate from optional U3, the 1 GByte SD NAND device.

#### Identification status

The schematic identifies the device only as `ESP32-S`; it does not show a complete Espressif module ordering code such as an ESP32-WROOM or ESP32-WROVER variant. Therefore the exact module memory size, antenna variant, package pin numbering, and fitted flash/PSRAM configuration cannot be confirmed from the schematic alone.

Espressif documents different ESP32 module variants with different flash and PSRAM configurations. Use the fitted module marking or the approved BOM to select the correct module datasheet before publishing fixed memory numbers.

#### Memory and storage

| Storage/resource | Evidence in GX5s schematic | Documentation status |
|---|---|---|
| ESP32 internal ROM/RAM | Part is identified as ESP32-S | Capacity depends on the exact ESP32 variant; confirm from module marking/datasheet |
| Module SPI flash | No module ordering code or flash size is shown | Expected to hold firmware and non-volatile application data, but capacity is unconfirmed |
| Optional SD NAND, U3 | `XTSD08GLGEAG`, labelled **Flash Storage - Optional** | 8 Gbit / 1 GByte nominal managed SD NAND; see [1 GB Flash Storage](components/flash-1g.md) |
| PSRAM | No PSRAM device or dedicated PSRAM wiring is shown | Do not assume PSRAM is fitted |
| RTC timekeeping | U17 `PCF8563T/5,518` | Separate real-time clock, not MCU program storage |
| eSIM data | G1 MFF2 eSIM | Modem identity storage, not MCU storage |

#### Optional external flash pinout

The optional U3 device is connected as a standard four-wire SPI memory device:

| U3 signal | GX5s net | ESP32-S GPIO |
|---|---|---:|
| CS | `SD_CS` | GPIO5 |
| CLK | `SD_CLK` | GPIO18 |
| DO / MISO | `SD_MISO` | GPIO16 |
| DI / MOSI | `SD_MOSI` | GPIO4 |
| VDD | `SD-3V3` | 3.3 V rail |
| VSS | GND | GND |

#### Functional pinout

The table below is the board-level signal map shown by the schematic. It is intentionally expressed as a functional map rather than a physical module pin-number table because the exact ESP32-S module variant is not identified.

#### Schematic connections

| MCU function | Net / connected function |
|---|---|
| `EN` / reset | `RESET`, reset switch |
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
| GPIO23 | no functional net shown |
| GPIO22 | `GSM-TX` |

#### Power, reset, and boot controls

| Function | Board connection | Notes |
|---|---|---|
| 3.3 V supply | `3V3` | Main MCU supply rail |
| Ground | GND | Multiple ground connections shown |
| Enable/reset | `RESET` / EN | Reset switch and USB auto-reset circuitry |
| Boot mode | GPIO0 / `MODE` | Boot-mode switch; schematic note says GPIO0 must be high at boot |
| Boot strap | GPIO12 | Schematic note says GPIO12 must be low to boot |
| Boot messages | GPIO15 | Schematic note says low disables boot messages |

The ESP32-S page is therefore complete for firmware integration at the net level, but a production memory specification still needs the physical module marking or approved BOM.

For module-level memory variants, use Espressif's [ESP32 Modules and Boards documentation](https://docs.espressif.com/projects/esp-idf/en/v3.3/hw-reference/modules-and-boards.html) together with the exact module datasheet.

The schematic notes these boot and wake requirements:

- GPIO0 must be high at boot.
- GPIO12 must be low to boot.
- GPIO15 is low to disable boot messages.
- `VCC-INT` must go high to wake the MCU.
- `ACC-INT` must go high to wake the MCU.
- `APS-INT` must go high to wake the MCU.
- `INT` must go high to wake the MCU.

**Datasheet items to add later:** exact module part number, flash/PSRAM configuration, CPU clock, operating voltage limits, current consumption, antenna arrangement, physical package pinout, and validated boot strap behavior.

### Page B - CAN transceiver

**Reference:** U4  
**Part marking:** `SN65HVD230DR`  
**Role:** Converts the MCU’s logic-level CAN signals to the differential CAN bus.

| U4 function | Board connection |
|---|---|
| CANH | External harness `CANH` |
| CANL | External harness `CANL` |
| D / TXD | `CAN-TX` from ESP32-S |
| R / RXD | `CAN-RX` to ESP32-S |
| VCC | 3.3 V |
| RS | `CAN-EN` |
| GND | GND |

Protection is provided by D7, labelled `PESD2CAN`. `C30` is a 100 µF capacitor on the CAN/transceiver supply area. Termination resistance is not shown on this sheet; confirm whether it is fitted elsewhere or supplied externally.

### Page C - Cellular modem interface and eSIM

Standalone component page: [Modem Interface and eSIM](components/modem-interface.md)

**References:** X2 Mini PCIe connector, G1 MFF2 eSIM  
**Role:** Cellular data connection and subscriber identity interface.

The modem module is connected through X2 and the MFF2 eSIM through G1. See the standalone page for connector groups, signal polarity, eSIM mapping, and verification items.

### Page D - MPU-6050 motion sensor

**Reference:** U1  
**Part marking:** `MPU-6050_C24112`  
**Schematic label:** APS (MPU-6050)  
**Role:** 6-axis inertial measurement: accelerometer and gyroscope.

| Sensor function | Board connection |
|---|---|
| SDA | `I2C-SDA` |
| SCL | `I2C-SCL` |
| INT | `APS-INT` |
| AD0 | GND |
| VLOGIC | 3.3 V |
| VDD | 3.3 V |
| GND | GND |
| CPOUT | C22, 10 nF |
| REGOUT | C24, 100 nF |
| FSYNC | GND |
| AUX_DA/AUX_CL | No external functional connection shown |

Local capacitors include C22 10 nF, C24 100 nF, C29 2.2 nF, and C38 100 nF. The I2C address implied by AD0 tied low should be confirmed against the selected MPU-6050 variant.

### Page E - Real-time clock

Standalone component page: [PCF8563 Real-Time Clock](components/rtc-pcf8563.md)

**Reference:** U17  
**Part marking:** `PCF8563T/5,518`  
**Role:** Battery-backed/calendar timekeeping over I2C.

| RTC function | Board connection |
|---|---|
| SDA | `I2C-SDA` |
| SCL | `I2C-SCL` |
| INT | No functional net shown in the schematic extract |
| OSCI/OSCO | X1 crystal |
| VDD | `VBUS` / RTC supply net; external wire harness or optional internal battery |
| VSS | GND |

C14 is 100 nF at the RTC supply. PCF8563T may be powered from the wire harness or from the internal connected battery when that optional battery path is fitted; confirm the source-selection/isolation circuit against the PCB and BOM.

### Page F - GPIO expander

Standalone component page: [TCA9555 GPIO Expander](components/gpio-expander-tca9555.md)

**Reference:** U12  
**Part marking:** `TCA9555RTWR`  
**Role:** Adds low-speed digital I/O for power, modem, charger, and status functions.

| Expander output | Board function |
|---|---|
| P00 | `IO-40` |
| P01 | `IO-42` |
| P02 | `IO-43` |
| P03 | `BAT-CHRG` |
| P04 | `CHRG-EN` |
| P05 | No functional net shown |
| P06 | No functional net shown |
| P07 | No functional net shown |
| P10 | No functional net shown |
| P11 | No functional net shown |
| P12 | `LED-SVR` / connected indicator |
| P13 | `GSM-SLP` |
| P14 | `LED-IG` / ignition indicator |
| P15 | No functional net shown |
| P16 | No functional net shown |
| P17 | `APS-RST` |

The expander is on the shared I2C bus. A0, A1, and A2 are tied according to the schematic symbol wiring; confirm the resulting I2C address during firmware integration.

### Page G - Power conversion and battery charging

**Primary references:** U18 `TPS5430DDAR`, U10 `TPS63001DRCR`, U5 `TP4056`  
**Role:** External-input protection/conversion, 3.3 V regulation, and single-cell battery charging.

#### External-input path

- Input is labelled `VCC-IN` / `GND-IN` and the sheet title describes a **36 V to 5 V switching power supply**.
- F2 is marked `SMD1812P110TF/33`.
- D10 is marked 33 V / 2 A and D5 is marked `SMAJ34A` in the input protection area.
- U18 is marked `TPS5430DDAR`.
- The 5 V output is labelled `5V0`.
- `VCC-OUT` detection is routed to the MCU as `VCC-INT`; the schematic note says high means no power and low means power.

#### 5 V to 3.3 V path

- U10 is marked `TPS63001DRCR`.
- The output rail is labelled `3V3`.
- The input/output inductor markings include 2.2 µH and the surrounding capacitors include 10 µF-class parts; confirm values against the assembled BOM.

#### Battery charger

- U5 is marked `TP4056`.
- The sheet describes the charger as **3.7 V Li-poly single cell, with overcharge protection**.
- `CHRG-EN` and `BAT-CHRG` are routed to the GPIO expander.
- `BAT+` is the battery net; `BAT-ADC` is routed to the MCU for battery measurement.

#### ACC detection

The ACC circuit uses a transistor stage with 470 kΩ resistors and a TVS/protection part marked `SMAJ36A`. The sheet note says **low = ignition on** and **high = ignition off** at the relevant detected signal. Firmware should use the named net polarity rather than assuming ACC is active-high.

### Page H - USB-to-TTL service interface

Standalone component page: [USB Debugging](components/usb-debugging.md)

**Enclosure note:** A long-reach USB-C connector is required when the board is installed in the enclosure so the plug can extend through the enclosure wall and insert correctly.

The device can be reprogrammed over USB. The CP2102 virtual COM port and its DTR/RTS control signals support automatic ESP32 bootloader entry and reset after programming.

The CP2102 is powered only via the USB port, while the GX5s Logger Board itself can be powered via the USB-C port.

Typical serial device names are `/dev/cu.SLAB_USBtoUART` or `/dev/cu.usbserial-*` on macOS, and `/dev/ttyUSB0` or `/dev/ttyUSB*` on Linux. The tested board appeared as `/dev/cu.usbserial-0001` on macOS.

`esptool` can use the CP2102 DTR/RTS lines to reset the ESP32 and enter the serial bootloader automatically. For a read-only identification check, use `esptool --port /dev/cu.usbserial-0001 chip-id` on macOS or `esptool --port /dev/ttyUSB0 chip-id` on Linux.

**Reference:** U8  
**Part marking:** `CP2102-GMR`  
**Role:** USB service/programming interface for the ESP32-S UART.

| CP2102 signal | Board connection |
|---|---|
| USB D+ / D- | USB-C `USB-D+` / `USB-D-` |
| TXD | `USB-TX` to MCU UART input path |
| RXD | `USB-RX` from MCU UART output path |
| DTR | Auto-reset circuit through Q1/R31 |
| RTS | `MODE` circuit through Q13/R35 |
| VBUS | `VBUS-USB` |
| VDD | Local USB bridge supply |
| `/RST` | Reset support |

The schematic labels the DTR network **Auto Reset** and the RTS network **MODE**, indicating that the USB bridge supports reset and boot-mode entry during programming. Confirm whether the CP2102’s 3.3 V regulator is used or whether it is powered only from the board rail.

### Page I - Optional flash storage

**Reference:** U3  
**Part marking:** `XTSD08GLGEAG`  
**Role:** Optional SPI flash storage.

| Flash signal | Board net |
|---|---|
| CS | `SD_CS` |
| CLK | `SD_CLK` |
| DO / MISO | `SD_MISO` |
| DI / MOSI | `SD_MOSI` |
| VDD | `SD-3V3` |
| VSS | GND |

The sheet explicitly labels this block **Flash Storage - Optional**. The SPI lines use MCU GPIOs GPIO5, GPIO18, GPIO16, and GPIO4 respectively.

### Page J - Status indicators

The schematic shows three board-level indicators plus a modem connection indicator:

| Indicator | Colour/label | Function |
|---|---|---|
| MCU power | Red, `PWR` | Device is on and running; driven through R13 2.7 kΩ |
| Ignition | Yellow, `ING` | External ACC/ignition detected; driven through R11 2.7 kΩ |
| Connection | Green, `CON` | Connection status; driven through R6 4.7 kΩ |
| GSM connection | Blue, `GSM` | Modem connection; driven through R16 1 kΩ |

The schematic notes these signals as high-active where shown. Confirm LED polarity and actual current at production voltage.

## 5. Shared buses and signal ownership

### I2C bus

The shared I2C nets are `I2C-SCL` and `I2C-SDA`. Devices shown on the bus are:

- ESP32-S MCU
- MPU-6050 motion sensor
- PCF8563 RTC
- TCA9555 GPIO expander
- LTC4311 active terminator

The bus is also brought to the wire harness. The LTC4311 is labelled **I2C Active Terminator / Auto Capacitance Compensation** and sits on the 3.3 V bus with 10 kΩ pull-up components shown.

### UARTs

- `USB-TX` / `USB-RX`: CP2102 service interface to the ESP32-S UART.
- `GSM-TX` / `GSM-RX`: MCU-to-modem UART.
- `SAT-TX` / `SAT-RX`: satellite/GNSS serial interface to the harness.

### SPI

The optional flash device uses `SD_CS`, `SD_CLK`, `SD_MISO`, and `SD_MOSI`.

## 6. Verification and open items

The following items should be resolved before calling this a production specification:

1. Confirm the exact ESP32-S module ordering code and whether external flash/PSRAM is fitted.
2. Obtain the approved BOM so schematic labels can be matched to manufacturer part numbers and tolerances.
3. Confirm the missing/unused pins on the wire harness connector, especially pins 4 and 10.
4. Confirm CAN termination strategy and whether the board is intended to be a bus endpoint.
5. Confirm the cellular modem module, antenna path, supported bands, and SIM voltage/timing requirements.
6. Confirm the RTC backup supply and the intended crystal/load-capacitor network.
7. Confirm I2C addresses and pull-up values for all devices on the shared bus.
8. Confirm the actual 3.3 V regulator/current budget and USB-C power-sharing behavior.
9. Confirm battery chemistry, charge current, protection circuitry, and connector polarity.
10. Add measured limits: input voltage range, sleep/active current, CAN common-mode range, thermal limits, and ESD/surge ratings.

## 7. Repository/source handling

The original schematic PDF is not copied into this repository by this report. The report stores the component identities and net connections needed for firmware, test, and integration work while avoiding an implicit redistribution decision for the source drawing. If the PDF is later added, place it under a clearly named source-artifact directory and record its ownership/licence and whether it is an authoritative revision.

## 8. Revision history

| Date | Revision | Change |
|---|---|---|
| 2026-09-19 | Draft 0.1 | Initial report derived from supplied five-sheet schematic. |

---

<table align="center"><tr><td align="center"><a href="https://reactiot.com.au"><img src="../../design/handoff_reactiot_website/assets/logo.svg" alt="React IoT" width="96" height="42"><br>reactiot.com.au</a></td><td width="32"></td><td align="center"><a href="https://www.srmiles.com"><img src="https://srmiles.com/assets/srm-logo.svg" alt="srmiles" width="168" height="42"><br>srmiles.com</a></td></tr></table>
