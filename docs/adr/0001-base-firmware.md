# ADR 0001 — GX5s base firmware

Status: accepted (2026-11-15)

## Decisions

1. **ESP-IDF 6.x**, plain C, no managed components. Project lives in `fw/`, split into
   ESP-IDF components (decision 11).
2. **Event driven.** One `GX5_EVENT` base on the default `esp_event` loop. ISRs and
   `esp_timer` callbacks only post events; all state lives in one handler
   (`fw/main/main.c:on_event`), so no locks there. The modem owns one task (blocking AT I/O).
3. **MQTT via the modem's own stack** (Quectel BG96 `AT+QMT*`), not PPP + esp-mqtt.
   TLS is done by the modem (`AT+QSSLCFG`); the CA cert lives on the modem filesystem.
4. **Pin map** (`gx5_core/include/gx5.h`):
   modem UART ESP TX 23 / RX 22, `EXT-PWR` GPIO13 **active low**,
   CAN enable GPIO0, SD CS 18 / CLK 5, expander LED-SVR P12, LED-IG P14, LED-PWR
   P15, CHRG-EN P05. The UART pins, `EXT-PWR` pin and polarity, and CAN enable are
   menuconfig options.
   Bench (2026-11-15): with these defaults the BG96 answers AT and registers, so the
   UART direction is right. `EXT-PWR` is **not** a modem rail: it enables the
   36V->5V buck (TPS5430 via Q2; schematic v210620 sheet 2). Low = the board runs from
   the vehicle supply, high = it runs from the LiPo (or USB). `3V3-PSM` (GPIO15) is the
   3V3 buck-boost's power-save pin (TPS63001 PS/SYNC), not a rail switch. The modem,
   IMU, expander and RTC all hang off 3V3, so they stay powered from the LiPo.
   Both pins measured on the bench (decision 12). `docs/gx5s-board-spec.md` still shows SD CS 5 / CLK 18.
5. **Power states**

   | State | MCU | Modem | Entered when | Wake |
   |---|---|---|---|---|
   | RUN | 80 MHz + auto light sleep | MQTT connected | ignition on | — |
   | LINGER | same | MQTT connected | ignition off / timer or RI wake | — |
   | SLEEP | deep sleep | registered, `QSCLK=1`, GNSS off | linger or connect timeout expired | ignition (ext0), RI or motion (ext1), heartbeat timer |
   | HIBERNATE | deep sleep | powered down (`QPOWD`) | as SLEEP, when `GX5_HIBERNATE=y` | ignition (ext0), motion (ext1), heartbeat timer |
   | PROTECT | deep sleep | powered down (`QPOWD`) | supply below cutoff | ignition (only if off), recheck timer |

   In all sleep states the regulator is in power-save mode, the charger is off and
   the harness feed is off, so the board lives on its LiPo and draws ~0.15 mA from
   the vehicle (11.4 mA with feed and normal mode left on). The feed stays on
   (6.6 mA) only when the LiPo is below `GX5_BAT_MIN_MV` or missing: with neither
   supply the MCU would brown out, the buck enable would float back on and the board
   would reboot in a loop. Bench 2026-11-15: a modem URC (RI pulse) woke the board
   from SLEEP (`wake=modem`) and the modem answered at once. But the modem card
   still costs ~16 mA at 12 V (feed on) when asleep and registered, against 6.6 mA
   for the board alone, so on the LiPo it will not last long; eDRX/PSM not tried yet. The battery-sense divider alone (R8 1K + R5 1.2K across the LiPo,
   schematic sheet 2) takes ~1.9 mA from it at all times. Both ADC readings go
   through the chip's factory calibration (eFuse Vref).

6. **Never drain the supply.** `VCC < GX5_VCC_CUTOFF_MV` for 3 consecutive samples
   (or once, on a wake with ignition off) forces PROTECT. Leaving PROTECT needs
   `cutoff + GX5_VCC_HYST_MV`. With ignition off, the awake window is always bounded
   (`GX5_CONNECT_TIMEOUT_S`, `GX5_LINGER_S`). A reading below `GX5_VCC_ABSENT_MV`
   means no harness supply (USB / LiPo bench power) and disables protection.
   The cutoff defaults to 11.0 V, as in GX5sV2 (`vccLow = 11`). It was 11.8 V until
   the soak of 2026-11-17: with the hat measuring 12.1 V the board read 11.5 V while
   the LiPo charged (~340 mA) and went to PROTECT with ignition on, then stayed there
   30 minutes because 12.09 V was under cutoff + hysteresis. Where the ~0.6 V goes is
   not found yet; until it is, the cutoff acts that much early whenever the charger runs.
7. **Light sleep while online** needs the modem to announce URCs on RI
   (`urc/ri/other` pulse + `urc/delay`), since UART bytes are lost in light sleep and
   ESP32 UART wake only works on IO_MUX pins. `GX5_LIGHT_SLEEP=n` turns it off.
   Every modem command holds the no-light-sleep lock for its whole exchange (in `at()`
   itself, not in the callers). Bench 2026-11-17: the announcement sent right after
   connecting ran without the lock; with ignition off (CAN is off, so nothing else holds
   one) the MCU slept between the modem's `OK` and its `+QMTPUB:`, lost that reply and
   every incoming message, declared the link dead and reconnected, which the broker
   logged as "Duplicate Client ID". With ignition on, CAN's lock hid it.
8. **Provisioning.** The broker is not compiled in. Firmware carries only a
   *bootstrap* broker (menuconfig; default `mqtt.reactiot.com.au:1883`, no TLS, so the
   provisioning message and the credentials in it travel unencrypted). Selection each connect: candidate > stored (NVS
   `gx5/broker`) > bootstrap.
   - Device id = factory MAC with the last byte + 3, as `AA:BB:CC:DD:EE:FF`, printed in a startup banner.
   - Bootstrap connects anonymously (no username/password), client id = device id, so
     no credential ships in the firmware. Credentials arrive with the broker message.
   - No stored broker: connect to bootstrap, publish `discovery/<id>`
     (`{"id","fw","reason":"unprovisioned"}`), status `provisioning`.
   - Topics: the device publishes under `device/<id>/<location>/` (`status/connected`,
     `telemetry`, `event`, `can`, `ext`) and takes commands on `device/<id>` and
     `location/<location>` (all devices at a location). The location id is `0000`
     until provisioning sets `config.location` (1-9 of `A-Z a-z 0-9 _ -`, NVS
     `gx5/loc`); a change reconnects so the last will and subscription follow. The
     retained status under the old location is not cleared. There is no site id in
     the topics: provisioning decides which broker a device belongs to.
   - Backend answers on `device/<id>` with one provisioning message:
     `{"provision":{"pop":"<PIN>","broker":{"host","port","user","pass","tls"},"config":{"location":"0001","can":{..}}}}`.
     `pop` is a proof-of-possession PIN in the style of ESP BLE provisioning: the last
     three bytes of the device id as one decimal number, 8 digits
     (`C4:DD:57:E0:9F:AF` -> `14720943`); it is also printed in the startup banner.
     A wrong or missing PIN, or a bare `{"broker":..}`, changes nothing and publishes
     `event` `provision-rejected`. `config` is applied first (today: `location` and
     the `can` object), then the broker is tried at once and stored **only if the connection
     succeeds**; otherwise the device returns to the broker it came from and
     publishes `event` `broker-change-failed`. Messages may be up to 511 bytes.
   - The same message on the production broker re-provisions.
   - **Login on the bootstrap broker itself** (the `mqtt.reactiot.com.au` broker,
     2026-11-16). That broker lets an anonymous device use `discovery/<id>` and nothing
     else, so on the bootstrap broker the firmware sets no last will, subscribes only
     `discovery/<id>`, publishes nothing but the announcement
     (`{"id","hw","fw","reason"}`, repeated on every telemetry tick) and drops the rest.
     The answer comes on the same topic:
     `{"cmd":"config","location":"0001","user":"<id>","pass":"<secret>"}`. Location,
     user and password are all required; a `site` is ignored (no site id in the topics).
     The device keeps host, port and TLS of the bootstrap broker, tries the login at
     once and stores it only if it connects. No PIN: the broker only lets the owner of
     a client id read `discovery/<id>`, and the client id is self-asserted, so anyone
     connecting as a device's id could read its secret. Bench 2026-11-16: the anonymous connect and the
     `discovery/<id>` subscription work (`MQTT connected … (bootstrap)`). Switching the
     will off needs two steps (set it with QoS 0 / no retain, then off): switched off
     alone, the broker dropped the CONNECT unanswered. Bench 2026-11-17 (ignition off):
     the board announced, took the login, disconnected, connected with it
     (`MQTT connected to mqtt.reactiot.com.au`, no `(bootstrap)`) and after a reset
     connected with the stored login directly. Two faults had to go first:
     - A reconnect with a session still open (the login arriving is one) must send
       `AT+QMTDISC` **before** `bringup()`, which resets the data context
       (`AT+QIDEACT`). After the reset the modem answers DISC with ERROR, never finishes
       `AT+QMTCLOSE`, and refuses the next open with `+QMTOPEN: 0,2` (client identifier
       occupied); the login was then dropped as "new broker unreachable".
     - DISC waits 15 s for `+QMTDISC:`, not 5 s. Once the 5 s wait gave up at sleep time
       and the board woke again 2 s later with `wake=modem`. Confirmations measured
       since: 1.4 to 2.3 s.
   - The PIN is derived from the id the device announces, so it stops mistakes
     (provisioning the wrong unit), not an attacker who can read `discovery/#`.
   - `GX5_PROVISION_AFTER_FAILS` consecutive failures to reach the stored broker
     (network up) send the device to bootstrap once (`reason":"broker-unreachable"`),
     then the stored broker gets another round. The stored broker is never erased.
   - Factory reset: five resets in a row (reset button or power cycle), each within
     about 5 s of the previous boot, erase NVS (broker, CAN configuration) and the
     offline queue; the power LED blinks five times and the device starts
     unprovisioned. The count lives in NVS and only real power-on resets add to it
     (not deep-sleep wakes, crashes, watchdogs or brownouts).
   - Trust: authorisation is enforced by the broker, not the firmware. Broker
     permission controls restrict publishing to a device's command topics to the
     permitted controller; other users on the account cannot send commands. The
     firmware therefore acts on any command message it receives (no message signing),
     and TLS (seclevel >= 1) authenticates the broker.
9. **Peripherals** (`gx5_io`, `gx5_store`, GNSS in `gx5_modem`). Each is optional at build time (menuconfig) and at
   runtime: a device that does not answer is logged once and skipped.
   - **TCA9555**: LEDs, charger enable (only from a healthy supply, and never in
     deep sleep: the charger alone drew ~200 mA at 12 V on the bench), `APS-RST`
     released, `BAT-CHRG` read for telemetry. `GSM-SLP` is the modem's DTR (mPCIe
     pin 31): held low while awake, driven high before deep sleep so the modem can
     sleep (`QSCLK=1`). Left floating it is pulled high, the modem stops answering AT
     and every wake had to reset it.
   - **MPU-6050**: acceleration (`acc`, mg) and board temperature (`temp`, degC) in
     every telemetry message, rotation rate (`gyro`, deg/s) while ignition is on (the
     gyro is in standby otherwise); one sample per message. Motion interrupt (accel on, gyro standby, high-pass HOLD, latched
     active-high INT) wakes SLEEP via ext1; confirmed on the bench 2026-11-15 by
     knocking the board twice (`wake=motion` both times at threshold 40, no wake in
     200 quiet seconds between). The IMU stays powered in both sleep
     states (the regulator only changes mode). Latched INT on the GPIO12 strap is a known risk on
     modules without the flash-voltage eFuse (see `GX5_MOTION_WAKE` help).
   - **PCF8563**: sets the system clock at boot if valid; rewritten from network
     time (`AT+CCLK?`, `AT+CTZU=1`) each time the network comes up (no broker needed). Telemetry `ts` is 0 until then.
     GPS time does the same on every fix (host-tested only: no fix on the bench).
     With `CTZU=1` the modem clock is UTC and the zone suffix is not applied (bench:
     `19:21:05+32` at 19:21:04 UTC in a UTC+8 zone). Bench 2026-11-16: the Pi sets
     the RTC 400 days back with the board in reset; it boots with that time
     (`test_rtc_sets_clock`), then network time corrects the RTC with no broker
     (`test_network_time_sets_rtc`).
   - **CAN**: see §10.
   - **SD NAND**: offline telemetry queue (`/sd/queue.txt` + offset file). Powered
     and mounted per operation. GPIO2 supplies the NAND directly, so it is driven at
     maximum strength; at the default the card mounts but writes time out (bench 2026-11-15). Replayed 10 lines per telemetry tick once on the
     production broker; at-most-once after hand-over to the modem queue.
   - **GNSS** (modem, runs alongside LTE): `AT+QGPSLOC=2` on every bring-up and
     after each telemetry publish; the last fix (`lat`/`lon`/`kmh`/`cog` course/`sat` satellites) goes into all
     telemetry, live and offline, so it can be up to one interval old. Stopped
     (`QGPSEND`) before SLEEP. The log shows operator, cell and signal
     (`AT+COPS?`, `AT+QNWINFO`, `AT+QCSQ`) on every bring-up.
10. **CAN** (`gx5_can`, protocol logic in `gx5_proto/canproto.h`). Plain C with
    fixed-size state (no heap).
    - Runs only while ignition is on. The TWAI driver holds a no-light-sleep lock
      while enabled, so RUN does not light-sleep with CAN on.
    - Modes: `off`, `listen` (default; never drives the bus), `obd2`, `j1939`. The
      active modes acknowledge and transmit on the vehicle bus.
    - Configured over MQTT and kept in NVS (`gx5/can`): `{"can":{mode,bitrate,filter}}`
      and `{"can_req":{slot,id,data,interval}}` — 16 request slots, each a raw frame
      sent every `interval` seconds (OBD-II PID request or J1939 PGN request alike).
    - OBD-II: ISO-TP single/first/consecutive frames with flow control, 16-bit PIDs
      for service 0x22, gateway addressing on 0x6xx ids, one request at a time
      (next after completion or 300 ms). Negative responses are dropped.
    - J1939: address claim as 0xF8 (at start and on request), TP.BAM and TP RTS/CTS
      assembly with end-of-message ACK, optional "requested PGNs only" filter.
    - Latest value per (ECU, service, PID) or (PGN, source); only changes are
      published, to `device/<id>/<location>/can`, as text
      (`7E8 01 0C 1AF8`, `FEEE 00 8C7D..`) inside a JSON array. Limits: 40 values,
      96 bytes each (longer responses truncated), 4 concurrent multi-frame messages.
    - Deliberately left out: CAN-based ignition detection (BMW, Toyota, Kia, Ford,
      Nissan, Hyundai, OBD run-time) and "CAN traffic delays sleep". Both need the
      device awake with ignition off, which breaks decision 6. Ignition is the ACC wire.
    - Not done yet: Tesla decoder, ALDL (GM serial), CANopen, raw capture of
      broadcast frames in `obd2` mode.
11. **Component layout** (`fw/components/`). One component per piece of hardware or
    concern, each with its own public header, `Kconfig` menu and declared dependencies.

    | Component | Owns | Depends on |
    |---|---|---|
    | `gx5_core` | board description (pins, expander map, pin Kconfig), the `GX5_EVENT` bus | esp_event |
    | `gx5_proto` | header-only parsing and CAN protocol logic; no ESP-IDF, host-tested | — |
    | `gx5_modem` | BG96 AT engine, MQTT, broker provisioning, GNSS | core, proto |
    | `gx5_io` | TCA9555, MPU-6050, PCF8563 | core, proto |
    | `gx5_store` | SD NAND offline queue | core |
    | `gx5_can` | TWAI driver, request scheduler, CAN config | core, proto |
    | `gx5_ext` | external UART on the harness (SAT-TX / SAT-RX), line based | core |
    | `main` | the application: power states, ignition, supply protection, telemetry | all |

    Rules: hardware components never include each other. They talk to the
    application through `GX5_EVENT` (e.g. `gx5_modem` posts `GX5_EV_TIME`, `main`
    sets the RTC) or through callbacks the application passes in
    (`gx5_can_publish(send)`, `gx5_store_drain(max, send)`). Only `main` wires
    them together, and only `main` initialises shared services (NVS, event loop,
    GPIO ISR service, rails). A feature switched off in menuconfig keeps its API as
    no-op stubs, so `main` has no `#ifdef` per feature. `MINIMAL_BUILD` compiles
    only what these components require.

12. **Bench tests** (`fw/test/bench/`). A Raspberry Pi 4 (`can-workbench`) with the
    "rPI CAN Device Tester" hat switches the device's VCC (GPIO4) and ACC (GPIO17),
    measures its supply (INA219 at 0x40, 0.1 ohm shunt) and talks CAN (MCP25625 on
    SPI0, 16 MHz, INT GPIO25, reset GPIO24). ESP32 Workbench on the Pi bridges the
    USB console over RFC2217 (`:4001`) for flashing and for the tests. `setup-pi.sh`
    sets the Pi up once; `test_bench.py` (pytest) checks supply reading, ignition,
    CAN receive, the bounded awake window, sleep current and wake on ignition.
    Bench results 2026-11-15: a plain divider formula read ~10 % high (13.2-13.6 V at
    12.14 V). The ADC node is a 100K/10K divider loaded by the VCC-detect transistor
    Q4; `gx5_vcc_node_to_mv` now models that circuit from the eFuse-calibrated ADC
    voltage, with Q4's Vbe as the per-board calibration (`GX5_VCC_VBE_MV`, 605 mV
    fitted to one board at one voltage). Not verified: other voltages, other boards,
    and temperature (Vbe drift should move the reading about -40 mV/degC);
    SLEEP draws 0.15 mA from the harness (USB attached, so
    the LiPo's share is not measured).

13. **Driving events** (`gx5_proto/motion.h`, sampled by `gx5_io`). While ignition is
    on the IMU is read at 50 Hz and gyro + accelerometer are fused:
    - Calibration at every ignition on: one quiet second gives gravity ("up", and the
      accelerometer offset) and the gyro bias, so the board can be mounted any way up.
    - Gravity is rotated by the gyro each sample and pulled slowly back to the
      accelerometer, so hills and pitch under braking do not look like acceleration.
    - Turns: yaw rate (gyro about "up") plus horizontal acceleration; left or right
      from the yaw sign.
    - Accelerating and braking need "forward". It is learned from turns (the car
      accelerates toward the inside of a turn, which gives "left"; forward =
      left x up) and kept in RTC memory. Until learned, a straight-line event is
      reported as `harsh`.
    - Events (`accel`, `brake`, `left`, `right`, `harsh`) must last 0.3 s and exceed
      `GX5_HARSH_ACCEL_MG` / `_BRAKE_MG` / `_TURN_MG` (300 / 400 / 400). They are logged
      and published to `device/<id>/<location>/event` as `{"ts","ev","mg"}`; not queued offline.
    - Host-tested with a simulated drive (`fw/test/test_motion.c`). On the bench only
      calibration and "no false events at rest" are checked: nothing has been driven.

14. **External UART** (`gx5_ext`, harness SAT-TX GPIO19 / SAT-RX GPIO21, 9600 baud by
    default). Line based and device-agnostic: every received line is logged and
    published to `device/<id>/<location>/ext`; the command `{"ext":"text"}` sends a line. The
    line `gx5 ping` is answered with `gx5 pong <id>` so wiring can be checked without a
    broker. Bench 2026-11-15: works both ways against the Pi hat's serial header, but
    only with TX and RX swapped (`GX5_EXT_TX_GPIO=21`, `GX5_EXT_RX_GPIO=19`): the hat's
    TX arrives on the pin named SAT-TX. Whether the hat cable is uncrossed or the pin
    names are from the external device's side is not settled; defaults follow the names.
    Bytes arriving during automatic light sleep are lost.
15. **External I2C.** The harness I2C pins are the on-board bus (GPIO27/14), so there
    is no separate interface. At power-on the firmware scans the bus and logs every
    address that answers. The bench hat's INA219 (0x40) reaches the bus through the
    harness wiring and serves as the external test device. No driver for any
    particular external sensor yet. The Pi is a second master on that bus during
    bench runs and nothing prevents collisions. Soak 2026-11-16 (the Pi reading the
    INA219 11 times a second): 3 of 5 wakes had an `I2C hardware timeout` that dropped
    the IMU or the RTC for that wake; cause not proven (one coincided with an I2C error
    on the Pi). Each transfer is now tried three times before a device is dropped.

16. **OTA** (`gx5_modem`). Flash (16 MB): `nvs` 20 K, `otadata`, two app slots of 4 MB and
    a 7.9 MB `storage` data partition, unused today (`fw/partitions.csv`). The table
    cannot be changed by OTA, so the whole flash is laid out now; the image is ~420 K. `{"cmd":"ota","version":"1.4.2"}` (no version = `latest`) makes the modem
    GET `<GX5_OTA_URL>/fw/<hardware id>-<version>.bin?id=<id>&v=<running>` into its
    own file system (`AT+QHTTPGET`, `AT+QHTTPREADFILE`); the image is then pulled
    across in 1 KB pieces (`AT+QFREAD`), written to the other slot, checked
    (`esp_ota_end`) and booted. Pulling instead of streaming keeps the UART from
    overrunning while flash is written. The result goes to `event`
    (`{"ev":"ota","version","result"}`); the device stays awake until it is done.
    Limits: plain http and no signature, so whoever answers for the host can install
    firmware; no rollback if the new image boots but misbehaves. Built and flashed,
    boots from slot 0; **the update itself has never run** (needs a broker connection
    and an image on the server). AT syntax taken from Quectel's HTTP(S) manual (BG96)
    and FILE note (BG95 series).

## Not done (add when needed)

Signed or https OTA and rollback, Tesla / ALDL / CANopen decoders, a protocol for whatever sits on the external UART, SMS parsing.
