# GX5s base firmware

Open-source base firmware for the board: ESP-IDF 6.x, event driven, MQTT through the
cellular modem (Quectel BG96 `AT+QMT*`), built to never drain the vehicle battery.
The production broker is provisioned over the air; only a bootstrap broker is compiled in.
Design and power-state table: [`docs/adr/0001-base-firmware.md`](../docs/adr/0001-base-firmware.md).

## Build and test

```sh
. ~/esp/v6.1.1/esp-idf/export.sh    # all commands below run from fw/
idf.py menuconfig          # "GX5s application" and "Component config > GX5s …": broker, APN, cutoff, pins
idf.py build flash monitor
cc -Icomponents/gx5_proto/include test/test_parse.c -o test/test_parse && test/test_parse   # host checks
cc -Icomponents/gx5_proto/include test/test_can.c -o test/test_can && test/test_can
cc -Icomponents/gx5_proto/include test/test_motion.c -lm -o test/test_motion && test/test_motion
```

## Components

| Component | What it is |
|---|---|
| `main` | The application: power states, ignition, supply protection, telemetry |
| `gx5_core` | Board pins and the event bus every component posts to |
| `gx5_proto` | Header-only parsers and CAN protocol logic (host-tested, no ESP-IDF) |
| `gx5_modem` | BG96 AT engine, MQTT, broker provisioning, GNSS |
| `gx5_io` | TCA9555 expander, MPU-6050 motion wake, PCF8563 RTC |
| `gx5_store` | Offline telemetry queue on the SD NAND |
| `gx5_can` | CAN driver, request scheduler, OBD-II / J1939 |
| `gx5_ext` | External UART on the harness: lines in, lines out, `gx5 ping` wiring check |

Components do not call each other; only `main` wires them together.

## MQTT topics

| Topic | Direction | Payload |
|---|---|---|
| `discovery/<mac>` (`<mac>` = device id `AA:BB:CC:DD:EE:FF`, factory MAC + 3) | device → bootstrap broker | `{"id":"<mac>","hw":"<product>","fw":"<ver>","reason":"unprovisioned"}` (or `broker-unreachable`) |
| `device/<mac>/<loc>/status/connected` | device → broker, retained | `online`, `provisioning`, `sleep`, `hibernate`, `protect`, `offline` (last will) |
| `device/<mac>/<loc>/event` | device → broker | `{"id","fw","reason":"broker-changed"}` or `broker-change-failed`; driving events `{"ts":…,"ev":"brake","mg":520}` (`accel`, `brake`, `left`, `right`, `harsh`) |
| `device/<mac>/<loc>/telemetry` | device → broker | `{"ts":1794745845,"ign":1,"vcc_mv":12850,"bat_mv":4100,"ext":1,"chg":0,"can":1532,"wake":"timer","up_s":42,"acc":[12,-30,1004],"temp":31.5,"gyro":[0.4,-1.2,0.1],"lat":-33.86785,"lon":151.20732,"kmh":48.6,"cog":113.48,"sat":9}` |
| `device/<mac>/<loc>/can` | device → broker | changed CAN values: `{"ts":…,"obd":["7E8 01 0C 1AF8"]}` or `{"ts":…,"j1939":["FEEE 00 8C7DFFFFFFFFFFFF"]}` |
| `device/<mac>/<loc>/ext` | device → broker | each line received on the external UART (harness SAT pins) |
| `device/<mac>` and `location/<loc>` | broker → device, QoS 1 | `{"provision":{"pop":"14720943","broker":{"host":"…","port":8883,"user":"…","pass":"…","tls":true},"config":{"location":"0001","can":{"mode":"obd2","bitrate":500000}}}}` provisions; anything else is answered with telemetry |

`bat_mv` is the LiPo at rest: it is read only while the charger is off, so during charging
(`chg` 1) it holds the last value from before.

`<loc>` is the location id: `0000` until provisioning sets `config.location` (1-9 of
`A-Z a-z 0-9 _ -`). A command sent to `location/<loc>` reaches every device at that location.

## Provisioning

Provisioning: a new device connects to the bootstrap broker anonymously (no username or
password, client id = device id) and announces itself on
`discovery/<mac>`. Publish the provisioning message to `device/<mac>`. `pop` is the device's
PIN: the last three bytes of its id as one decimal number, 8 digits (`C4:DD:57:E0:9F:AF` →
`14720943`), also shown in the startup banner. With the right PIN the device applies `config`,
tries the new broker immediately and stores it only if it connects; with a wrong one it changes
nothing and publishes `provision-rejected`. The same message on the production broker moves the
device again. After `GX5_PROVISION_AFTER_FAILS` failed connects it checks
back with the bootstrap broker. `idf.py erase-flash` returns a device to unprovisioned.

On the React IoT broker the login comes on the announcement topic instead: the device
subscribes `discovery/<mac>` and takes
`{"cmd":"config","location":"0001","user":"<mac>","pass":"<secret>"}`, then reconnects to the
same broker with that username and password. Until then it publishes nothing but the
announcement.

## Firmware update and clock

Firmware update: `{"cmd":"ota","version":"1.4.2"}` (no version = `latest`) on a command topic.
The modem fetches `<GX5_OTA_URL>/fw/<hardware id>-<version>.bin` (default
`http://ota.reactiot.com.au`), the device writes it to its second app slot, restarts and
reports `{"ev":"ota","version":"…","result":"ok"}` (or the reason it failed) on `event`.

The clock comes from the RTC at boot and is corrected from network time whenever the cell
is up and from GPS time on every fix; all times are UTC.

## Supply calibration

The supply reading depends on one transistor per board and can be a volt out until it
is calibrated. `{"vcc_cal":{"mv":12340}}` tells the device what the supply really is at
that moment (a meter at the plug, or the vehicle's own voltage reading);
`{"vcc_cal":{"vbe_mv":563}}` sets the underlying value directly. Kept across reboots.
Do it before relying on the low-supply protection (`GX5_VCC_CUTOFF_MV`, 11.0 V): an
uncalibrated board in a 2021 Cerato read 11.1 V on a resting battery the car itself
put at 12.0 V.

## Debug mode

`{"debug":true}` (MQTT or USB; kept across reboots, `false` turns it off) adds `dbg`
lines to the console: every modem command and reply, CAN frames sent, one received
frame per CAN id every 5 s, completed answers, what is published on `can`, the
ignition rule's decisions, supply and battery every 5 s, and which wake sources are
armed before each sleep. Use it to work out what a vehicle's bus carries; leave it off
otherwise, the console traffic costs power.

## CAN

CAN is set up as a **vehicle profile**, sent over the command topic (or the USB
console, one JSON object per line) and kept across reboots. A profile has four parts,
one message each:

```json
{"vehicle":{"name":"kia-cerato-2021","bitrate":500000,"pre_s":60,"ign":{"src":"sig","key":"rpm","min":1,"hold_s":10}}}
{"pid":{"n":0,"id":"7E0","data":"02010C"}}
{"pid":{"n":1,"id":"7C6","data":"0322B002"}}
{"sig":{"n":0,"key":"rpm","pid":0,"bix":0,"len":16,"div":4}}
{"sig":{"n":1,"key":"fuel_l","pid":1,"bix":32,"len":8,"div":2}}
{"can":{"state":"pre","mode":"obd2","poll":[[0,2]]}}
{"can":{"state":"on","mode":"obd2","poll":[[0,1],[1,60]]}}
{"can":{"state":"off","mode":"off"}}
```

- **`pid`** — the library: up to 32 requests the vehicle answers, as raw frames (an
  OBD-II / UDS single-frame request, or a J1939 PGN request). No `id` clears entry `n`.
  Requests with an 11-bit id are sent 8 bytes long, zero padded; vehicles ignore
  shorter ones.
- **`sig`** — up to 48 named numbers decoded on the device from the answer to library
  entry `pid`. The fields are those of the [OBDb](https://github.com/OBDb) signal sets:
  `len` bits at bit `bix` of the data after the PID, times `mul`, over `div`, plus `add`;
  `sign` and `blsb` (little-endian bytes) as there. Decoded values are published as
  `{"ts":..,"sig":{"rpm":1726}}`; answers without a signal stay raw text.
- **`can`** — one of three states: `pre` (awake, ignition not seen yet), `on` (ignition
  on), `off` (awake after ignition off). Each has a mode — `off`, `listen` (never
  transmits), `obd2`, `j1939` (both acknowledge and transmit) — and a list of
  `[library entry, seconds]` pairs (0.1 s steps). As shipped: `on` listens, `pre` and
  `off` keep the bus off.
- **`vehicle`** — name, bitrate, and where ignition comes from:
  `"acc"` the ACC wire (default); `"vcc"` supply voltage (`on_mv`, `off_mv`, `hold_s`);
  `"sig"` a decoded signal answered with at least `min` (no `min`: any answer);
  `"frame"` a broadcast frame (`id`, byte `at`, `mask`, `eq`). `sig` and `frame` turn
  off when the condition has not been seen for `hold_s`.

The profile can also name the request that returns the VIN (`"vin_pid":2` in
`vehicle`). After a power loss the device may be in another vehicle, so at the first
ignition on it asks again, stores the answer and publishes
`{"ev":"vin","vin":"..","was":"..","changed":true}` on `event`. It needs the `on`
state to be in a mode that transmits.

**Ignition and trip events** go out on `event`, queued until the broker is reachable:

```json
{"ts":1795509648,"ev":"ign-on"}
{"ts":1795509653,"ev":"trip-start","dur_s":6,"odo_km":78882,"fuel_l":10.5,"run_s":593}
{"ts":1795512000,"ev":"ign-off"}
{"ts":1795512000,"ev":"trip-end","dur_s":2352,"odo_km":78901,"fuel_l":9.0,"run_s":2940}
```

`trip-start` follows `ign-on` by 6 s, once the vehicle has answered. The values are
the signals named in `"trip":["odo_km","fuel_l","run_s"]` of `vehicle` (up to four),
each at its last good reading; `"null":127.5` on a signal marks the reading that means
"no value" (the Cerato's fuel level with ignition off) so it is dropped. `dur_s` is
the time since ignition on.

`{"profile":"clear"}` puts the whole profile back as shipped.

Ready-made profiles are in `fw/profiles/`, one message per line, to send in order.
`kia-cerato-2021.jsonl` (run time as ignition, speed, fuel, odometer, doors, locks,
hazard switch, brake light, handbrake, VIN) is built from the
[OBDb Kia Cerato](https://github.com/OBDb/Kia-Cerato) signal set and decodes OBDb's
2021 test answers correctly and was run in a 2021 Cerato on 2026-11-24: every
request is answered and the odometer matched the dash. The file restores ignition
detection in its first lines; while a profile is being loaded with ignition off the
device may still go to sleep (30 s after connecting), so a loader should check each
line is acknowledged with `can: profile` on the console and resend after a wake.

With any source but the ACC wire the device also **wakes on CAN traffic** (as well as
motion and the heartbeat) and waits `pre_s` for ignition. Many OBD-II ports are gated
and carry no traffic until something asks; for those set `"wake":"vcc"` in `vehicle`:
the device then wakes when the supply rises `GX5_VCC_WAKE_MV` (0.8 V) above the lowest
level seen during the sleep, which is what an engine start does (the dip while
cranking and the recovery after it are enough). The chip cannot watch CAN traffic and
the supply in the same sleep, so it is one or the other. A bus that never goes quiet
is not armed as a wake source, and after three CAN wakes in a row without ignition it
is left alone until something else wakes the device. A `sig` rule only works if the
`pre` state polls that request, which means transmitting before ignition is known.

## Factory reset

**Factory reset:** press reset (or cycle power) five times, each within about 5 seconds of
the last. The stored broker, CAN configuration and offline queue are erased, the power LED
blinks five times and the device announces itself for provisioning again.

## Behaviour

- **Ignition on** — connected, telemetry every `GX5_TELEMETRY_S` with a GNSS fix; CAN requests sent and changed values published.
- **Ignition off** — connect, report, stay `GX5_LINGER_S`, then deep sleep with the modem still
  registered. Wakes on ignition, motion (MPU-6050), modem RI (e.g. an SMS) or every `GX5_HEARTBEAT_S`; commands
  sent meanwhile are queued by the broker (persistent session).
- **Hibernate** (`GX5_HIBERNATE=y`, off by default) — as ignition off, but the modem radio is
  off while asleep: nothing can reach the device until ignition, motion or the heartbeat timer
  wakes it, and each wake registers again.
- **Offline** — telemetry is queued on the SD NAND with an RTC timestamp and replayed when the broker is back.
- **Supply below `GX5_VCC_CUTOFF_MV`** — modem radio off, deep sleep, supply re-measured every
  `GX5_PROTECT_RECHECK_S`; resumes above cutoff + `GX5_VCC_HYST_MV`.

Sleep current depends on the modem card: an unmodified BG96 Mini PCIe card keeps the board at
about 20 mA (12 V) asleep, a card with its USB-detect link removed at about 8.5 mA
(`docs/adr/0001-base-firmware.md`, decision 5). `GX5_MODEM_AT_LOG=y` prints every modem command
and reply for debugging.

TLS is terminated in the modem: upload the broker CA once (`AT+QFUPL="cacert.pem",<size>`), or
set `GX5_MQTT_TLS_SECLEVEL=0` for an unauthenticated (insecure) link while testing.

## Bench tests (Raspberry Pi + CAN tester hat)

[`test/bench/`](test/bench/) runs the firmware against real hardware: the Pi hat switches the board's
supply and ignition, measures its current and sends CAN frames; the console comes through
[ESP32 Workbench](https://github.com/react-iot/esp32-workbench) on the Pi.

Long runs: `soak.py` on the same Pi logs the supply current once a second and every boot
(wake cause, reset reason), sleep and fault from the console to `~/gx5s-soak`, with a
10-minute ignition cycle every 6 hours. `sudo systemctl start gx5s-soak` starts it,
`python3 soak.py report` summarises; stop it before running pytest (both drive the hat).
With a second INA219 at `0x41` on the Pi's I2C bus, in series with the LiPo positive lead,
it also logs LiPo voltage and current and reports the drain while asleep.

ECU simulator: `ecu_sim.py` on the Pi plays one engine ECU on `can0` so the device has
something to ask. It answers OBD-II on `7DF`/`7E0` from `7E8` (service 01 PIDs, VIN by
`09 02` and `22 F190`, multi-frame included) and J1939 as address `00` (PGN requests,
address claim, periodic EEC1 / CCVS / ET1 / VEP1 / DD1 / position `FEF3`, time and date
`FEE6` and VIN over TP on request). `python3 ecu_sim.py`
serves until Ctrl-C (`-v` prints every frame, `--quiet` stops the broadcasts);
`python3 ecu_sim.py check` tests its logic anywhere, without CAN. The device needs ignition
on and an active mode at the bus's 500 kbit/s:

```json
{"pid":{"n":0,"id":"7DF","data":"02010C"}}
{"can":{"state":"on","mode":"obd2","poll":[[0,10]]}}
{"pid":{"n":1,"id":"18EA00F8","data":"E5FE00"}}
{"pid":{"n":2,"id":"18EA00F8","data":"E6FE00"}}
{"can":{"state":"on","mode":"j1939","poll":[[1,30],[2,60]]}}
```

```sh
scp test/bench/* can-workbench:gx5s-bench/
ssh can-workbench 'sudo bash gx5s-bench/setup-pi.sh && sudo reboot'    # once
idf.py -p 'rfc2217://<pi-address>:4001' -b 115200 flash                # flashes over the network
ssh can-workbench 'cd gx5s-bench && python3 -m pytest -s test_bench.py'   # about 5 minutes
```
