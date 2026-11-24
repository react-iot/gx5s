# Battery baseline test plan

Status: draft, 2026-11-23. Nothing here has been run yet.

Goal: measured battery run time and drain rate for each power mode, from a full
1100 mAh LiPo, at a resolution good enough to publish as the baseline charts for the
open-source firmware.

## 1. Runs

| Run | Mode | Supply | Build settings | Ends when |
|---|---|---|---|---|
| R1 | Heartbeat only, vehicle parked | 12 V on, ignition off | `GX5_HIBERNATE=y`, `GX5_HEARTBEAT_S=3600`, `GX5_BAT_MIN_MV=3500` (default) | 48 h |
| R2 | Hibernate: deep sleep, modem radio off | LiPo only | `GX5_HIBERNATE=y`, `GX5_HEARTBEAT_S=3600` | cell at 3.30 V |
| R3 | Standby: deep sleep, modem registered and asleep | LiPo only | `GX5_HIBERNATE=n`, `GX5_HEARTBEAT_S=3600` | cell at 3.30 V |
| R4 | Awake, heartbeat every 5 min | LiPo only | ignition on, `GX5_TELEMETRY_S=300` | cell at 3.30 V |

Assumptions to confirm (they change the runs):

- **R1 has 12 V present.** It is the only item without "No External Power". It answers
  two things: what the vehicle battery gives up per day, and whether the charge taken
  during each 45 s wake keeps the LiPo level or lets it run down to `GX5_BAT_MIN_MV`.
  The current soak build has `GX5_BAT_MIN_MV=5000` (feed always on), which is not the
  default, so the existing soak data does not answer the second question.
- **R2 wakes hourly**, as the firmware does. The pure sleep floor (no wakes) comes from
  the sleep stretches between wakes, so it needs no separate run.
- **R3 does not use PSM (agreed 2026-11-23).** The BG96 Mini PCIe card restarts about
  6 s after entering PSM and the firmware sends `AT+CPSMS=0` at every start
  (`docs/adr/0001-base-firmware.md`, decision 5). R3 is the firmware's standby mode:
  registered, `AT+QSCLK=1`.
- **R4 needs ignition on with 12 V off.** Checked 2026-11-23: with VCC off the hat's
  ACC output woke the board (`wake=ignition ign=1`) and it stayed awake and connected.

Bench check 2026-11-23 (12 V off, USB attached, cell at 4.11 V, debug build):
asleep in hibernate 5.3 mA; awake and connected about 99 mA. The firmware reported
`vcc_mv` 2136 and `chg` 1 with 12 V off (hat read 0.36 V on the harness); not looked
into. USB leak check: 5.36 mA asleep with USB attached, 5.36 mA unplugged, so USB can
stay attached. Unplugging the cable woke the board for about 45 s (peak 162 mA), so
do not touch it during a run.

Fixed for all runs: bench board `C4:DD:57:E0:9F:AF`, the modified modem card, same SIM,
same antenna position, same LiPo, GNSS on, light sleep on, `GX5_MODEM_AT_LOG=n`
(the published numbers should be the shipping build, not the debug one).

## 2. Rig

What is on the bench now (`can-workbench`):

- Hat: switches VCC and ignition, INA219 0x40 on the harness (volts, mA).
- INA219 0x41 in the LiPo positive lead, 0.1 ohm shunt: 0.1 mA per step, cell volts in
  4 mV steps. `soak.py` reads both once a second.
- USB console through the RFC2217 bridge.

Changes needed before the first run:

1. **Sampling.** Each INA219 reading is already a 68 ms average, but `soak.py` takes
   one per second, so 93 % of the time is not measured and a wake (45 s, peaks over
   150 mA) is badly sampled. Read the battery sensor back to back (about 14 per second)
   and write every reading. About 1.2 million rows a day.
2. **Zero and gain.** Asleep the sensor reads a constant -0.5 mA with 12 V on; that may
   be its zero offset. Before each run record the reading with the board side of the
   LiPo lead open, and subtract it. Check gain once against a known load.
3. **USB stays attached.** D9 is removed on this board, so USB does not feed it and the
   console (boot, sleep and `bat_mv` lines) is logged for every run. One check before
   the first run: VCC off, note the LiPo current asleep, unplug USB for a minute and
   compare. `test_bench.py` notes about 1 V left on the supply with VCC off and USB
   attached, so a small leak through the serial lines is possible.
4. **End of run.** When the cell reads 3.30 V or less for 10 s the harness switches
   VCC and ignition on to recharge and marks the
   run finished. Unknown whether the cell has its own protection circuit, so the cell
   must never be left flat.

Optional, for the fine detail (needs the board moved to the PPK2 rig, so it is a
manual step): power the board at the LiPo terminals from the PPK2 at 4.1, 3.8 and
3.5 V and record at 100 kHz: 10 min of sleep in each mode, one whole wake, 10 min
awake. This gives the sleep floor in microamps and the charge per wake, which the
INA219 can only give to about 0.1 mA.

## 3. Procedure per run

1. Flash the build for the run. Save the commit hash and `sdkconfig` with the data.
2. Charge: 12 V on, ignition on (the charger only runs while the board is awake) until
   the LiPo current stays under 10 mA for 10 minutes. From flat this is estimated at
   8 to 10 h (charge current seen so far: about 150 mA).
3. Rest 30 minutes with the board asleep; record the resting voltage.
4. Record the sensor zero (step 2 of the rig list).
5. Start logging, switch VCC off (R2 to R4).
6. Leave it alone until the end condition. No flashing; the logger holds the console
   open for the whole run (opening the port resets the board).
7. Recharge at once. Copy the data off the Pi.

Order: R4 first (shortest, proves the automation), then R2, R3, R1, then R4 again to
show how repeatable the result is on the same cell.

## 4. Expected length

Estimates only; none of these currents has been measured on LiPo alone. Known parts:
the battery-sense divider takes about 1.9 mA at all times, the modem card asleep took
1.92 mA at 3.3 V on the PPK2, and a wake is about 45 s at roughly 100 mA.

| Run | Guess at average current | Guess at run time |
|---|---|---|
| R2 hibernate | 6 to 7 mA (5.3 mA asleep measured, plus hourly wakes) | 6.5 to 7.5 days |
| R3 standby | 5 to 8 mA | 6 to 9 days |
| R4 awake | about 100 mA (99 mA measured) | about 11 h |
| R1 | n/a | 48 h fixed |

With recharges, about four weeks on one board.

## 5. Results to publish

Per run, from the raw samples:

- Run time to 3.30 V, and charge delivered (mAh).
- Average current overall, asleep, and awake; charge per wake (mAh) and wake length.
- Cell voltage at start, and at 25 / 50 / 75 % of the run.

Charts:

1. Cell voltage against time, all modes on one chart.
2. Cell voltage against charge delivered (the discharge curve; also the table for a
   state-of-charge estimate in firmware).
3. Current against time for one wake in each mode, log scale.
4. Bars per mode: sleep current, charge per wake, average current, run time.
5. Measured run time against the simple model `capacity / (sleep current + charge per wake / interval)`.
6. Firmware `bat_mv` against the sensor voltage over the whole discharge (the battery
   ADC calibration, at no extra cost).
7. R1: harness mAh per day and LiPo voltage over 48 h.

Data and the plotting script go in `fw/test/bench/battery/`; charts in `docs/assets/`.

## 6. Limits

- One board, one modem card, one cell, room temperature. Other cards (unmodified:
  about 12 mA more asleep) and cold will differ.
- Network conditions change the wake length (41 to 66 s seen on the bench), so wake
  charge is reported as a spread, not one number.
- Cell capacity is taken as measured in the run, not as the 1100 mAh on the label.
