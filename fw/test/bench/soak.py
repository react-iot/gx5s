"""GX5s soak logger: supply current and wake/sleep events over days. Runs on the
can-workbench Pi next to test_bench.py (it reuses the hat driver from there).

    sudo systemctl start gx5s-soak      # start (unit installed by setup-pi.sh); stop to run pytest
    python3 soak.py report              # summary of what has been recorded so far

Writes to ~/gx5s-soak (GX5_SOAK_DIR):
    samples.csv   one row per second: time, supply V, harness mA, awake (1/0), ignition,
                  LiPo V, LiPo mA (blank without the battery sensor)
    events.csv    boot (wake cause, reset reason, vcc, bat), sleep, fault, ign-on/ign-off
    console.log   every console line, timestamped

The hat keeps VCC on. Ignition is off except for GX5_SOAK_IGN_MIN minutes (10) every
GX5_SOAK_IGN_EVERY_H hours (6; 0 = never), so both the hourly timer wake and a drive
are exercised. Times are UTC.

Battery current: a second INA219 on the Pi's I2C bus, in series with the LiPo positive
lead (VIN+ to the cell, VIN- to the board), so discharge reads positive and charging
negative. GX5_BAT_INA is its address (0x41: A0 bridged; the hat's own sensor is 0x40),
GX5_BAT_SHUNT_OHM its shunt (0.1; read the value off the breakout) and GX5_BAT_MAH the
cell capacity for the run-time estimate (1000). Without the sensor those columns stay empty.

Limits: with USB attached (needed for the console) a board with D9 fitted is also fed
from USB, so the harness current is only the harness share. An I2C error on the Pi's bus
is logged as a pi-i2c-error event.
"""
import csv
import os
import re
import sys
import threading
import time

DIR = os.path.expanduser(os.environ.get("GX5_SOAK_DIR", "~/gx5s-soak"))
PORT = os.environ.get("GX5_PORT", "rfc2217://localhost:4001")
IGN_EVERY_S = float(os.environ.get("GX5_SOAK_IGN_EVERY_H", "6")) * 3600
IGN_S = float(os.environ.get("GX5_SOAK_IGN_MIN", "10")) * 60
BAT_INA = int(os.environ.get("GX5_BAT_INA", "0x41"), 0)
BAT_SHUNT_OHM = float(os.environ.get("GX5_BAT_SHUNT_OHM", "0.1"))
BAT_MAH = float(os.environ.get("GX5_BAT_MAH", "1000"))
BOOT = re.compile(r"gx5: id=[\w:]+ wake=([\w-]+) ign=(\d) vcc=(\d+) mV bat=(\d+) mV rst=(\d+)")
SLEEP = re.compile(r"gx5: (SLEEP|PROTECT|HIBERNATE): vcc=(\d+) mV bat=(\d+) mV")
FAULT = re.compile(r"Guru Meditation|abort\(\) was called|Backtrace:|[Bb]rownout|task_wdt|assert failed|ESP_ERROR_CHECK failed|stack overflow")
RST = ["unknown", "power-on", "ext", "sw", "panic", "int-wdt", "task-wdt", "wdt", "deep-sleep", "brownout"]  # esp_reset_reason_t
EVENT_COLS = ["time", "epoch", "event", "wake", "ign", "vcc_mv", "bat_mv", "rst", "note"]
awake = ""  # "1" / "0" once the console has told us


def now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def open_csv(name, header):
    path = os.path.join(DIR, name)
    new = not os.path.exists(path)
    f = open(path, "a", newline="", buffering=1)
    w = csv.writer(f)
    if new:
        w.writerow(header)
    return w


def ina_word(bus, addr, reg):
    v = bus.read_word_data(addr, reg)
    return ((v & 0xFF) << 8) | (v >> 8)  # INA219 is big-endian


def bat_read(bus):
    """LiPo volts and mA (+ = discharging) from the battery INA219."""
    raw = ina_word(bus, BAT_INA, 1)  # shunt voltage, 10 uV per bit, signed
    raw = raw - 0x10000 if raw & 0x8000 else raw
    return (ina_word(bus, BAT_INA, 2) >> 3) * 0.004, raw * 1e-5 / BAT_SHUNT_OHM * 1000


def console(events):
    """Follow the device console for ever; turn boot/sleep/fault lines into events."""
    global awake
    import serial

    log = open(os.path.join(DIR, "console.log"), "a", buffering=1)
    while True:
        try:
            s = serial.serial_for_url(PORT, baudrate=115200, timeout=1, do_not_open=True)
            s.dtr = s.rts = False  # opening the port must not reset the board
            s.open()
            buf = b""
            while True:
                *lines, buf = (buf + s.read(4096)).split(b"\n")
                for raw in lines:
                    line = raw.decode("utf-8", "replace").strip()
                    if not line:
                        continue
                    log.write(f"{now()} {line}\n")
                    ev = lambda *a: events.writerow([now(), int(time.time()), *a])
                    if m := BOOT.search(line):
                        awake = "1"
                        rst = int(m[5])
                        ev("boot", m[1], m[2], m[3], m[4], RST[rst] if rst < len(RST) else rst, "")
                    elif m := SLEEP.search(line):
                        awake = "0"
                        ev(m[1].lower(), "", "", m[2], m[3], "", "")
                    elif FAULT.search(line):
                        ev("fault", "", "", "", "", "", line[:200])
        except Exception as e:  # bridge restarted, USB replugged: note it and try again
            log.write(f"{now()} soak: console lost: {e!r}\n")
            time.sleep(5)


def run():
    from test_bench import Hat

    os.makedirs(DIR, exist_ok=True)
    events = open_csv("events.csv", EVENT_COLS)
    samples = open_csv("samples.csv", ["time", "epoch", "volts", "ma", "awake", "ign", "bat_v", "bat_ma"])
    rig = lambda what: events.writerow([now(), int(time.time()), what, "", "", "", "", "", ""])
    hat = Hat()
    hat.acc(False)
    hat.vcc(True)
    rig("soak-start")
    try:  # 128-sample averaging (68 ms per reading): sleep current is a fraction of a mA
        hat.bus.write_word_data(BAT_INA, 0, 0xFF3F)  # config 0x3FFF, big-endian on the wire
        bat = True
    except OSError:
        bat = False
    rig(f"battery sensor at {BAT_INA:#x}, shunt {BAT_SHUNT_OHM} ohm" if bat else f"no battery sensor at {BAT_INA:#x}")
    threading.Thread(target=console, args=(events,), daemon=True).start()

    start, ign, errors = time.time(), False, 0
    while True:
        t = time.time()
        want = IGN_EVERY_S > 0 and (t - start) % IGN_EVERY_S >= IGN_EVERY_S - IGN_S  # last minutes of each period
        if want != ign:
            ign = want
            hat.acc(ign)
            rig("ign-on" if ign else "ign-off")
        try:
            row = [now(), f"{t:.1f}", f"{hat.volts():.2f}", f"{hat.milliamps(10):.2f}", awake, int(ign)]
            if bat:
                v, ma = bat_read(hat.bus)
                row += [f"{v:.3f}", f"{ma:.2f}"]
            samples.writerow(row)
        except OSError as e:  # the board is the other master on this bus
            errors += 1
            if errors in (1, 10, 100, 1000):
                rig(f"pi-i2c-error x{errors}: {e}")
        time.sleep(max(0, 1 - (time.time() - t)))


def report():
    ev = list(csv.DictReader(open(os.path.join(DIR, "events.csv"))))
    if not ev:
        sys.exit("no events yet")
    span = (int(ev[-1]["epoch"]) - int(ev[0]["epoch"])) / 3600
    print(f"{ev[0]['time']} .. {ev[-1]['time']}  ({span:.1f} h of events)")

    boots = [e for e in ev if e["event"] == "boot"]
    by = {}
    for b in boots:
        key = f"{b['wake']} (reset: {b['rst']})"
        by[key] = by.get(key, 0) + 1
    print(f"\nboots: {len(boots)}")
    for k, n in sorted(by.items(), key=lambda kv: -kv[1]):
        print(f"  {n:5d}  {k}")

    up, down, last = [], [], None  # awake and asleep stretches, seconds
    for e in ev:
        if e["event"] in ("boot", "sleep", "protect", "hibernate"):
            if last and last["event"] == "boot" and e["event"] != "boot":
                up.append(int(e["epoch"]) - int(last["epoch"]))
            if last and last["event"] != "boot" and e["event"] == "boot":
                down.append((int(e["epoch"]) - int(last["epoch"]), e["time"], e["wake"]))
            last = e
    stat = lambda v: f"n={len(v)} min={min(v)} avg={sum(v) / len(v):.0f} max={max(v)} s" if v else "none"
    print(f"\nawake stretches: {stat(up)}")
    print(f"asleep stretches: {stat([d[0] for d in down])}")
    late = [d for d in down if d[0] > 3600 * 1.05]
    print(f"asleep longer than the hourly wake allows: {len(late)}")
    for d in late[:20]:
        print(f"  {d[1]}  {d[0]} s, woke by {d[2]}")

    # Battery health: the LiPo carries the board while asleep, so its voltage at each
    # wake (before the charger has done anything) and the drop per sleep show its state.
    wakes = [b for b in boots if b["rst"] == "deep-sleep"]
    if wakes:
        bat = [int(b["bat_mv"]) for b in wakes]
        days = (int(wakes[-1]["epoch"]) - int(wakes[0]["epoch"])) / 86400
        print(f"\nLiPo at wake: first {bat[0]} mV, last {bat[-1]} mV, min {min(bat)}, max {max(bat)}" +
              (f", trend {(bat[-1] - bat[0]) / days:+.0f} mV/day" if days > 0.5 else ""))
        drops, prev = [], None  # (mV lost, hours asleep) for each sleep -> wake pair
        for e in ev:
            if e["event"] in ("sleep", "hibernate"):
                prev = e
            elif e["event"] == "boot" and prev and e["rst"] == "deep-sleep":
                h = (int(e["epoch"]) - int(prev["epoch"])) / 3600
                if h > 0.05:
                    drops.append((int(prev["bat_mv"]) - int(e["bat_mv"])) / h)
                prev = None
        if drops:
            print(f"LiPo drop while asleep: avg {sum(drops) / len(drops):.0f} mV/h, worst {max(drops):.0f} mV/h over {len(drops)} sleeps")
        vcc = [int(b["vcc_mv"]) for b in wakes]
        print(f"supply at wake: min {min(vcc)} mV, max {max(vcc)} mV")
        print("  time                  wake      LiPo mV  supply mV")
        for b in wakes[-12:]:
            print(f"  {b['time']}  {b['wake']:8s}  {b['bat_mv']:>7s}  {b['vcc_mv']:>9s}")

    # Trouble: anything that is not a clean power-on at the start or a deep-sleep wake.
    bad = [b for b in boots[1:] if b["rst"] != "deep-sleep"] + [e for e in ev if e["event"] == "fault" or e["event"].startswith("pi-i2c")]
    print(f"\nunexpected resets, faults, rig errors: {len(bad)}")
    for e in sorted(bad, key=lambda e: e["epoch"])[:40]:
        print(f"  {e['time']}  {e['event']} {e['wake']} {e['rst']} {e['note']}")

    tot = {"1": [0.0, 0], "0": [0.0, 0], "": [0.0, 0]}
    peak = {"1": 0.0, "0": 0.0, "": 0.0}
    btot = {"1": [0.0, 0], "0": [0.0, 0], "": [0.0, 0]}  # LiPo mA, + = discharging
    bpeak = {"1": 0.0, "0": 0.0, "": 0.0}
    for r in csv.DictReader(open(os.path.join(DIR, "samples.csv"))):
        ma = float(r["ma"])
        tot[r["awake"]][0] += ma
        tot[r["awake"]][1] += 1
        peak[r["awake"]] = max(peak[r["awake"]], ma)
        if r.get("bat_ma"):
            b = float(r["bat_ma"])
            btot[r["awake"]][0] += b
            btot[r["awake"]][1] += 1
            bpeak[r["awake"]] = max(bpeak[r["awake"]], b)
    n = sum(v[1] for v in tot.values())
    print(f"\nharness current over {n / 3600:.1f} h of samples (1 per second):")
    for key, name in (("1", "awake"), ("0", "asleep"), ("", "unknown")):
        s, c = tot[key]
        if c:
            print(f"  {name:8s} {c / n * 100:5.1f} % of the time, avg {s / c:7.2f} mA, peak {peak[key]:7.1f} mA")
    mah = sum(v[0] for v in tot.values()) / 3600
    print(f"  total {mah:.1f} mAh, average {mah / (n / 3600):.2f} mA" if n else "  no samples")

    bn = sum(v[1] for v in btot.values())
    if not bn:
        print("\nLiPo current: no battery sensor samples")
        return
    print(f"\nLiPo current over {bn / 3600:.1f} h of samples (+ = discharging, - = charging):")
    for key, name in (("1", "awake"), ("0", "asleep"), ("", "unknown")):
        s, c = btot[key]
        if c:
            print(f"  {name:8s} avg {s / c:8.2f} mA, peak discharge {bpeak[key]:7.1f} mA")
    net = sum(v[0] for v in btot.values()) / 3600
    print(f"  net {net:+.1f} mAh taken from the LiPo, average {net / (bn / 3600):+.2f} mA")
    s, c = btot["0"]
    if c and s / c > 0:
        h = BAT_MAH / (s / c)
        print(f"  asleep on the LiPo alone: a {BAT_MAH:.0f} mAh cell lasts about {h:.0f} h ({h / 24:.1f} days), wakes not counted")


if __name__ == "__main__":
    report() if sys.argv[1:] == ["report"] else run()
