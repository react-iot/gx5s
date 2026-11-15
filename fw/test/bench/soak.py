"""GX5s soak logger: supply current and wake/sleep events over days. Runs on the
can-workbench Pi next to test_bench.py (it reuses the hat driver from there).

    sudo systemctl start gx5s-soak      # start (unit installed by setup-pi.sh); stop to run pytest
    python3 soak.py report              # summary of what has been recorded so far

Writes to ~/gx5s-soak (GX5_SOAK_DIR):
    samples.csv   one row per second: time, supply V, harness mA, awake (1/0), ignition
    events.csv    boot (wake cause, reset reason, vcc, bat), sleep, fault, ign-on/ign-off
    console.log   every console line, timestamped

The hat keeps VCC on. Ignition is off except for GX5_SOAK_IGN_MIN minutes (10) every
GX5_SOAK_IGN_EVERY_H hours (6; 0 = never), so both the hourly timer wake and a drive
are exercised. Times are UTC.

Limits: with USB attached (needed for the console) the board is also fed from USB, so
the current is the harness share only. The Pi shares the board's I2C bus and reads the
INA219 ten times a second; a bus clash would show as an I2C error in console.log.
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
BOOT = re.compile(r"gx5: id=[\w:]+ wake=([\w-]+) ign=(\d) vcc=(\d+) mV bat=(\d+) mV rst=(\d+)")
SLEEP = re.compile(r"gx5: (SLEEP|PROTECT): vcc=(\d+) mV bat=(\d+) mV")
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
    samples = open_csv("samples.csv", ["time", "epoch", "volts", "ma", "awake", "ign"])
    rig = lambda what: events.writerow([now(), int(time.time()), what, "", "", "", "", "", ""])
    hat = Hat()
    hat.acc(False)
    hat.vcc(True)
    rig("soak-start")
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
            samples.writerow([now(), f"{t:.1f}", f"{hat.volts():.2f}", f"{hat.milliamps(10):.2f}", awake, int(ign)])
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
        if e["event"] in ("boot", "sleep", "protect"):
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

    # Trouble: anything that is not a clean power-on at the start or a deep-sleep wake.
    bad = [b for b in boots[1:] if b["rst"] != "deep-sleep"] + [e for e in ev if e["event"] == "fault" or e["event"].startswith("pi-i2c")]
    print(f"\nunexpected resets, faults, rig errors: {len(bad)}")
    for e in sorted(bad, key=lambda e: e["epoch"])[:40]:
        print(f"  {e['time']}  {e['event']} {e['wake']} {e['rst']} {e['note']}")

    tot = {"1": [0.0, 0], "0": [0.0, 0], "": [0.0, 0]}
    peak = {"1": 0.0, "0": 0.0, "": 0.0}
    for r in csv.DictReader(open(os.path.join(DIR, "samples.csv"))):
        ma = float(r["ma"])
        tot[r["awake"]][0] += ma
        tot[r["awake"]][1] += 1
        peak[r["awake"]] = max(peak[r["awake"]], ma)
    n = sum(v[1] for v in tot.values())
    print(f"\nharness current over {n / 3600:.1f} h of samples (1 per second):")
    for key, name in (("1", "awake"), ("0", "asleep"), ("", "unknown")):
        s, c = tot[key]
        if c:
            print(f"  {name:8s} {c / n * 100:5.1f} % of the time, avg {s / c:7.2f} mA, peak {peak[key]:7.1f} mA")
    mah = sum(v[0] for v in tot.values()) / 3600
    print(f"  total {mah:.1f} mAh, average {mah / (n / 3600):.2f} mA" if n else "  no samples")


if __name__ == "__main__":
    report() if sys.argv[1:] == ["report"] else run()
