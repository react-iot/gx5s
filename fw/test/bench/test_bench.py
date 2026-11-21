"""GX5s bench tests. Run on the can-workbench Pi:  pytest -x -s test_bench.py

The Pi hat switches the device's VCC and ACC (ignition), measures its supply
(INA219, 0.1 ohm shunt) and talks CAN. The device console comes through the ESP32
Workbench RFC2217 bridge. Tests run in file order and share one powered session.

Limits: with USB attached the board is also fed from USB, so the measured sleep
current is the harness share only. The hat cannot lower VCC, so PROTECT is not tested.
"""
import os
import re
import time

import can
import gpiod
import pytest
import serial
from gpiod.line import Direction, Value
from smbus2 import SMBus

PORT = os.environ.get("GX5_PORT", "rfc2217://localhost:4001")
SLEEP_MA = float(os.environ.get("GX5_SLEEP_MA", "1"))  # 0.15 mA measured: asleep the board lives on its LiPo
VCC_TOL_MV = int(os.environ.get("GX5_VCC_TOL_MV", "800"))  # ADC vs INA219 (input diode included)
PIN_VCC, PIN_ACC = 4, 17
SHUNT_OHM = 0.1


class Hat:
    def __init__(self):
        out = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
        self.lines = gpiod.request_lines("/dev/gpiochip0", consumer="gx5-bench", config={(PIN_VCC, PIN_ACC): out})
        self.bus = SMBus(1)
        self.addr = next(a for a in range(0x40, 0x50) if self._probe(a))  # A0/A1 straps

    def _probe(self, a):
        try:
            self.bus.read_byte(a)
            return True
        except OSError:
            return False

    def _reg(self, r):
        v = self.bus.read_word_data(self.addr, r)
        return ((v & 0xFF) << 8) | (v >> 8)  # INA219 is big-endian

    def vcc(self, on):
        self.lines.set_value(PIN_VCC, Value.ACTIVE if on else Value.INACTIVE)

    def acc(self, on):
        self.lines.set_value(PIN_ACC, Value.ACTIVE if on else Value.INACTIVE)

    def volts(self):
        return (self._reg(2) >> 3) * 0.004

    def milliamps(self, samples=50):
        total = 0
        for _ in range(samples):
            raw = self._reg(1)
            total += raw - 0x10000 if raw & 0x8000 else raw  # 10 uV per bit
            time.sleep(0.02)
        return total / samples * 1e-5 / SHUNT_OHM * 1000


class Console:
    def __init__(self):
        self.s = serial.serial_for_url(PORT, baudrate=115200, timeout=0.2)
        self.buf, self.lines = b"", []  # kept between expect() calls: lines arrive in bursts
        self.log = []  # every console line seen

    def reset(self):
        self.s.dtr = False
        self.s.rts = True
        time.sleep(0.2)
        self.s.rts = False
        self.s.reset_input_buffer()
        self.buf, self.lines = b"", []

    def expect(self, pattern, timeout):
        """Return the match for the first console line matching `pattern`."""
        end = time.time() + timeout
        while time.time() < end:
            if not self.lines:
                *self.lines, self.buf = (self.buf + self.s.read(4096)).split(b"\n")
            while self.lines:
                text = self.lines.pop(0).decode("utf-8", "replace").strip()
                print("   |", text)
                self.log.append(text)
                m = re.search(pattern, text)
                if m:
                    return m
        pytest.fail(f"no console line matching {pattern!r} within {timeout}s")


@pytest.fixture(scope="session")
def hat():
    h = Hat()
    yield h
    h.acc(False)
    h.vcc(False)


@pytest.fixture(scope="session")
def con():
    return Console()


BOOT = r"gx5: id=([\w:]+) wake=([\w-]+) ign=(\d) vcc=(\d+) mV"


def test_hat_switches_supply(hat):
    hat.vcc(False)
    time.sleep(0.5)
    assert hat.volts() < 3.0  # ~1 V remains: the USB-powered board feeds back a little
    hat.vcc(True)
    time.sleep(0.5)
    assert 9.0 < hat.volts() < 30.0


def test_boot_reads_supply(hat, con):
    con.reset()
    m = con.expect(BOOT, 10)
    assert m[3] == "0", "ignition should read off"
    assert abs(int(m[4]) - hat.volts() * 1000) < VCC_TOL_MV, "VCC ADC disagrees with INA219"


def test_factory_reset(hat, con):
    # Five fast resets wipe the stored configuration; four do not.
    time.sleep(8)  # past the 5 s window, so earlier resets in this run do not count
    for n in range(1, 5):
        con.reset()
        con.expect(rf"fast reset {n} of 5", 10)
        time.sleep(1)  # let the count reach flash
    con.reset()
    con.expect(r"factory reset: configuration erased", 10)
    time.sleep(8)  # stays up past the window: the count starts again
    con.reset()
    con.expect(r"fast reset 1 of 5", 10)


def test_external_i2c(hat, con):
    # The hat's INA219 (0x40) hangs on the board's I2C bus through the harness pins,
    # so it stands in for an external sensor; 0x20/0x51/0x68 are the board's own.
    con.reset()
    m = con.expect(r"i2c devices:(.*)", 10)
    assert {"0x20", "0x40", "0x51", "0x68"} <= set(m[1].split()), m[1]


def rtc_write(hat, t):
    """Set the board's PCF8563 (0x51) to UTC time t, from the Pi over the shared I2C bus."""
    bcd = lambda v: v // 10 << 4 | v % 10
    g = time.gmtime(t)
    hat.bus.write_i2c_block_data(0x51, 0x02, [bcd(g.tm_sec), bcd(g.tm_min), bcd(g.tm_hour), bcd(g.tm_mday),
                                              (g.tm_wday + 1) % 7, bcd(g.tm_mon), bcd(g.tm_year % 100)])


def boot_with_rtc(hat, con, t):
    """Set the RTC to t with the board held in reset (running, it rewrites the RTC from
    network time every few seconds), then let it boot. Returns the clock it starts with."""
    con.s.dtr = False
    con.s.rts = True
    time.sleep(0.3)
    rtc_write(hat, t)
    con.reset()
    return int(con.expect(r"io: clock from RTC: (\d+)", 10)[1])


def test_rtc_sets_clock(hat, con):
    # At boot the system clock comes from the RTC (network or GPS time corrects it later).
    back = 400 * 86400
    off = boot_with_rtc(hat, con, int(time.time()) - back) - (time.time() - back)
    assert abs(off) <= 3, f"clock is {off:.0f} s off the RTC"


def test_network_time_sets_rtc(hat, con):
    # The modem's network time (UTC) must reach the RTC without any broker. Needs a SIM
    # and coverage. Follows test_rtc_sets_clock: the RTC is still 400 days behind.
    import calendar
    con.expect(r"modem: time ", 200)
    time.sleep(1)
    d = hat.bus.read_i2c_block_data(0x51, 0x02, 7)
    dec = lambda b: (b >> 4) * 10 + (b & 0x0F)
    rtc = calendar.timegm((2000 + dec(d[6]), dec(d[5] & 0x1F), dec(d[3] & 0x3F), dec(d[2] & 0x3F), dec(d[1] & 0x7F), dec(d[0] & 0x7F)))
    assert abs(rtc - time.time()) <= 5, f"RTC is {rtc - time.time():.0f} s off after network time"


def test_ignition_on(hat, con):
    hat.acc(True)
    con.expect(r"ignition on", 5)
    m = con.expect(r'telemetry .*"ign":(\d).*"acc":\[(-?\d+),(-?\d+),(-?\d+)\],"temp":([\d.]+)', 5)
    assert m[1] == "1"
    g = sum(int(m[i]) ** 2 for i in (2, 3, 4)) ** 0.5
    assert 800 < g < 1250, f"IMU at rest should read about 1 g, got {g:.0f} mg"  # uncalibrated: 1160 mg on the bench board
    assert 0 < float(m[5]) < 80, "IMU temperature implausible"
    w = [float(v) for v in re.search(r'"gyro":\[([-\d.,]+)\]', m.string)[1].split(",")]
    assert all(abs(v) < 25 for v in w) and any(w), f"gyro at rest should be near zero but alive, got {w}"
    con.expect(r"gx5: motion calibrated", 5)  # driving-event detection found gravity and gyro bias


def test_external_uart(hat, con):
    # The hat's serial header is wired to the board's harness UART (SAT-TX / SAT-RX).
    with serial.Serial("/dev/serial0", 9600, timeout=3) as u:
        u.reset_input_buffer()
        u.write(b"gx5 ping\r\n")
        con.expect(r"ext rx: gx5 ping", 5)  # Pi -> board
        assert u.readline().strip().startswith(b"gx5 pong"), "no reply from the board"  # board -> Pi


def test_can_receive(hat, con):
    # The device listens only, so nothing acknowledges: the Pi keeps retransmitting
    # the first frame, which is all the device needs to count frames.
    with can.Bus(interface="socketcan", channel="can0") as bus:
        try:
            bus.send(can.Message(arbitration_id=0x7DF, data=bytes.fromhex("0201 0C00 0000 0000"), is_extended_id=False))
        except can.CanError:
            pass
        m = con.expect(r'telemetry .*"can":(\d+)', 75)  # next periodic telemetry
    assert int(m[1]) > 0, "device saw no CAN frames"


def test_ignition_off_then_sleep(hat, con):
    hat.acc(False)
    con.expect(r"ignition off", 5)
    assert "gyro" not in con.expect(r"telemetry .*", 5).string, "gyro should be off with ignition off"
    false_events = [l for l in con.log if re.search(r"gx5: motion (accel|brake|left|right|harsh)", l)]
    assert not false_events, f"driving events reported on a stationary bench: {false_events}"
    con.expect(r"gx5: SLEEP", 260)  # connect timeout (180 s) + linger + modem park
    time.sleep(8)
    ma = hat.milliamps()
    print(f"   sleep current (harness): {ma:.2f} mA at {hat.volts():.2f} V")
    assert ma < SLEEP_MA


def test_wake_on_ignition(hat, con):
    hat.acc(True)
    m = con.expect(BOOT, 10)
    assert m[2] == "ignition" and m[3] == "1"


def test_wake_on_imu_interrupt(hat, con):
    # Checks the path IMU INT -> APS-INT (GPIO12) -> deep-sleep wake. The Pi shares the
    # board's I2C bus, so it makes the IMU raise its interrupt (data-ready) instead of
    # shaking the board; the motion detection itself still needs a real knock.
    hat.acc(False)
    con.expect(r"ignition off", 5)
    con.expect(r"gx5: SLEEP", 260)
    time.sleep(8)
    hat.bus.write_byte_data(0x68, 0x38, 0x01)  # INT_ENABLE: data ready
    m = con.expect(BOOT, 10)
    assert m[2] == "motion"

