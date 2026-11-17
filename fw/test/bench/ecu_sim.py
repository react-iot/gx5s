#!/usr/bin/env python3
"""ECU simulator for the can-workbench Pi: answers the GX5s' OBD-II and J1939 requests.

    python3 ecu_sim.py            # serve on can0 until Ctrl-C (-v shows every frame sent)
    python3 ecu_sim.py check      # self-check of the protocol logic, no CAN needed

One simulated engine ECU, both protocols at once (11-bit ids are OBD-II, 29-bit J1939):
  OBD-II  requests on 7DF / 7E0, answers from 7E8. Service 01 (PIDs below, several per
          request), 09 02 (VIN) and 22 F190 (VIN); long answers go out as ISO-TP
          multi-frame after the tester's flow control.
  J1939   address 00. Answers PGN requests (EA00), claims its address, broadcasts the
          PGNs with a period below (--quiet turns that off). Answers over 8 bytes use
          TP: RTS/CTS to the requester, BAM when the request was global.
Values follow a slow simulated drive, so they keep changing.

The device must be in an active mode ("obd2" or "j1939"): in "listen" it never
acknowledges, so nothing the simulator sends gets through. can0 runs at 500 kbit/s.
"""
import argparse
import math
import struct
import sys
import time

VIN = b"1GX5SBENCHTEST001"
OBD_FUNC, OBD_PHYS, OBD_TX = 0x7DF, 0x7E0, 0x7E8
ADDR = 0x00  # J1939 source address: engine #1


def drive(t):
    """Simulated vehicle state, t seconds after start."""
    s = math.sin(t / 10)
    return dict(rpm=1800 + 1000 * s, kmh=60 + 40 * math.sin(t / 15), load=50 + 30 * s, throttle=30 + 20 * s,
                coolant=min(90, 20 + t / 4), oil=min(95, 20 + t / 5), intake=30, ambient=22,
                fuel=max(5, 80 - t / 600), volts=13.8 + 0.2 * math.sin(t / 7), run_s=t,
                km=123456.7 + t / 60, hours=4321 + t / 3600, litres=54321 + t / 1000)


def u8(v):
    return bytes([max(0, min(255, round(v)))])


def be16(v):
    return struct.pack(">H", max(0, min(65535, round(v))))


def le16(v):
    return struct.pack("<H", max(0, min(65535, round(v))))


# Service 01 PIDs, encoded as in SAE J1979.
OBD = {
    0x04: lambda d: u8(d["load"] * 2.55),         # engine load, %
    0x05: lambda d: u8(d["coolant"] + 40),        # coolant, degC
    0x0C: lambda d: be16(d["rpm"] * 4),           # engine speed, rpm
    0x0D: lambda d: u8(d["kmh"]),                 # vehicle speed, km/h
    0x0F: lambda d: u8(d["intake"] + 40),         # intake air, degC
    0x11: lambda d: u8(d["throttle"] * 2.55),     # throttle, %
    0x1F: lambda d: be16(d["run_s"]),             # run time, s
    0x2F: lambda d: u8(d["fuel"] * 2.55),         # fuel level, %
    0x42: lambda d: be16(d["volts"] * 1000),      # control module voltage, mV
    0x46: lambda d: u8(d["ambient"] + 40),        # ambient air, degC
    0x5C: lambda d: u8(d["oil"] + 40),            # oil, degC
    0xA6: lambda d: struct.pack(">I", round(d["km"] * 10)),  # odometer, 0.1 km
}

# PGN: (priority, broadcast period in s or 0 = on request only, data), as in SAE J1939-71.
J1939 = {
    0xF004: (3, 0.1, lambda d: b"\xFF\xFF" + u8(d["load"] + 125) + le16(d["rpm"] * 8) + b"\xFF" * 3),  # EEC1
    0xFEF1: (6, 0.1, lambda d: b"\xFF" + le16(d["kmh"] * 256) + b"\xFF" * 5),                           # CCVS
    0xFEEE: (6, 1, lambda d: u8(d["coolant"] + 40) + b"\xFF" + le16((d["oil"] + 273) * 32) + b"\xFF" * 4),  # ET1
    0xFEF7: (6, 1, lambda d: b"\xFF" * 4 + le16(d["volts"] * 20) * 2),                                  # VEP1
    0xFEFC: (6, 1, lambda d: b"\xFF" + u8(d["fuel"] * 2.5) + b"\xFF" * 6),                              # DD1
    0xFEE5: (6, 0, lambda d: struct.pack("<I", round(d["hours"] * 20)) + b"\xFF" * 4),                  # engine hours
    0xFEC1: (6, 0, lambda d: struct.pack("<I", round(d["km"] * 200)) + b"\xFF" * 4),                    # distance, 5 m
    0xFEE9: (6, 0, lambda d: b"\xFF" * 4 + struct.pack("<I", round(d["litres"] * 2))),                  # fuel used
    0xFEEC: (6, 0, lambda d: VIN + b"*"),                                                               # VIN (TP)
}


def supported(base):
    """Service 01 PID base (00, 20, ..): which of the next 32 PIDs exist."""
    bits = sum(1 << (base + 0x20 - p) for p in OBD if base < p <= base + 0x20)
    return struct.pack(">I", bits | any(p > base + 0x20 for p in OBD))


class Ecu:
    """Frames in (rx), frames out (tick). No CAN in here, so `check` can drive it."""

    def __init__(self, now, broadcast=True):
        self.t0, self.broadcast = now, broadcast
        self.out = []   # (due, id, ext, data)
        self.cf = []    # ISO-TP consecutive frames waiting for flow control
        self.tp = {}    # J1939 RTS/CTS: requester address -> packets
        self.next = {}  # PGN -> next broadcast time
        self.claim(now)

    def send(self, at, can_id, data, ext=False):
        self.out.append((at, can_id, ext, bytes(data)))

    def claim(self, now):
        self.send(now, 0x18EEFF00 | ADDR, bytes.fromhex("0100000000820080"), True)  # arbitrary NAME

    def tick(self, now):
        """Frames that are due, in order."""
        for pgn, (_, period, _) in J1939.items():
            if self.broadcast and period and now >= self.next.get(pgn, 0):
                self.next[pgn] = now + period
                self.pgn(pgn, now)
        self.out.sort(key=lambda f: f[0])  # stable: frames due together keep their order
        due = [f[1:] for f in self.out if f[0] <= now]
        self.out = [f for f in self.out if f[0] > now]
        return due

    def rx(self, can_id, ext, data, now):
        """Feed one received frame. True if it was a request for this ECU."""
        if ext:
            return self.j1939(can_id, data, now)
        return can_id in (OBD_FUNC, OBD_PHYS) and len(data) >= 2 and self.obd(can_id, data, now)

    # ── OBD-II over ISO-TP ──────────────────────────────────────────────────

    def obd(self, can_id, data, now):
        if data[0] >> 4 == 3:  # flow control: send the rest
            # ponytail: block size ignored (the GX5s asks for everything); one transfer at a time
            gap = data[2] / 1000 if len(data) > 2 and data[2] <= 127 else 0
            for i, f in enumerate(self.cf):
                self.send(now + i * gap, OBD_TX, f)
            self.cf = []
            return True
        n = data[0]
        if not 1 <= n <= 7 or n > len(data) - 1:
            return False  # only single-frame requests
        req = bytes(data[1:1 + n])
        rsp = self.service(req, drive(now - self.t0))
        if rsp is None:  # a functional request nobody supports gets no answer
            if can_id == OBD_PHYS:
                self.send(now, OBD_TX, bytes([3, 0x7F, req[0], 0x31]).ljust(8, b"\0"))
        elif len(rsp) <= 7:
            self.send(now, OBD_TX, (bytes([len(rsp)]) + rsp).ljust(8, b"\0"))
        else:
            self.send(now, OBD_TX, bytes([0x10 | len(rsp) >> 8, len(rsp) & 0xFF]) + rsp[:6])
            rest = rsp[6:]
            self.cf = [bytes([0x20 | (i + 1) & 0x0F]) + rest[i * 7:i * 7 + 7].ljust(7, b"\0") for i in range(-(-len(rest) // 7))]
        return True

    def service(self, req, d):
        if req[0] == 0x01:
            out = b""
            for pid in req[1:]:
                if pid % 0x20 == 0 and pid <= max(OBD):
                    out += bytes([pid]) + supported(pid)
                elif pid in OBD:
                    out += bytes([pid]) + OBD[pid](d)
            return b"\x41" + out if out else None
        if req == b"\x09\x02":
            return b"\x49\x02\x01" + VIN
        if req == b"\x22\xF1\x90":
            return b"\x62\xF1\x90" + VIN
        return None

    # ── J1939 ───────────────────────────────────────────────────────────────

    def j1939(self, can_id, data, now):
        pf, da, sa = can_id >> 16 & 0xFF, can_id >> 8 & 0xFF, can_id & 0xFF
        if pf >= 240 or da not in (ADDR, 0xFF):
            return False
        if pf == 0xEA and len(data) >= 3:  # request
            pgn = int.from_bytes(data[:3], "little")
            if pgn == 0xEE00:
                self.claim(now)
            elif pgn in J1939:
                self.pgn(pgn, now, da if da == 0xFF else sa)
            elif da == ADDR:  # not ours to give: negative acknowledgement
                self.send(now, 0x18E8FF00 | ADDR, b"\x01\xFF\xFF\xFF" + bytes([sa]) + data[:3], True)
            return True
        if pf == 0xEC and sa in self.tp and len(data) >= 3:  # TP.CM from a requester
            if data[0] == 0x11:  # clear to send
                for i, p in enumerate(self.tp[sa][data[2] - 1:data[2] - 1 + data[1]]):
                    self.send(now + i * 0.005, 0x1CEB0000 | sa << 8 | ADDR, p, True)
            elif data[0] in (0x13, 0xFF):  # end of message, or abort
                del self.tp[sa]
            return True
        return False

    def pgn(self, pgn, now, to=0xFF):
        prio, _, fn = J1939[pgn]
        data = fn(drive(now - self.t0))
        if len(data) <= 8:
            self.send(now, prio << 26 | pgn << 8 | ADDR, data, True)
            return
        packets = [bytes([i + 1]) + data[i * 7:i * 7 + 7].ljust(7, b"\xFF") for i in range(-(-len(data) // 7))]
        head = struct.pack("<HBB", len(data), len(packets), 0xFF) + pgn.to_bytes(3, "little")
        if to == 0xFF:  # broadcast announce, then the packets 50 ms apart
            self.send(now, 0x1CECFF00 | ADDR, b"\x20" + head, True)
            for i, p in enumerate(packets):
                self.send(now + 0.05 * (i + 1), 0x1CEBFF00 | ADDR, p, True)
        else:  # request to send; the packets follow the requester's CTS
            self.tp[to] = packets
            self.send(now, 0x1CEC0000 | to << 8 | ADDR, b"\x10" + head, True)


def check():
    e = Ecu(0, broadcast=False)
    h = bytes.fromhex
    assert e.tick(0) == [(0x18EEFF00, True, h("0100000000820080"))], "address claim at start"

    def ask(can_id, data, ext=False, at=10.0):
        assert e.rx(can_id, ext, h(data), at), "request not taken"
        return e.tick(at + 1)

    d = drive(10.0)
    assert ask(0x7DF, "02010C0000000000") == [(0x7E8, False, b"\x04\x41\x0C" + be16(d["rpm"] * 4) + b"\0\0\0")]
    (_, _, r), = ask(0x7E0, "020100")
    assert r[:3] == h("064100") and r[3] & 0x18 == 0x18 and r[4] & 0x10 and r[6] & 1, "PIDs 04, 05, 0C and 'more' flagged"
    (_, _, r), = ask(0x7DF, "03010C0D")  # two PIDs in one request
    assert r[:3] == h("06410C") and r[5] == 0x0D and r[6] == round(d["kmh"])
    assert ask(0x7E0, "0201FF") == [(0x7E8, False, h("037F013100000000"))], "negative response"
    assert not e.rx(0x7DF, False, h("0201FF"), 10) or e.tick(11) == [], "functional: silence"

    for req, head in (("020902", "490201"), ("0322F190", "62F190")):  # VIN, multi-frame
        (_, _, ff), = ask(0x7E0, req)
        assert ff[:2] == bytes([0x10, len(h(head)) + 17]), "first frame"
        cfs = ask(0x7E0, "3000000000000000")
        assert [f[2][0] for f in cfs] == [0x21, 0x22], "consecutive frames after flow control"
        assert (ff[2:] + b"".join(f[2][1:] for f in cfs))[:len(h(head)) + 17] == h(head) + VIN

    assert ask(0x18EA00F8, "EEFE00", True) == [(0x18FEEE00, True, J1939[0xFEEE][2](d))], "requested PGN"
    assert ask(0x18EAFFF8, "04F000", True)[0][0] == 0x0CF00400, "EEC1 at priority 3"
    assert ask(0x18EAFFF8, "00EE00", True)[0][0] == 0x18EEFF00, "address claim on request"
    assert ask(0x18EA00F8, "00FF00", True) == [(0x18E8FF00, True, h("01FFFFFFF800FF00"))], "NACK"
    assert ask(0x18EAFFF8, "00FF00", True) == [], "global request for an unknown PGN: silence"

    rts, = ask(0x18EA00F8, "ECFE00", True)  # VIN to one requester: RTS/CTS
    assert rts == (0x1CECF800, True, h("10120003FFECFE00"))
    dts = ask(0x1CEC00F8, "1103" + "01" + "FFFFECFE00", True)
    assert [f[0] for f in dts] == [0x1CEBF800] * 3 and [f[2][0] for f in dts] == [1, 2, 3]
    assert b"".join(f[2][1:] for f in dts)[:18] == VIN + b"*"
    assert ask(0x1CEC00F8, "13120003FFECFE00", True) == [] and not e.tp, "end of message closes the transfer"

    bam = ask(0x18EAFFF8, "ECFE00", True)  # global request: BAM
    assert bam[0] == (0x1CECFF00, True, h("20120003FFECFE00")) and len(bam) == 4
    assert b"".join(f[2][1:] for f in bam[1:])[:18] == VIN + b"*"

    b = Ecu(0)  # broadcasts: 10 Hz and 1 Hz PGNs over one second
    ids = [f[0] for t in range(100) for f in b.tick(t / 100)]
    assert ids.count(0x0CF00400) == 10 and ids.count(0x18FEEE00) == 1 and 0x18FEE500 not in ids
    print("ecu_sim check ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", nargs="?", choices=["check"])
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--quiet", action="store_true", help="no periodic J1939 broadcasts")
    ap.add_argument("-v", action="store_true", help="print every frame sent")
    a = ap.parse_args()
    if a.cmd == "check":
        return check()
    import can  # only needed on the Pi

    ecu = Ecu(time.monotonic(), not a.quiet)
    with can.Bus(interface="socketcan", channel=a.channel) as bus:
        print(f"ECU simulator on {a.channel}: OBD-II {OBD_TX:03X}, J1939 address {ADDR:02X}", flush=True)
        while True:
            m = bus.recv(0.005)
            now = time.monotonic()
            if m and not m.is_error_frame and not m.is_remote_frame:
                if ecu.rx(m.arbitration_id, m.is_extended_id, bytes(m.data), now):
                    print(f"rx {m.arbitration_id:X} {bytes(m.data).hex().upper()}", flush=True)
            for can_id, ext, data in ecu.tick(now):
                if a.v:
                    print(f"tx {can_id:X} {data.hex().upper()}", flush=True)
                try:
                    bus.send(can.Message(arbitration_id=can_id, is_extended_id=ext, data=data))
                except can.CanError:
                    pass  # nobody acknowledging (device asleep or in listen mode): queue full


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
