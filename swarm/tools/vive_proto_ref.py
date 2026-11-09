#!/usr/bin/env python3
"""Reference decoder for the HTC Vive / Valve Watchman HID protocol, as libsurvive implements it.

Companion to docs/lighthouse-protocol.md. It is a line-by-line port of libsurvive's decoding
(src/driver_vive.c) meant as an executable spec for the Rust port, and it validates itself against
libsurvive recordings made with --record-rawhid:

    python3 swarm/tools/vive_proto_ref.py captures/vive/<file>.rec.gz [--verbose N] [--max-mismatch K]

For every HIDIN report in the recording it decodes the bytes, then compares the lightcap elements
("C" lines) and raw IMU samples ("i" lines) that libsurvive logged right after that report with what
this decoder produced. It also reassembles the device config from the HIDFEAT 0x10/0x11 feature
reports and compares it with the CONFIG line, and checks the IMU_SCALES feature report.

Stdlib + numpy only (numpy is used only for the scaling helpers).

All integers are little-endian unless stated otherwise. Timecodes are 48 MHz ticks, uint32.
"""

import argparse
import gzip
import json
import re
import struct
import sys
import zlib
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

U32 = 0xFFFFFFFF

# ---------------------------------------------------------------------------------------------
# Report ids (driver_vive.c enum vive_report_ids)
# ---------------------------------------------------------------------------------------------
REPORT_IMU_SCALES = 0x01  # feature, GET: [01, gyro_mode, acc_mode, ...]
REPORT_CHANGE_MODE = 0x04  # feature, SET: [04, mode, 0, 0, 0] (USB devices)
REPORT_VERSION = 0x05  # feature, GET
REPORT_LIGHTCAP_VERBOSITY = 0x07  # feature, SET: [07, 03, 0, 0, 0]
REPORT_CONFIG_READMODE = 0x10  # feature, GET: [10, len_hi, len_lo, ...]
REPORT_CONFIG_READ = 0x11  # feature, GET: [11, n, data[n] ...]
REPORT_IMU = 0x20  # input, USB IMU interface
REPORT_USB_TRACKER_LIGHTCAP_V1 = 0x21  # input, TR0/WW0 lightcap interface
REPORT_RF_WATCHMAN = 0x23  # input, dongle: one 29-byte RF sub-packet
REPORT_RF_WATCHMANx2 = 0x24  # input, dongle: two 29-byte RF sub-packets
REPORT_USB_LIGHTCAP_V1 = 0x25  # input, T20/HMD lightcap interface, LH1
REPORT_RF_TURN_OFF = 0x26  # input, dongle: tracker turned off
REPORT_USB_LIGHTCAP_V2 = 0x27  # input, LH2 (not decoded by libsurvive beyond a debug dump)
REPORT_USB_LIGHTCAP_RAW_MODE_1 = 0x28  # input, LH2 raw mode 1
REPORT_COMMAND = 0xFF  # feature, SET: [ff, cmd, len, data...]


# ---------------------------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------------------------
@dataclass
class LightcapElement:
    sensor_id: int  # uint8, raw (un-mapped) sensor/channel id
    timestamp: int  # uint32, rising edge, 48 MHz ticks
    length: int  # uint16, ticks

    def key(self):
        return ("C", self.sensor_id, self.timestamp, self.length)


@dataclass
class RawImu:
    timecode: int  # uint32
    acc: Tuple[int, int, int]  # int16 raw counts
    gyro: Tuple[int, int, int]  # int16 raw counts
    seq: int = 0  # USB: 8-bit sample counter; RF: libsurvive passes 0

    def key(self):
        return ("i", self.timecode) + tuple(self.acc) + tuple(self.gyro) + (self.seq,)


@dataclass
class Event:
    kind: str  # 'battery', 'buttons', 'heartbeat', 'turn_off', 'error', 'meta'
    data: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------------------------
# Wired (USB) reports
# ---------------------------------------------------------------------------------------------
class UsbImuState:
    """libsurvive keeps one 8-bit 'oldcode' per object (SurviveObject.oldcode, starts at 0)."""

    def __init__(self):
        self.oldcode = 0


def decode_usb_imu_0x20(report: bytes, state: UsbImuState) -> List[RawImu]:
    """Report 0x20 on the IMU interface (survive_data_cb_locked, case USB_IF_*_IMU).

    [0]      0x20
    [1+17k]  int16 acc[3], int16 gyro[3], uint32 timecode, uint8 seq   for k = 0, 1, 2
    A sample is new iff (int8)(seq - oldcode) > 0; then oldcode = seq. Others are re-sends.
    """
    assert report[0] == REPORT_IMU
    out = []
    p = 1
    for _ in range(3):
        if p >= len(report):
            break
        ax, ay, az, gx, gy, gz, tc, seq = struct.unpack_from("<6hIB", report, p)
        p += 17
        cd = ((seq - state.oldcode) + 128) % 256 - 128  # int8_t cd = code - obj->oldcode
        if cd > 0:
            state.oldcode = seq
            out.append(RawImu(tc, (ax, ay, az), (gx, gy, gz), seq))
    return out


def decode_usb_lightcap_0x25(report: bytes) -> List[LightcapElement]:
    """Report 0x25 (LH1 lightcap, T20 / HMD lightcap interface).

    [0]     0x25
    [1+7k]  uint8 sensor, uint16 length, uint32 timestamp    for k = 0..8
    sensor > 0xfd (0xfe, 0xff) marks an unused slot and is skipped.
    """
    assert report[0] == REPORT_USB_LIGHTCAP_V1
    out = []
    p = 1
    for _ in range(9):
        if p >= len(report):
            break
        sensor, length, ts = struct.unpack_from("<BHI", report, p)
        p += 7
        if sensor > 0xFD:
            continue
        out.append(LightcapElement(sensor, ts, length))
    return out


def decode_usb_lightcap_0x21(report: bytes) -> List[LightcapElement]:
    """Report 0x21 (TR0 / wired watchman lightcap): 7 x {uint16 sensor, uint16 length, uint32 ts}."""
    out = []
    p = 1
    for _ in range(7):
        if p + 8 > len(report):
            break
        sensor, length, ts = struct.unpack_from("<HHI", report, p)
        p += 8
        sensor &= 0xFF
        if sensor > 0xFD:
            continue
        out.append(LightcapElement(sensor, ts, length))
    return out


# ---------------------------------------------------------------------------------------------
# Watchman RF (dongle reports 0x23 / 0x24)
# ---------------------------------------------------------------------------------------------
def read_light_data(time16: int, buf: bytes, p0: int, pend: int, reference_time: int):
    """Port of read_light_data() (driver_vive.c). buf[p0:pend] is the light block:

        [sensor bytes S...][time deltas D... (VLQ, read backwards)][T0 T1 T2]

    Returns (code, elements) where code is libsurvive's return value (event count, or a negative
    error) and elements are in the order libsurvive hands them to handle_lightcap (ascending start
    time). reference_time is the uint32 timecode of the last IMU sample (activations.last_imu).
    """
    if pend - p0 <= 3:
        return 0, []

    ids = p0
    ev = pend - 4
    last = (((time16 >> 8) & 0xFF) << 24) | (buf[ev + 3] << 16) | (buf[ev + 2] << 8) | buf[ev + 1]
    ref = reference_time & U32
    # Pick the 2^24 epoch closest to the last IMU timecode.
    if last > ref and last - ref > (1 << 23):
        last = (last - (1 << 24)) & U32
    elif ref > last and ref - last > (1 << 23):
        last = (last + (1 << 24)) & U32

    times = [0] * 16
    times[0] = last
    ti = 0
    exited = False
    while ids + (ti >> 1) < ev:
        delta = 0
        ev_start = ev
        while True:
            delta = ((delta << 7) | (buf[ev] & 0x7F)) & U32
            b = buf[ev]
            ev -= 1
            if b & 0x80:
                break
            if ids + (ti >> 1) > ev:
                ev = ev_start
                exited = True
                break
        if exited:
            break
        last = (last - delta) & U32
        if ti >= 16:
            return -8, []
        if ti + 1 >= 16:
            # C writes times[16] here (one past the array). Never seen with <=27-byte payloads.
            return -9, []
        ti += 1
        times[ti] = last

    if ti % 2 == 0 and ti > 0:
        ti -= 1
        while True:
            ev += 1
            if buf[ev] & 0x80:
                break

    sensor_byte_cnt = ev - ids + 1
    if sensor_byte_cnt > (ti >> 1) + 1:
        # More leading bytes than events: libsurvive uses the bytes right before the deltas.
        ids = ev - (ti >> 1)

    sensors = []
    for _ in range((ti >> 1) + 1):
        s = buf[ids]
        sensors.append(((s >> 3) & 0x1F, s & 0x7))
        ids += 1

    event_count = (ti + 1) >> 1
    les = [None] * 8
    report_order = [0] * 16
    t = -1
    for i in range(event_count):
        while True:
            t += 1
            if times[t] != 0:
                break
            if t + 1 >= 16:
                return -2, []
        if t >= 16:
            return -3, []
        start = t + sensors[i][1] + 1
        if start >= 16:
            return -4, []
        if report_order[start] != 0:
            return -5, []
        report_order[start] = i + 1
        les[i] = LightcapElement(sensors[i][0], times[start], (times[t] - times[start]) & 0xFFFF)
        times[start] = 0

    ordered = []
    for i in range(16):
        if len(ordered) >= 10:
            break
        o = report_order[i]
        if o:
            le = les[o - 1]
            if not (le.length != 0 or le.timestamp != 0):
                return -6, []
            ordered.append(le)
    # parse_and_process_lightcap calls handle_lightcap for i = cnt-1 .. 0 -> ascending time.
    return event_count, list(reversed(ordered[:event_count]))


@dataclass
class WatchmanState:
    last_imu: int = 0  # activations.last_imu (lower 32 bits)
    lightcap_mode: str = "raw0"  # raw0 for LH1 (set when the startup magics finish)
    charge: Optional[int] = None
    charging: Optional[bool] = None


def handle_watchman_v2(slot: bytes, state: WatchmanState):
    """One 29-byte RF sub-packet (survive_handle_watchman -> handle_watchman_v2, LH1/raw0 path).

    slot[0]   time bits 15..8   (time16 = slot[0] << 8 | slot[2])
    slot[1]   size: payload is slot[3 : 3 + size - 1]   (uint8 arithmetic)
    slot[2]   time bits 7..0
    slot[3..] payload: flags byte, then [IMU 13][metadata 2|3][input ...][light block]
    Returns a list of RawImu / LightcapElement / Event in libsurvive's emission order.
    """
    out = []
    time16 = (slot[0] << 8) | slot[2]
    size = (slot[1] - 1) & 0xFF
    p = 3
    pend = 3 + size
    buf = slot if len(slot) >= pend else slot + bytes(pend - len(slot))

    flags = buf[p]
    p += 1
    if flags == 0xE2:
        return [Event("heartbeat", {})]

    has_errors = False
    if flags & 0x80:  # IMU: uint8 time_lsb, int16 acc[3], int16 gyro[3]
        if pend - p >= 7:  # (sic) libsurvive only checks for 7 bytes, then reads 13
            tl = buf[p]
            ax, ay, az, gx, gy, gz = struct.unpack_from("<6h", buf, p + 1)
            p += 13
            tc = ((time16 << 16) | (tl << 8)) & U32
            out.append(RawImu(tc, (ax, ay, az), (gx, gy, gz), 0))
            state.last_imu = tc
    if flags & 0x40:  # metadata
        marker = buf[p]
        p += 1
        if marker == 0x80:
            bat = buf[p]
            p += 1
            pct, chg = bat & 0x7F, bool(bat & 0x80)
            if pct != 0 or chg:
                state.charge, state.charging = pct, chg
            out.append(Event("battery", {"percent": pct, "charging": chg, "raw": bat}))
        elif marker == 0x20:
            out.append(Event("meta", {"marker": 0x20, "byte": buf[p]}))
            p += 1
        elif marker == 0x30:
            out.append(Event("meta", {"marker": 0x30, "bytes": bytes(buf[p:p + 2]).hex()}))
            p += 2
        else:
            out.append(Event("error", {"why": "unknown metadata marker 0x%02x; rest dropped" % marker}))
            return out
    if flags & 0x20:  # input (buttons etc.)
        if pend - p < 1:
            out.append(Event("error", {"why": "input flag without data"}))
            return out
        in_flags = buf[p]
        p += 1
        ok, p, ev = handle_input(in_flags, buf, p, pend)
        out.append(ev)
        has_errors = not ok
    if flags & 0x10 and not has_errors:
        if state.lightcap_mode == "raw0":
            code, les = read_light_data(time16, buf, p, pend, state.last_imu)
            if code < 0:
                out.append(Event("error", {"why": "read_light_data returned %d" % code}))
            else:
                # handle_lightcap drops zero-length elements before recording them.
                out.extend(le for le in les if le.length != 0)
        else:
            out.append(Event("error", {"why": "raw1 (LH2) lightcap not implemented here"}))
    return out


def handle_input(in_flags: int, buf: bytes, p: int, pend: int):
    """handle_input(): RF button/trigger/trackpad or Knuckles 'gen two' block."""
    d = {"flags": in_flags}
    if in_flags & 0x7:
        if in_flags & 0x1:
            if pend - p < 1:
                return False, p, Event("error", {"why": "short input"})
            d["buttons"] = buf[p]
            p += 1
        if in_flags & 0x4:
            if pend - p < 1:
                return False, p, Event("error", {"why": "short input"})
            d["trigger"] = buf[p] / 255.0
            p += 1
        if in_flags & 0x2:
            if pend - p < 4:
                return False, p, Event("error", {"why": "short input"})
            x, y = struct.unpack_from("<hh", buf, p)
            d["trackpad"] = (x / 32768.0, y / 32768.0)
            p += 4
        return True, p, Event("buttons", d)
    gen2 = buf[p]
    p += 1
    if gen2 == 0xA1:
        if pend - p < 7:
            return False, p, Event("error", {"why": "short knuckles input"})
        d["touch"] = buf[p]
        d["proximity"] = [b / 255.0 for b in buf[p + 1:p + 7]]
        p += 7
        return True, p, Event("buttons", d)
    return False, p, Event("error", {"why": "unknown gen-two input type 0x%02x" % gen2})


def decode_dongle_report(report: bytes, state: WatchmanState):
    """Dongle input report (USB_IF_WATCHMAN1). 0x23 = one sub-packet at report[1:30];
    0x24 = two, at report[1:30] and report[30:59] (survive_data_cb_locked)."""
    rid = report[0]
    if rid == REPORT_RF_WATCHMAN:
        return handle_watchman_v2(report[1:30], state)
    if rid == REPORT_RF_WATCHMANx2:
        return handle_watchman_v2(report[1:30], state) + handle_watchman_v2(report[30:59], state)
    if rid == REPORT_RF_TURN_OFF:
        return [Event("turn_off", {})]
    return [Event("error", {"why": "unknown watchman report 0x%02x" % rid})]


# ---------------------------------------------------------------------------------------------
# Feature reports: config, version, IMU scales
# ---------------------------------------------------------------------------------------------
def reassemble_config(feature_gets: List[bytes]):
    """0x10 gives the compressed length (big-endian uint16 at [1..2]); each 0x11 gives
    n = [1] bytes at [2..2+n); n == 0 ends. The result is a zlib stream (78 9c ...)."""
    expected = None
    data = b""
    for b in feature_gets:
        if b[0] == REPORT_CONFIG_READMODE:
            expected = (b[1] << 8) | b[2]
            data = b""
        elif b[0] == REPORT_CONFIG_READ and expected is not None:
            n = b[1]
            if n == 0:
                return expected, data, zlib.decompress(data)
            data += b[2:2 + n]
    return expected, data, None


def parse_version(b: bytes) -> dict:
    """Feature 0x05 (parse_tracker_version_info), fields after the report id, packed LE."""
    rev, board, name, hwid, a, fmaj, fmin, fpat, h, k = struct.unpack_from("<II32sIIHBBBQ", b, 1)
    return {"revision": rev, "board_model": board, "fw_name": name.split(b"\0")[0].decode("latin1"),
            "hardware_id": hwid, "a": a, "fpga": (fmaj, fmin, fpat), "h": h, "k": k}


def imu_raw_scales(gyro_mode: int, acc_mode: int):
    """survive_default_set_imu_scale_modes: counts -> g and rad/s."""
    acc = [2, 4, 8, 16][acc_mode] / 32768.0
    gyro = [250, 500, 1000, 2000][gyro_mode] / 32768.0 * np.pi / 180.0
    return acc, gyro


def calibrate_imu(raw: RawImu, acc_scale_raw, gyro_scale_raw, cfg_imu: dict):
    """survive_default_raw_imu_process: (raw*raw_scale - bias) * scale. Config acc_bias is in
    milli-g (divided by 1000 when loaded); gyro_bias is in rad/s."""
    acc = np.array(raw.acc, float) * acc_scale_raw
    gyro = np.array(raw.gyro, float) * gyro_scale_raw
    acc = (acc - np.array(cfg_imu.get("acc_bias", [0, 0, 0])) / 1000.0) * np.array(cfg_imu.get("acc_scale", [1, 1, 1]))
    gyro = (gyro - np.array(cfg_imu.get("gyro_bias", [0, 0, 0]))) * np.array(cfg_imu.get("gyro_scale", [1, 1, 1]))
    return acc, gyro


# ---------------------------------------------------------------------------------------------
# LH1 timing and OOTX (ootx_decoder.c, survive_process_gen1.c, disambiguator_statebased.c)
# ---------------------------------------------------------------------------------------------
TICKS_PER_SECOND = 48_000_000
TICKS_PER_ROTATION = 400_000  # one sweep period (120 Hz)


def acode_timing(acode: int) -> int:
    """ACODE_TIMING: expected sync length for acode = skip<<2 | data<<1 | axis (ticks)."""
    return 3000 + (acode & 1) * 500 + ((acode >> 1) & 1) * 1000 + ((acode >> 2) & 1) * 2000 - 250


def find_acode(length: int) -> int:
    """find_acode: classify a sync pulse length (ticks) into acode 0..7, -1 if not a sync."""
    for a, upper in enumerate(range(3000, 6501, 500)):
        if length < 2500 + 50:
            return -1
        if length < upper + 50:
            return a
    return -1


def sweep_angle(timeinsweep: int) -> float:
    """survive_default_light_process: radians, 0 at the middle of the sweep."""
    return timeinsweep * (np.pi / TICKS_PER_ROTATION) - np.pi / 2


def half_to_float(x: int) -> float:
    return float(np.frombuffer(struct.pack("<H", x), dtype="<f2")[0])


def parse_lighthouse_info_v6(d: bytes) -> dict:
    """OOTX payload (lighthouse_info_v6, packed, little-endian; float16 = IEEE half)."""
    fw, lhid = struct.unpack_from("<HI", d, 0)
    h = lambda o: half_to_float(struct.unpack_from("<H", d, o)[0])
    return {"fw_version": fw >> 6, "protocol": fw & 0x3F, "id": lhid,
            "fcal_phase": (h(6), h(8)), "fcal_tilt": (h(10), h(12)),
            "sys_unlock_count": d[14], "hw_version": d[15],
            "fcal_curve": (h(16), h(18)), "accel_dir": struct.unpack_from("<3b", d, 20),
            "fcal_gibphase": (h(23), h(25)), "fcal_gibmag": (h(27), h(29)),
            "mode_current": d[31], "sys_faults": d[32]}


class OotxDecoder:
    """Bit-level port of ootx_pump_bit: preamble = 17 zeros then a 1; after it a sync bit (must be
    1) follows every 16 data bits; packet = uint16 LE length, payload (padded to even), uint32 LE
    CRC32 (zlib crc32 of the unpadded payload). Data bits fill bytes MSB first."""

    def __init__(self):
        self.preamble = 0xFFFFFFFF
        self.bits_processed = 0
        self.found = False
        self.buf = bytearray(64)
        self.off = 0
        self.nbits = 0
        self.packets, self.bad_crc, self.bad_sync = [], 0, 0

    def _reset(self):
        self.off = self.nbits = 0
        self.found = False
        self.buf[0] = self.buf[1] = 0

    def pump(self, bit: int):
        self.bits_processed += 1
        self.preamble = ((self.preamble << 1) | bit) & 0xFFFFFFFF
        if (self.preamble & 0x3FFFF) == 0x1:
            self._reset()
            self.bits_processed = 0
            self.found = True
            return
        if self.bits_processed > 16:
            if bit == 0:
                if self.found:
                    self.bad_sync += 1
                self._reset()
            self.bits_processed = 0
            return
        if not self.found:
            return
        mask = 1 << (7 - self.nbits)
        self.buf[self.off] = (self.buf[self.off] | mask) if bit else (self.buf[self.off] & ~mask)
        self.nbits += 1
        if self.nbits > 7:
            self.nbits = 0
            self.off += 1
            if self.off >= 64:
                self.off = 0
                self.found = False
        n = self.buf[0] | (self.buf[1] << 8)
        padded = n + (n & 1)
        if self.found and self.off >= padded + 6:
            payload = bytes(self.buf[2:2 + n])
            crc = struct.unpack_from("<I", self.buf, 2 + padded)[0]
            if zlib.crc32(payload) == crc:
                self.packets.append(payload)
            else:
                self.bad_crc += 1
            self._reset()


# ---------------------------------------------------------------------------------------------
# Recording parsing and validation
# ---------------------------------------------------------------------------------------------
LINE_RE = re.compile(r"(\d+\.\d+) (\S+) (HIDIN|HIDFEAT|CONFIG|IMU_SCALES|C|i|I) (.*)$")


# Disambiguator output for a sync pulse: <dev> L|R X|Y <-1|-2> <acode> <timeinsweep> <timecode> <length> <lh>
SYNC_RE = re.compile(r"(\d+\.\d+) (\S+) [LR] [XY] (-[12] \d+ \d+ \d+ \d+ \d+)\s*$")
GEN_RE = re.compile(r"(\d+\.\d+) INFO LOG Detected LH gen (\d) system")


def parse_lines(path):
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", errors="replace") as f:
        for raw in f:
            m = LINE_RE.search(raw.rstrip("\r\n"))
            g = GEN_RE.search(raw)
            sy = SYNC_RE.search(raw)
            if sy:
                yield float(sy.group(1)), sy.group(2), "SYNC", sy.group(3)
            elif g:
                yield float(g.group(1)), "", "GEN", g.group(2)
            elif m:
                yield float(m.group(1)), m.group(2), m.group(3), m.group(4)


def oracle_key(op, rest):
    """Key of a libsurvive C / i line, or None if the line is damaged."""
    t = rest.split()
    try:
        if op == "C" and len(t) == 3:
            return ("C", int(t[0]), int(t[1]), int(t[2]))
        # i <mask> <timecode> ax ay az gx gy gz mx my mz <id>
        if op == "i" and len(t) == 12:
            return ("i", int(t[1])) + tuple(int(float(x)) for x in t[2:8]) + (int(t[11]),)
        if op == "I" and len(t) == 12:
            return ("I", int(t[1])) + tuple(float(x) for x in t[2:8])
    except ValueError:
        pass
    return None


def validate(path, verbose=0, max_mismatch=10):
    st = WatchmanState()
    usb_imu = UsbImuState()
    have_ref = False  # an IMU timecode is known (needed to place light timestamps)
    await_gen = False  # after a libsurvive restart: wait until it re-detects LH1 and logs light again
    n_reports = 0
    last_t = None
    feature_gets = []
    config_json = None
    imu_scales_line = None
    imu_scales_feature = None
    stats = {"C_exp": 0, "C_got": 0, "C_match": 0, "i_exp": 0, "i_got": 0, "i_match": 0,
             "reports": 0, "reports_ok": 0, "pre_first_report": 0, "skipped_no_ref": 0, "errors": 0,
             "corrupt_lines": 0, "restarts": 0, "skipped_restart": 0, "orphan_lines": 0,
             "truncated_last": 0, "I_cmp": 0, "I_match": 0, "events": {}}
    cal_got = []  # libsurvive's calibrated IMU lines for the pending report
    scales = None  # (acc, gyro) raw scale from the IMU_SCALES feature report
    cfg_imu = None  # "imu" object of the device config
    mismatches = []
    syncs = {}  # lh -> [(acode, length)] from libsurvive's disambiguated sync records

    pending = None  # [report hex, expected keys, libsurvive keys, counted]

    def close(p, at_eof=False):
        cal = list(cal_got)
        cal_got.clear()
        if p is None:
            return
        rhex, exp, got, counted = p
        if not counted:
            return
        if at_eof and exp != got and exp[:len(got)] == got:
            # The recording (or survive-cli) stopped while libsurvive was logging this report's output.
            stats["truncated_last"] += 1
            return
        stats["reports"] += 1
        ce = [k for k in exp if k[0] == "C"]
        cg = [k for k in got if k[0] == "C"]
        ie = [k for k in exp if k[0] == "i"]
        ig = [k for k in got if k[0] == "i"]
        stats["C_exp"] += len(ce)
        stats["C_got"] += len(cg)
        stats["i_exp"] += len(ie)
        stats["i_got"] += len(ig)
        stats["C_match"] += sum(1 for a, b in zip(ce, cg) if a == b)
        stats["i_match"] += sum(1 for a, b in zip(ie, ig) if a == b)
        if exp == got:
            stats["reports_ok"] += 1
            # Calibrated IMU ("I" lines): raw * IMU_SCALES scale, minus config bias, times config scale.
            if scales is not None and cfg_imu is not None:
                raws = [k for k in exp if k[0] == "i"]
                for k, g in zip(raws, cal):
                    acc, gyro = calibrate_imu(RawImu(k[1], k[2:5], k[5:8]), scales[0], scales[1], cfg_imu)
                    stats["I_cmp"] += 1
                    if g[1] == k[1] and np.max(np.abs(np.concatenate([acc, gyro]) - np.array(g[2:8]))) < 1.5e-6:
                        stats["I_match"] += 1
        elif len(mismatches) < max_mismatch:
            mismatches.append((rhex, exp, got))

    for t, dev, op, rest in parse_lines(path):
        if n_reports and last_t is not None and t < last_t - 1.0:
            # survive-cli was restarted inside this recording (its clock starts over). The new
            # instance re-runs startup and LH generation detection, which resets last_imu and
            # swallows light until LH1 is recognised; resynchronise on its first logged light.
            close(pending, at_eof=True)  # the old instance may have been killed mid-output
            pending = None
            stats["restarts"] += 1
            st = WatchmanState()
            usb_imu = UsbImuState()
            have_ref = False
            await_gen = True
        last_t = t
        if op == "HIDFEAT":
            parts = rest.split()
            if parts[0] == "GET" and int(parts[2]) > 0 and n_reports == 0:
                b = bytes.fromhex(parts[3])
                feature_gets.append(b)
                if b[0] == REPORT_IMU_SCALES:
                    imu_scales_feature = (b[1], b[2])
            continue
        if op == "CONFIG":
            config_json = config_json or rest
            try:
                cfg_imu = json.loads(config_json).get("imu")
            except ValueError:
                pass
            continue
        if op == "IMU_SCALES":
            p = rest.split()
            imu_scales_line = imu_scales_line or (int(p[0]), int(p[1]))
            scales = imu_raw_scales(*imu_scales_line)
            continue
        if op == "HIDIN":
            close(pending)
            pending = None
            parts = rest.split()
            if len(parts) != 4 or not re.fullmatch(r"[0-9a-f]+", parts[3]) or len(parts[3]) != 2 * int(parts[2]):
                stats["corrupt_lines"] += 1  # line cut short by the restart / output interleaving
                have_ref = False
                continue
            report = bytes.fromhex(parts[3])
            n_reports += 1
            rid = report[0]
            if rid in (REPORT_RF_WATCHMAN, REPORT_RF_WATCHMANx2, REPORT_RF_TURN_OFF):
                counted = have_ref and not await_gen
                evs = decode_dongle_report(report, st)
                if any(isinstance(e, RawImu) for e in evs):
                    have_ref = True
            elif rid == REPORT_IMU:
                evs = decode_usb_imu_0x20(report, usb_imu)
                counted = not await_gen
            elif rid == REPORT_USB_LIGHTCAP_V1:
                evs = decode_usb_lightcap_0x25(report)
                counted = not await_gen
            else:
                evs = [Event("error", {"why": "report 0x%02x not decoded" % rid})]
                counted = not await_gen
            exp = []
            for e in evs:
                if isinstance(e, (RawImu, LightcapElement)):
                    exp.append(e.key())
                else:
                    stats["events"][e.kind] = stats["events"].get(e.kind, 0) + 1
                    if e.kind == "error":
                        stats["errors"] += 1
                        if verbose:
                            print("  %.6f decode error: %s  [%s]" % (t, e.data["why"], parts[3]))
            if not counted:
                stats["skipped_restart" if await_gen else "skipped_no_ref"] += 1
            pending = [parts[3], exp, [], counted]
            continue
        if op == "I":
            key = oracle_key(op, rest)
            if key is not None and pending is not None and pending[3]:
                cal_got.append(key)
            continue
        if op in ("C", "i"):
            key = oracle_key(op, rest)
            if key is None:
                stats["corrupt_lines"] += 1
                if pending is not None:
                    pending[3] = False  # its libsurvive output is damaged; don't score this report
                continue
            if pending is None:
                if n_reports == 0:
                    stats["pre_first_report"] += 1
                else:
                    stats["orphan_lines"] += 1
                if op == "i":  # seed the IMU reference from an IMU whose report we don't have
                    st.last_imu = key[1]
                    have_ref = True
                continue
            pending[2].append(key)
            continue
        if op == "SYNC":
            sid, acode, _, tc, length, lh = (int(x) for x in rest.split())
            syncs.setdefault(lh, []).append((acode, length))
            continue
        if op == "GEN":
            # survive_default_gen_detected_process: from here on light is logged, and the
            # activations were reset, so last_imu is 0 until the next IMU sample (light in between
            # is placed relative to 0). The report being processed is only partly logged.
            if pending is not None:
                pending[3] = False
            await_gen = False
            st.last_imu = 0
            have_ref = True
    close(pending, at_eof=True)

    # ---- report ----
    print("file: %s" % path)
    print("HID input reports: %d (compared: %d, skipped before first IMU reference: %d)" %
          (n_reports, stats["reports"], stats["skipped_no_ref"]))
    print("C/i lines before the first HIDIN (decoded from reports not in the recording): %d" %
          stats["pre_first_report"])
    if stats["restarts"] or stats["corrupt_lines"] or stats["orphan_lines"]:
        print("libsurvive restarts inside the recording: %d (reports skipped while it re-detected LH1: %d); "
              "corrupt HIDIN/C/i lines: %d; C/i lines without their report: %d" %
              (stats["restarts"], stats["skipped_restart"], stats["corrupt_lines"], stats["orphan_lines"]))
    if stats["truncated_last"]:
        print("last report excluded: recording stopped while libsurvive was logging its output (prefix matched)")

    def pct(a, b):
        return 100.0 * a / b if b else 100.0

    print("lightcap elements: libsurvive %d, decoder %d, identical in order %d (%.4f%%)" %
          (stats["C_got"], stats["C_exp"], stats["C_match"], pct(stats["C_match"], max(stats["C_got"], stats["C_exp"]))))
    print("raw IMU samples:   libsurvive %d, decoder %d, identical in order %d (%.4f%%)" %
          (stats["i_got"], stats["i_exp"], stats["i_match"], pct(stats["i_match"], max(stats["i_got"], stats["i_exp"]))))
    if stats["I_cmp"]:
        print("calibrated IMU (I lines) reproduced within 1.5e-6: %d / %d" % (stats["I_match"], stats["I_cmp"]))
    print("reports whose full C/i output is identical: %d / %d" % (stats["reports_ok"], stats["reports"]))
    print("non-sample events: %s" % json.dumps(stats["events"], sort_keys=True))

    if feature_gets:
        exp_len, comp, cfg = reassemble_config(feature_gets)
        if cfg is None:
            print("config: incomplete 0x10/0x11 sequence in the recording")
        else:
            same = None
            if config_json is not None:
                try:
                    same = json.loads(cfg) == json.loads(config_json)
                except ValueError:
                    same = False
            print("config: 0x10 announced %d bytes, 0x11 chunks gave %d bytes zlib -> %d bytes JSON; "
                  "equals CONFIG line: %s" % (exp_len, len(comp), len(cfg), same))
        for b in feature_gets:
            if b[0] == REPORT_VERSION:
                v = parse_version(b)
                print("version (0x05): revision %d board %d fw '%s' hw_id 0x%08x fpga %s" %
                      (v["revision"], v["board_model"], v["fw_name"], v["hardware_id"], v["fpga"]))
                break
    if imu_scales_feature:
        a, g = imu_raw_scales(*imu_scales_feature)
        print("IMU_SCALES feature (0x01): gyro mode %d, acc mode %d -> acc %.6g g/count, gyro %.6g rad/s/count; "
              "matches IMU_SCALES line: %s" % (imu_scales_feature[0], imu_scales_feature[1], a, g,
                                                imu_scales_line == imu_scales_feature))

    if syncs:
        # OOTX from the data bit of each lighthouse's syncs, as libsurvive's disambiguator classified them.
        for lh in sorted(syncs):
            dec = OotxDecoder()
            for acode, _ in syncs[lh]:
                dec.pump((acode >> 1) & 1)
            ids = sorted({parse_lighthouse_info_v6(p)["id"] for p in dec.packets})
            print("OOTX lh%d: %d sync bits -> %d packets with good CRC32 (bad CRC %d, bad sync bit %d); ids %s" %
                  (lh, len(syncs[lh]), len(dec.packets), dec.bad_crc, dec.bad_sync, ids))
            if dec.packets and verbose:
                print("   ", parse_lighthouse_info_v6(dec.packets[-1]))
        lens = {}
        for lh in syncs:
            for acode, length in syncs[lh]:
                lens.setdefault(acode, []).append(length)
        print("sync length by acode (median ticks / ACODE_TIMING): " + ", ".join(
            "%d: %d/%d" % (a, int(np.median(v)), acode_timing(a)) for a, v in sorted(lens.items())))
    if mismatches:
        print("first %d mismatching reports:" % len(mismatches))
        for rhex, exp, got in mismatches:
            print("  report %s" % rhex)
            print("    decoder:    %s" % exp)
            print("    libsurvive: %s" % got)
    return stats


def selftest():
    """Synthetic checks of the wired layouts (no wired capture available) and the worked RF example
    from docs/lighthouse-protocol.md."""
    # 0x20: three samples; the third repeats the second's sequence number and must be ignored.
    rep = bytes([REPORT_IMU])
    for seq, tc in ((7, 1000), (8, 1192000), (8, 1192000)):
        rep += struct.pack("<6hIB", 1, -2, 4096, 10, -20, 30, tc, seq)
    st = UsbImuState()
    got = decode_usb_imu_0x20(rep, st)
    assert [g.seq for g in got] == [7, 8] and got[0].acc == (1, -2, 4096) and got[1].timecode == 1192000, got
    assert decode_usb_imu_0x20(rep, st) == []  # nothing new on a resend
    # seq wraps: 255 -> 0 is "newer" ((int8)(0 - 255) = 1)
    st.oldcode = 255
    assert len(decode_usb_imu_0x20(bytes([REPORT_IMU]) + struct.pack("<6hIB", 0, 0, 0, 0, 0, 0, 5, 0) * 3, st)) == 1
    # 0x25: 9 slots of 7 bytes, 0xff = empty
    rep = bytes([REPORT_USB_LIGHTCAP_V1]) + struct.pack("<BHI", 3, 2950, 123456) + struct.pack("<BHI", 0xFF, 0xFFFF, 0xFFFFFFFF) * 8
    assert decode_usb_lightcap_0x25(rep) == [LightcapElement(3, 123456, 2950)]
    # RF worked example (docs, section 5.5)
    ws = WatchmanState(last_imu=2155894784)
    les = decode_dongle_report(bytes.fromhex("2380188010422a32988718930a0db40b9c02b602a7019702310a8531cec8"), ws)
    assert [(e.sensor_id, e.timestamp, e.length) for e in les] == [
        (19, 2155982675, 3079), (6, 2156200045, 2054), (5, 2156201505, 761), (8, 2156201789, 756)], les
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", nargs="?")
    ap.add_argument("--selftest", action="store_true", help="run the synthetic layout checks and exit")
    ap.add_argument("--verbose", type=int, default=0)
    ap.add_argument("--max-mismatch", type=int, default=10)
    a = ap.parse_args()
    if a.selftest or not a.recording:
        selftest()
        if not a.recording:
            return
    s = validate(a.recording, a.verbose, a.max_mismatch)
    ok = s["C_match"] == s["C_got"] == s["C_exp"] and s["i_match"] == s["i_got"] == s["i_exp"]
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
