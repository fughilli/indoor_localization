"""Live window of the raw HID reports from a Vive Tracker (or Watchman dongle).

  python3 swarm/tools/vive_raw_view.py [--log FILE.jsonl]

Talks to the tracker directly through hidapi (brew install hidapi); no
libsurvive or SteamVR. Don't run it at the same time as survive-cli.

What it shows:
  - every HID interface of every Valve device (VID 28de), with packet rates;
  - a scrolling log of decoded reports: IMU (0x20), LHv1 lightcap (0x25),
    LHv2 lightcap (0x27), anything else as hex;
  - light statistics over the last second: hits per sensor, the pulse-width
    histogram, and how many pulses are long enough to be LHv1 sync flashes
    (60-135 us; sweeps hits are shorter);
  - buttons to send the lightcap mode switch libsurvive uses (feature
    report 0x04: 0 = LHv1, 1 = LHv2, 3 = raw2) and the verbosity report
    (0x07 0x03), plus free-form get/send feature report boxes.

Timestamps in light and IMU reports are ticks of the tracker's 48 MHz clock.
"""

import argparse
import collections
import ctypes
import ctypes.util
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

VID = 0x28DE
TICKS_PER_US = 48.0
SYNC_MIN_US = 55.0


class HidDeviceInfo(ctypes.Structure):
    pass


HidDeviceInfo._fields_ = [
    ("path", ctypes.c_char_p),
    ("vendor_id", ctypes.c_ushort),
    ("product_id", ctypes.c_ushort),
    ("serial_number", ctypes.c_wchar_p),
    ("release_number", ctypes.c_ushort),
    ("manufacturer_string", ctypes.c_wchar_p),
    ("product_string", ctypes.c_wchar_p),
    ("usage_page", ctypes.c_ushort),
    ("usage", ctypes.c_ushort),
    ("interface_number", ctypes.c_int),
    ("next", ctypes.POINTER(HidDeviceInfo)),
]


def load_hidapi():
    for name in ("/opt/homebrew/lib/libhidapi.dylib", ctypes.util.find_library("hidapi"), "libhidapi.so.0"):
        if not name:
            continue
        try:
            lib = ctypes.CDLL(name)
            break
        except OSError:
            continue
    else:
        raise SystemExit("hidapi not found (brew install hidapi)")
    lib.hid_init.restype = ctypes.c_int
    lib.hid_enumerate.restype = ctypes.POINTER(HidDeviceInfo)
    lib.hid_enumerate.argtypes = [ctypes.c_ushort, ctypes.c_ushort]
    lib.hid_free_enumeration.argtypes = [ctypes.POINTER(HidDeviceInfo)]
    lib.hid_open_path.restype = ctypes.c_void_p
    lib.hid_open_path.argtypes = [ctypes.c_char_p]
    lib.hid_close.argtypes = [ctypes.c_void_p]
    lib.hid_read_timeout.restype = ctypes.c_int
    lib.hid_read_timeout.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_int]
    lib.hid_send_feature_report.restype = ctypes.c_int
    lib.hid_send_feature_report.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
    lib.hid_get_feature_report.restype = ctypes.c_int
    lib.hid_get_feature_report.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
    lib.hid_error.restype = ctypes.c_wchar_p
    lib.hid_error.argtypes = [ctypes.c_void_p]
    lib.hid_init()
    return lib


def enumerate_valve(lib):
    head = lib.hid_enumerate(VID, 0)
    out = []
    p = head
    while p:
        d = p.contents
        out.append(dict(path=d.path, pid=d.product_id, serial=d.serial_number or "",
                        product=d.product_string or "", iface=d.interface_number))
        p = d.next
    if head:
        lib.hid_free_enumeration(head)
    return sorted(out, key=lambda d: (d["pid"], d["serial"], d["iface"]))


def s16(b, i):
    return int.from_bytes(b[i:i + 2], "little", signed=True)


def u16(b, i):
    return int.from_bytes(b[i:i + 2], "little")


def u32(b, i):
    return int.from_bytes(b[i:i + 4], "little")


def decode(data):
    """Returns (kind, text, light_events) for one report."""
    rid = data[0]
    if rid == 0x20 and len(data) >= 52:
        parts = []
        for k in range(3):
            o = 1 + 17 * k
            acc = [s16(data, o + 2 * j) for j in range(3)]
            gyr = [s16(data, o + 6 + 2 * j) for j in range(3)]
            parts.append("t=%u seq=%d acc=%s gyr=%s" % (u32(data, o + 12), data[o + 16], acc, gyr))
        return "imu", "IMU  " + " | ".join(parts[:1]), []
    if rid == 0x25:
        evs = []
        for k in range(9):
            o = 1 + 7 * k
            if o + 7 > len(data) or data[o] > 0xFD:
                continue
            evs.append((data[o], u16(data, o + 1), u32(data, o + 3)))
        txt = "LH1  " + "  ".join("s%02d %6.1fus @%u" % (s, l / TICKS_PER_US, t) for s, l, t in evs)
        return "light1", txt, evs
    if rid == 0x27:
        ents = []
        for k in range(4):
            o = 1 + 13 * k
            if o + 13 > len(data) or data[o] == 0xFF:
                break
            ents.append("s%02d%s t=%u d=%08x m=%08x" % (data[o] & 0x7F, "*" if data[o] & 0x80 else " ",
                                                        u32(data, o + 1), u32(data, o + 5), u32(data, o + 9)))
        return "light2", "LH2  " + "  ".join(ents), []
    return "other", "0x%02x " % rid + data[1:].hex(" "), []


class Reader(threading.Thread):
    def __init__(self, lib, dev, q):
        super().__init__(daemon=True)
        self.lib, self.dev, self.q = lib, dev, q
        self.handle = lib.hid_open_path(dev["path"])
        self.count = 0
        self.stop = False

    def run(self):
        if not self.handle:
            return
        buf = ctypes.create_string_buffer(256)
        while not self.stop:
            n = self.lib.hid_read_timeout(self.handle, buf, 256, 200)
            if n < 0:
                self.q.put(("data", time.time(), self.dev, None))
                break
            if n > 0:
                self.count += 1
                self.q.put(("data", time.time(), self.dev, bytes(buf.raw[:n])))
        self.lib.hid_close(self.handle)


class HidWorker(threading.Thread):
    """Owns every hidapi call except the readers' reads.

    On macOS hidapi spins the calling thread's CFRunLoop while enumerating and
    opening; on Tk's main thread that re-enters Tk without the GIL and aborts.
    """

    def __init__(self, q):
        super().__init__(daemon=True)
        self.q = q
        self.cmds = queue.Queue()
        self.readers = []

    def say(self, text):
        self.q.put(("say", text))

    def run(self):
        self.lib = load_hidapi()
        while True:
            cmd, *args = self.cmds.get()
            getattr(self, cmd)(*args)

    def connect(self, auto):
        for rd in self.readers:
            rd.stop = True
        for rd in self.readers:
            rd.join(1.0)
        self.readers = []
        devs = enumerate_valve(self.lib)
        for d in devs:
            rd = Reader(self.lib, d, self.q)
            self.readers.append(rd)
            self.say("open %04x:%04x if%d %s %s -> %s" % (VID, d["pid"], d["iface"], d["product"], d["serial"],
                                                         "ok" if rd.handle else "FAILED"))
            rd.start()
        if not devs:
            self.say("no Valve HID devices found")
        elif auto:
            # The tracker streams nothing until it gets the lightcap mode report.
            # The first one after idle makes it reset and re-enumerate; the
            # reconnect after that sends it again, which starts the stream.
            time.sleep(0.3)
            self.send(bytes([0x04, 0x00, 0, 0, 0]))
            self.send(bytes([0x07, 0x03, 0, 0, 0]))

    def target(self):
        """Interface 0 of the first tracker, else any open interface."""
        live = [rd for rd in self.readers if rd.handle and not rd.stop and rd.is_alive()]
        for rd in live:
            if rd.dev["iface"] <= 0 and rd.dev["pid"] != 0x2101:
                return rd
        return live[0] if live else None

    def send(self, data):
        rd = self.target()
        if not rd:
            return self.say("no device to send to")
        n = self.lib.hid_send_feature_report(rd.handle, data, len(data))
        self.say("send feature to %04x if%d: %s -> %d %s" % (rd.dev["pid"], rd.dev["iface"], data[:8].hex(" "), n,
                                                             self.lib.hid_error(rd.handle) if n < 0 else ""))

    def get(self, rid):
        rd = self.target()
        if not rd:
            return self.say("no device to read from")
        buf = ctypes.create_string_buffer(256)
        buf[0] = rid
        n = self.lib.hid_get_feature_report(rd.handle, buf, 256)
        self.say("get feature 0x%02x -> %d: %s" % (rid, n, buf.raw[:max(n, 0)].hex(" ")))


class App:
    def __init__(self, root, log_path):
        self.root = root
        self.q = queue.Queue()
        self.hid = HidWorker(self.q)
        self.hid.start()
        self.log = open(log_path, "a", buffering=1) if log_path else None
        self.paused = False
        self.recent = collections.deque()  # (wall time, sensor, length ticks)
        self.rate_hist = {}
        self.imu_every = tk.IntVar(value=50)
        self.show = {k: tk.BooleanVar(value=True) for k in ("imu", "light1", "light2", "other")}
        self.imu_n = 0
        self.auto = tk.BooleanVar(value=True)
        self.reconnect_pending = False
        self.build()
        self.connect()
        self.root.after(50, self.pump)
        self.root.after(1000, self.stats)

    def build(self):
        r = self.root
        r.title("Vive raw reports")
        top = ttk.Frame(r)
        top.pack(fill="x", padx=6, pady=4)
        ttk.Button(top, text="Reconnect", command=self.connect).pack(side="left")
        ttk.Checkbutton(top, text="auto-start", variable=self.auto).pack(side="left")
        ttk.Button(top, text="Pause", command=self.toggle_pause).pack(side="left")
        ttk.Button(top, text="Clear", command=lambda: self.text.delete("1.0", "end")).pack(side="left")
        ttk.Label(top, text="  mode 0x04:").pack(side="left")
        for label, mode in (("LH1 (0)", 0), ("LH2 (1)", 1), ("raw2 (3)", 3)):
            ttk.Button(top, text=label, command=lambda m=mode: self.send_feature(bytes([0x04, m, 0, 0, 0]))).pack(side="left")
        ttk.Button(top, text="verbosity 07 03",
                   command=lambda: self.send_feature(bytes([0x07, 0x03, 0, 0, 0]))).pack(side="left")
        row2 = ttk.Frame(r)
        row2.pack(fill="x", padx=6)
        for k in self.show:
            ttk.Checkbutton(row2, text=k, variable=self.show[k]).pack(side="left")
        ttk.Label(row2, text="  show every Nth IMU:").pack(side="left")
        ttk.Spinbox(row2, from_=1, to=1000, width=5, textvariable=self.imu_every).pack(side="left")
        ttk.Label(row2, text="   get feature id (hex):").pack(side="left")
        self.get_id = ttk.Entry(row2, width=4)
        self.get_id.pack(side="left")
        ttk.Button(row2, text="Get", command=self.get_feature).pack(side="left")
        ttk.Label(row2, text="  send feature (hex):").pack(side="left")
        self.send_hex = ttk.Entry(row2, width=30)
        self.send_hex.pack(side="left")
        ttk.Button(row2, text="Send", command=lambda: self.send_feature(bytes.fromhex(self.send_hex.get()))).pack(side="left")

        mid = ttk.Panedwindow(r, orient="horizontal")
        mid.pack(fill="both", expand=True, padx=6, pady=4)
        left = ttk.Frame(mid)
        self.text = tk.Text(left, font=("Menlo", 11), wrap="none", height=40, width=130)
        sb = ttk.Scrollbar(left, command=self.text.yview)
        self.text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.text.pack(fill="both", expand=True)
        mid.add(left, weight=3)
        self.statbox = tk.Text(mid, font=("Menlo", 11), width=52)
        mid.add(self.statbox, weight=1)

    def connect(self):
        self.reconnect_pending = False
        self.last_data = time.time()
        self.hid.cmds.put(("connect", self.auto.get()))

    def schedule_reconnect(self):
        if not self.reconnect_pending:
            self.reconnect_pending = True
            self.q.put(("say", "device dropped; reconnecting in 3 s"))
            self.root.after(3000, self.connect)

    def send_feature(self, data):
        self.hid.cmds.put(("send", data))

    def get_feature(self):
        self.hid.cmds.put(("get", int(self.get_id.get() or "0", 16)))

    def toggle_pause(self):
        self.paused = not self.paused

    def say(self, line):
        self.text.insert("end", line + "\n")
        self.text.see("end")

    def pump(self):
        lines = []
        now = time.time()
        while True:
            try:
                item = self.q.get_nowait()
            except queue.Empty:
                break
            if item[0] == "say":
                lines.append(item[1])
                if self.log:
                    self.log.write(json.dumps({"t": round(time.time(), 4), "say": item[1]}) + "\n")
                continue
            _, t, dev, data = item
            self.last_data = t
            if data is None:
                lines.append("read error on %04x if%d (device gone?)" % (dev["pid"], dev["iface"]))
                self.schedule_reconnect()
                continue
            kind, txt, evs = decode(data)
            for s, l, _ in evs:
                self.recent.append((t, s, l))
            if self.log:
                self.log.write(json.dumps({"t": round(t, 4), "pid": dev["pid"], "if": dev["iface"],
                                           "hex": data.hex()}) + "\n")
            if self.paused or not self.show[kind].get():
                continue
            if kind == "imu":
                self.imu_n += 1
                if self.imu_n % max(1, self.imu_every.get()):
                    continue
            lines.append("%.3f %04x if%d %s" % (t % 1000, dev["pid"], dev["iface"], txt))
        if lines:
            self.text.insert("end", "\n".join(lines[-400:]) + "\n")
            n = int(self.text.index("end-1c").split(".")[0])
            if n > 5000:
                self.text.delete("1.0", "%d.0" % (n - 5000))
            self.text.see("end")
        while self.recent and self.recent[0][0] < now - 1.0:
            self.recent.popleft()
        self.root.after(50, self.pump)

    def stats(self):
        # After the tracker resets, macOS sometimes just stops delivering reads
        # instead of failing them; treat 4 s of silence as a drop.
        if self.auto.get() and not self.paused and time.time() - self.last_data > 4:
            self.schedule_reconnect()
        out = ["Interfaces (packets/s):"]
        for rd in list(self.hid.readers):
            prev = self.rate_hist.get(id(rd), 0)
            out.append("  %04x if%d %-8s %6d/s" % (rd.dev["pid"], rd.dev["iface"],
                                                   "ok" if rd.handle else "FAILED", rd.count - prev))
            self.rate_hist[id(rd)] = rd.count
        ev = list(self.recent)
        out.append("")
        out.append("LH1 light, last 1 s: %d pulses" % len(ev))
        sync = [e for e in ev if e[2] / TICKS_PER_US >= SYNC_MIN_US]
        out.append("  sync-length (>= %.0f us): %d" % (SYNC_MIN_US, len(sync)))
        out.append("  sensors seeing sync: %d" % len({s for _, s, _ in sync}))
        out.append("")
        out.append("Hits per sensor:")
        for s, n in sorted(collections.Counter(s for _, s, _ in ev).items()):
            ls = [l / TICKS_PER_US for _, ss, l in ev if ss == s]
            out.append("  s%02d %5d  width %5.1f-%5.1f us" % (s, n, min(ls), max(ls)))
        out.append("")
        out.append("Pulse width histogram (us):")
        bins = collections.Counter(min(int(l / TICKS_PER_US // 10) * 10, 200) for _, _, l in ev)
        for b in sorted(bins):
            out.append("  %3d%s %5d %s" % (b, "+" if b == 200 else " ", bins[b], "#" * min(40, bins[b] // 5 + 1)))
        self.statbox.delete("1.0", "end")
        self.statbox.insert("end", "\n".join(out))
        self.root.after(1000, self.stats)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="append every raw report to this JSONL file")
    a = ap.parse_args()
    root = tk.Tk()
    App(root, a.log)
    root.mainloop()


if __name__ == "__main__":
    main()
