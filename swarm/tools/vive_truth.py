"""Ground-truth poses from Vive Trackers, without SteamVR.

Reads 6-DoF tracker poses through libsurvive (an open-source Lighthouse
stack: https://github.com/collabora/libsurvive) and appends them to a JSONL
file in the capture record shape, stamped with host wall-clock time ("ht",
the clock swarm_view.py stamps node records with), for truth_eval.py:

  {"ht": 1793000000.1234, "t": 12.3456, "type": "truth", "dev": "T20",
   "node": 63124, "p": [x, y, z], "q": [w, x, y, z]}

"t" is libsurvive's own clock (s), "dev" the tracker's libsurvive codename
(or serial), "node" the swarm node it is strapped to (--map) or null, p in m
and q (scalar first) in libsurvive's world frame. Base stations (Lighthouse
1.0 or 2.0) are required; trackers connect over USB or a paired dongle.

Sources:
  # libsurvive's Python bindings (pip install pysurvive, or built from source)
  python3 swarm/tools/vive_truth.py --out captures/truth.jsonl \\
      --map T20=63124 --map T21=7408 --map T22=49736
  # survive-cli as a subprocess (no Python bindings needed); extra arguments
  # after -- go to survive-cli, e.g. a libsurvive recording to play back:
  python3 swarm/tools/vive_truth.py --source cli --out truth.jsonl -- \\
      --playback session.rec.gz
  # trackers on another machine (e.g. Linux): it sends, this one receives
  python3 swarm/tools/vive_truth.py --udp-out 192.168.1.20:9180   # there
  python3 swarm/tools/vive_truth.py --udp-in 9180 --out truth.jsonl # here

Run the first time with every tracker in view and still: libsurvive
locates the base stations (~10 s) and saves them to its config.json. Close
SteamVR if it's installed; both can't own the trackers.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time


def parse_survive_line(line):
    """(survive_t, codename, p, q_wxyz) from one libsurvive recording line
    ("<t> <codename> POSE x y z qw qx qy qz"), or None for anything else."""
    parts = line.split()
    if len(parts) != 10 or parts[2] != "POSE":
        return None
    try:
        vals = [float(v) for v in parts[3:]]
        t = float(parts[0])
    except ValueError:
        return None
    return t, parts[1], vals[0:3], vals[3:7]


def parse_map(items):
    """{"T20": 63124, ...} from ["T20=63124", ...]; ids may be hex (0xF694
    or a bare F694, as the viewer prints them)."""
    out = {}
    for item in items:
        dev, _, node = item.partition("=")
        if not dev or not node:
            raise ValueError("expected DEV=NODE, got %r" % item)
        node = node.strip()
        if node.lower().startswith("0x"):
            out[dev.strip()] = int(node, 16)
        else:
            try:
                out[dev.strip()] = int(node)
            except ValueError:
                out[dev.strip()] = int(node, 16)
    return out


class Recorder:
    """Decimates each tracker to `rate` Hz and writes truth records."""

    def __init__(self, out=None, udp=None, node_map=None, rate=50.0, clock=time.time):
        self.out = open(out, "a") if out else None
        self.udp = udp
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if udp else None
        self.node_map = node_map or {}
        self.period = 1.0 / rate if rate > 0 else 0.0
        self.clock = clock
        self.last = {}  # dev -> survive t of the last record written
        self.count = {}

    def pose(self, t, dev, p, q, serial=None):
        if self.period and t - self.last.get(dev, -1e9) < self.period:
            return None
        self.last[dev] = t
        node = self.node_map.get(dev, self.node_map.get(serial) if serial else None)
        rec = {"ht": round(self.clock(), 4), "t": round(t, 4), "type": "truth", "dev": dev,
               "node": node, "p": [round(v, 5) for v in p], "q": [round(v, 6) for v in q]}
        self.write(rec)
        return rec

    def write(self, rec):
        line = json.dumps(rec, separators=(",", ":"))
        if self.out:
            self.out.write(line + "\n")
            self.out.flush()
        if self.sock:
            self.sock.sendto(line.encode(), self.udp)
        self.count[rec["dev"]] = self.count.get(rec["dev"], 0) + 1

    def close(self):
        if self.out:
            self.out.close()


def run_pysurvive(rec, argv):
    import pysurvive  # libsurvive's bindings

    ctx = pysurvive.SimpleContext([sys.argv[0]] + argv)
    for obj in ctx.Objects():
        print("tracking", str(obj.Name(), "utf-8"), flush=True)
    while ctx.Running():
        updated = ctx.NextUpdated()
        if not updated:
            continue
        pose, t = updated.Pose()
        rec.pose(t, str(updated.Name(), "utf-8"), list(pose.Pos), list(pose.Rot))


def run_cli(rec, argv, binary="survive-cli"):
    """survive-cli dumping its recording stream to stdout; only POSE lines
    are kept (IMU and light data are switched off where the flags allow)."""
    cmd = [binary, "--record-stdout", "--record-rawlight", "0", "--record-imu", "0",
           "--record-angle", "0"] + argv
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    try:
        for line in proc.stdout:
            parsed = parse_survive_line(line)
            if parsed:
                rec.pose(*parsed)
    finally:
        proc.terminate()


def run_udp_in(rec, port):
    """Records from a remote vive_truth.py --udp-out, re-stamped with this
    machine's clock on receipt (the sender's stamp is kept as "ht_src")."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    while True:
        data, _ = sock.recvfrom(65536)
        try:
            r = json.loads(data)
        except ValueError:
            continue
        if not isinstance(r, dict) or r.get("type") != "truth":
            continue
        r["ht_src"], r["ht"] = r.get("ht"), round(rec.clock(), 4)
        if r.get("node") is None:
            r["node"] = rec.node_map.get(r.get("dev"))
        rec.write(r)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["pysurvive", "cli"], default="pysurvive")
    ap.add_argument("--survive-cli", default="survive-cli", help="survive-cli binary (--source cli)")
    ap.add_argument("--udp-in", type=int, metavar="PORT", help="receive from a remote sender instead")
    ap.add_argument("--out", help="append truth records here")
    ap.add_argument("--udp-out", metavar="HOST:PORT", help="also send each record here")
    ap.add_argument("--map", action="append", default=[], metavar="DEV=NODE",
                    help="tracker codename or serial -> swarm node id (decimal or hex)")
    ap.add_argument("--rate", type=float, default=50.0, help="per-tracker record rate, Hz (0 = all)")
    ap.add_argument("survive_args", nargs="*", help="after --: passed to libsurvive")
    args = ap.parse_args()
    udp = None
    if args.udp_out:
        host, _, port = args.udp_out.rpartition(":")
        udp = (host, int(port))
    if not args.out and not udp:
        ap.error("nowhere to write: give --out and/or --udp-out")
    rec = Recorder(args.out, udp, parse_map(args.map), args.rate)
    last_report = time.time()
    try:
        if args.udp_in:
            run_udp_in(rec, args.udp_in)
        elif args.source == "cli":
            run_cli(rec, args.survive_args, args.survive_cli)
        else:
            run_pysurvive(rec, args.survive_args)
    except KeyboardInterrupt:
        pass
    finally:
        rec.close()
        print("records per tracker:", rec.count, "(%.0f s)" % (time.time() - last_report))


if __name__ == "__main__":
    main()
