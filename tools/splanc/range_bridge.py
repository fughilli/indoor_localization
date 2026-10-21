"""Drive a Splanc light's pulse from the live UWB range.

Subscribes to the range viewer's event stream (swarm/tools/range_view.py,
so the serial port stays with the viewer), filters the range (0.3 s median to
drop multipath spikes, then a One Euro filter: steadier at rest, little lag
while moving), and maps it onto the `rate` and `hue` uniforms of
range_pulse.fx running on the Splanc player:

  near (<= --near m): fast, red      far (>= --far m): slow, blue

The rate is interpolated geometrically (equal distance steps feel like equal
tempo changes); the hue linearly. When the responder goes silent the light
drops to a slow, desaturated breath until ranges return.

  python3 tools/splanc/range_bridge.py --host 192.168.68.70 --deploy
  python3 tools/splanc/range_bridge.py --dry-run          # print the mapping only

Talks to the player through the splanc-agent hub (https://splanc.io/AGENTS.md)
run as one long-lived MCP server on stdio. --deploy uploads range_pulse.fx
first; Ctrl-C rolls back to what was running before unless --keep.
"""

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
import urllib.request

AGENT = ["npx", "-y", "https://splanc.io/agent/splanc-agent.tgz"]
EFFECT_ID = "range_pulse"
EFFECT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "range_pulse.fx")

HUE_NEAR, HUE_FAR = 0.0, 0.66      # red -> blue
SILENT_AFTER_S = 2.0               # no range for this long = responder gone
SILENT_LOOK = {"rate": 0.25, "sat": 0.0}
RETRY_S = 3.0                      # back-off after a failed call to the player


class OneEuro:
    """One Euro filter (Casiez et al., CHI 2012): a low-pass whose cutoff
    rises with the signal's speed, trading jitter at rest for low lag in
    motion. min_cutoff (Hz) sets smoothing at rest; beta how fast the
    cutoff opens up with speed (m/s)."""

    def __init__(self, min_cutoff=1.0, beta=3.0, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.t = self.x = self.dx = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, t, x):
        if self.t is None or t <= self.t:
            self.t, self.x, self.dx = t, x, 0.0
            return x
        dt = t - self.t
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * (x - self.x) / dt + (1 - a_d) * self.dx
        a = self._alpha(self.min_cutoff + self.beta * abs(self.dx), dt)
        self.x = a * x + (1 - a) * self.x
        self.t = t
        return self.x


class RangeFilter:
    """Median over the last `window_s` seconds (rejects multipath outliers),
    then One Euro. Tuned in range_bridge_test.py on a simulated walk with the
    mule pair's measured noise. A time window beats a short sample-count
    median: once it has slid past the moment motion stopped, every sample in
    it is at the new distance, so it arrives fast instead of easing in."""

    def __init__(self, window_s=0.3, **euro):
        self.window_s = window_s
        self.recent = []
        self.euro = OneEuro(**euro)

    def __call__(self, t, r):
        self.recent = [(tt, rr) for tt, rr in self.recent if tt > t - self.window_s]
        self.recent.append((t, r))
        return self.euro(t, statistics.median(rr for _, rr in self.recent))


def map_range(d, near, far, rate_near, rate_far):
    """Distance (m) -> {"rate", "hue", "sat"} for range_pulse.fx."""
    x = min(1.0, max(0.0, (d - near) / (far - near)))
    rate = rate_near * math.pow(rate_far / rate_near, x)
    hue = HUE_NEAR + (HUE_FAR - HUE_NEAR) * x
    return {"rate": round(rate, 3), "hue": round(hue, 4), "sat": 1.0}


def changed(prev, new):
    """Worth sending? Skips sub-perceptual updates to keep traffic low."""
    if prev is None:
        return True
    return (abs(new["hue"] - prev["hue"]) > 0.004
            or abs(new["rate"] - prev["rate"]) > 0.02 * prev["rate"]
            or new["sat"] != prev["sat"])


class Agent:
    """Minimal MCP client for the splanc-agent hub (JSON-RPC over stdio)."""

    def __init__(self, host):
        cmd = AGENT + (["--host", host] if host else [])
        # Own process group: npx runs the hub as a grandchild, and killing
        # only npx orphans it. An orphaned hub keeps the player's control
        # connection, and the player refuses every other client meanwhile.
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=sys.stderr, text=True, bufsize=1,
                                     start_new_session=True)
        self.next_id = 1
        self.request("initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                    "clientInfo": {"name": "range_bridge", "version": "1"}})
        self._send({"jsonrpc": "2.0", "method": "notifications/initialized"})
        # As an MCP server the hub connects to players lazily; connect up front
        # so the first real call doesn't race it ("not connected").
        if host:
            self.call("add_player", {"host": host})

    def _send(self, msg):
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()

    def request(self, method, params):
        rid = self.next_id
        self.next_id += 1
        self._send({"jsonrpc": "2.0", "id": rid, "method": method, "params": params})
        while True:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("splanc-agent exited")
            msg = json.loads(line)
            if msg.get("id") == rid:
                if "error" in msg:
                    raise RuntimeError("%s: %s" % (method, msg["error"]))
                return msg["result"]

    def call(self, tool, args):
        result = self.request("tools/call", {"name": tool, "arguments": args})
        text = "".join(c.get("text", "") for c in result.get("content", []))
        if result.get("isError"):
            raise RuntimeError("%s: %s" % (tool, text))
        return text

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            pass
        for sig in (signal.SIGTERM, signal.SIGKILL):  # the whole group, always
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                continue


def ranges(url):
    """Yield (t, range_m) from the viewer's SSE stream; (None, None) on stats
    lines so the caller can notice silence."""
    with urllib.request.urlopen(url, timeout=30) as resp:
        for raw in resp:
            line = raw.decode("utf-8", "replace").strip()
            if not line.startswith("data: "):
                continue
            msg = json.loads(line[6:])
            if msg.get("type") == "range":
                yield msg["t"], msg["range_m"]
            elif msg.get("type") == "stats":
                yield None, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", help="Splanc player address (default: discovery)")
    ap.add_argument("--viewer", default="http://127.0.0.1:8183/events")
    ap.add_argument("--near", type=float, default=0.3, help="m: fastest / reddest")
    ap.add_argument("--far", type=float, default=5.0, help="m: slowest / bluest")
    ap.add_argument("--rate-near", type=float, default=4.0, help="pulses/s when near")
    ap.add_argument("--rate-far", type=float, default=0.4, help="pulses/s when far")
    ap.add_argument("--update-hz", type=float, default=10.0)
    ap.add_argument("--window", type=float, default=0.3,
                    help="s: spike-rejecting median window")
    ap.add_argument("--min-cutoff", type=float, default=1.0,
                    help="Hz: smoothing at rest (lower = steadier, laggier)")
    ap.add_argument("--beta", type=float, default=3.0,
                    help="how fast smoothing relaxes with speed (higher = snappier)")
    ap.add_argument("--deploy", action="store_true", help="upload range_pulse.fx first")
    ap.add_argument("--keep", action="store_true", help="don't roll back on exit")
    ap.add_argument("--dry-run", action="store_true", help="print values, no light")
    args = ap.parse_args()
    # A plain `kill` cleans up like Ctrl-C: roll back, stop the hub.
    def on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    def deploy():
        with open(EFFECT_FILE) as f:
            print(agent.call("deploy_effect", {"effect_id": EFFECT_ID, "source": f.read()}),
                  flush=True)

    agent = None
    if not args.dry_run:
        agent = Agent(args.host)
        if args.deploy:
            for attempt in range(3):
                try:
                    deploy()
                    break
                except RuntimeError as e:
                    if attempt == 2:
                        raise
                    print("deploy failed (%s); retrying" % e, flush=True)
                    time.sleep(2)

    filt = RangeFilter(window_s=args.window, min_cutoff=args.min_cutoff, beta=args.beta)
    d, sent, last_send, last_range = None, None, 0.0, time.time()
    retry_at = 0.0
    try:
        for t, r in ranges(args.viewer):
            now = time.time()
            if t is not None:
                d = filt(t, r)
                last_range = now
            if now - last_send < 1.0 / args.update_hz or now < retry_at:
                continue
            if now - last_range > SILENT_AFTER_S:
                values, label = dict(sent or map_range(args.far, args.near, args.far,
                                                       args.rate_near, args.rate_far),
                                     **SILENT_LOOK), "no signal"
            elif d is not None:
                values = map_range(d, args.near, args.far, args.rate_near, args.rate_far)
                label = "%.2f m" % d
            else:
                continue
            if not changed(sent, values):
                continue
            last_send = now
            print("%-10s rate=%.2f/s hue=%.3f sat=%.0f" % (label, values["rate"],
                                                         values["hue"], values["sat"]), flush=True)
            if agent:
                # The player can reboot or drop its effect (seen on 1.6.0 under
                # memory pressure): log, back off, recover, keep streaming.
                try:
                    agent.call("set_uniforms", {"values": values})
                except RuntimeError as e:
                    print("player error: %s; retrying in %.0f s" % (e, RETRY_S), flush=True)
                    retry_at = now + RETRY_S
                    sent = None  # resend the full look once it's back
                    try:
                        if "exited" in str(e):
                            agent.close()
                            agent = Agent(args.host)
                        elif "no effect is loaded" in str(e) and args.deploy:
                            deploy()
                    except (RuntimeError, OSError) as e2:
                        print("recovery failed: %s" % e2, flush=True)
                    continue
            sent = values
    except KeyboardInterrupt:
        pass
    finally:
        if agent:
            try:
                if args.deploy and not args.keep:
                    print(agent.call("rollback_effect", {}), flush=True)
            except RuntimeError as e:
                print("rollback failed: %s" % e, flush=True)
            finally:
                agent.close()


if __name__ == "__main__":
    main()
