"""Replays a recording through lhd (f64) and lhd_f32, compares poses/calibration."""
import json, math, subprocess, sys, time
B = "/private/tmp/claude-501/-Users-kevin-Projects-git-checkouts-indoor-localization/ea7d830b-6f55-49b3-94bb-12e39a7b266a/scratchpad/bin/"
rec = sys.argv[1]
speed = sys.argv[2] if len(sys.argv) > 2 else "200"

def run(exe):
    t0 = time.time()
    out = subprocess.run([B + exe, "--replay", rec, "--speed", speed], stdin=subprocess.DEVNULL,
                         capture_output=True, text=True, timeout=1800).stdout
    poses, ev = {}, []
    for l in out.splitlines():
        try: v = json.loads(l)
        except Exception: continue
        t = v.get("type")
        if t == "pose": poses[(v["serial"], round(v["t"], 6))] = (v["p"], v["q"])
        elif t in ("calibrated", "tracking", "station"): ev.append(v)
    return poses, ev, time.time() - t0

EA = sys.argv[3] if len(sys.argv) > 3 else "lhd"
EB = sys.argv[4] if len(sys.argv) > 4 else "lhd_f32"
a, ea, ta = run(EA)
b, eb, tb = run(EB)
print(f"{rec.split('/')[-1]}: {EA} vs {EB}: f64 {len(a)} poses ({ta:.0f}s), f32 {len(b)} poses ({tb:.0f}s)")
for name, ev in (("f64", ea), ("f32", eb)):
    for v in ev:
        if v["type"] == "calibrated":
            print(f"  {name} calibrated {v['reason']}: " + "; ".join(
                f"{s['id']} p=({s['p'][0]:.4f},{s['p'][1]:.4f},{s['p'][2]:.4f}) rms {s['rms']*1e6:.0f}urad" for s in v["stations"]))
        elif v["type"] == "tracking":
            print(f"  {name} tracking {v.get('state')} {v.get('reason','')}")
common = sorted(set(a) & set(b))
if common:
    dp = sorted(math.dist(a[k][0], b[k][0]) for k in common)
    dq = sorted(2 * math.degrees(math.acos(min(1, abs(sum(x * y for x, y in zip(a[k][1], b[k][1])))))) for k in common)
    q = lambda v, f: v[int(f * (len(v) - 1))]
    print(f"  common {len(common)}: |dp| median {q(dp,.5)*1e3:.3f} mm, p95 {q(dp,.95)*1e3:.3f} mm, max {dp[-1]*1e3:.3f} mm; "
          f"rot median {q(dq,.5):.4f} deg, p95 {q(dq,.95):.4f}, max {dq[-1]:.4f}")
