"""Design checks for TruthRig: assembly interference, access, printability.

Run inside FreeCAD after build_rig.py:

    exec(open("hardware/truth_rig/check_rig.py").read())

Interference: the volume shared by parts that touch in the assembly (none
should overlap; mating faces touching is fine).
Screw clearance: the cap screw must clear its hole and counterbore all round,
so the pin and the key locate the tracker, not the screw.
Hex key path: a 12 mm cylinder 80 mm down from each counterbore hits nothing.
Printability: each printed part in its print orientation (all bottom-down, as
modelled), every downward-facing face that is not on the bed. Steeper than 45
deg from vertical needs support, except a planar bridge between two supports:
those are listed, and fail above BRIDGE_MAX.
Spreadsheet checks: the cells that must not go negative.
"""

import math

import FreeCAD as App
import Part

BRIDGE_MAX = 15  # mm: only blind-hole roofs should bridge
PRINTED = ["Head_Stylus", "Head_Mobile", "Cone_Static_1", "Cone_Pivot", "MastCollar"]
PAIRS = [(f"Head_{h}", f"{b}_{h}") for h in ("Stylus", "Mobile") for b in ("Screw", "Tube")] + [
    ("Tube_Stylus", "Ball_Stylus"), ("PlateMobile", "Tube_Mobile"), ("MastCollar", "Tube_Mobile"),
    ("MastCollar", "PlateMobile"), ("PlateMobile", "Head_Mobile"), ("Cone_Pivot", "Brick")] + [
    (f"Cone_{p}_{i}", other) for p in ("Static", "Mobile") for i in (1, 2, 3)
    for other in (f"Plate{p}", f"Nut_{p}", *(f"Standoff_{p}_{k}" for k in (1, 2, 3, 4)), f"{p}_ESP32")] + [
    (f"Nut_{p}", f"{p}_ESP32") for p in ("Static", "Mobile")]
# Must be >= 0 (the notes in Params give the margins wanted).
NONNEG = ["keepout_ok", "tip_ok", "ins_clear", "stylus_tube_len", "mast_len", "cbore_room", "socket_roof",
          "tube_wall", "taper_ok", "core_ok"]


def check():
    doc = App.getDocument("TruthRig")
    doc.recompute()
    s = doc.getObject("Params")
    g = lambda a: float(s.get(a))  # noqa: E731
    ok = True

    print("interference (mm^3, should be ~0):")
    for a, b in PAIRS:
        v = doc.getObject(a).Shape.common(doc.getObject(b).Shape).Volume
        ok &= v < 0.01
        if v >= 0.01:
            print(f"  {a:18s} x {b:18s} {v:8.3f}  <-- overlap")
    print(f"  {len(PAIRS)} pairs checked" + ("" if ok else ", overlaps above"))

    gap_shank = (g("screw_clear_d") - 6.35) / 2
    gap_cap = (g("cbore_d") - g("cap_d")) / 2
    loc_pin = (4.20 - g("pin_d")) / 2
    loc_key = g("key_clear_w")
    print(f"screw clearance per side: shank {gap_shank:.2f} mm, cap {gap_cap:.2f} mm "
          f"(locating: pin {loc_pin:.2f}, key {loc_key:.2f})")
    ok &= gap_shank > 2 * max(loc_pin, loc_key) and gap_cap > 2 * max(loc_pin, loc_key)

    print("hex key path (12 mm cylinder, 80 mm down from each counterbore):")
    others = [o for o in doc.Objects if o.TypeId.startswith("Part::") and App.GuiUp and o.ViewObject.Visibility
              and not o.Name.startswith(("Tracker_", "Screw_"))]
    for h in ("Stylus", "Mobile"):
        cap = doc.getObject(f"Screw_{h}").Shape.BoundBox
        key = Part.makeCylinder(6, 80, App.Vector(cap.Center.x, cap.Center.y, cap.ZMin - 80))
        hits = [o.Name for o in others if o.Name != f"Head_{h}" and key.common(o.Shape).Volume > 0.01]
        ok &= not hits
        print(f"  {h:8s} " + ("clear" if not hits else "blocked by " + ", ".join(hits)))

    print("printability (bottom-down, as modelled):")
    for name in PRINTED:
        shape = doc.getObject(name).Shape
        z0 = shape.BoundBox.ZMin
        issues, bridges = [], []
        for f in shape.Faces:
            if f.BoundBox.ZMax <= z0 + 0.01:
                continue  # on the bed
            u0, u1, v0, v1 = f.ParameterRange
            n = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2)  # outward (orientation included)
            down = -n.z
            if down <= math.cos(math.radians(45)) + 1e-6:
                continue  # wall or <= 45 deg overhang
            if f.Surface.__class__.__name__ == "Plane" and abs(n.z) > 0.999:
                bb = f.BoundBox
                span = min(bb.XLength, bb.YLength)
                bridges.append(f"{span:.1f} mm")
                if span > BRIDGE_MAX:
                    issues.append(f"bridge {span:.1f} mm at z={bb.ZMin - z0:.1f} > {BRIDGE_MAX}")
            else:
                issues.append(f"overhang {math.degrees(math.acos(min(1, down))):.0f} deg "
                              f"({f.Surface.__class__.__name__}, {f.Area:.1f} mm^2) at z={f.BoundBox.ZMin - z0:.1f}")
        ok &= not issues
        note = f" (bridges {', '.join(sorted(set(bridges)))})" if bridges else ""
        print(f"  {name:14s} {shape.Volume:7.0f} mm^3  " + ("ok" + note if not issues else "; ".join(sorted(set(issues)))))

    print("spreadsheet checks (must be >= 0):")
    for a in NONNEG:
        v = g(a)
        ok &= v >= -1e-9
        print(f"  {a:16s} {v:8.2f}{'' if v >= -1e-9 else '  <-- FAIL'}")
    print("cut list: stylus tube %.1f mm, mast tube %.1f mm" % (g("stylus_tube_len"), g("mast_len")))
    print("ALL OK" if ok else "FAILED")
    return ok


check()
