"""A flat tracker head, glued straight onto a laser-cut part: the Head's top plate,
stabilising pin and pogo-slot key, without its taper, core or tube socket.

Run inside FreeCAD with TruthRig open (sizes come from its Params sheet; the
document isn't changed):

    exec(open("hardware/truth_rig/head_flat.py").read())

Writes stl/HeadFlat.stl (bottom down, on z = 0: the pin and key point up, so it
prints without supports) and dxf/HeadFlat.dxf, the laser-cut part's side: the
screw hole, and the flange's outline to engrave or score as a glue guide, in the
head frame (mm; origin on the screw axis, y towards the stabilising pin).

The same 1/4"-20 x 0.6" cap screw as the Head goes up through the laser-cut part
and the flange into the tracker, driven from below. So the screw clamps the
tracker, the flange and the laser-cut part together (the glue only keeps the
flange in place when the tracker is off), and the flange's thickness is whatever
leaves `engage` of thread for the tracker: t_screw - STOCK_T.

Glue-up: screw the flange to the laser-cut part through its hole (a tracker on
top, or a nut, holds it), line the flange up with the engraved outline, glue, let
it set.
"""

import os

import FreeCAD as App
import Mesh
import Part

STOCK_T = None  # mm, the laser-cut part's thickness; None: the plate's (plate_t)
HOLE_D = None  # mm, the laser-cut screw hole; None: the Head's clearance (screw_clear_d)


def head_flat(stock_t=STOCK_T, hole_d=HOLE_D):
    doc = App.getDocument("TruthRig")
    s = doc.getObject("Params")
    p = lambda alias: float(s.get(alias))  # noqa: E731
    stock_t = p("plate_t") if stock_t is None else stock_t
    hole_d = p("screw_clear_d") if hole_d is None else hole_d
    t = p("t_screw") - stock_t
    if t < p("top_t"):
        raise ValueError("flange %.2f mm < top_t %.2f mm: stock %.2f mm is too thick for a %.1f mm screw; "
                         "use a shorter screw (bolt_len)" % (t, p("top_t"), stock_t, p("bolt_len")))
    w, y0, y1 = p("mount_w"), p("mount_y0"), p("mount_y1")

    # Head frame: datum A (the tracker's mount face) on z = 0, the flange below it.
    shape = Part.makeBox(w, y1 - y0, t, App.Vector(-w / 2, y0, -t))
    shape = shape.fuse(Part.makeCylinder(p("pin_d") / 2, p("pin_h") + 0.01, App.Vector(0, p("pin_off"), -0.01)))
    kx = p("pogo_half") - p("key_clear_l")
    kr = p("pogo_w") / 2 - p("key_clear_w")
    ky = -p("pogo_off")
    kh = p("key_h") + 0.01
    shape = shape.fuse(Part.makeBox(2 * kx, 2 * kr, kh, App.Vector(-kx, ky - kr, -0.01)))
    for x in (-kx, kx):
        shape = shape.fuse(Part.makeCylinder(kr, kh, App.Vector(x, ky, -0.01)))
    shape = shape.cut(Part.makeCylinder(p("screw_clear_d") / 2, t + 0.2, App.Vector(0, 0, -t - 0.1)))
    shape = shape.removeSplitter()
    if not shape.isValid():
        raise RuntimeError("invalid shape")

    out_dir = os.path.dirname(doc.FileName)
    stl = os.path.join(out_dir, "stl", "HeadFlat.stl")
    os.makedirs(os.path.dirname(stl), exist_ok=True)
    printed = shape.copy()
    printed.translate(App.Vector(0, 0, t))
    Mesh.Mesh(printed.tessellate(0.05)).write(stl)

    dxf = os.path.join(out_dir, "dxf", "HeadFlat.dxf")
    os.makedirs(os.path.dirname(dxf), exist_ok=True)
    write_dxf(dxf, [(-w / 2, y0), (w / 2, y0), (w / 2, y1), (-w / 2, y1)], hole_d / 2)

    print("wrote %s: flange %.0f x %.0f x %.2f mm, %.1f cm^3; glue face %.0f mm^2" % (
        stl, w, y1 - y0, t, shape.Volume / 1000, w * (y1 - y0) - 3.14159 * (p("screw_clear_d") / 2) ** 2))
    print("wrote %s: %.2f mm screw hole, outline (engrave); for %.2f mm stock, %.1f mm of thread in the tracker"
          % (dxf, hole_d, stock_t, p("engage")))
    return shape


def write_dxf(path, outline, hole_r):
    """A minimal R12 DXF in mm: the outline (layer ENGRAVE) and the hole (layer CUT)."""
    e = []
    for (xa, ya), (xb, yb) in zip(outline, outline[1:] + outline[:1]):
        e += ["0", "LINE", "8", "ENGRAVE", "10", "%.4f" % xa, "20", "%.4f" % ya, "30", "0",
              "11", "%.4f" % xb, "21", "%.4f" % yb, "31", "0"]
    e += ["0", "CIRCLE", "8", "CUT", "10", "0", "20", "0", "30", "0", "40", "%.4f" % hole_r]
    with open(path, "w") as f:
        f.write("\n".join(["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4", "0", "ENDSEC",
                           "0", "SECTION", "2", "ENTITIES"] + e + ["0", "ENDSEC", "0", "EOF"]) + "\n")


head_flat()
