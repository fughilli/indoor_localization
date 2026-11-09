"""A printed stand-in for the stylus's 1/2" steel ball, until the balls arrive.

Run inside FreeCAD with TruthRig open (sizes come from its Params sheet):

    exec(open("hardware/truth_rig/ball_standin.py").read())

Writes stl/BallStandIn.stl, peg down, on z = 0. The sphere is ball_d, so it sits
on the tube's bore rim exactly as the steel ball does, and the nominal tip
doesn't change. A peg (tube_id - 0.2, a slip fit) goes into the bore; glue it.
A 45 deg cone, tangent to the sphere, joins the peg so it prints without
supports. The cone puts the bore-rim contact about 0.06 mm lower than on a
true sphere: below print accuracy, and the pivot calibration measures the tip
anyway.
"""

import math
import os

import FreeCAD as App
import Mesh
import Part

PEG_LEN = 8.0  # mm into the tube's bore
PEG_CLEAR = 0.2  # mm on diameter: a slip fit (0.15 presses into the tube)


def ball_standin():
    doc = App.getDocument("TruthRig")
    s = doc.getObject("Params")
    r = float(s.get("ball_d")) / 2
    rp = (float(s.get("tube_id")) - PEG_CLEAR) / 2
    # Sphere centre at the origin; the cone is tangent to it at 45 deg below.
    zt = -r * math.sin(math.radians(45))  # tangent circle: radius -zt, height zt
    zc = zt - (-zt - rp)  # where the 45 deg cone meets the peg
    rim = -math.sqrt(r * r - (float(s.get("tube_id")) / 2) ** 2)  # steel ball's bore-rim contact height
    shape = Part.makeSphere(r)
    shape = shape.fuse(Part.makeCone(rp, -zt, zt - zc, App.Vector(0, 0, zc)))
    shape = shape.fuse(Part.makeCylinder(rp, PEG_LEN, App.Vector(0, 0, zc - PEG_LEN + 0.01)))
    shape = shape.removeSplitter()
    shape.translate(App.Vector(0, 0, -shape.BoundBox.ZMin))
    out_dir = os.path.join(os.path.dirname(doc.FileName), "stl")
    os.makedirs(out_dir, exist_ok=True)
    p = os.path.join(out_dir, "BallStandIn.stl")
    Mesh.Mesh(shape.tessellate(0.02)).write(p)
    shift = (zc + (float(s.get("tube_id")) / 2 - rp)) - rim  # cone height at the bore rim vs the sphere's
    print("wrote %s: sphere %.2f, peg %.2f x %.1f, height %.2f mm, %.0f mm^3; rim contact %.3f mm lower than a sphere"
          % (p, 2 * r, 2 * rp, PEG_LEN, shape.BoundBox.ZLength, shape.Volume, -shift))
    return shape


ball_standin()
