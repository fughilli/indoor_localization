"""Write the rig geometry the survey tool and the dataset need, from TruthRig.

Run inside FreeCAD after build_rig.py (or after editing the Params sheet):

    exec(open("hardware/truth_rig/export_geometry.py").read())

Writes hardware/truth_rig/geometry.json (metres, plate frame; see build_rig.py):
  seats         the three ball centres when the stylus sits in each seat cone:
                what the survey touches, and fits the plate pose to;
  sides         seat-to-seat distances (they identify which seat is which);
  antenna       nominal antenna reference point (the phase centre is
                estimated from data);
  imu           the MPU6050 position and its yaw about plate z (nominal: the
                gyro vs tracker alignment refines the rotation);
  mag           the magnetometer position;
  mobile_mount  the mobile rig's tracker mount (datum A at the screw axis):
                nominal only, the survey measures plate -> tracker;
  stylus        nominal tip (datum A to ball centre): the pivot calibration
                measures it, this is a sanity check;
  cut_list_mm   tube lengths to cut.
With fab=True it also writes, next to the document:
  stl/   one STL per printed part design, in its print orientation, on z = 0
         (Head: print 2; SeatCone: 3 per plate + 1 for the pivot brick);
  dxf/   the laser-cut plates' outlines and holes, in plate coordinates (mm).
"""

import json
import os

import FreeCAD as App

FAB = False
STL = {"Head": "Head_Stylus", "SeatCone": "Cone_Static_1", "MastCollar": "MastCollar"}
DXF = {"PlateStatic": "0", "PlateMobile": "mob_oy"}  # plate -> its y offset in the document


def export(fab=FAB):
    doc = App.getDocument("TruthRig")
    if not doc.FileName:
        raise RuntimeError("save TruthRig (hardware/truth_rig/TruthRig.FCStd) first: files go next to it")
    out_dir = os.path.dirname(doc.FileName)
    doc.recompute()
    s = doc.getObject("Params")
    g = lambda a: float(s.get(a))  # noqa: E731
    mm = lambda v: round(v / 1000.0, 6)  # noqa: E731
    out = {
        "units": "m",
        "frame": "plate: x from the ESP32 board's front edge (the antenna points past it, x < 0) "
                 "towards the back, y from its left edge (DC jack side, facing the antenna), z up; "
                 "z = 0 on the plate top, the board's bottom copper at z = standoff_h",
        "ball_d": mm(g("ball_d")),
        "seats": {str(i): [mm(g(f"seat{i}_x")), mm(g(f"seat{i}_y")), mm(g("ball_z"))] for i in (1, 2, 3)},
        "sides": {"12": mm(g("side12")), "23": mm(g("side23")), "31": mm(g("side31"))},
        "antenna": [mm(g("ant_x")), mm(g("ant_y")), mm(g("ant_z"))],
        "antenna_note": "nominal: ant_inset behind the tab tip; the phase centre is estimated from data",
        "imu": {"p": [mm(g("imu_x")), mm(g("imu_y")), mm(g("imu_z"))], "yaw_deg": g("imu_yaw")},
        "mag": {"p": [mm(g("mag_x")), mm(g("mag_y")), mm(g("mag_z"))]},
        "mobile_mount": {"p": [mm(g("mast_x") + g("tube_off")), mm(g("mast_y")), mm(g("mount_z"))],
                         "note": "datum A at the screw axis, pin along -y (head turned 180 deg), tracker above; "
                                 "nominal: the survey measures plate -> tracker"},
        "stylus": {"tip_len": mm(g("tip_len")), "tip_off": mm(g("tube_off")),
                   "note": "ball centre tip_len below datum A, tip_off along +x from the screw axis; nominal"},
        "cut_list_mm": {"stylus_tube": round(g("stylus_tube_len"), 1), "mast_tube": round(g("mast_len"), 1)},
        "checks": {a + "_mm": g(a) for a in ("keepout_ok", "tip_ok", "ins_clear", "cbore_room", "socket_roof",
                                             "tube_wall", "taper_ok", "core_ok")},
    }
    path = os.path.join(out_dir, "geometry.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    print("wrote", path)
    if not fab:
        return out

    import Mesh
    import importDXF
    stl_dir = os.path.join(out_dir, "stl")
    os.makedirs(stl_dir, exist_ok=True)
    for old in os.listdir(stl_dir):
        if old.endswith(".stl") and old[:-4] not in STL and old not in ("BallStandIn.stl", "HeadFlat.stl"):  # ball_standin.py, head_flat.py
            os.remove(os.path.join(stl_dir, old))  # parts no longer in the design
    for fname, obj in STL.items():
        shape = doc.getObject(obj).Shape.copy()
        shape.translate(App.Vector(-shape.BoundBox.Center.x, -shape.BoundBox.Center.y, -shape.BoundBox.ZMin))
        p = os.path.join(stl_dir, fname + ".stl")
        Mesh.Mesh(shape.tessellate(0.05)).write(p)
        print("wrote", p)

    dxf_dir = os.path.join(out_dir, "dxf")
    os.makedirs(dxf_dir, exist_ok=True)
    for name, oy in DXF.items():
        shape = doc.getObject(name).Shape
        top = [f for f in shape.Faces if abs(f.BoundBox.ZMin) < 1e-6 and abs(f.BoundBox.ZMax) < 1e-6][0].copy()
        top.translate(App.Vector(0, -(g(oy) if oy != "0" else 0.0), 0))
        tmp = doc.addObject("Part::Feature", name + "_dxf")
        tmp.Shape = top
        p = os.path.join(dxf_dir, name + ".dxf")
        importDXF.export([tmp], p)
        doc.removeObject(tmp.Name)
        print("wrote", p, "(%d holes)" % (len(top.Wires) - 1))
    return out


export()
