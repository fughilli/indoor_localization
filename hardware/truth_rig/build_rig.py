"""Ground-truth rig for UWB data collection, as one parametric FreeCAD document.

Printed parts:
  Head_*         tracker head (one design, for the stylus and the mobile rig):
                 a 3 mm plate the tracker sits on; the stabilising pin locates
                 it and the pogo-slot key (close across the slot, loose along
                 it) stops it turning; the screw only clamps, with clearance
                 all round its shank and cap. The plate tapers at 45 deg to a core around the 1/4"-20 cap
                 screw, which goes up from a counterbore and is driven with a
                 hex key from below; a side lobe holds the steel tube, off the
                 screw's axis so the key can reach the screw.
  Cone_*         seat cone (one design): a flanged puck with a cone for the
                 ball on top and a blind hole for a locating dowel underneath.
                 Three per baseplate; one glued to a brick is the pivot
                 ("tip") calibration fixture.
  MastCollar     keeps the mobile rig's mast square to its plate.
Laser-cut parts (DXF):
  PlateStatic    UWB board baseplate: M3 holes for the board's standoffs, one
                 dowel hole per seat cone, a 1/4" hole for a tripod.
  PlateMobile    the same plus a hole for the mast tube; a head sits on the
                 mast's top (turned 180 deg, so its screw is outboard).
Bought: steel tube (stylus shaft, mast), 1/2" steel balls, 1/4"-20 x 0.6"
socket cap screws, a 1/4" dowel (a glue-up fixture, removed after), M3
standoffs and screws, a 1/4"-20 nut per plate, epoxy.

Assembly:
  cones     push the 1/4" dowel up through the plate's seat hole into the
            cone's underside, glue the flange to the plate (keep glue off the
            dowel), let it set, pull the dowel out. Both holes are slip fits.
  nut       glue a 1/4"-20 nut over the plate's tripod hole, located by a
            1/4"-20 screw through both during glue-up.
  stylus    epoxy the tube into a head's socket and the ball onto the tube's
            other end (it centres on the bore's rim).
  mobile    the mast tube passes through the plate's hole (flush underneath)
            and the collar, both glued; a head on its top, screw outboard.
  tracker   drop the cap screw into the counterbore, pin and key into the
            tracker, drive the screw with a 5/32" hex key from below.

Printing, all without supports, every part bottom-down as modelled: every hole
is vertical, every cone opens upward, the head's underside tapers at 45 deg,
and the only bridges are blind-hole roofs.

Every dimension is an alias in the spreadsheet Params; parts are expression-
driven primitives, so editing a cell recomputes everything. Run inside FreeCAD
(it rebuilds the document TruthRig from the PARAMS table below):

    exec(open("hardware/truth_rig/build_rig.py").read())

then check_rig.py (interference, printability) and export_geometry.py (seat
centres, plate -> antenna / IMU / magnetometer, cut list, STLs, DXFs).

Frames (mm):
  plate   board-relative, as measured: x from the ESP32 board's front edge
          (the one the antenna points past; the antenna is at x < 0) towards
          the back, y from its left edge (the DC jack side, facing the antenna),
          z up; z = 0 on the plate's top. The ESP32 board's bottom copper is at
          z = standoff_h.
  head    origin on datum A (the tracker's mount face) at the screw axis, z
          towards the tracker, y towards the stabilising pin (HTC Tracker 3.0
          developer guidelines, pp. 15-18), the tube on +x.
"""

import FreeCAD as App

DOC_NAME = "TruthRig"
IN = 25.4

# (alias, value or "=formula", note). Rows whose alias starts with "#" are
# section titles. MEASURE marks values still to measure.
PARAMS = [
    ("# Steel ball and seat cones", None, ""),
    ("ball_d", 12.7, "mm, 1/2\" ball"),
    ("seat_half", 45, "deg, cone half-angle (45: prints without support)"),
    ("seat_top_ratio", 0.85, "cone mouth / ball diameter (shallow: the stylus can tilt)"),
    ("seat_top_d", "=ball_d * seat_top_ratio", "mm, cone mouth"),
    ("seat_depth", "=seat_top_d / 2 / tan(seat_half)", "mm, mouth to apex"),
    ("ball_rise", "=ball_d / 2 / sin(seat_half) - seat_depth", "mm, ball centre above the cone's top face"),
    ("seat_relief_d", 1.6, "mm, relief under the apex (printed apexes are blunt)"),
    ("seat_relief_h", 1.5, "mm"),
    ("cone_d", 22, "mm, puck body"),
    ("flange_d", 34, "mm, glue flange (to the plate, or to a brick for the pivot fixture)"),
    ("flange_t", 3, "mm"),
    ("dowel_d", 0.25 * IN, "mm, 1/4\" locating dowel (removed after the glue sets)"),
    ("dowel_hole_d", 6.5, "mm, slip fit in the printed cone"),
    ("dowel_depth", 5, "mm, dowel into the cone"),
    ("cone_h", "=dowel_depth + 1.2 + seat_relief_h + seat_depth - 0.3", "mm, puck height (1.2 mm above the dowel hole)"),
    ("ball_z", "=cone_h + ball_rise", "mm, ball centre above the plate top"),

    ("# Steel tube (stylus shaft and mobile mast)", None, ""),
    ("tube_od", 0.375 * IN, "mm, 0.375\""),
    ("tube_id", 0.310 * IN, "mm, 0.310\""),
    ("tube_clear", 0.15, "mm, printed sockets are net tube OD + this (tight fit)"),
    ("socket_depth", 15, "mm"),
    ("boss_d", "=tube_od + 8", "mm, socket boss"),
    ("tip_ok", "=ball_d - tube_id", "mm, must be > 0: the ball sits on the bore's rim"),
    ("ball_off", "=sqrt((ball_d / 2)^2 - (tube_id / 2)^2)", "mm, ball centre beyond the tube's end"),

    ("# Board (ESP32 in Arduino Uno R3 form)", None, ""),
    ("board_l", 68.6, "mm, front to back (MEASURE)"),
    ("board_w", 53.3, "mm, left to right (MEASURE)"),
    ("pcb_t", 1.6, "mm"),
    ("hole1_x", 14.0, "mm, Uno R3 hole pattern from the front-left corner (MEASURE)"),
    ("hole1_y", 2.5, "mm"),
    ("hole2_x", 15.3, "mm"),
    ("hole2_y", 50.7, "mm"),
    ("hole3_x", 66.1, "mm"),
    ("hole3_y", 7.6, "mm"),
    ("hole4_x", 66.1, "mm"),
    ("hole4_y", 35.5, "mm"),
    ("standoff_h", 8, "mm, M3 standoffs (bought)"),
    ("m3_hole_d", 3.0, "mm, laser-cut, net shape (kerf gives the clearance)"),

    ("# Antenna and sensors (measured from the ESP32 board's front/left edges and bottom copper)", None, ""),
    ("esp_z", "=standoff_h", "mm, ESP32 bottom copper in the plate frame"),
    ("ant_tip_fwd", 0.35 * IN, "mm, antenna tab tip ahead of the front edge"),
    ("ant_inset", 3, "mm, nominal reference point behind the tip (U antenna: the phase centre is estimated from data)"),
    ("ant_x", "=-ant_tip_fwd + ant_inset", "mm"),
    ("ant_y", "=board_w / 2", "mm, centred left/right"),
    ("ant_h", 0.54 * IN, "mm, above the bottom copper"),
    ("ant_z", "=esp_z + ant_h", "mm"),
    ("imu_x", 1.45 * IN, "mm, MPU6050 from the front edge"),
    ("imu_y", 0.76 * IN, "mm, from the left edge"),
    ("mag_x", 2.06 * IN, "mm, magnetometer from the front edge"),
    ("mag_y", 0.83 * IN, "mm, from the left edge"),
    ("sens_h", 1.17 * IN, "mm, both sensors above the bottom copper"),
    ("imu_z", "=esp_z + sens_h", "mm"),
    ("mag_z", "=esp_z + sens_h", "mm"),
    ("imu_yaw", 0, "deg, GY-521 X arrow from plate +x (MEASURE; data refines it)"),
    ("stack_top", "=imu_z + 3", "mm, top of the stack"),

    ("# Board stack (reference shapes only)", None, ""),
    ("shield_z", "=ant_z - 2.6 - pcb_t", "mm, shield PCB underside (approx.)"),
    ("module_l", 26, "mm, DWM3000 module incl. antenna tab"),
    ("module_w", 16, "mm"),
    ("perf_z", "=imu_z - 4 - pcb_t", "mm, perfboard underside (approx.)"),
    ("perf_l", 70, "mm"),
    ("perf_w", 50, "mm"),

    ("# Laser-cut baseplate", None, ""),
    ("plate_t", 0.25 * IN, "mm, sheet thickness (MEASURE: your stock)"),
    ("plate_dowel_d", "=dowel_d", "mm, net shape: the laser's kerf gives the slip fit"),
    ("plate_x0", 6, "mm (antenna keep-out side)"),
    ("plate_x1", 114, "mm, static plate"),
    ("plate_y0", -40, "mm"),
    ("plate_y1", 90, "mm"),
    ("seat1_x", 20, "mm, seats: a scalene triangle outside the board"),
    ("seat1_y", -20, "mm"),
    ("seat2_x", 94, "mm"),
    ("seat2_y", 6, "mm"),
    ("seat3_x", 82, "mm"),
    ("seat3_y", 70, "mm"),
    ("side12", "=sqrt((seat2_x - seat1_x)^2 + (seat2_y - seat1_y)^2)", "mm"),
    ("side23", "=sqrt((seat3_x - seat2_x)^2 + (seat3_y - seat2_y)^2)", "mm"),
    ("side31", "=sqrt((seat1_x - seat3_x)^2 + (seat1_y - seat3_y)^2)", "mm"),
    ("keepout", 10, "mm, no plate within this of the antenna tab"),
    ("keepout_ok", "=plate_x0 - (-ant_tip_fwd + keepout)", "mm, must be >= 0"),
    ("ins_x", "=board_l / 2", "mm, 1/4\" tripod hole, under the board's middle"),
    ("ins_y", "=board_w / 2", "mm"),
    ("tripod_d", 0.25 * IN, "mm, net shape (laser-cut)"),
    ("nut_af", 7 / 16 * IN, "mm, 1/4\"-20 hex nut across flats"),
    ("nut_h", 7 / 32 * IN, "mm"),
    ("ins_clear", "=standoff_h - nut_h", "mm, nut to the board's underside, must be > 1"),

    ("# Mobile plate and mast", None, ""),
    ("mob_x1", 136, "mm, mobile plate"),
    ("mast_x", 124, "mm, mast centre"),
    ("mast_y", "=board_w / 2", "mm"),
    ("mast_hole_d", "=tube_od", "mm, laser-cut, net shape: locates the mast"),
    ("collar_h", 15, "mm"),
    ("tracker_clear", 50, "mm, stack top to the tracker's mount face"),
    ("mount_z", "=stack_top + tracker_clear", "mm, datum A of the mobile head"),
    ("mast_len", "=mount_z - head_h + socket_depth + plate_t", "mm, tube to cut (flush with the plate's underside)"),

    ("# Tracker 3.0 interface (HTC developer guidelines pp. 15-18)", None, ""),
    ("pin_off", 14.001, "mm, screw to stabilising pin recess"),
    ("pin_d", 4.0, "mm, pin (recess is 4.20 +-0.10)"),
    ("pin_h", 4.2, "mm (recess is 4.70 deep)"),
    ("pogo_off", 17.219, "mm, screw to pogo slot centre (opposite the pin)"),
    ("pogo_half", 9.2, "mm, slot straight half-length (R3.30 ends)"),
    ("pogo_w", 6.6, "mm, slot width"),
    ("key_clear_w", 0.1, "mm, key to slot across its width, per side: stops rotation about the pin"),
    ("key_clear_l", 0.5, "mm, key short of the slot's straight length, per end: loose, the pin locates"),
    ("key_h", 1.5, "mm, key height (pads are 2.595 deep)"),

    ("# Head", None, ""),
    ("mount_w", 28, "mm, top plate width (x): the key is 25 long"),
    ("mount_y0", "=-(pogo_off + pogo_w / 2 + 1.5)", "mm"),
    ("mount_y1", "=pin_off + pin_d / 2 + 2", "mm"),
    ("top_t", 3, "mm, top plate"),
    ("head_h", 20, "mm, datum A to the head's underside"),
    ("lobe_drop", 3, "mm, the tube lobe's top below datum A (only the interface touches the tracker)"),
    ("tube_off", 22, "mm, tube axis from the screw axis"),
    ("web_t", 6, "mm, web between core and lobe"),
    ("screw_clear_d", 7.2, "mm, 1/4\" clearance: the screw clamps, the pin and key locate"),
    ("bolt_len", 0.6 * IN, "mm, 1/4\"-20 socket cap screw, 0.6\""),
    ("cap_d", 0.430 * IN, "mm, cap head 0.430\""),
    ("cap_h", 0.25 * IN, "mm, cap head height (1/4\" SHCS)"),
    ("drive_af", 5 / 32 * IN, "mm, 5/32\" hex drive"),
    ("engage", 5, "mm, thread into the tracker (ISO 1222: keep it short)"),
    ("t_screw", "=bolt_len - engage", "mm, material between the counterbore floor and datum A"),
    ("cbore_d", "=cap_d + 1.2", "mm, clear of the cap all round"),
    ("core_half", "=cbore_d / 2 + 2.4", "mm, square core around the counterbore"),
    ("taper", "=-mount_y0 - core_half", "mm, top plate to core: the widest overhang, so every side is <= 45 deg"),
    ("taper_ok", "=taper - max(mount_y1 - core_half; mount_w / 2 - core_half)", "mm, must be >= 0"),
    ("core_ok", "=head_h - top_t - taper", "mm, core below the taper, must be >= 0"),
    ("cbore_room", "=head_h - t_screw - cap_h", "mm, below the cap for the hex key, must be > 3"),
    ("socket_roof", "=head_h - lobe_drop - socket_depth", "mm, above the tube socket, must be > 1.5"),
    ("tube_wall", "=tube_off - cbore_d / 2 - (tube_od + tube_clear) / 2", "mm, between counterbore and socket, must be > 2"),

    ("# Stylus", None, ""),
    ("tip_len", 130, "mm, datum A to the ball centre, along z (nominal; pivot cal measures it)"),
    ("stylus_tube_len", "=tip_len - head_h + socket_depth - ball_off", "mm, tube to cut for the stylus"),

    ("# Layout of the parts in the document (display only)", None, ""),
    ("mob_oy", 160, "mm, mobile plate offset"),
    ("st_ox", -130, "mm, stylus head offset"),
    ("st_oy", 30, "mm"),
    ("pv_ox", -110, "mm, pivot cone (on a brick) offset"),
    ("pv_oy", 170, "mm"),
]

doc = None


def P(expr):
    """Expressions are written with P.alias; the sheet is the object Params."""
    return expr.replace("P.", "Params.")


def expr(o, prop, e):
    o.setExpression(prop, P(e))


def prim(kind, name, dims, pos, props=None, rot=None):
    o = doc.addObject("Part::" + kind, name)
    for k, v in (props or {}).items():
        setattr(o, k, v)
    if rot is not None:
        o.Placement.Rotation = rot
    for k, v in dims.items():
        expr(o, k, v)
    for axis, v in zip("xyz", pos):
        expr(o, "Placement.Base." + axis, v)
    return o


def at(ox, oy, oz):
    """Position helpers for a part placed at an offset (all expressions)."""
    return (lambda e: f"({ox}) + ({e})", lambda e: f"({oy}) + ({e})", lambda e: f"({oz}) + ({e})")


def fuse(name, objs):
    if len(objs) == 1:
        return objs[0]
    f = doc.addObject("Part::MultiFuse", name)
    f.Shapes = objs
    f.Refine = True
    return f


def cut(name, base, tools):
    c = doc.addObject("Part::Cut", name)
    c.Base = base
    c.Tool = fuse(name + "_tools", tools)
    c.Refine = True
    return c


class Local:
    """Primitives in a part's local frame, placed at (ox, oy, oz) and turned by
    0 or 180 deg about z (turn=-1)."""

    def __init__(self, ox, oy, oz, turn=1):
        self.X, self.Y, self.Z = at(ox, oy, oz)
        self.t = turn

    def _rx(self, x0, x1):
        return (x0, x1) if self.t > 0 else (f"-({x1})", f"-({x0})")

    def box(self, name, x0, x1, y0, y1, z0, z1):
        (x0, x1), (y0, y1) = self._rx(x0, x1), self._rx(y0, y1)
        return prim("Box", name, {"Length": f"({x1}) - ({x0})", "Width": f"({y1}) - ({y0})",
                                  "Height": f"({z1}) - ({z0})"},
                    (self.X(x0), self.Y(y0), self.Z(z0)))

    def cyl(self, name, r, x, y, z0, z1, kind="Cylinder", props=None, dims=None):
        if self.t < 0:
            x, y = f"-({x})", f"-({y})"
        d = {"Radius": r, "Height": f"({z1}) - ({z0})"} if dims is None else dims
        return prim(kind, name, d, (self.X(x), self.Y(y), self.Z(z0)), props)

    def frustum(self, name, top, bottom, z_top, depth):
        """A rectangular frustum hanging down from z_top: top = (x0, x1, y0, y1)
        at z_top, bottom = (x0, x1, y0, y1) at z_top - depth (a Part::Wedge
        turned so its y axis points down)."""
        tx, ty = self._rx(top[0], top[1]), self._rx(top[2], top[3])
        bx, by = self._rx(bottom[0], bottom[1]), self._rx(bottom[2], bottom[3])
        dims = {"Xmin": tx[0], "Xmax": tx[1], "Zmin": ty[0], "Zmax": ty[1], "Ymin": "0", "Ymax": depth,
                "X2min": bx[0], "X2max": bx[1], "Z2min": by[0], "Z2max": by[1]}
        return prim("Wedge", name, dims, (self.X("0"), self.Y("0"), self.Z(z_top)),
                    rot=App.Rotation(App.Vector(1, 0, 0), -90))


def head(sfx, ox, oy, oz, turn=1):
    """Tracker head: datum A at (ox, oy, oz), tracker above, tube socket open
    downwards at x = +tube_off (x = -tube_off when turned)."""
    L = Local(ox, oy, oz, turn)
    n = f"Head_{sfx}"
    c = "P.core_half"
    add = [
        L.box(n + "_top", "-P.mount_w / 2", "P.mount_w / 2", "P.mount_y0", "P.mount_y1", "-P.top_t", "0"),
        L.frustum(n + "_taper", ("-P.mount_w / 2", "P.mount_w / 2", "P.mount_y0", "P.mount_y1"),
                  (f"-{c}", c, f"-{c}", c), "-P.top_t", "P.taper"),  # flush: an overlap leaves a sliver
        L.box(n + "_core", f"-{c}", c, f"-{c}", c, "-P.head_h", "-P.top_t - P.taper + 0.01"),
        L.box(n + "_web", "0", "P.tube_off", "-P.web_t / 2", "P.web_t / 2", "-P.head_h", "-P.lobe_drop"),
        L.cyl(n + "_lobe", "P.boss_d / 2", "P.tube_off", "0", "-P.head_h", "-P.lobe_drop"),
        L.cyl(n + "_pin", "P.pin_d / 2", "0", "P.pin_off", "-0.01", "P.pin_h"),
        L.box(n + "_keymid", "-(P.pogo_half - P.key_clear_l)", "P.pogo_half - P.key_clear_l",
              "-P.pogo_off - (P.pogo_w / 2 - P.key_clear_w)", "-P.pogo_off + (P.pogo_w / 2 - P.key_clear_w)",
              "-0.01", "P.key_h"),
        L.cyl(n + "_keyL", "P.pogo_w / 2 - P.key_clear_w", "-(P.pogo_half - P.key_clear_l)", "-P.pogo_off",
              "-0.01", "P.key_h"),
        L.cyl(n + "_keyR", "P.pogo_w / 2 - P.key_clear_w", "P.pogo_half - P.key_clear_l", "-P.pogo_off",
              "-0.01", "P.key_h"),
    ]
    sub = [
        L.cyl(n + "_hole", "P.screw_clear_d / 2", "0", "0", "-P.t_screw - 0.1", "0.1"),
        L.cyl(n + "_cbore", "P.cbore_d / 2", "0", "0", "-P.head_h - 0.1", "-P.t_screw"),
        L.cyl(n + "_socket", "(P.tube_od + P.tube_clear) / 2", "P.tube_off", "0",
              "-P.head_h - 0.1", "-P.head_h + P.socket_depth"),
    ]
    part = cut(n, fuse(n + "_solid", add), sub)

    # The cap screw, tightened (reference): head on the counterbore floor.
    s = f"Screw_{sfx}"
    screw = cut(s, fuse(s + "_solid", [
        L.cyl(s + "_cap", "P.cap_d / 2", "0", "0", "-P.t_screw - P.cap_h", "-P.t_screw"),
        L.cyl(s + "_shank", "0.125 * 25.4", "0", "0", "-P.t_screw - 0.01", "-P.t_screw + P.bolt_len"),
    ]), [L.cyl(s + "_drive", None, "0", "0", "-P.t_screw - P.cap_h - 0.01", "-P.t_screw - P.cap_h + 3.5",
               kind="Prism", props={"Polygon": 6},
               dims={"Circumradius": "P.drive_af / sqrt(3)", "Height": "3.51"})])
    return part, screw


def seat_cone(name, x, y, z0):
    """Flanged seat cone standing on z0 (the plate top, or a brick)."""
    body = fuse(name + "_solid", [
        prim("Cylinder", name + "_flange", {"Radius": "P.flange_d / 2", "Height": "P.flange_t"}, (x, y, z0)),
        prim("Cylinder", name + "_body", {"Radius": "P.cone_d / 2", "Height": "P.cone_h - 0.01"},
             (x, y, f"({z0}) + 0.01")),
    ])
    top = f"({z0}) + P.cone_h"
    return cut(name, body, [
        prim("Cone", name + "_cone", {"Radius1": "0", "Radius2": "P.seat_top_d / 2 + 0.5 * tan(P.seat_half)",
                                      "Height": "P.seat_depth + 0.5"},
             (x, y, f"{top} - P.seat_depth")),
        prim("Cylinder", name + "_relief", {"Radius": "P.seat_relief_d / 2", "Height": "P.seat_relief_h"},
             (x, y, f"{top} - P.seat_depth - P.seat_relief_h + 0.3")),
        prim("Cylinder", name + "_dowel", {"Radius": "P.dowel_hole_d / 2", "Height": "P.dowel_depth + 0.1"},
             (x, y, f"({z0}) - 0.1")),
    ])


def plate(name, oy, x1, mobile):
    """Laser-cut baseplate (z in [-plate_t, 0]) and its glued-on printed parts.
    Returns (plate, printed parts, bought-part references)."""
    X, Y, Z = at("0", oy, "0")
    base = prim("Box", name + "_sheet", {"Length": f"{x1} - P.plate_x0", "Width": "P.plate_y1 - P.plate_y0",
                                         "Height": "P.plate_t"},
                (X("P.plate_x0"), Y("P.plate_y0"), Z("-P.plate_t")))
    through = lambda nm, r, x, y: prim("Cylinder", nm, {"Radius": r, "Height": "P.plate_t + 0.2"},  # noqa: E731
                                       (X(x), Y(y), Z("-P.plate_t - 0.1")))
    holes = [through(f"{name}_m3_{i}", "P.m3_hole_d / 2", f"P.hole{i}_x", f"P.hole{i}_y") for i in range(1, 5)]
    holes += [through(f"{name}_dowel{i}", "P.plate_dowel_d / 2", f"P.seat{i}_x", f"P.seat{i}_y") for i in range(1, 4)]
    holes.append(through(name + "_tripod", "P.tripod_d / 2", "P.ins_x", "P.ins_y"))
    sfx = name.replace("Plate", "")
    printed = [seat_cone(f"Cone_{sfx}_{i}", X(f"P.seat{i}_x"), Y(f"P.seat{i}_y"), "0") for i in range(1, 4)]
    refs = [prim("Prism", f"Nut_{sfx}", {"Circumradius": "P.nut_af / sqrt(3)", "Height": "P.nut_h"},
                 (X("P.ins_x"), Y("P.ins_y"), Z("0")), {"Polygon": 6})]
    refs += [prim("Cylinder", f"Standoff_{sfx}_{i}", {"Radius": "2.75", "Height": "P.standoff_h"},
                  (X(f"P.hole{i}_x"), Y(f"P.hole{i}_y"), Z("0"))) for i in range(1, 5)]
    if mobile:
        holes.append(through(name + "_mast", "P.mast_hole_d / 2", "P.mast_x", "P.mast_y"))
        printed.append(cut(f"MastCollar", prim("Cylinder", "MastCollar_body", {"Radius": "P.boss_d / 2",
                                                                               "Height": "P.collar_h"},
                                               (X("P.mast_x"), Y("P.mast_y"), Z("0"))),
                           [prim("Cylinder", "MastCollar_bore", {"Radius": "(P.tube_od + P.tube_clear) / 2",
                                                                 "Height": "P.collar_h + 0.2"},
                                 (X("P.mast_x"), Y("P.mast_y"), Z("-0.1")))]))
    return cut(name, base, holes), printed, refs


def stack_reference(pfx, oy):
    """Board stack, antenna and sensor markers above a plate (not printed)."""
    X, Y, Z = at("0", oy, "0")
    refs = [
        prim("Box", pfx + "ESP32", {"Length": "P.board_l", "Width": "P.board_w", "Height": "P.pcb_t"},
             (X("0"), Y("0"), Z("P.esp_z"))),
        prim("Box", pfx + "Shield", {"Length": "P.board_l", "Width": "P.board_w", "Height": "P.pcb_t"},
             (X("0"), Y("0"), Z("P.shield_z"))),
        prim("Box", pfx + "DWM3000", {"Length": "P.module_l", "Width": "P.module_w", "Height": "2.6"},
             (X("-P.ant_tip_fwd"), Y("P.ant_y - P.module_w / 2"), Z("P.shield_z + P.pcb_t"))),
        prim("Box", pfx + "Perfboard", {"Length": "P.perf_l", "Width": "P.perf_w", "Height": "P.pcb_t"},
             (X("0"), Y("0"), Z("P.perf_z"))),
        prim("Sphere", pfx + "Antenna", {"Radius": "1.5"}, (X("P.ant_x"), Y("P.ant_y"), Z("P.ant_z"))),
        prim("Box", pfx + "IMU", {"Length": "4", "Width": "4", "Height": "1"},
             (X("P.imu_x - 2"), Y("P.imu_y - 2"), Z("P.imu_z - 0.5"))),
        prim("Box", pfx + "Mag", {"Length": "3", "Width": "3", "Height": "1"},
             (X("P.mag_x - 1.5"), Y("P.mag_y - 1.5"), Z("P.mag_z - 0.5"))),
    ]
    for i in range(1, 4):
        refs.append(prim("Sphere", f"{pfx}Ball{i}", {"Radius": "P.ball_d / 2"},
                         (X(f"P.seat{i}_x"), Y(f"P.seat{i}_y"), Z("P.ball_z"))))
    return refs


def tube(name, x, y, z0, length):
    t = prim("Cylinder", name + "_outer", {"Radius": "P.tube_od / 2", "Height": length}, (x, y, z0))
    bore = prim("Cylinder", name + "_bore", {"Radius": "P.tube_id / 2", "Height": f"({length}) + 0.2"},
                (x, y, f"({z0}) - 0.1"))
    return cut(name, t, [bore])


def tracker_envelope(name, ox, oy, oz):
    """Rough Tracker 3.0 envelope (79 x 70.9 x 44.1) over a head (not printed)."""
    X, Y, Z = at(ox, oy, oz)
    return prim("Box", name, {"Length": "70.9", "Width": "79", "Height": "44.1"},
                (X("-70.9 / 2"), Y("-45"), Z("0")))


def build():
    global doc
    if App.listDocuments().get(DOC_NAME):
        App.closeDocument(DOC_NAME)
    doc = App.newDocument(DOC_NAME)
    sheet = doc.addObject("Spreadsheet::Sheet", "Params")
    sheet.set("A1", "alias")
    sheet.set("B1", "value")
    sheet.set("C1", "note")
    for row, (alias, value, note) in enumerate(PARAMS, start=2):
        if alias.startswith("#"):
            sheet.set(f"A{row}", alias[1:].strip())
            continue
        sheet.set(f"A{row}", alias)
        sheet.set(f"B{row}", value if isinstance(value, str) else f"{value:.6g}")
        sheet.set(f"C{row}", note)
        sheet.setAlias(f"B{row}", alias)
    doc.recompute()

    # Stylus: datum A at z = tip_len, so the ball centre is on z = 0, under the tube.
    st = ("P.st_ox", "P.st_oy", "P.tip_len")
    # Mobile: the head turned 180 deg, its screw outboard of the mast.
    mob = ("P.mast_x + P.tube_off", "P.mob_oy + P.mast_y", "P.mount_z")
    head_st, screw_st = head("Stylus", *st)
    head_mob, screw_mob = head("Mobile", *mob, turn=-1)
    plate_st, cones_st, refs_st = plate("PlateStatic", "0", "P.plate_x1", mobile=False)
    plate_mob, cones_mob, refs_mob = plate("PlateMobile", "P.mob_oy", "P.mob_x1", mobile=True)
    pivot = seat_cone("Cone_Pivot", "P.pv_ox", "P.pv_oy", "0")
    lasercut = [plate_st, plate_mob]
    printed = [head_st, head_mob, pivot] + cones_st + cones_mob
    refs = stack_reference("Static_", "0") + stack_reference("Mobile_", "P.mob_oy") + refs_st + refs_mob
    refs += [
        screw_st,
        screw_mob,
        tube("Tube_Stylus", "P.st_ox + P.tube_off", "P.st_oy", "P.ball_off", "P.stylus_tube_len"),
        prim("Sphere", "Ball_Stylus", {"Radius": "P.ball_d / 2"}, ("P.st_ox + P.tube_off", "P.st_oy", "0")),
        tube("Tube_Mobile", "P.mast_x", "P.mob_oy + P.mast_y", "-P.plate_t", "P.mast_len"),
        prim("Box", "Brick", {"Length": "100", "Width": "60", "Height": "40"},
             ("P.pv_ox - 50", "P.pv_oy - 30", "-40")),
        tracker_envelope("Tracker_Stylus", *st),
        tracker_envelope("Tracker_Mobile", *mob),
    ]
    doc.recompute()

    group = doc.addObject("App::DocumentObjectGroup", "Reference")
    group.addObjects(refs)
    if App.GuiUp:
        import FreeCADGui as Gui
        keep = set(o.Name for o in printed + lasercut + refs)
        for o in doc.Objects:
            if o.TypeId.startswith("Part::") and o.Name not in keep:
                o.ViewObject.Visibility = False
        for o in lasercut:
            o.ViewObject.ShapeColor = (0.80, 0.70, 0.50)
        for o in printed:
            o.ViewObject.ShapeColor = (0.95, 0.55, 0.15) if o.Name.startswith("Head") else (0.30, 0.55, 0.85)
        for o in refs:
            o.ViewObject.Transparency = 60
            o.ViewObject.ShapeColor = (0.2, 0.7, 0.3)
        for o in refs:
            if o.Name.startswith(("Tube_", "Ball_", "Screw_", "Nut_", "Standoff_")):
                o.ViewObject.Transparency = 0
                o.ViewObject.ShapeColor = (0.75, 0.75, 0.78)
        doc.getObject("Brick").ViewObject.ShapeColor = (0.6, 0.3, 0.2)
        for n, c in (("Antenna", (0.9, 0.1, 0.1)), ("IMU", (0.1, 0.2, 0.9)), ("Mag", (0.6, 0.1, 0.8))):
            for p in ("Static_", "Mobile_"):
                doc.getObject(p + n).ViewObject.ShapeColor = c
                doc.getObject(p + n).ViewObject.Transparency = 0
        for n in ("Tracker_Stylus", "Tracker_Mobile"):
            doc.getObject(n).ViewObject.Transparency = 80
        Gui.ActiveDocument.ActiveView.viewIsometric()
        Gui.SendMsgToActiveView("ViewFit")
    bad = [o.Name for o in doc.Objects if o.TypeId.startswith("Part::")
           and (o.Shape.isNull() or not o.Shape.isValid())]
    print("invalid shapes:", bad or "none")
    return doc


build()
