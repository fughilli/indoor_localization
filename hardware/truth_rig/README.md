# Ground-truth rig (Vive trackers → UWB boards)

Parts for collecting UWB data against Vive Tracker ground truth.

## Parts

| File | Make | Qty | What |
|---|---|---|---|
| `stl/Head.stl` | print | 2 | Tracker head, for the stylus and the mobile rig (details below). |
| `stl/SeatCone.stl` | print | 3 per plate, +1 | Flanged puck with a 45° cone for the 1/2" ball, and a blind 1/4" dowel hole underneath. The extra one glued to a brick is the pivot (tip) calibration fixture. |
| `stl/MastCollar.stl` | print | 1 | Keeps the mobile rig's mast square to its plate. |
| `stl/BallStandIn.stl` | print | 1, optional | Printed stand-in for the stylus's steel ball (from `ball_standin.py`): a 12.7 mm sphere on a 7.67 mm peg (bore − 0.2, a slip fit) that's glued into the tube's bore. It sits on the bore rim like the steel ball does. Print it peg down. |
| `dxf/PlateStatic.dxf` | laser | 1 per static board | Outline; 4× M3 holes (board standoffs); 3× 1/4" dowel holes (seat cones); 1/4" tripod hole. |
| `dxf/PlateMobile.dxf` | laser | 1 | The same, plus the mast tube's hole. |

The DXFs are in millimetres and plate coordinates. The plate thickness is the `plate_t` parameter (1/4" stock assumed). Laser-cut holes are net shape (dowel 6.35, mast tube 9.525, M3 3.0, tripod 6.35): the kerf gives the clearance. Printed tube sockets are the tube's OD + 0.15 mm.

**Head details.**
- A 3 mm plate carries the tracker. The stabilising pin locates it. The pogo-slot key fits the slot's width closely (0.1 mm per side) and stops it turning, but is loose along the slot so it doesn't fight the pin.
- The 1/4"-20 cap screw only clamps. It has clearance all round its shank (0.43 mm per side) and its cap (0.60 mm per side), so it never does the locating.
- Under the plate, a 45° taper runs down to a core around the counterbore. The screw goes up from the counterbore and is driven with a 5/32" key from below.
- A side lobe 22 mm off the screw axis holds the tube, so the key can reach the screw.
- Volume is 15.3 cm³, against 36.4 cm³ for the previous solid block.

**Printing.** Everything prints bottom-down, with no supports. Every hole is vertical, every cone opens upward, and the head's underside tapers at 45°. The only bridges are blind-hole roofs (≤ 12 mm). `check_rig.py` verifies this.

## Bought parts

- Steel tube, 0.375" OD × 0.310" ID. Cut lengths are in `geometry.json` → `cut_list_mm` (stylus 120 mm, mast 92 mm at the current parameters).
- 1/2" steel balls.
- 2× 1/4"-20 × 0.6" socket cap screws (5/32" hex). 10.2 mm of the head sits between the counterbore and the tracker, so 5 mm of thread goes into the tracker.
- 1/4" dowel pin: a glue-up fixture, removed afterwards.
- Per plate: 4× M3 × 8 mm standoffs and M3 screws, and 1× 1/4"-20 hex nut.
- Epoxy.
- A brick, for the pivot fixture.

## Assembly

1. **Seat cones.** Push the 1/4" dowel up through a seat hole in the plate and into a cone's underside. Glue the flange to the plate, keeping glue off the dowel and out of the hole. Let it set, then pull the dowel out from below. Repeat for each cone. The laser-cut hole sets the position; the glue only holds it.
2. **Tripod nut.** Glue a 1/4"-20 nut over the tripod hole on the plate's top, with a 1/4"-20 screw through both to locate it while the glue sets.
3. **Board.** Mount it on the M3 standoffs.
4. **Stylus.** Epoxy the tube into a head's socket. Epoxy the ball onto the tube's other end; it sits on the bore's rim, so it centres itself. The pivot calibration measures the actual tip.
5. **Mobile rig.** Push the mast tube through the plate's hole, flush underneath, and through the collar. Glue both. Epoxy a head on top, with its screw pointing outboard, away from the board.
6. **Pivot fixture.** Glue a seat cone's flange to a brick.
7. **Tracker on.** Drop the cap screw into the counterbore, set the tracker on (pin into its recess, key into the pogo slot), and drive the screw with a 5/32" hex key from below.

## Files

| File | |
|---|---|
| `build_rig.py` | Builds `TruthRig` in FreeCAD from the `PARAMS` table. Every dimension is an alias in the `Params` sheet, and the parts are expression-driven primitives. |
| `check_rig.py` | Interference (54 pairs), screw clearance against the locating fits, hex key access, overhangs and bridges, and the sheet's must-not-be-negative cells. |
| `export_geometry.py` | Writes `geometry.json` (seat ball centres, seat-to-seat distances, plate → antenna / IMU / magnetometer, nominal stylus tip, cut list). `export(fab=True)` also writes `stl/` and `dxf/`. |
| `TruthRig.FCStd` | The document. |

In FreeCAD's Python console (or through the MCP), from the repo root:

```python
exec(open("hardware/truth_rig/build_rig.py").read())        # rebuild from PARAMS
exec(open("hardware/truth_rig/check_rig.py").read())        # DFA / DFM checks
exec(open("hardware/truth_rig/export_geometry.py").read())  # geometry.json
```

Edit a number either in `PARAMS` (then rebuild) or in the `Params` sheet (FreeCAD recomputes). Keep the two in step: `PARAMS` is the record.

## Plate frame

- **x:** from the ESP32 board's front edge, the one the antenna points past (the antenna is at x < 0), towards the back.
- **y:** from the left edge. "Left" is the DC jack side, facing the antenna.
- **z:** up, with z = 0 on the plate top. The board's bottom copper is at z = `standoff_h`.

The measured antenna and sensor positions are entered relative to the board's edges and bottom copper:

| | Measured |
|---|---|
| Antenna | tab tip 0.35" ahead of the front edge, centred left/right, 0.54" above the bottom copper |
| MPU6050 | 1.45" from the front, 0.76" from the left |
| Magnetometer | 2.06" from the front, 0.83" from the left |
| Both sensors | 1.17" above the bottom copper |

The antenna is U-shaped, so the model's antenna point is nominal: `ant_inset` (3 mm) behind the tab tip. The phase centre, and how it varies with angle, is estimated from the data.

## Numbers still to measure (`MEASURE` in `PARAMS`)

- `board_l`, `board_w`, `hole*_x/y`: the ESP32 board and its holes. The defaults are the Uno R3 pattern, measured from the front-left corner.
- `plate_t`: the laser stock's thickness.
- `imu_yaw`: how the GY-521's X arrow points. The data refines it.
