# Reflectors and faders in the UWB mesh

Joint estimation of fixed reflectors and faders against the swarm, in the
simulator (`swarm/sim/multipath.py`, `landmarks.py`, `landmarks_run.py`,
`landmarks_test.py`). Status: simulation only. The channel numbers are plausible
DW3000 values, not bench fits; nothing here has run on hardware.

## Terms

- **Reflector**: a fixed point scatterer. A link a→b gets a second path of length
  |a−m| + |m−b|, amplitude `rho · d_ab / (d1 d2)` against a clear line of sight.
- **Fader**: a fixed sphere (centre, radius) that costs a path
  `loss_db · chord / (2r)` dB on each leg it crosses. It shadows the line of sight
  (first-path power) and can hide it, so the radio times a reflection instead.

"Fader" is my reading of the word; if you meant something else, the model is in
`multipath.Fader` and easy to change.

## The two problems

- **P1, map.** Where are the reflectors and faders, in the mesh's frame?
  `map_reflectors` grid-votes over the path-length (and bearing) constraints, peels
  off the best-supported point, refines it robustly and repeats; a peak must stand
  clear of the background vote level, so clutter alone yields nothing.
  `map_faders` is greedy tomography on the line-of-sight attenuation.
- **P2, locate the mesh.** Given references at known room positions, where is the
  mesh? The mesh knows up (gravity), so its offset from the room is four numbers:
  translation and yaw. `register` finds it from P1's map (pair-to-pair matching,
  no prior); `solve_gauge` then refines it per time window from the raw
  observations, so the mesh's drift is held to the references instead of wandering.

## What each observation constrains

| Observation | Constraint on a reflector m |
|---|---|
| first-arrival range, only when the LOS is lost | ellipsoid (foci a, b), with a ±0.5 m long-reading error |
| CIR path length | ellipsoid, ~0.15–0.3 m |
| + PDoA phase of that path | a cone about the baseline: one component of the bearing from b to m |
| + a second baseline | a second bearing component |

One link with two baselines fixes a reflector (a test checks the Jacobian rank:
1 for a length, 2 with one pair, 3 with two). With one baseline, or none, many links
are needed, and a **mirror ambiguity** remains: nodes at similar heights cannot tell
a reflector from its image across their mean plane (z = 3.0 reads as −0.1).

## What the DWM3000 / QM33120 supports

Checked against Qorvo's product pages and forum; the CIR size is from user code,
not a spec, and the register-level details are **not** verified. Read the DW3000
user manual before relying on them.

- Channels: **5 and 9 only** (also `libs/dw3000`: `CHANNEL_5`, `CHANNEL_9`).
- **DW3110 (the DWM3000's chip): one antenna port, no PDoA.** PDoA/AoA needs a part
  with two receive paths (DW3210, QM33120).
- On the QM33120 kit, the AoA antenna is tuned for channel 9; channel 5 through
  it gives inaccurate AoA (Qorvo forum). Your CH5/CH9-tuned patches address this.
- CIR: the driver and forum code use a 1016-sample Ipatov CIR; sample spacing is
  about 1 ns (0.3 m of path). Peaks closer than ~3 ns merge: the model uses 0.9 m.
- Per-frame diagnostics exist (first-path index and amplitude, peak, the
  total/first-path power the repo's `gap` is built from).

## Array model

Per your design: one pair of patches per exchange, CH5 on one board axis and CH9 on
the other, both in the board's plane. Patches hear the front hemisphere (gain
10·1.5·log10 cos θ, floored at −20 dB, which is also what anything behind gets; a
round-number model, not a measured pattern). Two mounts are simulated:

- **upright** (board vertical, facing body +y): CH5 baseline horizontal, CH9 vertical.
- **flat** (board horizontal, facing up): both baselines horizontal.

## Results (simulation: 6 widgets, 3 reflectors, 2 faders, 30 s, 3 seeds)

P1, reflectors (error vs truth):

| Tier | Found | Horizontal | Vertical |
|---|---|---|---|
| first arrival only | 0/9 | – | – |
| CIR paths | 9/9 | 0.14 m | **2.15 m** (mirrored) |
| + PDoA, one pair (CH5) | 9/9 | 0.13 m | **2.15 m** (mirrored) |
| + PDoA, CH5 + CH9 pairs, upright | 9/9 | 0.07 m | 0.09 m |
| + PDoA, CH5 + CH9 pairs, flat | 9/9 | 0.10 m | 0.98 m |

- First arrival only sees a reflector only when a fader makes the radio time it, and
  in this scene that almost never happened. It does localize faders from the
  attenuation (the gap) alone, coarsely (2.4 m horizontal, 5/6 found, 7 false).
- The vertical result depends on the mount. Upright, the CH9 baseline is vertical, so
  the phase resolves height; the board's own front/back mirror is then handled by
  the patch pattern. Flat, both baselines are horizontal and phase can't see
  elevation; only the patch pattern (reflectors above the board are in front, their
  images below are not) breaks the mirror, and not always.
- **Motion barely breaks the mirror.** CIR-only vertical error: still 2.09 m, orbit
  2.15 m, hand-carried (wrist wobble, 0.25 m lift) 1.84 m. The path length changes with reflector height as roughly (node height
  spread)/(range), here ~0.3 m of length for a 3 m ambiguity: inside the noise. A
  hand's 0.25 m lift would not change that; a staircase would.

P2, tracking the gauge against the three references (window 10 s, 6 seeds, error vs
the gauge averaged over the window; mesh drifting 0.03 m/√s and 0.004 rad/√s):

| Condition | xy | z | yaw |
|---|---|---|---|
| no mesh shape error | 0.035 m | 0.025 m | 0.13° |
| shape error 0.10 m per node, constant | 0.071 m | 0.073 m | 0.23° |
| same, decorrelating over 15 s | 0.101 m | 0.071 m | 0.27° |
| still / orbit / hand-carried (constant error) | 0.086 / 0.071 / 0.068 m | 0.101 / 0.073 / 0.063 m | 0.25 / 0.23 / 0.21° |
| no array / flat / upright mount | 0.067 / 0.120 / 0.071 m | 0.102 / 0.111 / 0.073 m | 0.20 / 0.34 / 0.23° |
| 4 / 6 / 12 widgets | 0.133 / 0.071 / 0.085 m | 0.111 / 0.073 / 0.036 m | 0.39 / 0.23 / 0.26° |

- The floor is the mesh's own shape error, about σ/√N of the node errors, not radio
  noise. Radio noise alone gives 3.5 cm.
- **Motion helps little** (still vs hand-carried differ by about 20 %, within the
  seed noise), because the error belongs to the node, not to the geometry.
- More nodes help (vertical error halves from 6 to 12 widgets); xy at 12 doesn't
  improve here, which at 6 seeds I can't distinguish from noise.
- Window length is a trade: longer windows average a decorrelating shape error
  (xy 11 cm at 5 s to 9 cm at 40 s, vs the window mean) but lag the real drift (vs
  the gauge *now*: 12 cm to 17 cm); at 80 s association fails. A drift-aware
  filter across windows is the obvious next step and is not built.
- The flat mount is worse than no array (0.120 vs 0.067 m xy). I believe the patch
  pattern is why (grazing arrivals at the upward-facing patches cost 10–20 dB), but I
  have not isolated it.
- `solve_gauge`'s own covariance (≈ 6 mm) is far too optimistic against these errors
  (the node errors are constant, not independent); do not use it for fusion as is.

## Not modelled / open

- **No clock sync or TDoA**: ranging is pairwise DS-TWR, one exchange per slot
  (§5.3 of the design); the sim matches. See below for what TDoA would and
  would not add.
- Scatterer model: point reflectors only (no planar walls, no
  second-order bounces beyond clutter); faders with a known radius.
- The transmitter is omnidirectional; only the receiving pair has a pattern.
- Calibration of the PDoA sign/offsets per node is a fixed random error per pair
  (0.05 rad), not estimated.
- Wiring into `World`'s live estimator loop: `MeshSampler` reuses the same widget
  trajectories but is standalone. A `World` hook needs the Rust build to test.

## Broadcast, and the 10-node fleet

The sim now has a **broadcast** slot (`MeshSampler(mode="broadcast")`): one
transmitter per slot, every other node hears it and logs its own CIR and PDoA, against
the pairwise two-way exchange (one pair, both ends). For 10 nodes that is 9
observations per slot instead of 2 (4.5×, measured). Fleet: 4 mobile nodes with IMUs
and 6 static ones without, placed by eye facing the room centre. Receivers' clock
errors (`SyncModel`: per-node, correlated over 5 s, in metres of path) enter every
length of a *one-way* frame and cancel in a two-way exchange.

Because a static node has no IMU, its attitude is unknown to ~0.3 rad, and PDoA is
useless without it. `landmarks.calibrate_yaw` recovers it from the PDoA of the line
of sight to peers at known mesh positions: 0.3 rad → 0.016–0.020 rad on all 6 nodes.
(It assumes the node is level.)

Reflector P1, weak reflectors (rho 0.5), equal airtime, 3 seeds, 9 reflectors found
per row:

| Exchange | 1 s: horiz / vert | 4 s: horiz / vert |
|---|---|---|
| pairwise two-way | 0.07 / 0.20 m | 0.07 / 0.16 m |
| broadcast, clocks perfect | 0.07 / 0.13 | 0.06 / 0.14 |
| broadcast, sync 0.3 m, absolute lengths | 0.17 / 0.15 | 0.12 / 0.13 |
| broadcast, sync 0.3 m, excess delay | 0.09 / 0.19 | 0.09 / 0.21 |
| broadcast, sync 1.0 m, absolute lengths | 0.60 / 0.79 | 0.45 / 0.58 |
| broadcast, sync 1.0 m, excess delay | 0.09 / 0.19 | 0.09 / 0.16 |

- **For static reflectors, broadcast does not beat pairwise.** Ten nodes find all nine
  within 1 s of slots pairwise already (and the scene is easy: every row finds 9/9).
- **Absolute lengths from a broadcast need sync better than ~0.1 m** to match
  pairwise (0.3 m of residual already costs 2× in horizontal error).
- **Excess delay (each later path relative to the first, the line of sight) is
  sync-free** and lands at about pairwise accuracy at any sync error. This needed the
  line-of-sight gate to widen with the sync uncertainty (`landmarks.los_gate`): at
  first I let a clock-shifted line of sight pass as a "reflection" and got false
  landmarks, which the study caught.
- **Fader mapping is where more observations per slot help**, and it needs no clock
  sync at all (it uses link power). On the fleet, all 8 faders found in every case;
  at 4 s of slots the horizontal error is 1.60 m pairwise vs 0.41 m broadcast, at 12 s
  0.36 vs 0.31 m; broadcast gave 2–4 false faders in 4 seeds where pairwise gave 0.
  Two things had to be right for this: the expected line-of-sight SNR is corrected for
  the receiving patch's gain (otherwise a grazing link looks shadowed), and the static
  nodes' yaw is calibrated first (otherwise that correction is wrong).
- Not run: P2 (tracking the gauge) with broadcast; whether its shorter windows would
  help against the drift lag is the open question there.

**Verdict:** not a clear win. Broadcast buys airtime, and airtime is not what limits
static-reflector mapping here. It is useful for faders and probably for anything that
must converge fast, and the sync-free forms (excess delay, link power) get that
without solving clock sync. The number that decides whether absolute TDoA is worth
pursuing is the real clock residual, which
[uwb-sync-experiment.md](uwb-sync-experiment.md) measures on the boards you have.

## Clock sync and TDoA

Today the mesh does **pairwise DS-TWR only**: one exchange per TDMA slot, 100 slots/s
network-wide (design §5.3). The POLL is already overheard by everyone (to refresh
adverts), but no receive timestamps are combined across nodes, and a "clock sync
budget" is still listed as future work (§9). The multipath evidence itself needs no
sync: a reflection's excess delay over the first path at the same receiver comes from
one CIR on one clock. What sync buys is absolute path lengths from a single
transmission, at ~0.25 ns (7 cm) to match today's shared range error; 1 ns is 30 cm.

## Run it

```sh
python3 -m unittest swarm.sim.landmarks_test          # ~10 s, 28 tests
python3 -m swarm.sim.landmarks_run --broadcast --rho 0.5 --airtimes 1 4   # fleet study, ~15 min
python3 -m swarm.sim.landmarks_run                    # P1 over the tiers
python3 -m swarm.sim.landmarks_run --p2 --study       # P2 error budget, ~5 min
```
