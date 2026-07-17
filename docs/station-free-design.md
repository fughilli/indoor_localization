# Station-free cooperative localization — system design

_Status: v1, grounded in the `//swarm` simulation (2026-07-17). Companion
code: `swarm/core` (Rust estimator), `swarm/sim` (simulation + viewer)._

## 1. Goal

Swarms of widgets localize off one another with no fixed infrastructure.
Any widget that knows its absolute position (GPS lock, surveyed placement)
anchors the network; every other widget converges to a fast-updating absolute
pose estimate. **With no anchored widget at all, the swarm still converges to
a consistent relative solution**: per-widget absolute attitude comes from
gravity+magnetometer, the mesh's global orientation is grounded through AoA,
and the unobservable common translation is pinned by bleeding the swarm
velocity to zero (valid while the shared reference frame is not accelerating
appreciably). Demonstrated in simulation (§7): 5–10 IMU+UWB-only mobile
widgets reach **7–14 cm absolute RMSE** with three surveyed peers, and
**~0.5 m relative RMSE with zero anchors**.

## 2. Widget hardware model

| Component | Role | Assumed class |
|---|---|---|
| 9-axis IMU (acc/gyro/mag) | 100 Hz pose propagation + heading | consumer MEMS (ICM-42688-class) |
| UWB radio | two-way ranging + AoA + short broadcasts | DW3000-class, σ≈10 cm |
| WiFi / BLE | high-bandwidth backhaul, telemetry | — |
| GPS (optional) | absolute anchor | σ≈1.5 m horizontal |
| Altimeter (optional) | absolute z | σ≈0.3 m |

Each widget knows the relative pose of its own components (IMU frame w.r.t.
UWB antenna); the simulation currently treats them as co-located — the
extrinsic transform is a fixed known rotation/offset applied at measurement
ingest and does not change the architecture.

## 3. Software architecture

```
             ┌────────────────────────── widget ──────────────────────────┐
             │                                                            │
  IMU 100Hz ─┼─► predict ─┐                                               │
  mag 20Hz ──┼─► update ──┤   ESKF (swarm/core, Rust, no alloc)           │
  GPS 1Hz ───┼─► update ──┼─►  15-state: p v q ba bg + 15x15 cov          │
  alt 10Hz ──┼─► update ──┤        │                                      │
             │            │        ├─► pose+vel+quality ──► advertise     │
  UWB slots ─┼─► range/AoA┘        │        (10 Hz broadcast)             │
             │   (peer adverts     └─► application (position out)         │
             │    feed nvar)                                              │
             └────────────────────────────────────────────────────────────┘
```

The estimator is a self-contained Rust library with **no crate dependencies,
no allocation in the hot path, and no matrix inversion** (sequential scalar
updates only), so the identical code runs on the embedded target. The
simulation drives it through a C ABI (`swarm/core/src/ffi.rs`) — the same
ABI an embedded C/C++ shell would use.

## 4. Estimation algorithm

### 4.1 Filter

Error-state Kalman filter per widget. Nominal state: position `p`, velocity
`v` (world ENU, m), attitude quaternion `q` (body→world), accel bias `ba`,
gyro bias `bg`. 15-dim error state `[δp δv δθ δba δbg]`, attitude error
body-side (`R_true = R_est·exp([δθ]×)`).

- **Propagation** (per IMU sample): standard strapdown; first-order discrete
  transition; process noise from continuous densities.
- **Updates** (all sequential scalar, each behind a 5σ innovation gate):
  - UWB range to peer: `h = |p−p_peer|`, noise `σ_uwb² + nvar_peer`.
  - UWB AoA: measured body-frame bearing unit vector, three scalar
    components; couples `δp` and `δθ`; peer noise projects as `nvar/r²`.
  - GPS: direct position; altimeter: direct z.
  - Magnetometer + quasi-static gravity: body-frame direction observations
    (world field / world up), Jacobian `[u_pred]×` on `δθ`.
- **Init**: TRIAD attitude from one accel+mag sample; position "anywhere"
  (σ=100 m) unless surveyed.

### 4.2 Cooperative fusion rules (the part that makes swarms work)

Every widget advertises `(p, v, quality, flags)` where quality = mean
position variance (trace(P_pos)/3); consumers dead-reckon a stale
advertisement forward with its `v`. On receiving a range/AoA measurement, a
widget classifies the peer by relative quality and applies one of three
treatments (all discovered/validated by simulation, §7.4):

1. **Downhill — full Kalman update** when `nvar_peer < 0.5 × my_var` (peer
   clearly better-localized). The peer variance folds into the measurement
   noise, floored at `0.5 × my_var` (**cold-start damping**): a cold widget
   (σ=100 m) hit by a tight anchor range would otherwise take a
   near-unity-gain jump from a garbage linearization point — observed to
   diverge to km scale. The floor turns acquisition into bounded,
   re-linearized steps (damped Gauss-Newton); no separate acquisition state
   machine is needed.
2. **Comparable — mean-only update** when quality is within [0.5×, 2×] of
   mine: the state moves by K·y but the covariance is untouched, so no
   information gain is ever claimed from mutually-derived measurements.
   This is what makes the anchorless swarm work: estimates relax onto a
   mutually consistent mesh while covariance honestly tracks the
   unobservable absolute frame. Within this regime, range updates are
   damped against my own variance (gain ≈ 2/3), while **AoA gain is divided
   by a further ~30×**: the mesh shape is already range-rigid, and AoA's
   only anchorless job is steering the slowly-drifting global mesh
   orientation — at relaxation gain its ~7° noise (≈1.8 m tangential at
   15 m) shakes the mesh instead.
3. **Uphill — ignore** when the peer is clearly worse; relaxing toward it
   only drags me off. Additionally, an **anchored widget (healthy absolute
   source; `flags`) never takes mean-only pulls**: the 33 Hz consensus
   relaxation would overpower its 1 Hz GPS and drag the swarm gauge off
   truth (observed: 6 m absolute error vs 1.5 m with the rule).

Two gauge mechanisms complete the anchorless mode:

- **Zero-velocity bleed**: every widget applies a weak zero-velocity
  pseudo-measurement (σ=3 m/s at 2 Hz). The swarm's common-mode velocity —
  unobservable without anchors — is bled to zero, pinning the swarm frame
  to quasi-static. Valid while the shared reference frame is not
  accelerating appreciably; the sigma sits far above real widget speeds so
  genuine motion is negligibly biased.
- **Orientation grounding**: gravity + magnetometer give each widget
  absolute attitude; AoA bearings (measured in the body frame, mapped
  through that attitude) tie the *relative position mesh* to those
  attitudes. Without AoA the mesh shape stays good but its global
  orientation random-walks (ranges are rotation-invariant): measured 3–20 m
  translation-aligned error vs 0.5 m rotation-aligned.

Dead ends, documented so nobody retries them (§7.4): naive mutual updates,
whole-state scalar covariance intersection, undamped cold-start updates,
full-gain mean-only AoA, per-widget adaptive innovation gating, and
mean-only pulls on anchored widgets.

### 4.3 Observability notes

- Gravity + magnetometer make attitude fully observable when quasi-static;
  motion + UWB does the rest.
- One anchored widget fixes translation; absolute yaw comes from the shared
  magnetic field, z from altimeters. Three non-collinear anchored widgets
  make the whole swarm absolutely observable without magnetometer trust.
- With zero anchored widgets, absolute position is unobservable by design.
  The relative mesh, mesh orientation, and per-widget attitude remain
  observable (ranges + AoA + gravity + mag); the 3-DOF translation gauge is
  pinned by the zero-velocity bleed and wanders slowly. The filters stay
  numerically sane with honestly large absolute covariance
  (`sim_test.py`).

## 5. Protocol design

### 5.1 Media

- **UWB**: ranging slots + the pose advertisement piggybacked on ranging
  frames. Payloads kept minimal to preserve air time.
- **BLE (advertising) or WiFi**: redundant advertisement channel and any
  high-bandwidth traffic (logs, config). Nothing in the estimation loop
  requires more than the UWB payloads below.

### 5.2 Pose advertisement (broadcast, 10 Hz per widget)

| Field | Type | Bytes | Notes |
|---|---|---|---|
| widget id | u16 | 2 | |
| seq | u16 | 2 | staleness detection |
| p (ENU, m) | 3×f32 | 12 | |
| v (m/s) | 3×f32 | 12 | prediction between broadcasts |
| quality | f32 | 4 | mean position variance, m² |
| flags | u8 | 1 | anchored / GPS-fix / static |
| **total** | | **33 B** | fits one UWB MAC frame |

Simulation shows the loop is insensitive to advertisement rate down to 2 Hz
(§7.3) because the estimator consumes advertised *variance*, and static /
slow peers dominate the downhill flow. 10 Hz is chosen to bound staleness
error at ≤0.1 s × v. On the wire this is the `widget.v1.PoseAdvertisement`
protobuf message (§11), ~40 B encoded with micropb on-device.

### 5.3 Ranging exchange (per TDMA slot)

Double-sided two-way ranging (DS-TWR), initiator A / responder B; both ends
compute the same range; each end measures AoA of the incoming frame if
equipped. The final message carries A's current advertisement; the response
carries B's — so every ranging pair also refreshes each other's pose.

```
A                                   B
│ ── POLL (id, seq) ──────────────► │  t_rx, AoA@B
│ ◄────── RESPONSE (B advert) ───── │  t_rx, AoA@A
│ ── FINAL (A advert, timestamps) ► │
│                                   │  B computes range, updates its ESKF
│  A computes range, updates ESKF   │
```

### 5.4 Medium access

- TDMA, `uwb_slot_rate = 100` slots/s network-wide; one DS-TWR exchange per
  slot (~1–2 ms air time at 6.8 Mb/s — comfortable).
- Round-robin over all pairs in radio range. N widgets → N(N−1)/2 pairs;
  13 widgets ≈ 78 pairs ≈ 1.3 Hz per pair, which still yields 9–14 cm
  (§7.2). A production schedule should be distributed (e.g. slotted-ALOHA
  with id-hashed slot pick, or token passing); the simulation's centralized
  round-robin is an idealization of the same average rate.
- Degradation is graceful: cutting the slot rate 4× (25/s) roughly ×1.5s
  the error and ×10 the convergence time (§7.3).

## 6. System requirements

Grounded in the sensitivity sweeps (§7.3); "req" = the point where the
system still meets 2× its baseline error.

| Quantity | Baseline (sim) | Requirement |
|---|---|---|
| UWB range noise (1σ) | 0.10 m | ≤ 0.30 m (error scales ~linearly) |
| UWB multipath | 5% outliers, +Exp(1 m) | gated (anchored regimes); absorbed (~0.15 m cost) in anchorless — see §9 NLOS |
| Ranging rate per widget | ~25/s | ≥ ~6/s (25 slots/s ÷ 8 widgets) |
| AoA | σ≈7° | **optional** — no measurable gain in range-rich geometry; keep for sparse geometries |
| IMU accel / gyro noise | 0.05 m/s², 0.002 rad/s per sample @100 Hz | consumer MEMS suffices |
| IMU bias stability | 0.05 m/s², 0.01 rad/s initial; RW tracked | |
| Magnetometer | σ≈3° on field direction | needed for absolute yaw |
| Altimeter | σ≈0.3 m | recommended (z observability in flat swarms) |
| Advertisement rate | 10 Hz | ≥ 2 Hz measured equivalent |
| Anchors for cm-level | 3 surveyed (or RTK-grade) | ≥ 3 non-collinear |
| Anchors for m-level | 2 plain GPS widgets | ≥ 1 (absolute error ~ GPS σ) |
| Anchorless (relative-only) | 0 anchors | AoA + mag REQUIRED (mesh orientation); ~0.5 m relative; frame must not accelerate (zero-vel gauge) |
| CPU (predict @100 Hz + updates) | ~0.9 MFLOP/s (2×15³ MAC ×100 Hz + scalar updates) | ESP32 (240 MHz, FPU) ample |
| RAM per estimator | ~1.1 kB (15×15 f32 cov + state) | |

Key physics constraint: **absolute** accuracy is bounded by the best
absolute reference in the swarm (GPS σ≈1.5 m ⇒ swarm ≈0.7–2.7 m; surveyed
anchors ⇒ cm-level). UWB tightens the swarm *relative* to its anchors.

## 7. Experimental results

All runs: seed 1234, 90 s, 30 m arena, sensors per §6 baseline,
`bazel run //swarm/sim:run_sim`. Tail RMSE = RMSE over the last 22.5 s.

### 7.1 Headline scenarios

| Scenario | Widgets | Absolute tail RMSE | Relative tail RMSE | Convergence |
|---|---|---|---|---|
| 3 surveyed + 5 mobile (no GPS) | 8 | **0.06–0.08 m** | 0.05–0.07 m | ~1 s |
| 2 GPS + 4 plain mobile | 6 | 1.4–2.1 m | 0.4–0.6 m (rot-aligned) | tens of s |
| **0 anchors** (6 mobile) | 6 | ~25 m stable gauge offset (unobservable) | **0.46–0.59 m** | seconds |

Relative RMSE = error after removing the best common translation; the
rotation-aligned (Kabsch) variant separates mesh-orientation error from mesh
deformation — in the final configuration they agree (orientation pinned).

### 7.2 Scaling

3 surveyed + 10 mobile (78 ranging pairs, ~1.3 Hz/pair): 0.09–0.14 m,
convergence 1–10 s.

### 7.3 Sensitivity (3 surveyed + 5 mobile)

| Variant | Tail RMSE | Note |
|---|---|---|
| baseline | 0.07–0.10 m | |
| AoA disabled | 0.075–0.098 m | AoA not load-bearing here |
| slot rate 100→25/s | 0.12–0.15 m | convergence 5–15 s |
| range σ 0.10→0.30 m | 0.14–0.21 m | ~linear in range noise |
| broadcast 10→2 Hz | 0.07–0.11 m | insensitive |

### 7.4 Negative results (do not retry)

| Approach | Failure |
|---|---|
| naive mutual KF range updates | covariance collapse <1 s → gate rejects truth → unbounded consensus drift |
| whole-state scalar CI | exponential covariance blowup in unobserved directions → f32 overflow |
| undamped updates from tight anchors on cold start | unity-gain snaps at wrong linearization → km-scale divergence |
| multilateration-only cold start (≥4 tight peers) | bootstrap deadlock when <4 anchored peers exist |
| anchorless: peer-var-deweighted AoA | AoA weight → 0 (everyone's var honestly huge); rotation info lost; but see next row |
| anchorless: full-gain mean-only AoA | ~7° bearing noise × 15 m × gain 2/3 shakes the mesh to ~2 m; fix: AoA gain ÷30 (orientation steering only) |
| anchorless: per-widget adaptive innovation gate (EWMA) | pair endpoints gate the shared range asymmetrically; thousands of one-sided half-updates pump mesh drift (~10× worse) |
| mean-only consensus pulls on GPS-anchored widgets | 33 Hz mesh relaxation overpowers 1 Hz GPS → swarm gauge dragged ~6 m off truth |

## 8. Porting plan (embedded)

1. `swarm_core` builds as a `rust_library` with no deps; embedded shell
   links it directly (or via the existing C ABI). Replace `std` float
   intrinsics with `libm` and add `#![no_std]` — the code already avoids
   allocation and dynamic dispatch.
2. Firmware layering mirrors the sim's `World`/`SimWidget` split: a thin
   scheduler feeds the estimator from IMU/UWB ISRs; the downhill +
   damping rules live in the same network layer that parses advertisements.
3. DW3000 driver: extend `libs/dw3000` with DS-TWR + payload piggyback and
   (hardware permitting) PDoA/AoA.
4. Validation path: replay recorded hardware sensor logs through the *same*
   Rust core in the sim harness before flashing.

## 9. Future work

- **Split covariance intersection** for sub-anchor-variance mutual
  refinement (the principled fix CI couldn't deliver whole-state).
- Relative-state formulation (estimate swarm-frame + frame-to-world
  separately) to decouple cm relative precision from m-level absolute
  wander in GPS-only swarms.
- NLOS/multipath classification from UWB CIR: statistical gating is blind
  in the mean-only regime (S is dominated by the honest absolute variance)
  and per-widget adaptive gating failed (§7.4) — costs ~0.15 m relative in
  anchorless swarms today. A pair-level gate (both endpoints agree on the
  same accept/reject decision from the shared measurement) is the next
  cheap thing to try.
- Distributed TDMA (id-hashed slot selection) + clock sync budget.
- Full 6-DOF truth trajectories (current sim: yaw-only attitude).
- Component extrinsics (IMU↔antenna lever arm) in the measurement models.

## 10. Sensor characterization (swarm/tools)

Before the fusion parameters (§6) can be trusted on hardware, each sensor is
characterized against ground truth with `//swarm/tools:characterize`. The
firmware needs only a *capture mode*: stream raw sensor records as newline
JSON (schema in `swarm/tools/records.py`) over the existing WiFi/TCP path;
ground truth lives in a session manifest describing the physical rig. Every
analysis is validated closed-loop against synthetic captures with known
truth (`analysis_test.py`), and `--demo` dry-runs any pipeline today.

| Analysis | Rig | What it reports |
|---|---|---|
| `aoa` | peer fixed at known position; DUT on a turntable, captures at discrete known angles across several elevations; manifest gives `true_dir_body` per segment from rig geometry | **which body-frame plane the reported angle is measured in** (fitted plane normal), sign convention, mounting offset, precision σ, and σ binned by out-of-plane angle (PDoA degradation off-axis) |
| `range` | static captures at tape-measured distances (e.g. 2/5/10/20/40 m) | per-distance bias + σ + outlier fraction (robust MAD gate), linear error model `err(d) = bias0 + scale·d` (antenna delay ↔ bias0, clock trim ↔ scale) |
| `imu` | widget stationary ≥10 min | per-axis Allan deviation → white-noise density + bias instability (these feed `Params` directly), gyro bias, accel scale error via the gravity-magnitude check |
| `mag` | rotate the widget through all orientations | hard-iron offset + unit-determinant soft-iron matrix (ellipsoid fit), residual field spread after correction |

The AoA plane fit is the piece that answers the mounting question: it
searches all candidate plane normals for the one where the reported angle
best matches `sign · azimuth(direction projected into plane) + offset`,
weighting out near-normal directions where azimuth is unobservable. On the
synthetic reference it recovers a plane normal to <1°, the offset to
<0.01 rad, and the σ within noise.

## 11. Widget configuration standard (widget.v1 protobuf)

The widget design is open for third-party manufacturing/customization, so
per-widget properties are configuration with a standard schema, not code.
**All firmware configuration and data transport/storage uses protocol
buffers** — one schema, `swarm/proto/widget_config.proto` (`widget.v1`),
consumed in three forms:

- **textproto** (`.txtpb`) on disk — the human-authored source of truth,
- **canonical proto JSON** — the configurator's HTTP API,
- **binary proto** — firmware storage and transport, decoded on-device by
  no_std Rust bindings generated with **micropb** (heapless containers, no
  allocation), plus a generated C byte-array header wrapping the same bytes
  for build-time baking.

The codegen base (hermetic prebuilt protoc, the micropb runtime + host-side
generator, and the Bazel macros) lives in the shared `firmware` module
(`@firmware//proto`, embedded repo branch `micropb-support`, ported from
led_mapper's bring-up) so every project in that ecosystem shares one
implementation. A genrule encodes the example config with canonical Python
protobuf and the Rust conformance test decodes it through micropb and
**re-encodes to the identical bytes** — byte parity pins presence rules and
packed encoding on both sides, in CI.

Schema contents:

- `Extrinsics` — *where every sensor sits*: per sensor an axis mapping +
  fine Euler trim + offset (human-authorable) and a derived canonical
  sensor→widget quaternion (`v_widget = R·v_sensor + offset`). Standard
  sensors imu/mag/uwb/gps/alt (ids 1–5); custom sensors use ids ≥128.
  Authored graphically with `//swarm/tools:extrinsics_configurator`
  (port 8082): live 3D preview, validation, one-click header/binpb export.
- `Intrinsics` — *how good every sensor is*: exactly what
  `//swarm/tools:characterize` measures, optionally shipped per widget
  model or serial. proto3 explicit presence replaces "NaN = use default":
  absent message/field means the firmware default.
  `characterize <kind> --intrinsics file.txtpb` merges measurement results
  straight into a config; `run_sim --intrinsics` maps it onto the estimator
  noise model (the same mapping firmware applies to its `Params`).
- `PoseAdvertisement` — the over-the-air broadcast of §5.2 as a proto
  message (~40 B encoded), so the wire format is pinned by the same
  conformance machinery before firmware exists.

Superseded (removed): the hand-rolled WEXT/WINT CRC'd binary containers and
JSON authoring format from the first iteration of this standard.
