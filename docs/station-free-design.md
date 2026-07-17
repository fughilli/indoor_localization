# Station-free cooperative localization — system design

_Status: v1, grounded in the `//swarm` simulation (2026-07-17). Companion
code: `swarm/core` (Rust estimator), `swarm/sim` (simulation + viewer)._

## 1. Goal

Swarms of widgets localize off one another with no fixed infrastructure.
Any widget that knows its absolute position (GPS lock, surveyed placement)
anchors the network; every other widget converges to a fast-updating absolute
pose estimate. Demonstrated in simulation: **5–10 IMU+UWB-only mobile widgets
reach 7–14 cm position RMSE within seconds** of three position-aware peers
being present (§7).

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

Every widget advertises `(p, v, quality)` where quality = mean position
variance (trace(P_pos)/3). Peers fold that variance into the range/AoA
measurement noise. That alone is **not** enough; two additional rules are
load-bearing, both discovered by simulation (§7.4):

1. **Downhill rule** (network layer): accept a relative update only when
   `nvar_peer < 0.5 × my_var`. Information flows outward from anchored
   widgets and cannot circulate back. Without it, mutual updates
   double-count shared information: every filter collapses to σ≈0.1 m
   inside 1 s while errors are meters, the gate then rejects all real
   measurements, and the swarm consensus-drifts away unboundedly.
2. **Cold-start damping** (network layer): the effective peer variance is
   floored at `0.5 × my_var`. A cold widget (σ=100 m) hit by a tight
   anchor range would otherwise take a near-unity-gain jump from a garbage
   linearization point — observed to diverge to km scale. The floor turns
   acquisition into bounded, re-linearized steps (damped Gauss-Newton
   behavior); no separate acquisition state machine is needed.

Dead end, documented so nobody retries it: whole-state scalar **covariance
intersection** (prior deweighted 1/w per update). Consistent in theory, but
the 1/w inflation hits never-measured directions (biases, velocity) on every
relative update → exponential variance blowup → f32 overflow. A future
split-CI (inflate only the position block correlated with the swarm) is the
principled upgrade path if sub-peer-variance accuracy is ever needed.

### 4.3 Observability notes

- Gravity + magnetometer make attitude fully observable when quasi-static;
  motion + UWB does the rest.
- One anchored widget fixes translation; absolute yaw comes from the shared
  magnetic field, z from altimeters. Three non-collinear anchored widgets
  make the whole swarm absolutely observable without magnetometer trust.
- With zero anchored widgets, absolute position is unobservable by design;
  the filters must (and do — `sim_test.py`) stay numerically sane with
  honestly large covariance.

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
error at ≤0.1 s × v.

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
| UWB multipath | 5% outliers, +Exp(1 m) | handled by 5σ gate |
| Ranging rate per widget | ~25/s | ≥ ~6/s (25 slots/s ÷ 8 widgets) |
| AoA | σ≈7° | **optional** — no measurable gain in range-rich geometry; keep for sparse geometries |
| IMU accel / gyro noise | 0.05 m/s², 0.002 rad/s per sample @100 Hz | consumer MEMS suffices |
| IMU bias stability | 0.05 m/s², 0.01 rad/s initial; RW tracked | |
| Magnetometer | σ≈3° on field direction | needed for absolute yaw |
| Altimeter | σ≈0.3 m | recommended (z observability in flat swarms) |
| Advertisement rate | 10 Hz | ≥ 2 Hz measured equivalent |
| Anchors for cm-level | 3 surveyed (or RTK-grade) | ≥ 3 non-collinear |
| Anchors for m-level | 2 plain GPS widgets | ≥ 1 (absolute error ~ GPS σ) |
| CPU (predict @100 Hz + updates) | ~0.9 MFLOP/s (2×15³ MAC ×100 Hz + scalar updates) | ESP32 (240 MHz, FPU) ample |
| RAM per estimator | ~1.1 kB (15×15 f32 cov + state) | |

Key physics constraint: **absolute** accuracy is bounded by the best
absolute reference in the swarm (GPS σ≈1.5 m ⇒ swarm ≈0.7–2.7 m; surveyed
anchors ⇒ cm-level). UWB tightens the swarm *relative* to its anchors.

## 7. Experimental results

All runs: seed 1234, 90 s, 30 m arena, sensors per §6 baseline,
`bazel run //swarm/sim:run_sim`. Tail RMSE = RMSE over the last 22.5 s.

### 7.1 Headline scenarios

| Scenario | Widgets | Tail RMSE (mobile) | Convergence |
|---|---|---|---|
| 3 surveyed + 5 mobile (no GPS) | 8 | **0.07–0.10 m** | 0.3–1.1 s |
| 2 GPS + 4 plain mobile | 6 | 0.7–2.7 m | tens of s |
| 0 anchors (sanity) | 4 | unbounded abs. error, filters sane, σ honest | n/a |

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
- NLOS/multipath classification from UWB CIR instead of pure gating.
- Distributed TDMA (id-hashed slot selection) + clock sync budget.
- Full 6-DOF truth trajectories (current sim: yaw-only attitude).
- Component extrinsics (IMU↔antenna lever arm) in the measurement models.
