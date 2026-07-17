# WORKLOG

_Last updated: 2026-07-17 (session 3, station-free build-out + anchorless
mode). Read together with `git log`._

## Goal

Two stacked scopes:
1. **Done** — CircuitDigest ESP32+DWM3000 RTLS (3 anchors + 1 tag + host
   trilateration/web view); build-only until hardware arrives (sessions 1–2).
2. **Active** — `station-free.md`: station-free cooperative swarm
   localization. Rust ESKF per widget (IMU + UWB range/AoA + GPS/alt/mag),
   simulation-driven development, browser viewer, design doc. All Bazel+Nix.

## State of play

**Station-free v1 is complete in simulation** (this session):

- `swarm/core` — Rust (rules_rust 0.71.3, Rust 1.84.0, zero crate deps,
  no-alloc, scalar-updates-only) 15-state ESKF + C ABI cdylib. Unit tests.
- `swarm/sim` — Python sim (sensor noise/bias/multipath models, TDMA
  ranging, 10 Hz pose-quality broadcasts, TRIAD init) driving one Rust
  estimator per widget via ctypes; `run_sim` metrics CLI with sweep flags;
  `viewer` = SSE+canvas browser view on :8081 (port in overlay.json).
- **Headline results** (locked in `sim_test`): 3 surveyed anchors + 5–10
  IMU+UWB-only mobiles → **6–14 cm tail RMSE, converge ≤10 s**. 2-GPS
  scenario: 1.4–2.1 m absolute (GPS-noise physics), ~0.5 m relative. **Zero
  anchors → 0.46–0.59 m relative RMSE** with a stable (unobservable,
  zero-vel-pinned) gauge offset. Full tables in
  `docs/station-free-design.md` §7.
- **Anchorless mode** (user request, this session): mean-only (covariance-
  preserving) relative updates in the comparable-quality regime, three-band
  peer classification (downhill full / comparable mean-only / uphill
  ignore), anchored widgets exempt from consensus pulls, zero-velocity
  gauge bleed (σ3 m/s @ 2 Hz), mean-only AoA gain ÷30 (orientation steering
  only), advertisement dead-reckoning. Relative + Kabsch rotation-aligned
  metrics in run_sim; "relative view" checkbox in the viewer.
- `docs/station-free-design.md` — architecture, protocol (33 B advert @
  10 Hz, DS-TWR piggyback, TDMA), requirements grounded in sweeps, porting
  plan, negative results, characterization methodology (§10).
- `swarm/tools` — sensor characterization (user request): capture JSONL
  schema (`records.py`, what the firmware capture mode must emit) +
  manifest ground truth; analyses = AoA plane/sign/offset/σ fit (answers
  "which plane is AoA measured in") with out-of-plane degradation bins,
  range bias/σ/outliers + linear error model, IMU Allan deviation
  (noise density/bias instability feed estimator Params), mag hard/soft-
  iron ellipsoid fit. All validated closed-loop vs synthetic captures
  (`analysis_test.py`); every subcommand has `--demo`.
- All cooperative-fusion rules live in the sim's network layer
  (`world.py::_run_slot`) + `Estimator::update_zero_vel`; design doc §4.2
  is the authoritative description of the regime logic.

## Next up

1. **Embedded port** of `swarm_core` per design doc §8: `#![no_std]` + libm,
   link into firmware; extend `libs/dw3000` with DS-TWR payload piggyback.
   Also add a firmware *capture mode* emitting the `swarm/tools/records.py`
   JSONL schema over TCP so `characterize` runs against real sensors.
2. Algorithm hill-climbs (design doc §9): split-CI for mutual refinement,
   relative-frame formulation, NLOS classification, distributed TDMA.
3. Sim realism: full 6-DOF truth attitude (now yaw-only), IMU↔antenna
   extrinsics, bias random-walk in truth (now constant per run).
4. (Carried over) RTLS hardware bring-up per README §Bring-up when boards
   arrive; merge embedded-repo branch `uwb-esp32-wifi` after hardware proof.
5. (Carried over) Root-repo force-push to origin still **pending user
   confirmation** (history rewrite from 2026-07-17); now ~9 unpushed commits.

## Open questions / blockers

- Hardware still not attached (blocks RTLS bring-up and swarm port testing).
- Force-push confirmation (above).
- cm-level *absolute* accuracy requires ≥3 surveyed/RTK-grade anchored
  widgets; plain GPS anchors give ~1–3 m absolute. Relative-frame split
  (design doc §9) is the route to cm-relative under GPS-only anchoring —
  user input welcome on which matters more.

## Don't retry (dead ends)

- Session 1–2 items: runtime `apt-get` without overlay; HTTPS clone of
  private embedded repo; nixpkgs-esp-dev/IDF CMake; removing
  `third_party/embedded` from `.bazelignore`.
- **Naive mutual KF range updates between widgets** — covariance collapses
  <1 s, gate then rejects truth, swarm consensus-drifts unboundedly.
- **Full-gain mean-only AoA** (anchorless) — 7° bearing noise at relaxation
  gain shakes the mesh to ~2 m; AoA must be heavily damped (÷30), its job
  is only steering global mesh orientation.
- **Peer-var-deweighted AoA in mean-only regime** — weight vanishes
  (everyone honestly uncertain), mesh orientation random-walks 3–20 m.
- **Per-widget adaptive innovation gate (EWMA) for mean-only ranges** —
  pair endpoints gate the shared range asymmetrically; one-sided
  half-updates pump ~10× mesh drift. If retried, must be pair-level
  (both endpoints share the accept/reject decision).
- **Letting GPS-anchored widgets take mean-only consensus pulls** — 33 Hz
  mesh relaxation overpowers 1 Hz GPS, drags the swarm gauge ~6 m off.
- **Whole-state scalar covariance intersection** in the ESKF — 1/w prior
  inflation compounds exponentially in unobserved directions (biases/vel)
  → f32 overflow. If revisited, it must be split-CI on the position block.
- **Undamped tight-anchor updates on a cold (σ=100 m) filter** — unity-gain
  snap at garbage linearization → km-scale divergence.
- **Multilateration-only cold start requiring ≥4 tight peers** — deadlocks
  when fewer anchored peers exist (2-GPS scenario). The var-floor damping
  subsumes it with no state machine.
- `Estimator.__init__` in `swarm/sim/estimator.py` treats 0.0 as "use Rust
  default" — pass `gate=-1`, not 0, to disable gating.
