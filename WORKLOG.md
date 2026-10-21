# WORKLOG

_Last updated: 2026-10-20 (session 5, hardware bring-up + embedded port).
Read together with `git log`._

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
  `viewer` = SSE+canvas browser view on :8181 (port in overlay.json).
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
- **widget.v1 protobuf config standard** (user request; supersedes the
  earlier WEXT/WINT/JSON iteration): all firmware config/transport/storage
  is protocol buffers. Schema `swarm/proto/widget_config.proto`
  (Extrinsics + Intrinsics + PoseAdvertisement); forms: .txtpb source of
  truth, proto JSON over the configurator HTTP API, binary proto for
  firmware (no_std micropb bindings via `//swarm/config`; C byte-array
  header for baking). Codegen base is SHARED in the embedded repo
  (`@firmware//proto`, branch `micropb-support`, pushed; ported from
  led_mapper feat/esp32-led-mapping per user direction — dedup led_mapper
  onto it later). Byte-parity conformance test (Python canonical encode ->
  micropb decode -> identical re-encode) in CI. Root Rust bumped to
  1.85.0 to match; pip dep `protobuf==6.33.5`. Design doc §11.
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

**Session 4 (2026-07-19): innovation-annealed hold + join/leave** (user
request: post-convergence jitter in the zero-gps/zero-survey view; devices
joining cold / leaving):

- Root cause of the jitter: mean-only updates deliberately never shrink
  covariance (absolute frame unobservable), so the relaxation gain stays at
  cold-start ~2/3 forever and every 10 cm-noise range (+5% multipath)
  kicks the state — ~0.5 m/frame at 10 Hz. The filter *cannot* anneal this
  natively; `pos_var` conflates common-mode (huge, correct) with mesh
  quality (converging, untracked).
- Fix: `Estimator::range_innov_ratio` (Rust, FFI getter, reset by
  `set_pose`) = capped EWMA of |range innovation|/σ — a self-measured
  **lock** signal (~1 locked, 10 none). `world.py` interpolates mean-only
  range damping 0.5 → 30 (K ⅔ → ~3%) as lock falls 6 → 2; innovations
  re-inflate on real motion, so the gain self-recovers. **Static anchorless:
  jitter 0.48–0.60 → 0.14–0.18 m/frame, rel RMSE 0.42–0.62 → 0.25–0.30 m.**
  Damping >30 trades rel accuracy for little jitter; kept 30.
- Lock also **replaces variance for peer ranking** (variance can't rank
  anchorless peers — see dead ends): advert now carries lock (surveyed/GPS
  advertise 1.0); downhill requires peer lock < 6; mean-only pulls from a
  ≥3×-worse-locked peer are refused (protects mesh from cold joiners);
  adverts older than 1 s are ignored (departed widgets).
- Join/leave: `World.add_widget` (cold at current t) / `remove_widget`;
  `last_links` carries wids not indices; viewer `--join-at/--leave-at/
  --leave-id` + per-widget lock in the info panel; `run_sim` adds
  `tail_jitter_m`, `lock`, `--range-damping` (0 = annealing off, for A/B).
- New `sim_test` cases (all 8 green): annealing jitter A/B, cold join pulls
  in ≤1.5 m rel without shaking veterans, anchor loss → honest variance
  regrowth with mesh held, stale-advert aging. Rust: `innov_ratio_tracks_lock`.

**Session 5 (2026-10-19/20): hardware + embedded port.**

- Hardware: ACEBOTT ESP32-Max-V1.0 (ESP32-WROOM-32) + Qorvo DWM3000EVB
  shield. UNO-position -> GPIO map in `apps/mule/pins.h` (SPI on 5/16/17,
  CS 18, RSTn 12 — a strapping pin, boots fine). IMU (MPU6050 + QMC5883L on
  its aux bus) goes on SDA GPIO27 / SCL GPIO4: 34/35 are input-only, and the
  shield owns 32/33 (EXTON/DW GPIO7), 12-14 and 25/26 (RSTn, DW GPIO0-3). DWM3000EVB has ONE antenna: no AoA on this hardware.
- `apps/mule`: bring-up + 2-board DS-TWR test (initiator/responder images,
  `probe`/`diag` tools); `swarm/tools/capture.py` + `range_view.py` (live
  dashboard, segment recording for `characterize range`). Found+fixed: the
  initiator wedged forever after a responder outage (RX left on + stale TX
  flag) — idle the radio before each exchange.
- **esp-rs**: the core stays Rust on the classic ESP32 (user direction). Nix
  prebuilds an Xtensa sysroot from the esp-rs 1.85.0.0 release; rules_rust
  toolchain + triple patch in the firmware module (design doc §8).
  `swarm_core` is no_std + libm; cooperative rules moved into
  `swarm_core::coop` (sim byte-identical); `swarm/node` runtime;
  `apps/node` firmware (no roles: TDMA rank slots, lowest-id sync, 5-frame
  DS-TWR, adverts on every POLL); `swarm/tools/swarm_view.py`.
- **Session 6 (2026-10-20), on hardware**: 4 nodes with IMUs (ids 53876,
  63124, 7408, 49736) range all-to-all at the full TDMA rate. Fixed a
  receiver re-arm that killed the next slot's POLL (36% timeouts -> 0).
  Anchorless swarms: centroid bleed to the origin (node.rs) and
  position-only mean-only range updates (`CoopConfig::mean_only_pos_only`;
  a still node had reached 41 m/s). A carried node still zigzags:
  `zero_vel_keep_position` removes the jumps but drifts ~5x more (test
  pins the trade-off) — tuning open.
- **Swarm network** (docs/swarm-network.md), all live on boards 53876 +
  63124:
  - `swarm/net` (Rust): Meshtastic-style managed-flood mesh (unicast +
    broadcast, attempt-aware dedupe, acks), live settings (swarm layer as
    a 4-byte-HMAC-tagged bundle with catch-up, per-device tagged
    overrides), and Deluge-style firmware pages with a summary-driven push
    fallback.
  - Crypto: MbedTLS on the device (boot KAT self-test), RustCrypto on the
    host. Firmware manifests are ECDSA P-256; keys are in ~/.config/swarm.
  - Flash: loader (factory) + main (ota_0) + staging + LittleFS. The loader
    installs and verifies; IDF rollback returns to it.
  - The radio uses spare TDMA slots for network frames.
  - Tools: `swarm/tools/swarm_ctl.py` (flash / key / settings / set / clear
    / push / status) and `swarm_wire.py`, whose encodings are pinned against
    Rust.
  - Measured: USB push of a 597 KB image 50 s; a neighbour fetched it over
    UWB in ~90 s and installed it; old (v3) firmware was pulled through by
    the fallback in ~11 min.
  - Embedded repo fix: `sdk_hdrs_esp32` was missing `MBEDTLS_CONFIG_FILE`
    (an ABI mismatch, so heap corruption in any MbedTLS user).

## Next up

1. **Swarm network**: all four boards are on the loader layout with keys,
   and v8 is deployed over the air. That took ~3.5 min from the USB push to
   all installed, with three receivers in parallel. Spare TDMA slots go to
   nodes with pages to serve (radio.cpp UpdateServers); with 4 nodes the
   rank rule had starved the servers.
   - Untested so far: the rollback path (a deliberately crashing main), the
     BOOT-button recovery, and multi-hop (all boards are within one hop).
   - Speed-ups: pre-erase pages (a 40 ms erase stalls the receiver), and
     shorten the request quiet time.
   - Then the anchorless mover tuning, now live via settings.
2. **Calibration**: survey >=3 nodes (no AoA ->
   anchorless can't fix orientation), range bias capture -> antenna delay,
   `characterize mag` -> `{"cmd":"magcal"}`, and IMU Allan deviation ->
   Params. Consider DW3000 IRQ instead of busy-polling, delayed-TX FINAL
   (drops the RESULT frame), and multi-hop advert relay for out-of-range
   peers (today one hop only). Also wire WEXT/WINT (widget.v1 extrinsics/
   intrinsics) into the node boot path.
2. Dedup led_mapper's feat/esp32-led-mapping micropb setup onto
   `@firmware//proto` (user direction); merge embedded `micropb-support`
   after led_mapper adopts it. Eventually this repo becomes a submodule of
   the embedded monorepo.
3. Algorithm hill-climbs (design doc §9): split-CI for mutual refinement,
   relative-frame formulation, NLOS classification, distributed TDMA.
4. Sim realism: full 6-DOF truth attitude (now yaw-only), IMU↔antenna
   extrinsics, bias random-walk in truth (now constant per run).
5. (Carried over) RTLS hardware bring-up per README §Bring-up when boards
   arrive; merge embedded-repo branch `uwb-esp32-wifi` after hardware proof.
6. (Carried over) Root-repo force-push to origin still **pending user
   confirmation** (history rewrite from 2026-07-17); now ~9 unpushed commits.

## Planned: AirTags as localizers

User direction (2026-10-20): reverse-engineer Apple AirTags as
localizers / anchors for this system, starting from seemoo-lab's work:
https://github.com/seemoo-lab/airtag (AirTag firmware research: nRF52832 +
U1 UWB chip; firmware dumping, patching, BLE/UWB behavior). Questions to
answer first:
- Can the U1 be driven to range with the DW3000 (both IEEE 802.15.4z HRP,
  but Apple's session setup is proprietary), or only tag-to-iPhone?
- Can patched AirTag firmware act as a swarm member or a fixed anchor
  (BLE for the advert/mesh side, UWB for ranging)?
- Power/size: AirTags as cheap surveyed anchors in place of 3 survey nodes.

## Open questions / blockers

- Hardware still not attached (blocks RTLS bring-up and swarm port testing).
- Force-push confirmation (above).
- cm-level *absolute* accuracy requires ≥3 surveyed/RTK-grade anchored
  widgets; plain GPS anchors give ~1–3 m absolute. Relative-frame split
  (design doc §9) is the route to cm-relative under GPS-only anchoring —
  user input welcome on which matters more.

## Housekeeping

- 2026-07-19: delete session-4 scratch files at the workspace root:
  `.bazel-robase`, `.bazel-install`, `.tmp`, `.run-viewer-ro.sh`,
  `.probe-rw.txt`. `.bazel-robase` is a broken partial extraction —
  hermetic python cannot extract onto the case-insensitive Mac volume,
  don't put a bazel output base there again. Re-run
  `bazel test //swarm/...` for a clean record, and relaunch the viewer:
  `bazelisk run //swarm/sim:viewer -- --widgets 6 --gps 0 --surveyed 0
  --join-at 45 --leave-at 90 --leave-id 0`.

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
- **Annealing the mean-only AoA gain with the range gain** (even by sqrt of
  the factor) — orientation pinning is AoA's only job there; the mesh
  slowly rotates (rel RMSE 3–5×, rot-rel stays tight). Anneal ranges only.
- **Variance-ranking anchorless peers** (the original uphill/downhill test
  vs a joiner) — a veteran's honest variance grows with process noise while
  a cold joiner's stays at its frozen init, so by variance the clueless
  joiner eventually outranks the converged mesh and veterans take full-gain
  updates toward its garbage advert. Rank mesh quality by advertised lock
  ratio; variance only qualifies genuinely-absolute sources.
- **micropb 0.5 + proto3 implicit packed repeated scalars** — micropb only
  honors the explicit descriptor option; annotate `[packed = true]` in the
  .proto or byte-parity with canonical protobuf breaks.
- **rules_rust crate universes in non-root modules** — must ship BOTH
  `lockfile` and `cargo_lockfile`; repin from the embedded repo as root.
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
