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
   - Rollback tested on hardware: hang and crash images both roll back
     to the loader, never spread (canary rule), and recover with a good
     push (docs/swarm-network.md "Canary rule"). Gotchas:
     - The prebuilt wdt_hal_* calls don't arm the RTC watchdog; use the
       inline rwdt_ll_*.
     - Arduino marks a pending app valid unless verifyRollbackLater()
       returns true.
   - **To fix: late failures.** A main that hangs or crashes *after*
     validating loops on itself forever: the image is valid, so no rollback.
     Needs a crash-loop guard: count consecutive short-lived boots (RTC
     no-init memory + reset reason), and after N of them invalidate the
     image and restart into the loader. A post-validation hang also needs
     the task watchdog on the radio/fusion/loop tasks (today it only
     watches core 0's idle task), so a hang becomes a counted reset.
   - **Nice to have: loader updates over the air.** The loader is USB-only
     today. FLAG_LOADER is reserved in the manifest; main would stage and
     install the loader image, then verify it before the next reset. Needs
     its own safety story, since a bad loader means no recovery path.
   - **Testing gaps** (none of these has been exercised on hardware):
     - BOOT-button (GPIO0) forced recovery.
     - Multi-hop: mesh relays, firmware pages and settings catch-up beyond
       one hop. All four boards are within range of each other; needs
       distance or attenuation, or an rssi-gated test mode.
     - Power loss during the loader's copy into main (it should redo the
       copy on the next boot).
     - Power loss mid-page while staging; corrupted staging; a manifest
       signed with the wrong key arriving over the air.
     - Two images pushed at once through different nodes (version race).
     - Settings: a bundle arriving while a node is mid-reassembly of an
       older one; an override sequence counter that went backwards (host
       state lost); key rotation (re-provisioning the HMAC key on all
       nodes).
     - Mesh: ack and retry under real loss, and a full outbox (send refused
       — seen once, unexercised since).
     - More than 6 nodes: spare slots disappear and the slot assignment
       wraps (MySlot % kSlots).
     - Long soak: hours of ranging plus OTA (LittleFS wear, heap, counter
       wrap).
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

## Solver on hardware (2026-10-20, 4 still boards, one roughly mid-triangle)

All tuned live through swarm settings and deployed over the air.
- Centroid bleed at 5 s distorted the mesh: each widget re-centers from its
  own view. With it off, lock went ~6 -> ~2. Default is now 60 s, x/y only
  in planar mode.
- `planar.on` (swarm setting, now on): an altitude pseudo-measurement at
  `planar.z`. A flat swarm is ill-conditioned in z: the middle widget
  popped out of plane either way. Now z = 0 and lock 0.2-0.5.
- Stationary detection (`still.*`, default on): ZUPT on velocity and biases
  only. Range updates are position-only, so nothing else pinned velocity;
  a still widget's velocity is now 0.001 m/s (was 2-4 cm/s).
- Left: the 7408 / 49736 / 53876 pairs still flex by ±12-28 cm while pairs
  with 63124 hold ±1-3 cm. In the fit those three are nearly collinear
  (~5°), which observes the across-line direction weakly. Lock also rises
  back to ~2 every few minutes; cause not yet known. Next: re-place 7408
  off that line, log all six ranges (rotate USB), calibrate antenna
  delays.

### Sim now runs the firmware's node runtime (2026-10-21)

`run_sim --scenario room|room_static` drives one swarm/node `Node` per widget
(host cdylib, swarm/node/src/sim_ffi.rs) with:
- an MPU6050-class IMU at 200 Hz: scale errors, bias random walk, a still
  boot period for calibration, and roll/pitch sway plus bob while carried;
- per-device range bias (uncalibrated antenna delay);
- TDMA-rate ranging, and adverts through the real wire format;
- metrics after best-fit (Kabsch) alignment, since the anchorless frame
  floats.

The ring scenarios (estimator backend) stay byte-identical. Findings that
went into v21, from 5-seed comparisons:
- Auto-anchoring at the old lock < 0.6 never triggers with range bias.
  At lock < 2.0, the walker's aligned error goes 22.8 -> 10.7 cm and its
  jitter 3x lower.
- An auto-anchor's ~5 cm sigma failed the variance-ratio downhill test, so
  a tracking widget fell back to position-only updates and never corrected
  its velocity. Anchored peers are now always downhill (PeerView.anchored).
- Gravity aiding only while still made walking worse (`gravity_still_only`
  setting kept, off).

Regression tests: sim_test RoomScenarioTest.

### Radio channel in the sim; blocked links; online range bias (v24, 2026-10-21)

Bench data (captures/swarm_v23.jsonl, 535k ranges): the DW3000's gap
between total and first-path received power predicts bad ranges.
- With a gap under 10 dB, 0.5% of ranges read more than 0.3 m long
  (residual against the solved mesh).
- With a gap over 10 dB, 15-30% do, by up to about 2 m, and almost never
  short.
- A carried board saw a gap over 15 dB on 52% of its ranges, against 31%
  still: the carrier's body.
- Two of six links were persistently over 12 dB, with a median residual
  of about 0.
- About 18% of POLLs time out.

The sim (`room_sensors`, `uwb_channel`) now models:
- the gap per range;
- blocked links (persistent, and through the carrier's body behind the
  heading) that read long: Exp(5 cm), plus 25% of the time Exp(0.8 m);
- dropped exchanges;
- 3 cm noise on clear-path ranges;
- an IMU reporting the mean specific force over each sample period, plus
  a smoothstepped pickup. The point-sampled pickup jerk was a 155 m/s²
  sample, a sim artifact.

**Metric change:** `tail_frame_rmse_m` is error after the still boards'
2-D rigid fit, with mirror handedness from the all-widget fit. The old
all-widget Kabsch spread a lost walker's error onto the statics and
flattered it: the walker read about 10 cm where it is really about 20 cm
on the old channel.

**Findings** (24 seeds, room, 120 s; walker / statics, static-frame RMSE):

| Configuration | Walker | Statics |
|---|---|---|
| v23 logic, old channel | 20.4 cm | 9.0 cm |
| v23 logic, new channel | 32.5 cm (max 1.55 m) | 12.1 cm |
| v24 defaults, new channel | 19.1 cm (max 54 cm) | 9.1 cm |

v24 defaults are also better on room_static (10.2 to 8.2 cm) and on the
old channel (walker 20.4 to 18.3 cm). One old-channel seed's walker hit
81 cm against a 35 cm max before; this is unexplained.

What worked, and what didn't:
- **Inflating sigma for flagged ranges (symmetric) was worse than doing
  nothing.** It let the long tail in past the innovation gate.
- **Rejecting flagged ranges was worse still.** A carried board loses a
  third of its links.
- **What works:**
  - widen sigma only when a flagged range reads longer than predicted
    (`nlos.asymmetric`), 0.05 m/dB above 10 dB, capped at 0.3 m;
  - clip a flagged range's excess at 5 cm while still (`nlos.clip_m`,
    Huber-style), so a still mesh doesn't average in the tail. Clipping
    movers hurt.
- **Divergence recovery** fired on consecutive gated ranges. A carried
  board with one clear link and two blocked ones sat at about 60% gated
  while 1-2 m off: the clear one kept resetting the count. It is now a
  window: regain when ≥ 50% of the last 16 ranges are gated.
- **Online range bias:** an estimator state `rb`, prior 0.1 m, advertised
  (PoseAdvertisement.range_bias). Peers subtract it.
  - A free widget learns it in full range updates.
  - An anchored one learns it from ranges to moving, locked peers
    (bias-only update) and re-anchors when it has moved 3 cm.
  - The walker goes 26.4 to 19.1 cm, the statics 10.6 to 9.1 cm.
  - It does not converge to the true per-device delays: about 6.4 cm mean
    error against a 7 cm spread. It is slow and only partly observable one
    filter at a time.
  - To do: a batch solve over logged ranges (still positions, biases,
    walker track) to calibrate properly, then push the result as settings.
  - To try: learn rb only from unflagged ranges.
  - Learning rb only while moving (`bias.learn_still=false`) was worse:
    walker 26.6 cm. The still-time rb absorbs per-node inconsistency,
    including blocked-link excess, which helps positioning even though
    it isn't the true delay. On hardware, board 3 learned -0.21 m within a
    minute of v24 booting.

### v24 bench walk: range-bias learning distorts the mesh (2026-10-21, end of day)

All four boards run v24, deployed over the air with
`swarm_ctl.py push ... --version 24`. The image came from the
`mule-build` container. On macOS the esp-rust Nix hash for aarch64-darwin
is a placeholder (fakeHash), so firmware only builds in the container.

The recording is `captures/swarm_v24.jsonl`. Board 3 (7408, on USB with
the laptop) was carried from t≈700 s to 800 s; the others stayed still.
The user has since unplugged board 3.

**Problem: the still boards' range biases ran away, and the mesh
moved.**
- At t≈320, before any walk, 53876 learned rb +0.31 and re-anchored
  about 0.35 m away.
- During the walk, the anchors' rb went from about 0 to +0.15..+0.46 m.
  Board 3's rb went from -0.21 to 0.
- Each 3 cm of rb change released an anchor (`bias.reanchor_m`) and
  re-solved it. 49736 moved from (0.11, -2.30) to (0.23, -1.17), 1.1 m,
  and the mesh contracted. Positive rb means shorter corrected ranges.
- Likely cause: the anchors learned rb (bias-only updates,
  `bias.learn_anchored`) from the carried board's ranges. Those were
  often body-blocked, so they read long.
- The mover and anchor rbs also trade off along an unobservable sum.
  Nothing pins it except the 0.1 m prior.
- The sim did not show this. The sim walker may be blocked less than a
  person carrying a laptop and board, or the sim runs (120 s) are too
  short to show the drift.

**Problem: "flagged" (power gap > 10 dB) is the norm in this room.**
- Before the walk, board 3 at the laptop had 89% of its ranges flagged:
  49736 at 100% with a median gap of 17 dB, 63124 at 98%. Those ranges
  were consistent (the board stayed anchored).
- During the walk, 45% were flagged.
- `nlos.gap_db` = 10 is likely too low for this room. The v23 stats
  showed the 12-15 dB bin was still clean; jumps start around 15 dB. Try
  13-15 dB.

**Uncommitted and untested (working tree, swarm/node node.rs and
settings.rs):**
- New setting `bias.learn_flagged` (id 352, default off): neither free
  nor anchored widgets learn rb from flagged ranges.
- It was written but not built or simulated (interrupted).

**Next, in order:**
1. Build and run the sim A/B already scripted in the scratchpad `ab.py`.
   Compare:
   - v24 (`bias.learn_flagged=true`);
   - unflagged only;
   - unflagged with `bias.learn_anchored=false`;
   - unflagged with `bias.reanchor_m=0`;
   - no bias (`est.init_range_bias_sigma=0`).

   Use 24 seeds of room. Also run longer runs (600 s) to look for rb
   drift, and check that the static boards' positions don't wander. Add a
   metric for anchor drift and mesh scale.
2. Make the sim's walker blocked more often (a person plus a laptop) and
   persistently-flagged-but-clean links (high gap, no jumps) like the
   bench's. Re-fit the gap thresholds.
3. Likely default change: no anchored rb learning and no re-anchoring
   (or a much larger threshold), and no learning from flagged ranges.
   Consider pinning the rb gauge, e.g. zero-mean rb across the swarm via
   adverts, like the centroid bleed.
4. Live mitigation without firmware is possible through settings:
   `bias.learn_anchored=false`, `bias.reanchor_m=0`, or
   `est.init_range_bias_sigma=0`. But rb isn't persisted and settings
   don't reset it, so a reboot clears the learned values (rb → 0 on
   boot).
5. Proper delay calibration remains a batch solve over a recorded walk,
   pushed as `antenna_delay`.
6. Then deploy v25, and have the user walk again. Plug board 3 back in on
   USB; the viewer runs with
   `swarm_view.py --port /dev/cu.usbserial-10 --capture captures/swarm_v25.jsonl`.



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
