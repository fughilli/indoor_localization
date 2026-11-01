# Working state: Splanc on the ESP32-P4 with a wired Vive tracker (2026-11-01)

A snapshot of where the work stood when it was saved, so it can be picked up or
sorted out later. Nothing here is final: every branch below is work in progress.
Where the files in `artifacts/` came from is in [ARTIFACTS.md](ARTIFACTS.md);
`provenance.tsv` lists each file.

## Where the code is

| Repo | Branch | Commit | What |
|---|---|---|---|
| fughilli/indoor_localization (private) | `wip/p4-tracker-2026-11-01` (off `lighthouse`) | this one | the Splanc demo's `--p4` mode, the truth rig's flat head, the vive_web change, this document and the artifacts |
| fughilli/lighthouse (public) | `wip/f32-photon-latency` (off `main` 4919232) | 324b2ba | f32 kernel for the P4, the latency work in kernel/device/lhd/lh_replay |
| fughilli/splanc-private (private) | `agent/p4-tracker` (off `main` 2c493fc0) | 11296d44 | the P4 player, the C6 front end, the tracker module, the photon-to-photon pipeline |

The Splanc work is kept out of the public fughilli/splanc until it's ready for
release. `fughilli/splanc-private` is a private copy of it: GitHub doesn't allow
a private fork of a public repo. In `~/Projects/splanc-p4` its remote is
`private`, and `origin` is still the public repo. To release, push or PR from
`private` to `origin`.

The local branch `f32` in the lighthouse checkout carried no commits; the new
branch holds its working tree, so the f32 and latency changes are one commit.

**Build dependency.** Splanc's MODULE.bazel still pins lighthouse at 4919232,
which predates the f32 kernel. Until the pin is bumped to a lighthouse commit
with that work in it, build Splanc with the override, from `~/Projects/splanc-p4`:

```
bazel run -c opt --//firmware/p4:tracker //firmware/player_app:flash_esp32p4_player_c6 \
  --override_module=lighthouse=$HOME/Projects/git_checkouts/lighthouse \
  -- --port /dev/cu.usbmodem5CF71536071
```

Flashing takes about 3 minutes. The console monitor (`artifacts/scratchpad/p4_mon.py`) holds the serial port, so stop it first.

**Tests.**
- Lighthouse: `bazel test //... --nocache_test_results` (5 targets, all pass).
- Splanc fps crate: its tests pass.
- Demo: from `lighthouse/demos/splanc`, run `/usr/bin/python3 -m unittest tests.<module>` (pytest isn't installed).

## The project

The Vive tracker is wired by USB-C to the HS OTG port of a Waveshare
ESP32-P4-WIFI6 (die v3.2), header P1. The P4 runs the Splanc player and the
lighthouse kernel itself, so the tracker-to-LED loop has no laptop and no Wi-Fi
in it. The C6 on the board is the network front end, linked over SDIO
(Splanc `docs/design/p4-c6-split.md`).

### Done and run on the board (through 2026-10-30)
- **Toolchain:** the P4 toolchain and the esp32p4_hello image. The esptool 5.6 and P4 build-target patches are on the Splanc branch.
- **Tracker on the P4's USB:** enumerates, handshakes, decodes stations, calibrates and tracks.
  - A raw dump replays identically in lhd.
- **f32 kernel:** the P4's FPU is single precision. A report takes about 25 µs.
- **Player and front end:**
  - Phase A: the player over a UART link. fx_bench runs all 72 programs.
  - Phase B: the C6 front end over SDIO, and the P4 reflashes the C6 when its image differs.
- **Tracker module:** a pluggable module that puts `tracker`, `mount`, `pose` and `tracker_ok` in the uniform pool and handles the `/lh/*` OSC messages.
- **Lag fix:** the tracker task fell behind in bright spots (pose bootstraps on every sweep). Fixed by rate-limiting the bootstraps and trimming the backlog.
- **Demo `--p4` mode:** while probing, the P4's own pose is the fixture tracker. The stylus fit goes out as `/lh/mount`.

### Built but NOT yet run on the board: photon-to-photon latency (2026-10-31)
Goal (user): make the time from a laser hit on a photodiode to the LEDs lighting
as short as possible, and predict where the fixture will be when the frame lights.

The changes:
1. **Hits go into the filter as each report arrives** (`stream_sweeps`), instead of waiting for the next cycle's sync. The wait used to cost up to about 7.5 ms.
   - On the wired dump, light → pose went from a 6.5 ms median (p99 8.4) to 1.4 ms (p99 3.6), with the same tracking events.
2. **The EKF propagate is structured and in place,** with a sparse P·h. It gives the same poses to the µm, for about 10× less arithmetic.
3. **The render task reads the pose at the start of every frame** (the new `frame` hook). The loop task used to publish it only between frames, so a pose could sit for one to two frames.
4. **Prediction:** each frame predicts the pose (constant acceleration and angular rate) to its own measured light-up time.
5. **Frame rate:** tracker builds drop the 80 fps ceiling (`wire_ceiling`), which gives 125 fps for 250 LEDs.
6. **Task priorities:** the tracker task runs at 12 while tracking and 7 otherwise; the USB host tasks run at 13.
7. **Instrumentation:** a 5 s console line with latency per stage, the prediction horizon and lit error, and a CPU profile.
   - OSC switches: `/lh/predict 0|1`, `/lh/lead <ms>`, `/lh/stream 0|1`.

The estimated mean photon-to-photon time is about 25 ms before and about 14 ms after, before prediction. It is not measured yet. Design write-up: Splanc `docs/design/p4-c6-split.md` §4.2.1.

Known costs and limits:
- **Accuracy under fast rotation.** Streaming loses some accuracy there: the turntable's radial RMS went from 18 to 31 mm with the same stations, while the slider was unchanged.
  - `/lh/stream 0` turns streaming off.
  - Compare replays with `--save-stations` plus `--no-auto-calibrate`; with auto-calibration on, the world frames differ.
- **A delay the P4 can't see.** The tracker's own fixed delay before its fastest report isn't visible from the P4, so every figure is "above the fastest report". `/lh/lead` stands in for it.

## Next steps
1. Plug the P4 into the Mac, flash, and watch the `[tracker] photon->photon` lines while moving the lamp.
2. Do drop tests with `/lh/predict` on and off, and tune `/lh/lead`; also try `/lh/stream`.
3. Optional: put a photodiode on a spare GPIO next to a tracker sensor to get an absolute photon-to-photon number.
4. Optional: WS2812B-V5 timing (0.9 µs bits) would cut the wire time from 7.2 to 5.4 ms, if the lamp's LEDs accept it.
5. Bump Splanc's lighthouse pin once the lighthouse branch is settled.

## Other open items (from before)
- **Effect deploy:** deploy `lighthouse/demos/splanc/effects/tracker_demo_pose.glsl` (2255 B compiled) to PixieLamp2, then run `measure_effect`.
- **LED probe:** re-probe the LEDs (Probe LEDs → converged → save, which sends `/lh/mount`).
- **Battery pack:** on a 5 V 2.1 A pack the tracker gave no poses. Not yet known whether the pack is the cause.
- **Stuck tracker:** `artifacts/scratchpad/stuck_tracker.log`.
- **Unverified pieces:** PARLIO output hasn't been checked on a logic analyzer, and the C6 chain loader is still Espressif's stub (the TODO is to write our own).
- **Not done yet:** multi-session wss.
- **Demo tests:** 2 synthetic calibration tests are just over their thresholds; the user hasn't decided whether to loosen them.

## Board and network facts worth keeping
- **USB:** the board's USB-C goes to a CH343P serial bridge (console and flashing). The P4's HS OTG port is P1: pin 1 is 5 V, 2 D−, 3 D+, 4 GND.
- **C6 wiring:**
  - C6 EN = P4 GPIO54.
  - C6 IO9 (H4 pin 1) is jumpered to P4 GPIO4.
  - The C6's UART is only on header H4.
- **Strip GPIOs** (free header pins):
  - right column: 20-23, 26, 27, 32, 33, 46-48
  - left column: 2-5, 28-31, 49-52
- **CPU and RTOS:** FreeRTOS at 1000 Hz, CPU at 400 MHz.
- **Console:** Arduino Serial prints nothing on the P4; use printf.
- **lhd calibration:** lhd writes calibration back to the file it was given, so replay from copies. `artifacts/lighthouse_config/` is a copy of `~/.config/lighthouse` as of this snapshot.
