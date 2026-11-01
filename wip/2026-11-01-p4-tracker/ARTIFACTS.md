# Artifacts: what they are and how they came to be

These are everything produced or collected while working on this, from 2026-10-20 to 2026-11-01. They are saved as found, to be sorted later.

## Layout and how the files were saved
- **`artifacts/scratchpad/`** is Claude Code's session scratch directory, copied with its layout intact.
  - Its original path was `/private/tmp/claude-501/-Users-kevin-Projects-git-checkouts-indoor-localization/ea7d830b-6f55-49b3-94bb-12e39a7b266a/scratchpad`.
  - File times are kept.
  - Text files over 1 MB (`.txt`, `.out`, `.jsonl`, `.html`) are xz-compressed: `xz -d` restores them.
- **`artifacts/captures/`** is a copy of `captures/vive/` (gitignored in this repo), including `bad/`. These are the recordings most of the replays ran on.
- **`artifacts/lighthouse_config/`** is a copy of `~/.config/lighthouse/*.json`, lhd's station calibrations, as of 2026-11-01.
- **`artifacts/third_party_refs/`** stands in for third-party things too big, or too reproducible, to keep:
  - **`libs.zip`** (502 MB): the Espressif esp32-arduino-libs SDK, downloaded on 2026-10-29 for the P4 part. Its sha256 is recorded, and the list of chips it holds.
  - **`esptool-venv`:** a Python venv with esptool 5 for reading the P4's chip ID. Its config and `pip freeze` are recorded.
  - **`embedded/`:** a clone of fughilli/embedded at Splanc's pin with Splanc's patches applied, plus one local commit "esp32p4 target". That commit is `embedded-esp32p4-target.patch`; the base is in `embedded-clone.txt`. The Splanc branch carries the same change as `patches/embedded-esp32p4-build-target.patch`.
  - **`ref_esf/esp-serial-flasher-master`:** its tarball `scratchpad/ref_esf/esf.tar.gz` is kept, and the unpacked tree left out.
- **`provenance.tsv`** has one row per file:
  - path;
  - file time;
  - when it was first written in the session log;
  - the description of the command that wrote it.
  - Rows with no command are files whose writer the log doesn't show plainly, such as files made by a tool's own `--out` or by a script.
- **Not included:**
  - `lhd.cmd`, a FIFO;
  - the session transcript itself;
  - `~/.config/swarm`, which holds keys;
  - `credentials/`.
  - The copied files were checked for the swarm keys and for credentials; none turned up.

## Groups, in the order the work happened

### 1. Firmware survey and early tools (2026-10-20)
- **`scratchpad/bin/`:** created on 2026-10-20 when inspecting the launcher. It now holds the lhd builds (§5).
- **`scratchpad/ws/`:** the Vive Tracker 3.0 developer guide (`vive_tracker3.pdf`) with page renders and extracted text (`t3.txt`). Downloaded 2026-10-29 to find the tracker's USB parts.
  - `bbox.html` (xz): a pdftotext bbox render used to find USB labels.

### 2. Lighthouse tracking work on the dongle trackers (2026-10-27 to 10-29)
- **`captures/*.rec.gz` + `.config.json` + `.json`:** raw lhd recordings, recorded with lhd's recorder.
  - Each has the station config and session notes saved with it.
  - Captures from 2026-10-27: turntable-center, turntable-r, slider, near, button-test, jitter.
  - Captures from 2026-10-28: linkcheck, jaggies, ab-handheld.
  - `bad/`: the earlier turntable takes that were redone.
  - `ab-handheld` is the capture compared against libsurvive.
- **`scratchpad/pair_test*.rec.gz`, `pair2/3.rec.gz`, `both_on.rec.gz`, `link3.rec.gz`:** recordings made while testing two Watchman dongles at once and measuring the radio packet rate. Result: one dongle carries one tracker (memory `watchman-dongle-one-tracker`).
  - `lhd.out`/`lhd.err`: lhd's output while recording these, with commands from a FIFO.
  - `replay_pair2/3.out`: replays of the two pair recordings.
- **`scratchpad/stale_cal.json`, `cal_pair2/3.json`:** calibration copies used to test the stale-calibration reset. `cal_rt.json` is a copy for the before/after-refactor replay check.
- **"Jaggies" investigation** (hand-held jitter, 2026-10-29 UTC):
  - `ev0.txt`: a sample of the viewer's live events.
  - `jag_ev.txt`: 15 s of poses with the jumps measured.
  - `jr*_poses.txt` / `jr*_ev.jsonl`: replays with per-station innovation stats; `jr2` uses libsurvive's station calibration.
  - `innov*.txt`: per-sweep innovations.
  - `scan*.txt`: an IMU time-offset scan against innovation RMS.
  - `sc*.txt`, `tt*.txt`, `smooth*.py`: jitter scores across filter settings, and against accel noise on the turntable.
  - `ab_*`, `ls_poses.txt`: our pipeline against libsurvive on the ab-handheld capture.
  - Findings are in memory `lighthouse-session-2026-10-28`: about 2 mm RMS in lhd against 8–15 mm in libsurvive. IMU timing, gyro scale and the time base were ruled out.
- **`scratchpad/vive_pdf/`:** the Tracker 3.0 developer guidelines (`g10.pdf`) with page renders. Downloaded 2026-10-28.

### 3. Splanc tracker demo (2026-10-29)
- **`scratchpad/patch_probe_app.py`:** a script that patched the demo's app for the stylus-probe calibration.
- **`appsynth.txt`, `app_dbg.txt`, `osc_dbg.log`:** the synthetic app test and OSC debugging output.
- **`scratchpad/headcheck/`:** a checkout of `lighthouse/demos/splanc` at an earlier commit, used to compare against the working tree. Created 2026-10-30.
- **`untracked.txt`:** the untracked files by directory, listed before the lighthouse commits (with a secrets scan at the time).

### 4. ESP32-P4 bring-up (2026-10-29)
- **Tooling and scaffold:**
  - `embedded_patches.txt`: the Splanc patches applied to the embedded clone.
  - `gen_p4_section.py` → `p4_section.BUILD`: generates the P4 scaffold BUILD section. It became Splanc `tools/gen_embedded_p4_section.py`.
  - `p4build.log`, `p4build2.log`: builds of the P4 hello image and the C6 image.
- **`ws_sch/`:** the Waveshare ESP32-P4-WIFI6 schematic (`sch.pdf`, text in `sch.txt`) with crops of the USB, C6, UART and P4 sections, used to find the OTG header P1 and the C6 wiring. `ws/sch.pdf` is the same file, downloaded earlier.
- **`p4_mon.py`:** a console monitor. `p4_mon.py <secs> [reset]` captures the P4 console from `/dev/cu.usbmodem*`.

### 5. Wired tracker on the P4 and the f32 kernel (2026-10-29 to 10-30)
- **`p4_wired.rec.gz`, `p4_wired2.rec.gz`:** raw dumps of the tracker on the P4's USB, from Splanc `esp32p4_tracker_dump` turned into lhd recordings by `tools/p4_dump_to_rec.py`. `p4_wired2` is the fresher one (2026-10-30), and the main input for the latency work.
  - `p4_lhd.jsonl`, `p4_wired_events.jsonl`: lhd replays of them. The `p4_wired_replay.*` files are empty, from a first try.
- **`cmp_f32.py`:** `cmp_f32.py <rec> <speed> [exeA exeB]` replays one recording through two lhd builds and compares their poses.
  - `cmp_*.txt`: f32 against f64.
  - `cmpf64_*.txt`: the new f64 against the pinned f64, on four captures.
- **`bin/`:**
  - `lhd`, `lhd_f32`, `lhd_pinned`: 2026-10-29 builds, for the f64/f32 comparison.
  - `lhd_before`, `lhd_f32_before`: 2026-10-31 builds before the latency work.
  - `lhd_new`, `lhd_f32_new`: builds after the structured propagate.
  - All are macOS arm64.
- **`tracker.dis`:** empty, from a search for kernel functions still calling soft-float double helpers.
- **`p4fx/`** (`tracker_fx.fx`, its `.fxb`, `uniforms.py`): a first tracker effect, loaded to check that the uniforms read back.
- **`plane_slice_pose.fxb`, `sphere_shell_pose.fxb`, `tdp.fxb`:** offline compiles of the demo's `effects/*_pose.glsl`. `tdp` is `tracker_demo_pose.glsl`, 2255 B.

### 6. P4 player, link and C6 front end (2026-10-29 to 10-30)
- **`bridge.log`:** the UART link bridge while running player_probe.
- **`fxbench.log`, `p4_bundle.json`:** fx_bench through the bridge (all 72 programs).
- **`regress.log`:** C6/C3 images and the player host tests, as a regression check.
- **`p4.patch.new`:** the regenerated P4 patch after adding the Preferences library.
- **`blescan.py`:** a BLE scan for the C6's advertisement.
- **`stub/esp32c6.json`:** the esp-flasher-stub v1.3.0 C6 stub, the one the P4 SIP-uploads to reflash the C6.
- **`ref_esf/esf.tar.gz`:** esp-serial-flasher source, for reference.
- **Boot logs:**
  - `boot_tracker.log`: a full boot log with the tracker handshake.
  - `boot_ident.log`: the identity the C6 pushes.

### 7. Running the demo on the P4 (2026-10-30 to 10-31)
- **`mapsend*.log`:** P4 (and C6) console logs while the Splanc app sent the LED map, over the attempts it took.
- **`posewatch.sh` → `posewatch.log`, `retry.log`:** the P4's pose while the user moved the lamp.
- **`drop1-7.log`, `ping_conc.log`:** chasing gaps in the P4's `/lh/pose` stream to the Mac: radio TX counters, device send timing against Mac receipt, and re-measurements after the fixes.
- **`now.log`, `reset1.log`, `stuck_tracker.log`:** the P4 after a reset, where the tracker stayed stuck. That bug is still open.
- **Tracker lag (task behind in a bright spot):**
  - `flash_lag.log`, `flash_lag2.log`: builds and flashes with lag instrumentation and the backlog fix.
  - `lag1.log`, `lag2.log`: the status lines with lag counters.
  - `console_new*.log`: the console after the fix.
- **`headflat.png`:** a render of the truth rig's flat head (`hardware/truth_rig/head_flat.py`).
- **`ut.log`:** a full demo unit-test run.

### 8. Photon-to-photon latency (2026-10-31): `scratchpad/lat/`
All of these are replays through lh_replay and lhd as the lighthouse code changed that day, so each file reflects the code at the time it was written. The code that's now on lighthouse `wip/f32-photon-latency` is the end state.

- **`base.txt`/`base.jsonl`, `base_p4_wired*.{txt,jsonl}`:** lh_replay of the wired dumps before any change.
- **Captures `<rec>_late` and `<rec>_early`, plus `_early4`, `_early5`:**
  - `late` is the old timing: a sweep goes out at the next sync.
  - `early` is the first `--clock`: the sweep goes out once the laser has passed.
  - `4` and `5` are variants with the clock lagged by 4 and 5 ms.
  - Run on slider, ab-handheld and turntable-r.
- **`p4_wired*_plain.jsonl`, `_hc.jsonl`:** lhd_f32 on the wired dumps, without and with `--host-clock`.
  - `cal_x.json` and `cal_plain/hc/st.json` are copies of `calibration.pre-move-2026-10-30.json`. lhd writes back to its calibration file, so each run got its own copy.
  - `p4_wired2_st.jsonl` adds `--stream-sweeps`. This is the 6.5 → 1.4 ms median result.
  - `p4_wired2_st1.jsonl`: an experiment with one candidate per sensor per streamed sweep, later reverted.
- **`<rec>_whole.txt` / `_parts.txt`:** lh_replay, whole sweeps against `--clock 0 --stream`, on turntable-r, slider and turntable-center.
  - `wholeF`/`partsF`: the same with each capture's own station config and `--no-auto-calibrate`.
  - `parts4`: the clock lagged 4.5 ms, with in-order parts.
  - `parts4d`: the once-per-sensor experiment, on turntable-r.
- **`<rec>_st.json`:** stations saved with `--save-stations` from one plain replay.
  - `wholeS`/`partsS` replay against those same stations; this is the fair comparison (turntable-r radial RMS 18 → 31 mm, slider 0.63 mm).
  - `parts1S` is the same with the P4's 1 ms tick.
- **Scripts:**
  - `cmp.py` compares two pose files.
  - `geom.py` fits the turntable circle and slider line to an lh_replay output.
- **`timing.rs.bak`, `timing.rs.bak2`:** `kernel/src/timing.rs` saved before each experiment.

## Where the conclusions went
- **Design:** Splanc `docs/design/p4-c6-split.md` §4.2.1 (photon to photon), and the rest of that doc for the P4/C6 split.
- **State and next steps:** [WORKING.md](WORKING.md).
- **Demo usage:** `lighthouse/demos/splanc/README.md` (the `--p4` section).
