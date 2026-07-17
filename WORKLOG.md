# WORKLOG

_Last updated: 2026-07-17 (session 1, bootstrap). Read together with `git log`._

## Goal

Build the CircuitDigest ESP32 + DWM3000 UWB indoor positioning system (3
anchors + 1 tag + host trilateration/web floor view) on the Bazel+Nix
`firmware` module at `third_party/embedded`. Build-only scope for now (no
hardware attached): deliver flashable `.bin`s, flash targets, tests, and a
bring-up guide. Full plan: `/claude/plans/dreamy-tumbling-bengio.md` (survives
in-container only; the plan's substance is mirrored below).

## State of play

- Session 1 scaffolded the workspace: container overlay (Nix + openssh-client,
  ports 7007/8080), submodule `third_party/embedded` @ `a14ef48` (master tip —
  classic-ESP32/WROOM support, added 2026-07-17), MODULE.bazel
  (`uwb_rtls` + `local_path_override` of `firmware`), .bazelrc/.bazelversion/
  .bazeliskrc mirroring the submodule's, GPL-3.0 LICENSE + README.
- **Nothing has been built yet** — this container has no Nix; the overlay
  takes effect only after the user relaunches claude-container.
- Reference implementation: https://github.com/Circuit-Digest/ESP32-DWM3000-UWB-Indoor-RTLS-Tracker
  (GPL-3.0). Self-contained: inline register-level DWM3000 driver, only
  SPI.h/WiFi.h needed. Key files: `src/Anchor/main_anchor.cpp` (~1900 lines,
  Arduino+SPI only), `src/Tag/main_tag.cpp` (~1554 lines, +WiFi/WiFiClient,
  TCP JSON to host:7007), `py_script/floor_view.py` (scipy least_squares
  trilateration), `test/packet.json` (sample tag JSON). Pins: CS=4, RST=27,
  IRQ=34, SPI 18/19/23. Re-clone to /tmp/uwb-ref when needed (public).
- User decisions (2026-07-17): embedded-repo changes go on branch
  `uwb-esp32-wifi` (not master); host software = Python solver + **web**
  visualizer (stdlib http + SSE, no matplotlib), hermetic pip deps
  (numpy/scipy) via rules_python + uv lockfile (see bazel-polyglot-nix skill);
  build-only until hardware is available.

## Next up

1. **User relaunches claude-container** (overlay installs Nix + ssh).
2. Bootstrap ritual (fresh container): write `~/.ssh/config` with the
   `github-fughilli` alias → `credentials/id_ed25519_personal_github`
   (see memory `submodule-ssh-key.md`); `git submodule update --init` if the
   checkout is missing; `nix --version`; then warm-up
   `bazel build @firmware//:blink_esp32` (first run downloads GBs — expected;
   caches persist in `.bazel-*-cache/`).
3. In `third_party/embedded`, branch `uwb-esp32-wifi`: add `network_esp32` +
   `wifi_esp32` cc_library targets to `nix/arduino_esp32.BUILD`, mirroring the
   C6 `network`/`wifi` targets (~lines 518–546) but with `_CXX_COPTS_ESP32` +
   `:sdk_hdrs_esp32`. Likely also public wrapper aliases (consumers cannot see
   `@arduino_esp32` — extension repos are module-private; mirror the
   `libs/vendor` alias pattern from embedded commit `02b3d99`). Push branch,
   pin submodule.
4. Port firmware: `libs/dw3000` shared driver (diff the reference tag/anchor
   files to extract common code; fall back to two self-contained cpps if too
   entangled), `apps/anchor` (ANCHOR_ID 1/2/3 via local_defines →
   `firmware_binary(board="esp32")` ×3 + esptool_flash), `apps/tag`
   (config.h placeholders + gitignored config_local.h for WiFi/host-IP).
   Verify: `bazel build //...` + `esptool image-info` (chip ESP32) on each bin.
5. Host software under `host/`: rules_python + uv lockfile (numpy, scipy);
   `solver.py` (least-squares trilateration), `server.py` (TCP 7007 ingest +
   stdlib HTTP 8080: canvas floor-view page + SSE), `simulator.py` (synthetic
   tag → full-stack headless demo); py_tests: solver round-trip + integration
   (simulator→server→SSE). `bazel test //host/...`.
6. README bring-up guide (flash commands, anchor placement, antenna-delay
   calibration 16350) + WORKLOG handoff.

## Open questions / blockers

- **BLOCKER: container relaunch** needed before any build (no Nix here).
- Push access to fughilli/embedded with the repo-local deploy key is untested
  (read worked). If push is refused, keep the branch local and flag here.
- Remote `origin` = `git@github-fughilli:fughilli/indoor_localization.git`
  (host alias needs `~/.ssh/config`, see memory). History was rewritten
  locally on 2026-07-17 (author identity fix, user-requested), so the remote's
  old `master` (pre-rewrite initial commit) requires a **force-push** — user
  confirmation pending.

## Don't retry (dead ends)

- `apt-get install` at runtime — no root/sudo in the container; bake packages
  into `.claude-container-overlay/Dockerfile` instead.
- Cloning `github.com/fughilli/embedded` over HTTPS — private, prompts for
  credentials. Use SSH with the repo-local key. (No `ssh` binary in the
  pre-overlay container: node wrapper `/tmp/ssh-wrapper.js` + npm `ssh2`
  worked as `GIT_SSH_COMMAND`.)
- Don't fetch ESP-IDF via nixpkgs-esp-dev flake or run IDF CMake — the
  embedded repo intentionally links prebuilt SDK `.a`s (see its WORKLOG).
