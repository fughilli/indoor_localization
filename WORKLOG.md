# WORKLOG

_Last updated: 2026-07-17 (session 2, build-out). Read together with `git log`._

## Goal

Build the CircuitDigest ESP32 + DWM3000 UWB indoor positioning system (3
anchors + 1 tag + host trilateration/web floor view) on the Bazel+Nix
`firmware` module at `third_party/embedded`. Build-only scope until hardware
is attached.

## State of play

**The build-only scope is complete.** All software deliverables build green
and are test-verified; nothing has touched hardware yet.

- Firmware: `libs/dw3000` shared driver + `apps/anchor` (×3 IDs) + `apps/tag`
  (see `92cfd2f`); all four `.bin`s verified with esptool image-info (chip
  ESP32, valid hashes). Embedded-repo branch `uwb-esp32-wifi` (pushed) adds
  the classic-ESP32 WiFi/Network targets; submodule pinned at `394af90`.
- Host: solver/server/simulator + 2 passing tests (see `c8288ee`); hermetic
  Python via rules_python+rules_uv (numpy/scipy in `requirements.lock`).
- README now carries the full build/flash/bring-up/calibration guide.
- Per-container bootstrap ritual lives in memory `submodule-ssh-key.md`
  (ssh config needs BOTH the `github-fughilli` alias and a plain `github.com`
  block — the latter for the submodule's origin; push access confirmed).

## Next up

1. **Hardware bring-up when boards are available**: flash per README §Bring-up
   (`bazelisk run //apps/anchor:flash_anchor_N -- --port /dev/ttyUSBx`),
   create `apps/tag/config_local.h` with real WiFi credentials, run
   `//host:server`, verify live ranging end-to-end, then antenna-delay
   calibration (README §4).
2. Optional (no hardware needed): merge `uwb-esp32-wifi` into embedded
   `master` once it's proven on hardware; consider a root `BUILD.bazel` alias
   set (`//:anchor_1` …) mirroring the embedded repo's style.

## Open questions / blockers

- **Hardware not attached** — everything past build+simulation is blocked on
  boards.
- Root repo `origin` (`git@github-fughilli:fughilli/indoor_localization.git`)
  still holds the pre-rewrite `master`; a **force-push is pending user
  confirmation** (history was rewritten 2026-07-17 for the author-identity
  fix). Local master has 4 unpushed commits on top of that rewrite.

## Don't retry (dead ends)

- `apt-get install` at runtime — no root/sudo; bake packages into
  `.claude-container-overlay/Dockerfile`.
- Cloning `github.com/fughilli/embedded` over HTTPS — private; use SSH with
  the repo-local key.
- Don't fetch ESP-IDF via nixpkgs-esp-dev flake or run IDF CMake — the
  embedded repo intentionally links prebuilt SDK `.a`s (see its WORKLOG).
- `bazel build //...` without `.bazelignore`ing `third_party/embedded` —
  Bazel walks the submodule as root-repo packages and errors on its .bzl
  files (fixed in `92cfd2f`; don't remove the ignore).
