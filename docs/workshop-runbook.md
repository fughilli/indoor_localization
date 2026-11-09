# Workshop runbook: first hardware bring-up

State of the code when this was written (2026-11-09). **No firmware in this repo has been
compiled or run since the CIR/sync-log work and the protobuf migration**: the host test suites
pass, `radio.cpp` and `netcmd.cpp` were syntax-checked against stub headers, and that is all.
Expect compile errors on the first firmware build; they should be small.

| Thing | Verified | Not verified |
|---|---|---|
| Simulator, solver, host tools, swarm_net (Rust), link codec | host tests pass | |
| `radio.cpp`, `netcmd.cpp`, `libs/dw3000` CIR read, `node.cpp` commands | type-checked against stubbed ESP headers | xtensa compile, link, run |
| Loader (`apps/loader`) | not even stub-checked | everything |
| DS-TWR with the larger protobuf frames | | timing on air |
| CIR register sequence | read from Qorvo's public driver | offset unit and first-path format (checks below) |

## 0. Get a toolchain that builds (this Mac's `/nix` is nearly full)

`bazel build //apps/node:node_esp32` pulls the Xtensa toolchain through nix. When this was tried
the `/nix` volume had 2.9 GB free and the build died with `No space left on device`; the esp
toolchain, Arduino core and Rust need roughly 10 GB. Free space first (`nix-collect-garbage`
removes unreferenced store paths; I did not run it).

The `third_party/embedded` submodule (fughilli/embedded) has `fakeHash` placeholders for
aarch64-darwin. Two are filled in (uncommitted) in this checkout's submodule from what nix
reported; if you are on another checkout, set them:

- `nix/esp_rust.nix`, `aarch64-darwin`: `hash = "sha256-WP2OEskfG0YtHarn2BhmGxyBZzenJMj3f4VlgGLpvCM=";`
- `nix/esp_xtensa_gcc.nix`, `aarch64-darwin`: `hash = "sha256-djdVF4wVKZ+NbzyIsSH59D0GvkmfEb5KXeUb8+dbMCI=";`
- `nix/esp32_arduino_libs.nix` and `nix/esp_riscv_gcc.nix` may need the same treatment
  (`nix` prints the correct `got: sha256-...` on a mismatch).

These should be committed to the embedded repo; I did not push anything there.

## 1. Which firmware

Two protocols exist. **Never mix them on one mesh**: a board flashed with one cannot talk to the
other, and the loader cannot be updated over the air.

- **Experiment build (old packed protocol + sync_log + CIR):** `git checkout uwb-landmarks`
  (tip `00cc94e`). Adds `sync_log`, `cir`, `SWARM_SLOTS`, nothing else. Use this for the
  clock-agreement and multipath experiment (`docs/uwb-sync-experiment.md`). The 10-node fleet
  needs `--copt=-DSWARM_SLOTS=12`.
- **Protobuf build (`master`, swarm.v1 wire format):** has the same experiment code on the new
  protocol. Use it to test the migration (section 4). Same `--copt=-DSWARM_SLOTS=12` for 10 nodes.

## 2. Build

```
git submodule update --init third_party/embedded      # once
bazel build //apps/loader:loader_esp32 //apps/node:node_esp32 //apps/common:partitions \
    --copt=-DSWARM_SLOTS=12
```

`swarm_ctl.py flash` also needs the stock ESP-IDF second-stage bootloader image
(`--bootloader`, rollback enabled) from the Arduino core: use the one you flashed the current
boards with.

## 3. Flash and provision (every board, over USB)

```
python3 swarm/tools/swarm_ctl.py flash --port P --bootloader B.bin --partitions partitions.bin \
    --loader loader_esp32.bin --main node_esp32.bin
python3 swarm/tools/swarm_ctl.py key --port P        # settings HMAC key, once per board
```

`flash` erases `otadata`; the loader boots first and hands over to main. Check the `boot` record on
the serial port (460800 baud, JSON lines): `app`, `version`, `id`, `radio`, `lfs`, `staging`.
Node ids come from the MAC. Old stored settings bundles fail their tag on the protobuf build:
re-push them (`swarm_ctl.py set --swarm ...`).

## 4. Protobuf-migration checks (master build), in this order

Watch the `stats` records (`timeouts`, `rx_errors`, `foreign`, `polls_tx`, `ranges_init`,
`ranges_resp`, `dropped`) and the `range` stream throughout.

1. **Ranging runs.** Two boards, then four: `range` records appear at about the old rate
   (~22 per pair per second for 4 nodes). A rate far below that, or `timeouts` climbing, points
   at the larger frames eating the reply budget (`kReplyTimeoutUs` 3 ms in `protocol.h`).
2. **No frame rejects.** `rx_errors` stays near zero. A burst means frames over 125 bytes or a
   wrong-version/malformed frame; `apps/common/link_test.cc` bounds the worst cases.
3. **Settings.** `swarm_ctl.py settings --port P`, `set --port P range_sigma=0.08`,
   `set --port P --node <other id> antenna_delay=16380` (a tagged override over the mesh),
   `set --swarm ...` (bundle fragments across the mesh).
4. **Firmware over the air.** `swarm_ctl.py push --port P node_esp32.bin` to one board, watch
   `swarm_ctl.py status`: manifest frame (`FRAME_TYPE_FW_MANIFEST`), page requests and data,
   verification, install through the loader, `installed` record, new version running. Check a
   second board picks it up from the first.
5. **Loader paths.** `{"cmd":"reboot_loader"}`, then hold BOOT at reset; the loader joins the
   mesh and still serves/receives images.
6. **Canary/rollback** (optional): the `node_hang_esp32` / `node_crash_esp32` images must roll
   back and not spread.

If something fails, capture the serial output of both ends plus `swarm_ctl.py status`; the codec
and the Rust state machine are host-tested, so the likely suspects are timing and the C++/FFI
glue.

## 5. The experiment (either build)

Procedure, thresholds and the analysis are in `docs/uwb-sync-experiment.md`. Short form:

1. `{"cmd":"sync_log","on":1,"mask":2}` on every node, capture 5+ minutes (`swarm/tools/capture.py`).
2. `python3 swarm/tools/sync_residual.py --log ID=file ... --pos ID=x,y,z ...`
   (below ~0.1 m: absolute TDoA is worth building; ~1 m: use excess delay only).
3. CIR: `{"cmd":"cir","on":1,"pre":8,"n":48,"every":4}`, two nodes 1 m apart in line of sight,
   one minute, then `python3 swarm/tools/cir_peaks.py capture.jsonl`. **Pass criteria:** the
   magnitude threshold crossing and the chip's first path (`fp >> 6`) agree to about a tap, the
   peak is a few taps after. A constant offset means the first-path index is not 6 fractional
   bits; garbage, or a window that does not move with `pre`, means `ADDR_OFFSET_A` is not in
   taps (`libs/dw3000/dw3000.cpp` `readCir`). Both are one-line fixes.
4. The CIR read adds ~0.5 ms of SPI right after a frame, before the reply: watch `timeouts`.

## 6. Documents

- `docs/uwb-landmarks.md`: the estimation model and simulation results.
- `docs/swarm-wire.md`: the protobuf formats and byte budgets.
- `docs/pbview.md`: how embedded structures are defined and read.
- `docs/swarm-network.md`: the network design (has some stale statements; the wire docs win).
