# Experiment: how well do the nodes' clocks agree?

Companion to [uwb-landmarks.md](uwb-landmarks.md). The simulation says whether
broadcast (one frame heard by every node, TDoA) is worth building depends on one
number nobody has measured on this hardware: the residual timing error between two
nodes' clocks after offset and drift are corrected, as a function of how often they
are corrected. This experiment measures it on the existing DWM3000 boards, with no
new radio hardware and no protocol change.

## What the firmware now does

- **`sync_log` command** (`apps/node/node.cpp`, `apps/common/radio.cpp`):
  `{"cmd":"sync_log","on":1,"mask":2,"every":1}`. While on, the node prints
  - a `tx` record for each frame it sends: `{"type":"tx","to","ft","seq","ts"}`, `ts`
    being its DW3000 transmit timestamp (40 bits, 15.65 ps units);
  - an `rx` record for each frame it hears whose type is in `mask` (bit n = frame type
    n; 2 = POLL, 4 = RESPONSE, 8 = FINAL), every `every`th:
    `{"type":"rx","from","to","ft","seq","ts","clk","rssi","fp_rssi"}`: receive
    timestamp, the raw carrier-integrator clock offset the chip reports for that
    frame, and total/first-path power.
  Records are read immediately after the frame is received, before the node replies,
  because the offset and power registers describe the *last* frame.
- **`SWARM_SLOTS`** (`apps/common/protocol.h`), default 6. A node owns TDMA slot
  `rank % kSlots`, so **more than 6 nodes collide**: with 10 nodes, ranks 6–9 share
  slots with ranks 0–3. Build the 10-node fleet with `--copt=-DSWARM_SLOTS=12`. The
  superframe is fixed (12 × 5 ms), so a 4-node mesh built this way ranges at half the
  rate of the default build: use the define only for the 10-node fleet.
- **`swarm/tools/sync_residual.py`**: joins the logs of several nodes and, for nodes
  at surveyed positions, computes `(ts_rx − ts_tx) · c − distance` per sender→receiver
  pair, removes a straight line (offset + drift) in windows of 0.5 … 10 s, and prints
  the RMS in metres. That residual is the `sigma_sync` of
  `multipath.SyncModel` / `landmarks.SolveConfig`. It also reports the fitted drift
  (ppm) next to the chip's raw clock-offset reading, to see whether the chip's own
  estimator could replace fitting. Tested on synthetic logs (`sync_residual_test.py`:
  sequence wrap, unrelated boot clocks, 40-bit wrap, dropped frames, drift and noise
  recovery).

**Not compiled or run on hardware.** The firmware changes are small and were reviewed
by eye, but `third_party/embedded` (the toolchain) is an uninitialised submodule and I
did not start the multi-GB toolchain fetch. Build it before relying on it.

## Procedure

1. Build and flash all 10 nodes with `--copt=-DSWARM_SLOTS=12` (`swarm_ctl.py flash`).
2. Place at least three nodes at surveyed positions, a metre or more apart and
   not collinear. Distances need to be right to ~1 cm for the *residual*; a constant
   survey error is absorbed by the fitted offset (there is a test for that), a
   position error that varies with time is not.
3. Let the network settle, then on every node: `{"cmd":"sync_log","on":1,"mask":2}`
   and capture its serial output (`capture.py --port …`) for ≥ 5 min. Leave the nodes
   still, with fixed power. Repeat with the nodes' antennas indoors and in a more
   reflective spot, and once warm (the crystal drifts as the board heats).
4. `python3 swarm/tools/sync_residual.py --log ID=file … --pos ID=x,y,z …`

Data rate: with POLLs only the log is about one 110-byte line per slot heard; the link
is 460800 baud, shared with the pose stream. If lines are dropped, raise `every`
(sequence gaps are harmless to the analysis).

## How to read the result

From the fleet study in docs/uwb-landmarks.md (reflector mapping, absolute path lengths
from broadcast frames vs pairwise two-way ranging):

| Residual at the window you can sustain | Meaning |
|---|---|
| ≲ 0.1 m (0.3 ns) | absolute TDoA from broadcast matches pairwise; worth building |
| ~0.3 m (1 ns) | absolute broadcast lengths cost about 2× the horizontal error; use excess delay |
| ≳ 1 m | absolute broadcast lengths are useless (0.45–0.6 m error); excess delay only |

Excess delay and link-power (fader) mapping do not depend on this number, so a bad
result does not kill the broadcast idea; it only rules out absolute TDoA. Pick the
window W that matches how often the nodes can re-estimate each other's offset and
drift: a few seconds from the existing DS-TWR exchanges.

## Not built yet

- **CIR capture** (the multipath itself). The driver has no accumulator readout. It
  needs the ACC_MEM clock enables, an addressing mode that reaches offsets beyond the
  7-bit sub-address, a dummy leading byte, the 18-bit complex tap packing and the CIA
  first-path index; I have not verified any of that against the DW3000 user manual and
  did not write register-level code from memory. The DWM3000 / DW3110 is single-antenna
  without PDoA; PDoA waits for the QM33120 boards.
- A broadcast frame with a scheduled (delayed) transmit time. The experiment above
  does not need one: it joins transmit and receive timestamps on the host.
- Per-node yaw self-calibration (`landmarks.calibrate_yaw`) is simulation-only.
