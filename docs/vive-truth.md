# Ground truth from Vive Trackers

Vive Tracker 3.0 pucks strapped to the nodes give a 6-DoF reference with
millimetre-level precision to score the swarm against. libsurvive does the
tracking instead of SteamVR: it's an open-source Lighthouse stack that talks
to the trackers directly. SteamVR doesn't need to be installed.

| Tool | What it does |
|---|---|
| `swarm/tools/vive_truth.py` | Logs tracker poses, stamped with host time, to a JSONL file |
| `swarm/tools/swarm_view.py --capture` | Now stamps every node record with the same host time (`ht`) |
| `swarm/tools/truth_eval.py` | Scores a swarm capture against a truth file |

## What's needed

- **Base stations, SteamVR 1.0 or 2.0.** At least one is required, and two
  is better. Mount them high in opposite corners, angled down over the
  area. The trackers can't localize without them.
- **Trackers connected by USB-C cable or by a paired dongle.**
  - USB works with no pairing at all, so start with it.
  - A dongle has to be paired to its tracker first. SteamVR normally does
    the pairing, once, on any machine, and the pairing stays with the
    dongle. Whether libsurvive can pair a dongle on its own hasn't been
    checked here.
  - The carried node needs the dongle, or a long cable.
- **libsurvive.** Its macOS support is recent: hidapi and plugin-loading
  fixes were merged in May 2026 (collabora/libsurvive#363). It is not yet
  tested on this Mac. If it won't run here, use a Linux machine for the
  trackers and send the poses over UDP (see below).

## Building libsurvive (macOS)

```sh
brew install cmake hidapi libusb zlib
git clone https://github.com/collabora/libsurvive.git
cd libsurvive && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j8
./survive-cli   # trackers + base stations visible: should start tracking
```

The pip wheels for the Python bindings cover Windows and Linux only. On
macOS, either:

- build the bindings with `python3 setup.py install` from the repo root;
- or use `--source cli` below, which needs only `survive-cli`.

The first run, with everything in view and still, locates the base stations
(about 10 s) and saves them to libsurvive's `config.json`. If a base station
moves, recalibrate with `--force-calibrate`.

## Session

1. **Map trackers to nodes.** Start `vive_truth.py` with no `--map`, move one
   tracker, and see which `dev` changes. Node ids can be decimal or hex, as
   the viewer prints them (`F694` = 63124).
2. **Record both streams on this machine:**

   ```sh
   python3 swarm/tools/swarm_view.py --port /dev/cu.usbserial-120 \
       --capture captures/swarm_session.jsonl
   python3 swarm/tools/vive_truth.py --out captures/truth_session.jsonl \
       --map T20=F694 --map T21=1CF0 --map T22=C248
   # or, with only survive-cli built:
   python3 swarm/tools/vive_truth.py --source cli --out captures/truth_session.jsonl \
       --map T20=F694 ...
   ```

   With the trackers on a Linux machine instead:

   ```sh
   python3 swarm/tools/vive_truth.py --udp-out <this mac>:9180 --map ...   # Linux side
   python3 swarm/tools/vive_truth.py --udp-in 9180 --out captures/truth_session.jsonl
   ```

   The receiver re-stamps each pose with this machine's clock on arrival.
3. **Score:**

   ```sh
   python3 swarm/tools/truth_eval.py --capture captures/swarm_session.jsonl \
       --truth captures/truth_session.jsonl --lever F694=0,0,-0.04 --fit-dt
   ```

## Getting the most out of 3 trackers

- **Track the USB node.** The capture only contains the USB node's own
  ranges. Each range is checked against the true distance only when both
  ends are tracked, and that comparison, binned by first-path gap, is what
  the NLOS settings (`nlos.*`) get fitted to.
- **Track the carried node.** It's the one whose error matters most.
- **Leave the untracked node still.** It is left out of the scores, but it
  still shapes the mesh.

## Lever arm

The tracker measures its own pose, not the UWB antenna's.

- **For a still node,** the offset is a constant shift that the frame fit
  absorbs.
- **For a carried, tilting node,** it doesn't. Tilting 30° with the antenna
  5 cm below the tracker moves the antenna about 2.5 cm sideways.
- **Mount rigidly,** with the tracker directly above the antenna.
- **Give the offset with `--lever NODE=x,y,z`** (m), in the tracker's frame.

The direction of libsurvive's tracker axes hasn't been checked yet. To
check it, rock the carried node about its antenna. The right lever is the
one that keeps the antenna's truth position still.

## Reading the report

| Section | What it shows |
|---|---|
| pair | Estimated minus true distance for each tracked pair. No frame fit, so nothing can hide in it. |
| global | One 2-D rigid fit for the whole session, mirror allowed (ranges can't tell a layout from its mirror image). Includes frame drift. |
| per-snapshot | A rigid fit per swarm record: shape error alone. |
| range error by gap | Measured minus true range for the USB node's links. The NLOS model's input. |

`--fit-dt` finds the clock offset between the two streams from the ranges,
which are raw measurements. So the filter's own lag still counts as position
error, rather than being fitted away.
