# UWB Indoor Positioning System — ESP32-WROOM + Qorvo DWM3000

Real-time indoor location (RTLS): three fixed **anchors** and one mobile
**tag**, each an ESP32-WROOM driving a Qorvo DWM3000 UWB module over SPI.
Tag ↔ anchor pairs measure distance with Double-Sided Two-Way Ranging
(DS-TWR); the tag streams per-anchor distances as newline-JSON over WiFi/TCP
(port 7007) to a host that solves a least-squares trilateration and serves a
live web floor view (port 8080).

Derived from CircuitDigest's
[ESP32-DWM3000-UWB-Indoor-RTLS-Tracker](https://github.com/Circuit-Digest/ESP32-DWM3000-UWB-Indoor-RTLS-Tracker)
(GPL-3.0; see `LICENSE`) — the register-level DWM3000 driver and DS-TWR state
machines originate there. Article:
[How to Build an UWB Indoor Positioning System using ESP32 and Qorvo DWM3000](https://circuitdigest.com/microcontrollers-projects/diy-indoor-uwb-positioning-system-using-esp32-and-qorvo-dwm3000).

Built with **Bazel (bzlmod) + Nix** on the
[`firmware`](https://github.com/fughilli/embedded) module (vendored at
`third_party/embedded`): Nix supplies the xtensa cross toolchain, the
arduino-esp32 core, and esptool; Bazel compiles/links and packages flashable
`.bin` images.

## Layout

| Path | What |
| --- | --- |
| `third_party/embedded` | `firmware` bzlmod module (toolchains, cores, rules) |
| `libs/dw3000/` | shared register-level DWM3000 driver |
| `apps/anchor/` | anchor firmware — `anchor_{1,2,3}_esp32.bin` (ID via define) |
| `apps/tag/` | tag firmware — DS-TWR cycle + WiFi/TCP JSON streaming |
| `host/` | Python: TCP ingest, trilateration solver, web floor view, simulator |

## Building

Everything goes through Bazelisk (first build fetches the Nix toolchain —
expect several GB and many minutes; later builds hit `.bazel-*-cache/`):

```sh
bazelisk build //...          # all firmware images
bazelisk test //host/...      # solver + full-stack integration tests
```

Flashable images land in `bazel-bin/apps/anchor/anchor_{1,2,3}_esp32.bin` and
`bazel-bin/apps/tag/tag_esp32.bin`.

## Wiring (all four boards)

| DWM3000 | ESP32 | Function |
| --- | --- | --- |
| VCC | 3.3V | Power |
| GND | GND | Ground |
| SCK | GPIO18 | SPI clock |
| MISO | GPIO19 | SPI data in |
| MOSI | GPIO23 | SPI data out |
| CS | GPIO4 | Chip select |
| RST | GPIO27 | Reset |
| IRQ | GPIO34 | Interrupt |

## Bring-up

### 1. Configure the tag's WiFi + host address

Create `apps/tag/config_local.h` (gitignored):

```c
#define UWB_WIFI_SSID "my-network"
#define UWB_WIFI_PASSWORD "my-password"
#define UWB_HOST_IP "192.168.1.50"   // machine that will run host/server.py
#define UWB_HOST_PORT 7007
```

### 2. Flash the four boards

Each anchor binary has its ANCHOR_ID (1/2/3) baked in; flash a different one
to each board:

```sh
bazelisk run //apps/anchor:flash_anchor_1 -- --port /dev/ttyUSB0
bazelisk run //apps/anchor:flash_anchor_2 -- --port /dev/ttyUSB0
bazelisk run //apps/anchor:flash_anchor_3 -- --port /dev/ttyUSB0
bazelisk run //apps/tag:flash_tag -- --port /dev/ttyUSB0
```

Serial monitor at 115200 baud shows `> ANCHOR n - Ready for ranging <` /
`> TAG - Three Anchor Ranging System <` on boot.

### 3. Place the anchors and start the host

Anchor positions live in floor-plan centimetres, measured from the room's
bottom-left corner. Defaults match the reference floor plan
(A1 `[15,5]`, A2 `[290,5]`, A3 `[165,625]`, room 480×650 cm) — pass your own:

```sh
bazelisk run //host:server -- \
    --anchors '[[15,5],[290,5],[165,625]]' --room 480 650
```

Open `http://<host>:8080/` for the live floor view (anchors, tag dot, trail).
`/state` returns the latest solve as JSON.

For a headless demo without hardware, feed the server synthetic packets:

```sh
bazelisk run //host:simulator -- --host 127.0.0.1   # orbiting tag
```

### 4. Antenna-delay calibration

Ranges carry a per-board constant offset until the TX antenna delay is
calibrated. Both firmwares default to `16350` (set in `setup()` via
`DWM3000.setTXAntennaDelay(16350)`). To calibrate: place tag and one anchor a
measured distance apart (e.g. 100 cm), read the reported distance from the
tag's serial output, and nudge the constant (larger delay ⇒ shorter reported
range, roughly 2.2 cm per 10 units) in `apps/anchor/anchor.cpp` /
`apps/tag/tag.cpp` until it matches. Repeat per board if your modules vary.

## Status

Firmware and host stack build green and are test-verified in software
(esptool image-info validates all four images; simulator → server → SSE
round-trips in `//host:integration_test`). **Not yet run on hardware** — see
`WORKLOG.md` for the current state of play.
