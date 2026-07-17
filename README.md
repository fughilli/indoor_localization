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

## Layout (planned)

| Path | What |
| --- | --- |
| `third_party/embedded` | `firmware` bzlmod module (toolchains, cores, rules) |
| `libs/dw3000/` | shared register-level DWM3000 driver |
| `apps/anchor/` | anchor firmware — `anchor{1,2,3}.bin` (ID via define) |
| `apps/tag/` | tag firmware — DS-TWR cycle + WiFi/TCP JSON streaming |
| `host/` | Python: TCP ingest, trilateration solver, web floor view, simulator |

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

## Status

Bootstrap phase — see `WORKLOG.md`. Building requires the container overlay
(Nix) to be active; flashing and calibration steps will be documented once the
firmware builds green.
