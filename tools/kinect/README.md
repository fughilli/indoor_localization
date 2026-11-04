# Kinect v2 → hull → Splanc lamp

A Kinect v2 on a USB 3.0 box (NanoPi R6S) drives the lamp: the LEDs reach out
toward whoever is in view, brighter and warmer the closer they are.

```
Kinect v2 ──USB3──▶ kinect_bridge.py ──OSC/UDP :9000──▶ Splanc player (P4)
                    (R6S: libfreenect2)                  hull_glow.fx
```

- `hull.py` — the pipeline (numpy only): block-median decimation of the depth
  frame to ~32×26 cells, background subtraction, unprojection, a 14-direction
  bounding polytope (6 axes + 8 diagonals; support function per direction) and
  the OSC packet. The planes keep their identity between frames, so nothing
  flickers; ~2.6 ms/frame on an M-series laptop.
- `kinect_bridge.py` — sources (`freenect2`, `synth`, `npy`), background
  learning, rate limiting, status line. `--dry-run` prints instead of sending.
- `hull_glow.fx` — the effect. Uniforms `hull0..hull3` (mat4, plane *i* = column
  *i* = `nx ny nz h`), `present`, and the knobs `grow` (how far the hull is grown
  outwards, m), `soft`, `fade`, `hue_near`, `hue_far`. Uses `led.room`, so the
  fixture must be posed in the room frame (`set_fixture_pose`).
- `hull_test.py` — `cd tools/kinect && python3 -m unittest hull_test`.

## Using it

1. Deploy `hull_glow.fx` to the lamp (`deploy_effect`, or compile it first with
   `compile_effect`: 1850 B).
2. Place the Kinect in the room frame: `--cam-pos X Y Z` (m, y up) and
   `--cam-pitch` / `--cam-yaw` (degrees). Measure it, or fit it later with the
   Vive tracker held at known points in view.
3. Run with the room empty for `--learn-bg` seconds, then step in:
   `python3 kinect_bridge.py --host <lamp> --cam-pos 0 1.2 0 --cam-pitch 10 --learn-bg 3 --bg-file bg.npy`

## Not done yet

- Not run against a real Kinect (the `freenect2` source is untested) or the lamp.
- Packaging for the R6S through sbc-deploy (libfreenect2 + pylibfreenect2, udev
  rule for 045e:02c4, a systemd service).
- A P4 watchdog module: if the bridge dies, the last hull stays up (the bridge
  sends `present 0` on a clean exit only).
- A tracked fixture: the effect uses `led.room`; for a moving lamp, transform the
  planes by `pose` in `update()` as `plane_slice_pose.glsl` does.
- One hull for everyone in view; separate people would need clustering.
