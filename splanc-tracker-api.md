# Driving a Splanc fixture from a tracked pose (API handoff)

Checked against firmware **1.12.7**. Everything here works on it unless marked "coming later".

Audience: the agent building the tracker app (other repo). It reads Vive Tracker 3.0
poses, lets the user place a virtual plane, and streams that plane to a Splanc fixture,
which lights the LEDs where the fixture crosses the plane. Splanc provides:

- the fixture,
- its LED map,
- the effect that runs on the fixture,
- an OSC input that sets the effect's parameters.

Your app does the pose maths and sends the result.

## 1. Data flow

```
SteamVR / tracker agent ──pose T_WT──▶ your app ──OSC /plane (4 floats)──▶ fixture (udp/9000)
                                         ▲
             fixture LED map (CSV, once) ┘   + calibration T_TF (once per mount)
```

The fixture renders each LED from its own map position `p` (metres, the fixture's map
frame). It needs only the plane expressed in that frame, as `n·p = d`. Everything else
happens in your app.

## 2. Frames and maths

| Symbol | Meaning |
| --- | --- |
| W | Tracking world (SteamVR). Metres. Note its handedness and up axis; SteamVR is right-handed with +Y up. |
| T | The tracker's own frame. |
| F | The fixture's LED map frame. Metres, +Y up. The CSV in §4 is in this frame. |
| `T_WT` | The tracker pose in W, streamed by the tracker agent. |
| `T_TF` | The fixture map frame relative to the tracker (rigid, from calibration, §5). |
| `T_WF = T_WT · T_TF` | The fixture pose in W, updated on every tracker sample. |

The plane is held in W as a unit normal `n_W` and offset `d_W` (points `x` with
`n_W·x = d_W`). Write `T_WF` as `x_W = s·R·p + t`, where `s` is the map scale from
calibration (1 if the map is metrically exact). Then:

```
n_F = Rᵀ · n_W                 (unit length; R is a pure rotation)
d_F = (d_W − n_W · t) / s
send /plane  n_F.x  n_F.y  n_F.z  d_F
```

On every frame, the fixture lights the LEDs with `|n_F·p − d_F| < width`.

## 3. Sending to the fixture: OSC over UDP

- **Endpoint:** UDP to `<fixture IP>:9000`. Get the IP from Splanc (`list_players`; the
  fixture's name, e.g. PixieLamp) or from mDNS `<name>.local`.
- **Network:** OSC is only accepted once the fixture holds a DHCP lease. A fixture on a
  link-local `169.254.x.x` address ignores it. Put the fixture and your app on the same
  ordinary network (DHCP, 2.4 GHz). There's no auth, so it's LAN-only.
- **Message:** OSC 1.0, address `/plane`, type tag `,ffff`, four big-endian float32:
  `n_F.x n_F.y n_F.z d_F`. The other forms below also work:
  - one component: `/plane/x` `,f` (also `y`, `z`, `w`);
  - any parameter by name: `/width` `,f`, `/glow` `,fff`;
  - `#bundle` (several messages in one datagram).
- **Semantics:**
  - Latest value wins.
  - The value is applied at the next frame boundary (frames are about 12–40 ms; the
    fixture picks its own frame rate).
  - No ack. Values aren't persisted, so after a reboot the effect's defaults return.
- **Rate:** send on every tracker update, capped around 60–120 Hz, and also re-send at
  ≥ 5 Hz while idle, since UDP can drop. Sending faster than the fixture's frame rate
  just overwrites.
- **Ranges:** the OSC path writes values as sent (no clamping). Normalise `n_F`
  yourself. Splanc's own `set_uniforms` currently refuses negative vector components;
  that is being fixed (the effect-language PR adding signed ranges and `mat3`/`mat4`
  uniforms).
- **Coming later:** a `mat4 pose` parameter (16 floats), so the fixture can do the
  transform itself. The OSC address for that will be set by the PR above; this doc will
  be updated when it lands. For now `/plane` is the API.

Minimal sender (Python, stdlib):

```python
import socket, struct
def osc(addr, *floats):
    pad = lambda b: b + b"\0" * (4 - len(b) % 4)
    return pad(addr.encode()) + pad(("," + "f" * len(floats)).encode()) + b"".join(struct.pack(">f", f) for f in floats)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(osc("/plane", nx, ny, nz, d), (fixture_ip, 9000))
```

### `plane_slice` effect (checked against PixieLamp's firmware, 763 B bytecode)

```glsl
// Light the LEDs where the fixture crosses a plane streamed over OSC.
// plane: xyz = unit normal, w = offset (metres), both in the fixture's map frame:
// LEDs with |dot(plane.xyz, led.pos) - plane.w| < width glow. Set by OSC /plane ,ffff.
uniform vec4 plane = vec4(0.0, 1.0, 0.0, 0.1);
uniform float width : 0.002 .. 0.1 = 0.01;       // slice half-thickness, metres
uniform vec3 glow = vec3(1.0, 0.4, 0.1);          // slice colour
uniform float halo : 0.0 .. 0.2 = 0.03;          // soft falloff beyond width, metres

vec3 shade() {
  float d = abs(dot(plane.xyz, led.pos) - plane.w);
  float k = 1.0 - smoothstep(width, width + halo, d);
  return glow * k;
}
```

Deploy it with MCP `deploy_effect` (§5b). On today's firmware, `plane` has no declared
range, so Splanc shows it as a colour (0..1) and `set_uniforms` refuses negative values.
OSC is unaffected. Once the signed-range PR lands, declare it as
`uniform vec4 plane : -5.0 .. 5.0 = ...` so the UI and `set_uniforms` accept it too.

## 4. The fixture's LED map (point cloud)

Splanc exports it as CSV. Your app reads the file; it doesn't talk to the fixture for
this. Splanc's MCP `export_map` (format `csv`) produces it, and the user or the Splanc
agent hands you the file.

```
index,P(0),P(1),P(2),mapped,confidence,channel,channel_index,segment,arclength
0,-0.00731,0.11893,-0.01799,1,0,0,0,0,0
```

- `P(0..2)` = x, y, z in metres, frame F, +Y up.
- One row per LED, in output order. `mapped=0` rows are interpolated, so weight them
  low in fits.
- The map came from a camera solve, so its absolute scale can be off by a few percent.
  Check it once against a ruler, or let the calibration fit estimate `s`.
- Ignore `segment`/`arclength` for now (issue #396).

## 5. Calibration (tracker ↔ fixture), one tracker

There's one tracker, used first as the probe, then mounted on the fixture. **The fixture
must not move from the first rub until step 5 records the mounted tracker's pose.**

### Procedure

1. **Rub.** The user sweeps the tracker's contact face (the flat base with the 1/4"
   mount) over the fixture's surface. Record `T_WT` samples, a few hundred to a few
   thousand, from as many surface regions and tracker orientations as possible.
2. **Fit the surface.** Find `T_WF` (6 DoF), the tracker's contact point `c` in the T
   frame, the surface offset `o`, and optionally the map scale `s`. Minimise
   `Σ ρ( dist(T_WT,i · c, surface_F(T_WF)) − o )²`:
   - `ρ` is a robust loss (Huber or Cauchy), because lift-offs and edges are outliers.
   - The surface comes from the LED cloud: the signed distance to a local plane fitted
     through the k nearest LEDs, with its normal pointing out of the fixture.
   - `o` covers LEDs sitting under a diffuser and the tracker housing between its origin
     and the contact face. Initialise `c` at the centre of the tracker's base and `o` at
     0. The first fit is rough: surface-only fits are degenerate on symmetric fixtures
     (step 3 fixes that).
3. **Point correspondences: light LED k → place the tracker on it → hold → accept.**
   Repeat.
   - **Light LED k:** deploy the `led_pick` effect (source below), then send OSC
     `/pick ,f <k>`. The chosen LED blinks white, and the rest glow dimly so the outline
     stays visible.
   - **Place:** the user puts the tracker's contact point on that LED.
   - **Hold:** detect stillness. For example, over 1 s the position spread is < 1 mm and
     the rotation spread is < 0.5°. Then accept the mean pose and give audio or visual
     feedback. Each accepted touch adds the residual `T_WT · c − (T_WF · p_k + o · n_k)`,
     with `n_k` the local outward normal at LED k.
   - **Re-fit:** after every accepted touch, re-fit everything (rub plus touches).
   - **Choose the next k** to be most informative: far from the LEDs already touched, and
     on a different side of the fixture. On a cylinder, go around the axis and to both
     ends; this is what pins the rotation about the axis.
   - **Vary the tracker's tilt between touches.** A single orientation can't separate
     `c` from `T_WF`.
4. **Converge.** Stop when the parameters move less than about 0.5 mm / 0.2° between
   re-fits, and the touch residual RMS is below a few mm. Typically that takes 4–8
   touches. Show the residual per touch and drop or redo outliers. More than 1 cm usually
   means a wrong `c`, the wrong scale, or the fixture moved.
5. **Mount and record.** Attach the tracker to the fixture without moving the fixture.
   Hold still, take the mean `T_WT`, and compute `T_TF = T_WT⁻¹ · T_WF`. Save `T_TF` (and
   `s`) for this mount. From now on, `T_WF = T_WT · T_TF` live.

The map scale `s` came from a camera solve. Estimate it only once there are touches
spread across the whole fixture; otherwise it trades off against `o`.

### `led_pick` effect (Splanc deploys it; checked against PixieLamp's firmware)

```glsl
// Calibration helper: light one LED (pick) so a probe can be placed on it.
uniform float pick : 0.0 .. 2047.0 = 0.0;   // LED index to light (OSC /pick ,f)
uniform vec3 color = vec3(1.0, 1.0, 1.0);    // the picked LED
uniform float dim : 0.0 .. 0.2 = 0.02;       // everything else, so the fixture outline stays visible
state float blink;

void update() {
  blink = fract(blink + dt * 2.0);
}

vec3 shade() {
  if (led.idx == int(pick + 0.5)) {
    return color * (0.6 + 0.4 * step(0.5, blink));
  }
  return vec3(dim, dim, dim);
}
```

`k` is the CSV's `index` column. The app switches between the calibration effect and
`plane_slice` with MCP `deploy_effect` (§5b), or the user does it. OSC sets parameters
only; it can't change the running effect.

**On firmware 1.12.7, switch effects with `deploy_effect`, never `crossfade_to`.** OSC
always writes deck A. `deploy_effect` loads onto deck A, but `crossfade_to` loads the new
effect onto deck B and fades to it, after which OSC changes the hidden deck and the
fixture stops responding. If a crossfade has happened, a plain `deploy_effect` puts
things back on deck A (`list_decks` shows which deck is visible).

## 5b. Splanc MCP tools (deploying and switching effects)

You have Splanc's MCP tools, so you can write, deploy and switch effects yourself during
the build and calibration. There are two ways to connect:

- **Through the user's relay (works from anywhere):**
  - Connect to `https://splanc.pages.dev/mcp` with the header
    `Authorization: Bearer <token>`.
  - Get your own token: the user runs `ssh root@splanc-bridge.local splanc-bridge
    new-code` and enters the code at `https://splanc.pages.dev/pair`. Each agent pairs
    with its own code.
  - In Claude Code: `claude mcp add --transport http splanc https://splanc.pages.dev/mcp
    --header "Authorization: Bearer <token>"`.
  - If you call it from a script, send a real `User-Agent` header. Cloudflare blocks
    default library user agents such as `Python-urllib` (error 1010).
- **Local stdio (`npx -y https://splanc.io/agent/splanc-agent.tgz`):** only works on a
  machine on the same LAN as the fixtures. It finds them by mDNS or a LAN sweep. It sees
  nothing from inside a container or a different subnet.

Tools you'll likely use:

| Tool | Use |
| --- | --- |
| `list_players` / `refresh_players` | find the fixture, its IP (for OSC) and firmware |
| `export_map {format: "csv"}` | the LED point cloud (§4) |
| `compile_effect` | check an effect against the fixture's firmware without touching it |
| `deploy_effect {source, activate: true, players}` | run `led_pick` / `plane_slice` |
| `list_decks` | which deck is visible (OSC drives deck A only on 1.12.7) |
| `set_uniforms` / `list_uniforms` | one-off parameter changes (stream with OSC instead) |
| `measure_effect` | frame rate / budget check after deploying |
| `rollback_effect` | go back to the previous effect (after a deploy it re-uploads to deck A) |

Rules:
- **Private library.** The user's effects library is private. Don't copy library entries
  into your repo.
- **Deploys get saved to the library automatically.** Give your deploys clear names
  (`name`, `description`, `tags`), e.g. "Tracker calibration: LED pick", and reuse them
  with `library_id` instead of re-deploying source each time, or the library fills with
  copies.
- **Don't overwrite** (`update_effect` / `delete_effect`) entries you didn't create
  without asking the user.
- **`force` on deploys.** The cost estimator is pessimistic (issue #399). Use
  `force: true` only after `measure_effect` shows the effect really fits. `led_pick` and
  `plane_slice` are both tiny.
- **Don't `commit: true`** uniform changes while streaming (flash wear).
- **Don't use `update_firmware`, `set_device_name`, `trust_player` or `forget_player`**
  without asking the user.

## 6. What to show in the workspace (your app, for now)

- The fixture's LED cloud transformed by `T_WF` (live).
- The tracker pose.
- The plane, with a handle to move and rotate it.
- Optionally, the LEDs currently within `width` of the plane highlighted. That's a cheap
  preview of what the fixture should be showing, and a good end-to-end check.

## 7. Contract summary

| Item | Value |
| --- | --- |
| Transport | OSC 1.0 over UDP, `<fixture-ip>:9000`, DHCP-leased network |
| Main message | `/plane ,ffff  n_F.x n_F.y n_F.z d_F` (unit normal, metres, fixture map frame F) |
| Other params | `/width ,f` (metres, the slice half-thickness), `/halo ,f` (metres, soft edge), `/glow ,fff` (RGB 0..1) |
| Map | CSV from Splanc `export_map`, metres, +Y up, frame F |
| Rate | ≤ ~120 Hz, ≥ 5 Hz keep-alive; latest value wins at the next frame |
| Effect | `plane_slice` (deployed by Splanc), parameters `plane`, `width`, `glow` |
| Calibration effect | `led_pick`, `/pick ,f <LED index>` lights that LED; `/dim ,f`, `/color ,fff` |
