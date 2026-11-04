// Hull glow: the LEDs reach out toward a person seen by a Kinect.
//
// tools/kinect/kinect_bridge.py streams an approximate hull of the person (a
// 14-direction bounding polytope, room frame, metres) as four mat4 uniforms. Plane
// i of block b is column i of hullb: xyz unit normal, w the support offset, so a
// point p is on the inside of the plane when dot(n, p) - w < 0. The signed
// distance to the polytope is the largest of the 14 (negative inside).
//
// The hull is grown outwards by `grow` metres and intersected with the LED array
// (led.room, the shared room frame; the fixture's pose must be set in the room).
// An LED lights when it is inside the grown hull; the nearer the person, the
// brighter and warmer it gets. `present` is the bridge's "someone is there" flag.
uniform mat4  hull0;
uniform mat4  hull1;
uniform mat4  hull2;
uniform mat4  hull3;
uniform float present : 0.0 .. 1.0 = 0.0;
uniform float grow    : 0.05 .. 2.0 "m" = 0.6;    // how far the hull is grown outwards
uniform float soft    : 0.02 .. 1.0 "m" = 0.25;   // feather at the grown boundary
uniform float fade    : 0.1 .. 5.0 "s" = 0.6;     // fade in/out when someone arrives/leaves
uniform float hue_near : 0.0 .. 1.0 = 0.02;       // colour at the person (red-orange)
uniform float hue_far  : 0.0 .. 1.0 = 0.60;       // colour at the grown boundary (blue)

state float level;

void update() {
  float target = step(0.5, present);
  level = clamp(level + (target * 2.0 - 1.0) * dt / fade, 0.0, 1.0);
}

vec3 shade(Led led) {
  vec3 p = led.room;
  float s = dot(hull0[0].xyz, p) - hull0[0].w;
  s = max(s, dot(hull0[1].xyz, p) - hull0[1].w);
  s = max(s, dot(hull0[2].xyz, p) - hull0[2].w);
  s = max(s, dot(hull0[3].xyz, p) - hull0[3].w);
  s = max(s, dot(hull1[0].xyz, p) - hull1[0].w);
  s = max(s, dot(hull1[1].xyz, p) - hull1[1].w);
  s = max(s, dot(hull1[2].xyz, p) - hull1[2].w);
  s = max(s, dot(hull1[3].xyz, p) - hull1[3].w);
  s = max(s, dot(hull2[0].xyz, p) - hull2[0].w);
  s = max(s, dot(hull2[1].xyz, p) - hull2[1].w);
  s = max(s, dot(hull2[2].xyz, p) - hull2[2].w);
  s = max(s, dot(hull2[3].xyz, p) - hull2[3].w);
  s = max(s, dot(hull3[0].xyz, p) - hull3[0].w);
  s = max(s, dot(hull3[1].xyz, p) - hull3[1].w);
  s = max(s, dot(hull3[2].xyz, p) - hull3[2].w);
  s = max(s, dot(hull3[3].xyz, p) - hull3[3].w);
  // 0 at the person's hull, 1 at the grown boundary.
  float t = clamp(s / grow, 0.0, 1.0);
  float edge = 1.0 - smoothstep(grow - soft, grow, s);
  float v = edge * (1.0 - 0.6 * t) * level;
  return hsv2rgb(mix(hue_near, hue_far, t), 1.0, v);
}
