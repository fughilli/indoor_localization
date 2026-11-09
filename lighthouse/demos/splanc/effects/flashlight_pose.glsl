// Flashlight: a cone of light from a point in the WORLD frame (the stylus tip,
// streamed by flashlight.py), lighting the LEDs inside it. Each LED's world
// position comes from the fixture's own tracker when it has one (the Splanc P4
// tracker module fills `pose`, world <- fixture map, and `tracker_ok`) and from
// its room pose otherwise (`led.room`, set_fixture_pose), so tracked and fixed
// fixtures share one beam. With `decay` > 0 the beam leaves a trail: each LED keeps the
// brighter of the beam now and its own glow fading with time constant `decay`.
uniform mat4 pose;                                  // pool-driven: world <- fixture map frame
uniform float tracker_ok : 0.0 .. 1.0 = 0.0;        // pool-driven: 1 while tracking
uniform vec3 origin : -10.0 .. 10.0 = vec3(0.0, 0.0, 2.0);  // beam apex (m, world; OSC /origin ,fff)
uniform vec3 dir : -1.0 .. 1.0 = vec3(0.0, 0.0, -1.0);      // beam axis (world; OSC /dir ,fff)
uniform float width : 0.01 .. 1.2 = 0.2;            // half-angle (rad; OSC /width ,f)
uniform float soft : 0.0 .. 0.5 = 0.15;             // edge, as a fraction of the half-angle
uniform vec3 color = vec3(1.0, 0.9, 0.7);           // beam colour (OSC /color ,fff)
uniform float reach : 0.5 .. 20.0 = 4.0;            // m: the light fades out by here
uniform float ambient : 0.0 .. 0.2 = 0.0;           // light outside the beam
uniform float own_pose : 0.0 .. 1.0 = 1.0;          // 1: use the tracker pose when tracking
uniform float decay : 0.0 .. 20.0 = 0.0;            // s: trail time constant, 0 = none (OSC /decay ,f)

buffer vec3 trail : fixed16;  // each LED's fading glow (fixed8 would stall a slow fade a step above black)

state vec3 axis;
state float c_in;   // cos of the inner (full-strength) angle
state float c_out;  // cos of the outer (dark) angle
state float keep;   // this frame's trail factor, exp(-dt / decay)

void update() {
  axis = normalize(dir);
  c_in = cos(width * (1.0 - soft));
  c_out = cos(width);
  keep = 0.0;
  if (decay > 0.0) {
    keep = exp(-dt / decay);
  }
}

vec3 shade() {
  vec3 p = led.room;
  if (own_pose > 0.5 && tracker_ok > 0.5) {
    p = (pose * vec4(led.pos, 1.0)).xyz;
  }
  vec3 v = p - origin;
  float r = length(v);
  float c = dot(v, axis) / max(r, 0.001);
  float k = smoothstep(c_out, c_in, c) * (1.0 - smoothstep(0.5 * reach, reach, r));
  vec3 lit = max(color * k, trail[led.idx] * keep);
  trail[led.idx] = lit;
  return max(lit, color * ambient);
}
