// Range pulse: the whole fixture breathes in one colour; a live UWB range
// drives the pulse rate and hue (tools/splanc/range_bridge.py sets them).
// The phase is integrated in update() so a changing rate never jumps.
uniform float rate  : 0.1 .. 6.0 = 1.0;    // pulses per second
uniform float hue   : 0.0 .. 1.0 = 0.0;    // 0 red .. 0.33 green .. 0.66 blue
uniform float depth : 0.0 .. 1.0 = 0.85;   // how dark the trough gets
uniform float sat   : 0.0 .. 1.0 = 1.0;

state float phase;
state float level;
state vec3 color;

void update() {
  phase = fract(phase + dt * rate);
  // raised cosine, peak at phase 0
  level = 1.0 - depth * (0.5 - 0.5 * cos(phase * 6.2831853));
  color = hsv2rgb(hue, sat, 1.0);
}

vec3 shade(Led led) {
  return color * level;
}
