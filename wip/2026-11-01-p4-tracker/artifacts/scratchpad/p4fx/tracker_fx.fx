uniform vec3 tracker_pos = 0.0, 0.0, 0.0;
uniform vec4 tracker_quat = 1.0, 0.0, 0.0, 0.0;
uniform float tracker_ok : 0.0 .. 1.0 = 0.0;
vec3 shade(Led led) {
  float d = length(led.pos - tracker_pos);
  return vec3(tracker_ok * clamp(1.0 - d, 0.0, 1.0), 0.0, 1.0 - tracker_ok);
}
