//! C ABI for the firmware shell (apps/node). One node per firmware, held in a
//! static: every call must come from the same task (the shell's fusion task
//! owns it; the radio task hands it frames through a queue). Vectors are flat
//! f32 arrays; times are microseconds on one monotonic clock (esp_timer).

use core::cell::UnsafeCell;

use swarm_core::math::{Mat3, Vec3};

use crate::advert::MAX_ENCODED;
use crate::node::{Node, NodeConfig, RangeResult, MAX_PEERS};

struct Singleton(UnsafeCell<Option<Node>>);
// Safety: single-task access is the documented contract of this ABI.
unsafe impl Sync for Singleton {}
static NODE: Singleton = Singleton(UnsafeCell::new(None));

fn node() -> Option<&'static mut Node> {
    unsafe { (*NODE.0.get()).as_mut() }
}

fn vec3(p: *const f32) -> Vec3 {
    let s = unsafe { core::slice::from_raw_parts(p, 3) };
    Vec3::new(s[0], s[1], s[2])
}

/// Largest advertisement swarm_node_advert_tx writes.
pub const SWARM_NODE_ADVERT_MAX: usize = 64;
/// Peer table capacity (swarm_node_peer indices are 0..this).
pub const SWARM_NODE_MAX_PEERS: usize = 16;
// Literals for cbindgen (it parses this file alone); pinned to the source.
const _: () = assert!(SWARM_NODE_ADVERT_MAX == MAX_ENCODED);
const _: () = assert!(SWARM_NODE_MAX_PEERS == MAX_PEERS);
/// Floats written by swarm_node_state.
pub const SWARM_NODE_STATE_LEN: usize = 13;
/// Floats written by swarm_node_peer.
pub const SWARM_NODE_PEER_LEN: usize = 12;

/// (Re)create the node. `range_sigma` <= 0 keeps the default (0.10 m).
#[no_mangle]
pub extern "C" fn swarm_node_init(id: u32, has_imu: bool, range_sigma: f32) {
    let mut cfg = NodeConfig::default();
    if range_sigma > 0.0 {
        cfg.range_sigma = range_sigma;
    }
    unsafe { *NODE.0.get() = Some(Node::new(id, has_imu, cfg)) };
}

/// Magnetometer calibration: calibrated = soft (row-major 3x3) * (raw - hard).
/// Until set, magnetometer samples are ignored.
#[no_mangle]
pub extern "C" fn swarm_node_set_mag_calibration(hard: *const f32, soft: *const f32) {
    let Some(n) = node() else { return };
    let s = unsafe { core::slice::from_raw_parts(soft, 9) };
    let m: Mat3 = [[s[0], s[1], s[2]], [s[3], s[4], s[5]], [s[6], s[7], s[8]]];
    n.set_mag_calibration(vec3(hard), m);
}

/// Pin to a known fixed position (m); this widget then anchors the swarm.
#[no_mangle]
pub extern "C" fn swarm_node_set_surveyed(p: *const f32, sigma: f32) {
    if let Some(n) = node() {
        n.set_surveyed(vec3(p), if sigma > 0.0 { sigma } else { 0.02 });
    }
}

#[no_mangle]
pub extern "C" fn swarm_node_clear_surveyed() {
    if let Some(n) = node() {
        n.clear_surveyed();
    }
}

/// One IMU sample: accel m/s^2 (specific force), gyro rad/s, body frame.
#[no_mangle]
pub extern "C" fn swarm_node_imu(t_us: i64, accel: *const f32, gyro: *const f32) {
    if let Some(n) = node() {
        n.imu(t_us, vec3(accel), vec3(gyro));
    }
}

/// IMU-less propagation; call every few ms (no-op when an IMU is fitted).
#[no_mangle]
pub extern "C" fn swarm_node_tick(t_us: i64) {
    if let Some(n) = node() {
        n.tick(t_us);
    }
}

/// Raw magnetometer sample (sensor units, body frame).
#[no_mangle]
pub extern "C" fn swarm_node_mag(t_us: i64, raw: *const f32) {
    if let Some(n) = node() {
        n.mag(t_us, vec3(raw));
    }
}

/// A UWB range to `peer_id`; `fp_gap_db`: total received power minus the
/// first path's (dB; NaN if unknown). Returns a RangeResult code: 2 used for
/// bias calibration (anchored), 1 accepted, 0 gated, -1 stale advert, -2
/// anchored, -3 cold peer, -4 unknown peer, -5 surveyed, -6 not initialized,
/// -7 moving peer, -8 blocked link.
#[no_mangle]
pub extern "C" fn swarm_node_range(t_us: i64, peer_id: u32, range_m: f32, fp_gap_db: f32) -> i32 {
    match node() {
        Some(n) => n.range(t_us, peer_id, range_m, fp_gap_db) as i32,
        None => RangeResult::NotReady as i32,
    }
}

/// An advertisement off the air. Returns the sender id, or 0 if it was
/// malformed, our own, or the peer table is full.
#[no_mangle]
pub extern "C" fn swarm_node_advert_rx(t_us: i64, bytes: *const u8, len: usize) -> u32 {
    let Some(n) = node() else { return 0 };
    let b = unsafe { core::slice::from_raw_parts(bytes, len) };
    n.advert_rx(t_us, b).unwrap_or(0)
}

/// Encode my advertisement into `out` (cap bytes). Returns its length, 0 on
/// failure (buffer too small).
#[no_mangle]
pub extern "C" fn swarm_node_advert_tx(out: *mut u8, cap: usize) -> usize {
    let Some(n) = node() else { return 0 };
    let b = unsafe { core::slice::from_raw_parts_mut(out, cap) };
    n.advert_tx(b).unwrap_or(0)
}

/// My state: p[3] v[3] q[4] (w,x,y,z) pos_var lock range_bias — SWARM_NODE_STATE_LEN
/// floats. Returns false until the filter is initialized.
#[no_mangle]
pub extern "C" fn swarm_node_state(out: *mut f32) -> bool {
    let Some(n) = node() else { return false };
    let o = unsafe { core::slice::from_raw_parts_mut(out, SWARM_NODE_STATE_LEN) };
    let e = &n.est;
    o[0..3].copy_from_slice(&e.p.to_array());
    o[3..6].copy_from_slice(&e.v.to_array());
    o[6..10].copy_from_slice(&e.q.to_array());
    o[10] = e.pos_var();
    o[11] = e.range_innov_ratio();
    o[12] = e.rb;
    n.initialized()
}

/// Peer table entry `index` (0..SWARM_NODE_MAX_PEERS) as of `t_us`: p[3]
/// (dead-reckoned) v[3] var lock age_s flags seq range_bias — SWARM_NODE_PEER_LEN
/// floats. Returns the peer id, 0 for an empty or expired slot.
#[no_mangle]
pub extern "C" fn swarm_node_peer(index: usize, t_us: i64, out: *mut f32) -> u32 {
    let Some(n) = node() else { return 0 };
    let Some(Some(p)) = n.peers.get(index) else { return 0 };
    if t_us - p.rx_us > crate::node::PEER_EXPIRY_US {
        return 0;
    }
    let view = p.view(t_us);
    let o = unsafe { core::slice::from_raw_parts_mut(out, SWARM_NODE_PEER_LEN) };
    o[0..3].copy_from_slice(&view.p.to_array());
    o[3..6].copy_from_slice(&p.advert.v.to_array());
    o[6] = p.advert.quality;
    o[7] = p.advert.lock;
    o[8] = view.age;
    o[9] = p.advert.flags as f32;
    o[10] = p.advert.seq as f32;
    o[11] = p.advert.range_bias;
    p.advert.widget_id
}

/// The boot-time static IMU calibration: accel_scale, gyro_bias[3], still
/// (1/0) — 5 floats. Returns false until it has run.
#[no_mangle]
pub extern "C" fn swarm_node_imu_cal(out: *mut f32) -> bool {
    let Some(n) = node() else { return false };
    let Some(c) = n.imu_cal() else { return false };
    let o = unsafe { core::slice::from_raw_parts_mut(out, 5) };
    o[0] = c.accel_scale;
    o[1..4].copy_from_slice(&c.gyro_bias.to_array());
    o[4] = if c.still { 1.0 } else { 0.0 };
    true
}

// ---- settings ---------------------------------------------------------------
//
// The registry (settings.rs) is read-only and safe from any task. Settings
// values live in swarm_net's stack, owned by the radio task, which calls
// swarm_node_stage_settings() after a change; the fusion task picks the
// result up with swarm_node_take_settings(). The shell wraps both calls in
// the same critical section (they share PENDING).

use crate::settings::{self as reg, Kind, Tunables, REGISTRY};

struct Pending(UnsafeCell<Option<Tunables>>);
unsafe impl Sync for Pending {}
static PENDING: Pending = Pending(UnsafeCell::new(None));

pub const SWARM_NODE_SETTING_NAME_MAX: usize = 32;

#[no_mangle]
pub extern "C" fn swarm_node_setting_count() -> usize {
    REGISTRY.len()
}

/// Registry entry `i`: id, NUL-terminated name (SWARM_NODE_SETTING_NAME_MAX
/// bytes), kind (0 f32, 1 i32, 2 bool), range, default (raw). False past
/// the end.
#[no_mangle]
pub unsafe extern "C" fn swarm_node_setting_info(i: usize, id: *mut u16, name: *mut u8, kind: *mut i32,
                                                 min: *mut f32, max: *mut f32, default_raw: *mut u32) -> bool {
    let Some(s) = REGISTRY.get(i) else { return false };
    *id = s.id;
    let n = s.name.len().min(SWARM_NODE_SETTING_NAME_MAX - 1);
    core::ptr::copy_nonoverlapping(s.name.as_ptr(), name, n);
    *name.add(n) = 0;
    *kind = match s.kind {
        Kind::F32 => 0,
        Kind::I32 => 1,
        Kind::Bool => 2,
    };
    *min = s.min;
    *max = s.max;
    *default_raw = s.default_raw();
    true
}

/// Registry index of `name` (not NUL-terminated), or -1.
#[no_mangle]
pub unsafe extern "C" fn swarm_node_setting_find(name: *const u8, len: usize) -> i32 {
    let Ok(n) = core::str::from_utf8(core::slice::from_raw_parts(name, len)) else { return -1 };
    REGISTRY.iter().position(|s| s.name == n).map_or(-1, |i| i as i32)
}

/// Raw value of setting `i` for a number the shell parsed (bools: 0/1),
/// range-checked. (No text parsing here: Rust's float parser needs 128-bit
/// integer helpers the firmware link doesn't provide.)
#[no_mangle]
pub unsafe extern "C" fn swarm_node_setting_from_number(i: usize, value: f64, raw: *mut u32) -> bool {
    let Some(s) = REGISTRY.get(i) else { return false };
    let r = match s.kind {
        Kind::F32 => (value as f32).to_bits(),
        Kind::I32 => {
            if value != libm::round(value) {
                return false;
            }
            value as i32 as u32
        }
        Kind::Bool => match value {
            v if v == 0.0 => 0,
            v if v == 1.0 => 1,
            _ => return false,
        },
    };
    if !s.valid(r) {
        return false;
    }
    *raw = r;
    true
}

/// Write raw value `raw` of setting `i` as JSON into `out`; its length.
#[no_mangle]
pub unsafe extern "C" fn swarm_node_setting_format(i: usize, raw: u32, out: *mut u8, cap: usize) -> usize {
    struct W<'a>(&'a mut [u8], usize);
    impl core::fmt::Write for W<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let b = s.as_bytes();
            if self.1 + b.len() > self.0.len() {
                return Err(core::fmt::Error);
            }
            self.0[self.1..self.1 + b.len()].copy_from_slice(b);
            self.1 += b.len();
            Ok(())
        }
    }
    let Some(s) = REGISTRY.get(i) else { return 0 };
    let mut w = W(core::slice::from_raw_parts_mut(out, cap), 0);
    if reg::format(s, raw, &mut w).is_err() {
        return 0;
    }
    w.1
}

/// Radio task, after swarm_net reported a settings change: resolve the
/// layers, apply the mesh part right away (same task), stage the rest for
/// the fusion task. Returns the antenna delay (the radio applies it).
#[no_mangle]
pub extern "C" fn swarm_node_stage_settings() -> i32 {
    let t = swarm_net::ffi::with_net(|n, _| {
        let t = reg::resolve(|k| n.settings.swarm.get(k), |k| n.settings.overrides.get(k));
        n.mesh.cfg = t.mesh;
        t
    })
    .unwrap_or_default();
    unsafe { *PENDING.0.get() = Some(t) };
    t.antenna_delay
}

/// Fusion task: apply staged settings to the node, if any. True if applied.
#[no_mangle]
pub extern "C" fn swarm_node_take_settings() -> bool {
    let Some(t) = (unsafe { (*PENDING.0.get()).take() }) else { return false };
    match node() {
        Some(n) => {
            n.apply(&t);
            true
        }
        None => false,
    }
}

/// Bare metal: a panic resets the chip through the C runtime's abort()
/// (newlib on the ESP32 prints a backtrace and reboots).
#[cfg(target_os = "none")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    extern "C" {
        fn abort() -> !;
    }
    unsafe { abort() }
}
