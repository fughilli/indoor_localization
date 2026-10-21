//! Host C ABI for the simulation (swarm/sim, feature "sim"): any number of
//! Node instances by handle, so the sim runs the exact runtime the firmware
//! does — stationary detection, auto-anchoring, planar mode, centroid bleed,
//! the settings registry — against synthetic IMU, ranges and adverts. (The
//! firmware ABI in ffi.rs holds one global node instead.)

use std::boxed::Box;

use swarm_core::math::Vec3;

use crate::advert::MAX_ENCODED;
use crate::node::{Node, NodeConfig};
use crate::settings;

fn vec3(p: *const f32) -> Vec3 {
    let s = unsafe { core::slice::from_raw_parts(p, 3) };
    Vec3::new(s[0], s[1], s[2])
}

fn node<'a>(h: *mut Node) -> &'a mut Node {
    unsafe { &mut *h }
}

/// New node with the default configuration.
#[no_mangle]
pub extern "C" fn swarm_sim_node_new(id: u32, has_imu: bool) -> *mut Node {
    Box::into_raw(Box::new(Node::new(id, has_imu, NodeConfig::default())))
}

#[no_mangle]
pub extern "C" fn swarm_sim_node_free(h: *mut Node) {
    if !h.is_null() {
        unsafe { drop(Box::from_raw(h)) };
    }
}

/// Apply settings as the firmware would: `ids`/`raws` are registry ids and
/// raw values (an override layer over the defaults). Returns how many were
/// valid and applied.
#[no_mangle]
pub unsafe extern "C" fn swarm_sim_node_settings(h: *mut Node, ids: *const u16, raws: *const u32, n: usize) -> usize {
    let ids = core::slice::from_raw_parts(ids, n);
    let raws = core::slice::from_raw_parts(raws, n);
    let lookup = |k: u16| ids.iter().position(|&i| i == k).map(|i| raws[i]);
    let applied = ids
        .iter()
        .zip(raws)
        .filter(|(&id, &raw)| settings::by_id(id).is_some_and(|s| s.valid(raw)))
        .count();
    let t = settings::resolve(|_| None, lookup);
    node(h).apply(&t);
    applied
}

/// Registry id of a setting name (not NUL-terminated), or -1.
#[no_mangle]
pub unsafe extern "C" fn swarm_sim_setting_id(name: *const u8, len: usize, kind: *mut i32) -> i32 {
    let Ok(n) = core::str::from_utf8(core::slice::from_raw_parts(name, len)) else { return -1 };
    match settings::by_name(n) {
        Some(s) => {
            *kind = match s.kind {
                settings::Kind::F32 => 0,
                settings::Kind::I32 => 1,
                settings::Kind::Bool => 2,
            };
            s.id as i32
        }
        None => -1,
    }
}

#[no_mangle]
pub extern "C" fn swarm_sim_node_imu(h: *mut Node, t_us: i64, accel: *const f32, gyro: *const f32) {
    node(h).imu(t_us, vec3(accel), vec3(gyro));
}

#[no_mangle]
pub extern "C" fn swarm_sim_node_tick(h: *mut Node, t_us: i64) {
    node(h).tick(t_us);
}

#[no_mangle]
pub extern "C" fn swarm_sim_node_mag(h: *mut Node, t_us: i64, raw: *const f32) {
    node(h).mag(t_us, vec3(raw));
}

/// RangeResult code (see ffi.rs).
#[no_mangle]
pub extern "C" fn swarm_sim_node_range(h: *mut Node, t_us: i64, peer: u32, range_m: f32, fp_gap_db: f32) -> i32 {
    node(h).range(t_us, peer, range_m, fp_gap_db) as i32
}

/// Encoded advertisement into `out` (>= 64 bytes); its length or 0.
#[no_mangle]
pub unsafe extern "C" fn swarm_sim_node_advert_tx(h: *mut Node, out: *mut u8, cap: usize) -> usize {
    let buf = core::slice::from_raw_parts_mut(out, cap.min(MAX_ENCODED));
    node(h).advert_tx(buf).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn swarm_sim_node_advert_rx(h: *mut Node, t_us: i64, data: *const u8, len: usize) -> u32 {
    node(h).advert_rx(t_us, core::slice::from_raw_parts(data, len)).unwrap_or(0)
}

/// State for the sim's metrics: p(3) v(3) q(4) pos_var lock still anchored
/// initialized regains range_bias — 17 floats.
#[no_mangle]
pub unsafe extern "C" fn swarm_sim_node_state(h: *mut Node, t_us: i64, out: *mut f32) {
    let n = node(h);
    let o = core::slice::from_raw_parts_mut(out, 17);
    let e = &n.est;
    o[0..3].copy_from_slice(&e.p.to_array());
    o[3..6].copy_from_slice(&e.v.to_array());
    o[6..10].copy_from_slice(&e.q.to_array());
    o[10] = e.pos_var();
    o[11] = e.range_innov_ratio();
    o[12] = n.is_still(t_us) as u32 as f32;
    o[13] = n.auto_anchored() as u32 as f32;
    o[14] = n.initialized() as u32 as f32;
    o[15] = n.regains as f32;
    o[16] = e.rb;
}
