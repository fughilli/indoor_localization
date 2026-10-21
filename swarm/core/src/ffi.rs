//! C ABI over [`Estimator`] so the Python simulation (ctypes) — and later any
//! C/C++ embedded shell — can drive the filter. One opaque handle per widget.
//!
//! All vectors are little flat f32 arrays; quaternions are scalar-first
//! [w, x, y, z]. Every function is safe against a null handle (no-op / 0).
//!
//! Host-only (`ffi` feature): handles are heap-allocated, so this pulls in
//! std; the firmware links the no_std crate without it.
extern crate std;
use std::boxed::Box;

use crate::coop::{self, CoopConfig, Decision, Outcome, PeerView, Regime, SkipReason};
use crate::estimator::{Estimator, Params};
use crate::math::{Quat, Vec3};

fn vec3(ptr: *const f32) -> Vec3 {
    let s = unsafe { core::slice::from_raw_parts(ptr, 3) };
    Vec3::new(s[0], s[1], s[2])
}

/// Create an estimator. Noise densities as in [`Params`]; pass 0 for any
/// field to take the built-in default. `gate < 0` disables gating.
#[no_mangle]
pub extern "C" fn swarm_estimator_new(
    accel_noise: f32,
    gyro_noise: f32,
    accel_bias_rw: f32,
    gyro_bias_rw: f32,
    gate: f32,
) -> *mut Estimator {
    let d = Params::default();
    let pick = |v: f32, dv: f32| if v > 0.0 { v } else { dv };
    let params = Params {
        accel_noise: pick(accel_noise, d.accel_noise),
        gyro_noise: pick(gyro_noise, d.gyro_noise),
        accel_bias_rw: pick(accel_bias_rw, d.accel_bias_rw),
        gyro_bias_rw: pick(gyro_bias_rw, d.gyro_bias_rw),
        gate: if gate >= 0.0 { pick(gate, d.gate) } else { 0.0 },
        ..d
    };
    Box::into_raw(Box::new(Estimator::new(params)))
}

#[no_mangle]
pub extern "C" fn swarm_estimator_free(e: *mut Estimator) {
    if !e.is_null() {
        drop(unsafe { Box::from_raw(e) });
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_set_pose(
    e: *mut Estimator,
    p: *const f32,
    v: *const f32,
    q: *const f32,
    pos_sigma: f32,
    vel_sigma: f32,
    att_sigma: f32,
) {
    let Some(e) = (unsafe { e.as_mut() }) else { return };
    let qs = unsafe { core::slice::from_raw_parts(q, 4) };
    e.set_pose(
        vec3(p),
        vec3(v),
        Quat::from_array(&[qs[0], qs[1], qs[2], qs[3]]),
        pos_sigma,
        vel_sigma,
        att_sigma,
    );
}

#[no_mangle]
pub extern "C" fn swarm_estimator_predict(
    e: *mut Estimator,
    accel: *const f32,
    gyro: *const f32,
    dt: f32,
) {
    if let Some(e) = unsafe { e.as_mut() } {
        e.predict(vec3(accel), vec3(gyro), dt);
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_update_range(
    e: *mut Estimator,
    neighbor_p: *const f32,
    neighbor_var: f32,
    measured: f32,
    sigma: f32,
    mean_only: i32,
) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => {
            e.update_range(vec3(neighbor_p), neighbor_var, measured, sigma, mean_only != 0) as i32
        }
        None => 0,
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_update_aoa(
    e: *mut Estimator,
    neighbor_p: *const f32,
    neighbor_var: f32,
    bearing_body: *const f32,
    sigma: f32,
    mean_only: i32,
) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_aoa(
            vec3(neighbor_p),
            neighbor_var,
            vec3(bearing_body),
            sigma,
            mean_only != 0,
        ) as i32,
        None => 0,
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_update_gps(
    e: *mut Estimator,
    measured: *const f32,
    sigma_h: f32,
    sigma_v: f32,
) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_gps(vec3(measured), sigma_h, sigma_v) as i32,
        None => 0,
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_update_alt(e: *mut Estimator, z: f32, sigma: f32) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_alt(z, sigma) as i32,
        None => 0,
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_update_zero_vel(e: *mut Estimator, sigma: f32) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_zero_vel(sigma) as i32,
        None => 0,
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_update_mag(
    e: *mut Estimator,
    mag_body: *const f32,
    mag_world: *const f32,
    sigma: f32,
) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_mag(vec3(mag_body), vec3(mag_world), sigma) as i32,
        None => 0,
    }
}

#[no_mangle]
pub extern "C" fn swarm_estimator_update_gravity(
    e: *mut Estimator,
    accel: *const f32,
    sigma: f32,
) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_gravity(vec3(accel), sigma) as i32,
        None => 0,
    }
}

/// Copy the nominal state into out[17]: p(3), v(3), q(4, wxyz), ba(3), bg(3),
/// mean position variance (1).
#[no_mangle]
pub extern "C" fn swarm_estimator_state(e: *const Estimator, out: *mut f32) {
    let Some(e) = (unsafe { e.as_ref() }) else { return };
    let out = unsafe { core::slice::from_raw_parts_mut(out, 17) };
    out[0..3].copy_from_slice(&e.p.to_array());
    out[3..6].copy_from_slice(&e.v.to_array());
    out[6..10].copy_from_slice(&e.q.to_array());
    out[10..13].copy_from_slice(&e.ba.to_array());
    out[13..16].copy_from_slice(&e.bg.to_array());
    out[16] = e.pos_var();
}

/// Range-innovation lock ratio (EWMA of |y|/sigma, ~1 = locked, 10 = none).
#[no_mangle]
pub extern "C" fn swarm_estimator_range_innov_ratio(e: *const Estimator) -> f32 {
    match unsafe { e.as_ref() } {
        Some(e) => e.range_innov_ratio(),
        None => 0.0,
    }
}

/// Copy the 3x3 position covariance block (row-major) into out[9].
#[no_mangle]
pub extern "C" fn swarm_estimator_pos_cov(e: *const Estimator, out: *mut f32) {
    let Some(e) = (unsafe { e.as_ref() }) else { return };
    let out = unsafe { core::slice::from_raw_parts_mut(out, 9) };
    for i in 0..3 {
        for j in 0..3 {
            out[i * 3 + j] = e.cov[i][j];
        }
    }
}

/// Number of floats in a flattened [`CoopConfig`] (field order as declared).
pub const COOP_CONFIG_LEN: usize = 9;

fn coop_config(cfg: *const f32) -> CoopConfig {
    if cfg.is_null() {
        return CoopConfig::default();
    }
    let c = unsafe { core::slice::from_raw_parts(cfg, COOP_CONFIG_LEN) };
    CoopConfig {
        downhill_ratio: c[0],
        mean_only_aoa_damping: c[1],
        mean_only_range_damping: c[2],
        lock_ratio_full: c[3],
        lock_ratio_none: c[4],
        peer_lock_factor: c[5],
        max_advert_age: c[6],
        aoa_downhill_max_var: c[7],
        mean_only_pos_only: c[8] != 0.0,
    }
}

fn peer_view(p: *const f32, var: f32, lock: f32, age: f32) -> PeerView {
    PeerView { p: vec3(p), var, lock, age, anchored: false, still: false }
}

/// Cooperative range update ([`coop::range_update`]). `cfg` is a flattened
/// CoopConfig (COOP_CONFIG_LEN floats) or null for the defaults. Writes the
/// decision to `decision_out` (3 floats: regime 0=skip/1=downhill/
/// 2=mean-only, nvar, my_var) for a following AoA update. Returns 1 accepted,
/// 0 gated, -1 stale advert, -2 anchored, -3 cold peer.
#[no_mangle]
pub extern "C" fn swarm_coop_range_update(
    e: *mut Estimator,
    cfg: *const f32,
    peer_p: *const f32,
    peer_var: f32,
    peer_lock: f32,
    peer_age: f32,
    measured: f32,
    range_sigma: f32,
    anchored: i32,
    decision_out: *mut f32,
) -> i32 {
    let Some(e) = (unsafe { e.as_mut() }) else { return 0 };
    let peer = peer_view(peer_p, peer_var, peer_lock, peer_age);
    let (outcome, decision) =
        coop::range_update(&coop_config(cfg), e, &peer, measured, range_sigma, anchored != 0, false);
    if !decision_out.is_null() {
        let out = unsafe { core::slice::from_raw_parts_mut(decision_out, 3) };
        let flat: [f32; 3] = match decision {
            Decision::Skip(_) => [0.0, 0.0, 0.0],
            Decision::Use { regime, nvar, my_var } => {
                [if regime == Regime::Downhill { 1.0 } else { 2.0 }, nvar, my_var]
            }
        };
        out.copy_from_slice(&flat);
    }
    match outcome {
        Outcome::Applied { accepted, .. } => accepted as i32,
        Outcome::Skipped(SkipReason::StaleAdvert) => -1,
        Outcome::Skipped(SkipReason::Anchored) => -2,
        Outcome::Skipped(SkipReason::ColdPeer) => -3,
        Outcome::Skipped(SkipReason::MovingPeer) => -7,
    }
}

/// Cooperative AoA update ([`coop::aoa_update`]) under the 3-float decision
/// a preceding swarm_coop_range_update wrote. Returns accepted components.
#[no_mangle]
pub extern "C" fn swarm_coop_aoa_update(
    e: *mut Estimator,
    cfg: *const f32,
    peer_p: *const f32,
    decision: *const f32,
    bearing_body: *const f32,
    aoa_sigma: f32,
) -> i32 {
    let Some(e) = (unsafe { e.as_mut() }) else { return 0 };
    let d = unsafe { core::slice::from_raw_parts(decision, 3) };
    let decision = match d[0] as i32 {
        1 => Decision::Use { regime: Regime::Downhill, nvar: d[1], my_var: d[2] },
        2 => Decision::Use { regime: Regime::MeanOnly, nvar: d[1], my_var: d[2] },
        _ => return 0,
    };
    // AoA reads only the peer position from the view.
    let peer = peer_view(peer_p, 0.0, 0.0, 0.0);
    coop::aoa_update(&coop_config(cfg), e, &peer, decision, vec3(bearing_body), aoa_sigma) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ffi_round_trip() {
        let e = swarm_estimator_new(0.0, 0.0, 0.0, 0.0, 0.0);
        let p = [1.0f32, 2.0, 3.0];
        let v = [0.0f32; 3];
        let q = [1.0f32, 0.0, 0.0, 0.0];
        swarm_estimator_set_pose(e, p.as_ptr(), v.as_ptr(), q.as_ptr(), 1.0, 0.1, 0.1);
        let mut out = [0.0f32; 17];
        swarm_estimator_state(e, out.as_mut_ptr());
        assert_eq!(&out[0..3], &[1.0, 2.0, 3.0]);
        assert!((out[16] - 1.0).abs() < 1e-5);
        swarm_estimator_free(e);
    }
}
