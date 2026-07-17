//! C ABI over [`Estimator`] so the Python simulation (ctypes) — and later any
//! C/C++ embedded shell — can drive the filter. One opaque handle per widget.
//!
//! All vectors are little flat f32 arrays; quaternions are scalar-first
//! [w, x, y, z]. Every function is safe against a null handle (no-op / 0).

use crate::estimator::{Estimator, Params};
use crate::math::{Quat, Vec3};

fn vec3(ptr: *const f32) -> Vec3 {
    let s = unsafe { core::slice::from_raw_parts(ptr, 3) };
    Vec3::new(s[0], s[1], s[2])
}

/// Create an estimator. Noise densities as in [`Params`]; pass 0 for any
/// field to take the built-in default.
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
) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_range(vec3(neighbor_p), neighbor_var, measured, sigma) as i32,
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
) -> i32 {
    match unsafe { e.as_mut() } {
        Some(e) => e.update_aoa(vec3(neighbor_p), neighbor_var, vec3(bearing_body), sigma) as i32,
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
