//! Per-widget error-state Kalman filter (ESKF).
//!
//! Nominal state: position p, velocity v (world ENU, meters), attitude q
//! (body-to-world), accelerometer bias ba, gyro bias bg. The 15-dim error
//! state [dp, dv, dtheta, dba, dbg] carries the covariance; attitude error is
//! body-side (R_true = R_est * exp([dtheta]x)).
//!
//! Propagation consumes raw IMU samples. Corrections are processed as
//! sequential scalar updates (no matrix inversion anywhere), each guarded by
//! an innovation gate so UWB multipath outliers get rejected. Cooperative
//! measurements (range / AoA to a neighbor) take the neighbor's advertised
//! position variance and fold it into the measurement noise — a conservative
//! alternative to tracking cross-covariances between widgets.

use crate::math::*;

pub const GRAVITY: f32 = 9.80665;

/// Indices of the error-state blocks.
const DP: usize = 0;
const DV: usize = 3;
const DTH: usize = 6;
const DBA: usize = 9;
const DBG: usize = 12;

/// Noise model, expressed as continuous-time densities (per sqrt(Hz)).
#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// Accelerometer white noise, m/s^2/sqrt(Hz).
    pub accel_noise: f32,
    /// Gyro white noise, rad/s/sqrt(Hz).
    pub gyro_noise: f32,
    /// Accelerometer bias random walk, m/s^3/sqrt(Hz).
    pub accel_bias_rw: f32,
    /// Gyro bias random walk, rad/s^2/sqrt(Hz).
    pub gyro_bias_rw: f32,
    /// 1-sigma of the initial bias uncertainty.
    pub init_accel_bias_sigma: f32,
    pub init_gyro_bias_sigma: f32,
    /// Innovation gate in sigmas for scalar updates; 0 disables gating.
    pub gate: f32,
    /// EWMA coefficient for the range-innovation lock statistic.
    pub innov_alpha: f32,
}

/// Initial / saturated value of the range-innovation ratio: "no lock".
/// Individual samples are capped here too, so multipath outliers nudge the
/// EWMA instead of blowing it out.
pub const INNOV_RATIO_MAX: f32 = 10.0;

impl Default for Params {
    fn default() -> Params {
        Params {
            // Ballpark consumer MEMS (e.g. ICM-42688 class).
            accel_noise: 0.02,
            gyro_noise: 0.0015,
            accel_bias_rw: 0.0005,
            gyro_bias_rw: 2e-5,
            init_accel_bias_sigma: 0.05,
            init_gyro_bias_sigma: 0.01,
            gate: 5.0,
            innov_alpha: 0.02,
        }
    }
}

/// Which part of an update's K*y goes into the nominal state.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Inject {
    All,
    /// Position only. A mean-only update never shrinks P, so its
    /// cross-covariance gains keep their full size update after update; a
    /// persistent innovation (an anchorless mesh that can't close) would
    /// otherwise be integrated into velocity and the biases without bound.
    Position,
    /// Everything but position (see `update_zero_vel_masked`).
    AllButPosition,
}

#[derive(Clone, Debug)]
pub struct Estimator {
    pub p: Vec3,
    pub v: Vec3,
    pub q: Quat,
    pub ba: Vec3,
    pub bg: Vec3,
    pub cov: Cov,
    pub params: Params,
    /// EWMA of |range innovation| / sigma — a self-measured "location lock"
    /// signal. Near 1 the position is consistent with incoming ranges at
    /// measurement-noise level; large means unconverged (or recently
    /// disturbed). The network layer uses it to anneal the mean-only
    /// relaxation gain, which a Kalman filter cannot do natively in the
    /// anchorless regime: mean-only updates deliberately never shrink the
    /// covariance (the absolute frame is unobservable), so the gain would
    /// otherwise stay pinned at cold-start levels forever.
    range_innov_ratio: f32,
}

impl Estimator {
    /// A fresh estimator: origin pose with very loose position/velocity/
    /// attitude covariance ("I could be anywhere").
    pub fn new(params: Params) -> Estimator {
        let mut e = Estimator {
            p: Vec3::ZERO,
            v: Vec3::ZERO,
            q: Quat::IDENTITY,
            ba: Vec3::ZERO,
            bg: Vec3::ZERO,
            cov: cov_zero(),
            params,
            range_innov_ratio: INNOV_RATIO_MAX,
        };
        e.reset_cov(100.0, 1.0, 1.0);
        e
    }

    /// Overwrite pose and reset the covariance to the given 1-sigma values.
    pub fn set_pose(
        &mut self,
        p: Vec3,
        v: Vec3,
        q: Quat,
        pos_sigma: f32,
        vel_sigma: f32,
        att_sigma: f32,
    ) {
        self.p = p;
        self.v = v;
        self.q = q.normalized();
        self.reset_cov(pos_sigma, vel_sigma, att_sigma);
        self.range_innov_ratio = INNOV_RATIO_MAX;
    }

    fn reset_cov(&mut self, pos_sigma: f32, vel_sigma: f32, att_sigma: f32) {
        self.cov = cov_zero();
        for i in 0..3 {
            self.cov[DP + i][DP + i] = pos_sigma * pos_sigma;
            self.cov[DV + i][DV + i] = vel_sigma * vel_sigma;
            self.cov[DTH + i][DTH + i] = att_sigma * att_sigma;
            self.cov[DBA + i][DBA + i] =
                self.params.init_accel_bias_sigma * self.params.init_accel_bias_sigma;
            self.cov[DBG + i][DBG + i] =
                self.params.init_gyro_bias_sigma * self.params.init_gyro_bias_sigma;
        }
    }

    /// Mean position variance (m^2) — the "quality" a widget advertises.
    pub fn pos_var(&self) -> f32 {
        (self.cov[DP][DP] + self.cov[DP + 1][DP + 1] + self.cov[DP + 2][DP + 2]) / 3.0
    }

    /// Current range-innovation lock ratio (EWMA of |y|/sigma, capped at
    /// [`INNOV_RATIO_MAX`]). ~1 = locked to the mesh, large = unconverged.
    pub fn range_innov_ratio(&self) -> f32 {
        self.range_innov_ratio
    }

    /// IMU strapdown propagation: one raw accel (m/s^2, body, includes
    /// gravity reaction) + gyro (rad/s, body) sample over dt seconds.
    pub fn predict(&mut self, accel: Vec3, gyro: Vec3, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let a_b = accel.sub(self.ba);
        let w_b = gyro.sub(self.bg);
        let r = self.q.to_mat3();
        let g = Vec3::new(0.0, 0.0, -GRAVITY);
        let a_w = mat3_mul_vec(&r, a_b).add(g);

        // Nominal state.
        self.p = self.p.add(self.v.scale(dt)).add(a_w.scale(0.5 * dt * dt));
        self.v = self.v.add(a_w.scale(dt));
        self.q = self.q.mul(Quat::from_rotvec(w_b.scale(dt))).normalized();

        // Discrete transition F = I + A*dt (first order).
        let mut f = cov_zero();
        for i in 0..STATE_DIM {
            f[i][i] = 1.0;
        }
        let i_dt: Mat3 = [[dt, 0.0, 0.0], [0.0, dt, 0.0], [0.0, 0.0, dt]];
        cov_set_block(&mut f, DP, DV, &i_dt);
        // dv_dot = -R [a_b]x dtheta - R dba
        let r_ab = mat3_mul(&r, &skew(a_b));
        cov_set_block(&mut f, DV, DTH, &mat3_scale(&r_ab, -dt));
        cov_set_block(&mut f, DV, DBA, &mat3_scale(&r, -dt));
        // dtheta_dot = -[w_b]x dtheta - dbg
        let mut th_th = mat3_scale(&skew(w_b), -dt);
        for i in 0..3 {
            th_th[i][i] += 1.0;
        }
        cov_set_block(&mut f, DTH, DTH, &th_th);
        let neg_i_dt = mat3_scale(&i_dt, -1.0);
        cov_set_block(&mut f, DTH, DBG, &neg_i_dt);

        self.cov = cov_propagate(&self.cov, &f);

        // Additive process noise from the continuous densities.
        let qv = self.params.accel_noise * self.params.accel_noise * dt;
        let qth = self.params.gyro_noise * self.params.gyro_noise * dt;
        let qba = self.params.accel_bias_rw * self.params.accel_bias_rw * dt;
        let qbg = self.params.gyro_bias_rw * self.params.gyro_bias_rw * dt;
        for i in 0..3 {
            self.cov[DV + i][DV + i] += qv;
            self.cov[DTH + i][DTH + i] += qth;
            self.cov[DBA + i][DBA + i] += qba;
            self.cov[DBG + i][DBG + i] += qbg;
        }
        cov_symmetrize(&mut self.cov);
    }

    /// One scalar Kalman update; injects the error into the nominal state.
    /// Returns false if the innovation gate rejected the measurement.
    ///
    /// With `update_cov == false` this is a *mean-only* update: the state
    /// moves by K*y but the covariance is left untouched. Used for relative
    /// measurements between peers of comparable quality (no "downhill"
    /// direction): the estimates get pulled onto a mutually consistent mesh
    /// without ever claiming an information gain that would double-count
    /// shared history — covariance honesty is what keeps the swarm out of
    /// the overconfidence death spiral.
    ///
    /// Note on cooperative consistency: relative measurements fold the
    /// neighbor's advertised variance into `r`, and the *network layer* only
    /// takes covariance-reducing relative updates "downhill" (from
    /// better-localized peers). Whole-state scalar covariance intersection
    /// was tried here and removed: the 1/w prior inflation hits
    /// never-measured directions (biases, velocity) every update and grows
    /// their variance exponentially.
    fn scalar_update(
        &mut self,
        h: &[f32; STATE_DIM],
        y: f32,
        r: f32,
        update_cov: bool,
    ) -> bool {
        self.scalar_update_masked(h, y, r, update_cov, Inject::All)
    }

    /// [`scalar_update`](Self::scalar_update) injecting only part of K*y
    /// (see [`Inject`]).
    fn scalar_update_masked(
        &mut self,
        h: &[f32; STATE_DIM],
        y: f32,
        r: f32,
        update_cov: bool,
        inject: Inject,
    ) -> bool {
        let mut ph = [0.0f32; STATE_DIM];
        for i in 0..STATE_DIM {
            let mut s = 0.0;
            for j in 0..STATE_DIM {
                s += self.cov[i][j] * h[j];
            }
            ph[i] = s;
        }
        let mut a = 0.0;
        for i in 0..STATE_DIM {
            a += h[i] * ph[i];
        }
        if a + r <= 0.0 {
            return false;
        }
        let gate = self.params.gate;
        if gate > 0.0 && y * y > gate * gate * (a + r) {
            return false;
        }
        let s = a + r;
        let inv_s = 1.0 / s;
        let mut k = [0.0f32; STATE_DIM];
        for i in 0..STATE_DIM {
            k[i] = ph[i] * inv_s;
        }
        // Inject dx = K*y into the nominal state.
        let dx = |base: usize| Vec3::new(k[base] * y, k[base + 1] * y, k[base + 2] * y);
        if inject != Inject::AllButPosition {
            self.p = self.p.add(dx(DP));
        }
        if inject != Inject::Position {
            self.v = self.v.add(dx(DV));
            self.q = self.q.mul(Quat::from_rotvec(dx(DTH))).normalized();
            self.ba = self.ba.add(dx(DBA));
            self.bg = self.bg.add(dx(DBG));
        }
        if update_cov {
            // P <- P - K (P H^T)^T
            for i in 0..STATE_DIM {
                for j in 0..STATE_DIM {
                    self.cov[i][j] -= k[i] * ph[j];
                }
            }
            cov_symmetrize(&mut self.cov);
        }
        true
    }

    /// UWB two-way range (meters) to a neighbor whose advertised position is
    /// `np` with mean position variance `nvar` (m^2). `mean_only` for the
    /// symmetric (no-downhill) regime. Returns acceptance.
    pub fn update_range(
        &mut self,
        np: Vec3,
        nvar: f32,
        measured: f32,
        sigma: f32,
        mean_only: bool,
    ) -> bool {
        self.update_range_masked(np, nvar, measured, sigma, mean_only, false)
    }

    /// [`update_range`](Self::update_range); `pos_only` corrects position
    /// alone (see `scalar_update_masked`).
    pub fn update_range_masked(
        &mut self,
        np: Vec3,
        nvar: f32,
        measured: f32,
        sigma: f32,
        mean_only: bool,
        pos_only: bool,
    ) -> bool {
        let d = self.p.sub(np);
        let r_pred = d.norm();
        if r_pred < 0.05 {
            return false;
        }
        // Fold this innovation into the lock statistic (any regime: it
        // measures consistency with the mesh either way). Per-sample cap so
        // a multipath tail nudges the EWMA rather than resetting it.
        if sigma > 0.0 {
            let sample = (libm::fabsf(measured - r_pred) / sigma).min(INNOV_RATIO_MAX);
            let a = self.params.innov_alpha;
            self.range_innov_ratio += a * (sample - self.range_innov_ratio);
        }
        let u = d.scale(1.0 / r_pred);
        let mut h = [0.0f32; STATE_DIM];
        h[DP] = u.x;
        h[DP + 1] = u.y;
        h[DP + 2] = u.z;
        // Multipath outliers in the symmetric (mean-only) regime are absorbed,
        // not gated: the S-based gate is blind there (S is dominated by the
        // honestly-huge absolute variance) and a per-widget adaptive
        // innovation gate was tried and removed — endpoints gate the shared
        // range asymmetrically, and thousands of one-sided half-updates pump
        // net drift into the mesh (~10x worse relative error). Real NLOS
        // rejection belongs at the radio layer (CIR classification).
        let inject = if pos_only { Inject::Position } else { Inject::All };
        self.scalar_update_masked(&h, measured - r_pred, sigma * sigma + nvar, !mean_only, inject)
    }

    /// Move the position estimate by `dp` without touching the covariance:
    /// a gauge correction (e.g. re-centering an anchorless swarm), not a
    /// measurement.
    pub fn shift_position(&mut self, dp: Vec3) {
        self.p = self.p.add(dp);
    }

    /// UWB AoA: measured unit bearing to the neighbor in the *body* frame.
    /// Processed as three scalar updates on the bearing components. Returns
    /// how many components passed the gate.
    pub fn update_aoa(
        &mut self,
        np: Vec3,
        nvar: f32,
        bearing_body: Vec3,
        sigma: f32,
        mean_only: bool,
    ) -> u32 {
        let mut accepted = 0;
        for axis in 0..3 {
            let d_w = np.sub(self.p);
            let range = d_w.norm();
            if range < 0.05 {
                return accepted;
            }
            let u_w = d_w.scale(1.0 / range);
            let u_b = self.q.rotate_inv(u_w);
            // d(u_b)/d(dtheta) = [u_b]x ; d(u_b)/d(dp) = -R^T (I - u u^T)/range
            let su = skew(u_b);
            let rt = mat3_transpose(&self.q.to_mat3());
            let mut proj = MAT3_IDENTITY;
            let ua = u_w.to_array();
            for i in 0..3 {
                for j in 0..3 {
                    proj[i][j] -= ua[i] * ua[j];
                }
            }
            let dp_jac = mat3_scale(&mat3_mul(&rt, &proj), -1.0 / range);
            let mut h = [0.0f32; STATE_DIM];
            h[DP..DP + 3].copy_from_slice(&dp_jac[axis]);
            h[DTH..DTH + 3].copy_from_slice(&su[axis]);
            let y = [
                bearing_body.x - u_b.x,
                bearing_body.y - u_b.y,
                bearing_body.z - u_b.z,
            ][axis];
            // Neighbor position error projects onto the bearing as ~1/range.
            let r_eff = sigma * sigma + nvar / (range * range);
            if self.scalar_update(&h, y, r_eff, !mean_only) {
                accepted += 1;
            }
        }
        accepted
    }

    /// Absolute position fix (e.g. GPS), world ENU meters.
    pub fn update_gps(&mut self, measured: Vec3, sigma_h: f32, sigma_v: f32) -> u32 {
        let mut accepted = 0;
        let meas = measured.to_array();
        let sigmas = [sigma_h, sigma_h, sigma_v];
        for axis in 0..3 {
            let mut h = [0.0f32; STATE_DIM];
            h[DP + axis] = 1.0;
            let y = meas[axis] - self.p.to_array()[axis];
            if self.scalar_update(&h, y, sigmas[axis] * sigmas[axis], true) {
                accepted += 1;
            }
        }
        accepted
    }

    /// Altimeter: absolute height above the shared datum, meters.
    pub fn update_alt(&mut self, z: f32, sigma: f32) -> bool {
        let mut h = [0.0f32; STATE_DIM];
        h[DP + 2] = 1.0;
        self.scalar_update(&h, z - self.p.z, sigma * sigma, true)
    }

    /// Weak zero-velocity gauge prior: with no absolute-position widget in
    /// the swarm, the common-mode translation (and its velocity) is
    /// unobservable and would random-walk on IMU noise. Bleeding every
    /// widget's velocity toward zero with a large sigma pins the swarm
    /// frame to quasi-static — valid while the shared reference frame is
    /// not accelerating appreciably — at a negligible cost to genuinely
    /// moving widgets (sigma is chosen well above their real speeds).
    pub fn update_zero_vel(&mut self, sigma: f32) -> u32 {
        self.update_zero_vel_masked(sigma, false)
    }

    /// [`update_zero_vel`](Self::update_zero_vel); `keep_position` leaves
    /// the position mean alone. When mean-only range updates have been
    /// correcting position without touching P, P's position-velocity
    /// cross-covariance still claims every past velocity error is an
    /// uncorrected position error, and a full update re-applies a correction
    /// ranging already made: the position jumps every zero-velocity period.
    pub fn update_zero_vel_masked(&mut self, sigma: f32, keep_position: bool) -> u32 {
        let inject = if keep_position { Inject::AllButPosition } else { Inject::All };
        let mut accepted = 0;
        let v = self.v.to_array();
        for axis in 0..3 {
            let mut h = [0.0f32; STATE_DIM];
            h[DV + axis] = 1.0;
            if self.scalar_update_masked(&h, -v[axis], sigma * sigma, true, inject) {
                accepted += 1;
            }
        }
        accepted
    }

    /// Magnetometer: measured field direction (unit, body frame) vs the known
    /// world field direction (unit). Corrects attitude about two axes.
    pub fn update_mag(&mut self, mag_body: Vec3, mag_world: Vec3, sigma: f32) -> u32 {
        self.direction_update(mag_body.normalized(), mag_world.normalized(), sigma)
    }

    /// Gravity tilt correction from a quasi-static accel sample: the measured
    /// specific force direction should match body-frame "up". The caller is
    /// responsible for gating on |accel| ~ g.
    pub fn update_gravity(&mut self, accel: Vec3, sigma: f32) -> u32 {
        let dir = accel.sub(self.ba).normalized();
        self.direction_update(dir, Vec3::new(0.0, 0.0, 1.0), sigma)
    }

    /// Shared body-frame direction observation: measured_body should equal
    /// R^T * world_dir. Three sequential scalar updates on dtheta.
    fn direction_update(&mut self, measured_body: Vec3, world_dir: Vec3, sigma: f32) -> u32 {
        let mut accepted = 0;
        for axis in 0..3 {
            let pred_b = self.q.rotate_inv(world_dir);
            let su = skew(pred_b);
            let mut h = [0.0f32; STATE_DIM];
            h[DTH..DTH + 3].copy_from_slice(&su[axis]);
            let y = measured_body.to_array()[axis] - pred_b.to_array()[axis];
            if self.scalar_update(&h, y, sigma * sigma, true) {
                accepted += 1;
            }
        }
        accepted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quiet_params() -> Params {
        Params { gate: 0.0, ..Params::default() }
    }

    /// Perfect static IMU (only gravity reaction) must hold the pose.
    #[test]
    fn static_propagation_holds_pose() {
        let mut e = Estimator::new(quiet_params());
        e.set_pose(Vec3::new(1.0, 2.0, 3.0), Vec3::ZERO, Quat::IDENTITY, 1.0, 0.1, 0.1);
        let accel = Vec3::new(0.0, 0.0, GRAVITY);
        for _ in 0..500 {
            e.predict(accel, Vec3::ZERO, 0.01);
        }
        assert!((e.p.x - 1.0).abs() < 1e-3, "p.x = {}", e.p.x);
        assert!((e.p.z - 3.0).abs() < 1e-3, "p.z = {}", e.p.z);
        assert!(e.v.norm() < 1e-3);
        // Covariance must have grown, not collapsed.
        assert!(e.pos_var() > 1.0);
    }

    /// Noise-free ranges to four fixed, well-known points must pull a badly
    /// initialized position estimate onto the true position.
    #[test]
    fn ranges_to_known_points_converge() {
        let anchors = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(10.0, 10.0, 3.0),
        ];
        let truth = Vec3::new(3.0, 4.0, 1.0);
        let mut e = Estimator::new(quiet_params());
        e.set_pose(Vec3::new(6.0, 7.0, 0.0), Vec3::ZERO, Quat::IDENTITY, 10.0, 0.1, 0.1);
        for _ in 0..30 {
            e.predict(Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO, 0.01);
            for a in &anchors {
                let true_range = truth.sub(*a).norm();
                e.update_range(*a, 0.0, true_range, 0.1, false);
            }
        }
        let err = e.p.sub(truth).norm();
        assert!(err < 0.05, "position error {} m", err);
        assert!(e.pos_var() < 0.1);
    }

    /// GPS fixes alone must converge position and shrink covariance.
    #[test]
    fn gps_converges() {
        let mut e = Estimator::new(quiet_params());
        let truth = Vec3::new(-5.0, 8.0, 2.0);
        for _ in 0..20 {
            e.predict(Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO, 0.01);
            e.update_gps(truth, 1.0, 2.0);
        }
        assert!(e.p.sub(truth).norm() < 0.5);
        assert!(e.pos_var() < 0.5);
    }

    /// Mag + gravity must correct an initial yaw error.
    #[test]
    fn mag_corrects_yaw() {
        let mut e = Estimator::new(quiet_params());
        // True attitude: identity. Estimator starts with a 0.3 rad yaw error.
        e.set_pose(
            Vec3::ZERO,
            Vec3::ZERO,
            Quat::from_rotvec(Vec3::new(0.0, 0.0, 0.3)),
            1.0,
            0.1,
            0.5,
        );
        let mag_world = Vec3::new(1.0, 0.0, 0.4).normalized(); // north-ish + dip
        for _ in 0..50 {
            e.predict(Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO, 0.01);
            // True attitude = identity, so body measurements = world vectors.
            e.update_mag(mag_world, mag_world, 0.05);
            e.update_gravity(Vec3::new(0.0, 0.0, GRAVITY), 0.05);
        }
        assert!(e.q.yaw().abs() < 0.02, "residual yaw {}", e.q.yaw());
    }

    /// A mean-only update must move the state but leave covariance alone.
    #[test]
    fn mean_only_update_preserves_covariance() {
        let mut e = Estimator::new(quiet_params());
        e.set_pose(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO, Quat::IDENTITY, 2.0, 0.1, 0.1);
        let cov_before = e.cov;
        let p_before = e.p;
        // Range says the neighbor at origin is 3 m away (we think 1 m).
        assert!(e.update_range(Vec3::ZERO, 4.0, 3.0, 0.1, true));
        assert!(e.p.sub(p_before).norm() > 0.1, "state did not move");
        assert_eq!(e.cov, cov_before, "covariance changed on mean-only update");
    }

    /// The zero-velocity gauge prior must bleed velocity toward zero and
    /// barely disturb it within one update (weak prior).
    #[test]
    fn zero_vel_prior_bleeds_velocity() {
        let mut e = Estimator::new(quiet_params());
        e.set_pose(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, 1.0, 0.5, 0.1);
        assert_eq!(e.update_zero_vel(3.0), 3);
        assert!(e.v.x < 1.0);
        assert!(e.v.x > 0.9, "weak prior pulled too hard: {}", e.v.x);
        for _ in 0..200 {
            e.predict(Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO, 0.01);
            e.update_zero_vel(3.0);
        }
        // Velocity variance is bounded by the prior, not growing without limit.
        assert!(e.cov[DV][DV] < 9.5, "vel var {}", e.cov[DV][DV]);
    }

    /// The lock statistic must fall toward ~1 when ranges agree with the
    /// state at measurement-noise level, stay pinned high while innovations
    /// are large, and reset on set_pose.
    #[test]
    fn innov_ratio_tracks_lock() {
        let mut e = Estimator::new(quiet_params());
        e.set_pose(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO, Quat::IDENTITY, 2.0, 0.1, 0.1);
        assert!((e.range_innov_ratio() - INNOV_RATIO_MAX).abs() < 1e-6);

        // Consistent ranges (innovation ~0.5 sigma) anneal the ratio down.
        let anchor = Vec3::new(6.0, 0.0, 0.0);
        for _ in 0..300 {
            let d = e.p.sub(anchor).norm();
            e.update_range(anchor, 4.0, d + 0.05, 0.1, true);
        }
        assert!(e.range_innov_ratio() < 1.5, "ratio {}", e.range_innov_ratio());

        // Persistently large innovations re-inflate it (self-recovery when
        // the widget genuinely moves or the mesh shifts). Pin the state each
        // step so the 2 m disagreement is not absorbed.
        for _ in 0..300 {
            let d = e.p.sub(anchor).norm();
            e.update_range(anchor, 4.0, d + 2.0, 0.1, true);
            e.p = Vec3::new(1.0, 0.0, 0.0);
        }
        assert!(e.range_innov_ratio() > 5.0, "ratio {}", e.range_innov_ratio());

        // set_pose = cold restart: back to "no lock".
        e.set_pose(Vec3::ZERO, Vec3::ZERO, Quat::IDENTITY, 100.0, 1.0, 1.0);
        assert!((e.range_innov_ratio() - INNOV_RATIO_MAX).abs() < 1e-6);
    }

    /// The innovation gate must reject a wild multipath-style outlier.
    #[test]
    fn gate_rejects_outlier() {
        let mut e = Estimator::new(Params::default());
        e.set_pose(Vec3::ZERO, Vec3::ZERO, Quat::IDENTITY, 0.5, 0.1, 0.1);
        let anchor = Vec3::new(5.0, 0.0, 0.0);
        // True range 5 m, measured 50 m: gross outlier.
        assert!(!e.update_range(anchor, 0.0, 50.0, 0.1, false));
        // Sane measurement passes.
        assert!(e.update_range(anchor, 0.0, 5.1, 0.1, false));
    }
}
