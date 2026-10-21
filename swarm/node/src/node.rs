//! One widget's runtime: its estimator, the peers it hears, and the timing of
//! the periodic aiding updates — everything firmware does between "a sensor
//! sample / radio frame arrived" and "here is my pose". Hardware-free and
//! allocation-free; the firmware shell (apps/node) feeds it through the C ABI
//! in ffi.rs, host tests drive it directly.
//!
//! Rates and sigmas mirror the simulation (swarm/sim/world.py), whose results
//! the design doc reports; cooperative rules are swarm_core::coop itself.

use swarm_core::coop::{self, CoopConfig, Decision, Outcome, PeerView, SkipReason};
use swarm_core::estimator::{Estimator, Params, GRAVITY};
use swarm_core::math::{Mat3, Quat, Vec3, MAT3_IDENTITY};

use crate::advert::{self, Advert, FLAG_ANCHORED, FLAG_GPS, FLAG_STATIC};

pub const MAX_PEERS: usize = 16;

/// Peers silent for this long drop out of the table (and the swarm report).
/// Ranging already ignores adverts older than CoopConfig::max_advert_age.
pub const PEER_EXPIRY_US: i64 = 5_000_000;

#[derive(Clone, Copy, Debug)]
pub struct NodeConfig {
    pub params: Params,
    pub coop: CoopConfig,
    /// UWB range noise, m (1 sigma).
    pub range_sigma: f32,
    /// Gravity tilt aiding on every Nth IMU sample, when | |a| - g | < gate.
    pub gravity_every: u32,
    pub gravity_gate: f32,
    pub gravity_sigma: f32,
    /// Weak zero-velocity gauge prior (anchorless swarm frame pinning).
    pub zero_vel_sigma: f32,
    pub zero_vel_period_us: i64,
    /// The zero-velocity update leaves position alone (see
    /// Estimator::update_zero_vel_masked). With position-only mean-only
    /// ranging a full update makes positions jump every zero_vel_period_us
    /// (the zigzag of a carried node), but that same jump is the only thing
    /// trimming position drift from accumulated velocity error: keeping
    /// position removes the jumps and lets a constant accel error drift ~5x
    /// further. Off until the anchorless mover case is tuned (WORKLOG).
    pub zero_vel_keep_position: bool,
    /// Anchorless gauge: with no absolute source in range (nobody surveyed or
    /// on GPS), every widget bleeds its position toward minus the swarm
    /// centroid it sees (itself + fresh peer adverts) with this time
    /// constant, so the centroid relaxes to the origin instead of wandering
    /// with the integrated IMU errors. Relative geometry is untouched (each
    /// widget applies the same shift). <= 0 disables.
    pub centroid_bleed_tau_s: f32,
    pub centroid_period_us: i64,
    /// Planar deployment (every widget at about one height): an altitude
    /// pseudo-measurement holds z at `planar_z`. Without it a flat swarm is
    /// ill-conditioned in z (a widget inside a triangle of others can pop
    /// out of the plane either way to absorb range bias, and flips).
    /// Stationary detection (ZUPT): when the bias-corrected gyro rate and
    /// the deviation of |accel| from g both stay under their thresholds
    /// (EWMA, time constant `still_tau_us`) for `still_hold_us`, the widget
    /// is still, and velocity is pinned to zero with sigma `still_sigma`
    /// every `still_period_us`. Velocity and biases only, never position
    /// (no kick). Nothing else corrects velocity once range updates are
    /// position-only, and a few cm/s of IMU drift kept a still mesh flexing
    /// by tens of cm.
    pub still_on: bool,
    pub still_gyro: f32,
    pub still_accel: f32,
    pub still_tau_us: i64,
    pub still_hold_us: i64,
    pub still_sigma: f32,
    pub still_period_us: i64,
    pub planar_on: bool,
    pub planar_z: f32,
    pub planar_sigma: f32,
    pub planar_period_us: i64,
    /// Surveyed (fixed, known-position) widgets re-pin to it at this rate.
    pub survey_period_us: i64,
    pub survey_sigma: f32,
    pub mag_sigma: f32,
    pub mag_period_us: i64,
    /// Cold-start pose uncertainty (position unknown).
    pub init_pos_sigma: f32,
    pub init_vel_sigma: f32,
    pub init_att_sigma: f32,
    /// IMU gaps longer than this are clamped (a stall must not integrate
    /// one huge step).
    pub max_dt: f32,
    /// Without an IMU the filter is propagated by tick() at this period
    /// against a stationary model, with `no_imu_accel_noise` letting the
    /// position wander so ranges can move it.
    pub no_imu_period_us: i64,
    pub no_imu_accel_noise: f32,
    /// Boot-time static IMU calibration: IMU samples averaged before the
    /// filter starts. If the node is still (gyro spread below
    /// `cal_still_gyro_sd`) the accel scale is set from the gravity magnitude
    /// and the gyro bias from the mean rate — cheap consumer MEMS parts are
    /// off by >10% in scale (one MPU6050 here read 8.4 m/s^2), which both
    /// disables gravity aiding and integrates phantom acceleration.
    pub cal_samples: u32,
    pub cal_still_gyro_sd: f32,
    /// Accepted scale range; outside it the part is broken, not uncalibrated.
    pub cal_scale_min: f32,
    pub cal_scale_max: f32,
    /// Give up (start uncalibrated) after this many moving windows.
    pub cal_max_attempts: u32,
}

impl Default for NodeConfig {
    fn default() -> NodeConfig {
        NodeConfig {
            params: Params::default(),
            // No GPS indoors: a swarm can be entirely anchorless.
            coop: CoopConfig { mean_only_pos_only: true, ..CoopConfig::default() },
            range_sigma: 0.10,
            gravity_every: 5,
            gravity_gate: 0.5,
            gravity_sigma: 0.1,
            zero_vel_sigma: 3.0,
            zero_vel_period_us: 500_000,
            zero_vel_keep_position: false,
            // Slow next to ranging: each widget computes the centroid from
            // its own view of the others, and at 5 s those differences
            // distorted the mesh (hardware: lock ~6 -> ~2 with it off).
            centroid_bleed_tau_s: 60.0,
            centroid_period_us: 100_000,
            still_on: true,
            still_gyro: 0.03,
            still_accel: 0.25,
            still_tau_us: 100_000,
            still_hold_us: 500_000,
            still_sigma: 0.01,
            still_period_us: 50_000,
            planar_on: false,
            planar_z: 0.0,
            planar_sigma: 0.05,
            planar_period_us: 100_000,
            survey_period_us: 100_000,
            survey_sigma: 0.02,
            mag_sigma: 0.05,
            mag_period_us: 50_000,
            init_pos_sigma: 100.0,
            init_vel_sigma: 1.5,
            init_att_sigma: 0.3,
            max_dt: 0.05,
            no_imu_period_us: 10_000,
            no_imu_accel_noise: 0.5,
            cal_samples: 200,
            cal_still_gyro_sd: 0.01,
            cal_scale_min: 0.7,
            cal_scale_max: 1.3,
            cal_max_attempts: 5,
        }
    }
}

/// A peer as last heard: its advertisement and when it arrived.
#[derive(Clone, Copy, Debug)]
pub struct Peer {
    pub advert: Advert,
    pub rx_us: i64,
    /// Ranges to this peer used / gated / skipped, for diagnostics.
    pub ranges_ok: u32,
    pub ranges_gated: u32,
    pub ranges_skipped: u32,
}

impl Peer {
    /// The advertisement dead-reckoned to `t_us` by its velocity.
    pub fn view(&self, t_us: i64) -> PeerView {
        let age = (t_us - self.rx_us) as f32 * 1e-6;
        PeerView {
            p: self.advert.p.add(self.advert.v.scale(age)),
            var: self.advert.quality,
            lock: self.advert.lock,
            age,
        }
    }
}

/// Result of [`Node::range`], also the C ABI code.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(i32)]
pub enum RangeResult {
    Accepted = 1,
    Gated = 0,
    StaleAdvert = -1,
    Anchored = -2,
    ColdPeer = -3,
    /// No advertisement from this peer yet: nothing to range against.
    UnknownPeer = -4,
    /// I'm surveyed: I know where I am.
    Surveyed = -5,
    /// Filter not initialized yet (waiting for the first IMU sample).
    NotReady = -6,
}

/// Result of the boot-time static IMU calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuCal {
    /// Multiplies raw accel: g / |mean accel| while still.
    pub accel_scale: f32,
    /// Initial gyro bias (rad/s), seeded into the estimator.
    pub gyro_bias: Vec3,
    /// False if the node never held still: defaults were used.
    pub still: bool,
}

/// Running sums for the calibration window.
#[derive(Clone, Copy, Default)]
struct CalWindow {
    n: u32,
    attempts: u32,
    sum_a: [f32; 3],
    sum_g: [f32; 3],
    sum_g2: [f32; 3],
}

pub struct Node {
    pub id: u32,
    pub cfg: NodeConfig,
    pub est: Estimator,
    pub peers: [Option<Peer>; MAX_PEERS],
    pub has_imu: bool,
    /// Hard-iron offset and soft-iron matrix (calibrated = raw - hard, then
    /// soft *). None: magnetometer not used (uncalibrated fields poison yaw).
    pub mag_cal: Option<(Vec3, Mat3)>,
    /// World field direction, fixed at init from the first calibrated
    /// sample: x points to magnetic north (horizontal), z up.
    mag_world: Option<Vec3>,
    surveyed: Option<(Vec3, f32)>,
    seq: u32,
    initialized: bool,
    last_t_us: i64,
    imu_count: u32,
    next_zero_vel_us: i64,
    next_centroid_us: i64,
    next_planar_us: i64,
    /// Stationary detector: EWMAs of |gyro - bias| and ||a| - g|, when the
    /// widget last moved, next ZUPT.
    still_gyro_avg: f32,
    still_accel_avg: f32,
    moved_us: i64,
    next_still_us: i64,
    next_survey_us: i64,
    next_mag_us: i64,
    last_mag_body: Option<Vec3>,
    cal: CalWindow,
    imu_cal: Option<ImuCal>,
}

impl Node {
    pub fn new(id: u32, has_imu: bool, cfg: NodeConfig) -> Node {
        let mut params = cfg.params;
        if !has_imu {
            params.accel_noise = cfg.no_imu_accel_noise;
        }
        Node {
            id,
            cfg,
            est: Estimator::new(params),
            peers: [None; MAX_PEERS],
            has_imu,
            mag_cal: None,
            mag_world: None,
            surveyed: None,
            seq: 0,
            initialized: false,
            last_t_us: 0,
            imu_count: 0,
            next_zero_vel_us: 0,
            next_centroid_us: 0,
            next_planar_us: 0,
            still_gyro_avg: 1.0,
            still_accel_avg: 1.0,
            moved_us: 0,
            next_still_us: 0,
            next_survey_us: 0,
            next_mag_us: 0,
            last_mag_body: None,
            cal: CalWindow::default(),
            imu_cal: None,
        }
    }

    /// The boot-time IMU calibration, once the filter has started.
    pub fn imu_cal(&self) -> Option<ImuCal> {
        self.imu_cal
    }

    /// Accumulate one calibration sample; returns the calibration (and the
    /// mean accel to initialize from) when the window completes.
    fn calibrate(&mut self, accel: Vec3, gyro: Vec3) -> Option<(ImuCal, Vec3)> {
        let w = &mut self.cal;
        for (i, (a, g)) in accel.to_array().iter().zip(gyro.to_array()).enumerate() {
            w.sum_a[i] += a;
            w.sum_g[i] += g;
            w.sum_g2[i] += g * g;
        }
        w.n += 1;
        if w.n < self.cfg.cal_samples {
            return None;
        }
        let n = w.n as f32;
        let mean_a = Vec3::new(w.sum_a[0] / n, w.sum_a[1] / n, w.sum_a[2] / n);
        let mean_g = Vec3::new(w.sum_g[0] / n, w.sum_g[1] / n, w.sum_g[2] / n);
        let var = |i: usize, m: f32| (w.sum_g2[i] / n - m * m).max(0.0);
        let gyro_sd = libm::sqrtf(var(0, mean_g.x) + var(1, mean_g.y) + var(2, mean_g.z));
        let scale = GRAVITY / mean_a.norm();
        let still = gyro_sd < self.cfg.cal_still_gyro_sd
            && scale > self.cfg.cal_scale_min
            && scale < self.cfg.cal_scale_max;
        w.attempts += 1;
        if still {
            return Some((ImuCal { accel_scale: scale, gyro_bias: mean_g, still: true }, mean_a));
        }
        if w.attempts >= self.cfg.cal_max_attempts {
            return Some((ImuCal { accel_scale: 1.0, gyro_bias: Vec3::ZERO, still: false }, accel));
        }
        *w = CalWindow { attempts: w.attempts, ..CalWindow::default() };
        None
    }

    pub fn initialized(&self) -> bool {
        self.initialized
    }

    pub fn surveyed(&self) -> Option<Vec3> {
        self.surveyed.map(|(p, _)| p)
    }

    /// Pin this widget to a known, fixed position (m) — it becomes an anchor
    /// for the swarm, advertising near-zero variance.
    pub fn set_surveyed(&mut self, p: Vec3, sigma: f32) {
        self.surveyed = Some((p, sigma));
        let q = self.est.q;
        self.est.set_pose(p, Vec3::ZERO, q, sigma, 0.05, self.cfg.init_att_sigma);
    }

    /// Take a resolved configuration (settings changed). Estimator noise
    /// parameters, cooperative rules and aiding apply from the next update;
    /// survey and magnetometer calibration only re-initialize when they
    /// actually change.
    pub fn apply(&mut self, t: &crate::settings::Tunables) {
        let mut params = t.node.params;
        if !self.has_imu {
            params.accel_noise = t.node.no_imu_accel_noise;
        }
        self.cfg = t.node;
        self.est.params = params;
        if t.survey_on {
            if self.surveyed != Some((t.survey_p, t.survey_sigma)) {
                self.set_surveyed(t.survey_p, t.survey_sigma);
            }
        } else if self.surveyed.is_some() {
            self.clear_surveyed();
        }
        if t.mag_on {
            self.mag_cal = Some((t.mag_hard, t.mag_soft));
        } else {
            self.mag_cal = None;
        }
    }

    pub fn clear_surveyed(&mut self) {
        self.surveyed = None;
        let (p, q) = (self.est.p, self.est.q);
        self.est.set_pose(p, Vec3::ZERO, q, self.cfg.init_pos_sigma, self.cfg.init_vel_sigma,
                          self.cfg.init_att_sigma);
    }

    fn init(&mut self, t_us: i64, accel: Vec3) {
        let q = match (self.mag_cal, self.last_mag_body) {
            (Some(_), Some(m)) => {
                // World field: horizontal north along +x, dip from the
                // measured angle between the field and up.
                let up = accel.normalized();
                let mn = m.normalized();
                let s = mn.dot(up);
                let world = Vec3::new(libm::sqrtf((1.0 - s * s).max(0.0)), 0.0, s);
                self.mag_world = Some(world);
                triad(up, mn, Vec3::new(0.0, 0.0, 1.0), world)
            }
            _ => tilt_only(accel),
        };
        let (p, sigma) = match self.surveyed {
            Some((p, s)) => (p, s),
            None => (Vec3::ZERO, self.cfg.init_pos_sigma),
        };
        let vel_sigma = if self.surveyed.is_some() { 0.05 } else { self.cfg.init_vel_sigma };
        self.est.set_pose(p, Vec3::ZERO, q, sigma, vel_sigma, self.cfg.init_att_sigma);
        self.initialized = true;
        self.last_t_us = t_us;
        self.next_zero_vel_us = t_us + self.cfg.zero_vel_period_us;
        self.next_survey_us = t_us + self.cfg.survey_period_us;
    }

    /// One IMU sample: accel (m/s^2, specific force, body) + gyro (rad/s),
    /// raw from the sensor. The first ~second calibrates (see NodeConfig).
    pub fn imu(&mut self, t_us: i64, accel: Vec3, gyro: Vec3) {
        if !self.initialized {
            if let Some((cal, mean_a)) = self.calibrate(accel, gyro) {
                self.imu_cal = Some(cal);
                self.init(t_us, mean_a.scale(cal.accel_scale));
                self.est.bg = cal.gyro_bias;
            }
            return;
        }
        let accel = accel.scale(self.imu_cal.map_or(1.0, |c| c.accel_scale));
        let mut dt = (t_us - self.last_t_us) as f32 * 1e-6;
        self.last_t_us = t_us;
        if dt <= 0.0 {
            return;
        }
        if dt > self.cfg.max_dt {
            dt = self.cfg.max_dt;
        }
        self.est.predict(accel, gyro, dt);
        self.imu_count = self.imu_count.wrapping_add(1);
        self.stationary(t_us, accel, gyro, dt);
        // Quasi-static gravity tilt aiding.
        if self.imu_count % self.cfg.gravity_every == 0
            && libm::fabsf(accel.norm() - GRAVITY) < self.cfg.gravity_gate
        {
            self.est.update_gravity(accel, self.cfg.gravity_sigma);
        }
        self.periodic(t_us);
    }

    /// Is the widget still (see NodeConfig::still_on)?
    pub fn is_still(&self, t_us: i64) -> bool {
        self.cfg.still_on && self.initialized && t_us - self.moved_us >= self.cfg.still_hold_us
    }

    fn stationary(&mut self, t_us: i64, accel: Vec3, gyro: Vec3, dt: f32) {
        let a = (dt / (self.cfg.still_tau_us as f32 * 1e-6)).min(1.0);
        let g = gyro.sub(self.est.bg).norm();
        let f = libm::fabsf(accel.norm() - GRAVITY);
        self.still_gyro_avg += a * (g - self.still_gyro_avg);
        self.still_accel_avg += a * (f - self.still_accel_avg);
        if self.still_gyro_avg > self.cfg.still_gyro || self.still_accel_avg > self.cfg.still_accel {
            self.moved_us = t_us;
        }
        if self.is_still(t_us) && t_us >= self.next_still_us {
            self.next_still_us = t_us + self.cfg.still_period_us;
            self.est.update_zero_vel_masked(self.cfg.still_sigma, true);
        }
    }

    /// Without an IMU: propagate against a stationary model. Call often
    /// (every few ms); it steps at `no_imu_period_us`.
    pub fn tick(&mut self, t_us: i64) {
        if self.has_imu {
            return;
        }
        let rest = Vec3::new(0.0, 0.0, GRAVITY);
        if !self.initialized {
            self.init(t_us, rest);
            return;
        }
        while t_us - self.last_t_us >= self.cfg.no_imu_period_us {
            let dt = self.cfg.no_imu_period_us as f32 * 1e-6;
            self.last_t_us += self.cfg.no_imu_period_us;
            self.est.predict(rest, Vec3::ZERO, dt);
            self.periodic(self.last_t_us);
        }
    }

    fn periodic(&mut self, t_us: i64) {
        if t_us >= self.next_zero_vel_us && self.cfg.zero_vel_sigma > 0.0 {
            self.next_zero_vel_us = t_us + self.cfg.zero_vel_period_us;
            self.est.update_zero_vel_masked(self.cfg.zero_vel_sigma, self.cfg.zero_vel_keep_position);
        }
        if t_us >= self.next_centroid_us && self.cfg.centroid_bleed_tau_s > 0.0 {
            self.next_centroid_us = t_us + self.cfg.centroid_period_us;
            if let Some(mut c) = self.anchorless_centroid(t_us) {
                if self.cfg.planar_on {
                    c.z = 0.0; // the plane owns z
                }
                let k = self.cfg.centroid_period_us as f32 * 1e-6 / self.cfg.centroid_bleed_tau_s;
                self.est.shift_position(c.scale(-k.min(1.0)));
            }
        }
        if self.cfg.planar_on && self.surveyed.is_none() && t_us >= self.next_planar_us {
            self.next_planar_us = t_us + self.cfg.planar_period_us;
            self.est.update_alt(self.cfg.planar_z, self.cfg.planar_sigma);
        }
        if let Some((p, sigma)) = self.surveyed {
            if t_us >= self.next_survey_us {
                self.next_survey_us = t_us + self.cfg.survey_period_us;
                self.est.update_gps(p, sigma, sigma);
            }
        }
    }

    /// Centroid of my position and every fresh peer's (dead-reckoned)
    /// advertised one, or None if I or any of them has an absolute source:
    /// then the frame is pinned and must not be re-centered.
    fn anchorless_centroid(&self, t_us: i64) -> Option<Vec3> {
        if self.surveyed.is_some() {
            return None;
        }
        let mut sum = self.est.p;
        let mut n = 1.0;
        for peer in self.peers.iter().flatten() {
            let view = peer.view(t_us);
            if view.age > self.cfg.coop.max_advert_age {
                continue;
            }
            if peer.advert.flags & (FLAG_ANCHORED | FLAG_GPS) != 0 {
                return None;
            }
            sum = sum.add(view.p);
            n += 1.0;
        }
        Some(sum.scale(1.0 / n))
    }

    /// Raw magnetometer sample (sensor units, body frame). Ignored until a
    /// calibration is set; used for heading at `mag_period_us`.
    pub fn mag(&mut self, t_us: i64, raw: Vec3) {
        let Some((hard, soft)) = self.mag_cal else { return };
        let m = mat_mul_vec(&soft, raw.sub(hard));
        self.last_mag_body = Some(m);
        let Some(world) = self.mag_world else { return };
        if self.initialized && t_us >= self.next_mag_us {
            self.next_mag_us = t_us + self.cfg.mag_period_us;
            self.est.update_mag(m, world, self.cfg.mag_sigma);
        }
    }

    pub fn set_mag_calibration(&mut self, hard: Vec3, soft: Mat3) {
        self.mag_cal = Some((hard, soft));
    }

    fn peer_index(&self, id: u32) -> Option<usize> {
        self.peers.iter().position(|p| matches!(p, Some(p) if p.advert.widget_id == id))
    }

    /// A peer's advertisement arrived (decoded). Returns false if the table
    /// is full of fresher peers.
    pub fn heard(&mut self, t_us: i64, a: Advert) -> bool {
        if a.widget_id == self.id {
            return false;
        }
        let slot = match self.peer_index(a.widget_id) {
            Some(i) => i,
            None => match self.free_slot(t_us) {
                Some(i) => i,
                None => return false,
            },
        };
        let (ok, gated, skipped) = match self.peers[slot] {
            Some(p) if p.advert.widget_id == a.widget_id => {
                (p.ranges_ok, p.ranges_gated, p.ranges_skipped)
            }
            _ => (0, 0, 0),
        };
        self.peers[slot] = Some(Peer {
            advert: a,
            rx_us: t_us,
            ranges_ok: ok,
            ranges_gated: gated,
            ranges_skipped: skipped,
        });
        true
    }

    /// An empty slot, else the stalest expired one.
    fn free_slot(&mut self, t_us: i64) -> Option<usize> {
        if let Some(i) = self.peers.iter().position(|p| p.is_none()) {
            return Some(i);
        }
        let (i, oldest) = self
            .peers
            .iter()
            .enumerate()
            .filter_map(|(i, p)| p.map(|p| (i, p.rx_us)))
            .min_by_key(|&(_, rx)| rx)?;
        if t_us - oldest > PEER_EXPIRY_US {
            Some(i)
        } else {
            None
        }
    }

    /// Decode and record an advertisement off the air. Returns the sender id.
    pub fn advert_rx(&mut self, t_us: i64, bytes: &[u8]) -> Option<u32> {
        let a = advert::decode(bytes)?;
        let id = a.widget_id;
        self.heard(t_us, a).then_some(id)
    }

    /// A UWB range (m) to `peer_id`, measured at `t_us`.
    pub fn range(&mut self, t_us: i64, peer_id: u32, range_m: f32) -> RangeResult {
        if !self.initialized {
            return RangeResult::NotReady;
        }
        if self.surveyed.is_some() {
            return RangeResult::Surveyed;
        }
        let Some(i) = self.peer_index(peer_id) else { return RangeResult::UnknownPeer };
        let mut peer = self.peers[i].unwrap();
        let view = peer.view(t_us);
        let (outcome, _decision): (Outcome, Decision) = coop::range_update(
            &self.cfg.coop,
            &mut self.est,
            &view,
            range_m,
            self.cfg.range_sigma,
            false, // no GPS on these widgets
        );
        let r = match outcome {
            Outcome::Applied { accepted: true, .. } => RangeResult::Accepted,
            Outcome::Applied { accepted: false, .. } => RangeResult::Gated,
            Outcome::Skipped(SkipReason::StaleAdvert) => RangeResult::StaleAdvert,
            Outcome::Skipped(SkipReason::Anchored) => RangeResult::Anchored,
            Outcome::Skipped(SkipReason::ColdPeer) => RangeResult::ColdPeer,
        };
        match r {
            RangeResult::Accepted => peer.ranges_ok += 1,
            RangeResult::Gated => peer.ranges_gated += 1,
            _ => peer.ranges_skipped += 1,
        }
        self.peers[i] = Some(peer);
        r
    }

    /// My advertisement as of now. Surveyed widgets advertise their known
    /// position as an absolute source (var ~0, locked).
    pub fn advert(&mut self) -> Advert {
        self.seq = self.seq.wrapping_add(1);
        match self.surveyed {
            Some((p, _)) => Advert {
                widget_id: self.id,
                seq: self.seq,
                p,
                v: Vec3::ZERO,
                quality: 1e-4,
                flags: FLAG_ANCHORED | FLAG_STATIC,
                lock: 1.0,
            },
            None => Advert {
                widget_id: self.id,
                seq: self.seq,
                p: self.est.p,
                v: self.est.v,
                quality: self.est.pos_var(),
                flags: 0,
                lock: self.est.range_innov_ratio(),
            },
        }
    }

    /// Encode my advertisement (widget.v1.PoseAdvertisement) into `out`.
    pub fn advert_tx(&mut self, out: &mut [u8]) -> Option<usize> {
        let a = self.advert();
        advert::encode(&a, out)
    }

    /// Peers heard within PEER_EXPIRY_US of `t_us`.
    pub fn live_peers(&self, t_us: i64) -> impl Iterator<Item = &Peer> {
        self.peers
            .iter()
            .filter_map(|p| p.as_ref())
            .filter(move |p| t_us - p.rx_us <= PEER_EXPIRY_US)
    }
}

fn mat_mul_vec(m: &Mat3, v: Vec3) -> Vec3 {
    swarm_core::math::mat3_mul_vec(m, v)
}

/// Body-to-world attitude from two direction pairs (TRIAD): `up_body` /
/// `mag_body` measured, `up_world` / `mag_world` reference.
pub fn triad(up_body: Vec3, mag_body: Vec3, up_world: Vec3, mag_world: Vec3) -> Quat {
    let frame = |primary: Vec3, secondary: Vec3| -> Mat3 {
        let t1 = primary.normalized();
        let t2 = t1.cross(secondary).normalized();
        let t3 = t1.cross(t2);
        // Columns t1, t2, t3.
        [[t1.x, t2.x, t3.x], [t1.y, t2.y, t3.y], [t1.z, t2.z, t3.z]]
    };
    let body = frame(up_body, mag_body);
    let reference = frame(up_world, mag_world);
    let r = swarm_core::math::mat3_mul(&reference, &swarm_core::math::mat3_transpose(&body));
    quat_from_mat(&r)
}

/// Attitude with zero yaw whose body "up" (measured specific force) maps to
/// world +z: the TRIAD fallback when there's no magnetometer.
pub fn tilt_only(accel: Vec3) -> Quat {
    let up = accel.normalized();
    // Any secondary not parallel to up; prefer body +x for a stable yaw.
    let secondary = if libm::fabsf(up.x) < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    // Reference secondary: the same axis's horizontal projection (yaw 0).
    triad(up, secondary, Vec3::new(0.0, 0.0, 1.0), secondary)
}

/// Scalar-first quaternion from a rotation matrix (Shepperd's method).
pub fn quat_from_mat(r: &Mat3) -> Quat {
    let tr = r[0][0] + r[1][1] + r[2][2];
    let q = if tr > 0.0 {
        let s = libm::sqrtf(tr + 1.0) * 2.0;
        Quat { w: 0.25 * s, x: (r[2][1] - r[1][2]) / s, y: (r[0][2] - r[2][0]) / s, z: (r[1][0] - r[0][1]) / s }
    } else if r[0][0] >= r[1][1] && r[0][0] >= r[2][2] {
        let s = libm::sqrtf(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
        Quat { w: (r[2][1] - r[1][2]) / s, x: 0.25 * s, y: (r[0][1] + r[1][0]) / s, z: (r[0][2] + r[2][0]) / s }
    } else if r[1][1] >= r[2][2] {
        let s = libm::sqrtf(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
        Quat { w: (r[0][2] - r[2][0]) / s, x: (r[0][1] + r[1][0]) / s, y: 0.25 * s, z: (r[1][2] + r[2][1]) / s }
    } else {
        let s = libm::sqrtf(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
        Quat { w: (r[1][0] - r[0][1]) / s, x: (r[0][2] + r[2][0]) / s, y: (r[1][2] + r[2][1]) / s, z: 0.25 * s }
    };
    q.normalized()
}

/// Identity soft-iron matrix, for hard-iron-only calibrations.
pub const SOFT_IRON_IDENTITY: Mat3 = MAT3_IDENTITY;

#[cfg(test)]
mod tests {
    use super::*;

    const S: i64 = 1_000_000;

    fn locked_peer(id: u32, p: Vec3) -> Advert {
        Advert { widget_id: id, seq: 1, p, v: Vec3::ZERO, quality: 1e-4, flags: FLAG_ANCHORED, lock: 1.0 }
    }

    /// Three surveyed peers + noise-free ranges at 25 Hz each: a stationary
    /// IMU-less node converges onto its true position.
    #[test]
    fn converges_on_surveyed_peers_without_imu() {
        let truth = Vec3::new(2.0, 3.0, 1.0);
        let anchors = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(6.0, 0.0, 0.5), Vec3::new(0.0, 6.0, 2.0),
                       Vec3::new(6.0, 6.0, 0.0)];
        let mut n = Node::new(7, false, NodeConfig::default());
        let mut t = 0;
        n.tick(t);
        for step in 0..1000 {
            t += 10_000;
            n.tick(t);
            for (k, a) in anchors.iter().enumerate() {
                n.heard(t, locked_peer(100 + k as u32, *a));
                if step % 4 == k {
                    n.range(t, 100 + k as u32, truth.sub(*a).norm());
                }
            }
        }
        let err = n.est.p.sub(truth).norm();
        assert!(err < 0.05, "err {} m at {:?}", err, n.est.p);
        assert!(n.est.pos_var() < 0.05);
    }

    /// Four anchorless widgets, all displaced the same way: the bleed brings
    /// the swarm centroid to the origin and leaves the shape alone. One
    /// surveyed peer in range turns it off.
    #[test]
    fn anchorless_swarm_centroid_bleeds_to_origin() {
        let offset = Vec3::new(30.0, -10.0, 50.0);
        let shape = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), Vec3::new(0.0, 3.0, 0.0),
                     Vec3::new(4.0, 3.0, 1.0)];
        let mut cfg = NodeConfig::default();
        cfg.centroid_bleed_tau_s = 5.0;
        let mut nodes: Vec<Node> = (0..4).map(|i| Node::new(1 + i, false, cfg)).collect();
        for (n, s) in nodes.iter_mut().zip(shape) {
            n.tick(0);
            n.est.p = s.add(offset);
        }
        let mut t = 0;
        for _ in 0..4000 {
            t += 10_000;
            let adverts: Vec<Advert> = nodes.iter_mut().map(|n| n.advert()).collect();
            for n in nodes.iter_mut() {
                for a in &adverts {
                    n.heard(t, *a);
                }
                n.tick(t);
            }
        }
        let centroid = nodes.iter().fold(Vec3::ZERO, |c, n| c.add(n.est.p)).scale(0.25);
        assert!(centroid.norm() < 0.1, "centroid {:?}", centroid);
        let d = nodes[1].est.p.sub(nodes[0].est.p);
        assert!(d.sub(shape[1]).norm() < 0.01, "shape changed: {:?}", d);

        let mut n = Node::new(9, false, NodeConfig::default());
        n.tick(0);
        n.est.p = offset;
        n.heard(0, locked_peer(5, Vec3::ZERO));
        for i in 1..50 {
            n.heard(i * 10_000, locked_peer(5, Vec3::ZERO));
            n.tick(i * 10_000);
        }
        assert!(n.est.p.sub(offset).norm() < 1.0, "re-centered next to an anchor");
    }

    /// Anchorless mesh, a node with a residual vertical accel error (a
    /// carried MPU6050 showed v_z ramping ~0.07 m/s^2). With position-only
    /// ranging, a full zero-velocity update re-applies position corrections
    /// ranging already made through P's position-velocity cross-covariance:
    /// the position jumps every zero-velocity period (the zigzag seen on
    /// hardware). Keeping position out of that update removes the jumps.
    #[test]
    fn zero_velocity_update_position_trade_off() {
        /// Largest position step at a zero-velocity update, and largest
        /// position error, over the second half of a 60 s run.
        fn run(keep_position: bool) -> (f32, f32) {
            let mut cfg = NodeConfig::default();
            cfg.zero_vel_keep_position = keep_position;
            // The carried-node case: stationary detection would pin this
            // (motionless, synthetic) widget and hide the trade-off.
            cfg.still_on = false;
            let mut n = Node::new(1, true, cfg);
            let still = Vec3::new(0.0, 0.0, GRAVITY);
            let mut t = 0;
            for _ in 0..400 {
                t += 5_000;
                n.imu(t, still, Vec3::ZERO);
            }
            let peers = [Vec3::new(3.0, 0.0, 0.5), Vec3::new(-1.0, 2.5, 0.0), Vec3::new(-1.0, -2.5, -0.5)];
            let biased = Vec3::new(0.0, 0.0, GRAVITY + 0.07);
            let (mut max_jump, mut max_err) = (0.0f32, 0.0f32);
            for step in 0..12_000 {
                t += 5_000;
                for (k, p) in peers.iter().enumerate() {
                    // Comparable peers: as unanchored as me, well locked.
                    let a = Advert { widget_id: 10 + k as u32, seq: step, p: *p, v: Vec3::ZERO,
                                     quality: n.est.pos_var(), flags: 0, lock: 1.0 };
                    n.heard(t, a);
                }
                // ~33 exchanges/s, round-robin over the peers.
                if step % 6 == 0 {
                    let k = (step / 6) as usize % 3;
                    n.range(t, 10 + k as u32, peers[k].norm());
                }
                let before = n.est.p;
                n.imu(t, biased, Vec3::ZERO);
                if step > 6_000 {
                    // IMU propagation alone moves < 1 cm per 5 ms step.
                    max_jump = max_jump.max(n.est.p.sub(before).norm());
                    max_err = max_err.max(n.est.p.norm());
                }
            }
            (max_jump, max_err)
        }
        // Measured: full (0.22 m steps, 0.33 m drift), kept (0.007, 1.47).
        let (jump_full, err_full) = run(false);
        let (jump_kept, err_kept) = run(true);
        assert!(jump_kept < 0.02, "position steps {} m with position kept", jump_kept);
        assert!(jump_full > 5.0 * jump_kept, "full update should jump: {} vs {}", jump_full, jump_kept);
        // The trade-off: without the jumps, nothing trims velocity-driven
        // position drift. Pinned so a tuning change shows up here.
        assert!(err_full < 0.5 && err_kept < 2.0, "drift {} / {} m", err_full, err_kept);
    }

    /// Planar mode pulls an out-of-plane widget back to the plane's height
    /// and leaves x/y alone.
    #[test]
    fn planar_mode_holds_height() {
        let mut cfg = NodeConfig::default();
        cfg.planar_on = true;
        cfg.planar_z = 0.5;
        cfg.centroid_bleed_tau_s = 0.0; // a lone node would bleed to (0, 0)
        let mut n = Node::new(1, false, cfg);
        n.tick(0);
        n.est.p = Vec3::new(2.0, -1.0, -0.8);
        for i in 1..1000 {
            n.tick(i * 10_000);
        }
        assert!((n.est.p.z - 0.5).abs() < 0.05, "z {}", n.est.p.z);
        assert!((n.est.p.x - 2.0).abs() < 0.05 && (n.est.p.y + 1.0).abs() < 0.05, "{:?}", n.est.p);
    }

    /// A still widget with a residual accel bias: stationary detection pins
    /// velocity (and learns the bias) so it doesn't drift; without it the
    /// same widget drifts metres. Motion turns it off.
    #[test]
    fn stationary_detection_pins_velocity() {
        fn run(still_on: bool) -> (f32, bool) {
            let mut cfg = NodeConfig::default();
            cfg.still_on = still_on;
            cfg.centroid_bleed_tau_s = 0.0;
            let mut n = Node::new(1, true, cfg);
            let mut t = 0;
            for _ in 0..400 {
                t += 5_000;
                n.imu(t, Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO);
            }
            let p0 = n.est.p;
            let biased = Vec3::new(0.03, -0.02, GRAVITY + 0.04);
            let mut rng = 1u32;
            for _ in 0..6000 {
                t += 5_000;
                rng ^= rng << 13;
                rng ^= rng >> 17;
                rng ^= rng << 5;
                let noise = (rng % 1000) as f32 / 1000.0 - 0.5;
                n.imu(t, biased.add(Vec3::new(0.0, 0.0, 0.02 * noise)), Vec3::new(0.001 * noise, 0.0, 0.0));
            }
            (n.est.p.sub(p0).norm(), n.is_still(t))
        }
        let (drift_on, still) = run(true);
        let (drift_off, _) = run(false);
        assert!(still, "a still widget is detected as still");
        assert!(drift_on < 0.1, "drifted {} m with stationary detection", drift_on);
        assert!(drift_off > 5.0 * drift_on, "{} vs {}", drift_off, drift_on);

        // Shaking it: not still.
        let mut n = Node::new(1, true, NodeConfig::default());
        let mut t = 0;
        for _ in 0..400 {
            t += 5_000;
            n.imu(t, Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO);
        }
        for k in 0..400 {
            t += 5_000;
            let s = if k % 20 < 10 { 2.0 } else { -2.0 };
            n.imu(t, Vec3::new(s, 0.0, GRAVITY), Vec3::new(0.0, 0.0, 0.3 * s));
        }
        assert!(!n.is_still(t));
    }

    #[test]
    fn stale_and_unknown_peers_are_skipped() {
        let mut n = Node::new(1, false, NodeConfig::default());
        n.tick(0);
        assert_eq!(n.range(10, 2, 3.0), RangeResult::UnknownPeer);
        n.heard(0, locked_peer(2, Vec3::new(3.0, 0.0, 0.0)));
        assert_eq!(n.range(2 * S, 2, 3.0), RangeResult::StaleAdvert);
        // Own adverts are ignored.
        assert!(!n.heard(0, locked_peer(1, Vec3::ZERO)));
    }

    #[test]
    fn surveyed_node_advertises_as_anchor() {
        let mut n = Node::new(3, true, NodeConfig::default());
        n.set_surveyed(Vec3::new(1.0, 2.0, 0.5), 0.02);
        n.imu(0, Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO);
        for i in 1..400 {
            n.imu(i * 5_000, Vec3::new(0.0, 0.0, GRAVITY), Vec3::ZERO);
        }
        let a = n.advert();
        assert_eq!(a.flags & FLAG_ANCHORED, FLAG_ANCHORED);
        assert!(a.p.sub(Vec3::new(1.0, 2.0, 0.5)).norm() < 1e-6);
        assert!(n.est.p.sub(Vec3::new(1.0, 2.0, 0.5)).norm() < 0.01);
        assert_eq!(n.range(S, 9, 1.0), RangeResult::Surveyed);
    }

    #[test]
    fn peer_table_expires_and_reuses_slots() {
        let mut n = Node::new(1, false, NodeConfig::default());
        for k in 0..MAX_PEERS as u32 {
            assert!(n.heard(0, locked_peer(10 + k, Vec3::ZERO)));
        }
        assert!(!n.heard(1, locked_peer(99, Vec3::ZERO)), "table full of fresh peers");
        assert!(n.heard(PEER_EXPIRY_US + 1, locked_peer(99, Vec3::ZERO)));
        assert_eq!(n.live_peers(PEER_EXPIRY_US + 1).count(), 1);
    }

    /// A still node with a 14% low accel scale and a gyro bias: boot
    /// calibration fixes the scale, seeds the bias, and the filter then
    /// holds still instead of "falling" (the bug seen on a real MPU6050).
    #[test]
    fn boot_calibration_fixes_accel_scale_and_gyro_bias() {
        let mut n = Node::new(1, true, NodeConfig::default());
        let raw_a = Vec3::new(-0.08, 0.1, 8.4); // |a| ~ 8.4 m/s^2
        let bias = Vec3::new(0.0285, 0.0342, -0.0175);
        let mut t = 0;
        for k in 0..2000 {
            t += 5_000;
            // A little sensor noise so the stillness test sees real data.
            let jitter = if k % 2 == 0 { 0.0005 } else { -0.0005 };
            n.imu(t, raw_a, bias.add(Vec3::new(jitter, -jitter, jitter)));
        }
        let cal = n.imu_cal().expect("calibrated");
        assert!(cal.still);
        assert!(libm::fabsf(cal.accel_scale * raw_a.norm() - GRAVITY) < 1e-3, "{:?}", cal);
        assert!(cal.gyro_bias.sub(bias).norm() < 1e-3);
        // 10 s still: no phantom fall, attitude level-ish, gyro bias held.
        assert!(n.est.v.norm() < 0.2, "v {:?}", n.est.v);
        assert!(n.est.bg.sub(bias).norm() < 0.005, "bg {:?}", n.est.bg);
    }

    #[test]
    fn calibration_gives_up_when_never_still() {
        let mut n = Node::new(1, true, NodeConfig::default());
        let mut t = 0;
        for k in 0..2000 {
            t += 5_000;
            let spin = if (k / 10) % 2 == 0 { 0.5 } else { -0.5 };
            n.imu(t, Vec3::new(0.0, 0.0, GRAVITY), Vec3::new(spin, 0.0, 0.0));
        }
        let cal = n.imu_cal().expect("started anyway");
        assert!(!cal.still);
        assert_eq!(cal.accel_scale, 1.0);
        assert!(n.initialized());
    }

    #[test]
    fn triad_recovers_yaw() {
        // Body rotated 0.5 rad about z: world vectors seen rotated by -0.5.
        let q_true = Quat::from_rotvec(Vec3::new(0.0, 0.0, 0.5));
        let up_w = Vec3::new(0.0, 0.0, 1.0);
        let mag_w = Vec3::new(0.8, 0.0, -0.6);
        let q = triad(q_true.rotate_inv(up_w), q_true.rotate_inv(mag_w), up_w, mag_w);
        assert!(libm::fabsf(q.yaw() - 0.5) < 1e-4, "yaw {}", q.yaw());
    }

    #[test]
    fn tilt_only_levels_a_tilted_body() {
        let q_true = Quat::from_rotvec(Vec3::new(0.2, -0.1, 0.0));
        let accel = q_true.rotate_inv(Vec3::new(0.0, 0.0, GRAVITY));
        let q = tilt_only(accel);
        let up = q.rotate(accel.normalized());
        assert!(up.sub(Vec3::new(0.0, 0.0, 1.0)).norm() < 1e-4, "{:?}", up);
    }
}
