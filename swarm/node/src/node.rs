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
    /// Blocked-link (NLOS) weighting. The DW3000 reports the first path's
    /// power next to the total received power; with a clear line of sight
    /// they're within a few dB, and when the direct path is blocked (a
    /// body, a wall) the first path is weak and the range reads long. The
    /// bench (captures/swarm_v23.jsonl): gap < 10 dB, 0.5% of ranges read
    /// > 0.3 m long; above it 15-30% do, by up to ~2 m. A range's sigma
    /// grows by `nlos_sigma_per_db` per dB of gap above `nlos_gap_db`,
    /// capped at `nlos_sigma_max` (added in quadrature to range_sigma).
    /// Ranges with a gap above `nlos_reject_db` aren't used (0 = never).
    /// A flagged range's error is a precise core plus a one-sided long tail
    /// (sim: inflating sigma let the tail in past the gate and did worse
    /// than nothing), so flagged ranges can also take a tighter innovation
    /// gate, `nlos_gate` sigmas (0 = params.gate).
    pub nlos_gap_db: f32,
    pub nlos_sigma_per_db: f32,
    pub nlos_sigma_max: f32,
    pub nlos_reject_db: f32,
    pub nlos_gate: f32,
    /// Huber-style clip of a flagged range's excess: once I'm past cold
    /// start (lock < coop.lock_ratio_none), a flagged range reading more
    /// than this (m) longer than predicted is used as if it read only this
    /// much longer. It keeps the core's information without the tail's
    /// pull (the filter averages, and a link reading +6 cm usually but
    /// sometimes +1 m averages ~+25 cm). 0 = off. `nlos_clip_still_only`:
    /// only while I'm still — moving, my estimate lags and a long read is
    /// as likely my error as the link's (sim: clipping movers cost more
    /// than it saved).
    pub nlos_clip_m: f32,
    pub nlos_clip_still_only: bool,
    /// The sigma growth applies only to flagged ranges reading longer than
    /// predicted: a blocked path can only add length, so a flagged range
    /// reading short is as good as a clear one.
    pub nlos_asymmetric: bool,
    /// Online range-bias calibration (the estimator's rb state, prior
    /// params.init_range_bias_sigma): a widget learns its own share of every
    /// range's constant offset, and advertises it for peers to subtract.
    /// Free widgets learn it in full range updates; an anchored one, which
    /// doesn't range-update its position, learns it from ranges to moving,
    /// tracking (lock < `bias_learn_lock`) peers, trusting their position to
    /// `bias_learn_nvar_scale` x (their lock x range_sigma)^2 — moving
    /// widgets sweep geometry, which is what separates bias from position.
    /// The anchor point was solved with the old bias: past `reanchor_bias_m`
    /// of change it lets go (sigma auto_anchor_sigma) to re-solve and
    /// re-anchors once calm again. 0 = never.
    /// `bias_learn_still`: a free widget also learns rb while still. Still
    /// against still peers it has one geometry and as many unknowns (x, y,
    /// rb) as ranges, so rb soaks up whatever the peers' solved positions
    /// are off by (bench: -0.21 m within a minute) — it acts as a per-node
    /// consistency term more than an antenna-delay estimate. That still
    /// helps: sim, learning only while moving, walker 19.1 -> 26.6 cm.
    pub bias_learn_still: bool,
    /// Learn rb from flagged (probably blocked) ranges too. Off: they read
    /// long, and rb learned the excess (bench walk: still widgets' rb went
    /// +0.2..+0.46 m, re-anchoring shrank the mesh, one moved 1.1 m).
    pub bias_learn_flagged: bool,
    pub bias_learn_anchored: bool,
    pub bias_learn_lock: f32,
    pub bias_learn_nvar_scale: f32,
    pub reanchor_bias_m: f32,
    /// Gravity tilt aiding on every Nth IMU sample, when | |a| - g | < gate.
    pub gravity_every: u32,
    pub gravity_gate: f32,
    pub gravity_sigma: f32,
    /// Gravity tilt aiding only while still (stationary detection): carried,
    /// the specific force swings with every step even when its magnitude is
    /// near g, and those "tilt fixes" leak gravity into velocity.
    pub gravity_still_only: bool,
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
    /// Auto-anchoring: a widget that has been still, well locked
    /// (range-innovation ratio < `auto_anchor_lock`) and in a mesh of at
    /// least `auto_anchor_min_peers` fresh peers for `auto_anchor_hold_us`
    /// pins its position and advertises itself as an anchor with sigma
    /// `auto_anchor_sigma`. Without anchors every widget's variance stays
    /// huge (mean-only updates), so a moving widget never gets full
    /// (downhill) updates and lags then snaps; against anchors it does,
    /// velocity included. Moving releases the anchor at once, keeping the
    /// position with sigma `auto_anchor_release_sigma`.
    /// Divergence recovery: when `regain_gated_frac` of the last this-many
    /// (<= 32) ranges that reached the filter were gated, it is confidently
    /// wrong (the
    /// innovation gate rejects nearly every range and it dead-reckons
    /// away). Reset the position uncertainty to `init_pos_sigma` and
    /// re-acquire, as at boot. 0 = off. (Consecutive gated ranges missed
    /// it: a peer along the direction the error doesn't show in kept
    /// being accepted, resetting the count — sim, a carried widget 2.7 m
    /// off for 20 s.)
    pub regain_after_gated: u32,
    /// (~5% of ranges are gated in normal running; sim: a carried widget
    /// with one clear link and two blocked ones sat at ~60% while 1-2 m
    /// off.)
    pub regain_gated_frac: f32,
    pub auto_anchor_on: bool,
    pub auto_anchor_lock: f32,
    pub auto_anchor_hold_us: i64,
    pub auto_anchor_sigma: f32,
    pub auto_anchor_min_peers: u32,
    pub auto_anchor_release_sigma: f32,
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
            // Antenna delays are uncalibrated out of the box: a few cm to
            // ~0.1 m per device.
            // Range bias off: on the bench the raw ranges already match a
            // tape to ~1.5 cm, and rb absorbed geometry error instead (a
            // 0.46 m pair solved 0.27 m apart, +0.19 m of rb between them).
            // 0.1 turns it back on.
            params: Params {
                init_range_bias_sigma: 0.0,
                range_bias_rw: 1e-4,
                range_bias_max: 0.15,
                ..Params::default()
            },
            // No GPS indoors: a swarm can be entirely anchorless.
            coop: CoopConfig { mean_only_pos_only: true, ..CoopConfig::default() },
            range_sigma: 0.10,
            nlos_gap_db: 10.0,
            nlos_sigma_per_db: 0.05,
            nlos_sigma_max: 0.3,
            nlos_reject_db: 0.0,
            nlos_gate: 0.0,
            nlos_clip_m: 0.05,
            nlos_clip_still_only: true,
            nlos_asymmetric: true,
            bias_learn_still: true,
            bias_learn_flagged: false,
            bias_learn_anchored: true,
            bias_learn_lock: 2.0,
            bias_learn_nvar_scale: 4.0,
            reanchor_bias_m: 0.03,
            gravity_every: 5,
            gravity_gate: 0.5,
            gravity_sigma: 0.1,
            gravity_still_only: false,
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
            // ~0.5 s of a widget's ranges at the bench rate.
            regain_after_gated: 16,
            regain_gated_frac: 0.5,
            auto_anchor_on: true,
            // Tuned in the sim's room scenario (per-device range bias, carried
            // walker): at 0.6 a still mesh never anchors (lock sits ~0.6-0.8
            // with uncalibrated antenna delays); at 2.0 the walker's aligned
            // error halves (22.8 -> 10.7 cm over 5 seeds) and jitter drops 3x.
            auto_anchor_lock: 2.0,
            auto_anchor_hold_us: 10_000_000,
            auto_anchor_sigma: 0.05,
            auto_anchor_min_peers: 2,
            auto_anchor_release_sigma: 0.3,
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
            anchored: self.advert.flags & FLAG_ANCHORED != 0,
            still: self.advert.flags & FLAG_STATIC != 0,
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
    /// I'm still and the peer is moving (coop::SkipReason::MovingPeer).
    MovingPeer = -7,
    /// The radio flagged the link as blocked past nlos_reject_db.
    Blocked = -8,
    /// I'm anchored: used to calibrate my range bias only.
    Calibrated = 2,
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
    /// Auto-anchored here (see NodeConfig::auto_anchor_on), and the range
    /// bias it was solved with.
    auto_anchor: Option<Vec3>,
    anchor_rb: f32,
    /// Since when the auto-anchor conditions have held.
    calm_since_us: Option<i64>,
    /// Recent ranges' outcomes (bit set = gated; newest in bit 0, `gated_n`
    /// of them valid), and how often we re-acquired.
    gated_bits: u32,
    gated_n: u32,
    pub regains: u32,
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
            auto_anchor: None,
            anchor_rb: 0.0,
            calm_since_us: None,
            gated_bits: 0,
            gated_n: 0,
            regains: 0,
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
        if params.init_range_bias_sigma <= 0.0 {
            self.est.clear_range_bias();
        }
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
        if self.cfg.gravity_every > 0
            && self.imu_count % self.cfg.gravity_every == 0
            && libm::fabsf(accel.norm() - GRAVITY) < self.cfg.gravity_gate
            && (!self.cfg.gravity_still_only || self.is_still(t_us))
        {
            self.est.update_gravity(accel, self.cfg.gravity_sigma);
        }
        self.periodic(t_us);
    }

    /// Where this widget is pinned, if anywhere: surveyed, or auto-anchored.
    pub fn anchor(&self) -> Option<(Vec3, f32)> {
        self.surveyed.or(self.auto_anchor.map(|p| (p, self.cfg.auto_anchor_sigma)))
    }

    pub fn auto_anchored(&self) -> bool {
        self.auto_anchor.is_some()
    }

    fn auto_anchor_step(&mut self, t_us: i64) {
        if !self.cfg.auto_anchor_on || self.surveyed.is_some() {
            self.auto_anchor = None;
            self.calm_since_us = None;
            return;
        }
        let still = self.is_still(t_us);
        if self.auto_anchor.is_some() {
            let drifted = self.cfg.reanchor_bias_m > 0.0
                && libm::fabsf(self.est.rb - self.anchor_rb) > self.cfg.reanchor_bias_m;
            if still && drifted {
                // The bias calibration moved: re-solve the position with it
                // (still, so velocity stays pinned), then re-anchor.
                self.auto_anchor = None;
                self.calm_since_us = None;
                let (p, q) = (self.est.p, self.est.q);
                self.est.set_pose(p, Vec3::ZERO, q, self.cfg.auto_anchor_sigma, 0.05,
                                  self.cfg.init_att_sigma);
                return;
            }
            if !still {
                // Moving: back to a normal widget, keeping where it was.
                self.auto_anchor = None;
                self.calm_since_us = None;
                let (p, q) = (self.est.p, self.est.q);
                self.est.set_pose(p, Vec3::ZERO, q, self.cfg.auto_anchor_release_sigma, 0.1,
                                  self.cfg.init_att_sigma);
            }
            return;
        }
        let max_age = self.cfg.coop.max_advert_age;
        let peers = self.peers.iter().flatten().filter(|p| p.view(t_us).age <= max_age).count() as u32;
        let calm = still && peers >= self.cfg.auto_anchor_min_peers
            && self.est.range_innov_ratio() < self.cfg.auto_anchor_lock;
        if !calm {
            self.calm_since_us = None;
            return;
        }
        let since = *self.calm_since_us.get_or_insert(t_us);
        if t_us - since >= self.cfg.auto_anchor_hold_us {
            let (p, q) = (self.est.p, self.est.q);
            self.auto_anchor = Some(p);
            self.anchor_rb = self.est.rb;
            self.est.set_pose(p, Vec3::ZERO, q, self.cfg.auto_anchor_sigma, 0.05, self.cfg.init_att_sigma);
        }
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
        self.auto_anchor_step(t_us);
        if self.cfg.planar_on && self.anchor().is_none() && t_us >= self.next_planar_us {
            self.next_planar_us = t_us + self.cfg.planar_period_us;
            self.est.update_alt(self.cfg.planar_z, self.cfg.planar_sigma);
        }
        if let Some((p, sigma)) = self.anchor() {
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
        if self.anchor().is_some() {
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

    /// Range sigma for a measurement whose first-path power sits `gap_db`
    /// below the total (NaN: unknown, nominal); None if it's to be dropped.
    pub fn range_sigma_for(&self, gap_db: f32) -> Option<f32> {
        let c = &self.cfg;
        if !(gap_db > c.nlos_gap_db) {
            return Some(c.range_sigma); // also NaN
        }
        if c.nlos_reject_db > 0.0 && gap_db > c.nlos_reject_db {
            return None;
        }
        let extra = ((gap_db - c.nlos_gap_db) * c.nlos_sigma_per_db).min(c.nlos_sigma_max);
        Some(libm::sqrtf(c.range_sigma * c.range_sigma + extra * extra))
    }

    /// A UWB range (m) to `peer_id`, measured at `t_us`, its first-path
    /// power `fp_gap_db` below the total received power (NaN if unknown).
    pub fn range(&mut self, t_us: i64, peer_id: u32, range_m: f32, fp_gap_db: f32) -> RangeResult {
        if !self.initialized {
            return RangeResult::NotReady;
        }
        let Some(i) = self.peer_index(peer_id) else {
            return if self.anchor().is_some() { RangeResult::Surveyed } else { RangeResult::UnknownPeer };
        };
        let mut peer = self.peers[i].unwrap();
        let view = peer.view(t_us);
        // The peer's share of the range bias, as it advertises it.
        let range_m = range_m - peer.advert.range_bias;
        let Some(mut sigma) = self.range_sigma_for(fp_gap_db) else {
            peer.ranges_skipped += 1;
            self.peers[i] = Some(peer);
            return RangeResult::Blocked;
        };
        // Coincident with the peer (a swarm that boots together all starts
        // at the origin): no range can be linearized, and still widgets may
        // never drift apart on their own. Both ends get this range, so each
        // steps half of it apart, opposite ways along an axis from the pair.
        if self.anchor().is_none() && self.est.p.sub(view.p).norm() < 0.05 && range_m > 0.1 {
            let (lo, hi) = (self.id.min(peer_id), self.id.max(peer_id));
            let a = (lo.wrapping_mul(2_654_435_761) ^ hi) % 6283;
            let a = a as f32 / 1000.0;
            let s = if self.id > peer_id { 0.5 } else { -0.5 };
            self.est.shift_position(Vec3::new(libm::cosf(a), libm::sinf(a), 0.0).scale(s * range_m));
        }
        let predicted = self.est.p.sub(view.p).norm() + self.est.rb;
        if self.cfg.nlos_asymmetric && range_m <= predicted {
            sigma = self.cfg.range_sigma;
        }
        if self.anchor().is_some() {
            let flagged = fp_gap_db > self.cfg.nlos_gap_db;
            let r = if self.cfg.bias_learn_anchored && (!flagged || self.cfg.bias_learn_flagged)
                && !view.still && !view.anchored
                && view.age <= self.cfg.coop.max_advert_age && view.lock < self.cfg.bias_learn_lock
            {
                let rel = view.lock * self.cfg.range_sigma;
                let nvar = self.cfg.bias_learn_nvar_scale * rel * rel;
                if self.est.update_range_bias(view.p, nvar, range_m, sigma) {
                    RangeResult::Calibrated
                } else {
                    RangeResult::Surveyed
                }
            } else {
                RangeResult::Surveyed
            };
            return r;
        }
        // Coincident estimates (all widgets start at the origin) can't be
        // linearized: the estimator declines those ranges, which isn't a
        // gate rejection and mustn't count toward divergence recovery.
        let degenerate = self.est.p.sub(view.p).norm() < 0.05;
        let me_still = self.is_still(t_us);
        let flagged = fp_gap_db > self.cfg.nlos_gap_db;
        let mut range_m = range_m;
        if flagged && self.cfg.nlos_clip_m > 0.0 && (me_still || !self.cfg.nlos_clip_still_only)
            && self.est.range_innov_ratio() < self.cfg.coop.lock_ratio_none
        {
            range_m = range_m.min(predicted + self.cfg.nlos_clip_m);
        }
        let gate = self.est.params.gate;
        if flagged && self.cfg.nlos_gate > 0.0 {
            self.est.params.gate = self.cfg.nlos_gate;
        }
        self.est.learn_range_bias = (!me_still || self.cfg.bias_learn_still)
            && (!flagged || self.cfg.bias_learn_flagged);
        let (outcome, _decision): (Outcome, Decision) = coop::range_update_weighted(
            &self.cfg.coop,
            &mut self.est,
            &view,
            range_m,
            self.cfg.range_sigma,
            sigma,
            false, // no GPS on these widgets
            me_still,
        );
        self.est.params.gate = gate;
        self.est.learn_range_bias = true;
        let r = match outcome {
            Outcome::Applied { accepted: true, .. } => RangeResult::Accepted,
            Outcome::Applied { accepted: false, .. } => RangeResult::Gated,
            Outcome::Skipped(SkipReason::StaleAdvert) => RangeResult::StaleAdvert,
            Outcome::Skipped(SkipReason::Anchored) => RangeResult::Anchored,
            Outcome::Skipped(SkipReason::ColdPeer) => RangeResult::ColdPeer,
            Outcome::Skipped(SkipReason::MovingPeer) => RangeResult::MovingPeer,
        };
        match r {
            RangeResult::Accepted => peer.ranges_ok += 1,
            RangeResult::Gated => peer.ranges_gated += 1,
            _ => peer.ranges_skipped += 1,
        }
        self.peers[i] = Some(peer);
        let window = self.cfg.regain_after_gated.min(32);
        let gated = match r {
            RangeResult::Gated if !degenerate => Some(true),
            RangeResult::Accepted => Some(false),
            _ => None,
        };
        if let (Some(g), true) = (gated, window > 0) {
            self.gated_bits = (self.gated_bits << 1) | g as u32;
            self.gated_n = (self.gated_n + 1).min(window);
        }
        let mask = if window >= 32 { u32::MAX } else { (1u32 << window) - 1 };
        if window > 0 && self.gated_n >= window
            && (self.gated_bits & mask).count_ones() as f32 >= self.cfg.regain_gated_frac * window as f32
        {
            self.gated_bits = 0;
            self.gated_n = 0;
            self.regains += 1;
            let (p, q) = (self.est.p, self.est.q);
            self.est.set_pose(p, Vec3::ZERO, q, self.cfg.init_pos_sigma, self.cfg.init_vel_sigma,
                              self.cfg.init_att_sigma);
        }
        r
    }

    /// My advertisement as of now. Surveyed widgets advertise their known
    /// position as an absolute source (var ~0, locked).
    pub fn advert(&mut self) -> Advert {
        self.seq = self.seq.wrapping_add(1);
        let still = self.is_still(self.last_t_us);
        match self.anchor() {
            Some((p, sigma)) => Advert {
                widget_id: self.id,
                seq: self.seq,
                p,
                v: Vec3::ZERO,
                quality: if self.surveyed.is_some() { 1e-4 } else { sigma * sigma },
                flags: FLAG_ANCHORED | FLAG_STATIC,
                lock: 1.0,
                range_bias: self.est.rb,
            },
            None => Advert {
                widget_id: self.id,
                seq: self.seq,
                p: self.est.p,
                v: self.est.v,
                quality: self.est.pos_var(),
                flags: if still { FLAG_STATIC } else { 0 },
                lock: self.est.range_innov_ratio(),
                range_bias: self.est.rb,
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
        Advert { widget_id: id, seq: 1, p, v: Vec3::ZERO, quality: 1e-4, flags: FLAG_ANCHORED, lock: 1.0, range_bias: 0.0 }
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
                    n.range(t, 100 + k as u32, truth.sub(*a).norm(), f32::NAN);
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
                                     quality: n.est.pos_var(), flags: 0, lock: 1.0, range_bias: 0.0 };
                    n.heard(t, a);
                }
                // ~33 exchanges/s, round-robin over the peers.
                if step % 6 == 0 {
                    let k = (step / 6) as usize % 3;
                    n.range(t, 10 + k as u32, peers[k].norm(), f32::NAN);
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

    /// Three still widgets ranging each other lock and auto-anchor; a fourth
    /// started far off then converges onto the anchors through full
    /// (downhill) updates, which mean-only updates alone never manage.
    #[test]
    fn still_mesh_auto_anchors_and_a_newcomer_converges() {
        let truth = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0), Vec3::new(1.0, 2.5, 0.0),
                     Vec3::new(1.5, 1.0, 0.0)];
        let mut cfg = NodeConfig::default();
        cfg.planar_on = true;
        let mut nodes: Vec<Node> = (0..4).map(|i| Node::new(10 + i as u32, false, cfg)).collect();
        for (n, p) in nodes.iter_mut().zip(truth) {
            n.tick(0);
            n.est.p = p;
        }
        nodes[3].est.p = Vec3::new(-4.0, 3.0, 0.0); // the newcomer, way off
        let mut t = 0;
        let mut step = 0usize;
        let mut anchored_at = None;
        for _ in 0..6000 {
            t += 10_000;
            step += 1;
            let adverts: Vec<Advert> = nodes.iter_mut().map(|n| n.advert()).collect();
            for (i, n) in nodes.iter_mut().enumerate() {
                for (j, a) in adverts.iter().enumerate() {
                    if i != j {
                        n.heard(t, *a);
                    }
                }
                n.tick(t);
                // One range per tick, round-robin (both ends measure it).
                let j = (i + 1 + step % 3) % 4;
                n.range(t, 10 + j as u32, truth[i].sub(truth[j]).norm(), f32::NAN);
            }
            if anchored_at.is_none() && nodes[..3].iter().all(|n| n.auto_anchored()) {
                anchored_at = Some(t);
            }
        }
        assert!(anchored_at.is_some(), "the still mesh anchored itself");
        for n in &nodes[..3] {
            assert!(n.auto_anchored());
        }
        // Anchorless: the frame is wherever the mesh settled, so check the
        // shape (every distance) rather than absolute coordinates.
        for i in 0..4 {
            for j in i + 1..4 {
                let est = nodes[i].est.p.sub(nodes[j].est.p).norm();
                let tru = truth[i].sub(truth[j]).norm();
                assert!((est - tru).abs() < 0.1, "{}-{}: {} vs {} m", i, j, est, tru);
            }
        }
    }

    /// A widget whose estimate ran 30 m off with an overconfident
    /// covariance gets every range gated; it must notice and re-acquire
    /// against its anchored peers instead of dead-reckoning away for good.
    #[test]
    fn diverged_widget_reacquires() {
        let truth = Vec3::new(2.0, 3.0, 0.0);
        let anchors = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(6.0, 0.0, 0.0), Vec3::new(0.0, 6.0, 0.0),
                       Vec3::new(6.0, 6.0, 0.0)];
        let mut cfg = NodeConfig::default();
        cfg.centroid_bleed_tau_s = 0.0;
        let mut n = Node::new(7, false, cfg);
        let mut t = 0;
        n.tick(t);
        let q = n.est.q;
        n.est.set_pose(Vec3::new(30.0, -20.0, 0.0), Vec3::ZERO, q, 0.3, 0.1, 0.3);
        for step in 0..1500 {
            t += 10_000;
            n.tick(t);
            for (k, a) in anchors.iter().enumerate() {
                n.heard(t, locked_peer(100 + k as u32, *a));
                if step % 4 == k {
                    n.range(t, 100 + k as u32, truth.sub(*a).norm(), f32::NAN);
                }
            }
        }
        assert!(n.regains >= 1, "noticed the divergence");
        let err = n.est.p.sub(truth).norm();
        assert!(err < 0.1, "{} m off at {:?}", err, n.est.p);
    }

    #[test]
    fn blocked_ranges_widen_only_when_long_and_reject_past_the_limit() {
        let mut n = Node::new(1, false, NodeConfig::default());
        let c = n.cfg;
        assert_eq!(n.range_sigma_for(f32::NAN), Some(c.range_sigma));
        assert_eq!(n.range_sigma_for(c.nlos_gap_db - 1.0), Some(c.range_sigma));
        let s = n.range_sigma_for(c.nlos_gap_db + 4.0).unwrap();
        let extra = 4.0 * c.nlos_sigma_per_db;
        assert!((s * s - (c.range_sigma * c.range_sigma + extra * extra)).abs() < 1e-5);
        let capped = n.range_sigma_for(c.nlos_gap_db + 1000.0).unwrap();
        assert!((capped * capped - (c.range_sigma * c.range_sigma + c.nlos_sigma_max * c.nlos_sigma_max)).abs() < 1e-5);
        n.cfg.nlos_reject_db = 20.0;
        assert_eq!(n.range_sigma_for(21.0), None);
        n.tick(0);
        n.heard(0, locked_peer(2, Vec3::new(3.0, 0.0, 0.0)));
        assert_eq!(n.range(1000, 2, 3.0, 25.0), RangeResult::Blocked);
    }

    #[test]
    fn peer_range_bias_is_subtracted() {
        // Surveyed peers advertising a bias: ranges carry it, and the
        // widget converges to the truth anyway.
        let anchors = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(6.0, 0.0, 0.0), Vec3::new(0.0, 6.0, 0.0),
                       Vec3::new(6.0, 6.0, 2.0)];
        let truth = Vec3::new(2.0, 3.0, 1.0);
        let mut cfg = NodeConfig::default();
        cfg.params.init_range_bias_sigma = 0.0; // isolate the peers' bias
        let mut n = Node::new(1, false, cfg);
        let mut t = 0;
        for _ in 0..400 {
            t += 10_000;
            n.tick(t);
            for (k, a) in anchors.iter().enumerate() {
                let mut adv = locked_peer(100 + k as u32, *a);
                adv.range_bias = 0.15;
                n.heard(t, adv);
                n.range(t, 100 + k as u32, truth.sub(*a).norm() + 0.15, f32::NAN);
            }
        }
        assert!(n.est.p.sub(truth).norm() < 0.05, "error {}", n.est.p.sub(truth).norm());
    }

    #[test]
    fn stale_and_unknown_peers_are_skipped() {
        let mut n = Node::new(1, false, NodeConfig::default());
        n.tick(0);
        assert_eq!(n.range(10, 2, 3.0, f32::NAN), RangeResult::UnknownPeer);
        n.heard(0, locked_peer(2, Vec3::new(3.0, 0.0, 0.0)));
        assert_eq!(n.range(2 * S, 2, 3.0, f32::NAN), RangeResult::StaleAdvert);
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
        assert_eq!(n.range(S, 9, 1.0, f32::NAN), RangeResult::Surveyed);
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
