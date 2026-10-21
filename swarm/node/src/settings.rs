//! The settings registry: every live tunable of a node, with a stable id,
//! a name, a type, a valid range and its compiled default
//! (docs/swarm-network.md "Live settings"). swarm_net moves raw key/value
//! layers around the swarm; this decides what a key means and builds the
//! node's configuration from the layers.
//!
//! Ids are part of the wire format and of what nodes have stored: never
//! renumber or reuse one. Ranges are generous sanity bounds (a typo can't
//! make a node unusable), not tuning advice.

use core::fmt::Write;

use swarm_core::math::{Mat3, Vec3, MAT3_IDENTITY};
use swarm_net::mesh::MeshConfig;

use crate::node::NodeConfig;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    F32,
    I32,
    Bool,
}

/// Everything settings can change.
#[derive(Clone, Copy, Debug)]
pub struct Tunables {
    pub node: NodeConfig,
    pub mesh: MeshConfig,
    /// DW3000 TX/RX antenna delay (device units, ~15.65 ps).
    pub antenna_delay: i32,
    pub survey_on: bool,
    pub survey_p: Vec3,
    pub survey_sigma: f32,
    pub mag_on: bool,
    pub mag_hard: Vec3,
    pub mag_soft: Mat3,
}

impl Default for Tunables {
    fn default() -> Tunables {
        Tunables {
            node: NodeConfig::default(),
            mesh: MeshConfig::default(),
            antenna_delay: 16350,
            survey_on: false,
            survey_p: Vec3::ZERO,
            survey_sigma: 0.02,
            mag_on: false,
            mag_hard: Vec3::ZERO,
            mag_soft: MAT3_IDENTITY,
        }
    }
}

pub struct Setting {
    pub id: u16,
    pub name: &'static str,
    pub kind: Kind,
    pub min: f32,
    pub max: f32,
    pub get: fn(&Tunables) -> u32,
    pub set: fn(&mut Tunables, u32),
}

impl Setting {
    /// The raw value as a float (for range checks and reports).
    pub fn as_f32(&self, raw: u32) -> f32 {
        match self.kind {
            Kind::F32 => f32::from_bits(raw),
            Kind::I32 => raw as i32 as f32,
            Kind::Bool => (raw != 0) as u32 as f32,
        }
    }

    pub fn valid(&self, raw: u32) -> bool {
        let v = self.as_f32(raw);
        v.is_finite() && v >= self.min && v <= self.max
    }

    pub fn default_raw(&self) -> u32 {
        (self.get)(&Tunables::default())
    }
}

macro_rules! f32s {
    ($id:expr, $name:expr, $min:expr, $max:expr, $($f:tt)+) => {
        Setting { id: $id, name: $name, kind: Kind::F32, min: $min, max: $max,
                  get: |t| t.$($f)+.to_bits(), set: |t, v| t.$($f)+ = f32::from_bits(v) }
    };
}
macro_rules! i32s {
    ($id:expr, $name:expr, $min:expr, $max:expr, $ty:ty, $($f:tt)+) => {
        Setting { id: $id, name: $name, kind: Kind::I32, min: $min as f32, max: $max as f32,
                  get: |t| t.$($f)+ as i32 as u32, set: |t, v| t.$($f)+ = v as i32 as $ty }
    };
}
macro_rules! bools {
    ($id:expr, $name:expr, $($f:tt)+) => {
        Setting { id: $id, name: $name, kind: Kind::Bool, min: 0.0, max: 1.0,
                  get: |t| t.$($f)+ as u32, set: |t, v| t.$($f)+ = v != 0 }
    };
}

/// The registry. 1xx estimator, 2xx cooperative rules, 3xx node aiding and
/// timing, 4xx device calibration, 5xx mesh.
pub static REGISTRY: &[Setting] = &[
    f32s!(100, "est.accel_noise", 1e-5, 10.0, node.params.accel_noise),
    f32s!(101, "est.gyro_noise", 1e-6, 1.0, node.params.gyro_noise),
    f32s!(102, "est.accel_bias_rw", 0.0, 1.0, node.params.accel_bias_rw),
    f32s!(103, "est.gyro_bias_rw", 0.0, 1.0, node.params.gyro_bias_rw),
    f32s!(104, "est.init_accel_bias_sigma", 0.0, 10.0, node.params.init_accel_bias_sigma),
    f32s!(105, "est.init_gyro_bias_sigma", 0.0, 1.0, node.params.init_gyro_bias_sigma),
    f32s!(106, "est.gate", 0.0, 100.0, node.params.gate),
    f32s!(107, "est.innov_alpha", 0.0, 1.0, node.params.innov_alpha),
    f32s!(108, "est.init_range_bias_sigma", 0.0, 10.0, node.params.init_range_bias_sigma),
    f32s!(109, "est.range_bias_rw", 0.0, 1.0, node.params.range_bias_rw),
    f32s!(200, "coop.downhill_ratio", 0.0, 1.0, node.coop.downhill_ratio),
    f32s!(201, "coop.mean_only_aoa_damping", 0.0, 1e4, node.coop.mean_only_aoa_damping),
    f32s!(202, "coop.mean_only_range_damping", 0.0, 1e4, node.coop.mean_only_range_damping),
    f32s!(203, "coop.lock_ratio_full", 0.0, 100.0, node.coop.lock_ratio_full),
    f32s!(204, "coop.lock_ratio_none", 0.0, 100.0, node.coop.lock_ratio_none),
    f32s!(205, "coop.peer_lock_factor", 0.0, 100.0, node.coop.peer_lock_factor),
    f32s!(206, "coop.max_advert_age", 0.0, 60.0, node.coop.max_advert_age),
    f32s!(207, "coop.aoa_downhill_max_var", 0.0, 1e6, node.coop.aoa_downhill_max_var),
    bools!(208, "coop.mean_only_pos_only", node.coop.mean_only_pos_only),
    f32s!(300, "range_sigma", 0.001, 10.0, node.range_sigma),
    f32s!(338, "nlos.gap_db", 0.0, 100.0, node.nlos_gap_db),
    f32s!(339, "nlos.sigma_per_db", 0.0, 10.0, node.nlos_sigma_per_db),
    f32s!(340, "nlos.sigma_max", 0.0, 100.0, node.nlos_sigma_max),
    f32s!(341, "nlos.reject_db", 0.0, 100.0, node.nlos_reject_db),
    f32s!(346, "nlos.gate", 0.0, 100.0, node.nlos_gate),
    f32s!(347, "nlos.clip_m", 0.0, 100.0, node.nlos_clip_m),
    bools!(348, "nlos.clip_still_only", node.nlos_clip_still_only),
    bools!(349, "nlos.asymmetric", node.nlos_asymmetric),
    f32s!(350, "regain_gated_frac", 0.0, 1.0, node.regain_gated_frac),
    bools!(342, "bias.learn_anchored", node.bias_learn_anchored),
    f32s!(343, "bias.learn_lock", 0.0, 100.0, node.bias_learn_lock),
    f32s!(344, "bias.learn_nvar_scale", 0.0, 1e4, node.bias_learn_nvar_scale),
    f32s!(345, "bias.reanchor_m", 0.0, 10.0, node.reanchor_bias_m),
    i32s!(301, "gravity_every", 0, 1000, u32, node.gravity_every),
    f32s!(302, "gravity_gate", 0.0, 20.0, node.gravity_gate),
    f32s!(303, "gravity_sigma", 1e-4, 10.0, node.gravity_sigma),
    bools!(336, "gravity_still_only", node.gravity_still_only),
    f32s!(304, "zero_vel_sigma", 0.0, 1000.0, node.zero_vel_sigma),
    i32s!(305, "zero_vel_period_us", 1000, 60_000_000, i64, node.zero_vel_period_us),
    bools!(306, "zero_vel_keep_position", node.zero_vel_keep_position),
    f32s!(307, "centroid_bleed_tau_s", 0.0, 3600.0, node.centroid_bleed_tau_s),
    i32s!(308, "centroid_period_us", 1000, 60_000_000, i64, node.centroid_period_us),
    i32s!(309, "survey_period_us", 1000, 60_000_000, i64, node.survey_period_us),
    f32s!(310, "survey_sigma_default", 1e-4, 100.0, node.survey_sigma),
    f32s!(311, "mag_sigma", 1e-4, 10.0, node.mag_sigma),
    i32s!(312, "mag_period_us", 1000, 60_000_000, i64, node.mag_period_us),
    f32s!(313, "init_pos_sigma", 0.0, 1e5, node.init_pos_sigma),
    f32s!(314, "init_vel_sigma", 0.0, 1e3, node.init_vel_sigma),
    f32s!(315, "init_att_sigma", 0.0, 10.0, node.init_att_sigma),
    f32s!(316, "max_dt", 1e-4, 1.0, node.max_dt),
    i32s!(317, "no_imu_period_us", 1000, 1_000_000, i64, node.no_imu_period_us),
    f32s!(318, "no_imu_accel_noise", 0.0, 100.0, node.no_imu_accel_noise),
    bools!(319, "planar.on", node.planar_on),
    bools!(323, "still.on", node.still_on),
    bools!(330, "auto_anchor.on", node.auto_anchor_on),
    i32s!(337, "regain_after_gated", 0, 100_000, u32, node.regain_after_gated),
    f32s!(331, "auto_anchor.lock", 0.0, 100.0, node.auto_anchor_lock),
    i32s!(332, "auto_anchor.hold_us", 0, 600_000_000, i64, node.auto_anchor_hold_us),
    f32s!(333, "auto_anchor.sigma", 1e-3, 100.0, node.auto_anchor_sigma),
    i32s!(334, "auto_anchor.min_peers", 0, 16, u32, node.auto_anchor_min_peers),
    f32s!(335, "auto_anchor.release_sigma", 1e-3, 1000.0, node.auto_anchor_release_sigma),
    f32s!(324, "still.gyro", 0.0, 10.0, node.still_gyro),
    f32s!(325, "still.accel", 0.0, 20.0, node.still_accel),
    i32s!(326, "still.tau_us", 1000, 10_000_000, i64, node.still_tau_us),
    i32s!(327, "still.hold_us", 0, 60_000_000, i64, node.still_hold_us),
    f32s!(328, "still.sigma", 1e-4, 100.0, node.still_sigma),
    i32s!(329, "still.period_us", 1000, 10_000_000, i64, node.still_period_us),
    f32s!(320, "planar.z", -1e4, 1e4, node.planar_z),
    f32s!(321, "planar.sigma", 1e-3, 100.0, node.planar_sigma),
    i32s!(322, "planar.period_us", 1000, 60_000_000, i64, node.planar_period_us),
    i32s!(400, "antenna_delay", 0, 65535, i32, antenna_delay),
    bools!(401, "survey.on", survey_on),
    f32s!(402, "survey.x", -1e5, 1e5, survey_p.x),
    f32s!(403, "survey.y", -1e5, 1e5, survey_p.y),
    f32s!(404, "survey.z", -1e5, 1e5, survey_p.z),
    f32s!(405, "survey.sigma", 1e-4, 100.0, survey_sigma),
    bools!(410, "mag.on", mag_on),
    f32s!(411, "mag.hard.x", -1e5, 1e5, mag_hard.x),
    f32s!(412, "mag.hard.y", -1e5, 1e5, mag_hard.y),
    f32s!(413, "mag.hard.z", -1e5, 1e5, mag_hard.z),
    f32s!(414, "mag.soft.00", -100.0, 100.0, mag_soft[0][0]),
    f32s!(415, "mag.soft.01", -100.0, 100.0, mag_soft[0][1]),
    f32s!(416, "mag.soft.02", -100.0, 100.0, mag_soft[0][2]),
    f32s!(417, "mag.soft.10", -100.0, 100.0, mag_soft[1][0]),
    f32s!(418, "mag.soft.11", -100.0, 100.0, mag_soft[1][1]),
    f32s!(419, "mag.soft.12", -100.0, 100.0, mag_soft[1][2]),
    f32s!(420, "mag.soft.20", -100.0, 100.0, mag_soft[2][0]),
    f32s!(421, "mag.soft.21", -100.0, 100.0, mag_soft[2][1]),
    f32s!(422, "mag.soft.22", -100.0, 100.0, mag_soft[2][2]),
    i32s!(500, "mesh.hop_limit", 0, 7, u8, mesh.hop_limit),
    i32s!(501, "mesh.relay_window", 0, 16, u32, mesh.relay_window),
    f32s!(502, "mesh.rssi_strong", -150.0, 0.0, mesh.rssi_strong),
    f32s!(503, "mesh.rssi_weak", -150.0, 0.0, mesh.rssi_weak),
    i32s!(504, "mesh.ack_timeout_us", 10_000, 60_000_000, i64, mesh.ack_timeout_us),
    i32s!(505, "mesh.max_tries", 1, 15, u8, mesh.max_tries),
];

pub fn by_id(id: u16) -> Option<&'static Setting> {
    REGISTRY.iter().find(|s| s.id == id)
}

pub fn by_name(name: &str) -> Option<&'static Setting> {
    REGISTRY.iter().find(|s| s.name == name)
}

/// Where a setting's effective value came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Default,
    Swarm,
    Override,
    /// A stored value was out of range and ignored.
    Invalid,
}

/// Tunables from the layers (`swarm(id)`, `overrides(id)`: raw values if
/// set). Invalid stored values fall back to the next layer down.
pub fn resolve(swarm: impl Fn(u16) -> Option<u32>, overrides: impl Fn(u16) -> Option<u32>) -> Tunables {
    let mut t = Tunables::default();
    for s in REGISTRY {
        if let Some(v) = overrides(s.id).filter(|&v| s.valid(v)).or_else(|| swarm(s.id).filter(|&v| s.valid(v))) {
            (s.set)(&mut t, v);
        }
    }
    t
}

pub fn source(s: &Setting, swarm: Option<u32>, over: Option<u32>) -> Source {
    match (over, swarm) {
        (Some(v), _) if s.valid(v) => Source::Override,
        (Some(_), Some(v)) if s.valid(v) => Source::Swarm,
        (Some(_), _) => Source::Invalid,
        (None, Some(v)) if s.valid(v) => Source::Swarm,
        (None, Some(_)) => Source::Invalid,
        (None, None) => Source::Default,
    }
}

/// Parse a JSON-ish number/bool for `s` into its raw value.
pub fn parse(s: &Setting, text: &str) -> Option<u32> {
    let text = text.trim();
    let raw = match s.kind {
        Kind::Bool => match text {
            "true" | "1" => 1,
            "false" | "0" => 0,
            _ => return None,
        },
        Kind::I32 => text.parse::<i32>().ok().or_else(|| text.parse::<f32>().ok().map(|f| f as i32))? as u32,
        Kind::F32 => text.parse::<f32>().ok()?.to_bits(),
    };
    s.valid(raw).then_some(raw)
}

/// Write the value for JSON.
pub fn format(s: &Setting, raw: u32, out: &mut impl Write) -> core::fmt::Result {
    match s.kind {
        Kind::F32 => write!(out, "{}", f32::from_bits(raw)),
        Kind::I32 => write!(out, "{}", raw as i32),
        Kind::Bool => write!(out, "{}", raw != 0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_and_names_are_unique_and_defaults_valid() {
        for (i, a) in REGISTRY.iter().enumerate() {
            assert!(a.valid(a.default_raw()), "{} default out of range", a.name);
            for b in &REGISTRY[i + 1..] {
                assert_ne!(a.id, b.id, "duplicate id {}", a.id);
                assert_ne!(a.name, b.name);
            }
        }
    }

    #[test]
    fn setters_hit_their_own_field() {
        for s in REGISTRY {
            let mut t = Tunables::default();
            let raw = match s.kind {
                Kind::F32 => ((s.min + s.max) * 0.5 + 0.25).min(s.max).to_bits(),
                Kind::I32 => (s.min as i32 + 1) as u32,
                Kind::Bool => (s.default_raw() == 0) as u32,
            };
            (s.set)(&mut t, raw);
            assert_eq!((s.get)(&t), raw, "{}", s.name);
            for other in REGISTRY.iter().filter(|o| o.id != s.id) {
                assert_eq!((other.get)(&t), other.default_raw(), "{} changed {}", s.name, other.name);
            }
        }
    }

    #[test]
    fn override_beats_swarm_beats_default_and_invalid_falls_through() {
        let gate = by_name("est.gate").unwrap();
        let t = resolve(|id| (id == 106).then_some(7.0f32.to_bits()), |id| (id == 106).then_some(9.0f32.to_bits()));
        assert_eq!(t.node.params.gate, 9.0);
        let t = resolve(|id| (id == 106).then_some(7.0f32.to_bits()), |_| None);
        assert_eq!(t.node.params.gate, 7.0);
        let t = resolve(|id| (id == 106).then_some(7.0f32.to_bits()), |id| (id == 106).then_some(f32::NAN.to_bits()));
        assert_eq!(t.node.params.gate, 7.0, "NaN override ignored");
        assert_eq!(source(gate, Some(7.0f32.to_bits()), Some(f32::NAN.to_bits())), Source::Swarm);
        assert_eq!(parse(gate, "1e9"), None, "out of range");
        assert_eq!(parse(by_name("antenna_delay").unwrap(), "16400"), Some(16400));
        assert_eq!(parse(by_name("survey.on").unwrap(), "true"), Some(1));
    }
}
