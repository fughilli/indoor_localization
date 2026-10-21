//! Cooperative fusion rules: how a widget folds a UWB measurement to a peer
//! into its own [`Estimator`], given only the peer's *advertised* pose.
//!
//! This is the part that makes swarms work (design doc §4.2), and it lives
//! here so the simulation and the firmware run the same code: the sim's
//! `World` calls it through the C ABI (`ffi.rs`), the firmware's node runtime
//! (`swarm/node`) calls it directly. Each peer measurement is classified by
//! relative quality:
//!
//! - **downhill** (peer clearly better localized): full Kalman update, with
//!   the peer variance floored at half my own (cold-start damping — bounded,
//!   re-linearized steps instead of unity-gain snaps from a garbage
//!   linearization point);
//! - **comparable**: mean-only update — the state relaxes onto the mutually
//!   consistent mesh but no information gain is claimed, so covariance keeps
//!   honestly tracking the unobservable absolute frame. The relaxation gain
//!   anneals with my own range-innovation lock;
//! - **skip**: stale adverts, cold joiners (lock much worse than mine), and
//!   mean-only pulls on an anchored widget (a 1 Hz absolute source must not be
//!   overpowered by mesh consensus).

use crate::estimator::Estimator;
use crate::math::Vec3;

/// Rule constants. Defaults are the values the simulation was tuned with.
#[derive(Clone, Copy, Debug)]
pub struct CoopConfig {
    /// Downhill when peer variance < this fraction of mine.
    pub downhill_ratio: f32,
    /// Mean-only AoA gain divisor (~K = 1/(1+f)): AoA only steers the slowly
    /// drifting mesh orientation in the comparable regime.
    pub mean_only_aoa_damping: f32,
    /// Fully annealed mean-only range damping (K ~ 1/(1+f) at lock); <= 0.5
    /// disables annealing.
    pub mean_only_range_damping: f32,
    /// Lock ratio at/below which the mean-only gain is fully annealed.
    pub lock_ratio_full: f32,
    /// Lock ratio at/above which the mean-only gain is cold-start (and above
    /// which a peer can't be downhill).
    pub lock_ratio_none: f32,
    /// Refuse mean-only pulls from a peer whose lock is this many times worse
    /// than mine (a cold joiner's variance looks like a veteran's).
    pub peer_lock_factor: f32,
    /// Adverts older than this (s) are ignored: departed widgets.
    pub max_advert_age: f32,
    /// Below this own variance (m^2) AoA is also used in the downhill regime.
    pub aoa_downhill_max_var: f32,
    /// Mean-only range updates move position only, not velocity / attitude /
    /// biases. Mean-only never shrinks P, so its cross-covariance gains stay
    /// large forever; in a swarm with no absolute source at all a mesh that
    /// can't close feeds the same innovation into velocity every exchange
    /// (hardware: a still node reached 41 m/s). Off in the outdoor sim
    /// tuning, where GPS bounds it and the velocity coupling helps tracking.
    pub mean_only_pos_only: bool,
}

impl Default for CoopConfig {
    fn default() -> CoopConfig {
        CoopConfig {
            downhill_ratio: 0.5,
            mean_only_aoa_damping: 30.0,
            mean_only_range_damping: 30.0,
            lock_ratio_full: 2.0,
            lock_ratio_none: 6.0,
            peer_lock_factor: 3.0,
            max_advert_age: 1.0,
            aoa_downhill_max_var: 25.0,
            mean_only_pos_only: false,
        }
    }
}

/// A peer as this widget currently believes it to be: its last advertisement,
/// already dead-reckoned to now by the advertised velocity.
#[derive(Clone, Copy, Debug)]
pub struct PeerView {
    pub p: Vec3,
    /// Advertised mean position variance, m^2.
    pub var: f32,
    /// Advertised range-innovation lock ratio.
    pub lock: f32,
    /// Seconds since the advertisement was made.
    pub age: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Regime {
    Downhill,
    MeanOnly,
}

/// What [`decide`] concluded for one measurement to one peer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Decision {
    /// Don't use the measurement.
    Skip(SkipReason),
    /// Use it in `regime`, folding `nvar` (m^2) into the range noise.
    /// `my_var` is my position variance at decision time (before the range
    /// update), which the exchange's AoA update is scaled by.
    Use { regime: Regime, nvar: f32, my_var: f32 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SkipReason {
    StaleAdvert,
    /// Mean-only pull on an anchored (absolute-source) widget.
    Anchored,
    /// Peer's lock is far worse than mine: a cold joiner.
    ColdPeer,
}

/// Classify one range measurement to `peer` for a widget whose estimator is
/// `me`. `anchored`: I have a healthy absolute source (GPS). `range_sigma` is
/// the UWB range noise (m), used by the downhill cold-start floor.
pub fn decide(
    cfg: &CoopConfig,
    me: &Estimator,
    peer: &PeerView,
    anchored: bool,
    range_sigma: f32,
) -> Decision {
    if peer.age > cfg.max_advert_age {
        return Decision::Skip(SkipReason::StaleAdvert);
    }
    let my_var = me.pos_var();
    // Variance ranks peers only when meaningfully small; anchorless variances
    // are honestly huge on both sides and diverge for the wrong reasons, so
    // the advertised lock gates both regimes.
    let downhill = peer.var < cfg.downhill_ratio * my_var && peer.lock < cfg.lock_ratio_none;
    if downhill {
        // Damp with the peer's variance, floored for cold start.
        let floor = 0.5 * my_var - range_sigma * range_sigma;
        let nvar = if peer.var > floor { peer.var } else { floor };
        return Decision::Use { regime: Regime::Downhill, nvar, my_var };
    }
    if anchored {
        return Decision::Skip(SkipReason::Anchored);
    }
    let my_lock = me.range_innov_ratio();
    let lock_floor = if my_lock > cfg.lock_ratio_full { my_lock } else { cfg.lock_ratio_full };
    if peer.lock > cfg.peer_lock_factor * lock_floor {
        return Decision::Skip(SkipReason::ColdPeer);
    }
    // Annealing: damping factor from my own lock, 0.5 -> range damping.
    let mut anneal = 1.0;
    let mut u = (cfg.lock_ratio_none - my_lock) / (cfg.lock_ratio_none - cfg.lock_ratio_full);
    u = if u < 0.0 { 0.0 } else if u > 1.0 { 1.0 } else { u };
    if cfg.mean_only_range_damping > 0.5 {
        anneal = 1.0 + u * (cfg.mean_only_range_damping - 0.5) / 0.5;
    }
    // Mean-only: no covariance update, so no double-count risk — damp
    // against my own variance only (not the peer's honestly-huge absolute
    // one, or AoA vanishes and the mesh orientation floats).
    Decision::Use { regime: Regime::MeanOnly, nvar: anneal * 0.5 * my_var, my_var }
}

/// Outcome of [`range_update`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    Skipped(SkipReason),
    /// Applied in `regime`; `accepted` is false if the innovation gate
    /// rejected it.
    Applied { regime: Regime, accepted: bool },
}

/// Decide and apply one UWB range (m) to `peer`. Returns what happened and
/// the decision (callers with AoA pass it on to [`aoa_update`]).
pub fn range_update(
    cfg: &CoopConfig,
    me: &mut Estimator,
    peer: &PeerView,
    measured: f32,
    range_sigma: f32,
    anchored: bool,
) -> (Outcome, Decision) {
    let decision = decide(cfg, me, peer, anchored, range_sigma);
    let outcome = match decision {
        Decision::Skip(r) => Outcome::Skipped(r),
        Decision::Use { regime, nvar, .. } => {
            let mean_only = regime == Regime::MeanOnly;
            let pos_only = mean_only && cfg.mean_only_pos_only;
            let accepted =
                me.update_range_masked(peer.p, nvar, measured, range_sigma, mean_only, pos_only);
            Outcome::Applied { regime, accepted }
        }
    };
    (outcome, decision)
}

/// Apply an AoA bearing (unit, my body frame) to `peer`, under the decision
/// [`range_update`] made for the same exchange. AoA is used in the mean-only
/// regime (steering mesh orientation, with fixed — not annealed — damping)
/// and downhill only once I'm roughly localized. Returns accepted components.
pub fn aoa_update(
    cfg: &CoopConfig,
    me: &mut Estimator,
    peer: &PeerView,
    decision: Decision,
    bearing_body: Vec3,
    aoa_sigma: f32,
) -> u32 {
    let Decision::Use { regime, nvar, my_var } = decision else {
        return 0;
    };
    let mean_only = regime == Regime::MeanOnly;
    if !mean_only && my_var >= cfg.aoa_downhill_max_var {
        return 0;
    }
    // Not annealed: annealing AoA lets the whole mesh slowly rotate.
    let aoa_nvar = if mean_only { cfg.mean_only_aoa_damping * my_var } else { nvar };
    me.update_aoa(peer.p, aoa_nvar, bearing_body, aoa_sigma, mean_only)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::Params;
    use crate::math::Quat;

    fn est(var_sigma: f32) -> Estimator {
        let mut e = Estimator::new(Params { gate: 0.0, ..Params::default() });
        e.set_pose(Vec3::ZERO, Vec3::ZERO, Quat::IDENTITY, var_sigma, 0.1, 0.1);
        e
    }

    fn peer(var: f32, lock: f32, age: f32) -> PeerView {
        PeerView { p: Vec3::new(5.0, 0.0, 0.0), var, lock, age }
    }

    #[test]
    fn regimes() {
        let cfg = CoopConfig::default();
        let me = est(10.0); // var 100, lock 10 (cold)
        // Tight, locked peer: downhill, floored at half my variance.
        match decide(&cfg, &me, &peer(1e-4, 1.0, 0.1), false, 0.1) {
            Decision::Use { regime: Regime::Downhill, nvar, .. } => {
                assert!((nvar - (50.0 - 0.01)).abs() < 1e-3)
            }
            d => panic!("{:?}", d),
        }
        // Comparable: mean-only, cold-start damping 0.5 * my_var.
        assert_eq!(
            decide(&cfg, &me, &peer(100.0, 5.0, 0.1), false, 0.1),
            Decision::Use { regime: Regime::MeanOnly, nvar: 50.0, my_var: 100.0 }
        );
        // Stale advert, anchored, cold peer.
        assert_eq!(
            decide(&cfg, &me, &peer(1e-4, 1.0, 1.5), false, 0.1),
            Decision::Skip(SkipReason::StaleAdvert)
        );
        assert_eq!(
            decide(&cfg, &me, &peer(100.0, 5.0, 0.1), true, 0.1),
            Decision::Skip(SkipReason::Anchored)
        );
        let mut locked = est(10.0);
        locked.set_pose(Vec3::ZERO, Vec3::ZERO, Quat::IDENTITY, 10.0, 0.1, 0.1);
        // Drive my lock down with consistent ranges, then a cold peer is refused.
        let anchor = Vec3::new(6.0, 0.0, 0.0);
        for _ in 0..300 {
            let d = locked.p.sub(anchor).norm();
            locked.update_range(anchor, 4.0, d + 0.05, 0.1, true);
        }
        assert!(locked.range_innov_ratio() < 2.0);
        assert_eq!(
            decide(&cfg, &locked, &peer(100.0, 10.0, 0.1), false, 0.1),
            Decision::Skip(SkipReason::ColdPeer)
        );
    }

    #[test]
    fn annealed_mean_only_damping_grows_with_lock() {
        let cfg = CoopConfig::default();
        let mut me = est(10.0);
        let anchor = Vec3::new(6.0, 0.0, 0.0);
        for _ in 0..400 {
            let d = me.p.sub(anchor).norm();
            me.update_range(anchor, 4.0, d + 0.01, 0.1, true);
        }
        // Fully locked: anneal = 1 + (30-0.5)/0.5 = 60 -> nvar = 30 * my_var.
        match decide(&cfg, &me, &peer(100.0, 1.5, 0.1), false, 0.1) {
            Decision::Use { regime: Regime::MeanOnly, nvar, .. } => {
                assert!((nvar / me.pos_var() - 30.0).abs() < 1e-3, "{}", nvar / me.pos_var())
            }
            d => panic!("{:?}", d),
        }
    }
}
