//! Station-free cooperative localization core.
//!
//! Each widget runs one [`Estimator`]: an error-state Kalman filter over
//! position / velocity / attitude / IMU biases, propagated by raw IMU samples
//! and corrected by UWB range+AoA measurements to neighbor widgets (whose own
//! pose estimates are uncertain), plus optional absolute inputs (GPS,
//! altimeter) and magnetometer heading.
//!
//! Constraints: no external crates, no allocation in the hot path, f32-first
//! math — the same code must run on the embedded target.

pub mod estimator;
pub mod ffi;
pub mod math;

pub use estimator::Estimator;
