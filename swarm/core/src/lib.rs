//! Station-free cooperative localization core.
//!
//! Each widget runs one [`Estimator`]: an error-state Kalman filter over
//! position / velocity / attitude / IMU biases, propagated by raw IMU samples
//! and corrected by UWB range+AoA measurements to neighbor widgets (whose own
//! pose estimates are uncertain), plus optional absolute inputs (GPS,
//! altimeter) and magnetometer heading.
//!
//! Constraints: no allocation, f32-first math, `no_std` — the same code runs
//! on the embedded target (the classic ESP32 via the esp-rs Xtensa toolchain).
//! The one dependency is `libm`, used on host and device alike so the
//! simulation and the firmware run bit-identical float math.
#![cfg_attr(not(test), no_std)]

pub mod coop;
pub mod estimator;
#[cfg(feature = "ffi")]
pub mod ffi;
pub mod math;

pub use estimator::Estimator;
