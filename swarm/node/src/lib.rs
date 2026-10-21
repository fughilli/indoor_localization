//! Widget node runtime: the swarm_core estimator + cooperative rules wrapped
//! in what a device needs around them — a peer table fed by pose
//! advertisements, aiding-update timing, surveyed (anchor) mode, and the
//! widget.v1 advertisement wire format. no_std, no allocation: the firmware
//! (apps/node) links it as a static library through the C ABI in ffi.rs; the
//! same crate is unit-tested on the host.
#![cfg_attr(not(test), no_std)]

pub mod advert;
#[cfg(feature = "ffi")]
pub mod ffi;
pub mod node;
pub mod settings;

pub use node::{Node, NodeConfig, RangeResult};
