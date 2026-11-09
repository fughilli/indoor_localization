//! Swarm network (docs/swarm-network.md): mesh packets with managed
//! flooding, authenticated settings, signed firmware manifests, and the
//! page-by-page firmware distribution protocol. no_std, no allocation: the
//! loader and node firmware link it through the C ABI in ffi.rs and own the
//! radio and the flash; every format is a swarm.v1 protobuf message (wire.rs,
//! docs/swarm-wire.md); this crate decides what to store, what to ask
//! neighbours for and what to send them. Host tests run whole swarms of it
//! over a lossy channel.
#![cfg_attr(not(test), no_std)]

pub mod crypto;
pub mod manifest;
pub mod mesh;
pub mod net;
pub mod settings;
pub mod update;
pub mod wire;
#[cfg(test)]
mod budget;
#[cfg(test)]
mod golden;
#[cfg(feature = "ffi")]
pub mod ffi;

pub use manifest::Manifest;
pub use mesh::{Mesh, MeshConfig};
pub use update::{Flash, Status, Updater};
