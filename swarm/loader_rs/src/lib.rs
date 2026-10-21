//! The loader's Rust staticlib: swarm_net (mesh, settings, firmware
//! distribution, with its C ABI) and the bare-metal panic handler. The node
//! firmware gets the same through swarm_node instead.
#![no_std]

pub use swarm_net;

/// A panic resets the chip through the C runtime's abort().
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    extern "C" {
        fn abort() -> !;
    }
    unsafe { abort() }
}
