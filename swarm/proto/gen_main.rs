//! micropb code-generation driver for widget.v1: FileDescriptorSet ->
//! no_std Rust. Invoked by the @firmware//proto:micropb.bzl macro; this file
//! is the single home of the heapless container capacity table.

use micropb_gen::{Config, Generator};

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 4 || !matches!(args[3].as_str(), "host" | "firmware") {
        eprintln!(
            "usage: {} <descriptor-set.bin> <out.rs> host|firmware",
            args[0]
        );
        std::process::exit(2);
    }
    // widget.v1 is small, pure config traffic: one profile fits both sides;
    // the host/firmware distinction is kept for CLI parity with the macro.

    let mut g = Generator::new();
    g.use_container_heapless();

    // Strings: widget_model / widget_serial / sensor names / axis tokens.
    g.configure(".", Config::new().max_bytes(32).max_len(4));
    // A widget carries the 5 standard sensors + room for custom ones.
    g.configure(
        ".widget.v1.Extrinsics.sensors",
        Config::new().max_len(16),
    );
    // Row-major 3x3 soft-iron matrix.
    g.configure(
        ".widget.v1.MagIntrinsics.soft_iron",
        Config::new().max_len(9),
    );

    g.compile_fdset_file(&args[1], &args[2])
}
