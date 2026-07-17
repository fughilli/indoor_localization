//! On-device parser for the WINT sensor-intrinsics blob (authoring pipeline
//! and layout: swarm/tools/intrinsics.py). Intrinsics are optional per-unit
//! sensor-quality parameters measured by //swarm/tools:characterize; absent
//! sections/fields (NaN) mean "use the firmware default".
//!
//! Sectioned little-endian container, forward-compatible (unknown section
//! kinds are skipped):
//!
//!   magic 'WINT' | version u16 | section_count u16
//!   per section: kind u8 | pad u8 | payload_len u16 | payload f32[]
//!   crc32 (IEEE) u32 over everything before it

use crate::extrinsics::{crc32, ExtrinsicsError};
use crate::math::Vec3;

pub const MAGIC: [u8; 4] = *b"WINT";
pub const VERSION: u16 = 1;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuIntrinsics {
    pub accel_noise_density: f32,
    pub gyro_noise_density: f32,
    pub accel_bias_instability: f32,
    pub gyro_bias_instability: f32,
    pub gyro_bias: Vec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UwbRangeIntrinsics {
    pub bias0_m: f32,
    pub scale_ppm: f32,
    pub sigma_m: f32,
    pub outlier_frac: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UwbAoaIntrinsics {
    /// AoA measurement plane normal in the UWB *sensor* frame; compose with
    /// the UWB extrinsic rotation for the widget frame.
    pub plane_normal: Vec3,
    pub sign: f32,
    pub offset_rad: f32,
    pub sigma_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MagIntrinsics {
    pub hard_iron: Vec3,
    /// Row-major unit-determinant soft-iron correction.
    pub soft_iron: [[f32; 3]; 3],
    pub sigma: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GpsIntrinsics {
    pub sigma_h_m: f32,
    pub sigma_v_m: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AltIntrinsics {
    pub sigma_m: f32,
}

/// All sections optional; individual fields inside a present section may be
/// NaN, also meaning "not characterized — use the default".
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WidgetIntrinsics {
    pub imu: Option<ImuIntrinsics>,
    pub uwb_range: Option<UwbRangeIntrinsics>,
    pub uwb_aoa: Option<UwbAoaIntrinsics>,
    pub mag: Option<MagIntrinsics>,
    pub gps: Option<GpsIntrinsics>,
    pub alt: Option<AltIntrinsics>,
}

fn f(b: &[u8], i: usize) -> f32 {
    if 4 * i + 4 <= b.len() {
        f32::from_le_bytes([b[4 * i], b[4 * i + 1], b[4 * i + 2], b[4 * i + 3]])
    } else {
        f32::NAN
    }
}

fn v3(b: &[u8], i: usize) -> Vec3 {
    Vec3::new(f(b, i), f(b, i + 1), f(b, i + 2))
}

pub fn parse_blob(blob: &[u8]) -> Result<WidgetIntrinsics, ExtrinsicsError> {
    if blob.len() < 12 {
        return Err(ExtrinsicsError::TooShort);
    }
    if blob[0..4] != MAGIC {
        return Err(ExtrinsicsError::BadMagic);
    }
    if u16::from_le_bytes([blob[4], blob[5]]) != VERSION {
        return Err(ExtrinsicsError::BadVersion);
    }
    let body = &blob[..blob.len() - 4];
    let stored = u32::from_le_bytes([
        blob[blob.len() - 4],
        blob[blob.len() - 3],
        blob[blob.len() - 2],
        blob[blob.len() - 1],
    ]);
    if crc32(body) != stored {
        return Err(ExtrinsicsError::BadCrc);
    }
    let count = u16::from_le_bytes([blob[6], blob[7]]) as usize;
    let mut out = WidgetIntrinsics::default();
    let mut at = 8;
    for _ in 0..count {
        if at + 4 > body.len() {
            return Err(ExtrinsicsError::LengthMismatch);
        }
        let kind = blob[at];
        let plen = u16::from_le_bytes([blob[at + 2], blob[at + 3]]) as usize;
        if at + 4 + plen > body.len() {
            return Err(ExtrinsicsError::LengthMismatch);
        }
        let p = &blob[at + 4..at + 4 + plen];
        at += 4 + plen;
        match kind {
            1 => {
                out.imu = Some(ImuIntrinsics {
                    accel_noise_density: f(p, 0),
                    gyro_noise_density: f(p, 1),
                    accel_bias_instability: f(p, 2),
                    gyro_bias_instability: f(p, 3),
                    gyro_bias: v3(p, 4),
                })
            }
            2 => {
                out.uwb_range = Some(UwbRangeIntrinsics {
                    bias0_m: f(p, 0),
                    scale_ppm: f(p, 1),
                    sigma_m: f(p, 2),
                    outlier_frac: f(p, 3),
                })
            }
            3 => {
                out.uwb_aoa = Some(UwbAoaIntrinsics {
                    plane_normal: v3(p, 0),
                    sign: f(p, 3),
                    offset_rad: f(p, 4),
                    sigma_rad: f(p, 5),
                })
            }
            4 => {
                out.mag = Some(MagIntrinsics {
                    hard_iron: v3(p, 0),
                    soft_iron: [
                        [f(p, 3), f(p, 4), f(p, 5)],
                        [f(p, 6), f(p, 7), f(p, 8)],
                        [f(p, 9), f(p, 10), f(p, 11)],
                    ],
                    sigma: f(p, 12),
                })
            }
            5 => {
                out.gps = Some(GpsIntrinsics { sigma_h_m: f(p, 0), sigma_v_m: f(p, 1) })
            }
            6 => out.alt = Some(AltIntrinsics { sigma_m: f(p, 0) }),
            _ => {} // unknown section: skip (forward compatibility)
        }
    }
    if at != body.len() {
        return Err(ExtrinsicsError::LengthMismatch);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn golden_blob() -> Vec<u8> {
        // Generated by //swarm/tools:example_intrinsics_blob from
        // examples/widget_v1_intrinsics.json (Python encodes, Rust parses).
        let path = format!(
            "{}/_main/swarm/tools/examples/widget_v1_intrinsics.wint",
            std::env::var("TEST_SRCDIR").unwrap()
        );
        std::fs::read(path).unwrap()
    }

    #[test]
    fn parses_python_generated_blob() {
        let w = parse_blob(&golden_blob()).unwrap();
        let imu = w.imu.unwrap();
        assert!((imu.accel_noise_density - 0.005).abs() < 1e-7);
        assert!((imu.gyro_bias.y + 0.02).abs() < 1e-6);
        let rng = w.uwb_range.unwrap();
        assert!((rng.bias0_m - 0.05).abs() < 1e-6);
        assert!((rng.scale_ppm - 200.0).abs() < 1e-3);
        let aoa = w.uwb_aoa.unwrap();
        assert!((aoa.plane_normal.z - 1.0).abs() < 1e-6);
        assert!((aoa.offset_rad - 0.3).abs() < 1e-6);
        let mag = w.mag.unwrap();
        assert!((mag.hard_iron.x - 12.0).abs() < 1e-4);
        assert!((mag.soft_iron[1][1] - 0.875).abs() < 1e-5);
        assert!((w.gps.unwrap().sigma_h_m - 1.5).abs() < 1e-6);
        assert!((w.alt.unwrap().sigma_m - 0.3).abs() < 1e-6);
    }

    #[test]
    fn rejects_corruption_and_skips_unknown_sections() {
        let mut blob = golden_blob();
        let n = blob.len();
        blob[n - 6] ^= 0xFF;
        assert_eq!(parse_blob(&blob), Err(ExtrinsicsError::BadCrc));

        // Hand-build a blob with only an unknown section kind 99.
        let mut b = Vec::new();
        b.extend_from_slice(b"WINT");
        b.extend_from_slice(&VERSION.to_le_bytes());
        b.extend_from_slice(&1u16.to_le_bytes());
        b.extend_from_slice(&[99u8, 0]);
        b.extend_from_slice(&8u16.to_le_bytes());
        b.extend_from_slice(&1.0f32.to_le_bytes());
        b.extend_from_slice(&2.0f32.to_le_bytes());
        let crc = crc32(&b);
        b.extend_from_slice(&crc.to_le_bytes());
        let w = parse_blob(&b).unwrap();
        assert_eq!(w, WidgetIntrinsics::default());
    }
}
