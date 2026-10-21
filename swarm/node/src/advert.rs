//! The over-the-air pose advertisement (design doc §5.2): the
//! `widget.v1.PoseAdvertisement` protobuf, encoded/decoded with the same
//! micropb bindings the rest of the widget config uses.

use micropb::{MessageDecode, MessageEncode, PbEncoder};
use swarm_core::math::Vec3;
use widget_pb::widget_::v1_ as pb;

pub const FLAG_ANCHORED: u32 = 1 << 0;
pub const FLAG_STATIC: u32 = 1 << 1;
pub const FLAG_GPS: u32 = 1 << 2;

/// Largest encoded advertisement: 2 varints (<=5 B each + tag), 2 nested
/// Vec3f (2 + 3*5 B each), 2 fixed32 (5 B each), flags varint. ~60 B.
pub const MAX_ENCODED: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Advert {
    pub widget_id: u32,
    pub seq: u32,
    pub p: Vec3,
    pub v: Vec3,
    /// Mean position variance, m^2.
    pub quality: f32,
    pub flags: u32,
    /// Range-innovation lock ratio.
    pub lock: f32,
}

fn to_pb(v: Vec3) -> pb::Vec3f {
    let mut out = pb::Vec3f::default();
    out.r#x = v.x;
    out.r#y = v.y;
    out.r#z = v.z;
    out
}

fn from_pb(v: Option<&pb::Vec3f>) -> Vec3 {
    v.map(|v| Vec3::new(v.r#x, v.r#y, v.r#z)).unwrap_or(Vec3::ZERO)
}

/// Encode into `out`; None if it doesn't fit.
pub fn encode(a: &Advert, out: &mut [u8]) -> Option<usize> {
    let mut m = pb::PoseAdvertisement::default();
    m.r#widget_id = a.widget_id;
    m.r#seq = a.seq;
    m.set_p(to_pb(a.p));
    m.set_v(to_pb(a.v));
    m.r#quality = a.quality;
    m.r#flags = a.flags;
    m.r#lock = a.lock;
    let mut enc = PbEncoder::new(micropb::heapless::Vec::<u8, MAX_ENCODED>::new());
    m.encode(&mut enc).ok()?;
    let bytes = enc.into_writer();
    let n = bytes.len();
    out.get_mut(..n)?.copy_from_slice(&bytes);
    Some(n)
}

/// Decode; None on malformed input. Absent fields take proto3 defaults; an
/// absent lock (an older sender) reads as "no lock".
pub fn decode(bytes: &[u8]) -> Option<Advert> {
    let mut m = pb::PoseAdvertisement::default();
    m.decode_from_bytes(bytes).ok()?;
    Some(Advert {
        widget_id: m.r#widget_id,
        seq: m.r#seq,
        p: from_pb(m.r#p()),
        v: from_pb(m.r#v()),
        quality: m.r#quality,
        flags: m.r#flags,
        lock: if m.r#lock > 0.0 { m.r#lock } else { swarm_core::estimator::INNOV_RATIO_MAX },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_fits_one_frame() {
        let a = Advert {
            widget_id: 0xC248,
            seq: 70000,
            p: Vec3::new(1.5, -2.25, 0.75),
            v: Vec3::new(0.1, 0.0, -0.3),
            quality: 0.42,
            flags: FLAG_ANCHORED | FLAG_STATIC,
            lock: 1.3,
        };
        let mut buf = [0u8; MAX_ENCODED];
        let n = encode(&a, &mut buf).unwrap();
        assert!(n <= 60, "{} bytes", n);
        assert_eq!(decode(&buf[..n]).unwrap(), a);
        assert!(encode(&a, &mut [0u8; 8]).is_none(), "must refuse a short buffer");
        assert!(decode(&[0xff, 0xff, 0xff]).is_none());
    }
}
