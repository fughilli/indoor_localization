//! The signed description of a firmware image. It fits in one UWB frame, so
//! a node can check an offer before it accepts a single byte of the image.
//! Signed with ECDSA P-256 (verified by MbedTLS on the device); the public
//! key is compiled into the firmware, the private key stays on the host.

use crate::crypto::{self, P256_PUBLIC_KEY_LEN, P256_SIGNATURE_LEN};

/// version, size, sha256, flags: the signed part.
pub const SIGNED_LEN: usize = 4 + 4 + 32 + 4;
pub const SIGNATURE_LEN: usize = P256_SIGNATURE_LEN;
pub const ENCODED_LEN: usize = SIGNED_LEN + SIGNATURE_LEN;

/// Reserved: the image is a loader, not node firmware.
pub const FLAG_LOADER: u32 = 1 << 0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Manifest {
    /// Monotonic; nodes ignore images not newer than what they have.
    pub version: u32,
    /// Image length in bytes.
    pub size: u32,
    pub sha256: [u8; 32],
    pub flags: u32,
    /// ECDSA P-256 (r || s) over SHA-256 of the first SIGNED_LEN bytes.
    pub signature: [u8; SIGNATURE_LEN],
}

impl Manifest {
    pub fn encode(&self) -> [u8; ENCODED_LEN] {
        let mut out = [0u8; ENCODED_LEN];
        out[..SIGNED_LEN].copy_from_slice(&self.signed_bytes());
        out[SIGNED_LEN..].copy_from_slice(&self.signature);
        out
    }

    pub fn decode(b: &[u8]) -> Option<Manifest> {
        if b.len() < ENCODED_LEN {
            return None;
        }
        let u32_at = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let mut sha256 = [0u8; 32];
        sha256.copy_from_slice(&b[8..40]);
        let mut signature = [0u8; SIGNATURE_LEN];
        signature.copy_from_slice(&b[SIGNED_LEN..ENCODED_LEN]);
        Some(Manifest { version: u32_at(0), size: u32_at(4), sha256, flags: u32_at(40), signature })
    }

    pub fn signed_bytes(&self) -> [u8; SIGNED_LEN] {
        let mut out = [0u8; SIGNED_LEN];
        out[0..4].copy_from_slice(&self.version.to_le_bytes());
        out[4..8].copy_from_slice(&self.size.to_le_bytes());
        out[8..40].copy_from_slice(&self.sha256);
        out[40..44].copy_from_slice(&self.flags.to_le_bytes());
        out
    }

    /// Signature check against the public key built into the firmware.
    pub fn verify(&self, public_key: &[u8; P256_PUBLIC_KEY_LEN]) -> bool {
        crypto::p256_verify(public_key, &self.signed_bytes(), &self.signature)
    }

    /// An unsigned manifest for `image`; sign() fills the signature.
    pub fn for_image(image: &[u8], version: u32, flags: u32) -> Manifest {
        Manifest { version, size: image.len() as u32, sha256: crypto::sha256(image), flags, signature: [0; SIGNATURE_LEN] }
    }

    /// Sign with a P-256 private scalar (host side).
    #[cfg(not(feature = "mbedtls"))]
    pub fn sign(&mut self, secret: &[u8; 32]) {
        self.signature = crypto::signing::p256_sign(secret, &self.signed_bytes());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crypto::signing::p256_public_key;

    const SECRET: [u8; 32] = [7; 32];

    #[test]
    fn signed_manifest_round_trips_and_verifies() {
        let mut m = Manifest::for_image(b"firmware bytes", 3, 0);
        m.sign(&SECRET);
        let d = Manifest::decode(&m.encode()).unwrap();
        assert_eq!(d, m);
        assert!(d.verify(&p256_public_key(&SECRET)));
        assert_eq!(ENCODED_LEN, 108, "one mesh payload");
    }

    #[test]
    fn tampering_or_wrong_key_fails() {
        let mut m = Manifest::for_image(b"firmware bytes", 3, 0);
        m.sign(&SECRET);
        let pk = p256_public_key(&SECRET);
        let mut bumped = m;
        bumped.version = 4;
        assert!(!bumped.verify(&pk));
        let mut other = m;
        other.sha256[0] ^= 1;
        assert!(!other.verify(&pk));
        assert!(!m.verify(&p256_public_key(&[8; 32])));
    }
}
