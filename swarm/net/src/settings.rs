//! Live settings over the mesh (docs/swarm-network.md "Live settings"):
//! layered key/value storage, authenticated commands, and swarm-layer
//! catch-up. Keys are 16-bit ids, values 4 raw bytes; what they mean (names,
//! types, ranges, where they apply) is the firmware's settings registry.
//!
//! - **Swarm layer**: a whole snapshot, tagged by the host (4-byte HMAC over
//!   generation + body, crypto.rs) and broadcast in fragments. Nodes keep
//!   the tagged blob verbatim, so they can hand it to a neighbour that
//!   missed it; the neighbour checks the same tag.
//! - **Device overrides**: unicast tagged commands with an increasing
//!   sequence number (replay protection), bound to the destination id.
//!
//! Without a provisioned key a node accepts no remote settings (the USB
//! path still works: physical access is trusted).

use crate::crypto::{self, DOMAIN_BUNDLE, DOMAIN_OVERRIDE, HMAC_KEY_LEN, TAG_LEN};
use crate::mesh::{BROADCAST, MAX_PAYLOAD};

pub const MAX_KEYS: usize = 96;
/// key u16 + value u32.
pub const ENTRY_LEN: usize = 6;
/// count u16 + entries.
pub const MAX_LAYER: usize = 2 + MAX_KEYS * ENTRY_LEN;
/// Tagged swarm bundle: generation u32 + layer + tag.
pub const MAX_BUNDLE: usize = 4 + MAX_LAYER + TAG_LEN;

/// Mesh ports.
pub const PORT_BUNDLE_DATA: u8 = 2;
pub const PORT_BUNDLE_REQUEST: u8 = 3;
pub const PORT_OVERRIDE: u8 = 4;

pub const CMD_SET: u8 = 1;
/// Clear the listed keys; no keys clears every override.
pub const CMD_CLEAR: u8 = 2;
const CMD_HEAD: usize = 1 + 4 + 1; // kind, seq, count
/// Entries per override command: what fits next to the tag.
pub const CMD_MAX_ENTRIES: usize = (MAX_PAYLOAD - CMD_HEAD - TAG_LEN) / ENTRY_LEN;

/// Fragment: generation u32 + bundle length u16 + offset u16 + bytes.
const FRAG_HEAD: usize = 8;
pub const FRAG_DATA: usize = MAX_PAYLOAD - FRAG_HEAD;
// Fragments of a bundle are tracked in a u32 bitmap.
const _: () = assert!(MAX_BUNDLE.div_ceil(FRAG_DATA) <= 32);


#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry {
    pub key: u16,
    pub value: u32,
}

/// One layer: a sparse key -> value map, kept sorted by key.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layer {
    pub entries: [Entry; MAX_KEYS],
    pub len: usize,
}

impl Default for Layer {
    fn default() -> Layer {
        Layer { entries: [Entry { key: 0, value: 0 }; MAX_KEYS], len: 0 }
    }
}

impl Layer {
    pub fn get(&self, key: u16) -> Option<u32> {
        self.slice().binary_search_by_key(&key, |e| e.key).ok().map(|i| self.entries[i].value)
    }

    pub fn slice(&self) -> &[Entry] {
        &self.entries[..self.len]
    }

    /// False if the layer is full.
    pub fn set(&mut self, key: u16, value: u32) -> bool {
        match self.slice().binary_search_by_key(&key, |e| e.key) {
            Ok(i) => {
                self.entries[i].value = value;
                true
            }
            Err(i) => {
                if self.len == MAX_KEYS {
                    return false;
                }
                self.entries.copy_within(i..self.len, i + 1);
                self.entries[i] = Entry { key, value };
                self.len += 1;
                true
            }
        }
    }

    pub fn remove(&mut self, key: u16) {
        if let Ok(i) = self.slice().binary_search_by_key(&key, |e| e.key) {
            self.entries.copy_within(i + 1..self.len, i);
            self.len -= 1;
        }
    }

    /// count u16 + (key u16, value u32)*, little-endian.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let n = 2 + self.len * ENTRY_LEN;
        if out.len() < n {
            return None;
        }
        out[0..2].copy_from_slice(&(self.len as u16).to_le_bytes());
        for (i, e) in self.slice().iter().enumerate() {
            let at = 2 + i * ENTRY_LEN;
            out[at..at + 2].copy_from_slice(&e.key.to_le_bytes());
            out[at + 2..at + 6].copy_from_slice(&e.value.to_le_bytes());
        }
        Some(n)
    }

    pub fn decode(b: &[u8]) -> Option<Layer> {
        if b.len() < 2 {
            return None;
        }
        let n = u16::from_le_bytes([b[0], b[1]]) as usize;
        if n > MAX_KEYS || b.len() < 2 + n * ENTRY_LEN {
            return None;
        }
        let mut l = Layer::default();
        for i in 0..n {
            let at = 2 + i * ENTRY_LEN;
            let key = u16::from_le_bytes([b[at], b[at + 1]]);
            let value = u32::from_le_bytes([b[at + 2], b[at + 3], b[at + 4], b[at + 5]]);
            if !l.set(key, value) {
                return None;
            }
        }
        Some(l)
    }
}


/// Host side (and the USB path): a tagged swarm bundle. Returns the length.
pub fn make_bundle(layer: &Layer, generation: u32, key: &[u8; HMAC_KEY_LEN], out: &mut [u8; MAX_BUNDLE]) -> usize {
    out[0..4].copy_from_slice(&generation.to_le_bytes());
    let n = 4 + layer.encode(&mut out[4..4 + MAX_LAYER]).unwrap();
    let tag = crypto::tag(key, DOMAIN_BUNDLE, &[&out[..n]]).unwrap();
    out[n..n + TAG_LEN].copy_from_slice(&tag);
    n + TAG_LEN
}

/// Host side: a tagged override command for `dest`.
pub fn make_override(dest: u16, kind: u8, seq: u32, entries: &[Entry], key: &[u8; HMAC_KEY_LEN]) -> Option<([u8; MAX_PAYLOAD], usize)> {
    if entries.len() > CMD_MAX_ENTRIES {
        return None;
    }
    let mut out = [0u8; MAX_PAYLOAD];
    let n = CMD_HEAD + entries.len() * ENTRY_LEN;
    out[0] = kind;
    out[1..5].copy_from_slice(&seq.to_le_bytes());
    out[5] = entries.len() as u8;
    for (i, e) in entries.iter().enumerate() {
        let at = CMD_HEAD + i * ENTRY_LEN;
        out[at..at + 2].copy_from_slice(&e.key.to_le_bytes());
        out[at + 2..at + 6].copy_from_slice(&e.value.to_le_bytes());
    }
    // The tag covers the destination too: a command for one node can't be
    // replayed at another.
    let tag = crypto::tag(key, DOMAIN_OVERRIDE, &[&dest.to_le_bytes(), &out[..n]])?;
    out[n..n + TAG_LEN].copy_from_slice(&tag);
    Some((out, n + TAG_LEN))
}

/// What changed after handling a packet; the firmware re-resolves its
/// configuration and persists the named layer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Changes {
    pub swarm: bool,
    pub overrides: bool,
}

/// Something for the firmware to send on the mesh.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outgoing {
    pub dest: u16,
    pub port: u8,
    pub hop_limit: u8,
    pub len: usize,
    pub payload: [u8; MAX_PAYLOAD],
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SettingsStats {
    pub bundles_applied: u32,
    pub overrides_applied: u32,
    pub rejected_tag: u32,
    pub rejected_replay: u32,
    pub no_key: u32,
    pub served: u32,
}

/// Per-node settings state and sync logic.
pub struct Settings {
    pub me: u16,
    key: Option<[u8; HMAC_KEY_LEN]>,
    /// The tagged swarm bundle as received, and its length (0: none).
    bundle: [u8; MAX_BUNDLE],
    bundle_len: usize,
    pub swarm: Layer,
    pub overrides: Layer,
    /// Highest override command sequence accepted.
    pub override_seq: u32,
    /// Bundle being reassembled: generation, length, bytes, fragments seen.
    incoming: Option<(u32, usize, [u8; MAX_BUNDLE], u32)>,
    /// Serving our bundle to neighbours: next fragment index.
    serve_next: Option<usize>,
    last_request_us: i64,
    pub stats: SettingsStats,
}

impl Settings {
    pub fn new(me: u16, key: Option<[u8; HMAC_KEY_LEN]>) -> Settings {
        Settings {
            me,
            key,
            bundle: [0; MAX_BUNDLE],
            bundle_len: 0,
            swarm: Layer::default(),
            overrides: Layer::default(),
            override_seq: 0,
            incoming: None,
            serve_next: None,
            last_request_us: i64::MIN / 2,
            stats: SettingsStats::default(),
        }
    }

    pub fn set_key(&mut self, key: Option<[u8; HMAC_KEY_LEN]>) {
        self.key = key;
    }

    pub fn has_key(&self) -> bool {
        self.key.is_some()
    }

    /// Swarm-layer generation (0: none), for the summary in POLL frames.
    pub fn generation(&self) -> u32 {
        if self.bundle_len < 4 {
            return 0;
        }
        u32::from_le_bytes([self.bundle[0], self.bundle[1], self.bundle[2], self.bundle[3]])
    }

    /// Effective raw value: override, else swarm, else None (default).
    pub fn get(&self, key: u16) -> Option<u32> {
        self.overrides.get(key).or_else(|| self.swarm.get(key))
    }

    // ---- persistence (the firmware stores these bytes in LittleFS)

    /// The tagged bundle verbatim.
    pub fn bundle(&self) -> &[u8] {
        &self.bundle[..self.bundle_len]
    }

    /// Restore a saved bundle; its tag is checked again (flash isn't
    /// trusted, and a re-provisioned key invalidates old bundles).
    pub fn load_bundle(&mut self, b: &[u8]) -> bool {
        self.accept_bundle(b, true)
    }

    /// override_seq u32 + layer.
    pub fn save_overrides(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < 4 {
            return None;
        }
        out[..4].copy_from_slice(&self.override_seq.to_le_bytes());
        Some(4 + self.overrides.encode(&mut out[4..])?)
    }

    pub fn load_overrides(&mut self, b: &[u8]) -> bool {
        if b.len() < 4 {
            return false;
        }
        let Some(l) = Layer::decode(&b[4..]) else { return false };
        self.override_seq = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        self.overrides = l;
        true
    }

    // ---- local entry points (USB: trusted)

    /// A tagged bundle from the host: apply it, then serve it to the swarm.
    pub fn host_bundle(&mut self, b: &[u8]) -> Changes {
        if self.accept_bundle(b, false) {
            self.serve_next = Some(0);
            Changes { swarm: true, overrides: false }
        } else {
            Changes::default()
        }
    }

    /// Set (Some) or clear (None) one override on this node directly.
    pub fn local_override(&mut self, key: u16, value: Option<u32>) -> Changes {
        match value {
            Some(v) => {
                if !self.overrides.set(key, v) {
                    return Changes::default();
                }
            }
            None => self.overrides.remove(key),
        }
        Changes { swarm: false, overrides: true }
    }

    // ---- mesh

    /// A delivered mesh packet on one of the settings ports.
    pub fn on_packet(&mut self, dest: u16, port: u8, payload: &[u8]) -> Changes {
        match port {
            PORT_BUNDLE_DATA => self.fragment(payload),
            PORT_BUNDLE_REQUEST => {
                // A neighbour is behind: serve our bundle locally.
                if self.bundle_len > 0 && self.serve_next.is_none() {
                    self.serve_next = Some(0);
                    self.stats.served += 1;
                }
                Changes::default()
            }
            PORT_OVERRIDE if dest == self.me => self.override_command(payload),
            _ => Changes::default(),
        }
    }

    /// A neighbour's summary (from its POLL): if it has a newer swarm layer,
    /// ask it, at most once a second.
    pub fn neighbour(&mut self, t_us: i64, id: u16, generation: u32) -> Option<Outgoing> {
        if self.key.is_none() || generation <= self.generation() || t_us - self.last_request_us < 1_000_000 {
            return None;
        }
        self.last_request_us = t_us;
        let mut payload = [0u8; MAX_PAYLOAD];
        payload[..4].copy_from_slice(&generation.to_le_bytes());
        Some(Outgoing { dest: id, port: PORT_BUNDLE_REQUEST, hop_limit: 0, len: 4, payload })
    }

    /// Next fragment to send while serving the bundle. Local broadcasts
    /// (hop limit 0): every neighbour that needs it takes it, and anyone
    /// further away asks its own neighbours.
    pub fn next_outgoing(&mut self) -> Option<Outgoing> {
        let k = self.serve_next?;
        let len = self.bundle_len;
        let off = k * FRAG_DATA;
        if off >= len {
            self.serve_next = None;
            return None;
        }
        let n = (len - off).min(FRAG_DATA);
        let mut payload = [0u8; MAX_PAYLOAD];
        payload[0..4].copy_from_slice(&self.bundle[0..4]);
        payload[4..6].copy_from_slice(&(len as u16).to_le_bytes());
        payload[6..8].copy_from_slice(&(off as u16).to_le_bytes());
        payload[FRAG_HEAD..FRAG_HEAD + n].copy_from_slice(&self.bundle[off..off + n]);
        self.serve_next = if off + n < len { Some(k + 1) } else { None };
        Some(Outgoing { dest: BROADCAST, port: PORT_BUNDLE_DATA, hop_limit: 0, len: FRAG_HEAD + n, payload })
    }

    fn fragment(&mut self, p: &[u8]) -> Changes {
        if p.len() < FRAG_HEAD {
            return Changes::default();
        }
        let generation = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
        let len = u16::from_le_bytes([p[4], p[5]]) as usize;
        let off = u16::from_le_bytes([p[6], p[7]]) as usize;
        let data = &p[FRAG_HEAD..];
        if generation <= self.generation() || len > MAX_BUNDLE || off % FRAG_DATA != 0 || off + data.len() > len {
            return Changes::default();
        }
        if !matches!(self.incoming, Some((g, l, _, _)) if g == generation && l == len) {
            self.incoming = Some((generation, len, [0; MAX_BUNDLE], 0));
        }
        let (_, _, buf, have) = self.incoming.as_mut().unwrap();
        buf[off..off + data.len()].copy_from_slice(data);
        *have |= 1 << (off / FRAG_DATA);
        if *have != (1u32 << len.div_ceil(FRAG_DATA)) - 1 {
            return Changes::default();
        }
        let (_, len, buf, _) = self.incoming.take().unwrap();
        if self.accept_bundle(&buf[..len], false) {
            Changes { swarm: true, overrides: false }
        } else {
            Changes::default()
        }
    }

    /// Check and adopt a tagged bundle. `reload`: from our own storage at
    /// boot (any generation; it's what we had).
    fn accept_bundle(&mut self, b: &[u8], reload: bool) -> bool {
        let Some(key) = self.key else {
            self.stats.no_key += 1;
            return false;
        };
        if b.len() < 4 + 2 + TAG_LEN || b.len() > MAX_BUNDLE {
            return false;
        }
        let (body, tag) = b.split_at(b.len() - TAG_LEN);
        if !crypto::check_tag(&key, DOMAIN_BUNDLE, &[body], tag) {
            self.stats.rejected_tag += 1;
            return false;
        }
        let generation = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        if !reload && generation <= self.generation() {
            return false;
        }
        let Some(layer) = Layer::decode(&body[4..]) else { return false };
        self.bundle[..b.len()].copy_from_slice(b);
        self.bundle_len = b.len();
        self.swarm = layer;
        self.stats.bundles_applied += 1;
        true
    }

    fn override_command(&mut self, p: &[u8]) -> Changes {
        let Some(key) = self.key else {
            self.stats.no_key += 1;
            return Changes::default();
        };
        if p.len() < CMD_HEAD + TAG_LEN {
            return Changes::default();
        }
        let kind = p[0];
        let seq = u32::from_le_bytes([p[1], p[2], p[3], p[4]]);
        let count = p[5] as usize;
        let n = CMD_HEAD + count * ENTRY_LEN;
        if count > CMD_MAX_ENTRIES || p.len() < n + TAG_LEN {
            return Changes::default();
        }
        if !crypto::check_tag(&key, DOMAIN_OVERRIDE, &[&self.me.to_le_bytes(), &p[..n]], &p[n..n + TAG_LEN]) {
            self.stats.rejected_tag += 1;
            return Changes::default();
        }
        if seq <= self.override_seq {
            self.stats.rejected_replay += 1;
            return Changes::default();
        }
        self.override_seq = seq;
        let entry = |i: usize| {
            let at = CMD_HEAD + i * ENTRY_LEN;
            Entry {
                key: u16::from_le_bytes([p[at], p[at + 1]]),
                value: u32::from_le_bytes([p[at + 2], p[at + 3], p[at + 4], p[at + 5]]),
            }
        };
        match kind {
            CMD_SET => {
                for i in 0..count {
                    let e = entry(i);
                    self.overrides.set(e.key, e.value);
                }
            }
            CMD_CLEAR if count == 0 => self.overrides = Layer::default(),
            CMD_CLEAR => {
                for i in 0..count {
                    self.overrides.remove(entry(i).key);
                }
            }
            _ => return Changes::default(),
        }
        self.stats.overrides_applied += 1;
        Changes { swarm: false, overrides: true }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: [u8; HMAC_KEY_LEN] = [42; HMAC_KEY_LEN];

    fn layer(entries: &[(u16, u32)]) -> Layer {
        let mut l = Layer::default();
        for &(k, v) in entries {
            l.set(k, v);
        }
        l
    }

    fn bundle(l: &Layer, generation: u32, key: &[u8; HMAC_KEY_LEN]) -> Vec<u8> {
        let mut b = [0u8; MAX_BUNDLE];
        let n = make_bundle(l, generation, key, &mut b);
        b[..n].to_vec()
    }

    /// Run `from`'s serving queue into `to` (local broadcast, no loss).
    fn serve(from: &mut Settings, to: &mut Settings) -> Changes {
        let mut c = Changes::default();
        while let Some(o) = from.next_outgoing() {
            let r = to.on_packet(o.dest, o.port, &o.payload[..o.len]);
            c.swarm |= r.swarm;
            c.overrides |= r.overrides;
        }
        c
    }

    #[test]
    fn layer_is_sorted_and_round_trips() {
        let l = layer(&[(30, 3), (10, 1), (20, 2), (10, 9)]);
        assert_eq!(l.slice().iter().map(|e| e.key).collect::<Vec<_>>(), vec![10, 20, 30]);
        assert_eq!(l.get(10), Some(9));
        let mut buf = [0u8; MAX_LAYER];
        let n = l.encode(&mut buf).unwrap();
        assert_eq!(Layer::decode(&buf[..n]).unwrap(), l);
    }

    /// The host's bundle enters at one node, which serves it to a neighbour;
    /// a node that was off catches up from that neighbour later.
    #[test]
    fn swarm_bundle_spreads_and_late_joiner_catches_up() {
        let mut a = Settings::new(1, Some(KEY));
        let mut b = Settings::new(2, Some(KEY));
        let l = layer((0..60u16).map(|k| (k, k as u32 * 7)).collect::<Vec<_>>().as_slice());
        assert_eq!(a.host_bundle(&bundle(&l, 5, &KEY)), Changes { swarm: true, overrides: false });
        assert_eq!(a.generation(), 5);
        assert!(serve(&mut a, &mut b).swarm);
        assert_eq!(b.swarm, l);

        let mut c = Settings::new(3, Some(KEY));
        let req = c.neighbour(0, 2, b.generation()).expect("asks for the newer layer");
        b.on_packet(req.dest, req.port, &req.payload[..req.len]);
        assert!(serve(&mut b, &mut c).swarm);
        assert_eq!(c.get(59), Some(413));
        assert!(c.neighbour(10_000_000, 2, b.generation()).is_none(), "up to date: no request");

        // Reboot: the stored bundle reloads (and still checks out).
        let mut d = Settings::new(3, Some(KEY));
        assert!(d.load_bundle(c.bundle()));
        assert_eq!(d.generation(), 5);
    }

    #[test]
    fn wrong_key_stale_or_tampered_bundles_are_ignored() {
        let mut a = Settings::new(1, Some(KEY));
        assert!(!a.host_bundle(&bundle(&layer(&[(1, 1)]), 5, &[9; HMAC_KEY_LEN])).swarm, "wrong key");
        assert!(a.host_bundle(&bundle(&layer(&[(1, 1)]), 5, &KEY)).swarm);
        assert!(!a.host_bundle(&bundle(&layer(&[(1, 2)]), 5, &KEY)).swarm, "same generation");
        let mut t = bundle(&layer(&[(1, 3)]), 6, &KEY);
        t[6] ^= 1;
        assert!(!a.host_bundle(&t).swarm, "tampered");
        assert_eq!(a.get(1), Some(1));
        let mut keyless = Settings::new(2, None);
        assert!(!keyless.host_bundle(&bundle(&layer(&[(1, 1)]), 7, &KEY)).swarm, "no key provisioned");
    }

    #[test]
    fn overrides_need_tag_right_destination_and_fresh_seq() {
        let mut n = Settings::new(7, Some(KEY));
        n.host_bundle(&bundle(&layer(&[(1, 10), (2, 20)]), 1, &KEY));

        let (p, len) = make_override(7, CMD_SET, 100, &[Entry { key: 1, value: 11 }], &KEY).unwrap();
        assert!(n.on_packet(7, PORT_OVERRIDE, &p[..len]).overrides);
        assert_eq!(n.get(1), Some(11), "override wins");
        assert_eq!(n.get(2), Some(20), "swarm value elsewhere");

        assert!(!n.on_packet(7, PORT_OVERRIDE, &p[..len]).overrides, "replay");
        let (p8, len8) = make_override(8, CMD_SET, 200, &[Entry { key: 2, value: 0 }], &KEY).unwrap();
        assert!(!n.on_packet(7, PORT_OVERRIDE, &p8[..len8]).overrides, "tagged for node 8");
        let (pb, lenb) = make_override(7, CMD_SET, 300, &[Entry { key: 2, value: 0 }], &[1; HMAC_KEY_LEN]).unwrap();
        assert!(!n.on_packet(7, PORT_OVERRIDE, &pb[..lenb]).overrides, "wrong key");

        let (pc, lenc) = make_override(7, CMD_CLEAR, 101, &[Entry { key: 1, value: 0 }], &KEY).unwrap();
        assert!(n.on_packet(7, PORT_OVERRIDE, &pc[..lenc]).overrides);
        assert_eq!(n.get(1), Some(10), "back to the swarm value");

        let mut buf = [0u8; 4 + MAX_LAYER];
        let k = n.save_overrides(&mut buf).unwrap();
        let mut m = Settings::new(7, Some(KEY));
        assert!(m.load_overrides(&buf[..k]));
        assert_eq!(m.override_seq, 101);
    }

    /// The same bytes swarm/tools/swarm_wire.py produces (its test pins
    /// them too): host and node agree on the encoding and the tag.
    #[test]
    fn matches_host_tool_encoding() {
        let key = [5u8; HMAC_KEY_LEN];
        let mut b = [0u8; MAX_BUNDLE];
        let n = make_bundle(&layer(&[(106, 7.0f32.to_bits()), (300, 0.2f32.to_bits())]), 3, &key, &mut b);
        assert_eq!(hex(&b[..n]), "0300000002006a000000e0402c01cdcc4c3e6e1e97e2");
        let (o, len) = make_override(13, CMD_SET, 1, &[Entry { key: 300, value: 0.2f32.to_bits() }], &key).unwrap();
        assert_eq!(hex(&o[..len]), "0101000000012c01cdcc4c3e402c9832");
    }

    fn hex(b: &[u8]) -> String {
        b.iter().map(|x| format!("{:02x}", x)).collect()
    }

    #[test]
    fn capacities() {
        assert_eq!(CMD_MAX_ENTRIES, 16);
        assert_eq!(MAX_BUNDLE.div_ceil(FRAG_DATA), 6);
    }
}
