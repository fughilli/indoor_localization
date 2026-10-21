//! One node's network stack: mesh + settings sync + firmware distribution,
//! wired together the way the firmware's radio task drives them. The radio
//! hands over every network frame it hears and, in its own TDMA slot, asks
//! for frames to send; everything else (which port goes where, what to
//! request, what to serve, what to report) happens here, so a whole swarm
//! of these can be tested on the host.

use crate::crypto::{HMAC_KEY_LEN, P256_PUBLIC_KEY_LEN};
use crate::manifest::{Manifest, ENCODED_LEN as MANIFEST_LEN};
use crate::mesh::{Event, Mesh, MeshConfig, BROADCAST, MAX_PACKET, MAX_PAYLOAD};
use crate::settings::{self, Changes, Layer, Settings, ENTRY_LEN};
use crate::update::{self, Flash, Status, Summary, Updater, REQUEST_LEN};

/// Mesh ports beyond the settings ones (settings.rs: 2, 3, 4).
pub const PORT_SETTINGS_GET: u8 = 5;
pub const PORT_SETTINGS_REPORT: u8 = 6;
pub const PORT_FW_MANIFEST: u8 = 7;
/// Ports from here up are the firmware's own (delivered as AppPacket).
pub const PORT_APP: u8 = 16;

/// Summary carried in every POLL: firmware progress + settings generation.
pub const SUMMARY_LEN: usize = update::SUMMARY_LEN + 4;

/// What kind of link frame `next_tx` produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameKind {
    /// A mesh packet; link-layer broadcast.
    Mesh,
    /// A firmware page request, link-layer unicast to `dest`.
    FwRequest { dest: u16 },
    /// A firmware data chunk; link-layer broadcast.
    FwData,
}

/// Report packet: part u8, parts u8, generation u32, override_seq u32,
/// then entries (key | source << 14, value).
const REPORT_HEAD: usize = 10;
const REPORT_ENTRIES: usize = (MAX_PAYLOAD - REPORT_HEAD) / ENTRY_LEN;
pub const SOURCE_SWARM: u16 = 1;
pub const SOURCE_OVERRIDE: u16 = 2;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NetChanges {
    pub settings: Changes,
    /// A verified, newer firmware image is staged.
    pub fw_ready: bool,
}

/// A packet for the firmware itself (port >= PORT_APP), or a settings
/// report for the host.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AppPacket {
    pub origin: u16,
    pub dest: u16,
    pub port: u8,
    pub hops: u8,
    pub rssi: f32,
    pub len: usize,
    pub payload: [u8; MAX_PAYLOAD],
}

const APP_QUEUE: usize = 8;

pub struct Net {
    pub mesh: Mesh,
    pub settings: Settings,
    pub updater: Updater,
    changes: NetChanges,
    app: [Option<AppPacket>; APP_QUEUE],
    /// Pending settings report: (to, next part).
    report: Option<(u16, u8)>,
}

impl Net {
    pub fn new(me: u16, hmac_key: Option<[u8; HMAC_KEY_LEN]>, fw_key: [u8; P256_PUBLIC_KEY_LEN], running: u32, seed: u32) -> Net {
        Net {
            mesh: Mesh::new(me, MeshConfig::default(), seed),
            settings: Settings::new(me, hmac_key),
            updater: Updater::new(me, fw_key, running),
            changes: NetChanges::default(),
            app: [None; APP_QUEUE],
            report: None,
        }
    }

    pub fn me(&self) -> u16 {
        self.mesh.me
    }

    /// What changed since the last call (settings to apply/persist, an
    /// image ready to install).
    pub fn take_changes(&mut self) -> NetChanges {
        let c = self.changes;
        self.changes = NetChanges::default();
        c
    }

    pub fn next_app(&mut self) -> Option<AppPacket> {
        let i = self.app.iter().position(|a| a.is_some())?;
        let a = self.app[i].take();
        // Keep arrival order.
        self.app[i..].rotate_left(1);
        a
    }

    pub fn summary(&self) -> [u8; SUMMARY_LEN] {
        let mut b = [0u8; SUMMARY_LEN];
        b[..update::SUMMARY_LEN].copy_from_slice(&self.updater.summary().encode());
        b[update::SUMMARY_LEN..].copy_from_slice(&self.settings.generation().to_le_bytes());
        b
    }

    // ---- from the radio

    /// A neighbour's POLL summary.
    pub fn on_summary(&mut self, t_us: i64, from: u16, b: &[u8]) {
        if b.len() < SUMMARY_LEN {
            return;
        }
        if let Some(s) = Summary::decode(b) {
            self.updater.on_summary(t_us, from, s);
        }
        let generation = u32::from_le_bytes([b[12], b[13], b[14], b[15]]);
        if let Some(o) = self.settings.neighbour(t_us, from, generation) {
            self.mesh.send_with_hops(o.dest, o.port, &o.payload[..o.len], false, o.hop_limit);
        }
    }

    pub fn on_mesh(&mut self, t_us: i64, bytes: &[u8], rssi: f32, flash: &mut dyn Flash) {
        self.mesh.on_rx(bytes, rssi);
        self.drain_mesh(t_us, flash);
    }

    pub fn on_fw_request(&mut self, t_us: i64, payload: &[u8]) {
        self.updater.on_request_at(t_us, payload);
    }

    pub fn on_fw_data(&mut self, t_us: i64, payload: &[u8], flash: &mut dyn Flash) {
        let was = self.updater.status();
        self.updater.on_data(t_us, payload, flash);
        self.note_fw(was);
    }

    /// Start of this node's TDMA slot.
    pub fn on_my_slot(&mut self) {
        self.mesh.on_my_slot();
    }

    /// Housekeeping: retries, manifest offers, settings serving, reports.
    pub fn poll(&mut self, t_us: i64, flash: &mut dyn Flash) {
        self.mesh.poll(t_us);
        self.drain_mesh(t_us, flash);
        if let Some(m) = self.updater.manifest_offer(t_us) {
            self.mesh.send_with_hops(BROADCAST, PORT_FW_MANIFEST, &m.encode(), false, 0);
        }
        while self.mesh.can_send() {
            let Some(o) = self.settings.next_outgoing() else { break };
            self.mesh.send_with_hops(o.dest, o.port, &o.payload[..o.len], false, o.hop_limit);
        }
        self.pump_report();
    }

    /// Next frame for this node's slot: mesh first (control traffic is
    /// small and time-critical), then our firmware request, then data
    /// we're serving.
    pub fn next_tx(&mut self, t_us: i64, out: &mut [u8], flash: &mut dyn Flash) -> Option<(FrameKind, usize)> {
        if let Some(n) = self.mesh.next_tx(t_us, out) {
            return Some((FrameKind::Mesh, n));
        }
        if let Some((dest, req)) = self.updater.next_request(t_us) {
            if out.len() >= REQUEST_LEN {
                out[..REQUEST_LEN].copy_from_slice(&req);
                return Some((FrameKind::FwRequest { dest }, REQUEST_LEN));
            }
        }
        self.updater.next_data(out, flash).map(|n| (FrameKind::FwData, n))
    }

    // ---- from the host (node on USB)

    pub fn host_bundle(&mut self, b: &[u8]) -> bool {
        let c = self.settings.host_bundle(b);
        self.merge(c);
        c.swarm
    }

    pub fn local_override(&mut self, key: u16, value: Option<u32>) {
        let c = self.settings.local_override(key, value);
        self.merge(c);
    }

    /// A signed manifest from the host: adopt it. It isn't offered to the
    /// swarm until this node runs it and validates it (canary rule, see
    /// Updater::servable).
    pub fn host_manifest(&mut self, b: &[u8], flash: &mut dyn Flash) -> bool {
        let Some(m) = Manifest::decode(b) else { return false };
        if self.updater.manifest() == Some(m) {
            return true; // the host resuming an interrupted push
        }
        let was = self.updater.status();
        if !self.updater.offer(m, flash) {
            return false;
        }
        self.note_fw(was);
        true
    }

    pub fn host_chunk(&mut self, t_us: i64, page: u16, chunk: u8, data: &[u8], flash: &mut dyn Flash) {
        let was = self.updater.status();
        self.updater.host_chunk(t_us, page, chunk, data, flash);
        self.note_fw(was);
    }

    /// Queue a mesh packet the host built (e.g. a tagged override for
    /// another node, or a settings GET). Returns the packet id.
    pub fn send(&mut self, dest: u16, port: u8, payload: &[u8], want_ack: bool) -> Option<u16> {
        self.mesh.send(dest, port, payload, want_ack)
    }

    // ----

    fn drain_mesh(&mut self, t_us: i64, flash: &mut dyn Flash) {
        while let Some(e) = self.mesh.next_event() {
            let Event::Deliver { packet, rssi, hops } = e else { continue };
            let h = packet.header;
            let p = packet.payload();
            match h.port {
                settings::PORT_BUNDLE_DATA | settings::PORT_BUNDLE_REQUEST | settings::PORT_OVERRIDE => {
                    let c = self.settings.on_packet(h.dest, h.port, p);
                    self.merge(c);
                }
                PORT_SETTINGS_GET if h.dest == self.me() => self.report = Some((h.origin, 0)),
                PORT_FW_MANIFEST if p.len() >= MANIFEST_LEN => {
                    if let Some(m) = Manifest::decode(p) {
                        let was = self.updater.status();
                        if self.updater.offer(m, flash) {
                            // Pass it on (the mesh relay already floods
                            // global offers; this covers local ones).
                            let _ = t_us;
                        }
                        self.note_fw(was);
                    }
                }
                port if port >= PORT_APP || port == PORT_SETTINGS_REPORT => {
                    let mut a = AppPacket {
                        origin: h.origin,
                        dest: h.dest,
                        port,
                        hops,
                        rssi,
                        len: p.len(),
                        payload: [0; MAX_PAYLOAD],
                    };
                    a.payload[..p.len()].copy_from_slice(p);
                    if let Some(slot) = self.app.iter().position(|x| x.is_none()) {
                        self.app[slot] = Some(a);
                    }
                }
                _ => {}
            }
        }
    }

    /// Queue report parts while the mesh outbox has room.
    fn pump_report(&mut self) {
        while let Some((to, part)) = self.report {
            if !self.mesh.can_send() {
                return;
            }
            let entries = report_entries(&self.settings.swarm, &self.settings.overrides);
            let total = entries.1.max(1).div_ceil(REPORT_ENTRIES) as u8;
            let mut p = [0u8; MAX_PAYLOAD];
            p[0] = part;
            p[1] = total;
            p[2..6].copy_from_slice(&self.settings.generation().to_le_bytes());
            p[6..10].copy_from_slice(&self.settings.override_seq.to_le_bytes());
            let start = part as usize * REPORT_ENTRIES;
            let end = (start + REPORT_ENTRIES).min(entries.1);
            for (i, (k, v)) in entries.0[start..end].iter().enumerate() {
                let at = REPORT_HEAD + i * ENTRY_LEN;
                p[at..at + 2].copy_from_slice(&k.to_le_bytes());
                p[at + 2..at + 6].copy_from_slice(&v.to_le_bytes());
            }
            self.mesh.send(to, PORT_SETTINGS_REPORT, &p[..REPORT_HEAD + (end - start) * ENTRY_LEN], false);
            self.report = if part + 1 < total { Some((to, part + 1)) } else { None };
        }
    }

    fn merge(&mut self, c: Changes) {
        self.changes.settings.swarm |= c.swarm;
        self.changes.settings.overrides |= c.overrides;
    }

    fn note_fw(&mut self, was: Status) {
        if self.updater.status() == Status::Ready && was != Status::Ready {
            self.changes.fw_ready = true;
        }
    }
}

/// (key | source << 14, value) for both layers; overrides shadow nothing
/// here (the host sees both).
fn report_entries(swarm: &Layer, overrides: &Layer) -> ([(u16, u32); 2 * settings::MAX_KEYS], usize) {
    let mut out = [(0u16, 0u32); 2 * settings::MAX_KEYS];
    let mut n = 0;
    for e in swarm.slice() {
        out[n] = (e.key | SOURCE_SWARM << 14, e.value);
        n += 1;
    }
    for e in overrides.slice() {
        out[n] = (e.key | SOURCE_OVERRIDE << 14, e.value);
        n += 1;
    }
    (out, n)
}

/// Copy of a frame buffer size the radio needs.
pub const MAX_FRAME_PAYLOAD: usize = if MAX_PACKET > update::DATA_MAX { MAX_PACKET } else { update::DATA_MAX };

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crypto::signing::p256_public_key;
    use crate::settings::{make_bundle, make_override, Entry, CMD_SET, MAX_BUNDLE, PORT_OVERRIDE};
    use crate::update::PAGE_SIZE;

    const HMAC: [u8; HMAC_KEY_LEN] = [5; HMAC_KEY_LEN];
    const FW_SECRET: [u8; 32] = [6; 32];

    struct RamFlash(Vec<u8>);
    impl Flash for RamFlash {
        fn capacity(&self) -> u32 {
            self.0.len() as u32
        }
        fn erase(&mut self, o: u32, l: u32) -> bool {
            self.0[o as usize..(o + l) as usize].fill(0xFF);
            true
        }
        fn write(&mut self, o: u32, d: &[u8]) -> bool {
            self.0[o as usize..o as usize + d.len()].copy_from_slice(d);
            true
        }
        fn read(&mut self, o: u32, b: &mut [u8]) -> bool {
            b.copy_from_slice(&self.0[o as usize..o as usize + b.len()]);
            true
        }
    }

    struct Node {
        net: Net,
        flash: RamFlash,
    }

    fn swarm(n: usize) -> Vec<Node> {
        (0..n)
            .map(|i| Node {
                net: Net::new(10 + i as u16, Some(HMAC), p256_public_key(&FW_SECRET), 1, 77 + i as u32),
                flash: RamFlash(vec![0xFF; 32 * PAGE_SIZE]),
            })
            .collect()
    }

    /// Everyone hears everyone; each node in its slot sends its POLL
    /// summary and up to 3 network frames.
    fn run(nodes: &mut [Node], superframes: usize) {
        let mut t = 0i64;
        for _ in 0..superframes {
            for i in 0..nodes.len() {
                t += 5_000;
                let me = nodes[i].net.me();
                let s = nodes[i].net.summary();
                for j in 0..nodes.len() {
                    if j != i {
                        nodes[j].net.on_summary(t, me, &s);
                    }
                }
                {
                    let Node { net, flash } = &mut nodes[i];
                    net.on_my_slot();
                    net.poll(t, flash);
                }
                for _ in 0..3 {
                    let mut buf = [0u8; MAX_FRAME_PAYLOAD];
                    let Node { net, flash } = &mut nodes[i];
                    let Some((kind, n)) = net.next_tx(t, &mut buf, flash) else { break };
                    for j in 0..nodes.len() {
                        if j == i {
                            continue;
                        }
                        let Node { net, flash } = &mut nodes[j];
                        match kind {
                            FrameKind::Mesh => net.on_mesh(t, &buf[..n], -70.0, flash),
                            FrameKind::FwData => net.on_fw_data(t, &buf[..n], flash),
                            FrameKind::FwRequest { dest } if dest == net.me() => net.on_fw_request(t, &buf[..n]),
                            FrameKind::FwRequest { .. } => {}
                        }
                    }
                }
            }
            for x in nodes.iter_mut() {
                let Node { net, flash } = x;
                net.poll(t, flash);
                // Simulated install of a ready image, then a healthy boot.
                if net.updater.status() == Status::Ready {
                    net.updater.running = net.updater.manifest().unwrap().version;
                    net.updater.set_validated(true);
                }
            }
        }
    }

    #[test]
    fn settings_bundle_override_and_report_end_to_end() {
        let mut nodes = swarm(4);
        let mut l = Layer::default();
        l.set(106, 7.0f32.to_bits());
        let mut b = [0u8; MAX_BUNDLE];
        let n = make_bundle(&l, 3, &HMAC, &mut b);
        assert!(nodes[0].net.host_bundle(&b[..n]));
        run(&mut nodes, 20);
        for x in nodes.iter_mut() {
            assert_eq!(x.net.settings.get(106), Some(7.0f32.to_bits()));
        }
        assert!(nodes[2].net.take_changes().settings.swarm);

        // The host (via node 0) overrides one setting on node 13 only.
        let (p, len) = make_override(13, CMD_SET, 1, &[Entry { key: 300, value: 0.2f32.to_bits() }], &HMAC).unwrap();
        nodes[0].net.send(13, PORT_OVERRIDE, &p[..len], true);
        run(&mut nodes, 20);
        assert_eq!(nodes[3].net.settings.get(300), Some(0.2f32.to_bits()));
        assert_eq!(nodes[2].net.settings.get(300), None);

        // ... and reads node 13's layers back.
        nodes[0].net.send(13, PORT_SETTINGS_GET, &[], false);
        run(&mut nodes, 20);
        let r = nodes[0].net.next_app().expect("report");
        assert_eq!((r.origin, r.port), (13, PORT_SETTINGS_REPORT));
        assert_eq!(r.len, REPORT_HEAD + 2 * ENTRY_LEN, "one swarm + one override entry");
    }

    #[test]
    fn firmware_enters_at_one_node_and_reaches_all() {
        let mut nodes = swarm(4);
        let img: Vec<u8> = (0..5 * PAGE_SIZE + 77).map(|i| (i % 253) as u8).collect();
        let mut m = Manifest::for_image(&img, 2, 0);
        m.sign(&FW_SECRET);
        {
            let Node { net, flash } = &mut nodes[0];
            assert!(net.host_manifest(&m.encode(), flash));
            for (page, p) in img.chunks(PAGE_SIZE).enumerate() {
                for (c, d) in p.chunks(update::CHUNK).enumerate() {
                    net.host_chunk(0, page as u16, c as u8, d, flash);
                }
            }
            assert!(net.take_changes().fw_ready);
            // Install, boot healthy: now it may serve.
            net.updater.running = 2;
            net.updater.set_validated(true);
        }
        run(&mut nodes, 2000);
        for (i, x) in nodes.iter_mut().enumerate() {
            assert_eq!(x.net.updater.status(), Status::Serving, "node {}: {:?}", i, x.net.updater.summary());
        }
    }
}
