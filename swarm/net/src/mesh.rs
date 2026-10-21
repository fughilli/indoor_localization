//! Mesh packets with Meshtastic-style managed flooding (docs/swarm-network.md
//! "Mesh packets"). Link-agnostic: the radio hands over every mesh frame it
//! hears (with its signal strength) and, in its own TDMA slot, asks for the
//! next frame to send.
//!
//! - Duplicates are dropped by (origin, id, attempt): an origin's retry is a
//!   new attempt that relays carry again, while the destination delivers each
//!   (origin, id) once (and re-acks repeats, in case its ack was lost).
//! - A packet is relayed (hop limit permitting) after a delay of 0..=window
//!   of this node's slots; the *weaker* the signal it arrived on, the sooner,
//!   since distant relays extend coverage most. Overhearing another node
//!   relay it first cancels ours.
//! - Unicast with want-ack: the destination answers with an ack packet; the
//!   origin retries until it hears it (or gives up).

pub const BROADCAST: u16 = 0xFFFF;
pub const HEADER_LEN: usize = 9;
/// 127-byte frame - 2 FCS - 8 link header - 9 mesh header: exactly room
/// for a signed manifest (108 bytes).
pub const MAX_PAYLOAD: usize = 108;
pub const MAX_PACKET: usize = HEADER_LEN + MAX_PAYLOAD;

pub const FLAG_WANT_ACK: u8 = 1 << 0;
pub const FLAG_IS_ACK: u8 = 1 << 1;
/// The high nibble of the flags byte is the attempt number.
const ATTEMPT_SHIFT: u8 = 4;

/// Port of ack packets (payload: the acked id, u16 LE).
pub const PORT_ACK: u8 = 0;

const SEEN_LEN: usize = 64;
const DELIVERED_LEN: usize = 32;
const RELAY_LEN: usize = 8;
const OUTBOX_LEN: usize = 8;
const EVENTS_LEN: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshConfig {
    /// Hop limit given to packets this node originates.
    pub hop_limit: u8,
    /// Relay delay window, in this node's own slots.
    pub relay_window: u32,
    /// Signal strength (dBm) at/above which a relay waits the full window
    /// and at/below which it goes in the first slot.
    pub rssi_strong: f32,
    pub rssi_weak: f32,
    pub ack_timeout_us: i64,
    /// Transmissions of a want-ack packet before giving up.
    pub max_tries: u8,
}

impl Default for MeshConfig {
    fn default() -> MeshConfig {
        MeshConfig {
            hop_limit: 3,
            relay_window: 3,
            rssi_strong: -60.0,
            rssi_weak: -90.0,
            ack_timeout_us: 1_000_000,
            max_tries: 3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub origin: u16,
    pub id: u16,
    pub dest: u16,
    pub hop_limit: u8,
    pub hop_start: u8,
    /// FLAG_* bits.
    pub flags: u8,
    /// Retry number of the origin's transmission (0..=15).
    pub attempt: u8,
    pub port: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Packet {
    pub header: Header,
    pub len: u8,
    pub payload: [u8; MAX_PAYLOAD],
}

impl Packet {
    pub fn new(header: Header, payload: &[u8]) -> Option<Packet> {
        if payload.len() > MAX_PAYLOAD {
            return None;
        }
        let mut p = Packet { header, len: payload.len() as u8, payload: [0; MAX_PAYLOAD] };
        p.payload[..payload.len()].copy_from_slice(payload);
        Some(p)
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.len as usize]
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let n = HEADER_LEN + self.len as usize;
        if out.len() < n {
            return None;
        }
        let h = &self.header;
        out[0..2].copy_from_slice(&h.origin.to_le_bytes());
        out[2..4].copy_from_slice(&h.id.to_le_bytes());
        out[4..6].copy_from_slice(&h.dest.to_le_bytes());
        out[6] = (h.hop_limit & 0x0F) | (h.hop_start << 4);
        out[7] = (h.flags & 0x0F) | (h.attempt << ATTEMPT_SHIFT);
        out[8] = h.port;
        out[HEADER_LEN..n].copy_from_slice(self.payload());
        Some(n)
    }

    pub fn decode(b: &[u8]) -> Option<Packet> {
        if b.len() < HEADER_LEN || b.len() > MAX_PACKET {
            return None;
        }
        let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let header = Header {
            origin: u16_at(0),
            id: u16_at(2),
            dest: u16_at(4),
            hop_limit: b[6] & 0x0F,
            hop_start: b[6] >> 4,
            flags: b[7] & 0x0F,
            attempt: b[7] >> ATTEMPT_SHIFT,
            port: b[8],
        };
        Packet::new(header, &b[HEADER_LEN..])
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    /// A packet for this node (unicast to it, or broadcast). `rssi` of the
    /// copy that arrived first; `hops` it took.
    Deliver { packet: Packet, rssi: f32, hops: u8 },
    /// A want-ack packet this node sent was acknowledged.
    Acked { dest: u16, id: u16 },
    /// ... or ran out of tries.
    Failed { dest: u16, id: u16 },
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeshStats {
    pub originated: u32,
    pub delivered: u32,
    pub relayed: u32,
    /// Relays dropped because another node relayed first.
    pub relays_cancelled: u32,
    pub duplicates: u32,
    pub retries: u32,
    pub acked: u32,
    pub failed: u32,
    pub dropped_full: u32,
}

#[derive(Clone, Copy)]
struct Relay {
    packet: Packet,
    due_slot: u32,
}

#[derive(Clone, Copy)]
struct Pending {
    packet: Packet,
    tries: u8,
    next_us: i64,
}

pub struct Mesh {
    pub me: u16,
    pub cfg: MeshConfig,
    pub stats: MeshStats,
    next_id: u16,
    seen: [(u16, u16, u8); SEEN_LEN],
    seen_n: usize,
    seen_at: usize,
    delivered: [(u16, u16); DELIVERED_LEN],
    delivered_n: usize,
    delivered_at: usize,
    relays: [Option<Relay>; RELAY_LEN],
    outbox: [Option<Pending>; OUTBOX_LEN],
    events: [Option<Event>; EVENTS_LEN],
    ev_head: usize,
    ev_len: usize,
    slot: u32,
    rng: u32,
}

impl Mesh {
    /// `seed` varies the first packet id across reboots (so a rebooted node
    /// isn't taken for a duplicate of its old self).
    pub fn new(me: u16, cfg: MeshConfig, seed: u32) -> Mesh {
        Mesh {
            me,
            cfg,
            stats: MeshStats::default(),
            next_id: seed as u16,
            seen: [(0, 0, 0); SEEN_LEN],
            seen_n: 0,
            seen_at: 0,
            delivered: [(0, 0); DELIVERED_LEN],
            delivered_n: 0,
            delivered_at: 0,
            relays: [None; RELAY_LEN],
            outbox: [None; OUTBOX_LEN],
            events: [None; EVENTS_LEN],
            ev_head: 0,
            ev_len: 0,
            slot: 0,
            rng: seed | 1,
        }
    }

    /// Queue a packet from this node. Returns its id, or None if the outbox
    /// is full or the payload too long.
    pub fn send(&mut self, dest: u16, port: u8, payload: &[u8], want_ack: bool) -> Option<u16> {
        self.send_with_hops(dest, port, payload, want_ack, self.cfg.hop_limit)
    }

    /// send() with an explicit hop limit; 0 keeps a packet to direct
    /// neighbours (local serving traffic that every hop redoes anyway).
    pub fn send_with_hops(&mut self, dest: u16, port: u8, payload: &[u8], want_ack: bool, hop_limit: u8) -> Option<u16> {
        let slot = self.outbox.iter().position(|o| o.is_none())?;
        self.next_id = self.next_id.wrapping_add(1);
        let id = self.next_id;
        let header = Header {
            origin: self.me,
            id,
            dest,
            hop_limit,
            hop_start: hop_limit,
            flags: if want_ack && dest != BROADCAST { FLAG_WANT_ACK } else { 0 },
            attempt: 0,
            port,
        };
        let packet = Packet::new(header, payload)?;
        self.outbox[slot] = Some(Pending { packet, tries: 0, next_us: i64::MIN });
        self.stats.originated += 1;
        Some(id)
    }

    /// Room in the outbox for another packet?
    pub fn can_send(&self) -> bool {
        self.outbox.iter().any(|o| o.is_none())
    }

    /// A mesh frame heard off the air.
    pub fn on_rx(&mut self, bytes: &[u8], rssi: f32) {
        let Some(p) = Packet::decode(bytes) else { return };
        let h = p.header;
        if h.origin == self.me {
            return; // our own packet, relayed back
        }
        if self.is_seen(h.origin, h.id, h.attempt) {
            self.stats.duplicates += 1;
            // Someone else relayed it: ours is redundant.
            for r in self.relays.iter_mut() {
                if matches!(r, Some(r) if r.packet.header.origin == h.origin
                    && r.packet.header.id == h.id && r.packet.header.attempt == h.attempt)
                {
                    *r = None;
                    self.stats.relays_cancelled += 1;
                }
            }
            return;
        }
        self.remember(h.origin, h.id, h.attempt);
        let for_me = h.dest == self.me || h.dest == BROADCAST;
        if for_me {
            if h.flags & FLAG_IS_ACK != 0 && h.dest == self.me {
                if p.len >= 2 {
                    let acked = u16::from_le_bytes([p.payload[0], p.payload[1]]);
                    self.ack_received(h.origin, acked);
                }
            } else {
                if !self.was_delivered(h.origin, h.id) {
                    self.mark_delivered(h.origin, h.id);
                    self.stats.delivered += 1;
                    self.push_event(Event::Deliver {
                        packet: p,
                        rssi,
                        hops: h.hop_start.saturating_sub(h.hop_limit),
                    });
                }
                // Ack every attempt: the origin retrying means it missed ours.
                if h.flags & FLAG_WANT_ACK != 0 && h.dest == self.me {
                    self.send_ack(h.origin, h.id);
                }
            }
        }
        if h.dest != self.me && h.hop_limit > 0 {
            self.schedule_relay(p, rssi);
        }
    }

    /// Call at the start of each of this node's TDMA slots.
    pub fn on_my_slot(&mut self) {
        self.slot = self.slot.wrapping_add(1);
    }

    /// The next frame to send in this slot, encoded into `out`; None when
    /// nothing is due. Relays go first: they're time-critical for coverage.
    pub fn next_tx(&mut self, t_us: i64, out: &mut [u8]) -> Option<usize> {
        let slot = self.slot;
        if let Some(i) = self
            .relays
            .iter()
            .position(|r| matches!(r, Some(r) if r.due_slot.wrapping_sub(slot) as i32 <= 0))
        {
            let r = self.relays[i].take().unwrap();
            self.stats.relayed += 1;
            return r.packet.encode(out);
        }
        for i in 0..OUTBOX_LEN {
            let Some(mut o) = self.outbox[i] else { continue };
            if o.next_us > t_us {
                continue;
            }
            o.packet.header.attempt = o.tries.min(15);
            let h = o.packet.header;
            self.remember(h.origin, h.id, h.attempt);
            let n = o.packet.encode(out);
            if o.tries > 0 {
                self.stats.retries += 1;
            }
            o.tries += 1;
            if o.packet.header.flags & FLAG_WANT_ACK != 0 {
                o.next_us = t_us + self.cfg.ack_timeout_us;
                self.outbox[i] = Some(o);
            } else {
                self.outbox[i] = None;
            }
            return n;
        }
        None
    }

    /// Expire want-ack packets that used up their tries. Call regularly.
    pub fn poll(&mut self, t_us: i64) {
        for i in 0..OUTBOX_LEN {
            let Some(o) = self.outbox[i] else { continue };
            if o.packet.header.flags & FLAG_WANT_ACK != 0 && o.tries >= self.cfg.max_tries && t_us >= o.next_us {
                self.outbox[i] = None;
                self.stats.failed += 1;
                let h = o.packet.header;
                self.push_event(Event::Failed { dest: h.dest, id: h.id });
            }
        }
    }

    pub fn next_event(&mut self) -> Option<Event> {
        if self.ev_len == 0 {
            return None;
        }
        let e = self.events[self.ev_head].take();
        self.ev_head = (self.ev_head + 1) % EVENTS_LEN;
        self.ev_len -= 1;
        e
    }

    /// Anything queued to send (originated or relayed)?
    pub fn has_tx(&self, t_us: i64) -> bool {
        let slot = self.slot;
        self.relays.iter().flatten().any(|r| r.due_slot.wrapping_sub(slot) as i32 <= 0)
            || self.outbox.iter().flatten().any(|o| o.next_us <= t_us)
    }

    fn send_ack(&mut self, to: u16, id: u16) {
        let Some(slot) = self.outbox.iter().position(|o| o.is_none()) else {
            self.stats.dropped_full += 1;
            return;
        };
        self.next_id = self.next_id.wrapping_add(1);
        let header = Header {
            origin: self.me,
            id: self.next_id,
            dest: to,
            hop_limit: self.cfg.hop_limit,
            hop_start: self.cfg.hop_limit,
            flags: FLAG_IS_ACK,
            attempt: 0,
            port: PORT_ACK,
        };
        let packet = Packet::new(header, &id.to_le_bytes()).unwrap();
        self.outbox[slot] = Some(Pending { packet, tries: 0, next_us: i64::MIN });
    }

    fn ack_received(&mut self, from: u16, id: u16) {
        for i in 0..OUTBOX_LEN {
            if matches!(self.outbox[i], Some(o) if o.packet.header.dest == from && o.packet.header.id == id) {
                self.outbox[i] = None;
                self.stats.acked += 1;
                self.push_event(Event::Acked { dest: from, id });
            }
        }
    }

    fn schedule_relay(&mut self, mut p: Packet, rssi: f32) {
        let Some(slot) = self.relays.iter().position(|r| r.is_none()) else {
            self.stats.dropped_full += 1;
            return;
        };
        p.header.hop_limit -= 1;
        let span = self.cfg.rssi_strong - self.cfg.rssi_weak;
        let frac = if span > 0.0 { ((rssi - self.cfg.rssi_weak) / span).clamp(0.0, 1.0) } else { 0.0 };
        // Strong signal -> later; plus a little jitter so equals don't tie.
        let base = (frac * self.cfg.relay_window as f32 + 0.5) as u32;
        let jitter = self.rand() % 2;
        let delay = (base + jitter).min(self.cfg.relay_window);
        self.relays[slot] = Some(Relay { packet: p, due_slot: self.slot.wrapping_add(delay) });
    }

    fn is_seen(&self, origin: u16, id: u16, attempt: u8) -> bool {
        self.seen[..self.seen_n].iter().any(|&s| s == (origin, id, attempt))
    }

    fn remember(&mut self, origin: u16, id: u16, attempt: u8) {
        self.seen[self.seen_at] = (origin, id, attempt);
        self.seen_at = (self.seen_at + 1) % SEEN_LEN;
        self.seen_n = (self.seen_n + 1).min(SEEN_LEN);
    }

    fn was_delivered(&self, origin: u16, id: u16) -> bool {
        self.delivered[..self.delivered_n].iter().any(|&d| d == (origin, id))
    }

    fn mark_delivered(&mut self, origin: u16, id: u16) {
        self.delivered[self.delivered_at] = (origin, id);
        self.delivered_at = (self.delivered_at + 1) % DELIVERED_LEN;
        self.delivered_n = (self.delivered_n + 1).min(DELIVERED_LEN);
    }

    fn push_event(&mut self, e: Event) {
        if self.ev_len == EVENTS_LEN {
            // Drop the oldest: a reader that falls behind loses history,
            // not the newest news.
            self.ev_head = (self.ev_head + 1) % EVENTS_LEN;
            self.ev_len -= 1;
            self.stats.dropped_full += 1;
        }
        let at = (self.ev_head + self.ev_len) % EVENTS_LEN;
        self.events[at] = Some(e);
        self.ev_len += 1;
    }

    fn rand(&mut self) -> u32 {
        // xorshift32
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A swarm on a TDMA schedule over a lossy, range-limited channel. Nodes
    /// take turns (one slot each per superframe) and send up to
    /// `frames_per_slot` frames in their slot.
    pub(crate) struct Net {
        pub nodes: Vec<Mesh>,
        /// links[i][j]: rssi if j hears i.
        pub links: Vec<Vec<Option<f32>>>,
        pub loss: f32,
        pub t_us: i64,
        pub airtime: u32,
        pub delivered: Vec<Vec<Event>>,
        rng: u64,
    }

    impl Net {
        pub fn new(n: usize, links: Vec<Vec<Option<f32>>>, loss: f32) -> Net {
            Net {
                nodes: (0..n).map(|i| Mesh::new(i as u16 + 1, MeshConfig::default(), 100 + i as u32)).collect(),
                links,
                loss,
                t_us: 0,
                airtime: 0,
                delivered: vec![Vec::new(); n],
                rng: 0x9E37_79B9_7F4A_7C15,
            }
        }

        /// Everyone hears everyone at `rssi`.
        pub fn full(n: usize, rssi: f32, loss: f32) -> Net {
            let links = (0..n).map(|i| (0..n).map(|j| if i == j { None } else { Some(rssi) }).collect()).collect();
            Net::new(n, links, loss)
        }

        /// A line: each node hears only its neighbours.
        pub fn line(n: usize, loss: f32) -> Net {
            let links = (0..n)
                .map(|i| (0..n).map(|j| if (i as i64 - j as i64).abs() == 1 { Some(-80.0) } else { None }).collect())
                .collect();
            Net::new(n, links, loss)
        }

        fn lost(&mut self) -> bool {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 7;
            self.rng ^= self.rng << 17;
            ((self.rng >> 11) as f64 / (1u64 << 53) as f64) < self.loss as f64
        }

        pub fn superframe(&mut self, frames_per_slot: usize) {
            let n = self.nodes.len();
            for i in 0..n {
                self.t_us += 5_000;
                self.nodes[i].on_my_slot();
                for _ in 0..frames_per_slot {
                    let mut buf = [0u8; MAX_PACKET];
                    let t = self.t_us;
                    let Some(len) = self.nodes[i].next_tx(t, &mut buf) else { break };
                    self.airtime += 1;
                    for j in 0..n {
                        if let Some(rssi) = self.links[i][j] {
                            if !self.lost() {
                                self.nodes[j].on_rx(&buf[..len], rssi);
                            }
                        }
                    }
                }
            }
            for i in 0..n {
                let t = self.t_us;
                self.nodes[i].poll(t);
                while let Some(e) = self.nodes[i].next_event() {
                    self.delivered[i].push(e);
                }
            }
        }

        pub fn run(&mut self, superframes: usize) {
            for _ in 0..superframes {
                self.superframe(6);
            }
        }

        pub fn got(&self, i: usize, origin: u16, port: u8) -> usize {
            self.delivered[i]
                .iter()
                .filter(|e| matches!(e, Event::Deliver { packet, .. } if packet.header.origin == origin && packet.header.port == port))
                .count()
        }
    }

    #[test]
    fn packet_round_trips() {
        let h = Header { origin: 7, id: 300, dest: BROADCAST, hop_limit: 3, hop_start: 3, flags: FLAG_WANT_ACK, attempt: 2, port: 5 };
        let p = Packet::new(h, b"hello").unwrap();
        let mut buf = [0u8; MAX_PACKET];
        let n = p.encode(&mut buf).unwrap();
        assert_eq!(Packet::decode(&buf[..n]).unwrap(), p);
        assert!(Packet::new(h, &[0; MAX_PAYLOAD + 1]).is_none());
    }

    /// Everyone in range: a broadcast reaches all, once each, and the
    /// managed flood costs far fewer than one relay per node.
    #[test]
    fn broadcast_in_one_room_is_cheap() {
        let mut net = Net::full(8, -50.0, 0.0);
        net.nodes[0].send(BROADCAST, 9, b"set", false).unwrap();
        net.run(10);
        for i in 1..8 {
            assert_eq!(net.got(i, 1, 9), 1, "node {}", i);
        }
        assert!(net.airtime <= 3, "airtime {} frames", net.airtime);
    }

    /// A line of nodes: the flood carries a broadcast to the far end within
    /// the hop limit, and not past it.
    #[test]
    fn broadcast_crosses_hops_up_to_the_limit() {
        let mut net = Net::line(6, 0.0);
        net.nodes[0].send(BROADCAST, 9, b"x", false).unwrap();
        net.run(30);
        // Hop limit 3 = up to three relays: four hops out, no further.
        for i in 1..=4 {
            assert_eq!(net.got(i, 1, 9), 1, "node {}", i);
        }
        assert_eq!(net.got(5, 1, 9), 0);
    }

    /// Unicast with ack across a lossy multi-hop line: delivered once,
    /// acknowledged, and nobody else gets it.
    #[test]
    fn unicast_is_acked_across_hops_despite_loss() {
        let mut net = Net::line(4, 0.2);
        let id = net.nodes[0].send(4, 7, b"cfg", true).unwrap();
        net.run(200);
        assert_eq!(net.got(3, 1, 7), 1);
        for i in 1..3 {
            assert_eq!(net.got(i, 1, 7), 0, "node {} isn't the destination", i);
        }
        assert!(net.delivered[0].contains(&Event::Acked { dest: 4, id }), "{:?}", net.delivered[0]);
    }

    #[test]
    fn unicast_to_nobody_fails_after_retries() {
        let mut net = Net::full(3, -50.0, 0.0);
        let id = net.nodes[0].send(99, 7, b"?", true).unwrap();
        net.run(300);
        assert!(net.delivered[0].contains(&Event::Failed { dest: 99, id }));
        assert_eq!(net.nodes[0].stats.retries, 2);
    }

    #[test]
    fn weak_signal_relays_first() {
        let mut m = Mesh::new(1, MeshConfig::default(), 5);
        let h = Header { origin: 2, id: 1, dest: BROADCAST, hop_limit: 3, hop_start: 3, flags: 0, attempt: 0, port: 1 };
        let mut buf = [0u8; MAX_PACKET];
        let n = Packet::new(h, b"").unwrap().encode(&mut buf).unwrap();
        m.on_rx(&buf[..n], -95.0);
        m.on_my_slot();
        let mut out = [0u8; MAX_PACKET];
        assert!(m.next_tx(0, &mut out).is_some(), "weak link: relay in the first slot");

        let mut m = Mesh::new(1, MeshConfig::default(), 5);
        m.on_rx(&buf[..n], -40.0);
        m.on_my_slot();
        assert!(m.next_tx(0, &mut out).is_none(), "strong link: wait for farther nodes");
    }
}
