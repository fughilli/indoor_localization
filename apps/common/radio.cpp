// UWB radio task: see radio.h and protocol.h.
#include "apps/common/radio.h"

#include <esp_timer.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "apps/common/link.h"
#include "apps/common/pins.h"
#include "libs/dw3000/dw3000.h"

namespace radio {
namespace {

using namespace proto;
using FrameType = wire::FrameType;

static_assert(proto::kMaxFrame == wire::kMaxFrame && proto::kMaxAdvert == wire::kMaxAdvert, "link.h and protocol.h agree");
static_assert((int)FrameType::kPoll == proto::kPoll && (int)FrameType::kFwManifest == proto::kFwManifest &&
                  (int)FrameType::kResult == proto::kResult && (int)FrameType::kFwData == proto::kFwData,
              "frame type numbers");

// After bring-up the SPI clock goes up from the Arduino default (~1 MHz):
// frames carry ~70-byte payloads and every exchange step reads/writes them.
constexpr uint32_t kFastSpiHz = 8000000;
// Frame airtime + detection latency between the POLL leaving the sender
// (slot start + kPollOffsetUs) and a receiver noticing it; used to align
// superframes to the sync master.
constexpr int64_t kPollDetectUs = 300;
constexpr int kMaxKnown = 16;
// The task busy-polls the DW3000, so it blocks for one tick this often to let
// core 0's idle task run and feed the task watchdog. Frames keep arriving in
// the radio's buffer meanwhile; at worst one reply goes out ~1 ms late,
// inside kReplyTimeoutUs.
constexpr int64_t kIdleYieldPeriodUs = 100000;
// swarm_net housekeeping (retries, offers, settings changes) and app hooks.
constexpr int64_t kNetPollUs = 10000;

uint16_t g_id;
QueueHandle_t g_events;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t g_advert[kMaxAdvert];
int g_advert_len = 0;
RadioStats g_stats;

struct Known
{
  uint16_t id;
  int64_t last_us;
};
Known g_known[kMaxKnown];
int g_known_n = 0;

int64_t g_sf_start = 0;  // superframe origin on the local clock
uint8_t g_seq = 0;
uint16_t g_rr_last = 0;  // last peer this node initiated with
// The receiver is on and hasn't delivered a frame since. Re-arming then would
// abort any frame already arriving, so Listen leaves an armed receiver alone.
bool g_rx_armed = false;

// Sync log (SetSyncLog): which frames to report, and how often.
volatile bool g_sync_log = false;
volatile uint16_t g_sync_mask = 1 << kPoll;
volatile uint8_t g_sync_every = 1;
uint32_t g_sync_count = 0;
// CIR capture (SetCirCapture).
volatile bool g_cir_on = false;
volatile uint8_t g_cir_pre = 8, g_cir_n = 48, g_cir_every = 1;
uint32_t g_cir_count = 0;
QueueHandle_t g_cir_events;

// swarm_net glue: calls from other tasks, packets for the app.
NetHooks g_hooks;
bool g_running = false;
QueueHandle_t g_calls;    // NetCall*
QueueHandle_t g_packets;  // NetPacket
SemaphoreHandle_t g_call_lock;
SemaphoreHandle_t g_call_done;
int g_call_result;
volatile int g_new_antenna_delay = -1;
int64_t g_next_net_poll = 0;

int64_t Now() { return esp_timer_get_time(); }

void Heard(uint16_t id, int64_t t)
{
  if (id == g_id || id == kBroadcast)
    return;
  for (int i = 0; i < g_known_n; i++)
    if (g_known[i].id == id)
    {
      g_known[i].last_us = t;
      return;
    }
  if (g_known_n < kMaxKnown)
    g_known[g_known_n++] = {id, t};
}

void Prune(int64_t t)
{
  int j = 0;
  for (int i = 0; i < g_known_n; i++)
    if (t - g_known[i].last_us <= kPeerTimeoutUs)
      g_known[j++] = g_known[i];
  g_known_n = j;
}

// Rank of this node among every id it hears: the slot it owns.
int MySlot()
{
  int rank = 0;
  for (int i = 0; i < g_known_n; i++)
    if (g_known[i].id < g_id)
      rank++;
  return rank % kSlots;
}

// Lowest id in range sets the superframe timing.
uint16_t Master()
{
  uint16_t m = g_id;
  for (int i = 0; i < g_known_n; i++)
    if (g_known[i].id < m)
      m = g_known[i].id;
  return m;
}

// Round-robin: the next known id above the last one ranged, wrapping.
uint16_t NextPeer()
{
  uint16_t best = 0xFFFF, lowest = 0xFFFF;
  for (int i = 0; i < g_known_n; i++)
  {
    uint16_t id = g_known[i].id;
    if (id < lowest)
      lowest = id;
    if (id > g_rr_last && id < best)
      best = id;
  }
  g_rr_last = best != 0xFFFF ? best : lowest;
  return g_rr_last;
}

// A frame from this node (swarm.v1.LinkFrame, apps/common/link.h): its length.
int Build(uint8_t *buf, FrameType type, uint16_t dst, uint8_t seq, const wire::Body &body = wire::Body())
{
  return wire::Build(buf, type, g_id, dst, seq, body);
}

// POLL (also carrying our slot and the network summary) and RESPONSE: our advert rides along.
int BuildWithAdvert(uint8_t *buf, FrameType type, uint16_t dst, uint8_t seq)
{
  uint8_t advert[kMaxAdvert], summary[SWARM_NET_SUMMARY_MAX];
  int advert_len;
  portENTER_CRITICAL(&g_mux);
  advert_len = g_advert_len;
  memcpy(advert, g_advert, advert_len);
  portEXIT_CRITICAL(&g_mux);
  wire::Body body;
  body.advert = pbv::Bytes(advert, advert_len);
  if (type == FrameType::kPoll)
  {
    body.slot = MySlot();
    body.summary = pbv::Bytes(summary, swarm_net_summary(summary, sizeof(summary)));
  }
  return Build(buf, type, dst, seq, body);
}

void Tx(const uint8_t *buf, int len, bool then_rx)
{
  g_rx_armed = false;
  DWM3000.forceTRXOff();
  DWM3000.clearSystemStatus();
  DWM3000.writeBytes(0x14, 0, buf, len);
  DWM3000.setFrameLength(len);
  if (then_rx)
    DWM3000.TXInstantRX();
  else
    DWM3000.standardTX();
}

bool WaitTxDone()
{
  int64_t deadline = Now() + 2000;
  while (Now() < deadline)
    if (DWM3000.sentFrameSucc())
      return true;
  return false;
}

void ArmRx()
{
  DWM3000.forceTRXOff();
  DWM3000.clearSystemStatus();
  DWM3000.standardRX();
  g_rx_armed = true;
}

// One received frame into buf (without FCS), parsed into `f` (which borrows buf): its
// length, or 0 at the deadline. Frames that are not valid swarm frames are counted and the
// receiver re-armed.
int Rx(uint8_t *buf, wire::Frame *f, int64_t deadline)
{
  while (Now() < deadline)
  {
    int s = DWM3000.receivedFrameSucc();
    if (s == 1)
    {
      int len = DWM3000.rxFrameLength() - 2;
      if (len > 0 && len <= kMaxFrame)
      {
        DWM3000.readBytes(0x12, 0, buf, len);
        if (wire::Parse(buf, len, f))
        {
          g_rx_armed = false;  // the receiver stops after a good frame
          return len;
        }
      }
      g_stats.rx_errors++;
      ArmRx();
    }
    else if (s == 2)
    {
      g_stats.rx_errors++;
      ArmRx();
    }
  }
  return 0;
}

// Wait for `type` from `peer` in exchange `seq`; other frames are skipped
// (counted as foreign) and the receiver re-armed.
int Expect(uint8_t *buf, wire::Frame *f, FrameType type, uint16_t peer, uint8_t seq)
{
  int64_t deadline = Now() + kReplyTimeoutUs;
  while (true)
  {
    int len = Rx(buf, f, deadline);
    if (len == 0)
    {
      g_stats.timeouts++;
      return 0;
    }
    if (f->type == type && f->src == peer && f->dst == g_id && f->seq == seq)
      return len;
    g_stats.foreign++;
    ArmRx();
  }
}

void EmitAdvert(uint16_t from, int64_t t, const uint8_t *advert, int len)
{
  if (len <= 0 || len > kMaxAdvert)
    return;
  RadioEvent ev = {};
  ev.kind = RadioEvent::kAdvert;
  ev.peer = from;
  ev.t_us = t;
  ev.len = len;
  memcpy(ev.data, advert, ev.len);
  xQueueSend(g_events, &ev, 0);
}

// The frame just received, stamped `ts`. Call before anything else uses the receiver:
// the clock-offset and power registers describe the *last* frame.
void LogHeard(const wire::Frame *h, uint64_t ts)
{
  if (!g_sync_log || !(g_sync_mask & (1u << (int)h->type)) || (g_sync_count++ % g_sync_every))
    return;
  RadioEvent ev = {};
  ev.kind = RadioEvent::kHeard;
  ev.peer = h->src;
  ev.dst = h->dst;
  ev.ftype = (uint8_t)h->type;
  ev.seq = h->seq;
  ev.t_us = Now();
  ev.ts = ts;
  ev.clk = DWM3000.getRawClockOffset();
  ev.rssi = DWM3000.getSignalStrength();
  ev.fp_rssi = DWM3000.getFirstPathSignalStrength();
  xQueueSend(g_events, &ev, 0);
  if (g_cir_on && g_cir_events && !(g_cir_count++ % g_cir_every))
  {
    static CirEvent ce;   // radio task only
    ce.t_us = ev.t_us;
    ce.peer = h->src;
    ce.dst = h->dst;
    ce.ftype = (uint8_t)h->type;
    ce.seq = h->seq;
    ce.ts = ts;
    ce.fp_idx = DWM3000.readFirstPathIndex();
    int tap = ce.fp_idx >> 6;
    int n = g_cir_n > kCirWindow ? kCirWindow : g_cir_n;
    int first = tap - g_cir_pre;
    if (first < 0)
      first = 0;
    if (first + n > DWM3000Class::kCirTaps)
      first = DWM3000Class::kCirTaps - n;
    if (DWM3000.readCir(first, n, ce.data))
    {
      ce.first = first;
      ce.n = n;
      xQueueSend(g_cir_events, &ce, 0);
    }
  }
}

// A frame this node sent to `dst`, stamped `ts` by the transmitter.
void LogSent(uint8_t type, uint16_t dst, uint8_t seq, uint64_t ts)
{
  if (!g_sync_log || !(g_sync_mask & (1u << type)))
    return;
  RadioEvent ev = {};
  ev.kind = RadioEvent::kSent;
  ev.peer = dst;
  ev.ftype = type;
  ev.seq = seq;
  ev.t_us = Now();
  ev.ts = ts;
  xQueueSend(g_events, &ev, 0);
}

void EmitRange(uint16_t peer, bool initiator, float range_m, float rssi, float fp)
{
  RadioEvent ev = {};
  ev.kind = RadioEvent::kRange;
  ev.initiator = initiator;
  ev.peer = peer;
  ev.t_us = Now();
  ev.range_m = range_m;
  ev.rssi = rssi;
  ev.fp_rssi = fp;
  xQueueSend(g_events, &ev, 0);
}

void Initiate(uint16_t peer)
{
  uint8_t tx[kMaxFrame], rx[kMaxFrame];
  wire::Frame f;
  uint8_t seq = ++g_seq;
  Tx(tx, BuildWithAdvert(tx, FrameType::kPoll, peer, seq), true);
  if (!WaitTxDone())
    return;
  uint64_t t1 = DWM3000.readTXTimestamp();
  LogSent(kPoll, peer, seq, t1);
  g_stats.polls_tx++;

  if (!Expect(rx, &f, FrameType::kResponse, peer, seq))
    return;
  uint64_t t4 = DWM3000.readRXTimestamp();
  LogHeard(&f, t4);
  int clock_offset = DWM3000.getRawClockOffset();
  float rssi = DWM3000.getSignalStrength();
  float fp = DWM3000.getFirstPathSignalStrength();
  uint8_t advert[kMaxAdvert];
  int advert_len = f.advert.size <= (size_t)kMaxAdvert ? (int)f.advert.size : 0;
  memcpy(advert, f.advert.data, advert_len);

  Tx(tx, Build(tx, FrameType::kFinal, peer, seq), true);
  if (!WaitTxDone())
    return;
  uint64_t t5 = DWM3000.readTXTimestamp();
  LogSent(kFinal, peer, seq, t5);

  if (!Expect(rx, &f, FrameType::kReport, peer, seq))
    return;
  int tof = DWM3000.ds_processRTInfo(TsDiff(t4, t1), TsDiff(t5, t4), f.t_round_b, f.t_reply_b, clock_offset);
  float range_m = DWM3000.convertToCM(tof) / 100.0f;

  wire::Body res;
  res.result = true;
  res.range_mm = (int32_t)lroundf(range_m * 1000.0f);
  Tx(tx, Build(tx, FrameType::kResult, peer, seq, res), false);
  WaitTxDone();
  ArmRx();

  int64_t now = Now();
  Heard(peer, now);
  EmitAdvert(peer, now, advert, advert_len);
  EmitRange(peer, true, range_m, rssi, fp);
  g_stats.ranges_init++;
}

// A POLL addressed to us arrived at radio time t2.
void Respond(const wire::Frame *poll, uint64_t t2)
{
  uint8_t tx[kMaxFrame], rx[kMaxFrame];
  wire::Frame f;
  uint16_t peer = poll->src;
  uint8_t seq = poll->seq;
  Tx(tx, BuildWithAdvert(tx, FrameType::kResponse, peer, seq), true);
  if (!WaitTxDone())
    return;
  uint64_t t3 = DWM3000.readTXTimestamp();
  LogSent(kResponse, peer, seq, t3);

  if (!Expect(rx, &f, FrameType::kFinal, peer, seq))
    return;
  uint64_t t6 = DWM3000.readRXTimestamp();
  LogHeard(&f, t6);
  float rssi = DWM3000.getSignalStrength();
  float fp = DWM3000.getFirstPathSignalStrength();

  wire::Body rep;
  rep.report = true;
  rep.t_round_b = TsDiff(t6, t3);
  rep.t_reply_b = TsDiff(t3, t2);
  Tx(tx, Build(tx, FrameType::kReport, peer, seq, rep), true);
  if (!WaitTxDone())
    return;

  if (!Expect(rx, &f, FrameType::kResult, peer, seq))
    return;
  EmitRange(peer, false, f.range_mm / 1000.0f, rssi, fp);
  g_stats.ranges_resp++;
}

// Align our superframe to the sync master's POLL (sent kPollOffsetUs into
// its slot `h->slot`), seen at local time t.
void Sync(const wire::Frame *h, int64_t t)
{
  uint16_t master = Master();
  g_stats.sync_master = master;
  if (master == g_id || h->src != master)
    return;
  int64_t start = t - (h->slot * kSlotUs + kPollOffsetUs + kPollDetectUs);
  int64_t jump = (start - g_sf_start) % kSuperframeUs;
  if (jump > kSuperframeUs / 2)
    jump -= kSuperframeUs;
  if (jump < -kSuperframeUs / 2)
    jump += kSuperframeUs;
  g_sf_start += jump;
  g_stats.last_sync_jump_us = (int32_t)jump;
}

// Receive and serve until `until`: answer POLLs addressed to us, record
// every overheard advertisement, follow the sync master.
void HandleNetFrame(const wire::Frame *h, int64_t t)
{
  const uint8_t *p = h->payload.data;
  int n = (int)h->payload.size;
  g_stats.net_rx++;
  switch (h->type)
  {
    case FrameType::kMesh:
      swarm_net_on_mesh(t, p, n, DWM3000.getSignalStrength());
      break;
    case FrameType::kFwData:
      swarm_net_on_fw_data(t, p, n);
      break;
    case FrameType::kFwManifest:
      swarm_net_on_fw_manifest(t, p, n);
      break;
    case FrameType::kFwRequest:
      if (h->dst == g_id)
        swarm_net_on_fw_request(t, p, n);
      break;
    default:
      break;
  }
}

// Slots no node ranges in (ranks run 0..members-1) carry network frames
// only. They go to the nodes with firmware pages to serve: every node in
// range hears the same POLL summaries, so all of them derive the same list
// (ids in order) and spare slot k belongs to its k-th entry. With nothing
// to serve, spare slot s goes to rank (s - members) % members.
uint16_t g_servers[kMaxKnown + 1];
int g_server_n = 0;
int64_t g_servers_at = 0;

void UpdateServers(int64_t now)
{
  if (now - g_servers_at < 100000)
    return;
  g_servers_at = now;
  struct Node
  {
    uint16_t id;
    uint32_t running, staged;
    uint16_t pages;
  } nodes[kMaxKnown + 1];
  SwarmNetNeighbour nb[kMaxKnown];
  int n = (int)swarm_net_neighbours(now, kPeerTimeoutUs, nb, kMaxKnown);
  for (int i = 0; i < n; i++)
    nodes[i] = {nb[i].id, nb[i].fw_running, nb[i].fw_staged, nb[i].fw_pages};
  SwarmNetStatus st;
  swarm_net_status(&st);
  nodes[n++] = {g_id, st.fw_running, st.fw_staged, st.fw_pages};
  g_server_n = 0;
  for (int i = 0; i < n; i++)
  {
    bool serves = false;
    for (int j = 0; j < n && !serves; j++)
      serves = j != i && nodes[i].staged && nodes[j].staged == nodes[i].staged &&
               nodes[j].pages < nodes[i].pages && nodes[j].running < nodes[i].staged;
    if (serves)
      g_servers[g_server_n++] = nodes[i].id;
  }
  // Same order everywhere: by id.
  for (int i = 1; i < g_server_n; i++)
    for (int j = i; j > 0 && g_servers[j] < g_servers[j - 1]; j--)
    {
      uint16_t t = g_servers[j];
      g_servers[j] = g_servers[j - 1];
      g_servers[j - 1] = t;
    }
}

bool OwnsSpareSlot(int slot, int my)
{
  int members = g_known_n + 1;
  if (members >= kSlots || slot < members)
    return false;
  if (g_server_n)
    return g_servers[(slot - members) % g_server_n] == g_id;
  return (slot - members) % members == my;
}

// In our own slot, after the exchange: network frames until `deadline`.
void NetTx(int64_t deadline)
{
  uint8_t payload[SWARM_NET_MAX_FRAME], tx[kMaxFrame];
  while (Now() < deadline)
  {
    int32_t kind;
    uint16_t dest;
    size_t n = swarm_net_next_tx(Now(), payload, sizeof(payload), &kind, &dest);
    if (!n)
      break;
    FrameType type = kind == SWARM_NET_KIND_MESH           ? FrameType::kMesh
                     : kind == SWARM_NET_KIND_FW_REQUEST   ? FrameType::kFwRequest
                     : kind == SWARM_NET_KIND_FW_MANIFEST  ? FrameType::kFwManifest
                                                           : FrameType::kFwData;
    wire::Body body;
    body.payload = pbv::Bytes(payload, n);
    Tx(tx, Build(tx, type, dest, 0, body), false);
    WaitTxDone();
    g_stats.net_tx++;
  }
}

int Execute(NetCall &c)
{
  switch (c.kind)
  {
    case NetCall::kHostBundle:
      return swarm_net_host_bundle(c.data, c.len);
    case NetCall::kLocalOverride:
      swarm_net_local_override(c.a, c.value, c.flag);
      return 1;
    case NetCall::kHostManifest:
      return swarm_net_host_manifest(c.data, c.len);
    case NetCall::kHostChunk:
      swarm_net_host_chunk(Now(), c.a, (uint8_t)c.b, c.data, c.len);
      return 1;
    case NetCall::kHostPage:
      for (int off = 0, chunk = 0; off < c.len; off += 96, chunk++)
        swarm_net_host_chunk(Now(), c.a, (uint8_t)chunk, c.ptr + off, c.len - off < 96 ? c.len - off : 96);
      return 1;
    case NetCall::kSend:
      return swarm_net_send(c.a, (uint8_t)c.b, c.data, c.len, c.flag);
    case NetCall::kSetKey:
      swarm_net_set_hmac_key(c.len == 32 ? c.data : nullptr);
      return 1;
    case NetCall::kValidated:
      swarm_net_set_validated(true);
      return 1;
    case NetCall::kStatus:
      swarm_net_status(&c.status);
      c.neighbour_count = swarm_net_neighbours(Now(), 3000000, c.neighbours, 16);
      return 1;
    case NetCall::kLayerGet:
    {
      uint32_t over = 0;
      int r = swarm_net_layer_get(c.a, &c.value, &over);
      memcpy(c.data, &over, 4);
      return r;
    }
  }
  return -1;
}

// Between radio work: calls from other tasks, swarm_net housekeeping, app
// hooks, packets for the app.
void Service()
{
  NetCall *call;
  while (xQueueReceive(g_calls, &call, 0) == pdTRUE)
  {
    g_call_result = Execute(*call);
    xSemaphoreGive(g_call_done);
  }
  int delay = g_new_antenna_delay;
  if (delay >= 0)
  {
    g_new_antenna_delay = -1;
    DWM3000.setTXAntennaDelay(delay);
  }
  int64_t now = Now();
  if (now < g_next_net_poll)
    return;
  g_next_net_poll = now + kNetPollUs;
  swarm_net_poll(now);
  uint32_t bits = swarm_net_take_changes();
  if (bits && g_hooks.on_changes)
    g_hooks.on_changes(bits);
  NetPacket pk;
  int32_t n;
  while ((n = swarm_net_next_app(&pk.origin, &pk.dest, &pk.port, &pk.hops, &pk.rssi, pk.data,
                                 sizeof(pk.data))) >= 0)
  {
    pk.len = (uint8_t)n;
    xQueueSend(g_packets, &pk, 0);
  }
  if (g_hooks.on_idle)
    g_hooks.on_idle(now);
}

void Listen(int64_t until)
{
  uint8_t rx[kMaxFrame];
  wire::Frame f;
  if (!g_rx_armed)
    ArmRx();
  while (Now() < until)
  {
    int len = Rx(rx, &f, until);
    if (len == 0)
      return;
    int64_t t = Now();
    uint64_t ts = DWM3000.readRXTimestamp();
    const wire::Frame *h = &f;
    LogHeard(h, ts);
    Heard(h->src, t);
    if (h->type == FrameType::kPoll)
    {
      Sync(h, t);
      if (h->dst == g_id)
        Respond(h, ts);
      EmitAdvert(h->src, t, h->advert.data, (int)h->advert.size);
      if (h->summary.size)
        swarm_net_on_summary(t, h->src, h->summary.data, h->summary.size);
    }
    else if (h->type >= FrameType::kMesh && h->type <= FrameType::kFwManifest)
    {
      HandleNetFrame(h, t);
    }
    else if (h->dst != g_id)
    {
      g_stats.foreign++;
    }
    ArmRx();
  }
}

// Alone: broadcast a POLL now and then so others can find us.
void Beacon()
{
  uint8_t tx[kMaxFrame];
  Tx(tx, BuildWithAdvert(tx, FrameType::kPoll, kBroadcast, ++g_seq), false);
  WaitTxDone();
  if (g_sync_log)
    LogSent(kPoll, kBroadcast, g_seq, DWM3000.readTXTimestamp());
  g_stats.beacons++;
}

void Task(void *)
{
  int64_t last_sf = -1;
  int64_t last_data = -1;  // sf * kSlots + slot of the last spare slot used
  g_sf_start = Now();
  int64_t next_yield = Now() + kIdleYieldPeriodUs;
  for (;;)
  {
    int64_t now = Now();
    if (now >= next_yield)
    {
      next_yield = now + kIdleYieldPeriodUs;
      vTaskDelay(1);
    }
    Service();
    now = Now();
    Prune(now);
    UpdateServers(now);
    g_stats.peers = g_known_n;
    g_stats.slot = MySlot();
    if (g_known_n == 0)
    {
      // Listen a random 1-2 superframes, then announce ourselves.
      Listen(now + kSuperframeUs + (int64_t)(esp_random() % kSuperframeUs));
      if (g_known_n == 0)
        Beacon();
      continue;
    }
    int64_t since = now - g_sf_start;
    int64_t sf = since / kSuperframeUs;
    int64_t pos = since - sf * kSuperframeUs;
    int my = MySlot();
    int slot = (int)(pos / kSlotUs);
    int64_t in_slot = pos - slot * kSlotUs;
    int64_t slot_end = now - in_slot + kSlotUs;
    // Our ranging slot: the exchange, then network frames in what's left.
    if (slot == my && in_slot >= kPollOffsetUs && in_slot < kPollOffsetUs + kSlotUs / 2 && sf != last_sf)
    {
      last_sf = sf;
      swarm_net_on_my_slot();
      Initiate(NextPeer());
      NetTx(slot_end - kNetGuardUs);
      continue;
    }
    // A spare slot we own: network frames only, the whole slot. The
    // exchange fills most of a ranging slot, so this is where firmware
    // pages and settings traffic actually move.
    if (OwnsSpareSlot(slot, my) && in_slot >= kPollOffsetUs && in_slot < kSlotUs - kNetGuardUs &&
        sf * kSlots + slot != last_data)
    {
      last_data = sf * kSlots + slot;
      g_stats.spare_slots++;
      NetTx(slot_end - kNetGuardUs);
      continue;
    }
    // Sleep-listen until the next slot we act in.
    int64_t wait = kSuperframeUs;
    for (int s = 0; s < kSlots; s++)
    {
      if (s != my && !OwnsSpareSlot(s, my))
        continue;
      int64_t w = s * kSlotUs + kPollOffsetUs - pos;
      if (w <= 0)
        w += kSuperframeUs;
      if (w < wait)
        wait = w;
    }
    // Short listening windows so schedule changes (sync, peers) apply fast.
    Listen(now + (wait < 2000 ? wait : 2000));
  }
}

uint32_t ReadDevId() { return DWM3000.read(GEN_CFG_AES_LOW_REG, NO_OFFSET); }

bool DevIdOk(uint32_t id) { return id == 0xDECA0302 || id == 0xDECA0312; }

// Hard reset + DEV_ID, waking the module first on retries (a DW3000 left
// asleep reads all zeros through hard resets until CS is held low).
bool Probe()
{
  for (int attempt = 0; attempt < 5; attempt++)
  {
    if (attempt)
    {
      pinMode(NODE_PIN_WAKEUP, OUTPUT);
      digitalWrite(NODE_PIN_WAKEUP, HIGH);
      digitalWrite(NODE_PIN_CS, LOW);
      delay(1);
      digitalWrite(NODE_PIN_CS, HIGH);
      digitalWrite(NODE_PIN_WAKEUP, LOW);
      delay(2);
    }
    DWM3000.hardReset();
    delay(200);
    uint32_t id = ReadDevId();
    Serial.printf("{\"t\":%.3f,\"type\":\"devid\",\"attempt\":%d,\"value\":\"0x%08lX\"}\n",
                  millis() / 1000.0, attempt + 1, (unsigned long)id);
    if (DevIdOk(id))
      return true;
  }
  return false;
}

bool Fail(const char *stage)
{
  Serial.printf("{\"t\":%.3f,\"type\":\"radio_error\",\"stage\":\"%s\",\"devid\":\"0x%08lX\"}\n",
                millis() / 1000.0, stage, (unsigned long)ReadDevId());
  return false;
}

}  // namespace

bool Init(int antenna_delay)
{
  g_calls = xQueueCreate(2, sizeof(NetCall *));
  g_packets = xQueueCreate(8, sizeof(NetPacket));
  g_call_lock = xSemaphoreCreateMutex();
  g_call_done = xSemaphoreCreateBinary();
  DWM3000.setPins({NODE_PIN_SCK, NODE_PIN_MISO, NODE_PIN_MOSI, NODE_PIN_CS, NODE_PIN_RST});
  DWM3000.begin();
  if (!Probe())
    return Fail("devid");
  if (!DWM3000.checkForIDLE())
    return Fail("idle_before_reset");
  DWM3000.softReset();
  delay(200);
  if (!DWM3000.checkForIDLE())
    return Fail("idle_after_reset");
  DWM3000.init();
  DWM3000.setupGPIO();
  DWM3000.setTXAntennaDelay(antenna_delay);
  DWM3000.setSpiClock(kFastSpiHz);
  if (!DevIdOk(ReadDevId()))
  {
    // Fall back if the fast clock doesn't hold on this wiring.
    DWM3000.setSpiClock(2000000);
    if (!DevIdOk(ReadDevId()))
      return Fail("fast_spi");
  }
  return true;
}

void Start(uint16_t id, QueueHandle_t events, const NetHooks &hooks)
{
  g_id = id;
  g_events = events;
  g_cir_events = xQueueCreate(3, sizeof(CirEvent));
  g_hooks = hooks;
  g_running = true;
  xTaskCreatePinnedToCore(Task, "radio", 12288, nullptr, 20, nullptr, 0);
}

void SetSyncLog(bool on, uint16_t mask, uint8_t every)
{
  g_sync_mask = mask;
  g_sync_every = every ? every : 1;
  g_sync_log = on;
}

void SetCirCapture(bool on, uint8_t pre, uint8_t n, uint8_t every)
{
  g_cir_pre = pre;
  g_cir_n = n ? n : kCirWindow;
  g_cir_every = every ? every : 1;
  g_cir_on = on;
}

bool PollCir(CirEvent *out) { return g_cir_events && xQueueReceive(g_cir_events, out, 0) == pdTRUE; }

void SetAntennaDelay(int delay) { g_new_antenna_delay = delay; }

int Call(NetCall &call)
{
  if (!g_running)
    return Execute(call);  // no radio task: nothing else touches swarm_net
  xSemaphoreTake(g_call_lock, portMAX_DELAY);
  NetCall *p = &call;
  xQueueSend(g_calls, &p, portMAX_DELAY);
  int result = -1;
  if (xSemaphoreTake(g_call_done, pdMS_TO_TICKS(2000)) == pdTRUE)
    result = g_call_result;
  xSemaphoreGive(g_call_lock);
  return result;
}

bool NextPacket(NetPacket *out) { return g_packets && xQueueReceive(g_packets, out, 0) == pdTRUE; }

void SetAdvert(const uint8_t *data, int len)
{
  if (len > kMaxAdvert)
    return;
  portENTER_CRITICAL(&g_mux);
  memcpy(g_advert, data, len);
  g_advert_len = len;
  portEXIT_CRITICAL(&g_mux);
}

RadioStats Stats() { return g_stats; }

}  // namespace radio
