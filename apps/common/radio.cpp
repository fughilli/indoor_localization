// UWB radio task: see radio.h and protocol.h.
#include "apps/common/radio.h"

#include <esp_timer.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "apps/common/pins.h"
#include "libs/dw3000/dw3000.h"

namespace radio {
namespace {

using namespace proto;

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

int Build(uint8_t *buf, Type type, uint16_t dst, uint8_t seq, const void *payload, int len)
{
  Header h = {kMagic, type, g_id, dst, seq, (uint8_t)MySlot()};
  memcpy(buf, &h, sizeof(h));
  if (len)
    memcpy(buf + sizeof(h), payload, len);
  return sizeof(h) + len;
}

// Header + [advert length][advert]: POLL and RESPONSE.
int BuildWithAdvert(uint8_t *buf, Type type, uint16_t dst, uint8_t seq)
{
  uint8_t payload[1 + kMaxAdvert + kSummaryLen];
  portENTER_CRITICAL(&g_mux);
  payload[0] = g_advert_len;
  memcpy(payload + 1, g_advert, g_advert_len);
  portEXIT_CRITICAL(&g_mux);
  int n = 1 + payload[0];
  if (type == kPoll)
  {
    // [advert len][advert][network summary]
    swarm_net_summary(payload + n);
    n += kSummaryLen;
  }
  return Build(buf, type, dst, seq, payload, n);
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

// One received frame into buf (without FCS): its length, or 0 at the
// deadline. Bad frames are counted and the receiver re-armed.
int Rx(uint8_t *buf, int64_t deadline)
{
  while (Now() < deadline)
  {
    int s = DWM3000.receivedFrameSucc();
    if (s == 1)
    {
      int len = DWM3000.rxFrameLength() - 2;
      if (len >= (int)sizeof(Header) && len <= kMaxFrame)
      {
        DWM3000.readBytes(0x12, 0, buf, len);
        if (buf[0] == kMagic)
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
int Expect(uint8_t *buf, Type type, uint16_t peer, uint8_t seq)
{
  int64_t deadline = Now() + kReplyTimeoutUs;
  while (true)
  {
    int len = Rx(buf, deadline);
    if (len == 0)
    {
      g_stats.timeouts++;
      return 0;
    }
    const Header *h = (const Header *)buf;
    if (h->type == type && h->src == peer && h->dst == g_id && h->seq == seq)
      return len;
    g_stats.foreign++;
    ArmRx();
  }
}

void EmitAdvert(uint16_t from, int64_t t, const uint8_t *frame, int len)
{
  int at = sizeof(Header);
  if (len < at + 1 || frame[at] > kMaxAdvert || len < at + 1 + frame[at])
    return;
  RadioEvent ev = {};
  ev.kind = RadioEvent::kAdvert;
  ev.peer = from;
  ev.t_us = t;
  ev.len = frame[at];
  memcpy(ev.data, frame + at + 1, ev.len);
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
  uint8_t seq = ++g_seq;
  Tx(tx, BuildWithAdvert(tx, kPoll, peer, seq), true);
  if (!WaitTxDone())
    return;
  uint64_t t1 = DWM3000.readTXTimestamp();
  g_stats.polls_tx++;

  int resp_len = Expect(rx, kResponse, peer, seq);
  if (!resp_len)
    return;
  uint64_t t4 = DWM3000.readRXTimestamp();
  int clock_offset = DWM3000.getRawClockOffset();
  float rssi = DWM3000.getSignalStrength();
  float fp = DWM3000.getFirstPathSignalStrength();
  uint8_t resp[kMaxFrame];
  memcpy(resp, rx, resp_len);

  Tx(tx, Build(tx, kFinal, peer, seq, nullptr, 0), true);
  if (!WaitTxDone())
    return;
  uint64_t t5 = DWM3000.readTXTimestamp();

  if (!Expect(rx, kReport, peer, seq))
    return;
  Report rep;
  memcpy(&rep, rx + sizeof(Header), sizeof(rep));
  int tof = DWM3000.ds_processRTInfo(TsDiff(t4, t1), TsDiff(t5, t4), rep.t_round_b,
                                     rep.t_reply_b, clock_offset);
  float range_m = DWM3000.convertToCM(tof) / 100.0f;

  Result res = {(int32_t)lroundf(range_m * 1000.0f)};
  Tx(tx, Build(tx, kResult, peer, seq, &res, sizeof(res)), false);
  WaitTxDone();
  ArmRx();

  int64_t now = Now();
  Heard(peer, now);
  EmitAdvert(peer, now, resp, resp_len);
  EmitRange(peer, true, range_m, rssi, fp);
  g_stats.ranges_init++;
}

// A POLL addressed to us arrived at radio time t2.
void Respond(const Header *poll, uint64_t t2)
{
  uint8_t tx[kMaxFrame], rx[kMaxFrame];
  uint16_t peer = poll->src;
  uint8_t seq = poll->seq;
  Tx(tx, BuildWithAdvert(tx, kResponse, peer, seq), true);
  if (!WaitTxDone())
    return;
  uint64_t t3 = DWM3000.readTXTimestamp();

  if (!Expect(rx, kFinal, peer, seq))
    return;
  uint64_t t6 = DWM3000.readRXTimestamp();
  float rssi = DWM3000.getSignalStrength();
  float fp = DWM3000.getFirstPathSignalStrength();

  Report rep = {TsDiff(t6, t3), TsDiff(t3, t2)};
  Tx(tx, Build(tx, kReport, peer, seq, &rep, sizeof(rep)), true);
  if (!WaitTxDone())
    return;

  if (!Expect(rx, kResult, peer, seq))
    return;
  Result res;
  memcpy(&res, rx + sizeof(Header), sizeof(res));
  EmitRange(peer, false, res.range_mm / 1000.0f, rssi, fp);
  g_stats.ranges_resp++;
}

// Align our superframe to the sync master's POLL (sent kPollOffsetUs into
// its slot `h->slot`), seen at local time t.
void Sync(const Header *h, int64_t t)
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
void HandleNetFrame(const Header *h, const uint8_t *rx, int len, int64_t t)
{
  const uint8_t *p = rx + sizeof(Header);
  int n = len - (int)sizeof(Header);
  g_stats.net_rx++;
  switch (h->type)
  {
    case kMesh:
      swarm_net_on_mesh(t, p, n, DWM3000.getSignalStrength());
      break;
    case kFwData:
      swarm_net_on_fw_data(t, p, n);
      break;
    case kFwRequest:
      if (h->dst == g_id)
        swarm_net_on_fw_request(t, p, n);
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
  uint8_t sum[kSummaryLen];
  swarm_net_summary(sum);
  auto u32 = [&](int at) { return (uint32_t)sum[at] | sum[at + 1] << 8 | sum[at + 2] << 16 | (uint32_t)sum[at + 3] << 24; };
  nodes[n++] = {g_id, u32(0), u32(4), (uint16_t)(sum[8] | sum[9] << 8)};
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
    Type type = kind == SWARM_NET_KIND_MESH ? kMesh : kind == SWARM_NET_KIND_FW_REQUEST ? kFwRequest : kFwData;
    Tx(tx, Build(tx, type, dest, 0, payload, (int)n), false);
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
  if (!g_rx_armed)
    ArmRx();
  while (Now() < until)
  {
    int len = Rx(rx, until);
    if (len == 0)
      return;
    int64_t t = Now();
    uint64_t ts = DWM3000.readRXTimestamp();
    const Header *h = (const Header *)rx;
    Heard(h->src, t);
    if (h->type == kPoll)
    {
      Sync(h, t);
      if (h->dst == g_id)
        Respond(h, ts);
      EmitAdvert(h->src, t, rx, len);
      int at = sizeof(Header) + 1 + rx[sizeof(Header)];
      if (len >= at + kSummaryLen)
        swarm_net_on_summary(t, h->src, rx + at, kSummaryLen);
    }
    else if (h->type >= kMesh && h->type <= kFwData)
    {
      HandleNetFrame(h, rx, len, t);
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
  Tx(tx, BuildWithAdvert(tx, kPoll, kBroadcast, ++g_seq), false);
  WaitTxDone();
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
  g_hooks = hooks;
  g_running = true;
  xTaskCreatePinnedToCore(Task, "radio", 12288, nullptr, 20, nullptr, 0);
}

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
