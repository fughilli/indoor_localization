// UWB radio task: TDMA schedule + DS-TWR exchanges (protocol.h), and the
// swarm network (swarm_net: mesh, settings sync, firmware distribution).
// Owns the DW3000 and swarm_net exclusively and runs pinned to core 0.
// Ranging results and overheard adverts go to the app as RadioEvents; mesh
// packets for the app as NetPackets; the app reaches swarm_net only through
// radio::Call, which runs the request on the radio task.
#ifndef APPS_COMMON_RADIO_H
#define APPS_COMMON_RADIO_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "apps/common/protocol.h"
#include "swarm/net/swarm_net.h"

struct RadioEvent
{
  enum Kind : uint8_t
  {
    kAdvert,  // a peer's advertisement (data/len) heard at t_us
    kRange,   // a completed exchange with `peer`
  } kind;
  int8_t initiator;  // kRange: 1 if this node initiated
  uint16_t peer;
  int64_t t_us;
  float range_m;
  float rssi;
  float fp_rssi;
  uint8_t len;
  uint8_t data[proto::kMaxAdvert];
};

// A mesh packet for the app (ports >= 16, settings reports).
struct NetPacket
{
  uint16_t origin;
  uint16_t dest;
  uint8_t port;
  uint8_t hops;
  float rssi;
  uint8_t len;
  uint8_t data[112];
};

struct RadioStats
{
  uint32_t polls_tx;
  uint32_t ranges_init;    // completed as initiator
  uint32_t ranges_resp;    // completed as responder
  uint32_t timeouts;       // a reply didn't arrive in time
  uint32_t rx_errors;      // bad frames (CRC / PHR / SFD)
  uint32_t foreign;        // well-formed frames not for this exchange
  uint32_t beacons;        // broadcast POLLs while alone
  uint32_t net_tx;         // network frames sent in our slots
  uint32_t net_rx;         // network frames received
  uint32_t spare_slots;    // spare TDMA slots used for network frames
  uint16_t sync_master;    // whose superframe timing we follow (0: none)
  uint8_t slot;
  uint8_t peers;
  int32_t last_sync_jump_us;
};

// A request for swarm_net, run on the radio task by radio::Call.
struct NetCall
{
  enum Kind : uint8_t
  {
    kHostBundle,     // data: tagged swarm bundle
    kLocalOverride,  // a: key, value: raw, flag: set (else clear)
    kHostManifest,   // data: signed manifest
    kHostChunk,      // a: page, b: chunk, data
    kHostPage,       // a: page, ptr/len: up to 4096 bytes (chunked here)
    kSend,           // a: dest, b: port, flag: want-ack, data: payload
    kSetKey,         // data: 32-byte HMAC key, or len 0 to remove
    kStatus,         // fills `status`
    kValidated,      // the running image passed its health checks
    kLayerGet,       // a: key -> value: swarm raw, b: (unused), data[0..4]:
                     // override raw; returns bit 0 swarm set, bit 1 override set
  } kind;
  uint16_t a = 0;
  uint16_t b = 0;
  uint32_t value = 0;
  bool flag = false;
  uint16_t len = 0;
  const uint8_t *ptr = nullptr;  // kHostPage
  uint8_t data[640];
  SwarmNetStatus status;  // kStatus
  SwarmNetNeighbour neighbours[16];  // kStatus
  uint8_t neighbour_count = 0;
};

// App hooks, called on the radio task.
struct NetHooks
{
  // swarm_net changes (SWARM_NET_CHANGED_* bits).
  void (*on_changes)(uint32_t bits) = nullptr;
  // Every ~10 ms between radio work (persistence, housekeeping).
  void (*on_idle)(int64_t t_us) = nullptr;
};

namespace radio {

// Bring up the DW3000 (pins, reset, init). Returns false if the module
// doesn't answer.
bool Init(int antenna_delay);
// Start the radio task (after Init succeeded, swarm_net_init ran and the
// first advert is set). Without a radio, Call still works (runs inline).
void Start(uint16_t id, QueueHandle_t events, const NetHooks &hooks);
// Latest encoded advertisement to carry in POLL / RESPONSE frames (len 0:
// none, e.g. the loader).
void SetAdvert(const uint8_t *data, int len);
// Antenna delay change (settings), applied by the radio task.
void SetAntennaDelay(int delay);
// Run `call` on the radio task; returns its result (bool as 0/1, or a
// packet id for kSend; -1 on failure).
int Call(NetCall &call);
// Next mesh packet for the app.
bool NextPacket(NetPacket *out);
RadioStats Stats();

}  // namespace radio

#endif  // APPS_COMMON_RADIO_H
