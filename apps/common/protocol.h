// Station-free swarm radio protocol (design doc §5): TDMA slots, one DS-TWR
// exchange per slot initiated by the slot's owner, pose advertisements
// piggybacked on the POLL (overheard by everyone in range) and the RESPONSE.
//
//   A (slot owner)                         B (any peer, round-robin)
//   POLL    (A advert, slot, broadcast-readable) ─►      t1 / t2
//           ◄─ RESPONSE (B advert)                       t4 / t3
//   FINAL   ─►                                           t5 / t6
//           ◄─ REPORT  (B: t_round = t6-t3, t_reply = t3-t2)
//   RESULT  (range) ─►       A computes the DS-TWR range; B gets it here,
//                            so both ends fold the same measurement.
//
// After its exchange the slot owner sends network frames (swarm_net:
// docs/swarm-network.md) until the slot's guard time: mesh packets and
// firmware data as broadcasts, a firmware request to the neighbour serving
// it. Every POLL carries the sender's network summary after its advert.
//
// Every frame is a swarm.v1.LinkFrame protobuf message (swarm/proto/swarm_wire.proto,
// docs/swarm-wire.md), encoded and parsed by apps/common/link.h; this header keeps the
// protocol's timing and the numeric frame types the sync log reports. Frames stay under 125
// bytes (DW3000 standard PHR); the 2-byte FCS is appended by the radio. The summary and the
// manifest travel in the POLL and in a frame of their own (kFwManifest).
#ifndef APPS_COMMON_PROTOCOL_H
#define APPS_COMMON_PROTOCOL_H

#include <stdint.h>

namespace proto {

enum Type : uint8_t
{
  kPoll = 1,
  kResponse = 2,
  kFinal = 3,
  kReport = 4,
  kResult = 5,
  kMesh = 6,       // swarm_net mesh packet (link broadcast)
  kFwRequest = 7,  // swarm_net firmware page request (to the server)
  kFwData = 8,     // swarm_net firmware chunk (link broadcast)
  kFwManifest = 9, // swarm_net signed firmware manifest (link broadcast)
};

constexpr uint16_t kBroadcast = 0xFFFF;

// TDMA. A superframe has kSlots slots; each node owns the slot equal to its
// rank among the ids it hears (incl. itself), and timing follows the
// lowest-id node in range. 6 slots x 5 ms = 30 ms: each node initiates ~33
// times a second; with 4 nodes every pair ranges ~22 times a second.
// SWARM_SLOTS must be at least the number of nodes: a node owns slot (rank % kSlots),
// so more nodes than slots share a slot and collide. The default 6 keeps 4-node
// meshes ranging at full rate; a 10-node fleet builds with --copt=-DSWARM_SLOTS=12.
#ifndef SWARM_SLOTS
#define SWARM_SLOTS 6
#endif
constexpr int kSlots = SWARM_SLOTS;
constexpr int64_t kSlotUs = 5000;
constexpr int64_t kSuperframeUs = kSlots * kSlotUs;
// POLL goes out this long after the slot starts (guard for sync error).
constexpr int64_t kPollOffsetUs = 400;
// Each wait for the next frame of an exchange.
constexpr int64_t kReplyTimeoutUs = 3000;
// Ids not heard for this long leave the schedule.
constexpr int64_t kPeerTimeoutUs = 2000000;

constexpr int kMaxAdvert = 64;  // SWARM_NODE_ADVERT_MAX
// Network frames stop this long before the slot ends: a full frame takes
// ~450 us (SPI write + air), and the next slot's POLL is 400 us in.
constexpr int64_t kNetGuardUs = 500;
constexpr int kMaxFrame = 125;  // + 2 FCS = 127

// DW3000 timestamps are 40-bit; differences of an exchange fit 32 bits.
inline uint32_t TsDiff(uint64_t later, uint64_t earlier)
{
  return (uint32_t)((later - earlier) & 0xFFFFFFFFFFULL);
}

}  // namespace proto

#endif  // APPS_COMMON_PROTOCOL_H
