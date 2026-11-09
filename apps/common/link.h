// The UWB link layer's wire format: swarm.v1.LinkFrame (swarm/proto/swarm_wire.proto,
// docs/swarm-wire.md). Header-only and free of ESP-IDF, so the codec runs in a host test
// (apps/common/link_test.cc). Receiving reads the frame in place with pbview (a borrowed
// view over the DW3000 RX buffer, validated once); sending encodes in place into the TX
// buffer with pbv::Writer, fields in field-number order.
#ifndef APPS_COMMON_LINK_H
#define APPS_COMMON_LINK_H

#include <stdint.h>

#include "pbview.h"
#include "swarm/proto/swarm_wire_views.h"

namespace wire
{

using FrameType = swarm::v1::FrameType;

constexpr uint32_t kVersion = 1;
constexpr uint16_t kBroadcast = 0xFFFF;  // dst absent on the wire
// 127-byte standard PHR - 2 FCS.
constexpr int kMaxFrame = 125;
constexpr int kMaxAdvert = 64;  // SWARM_NODE_ADVERT_MAX

// A parsed frame. Bytes fields borrow the buffer given to Parse.
struct Frame
{
  FrameType type;
  uint16_t src;
  uint16_t dst;  // kBroadcast when absent
  uint8_t seq;
  uint8_t slot;
  pbv::Bytes payload;
  pbv::Bytes advert;
  pbv::Bytes summary;
  uint32_t t_round_b;  // REPORT
  uint32_t t_reply_b;
  int32_t range_mm;  // RESULT
};

inline bool ValidType(uint32_t t) { return t >= 1 && t <= 9; }

// Validate and decode `buf`: false for anything that is not a version-1 swarm frame (foreign
// UWB traffic, truncation, bad wire types, a wrong version, an id out of range).
inline bool Parse(const uint8_t *buf, int len, Frame *out)
{
  if (len <= 0 || len > kMaxFrame)
    return false;
  swarm::v1::LinkFrameView v;
  if (!swarm::v1::LinkFrameView::Parse(pbv::Bytes(buf, (size_t)len), &v).ok())
    return false;
  if (v.ver() != kVersion)
    return false;
  uint32_t type = (uint32_t)v.type();
  uint32_t src = v.src();
  auto dst = v.dst();
  if (!ValidType(type) || src > 0xFFFE || (dst && *dst > 0xFFFF))
    return false;
  out->type = (FrameType)type;
  out->src = (uint16_t)src;
  out->dst = dst ? (uint16_t)*dst : kBroadcast;
  out->seq = (uint8_t)v.seq();
  out->slot = (uint8_t)v.slot();
  out->payload = v.payload();
  out->advert = v.advert();
  out->summary = v.summary();
  out->t_round_b = (uint32_t)v.t_round_b();
  out->t_reply_b = (uint32_t)v.t_reply_b();
  out->range_mm = v.range_mm();
  return true;
}

// What a frame carries besides its header; unused members stay empty / negative.
struct Body
{
  int slot = -1;  // POLL: the sender's TDMA slot (-1: none)
  pbv::Bytes payload;
  bool report = false;
  uint32_t t_round_b = 0, t_reply_b = 0;
  bool result = false;
  int32_t range_mm = 0;
  pbv::Bytes advert;
  pbv::Bytes summary;
};

// Encode a frame into buf (kMaxFrame bytes at most) with the typed encoder generated from
// swarm_wire.proto (guardrails and the swarm_wire_bounds.txt limits apply). Returns its length,
// 0 if it does not fit or a limit is exceeded. Absent means default: dst for a broadcast, seq 0,
// no slot; fields are set in field-number order.
inline int Build(uint8_t *buf, FrameType type, uint16_t src, uint16_t dst, uint8_t seq, const Body &b)
{
  auto r = swarm::v1::LinkFrameEnc::Encode(buf, kMaxFrame, [&](swarm::v1::LinkFrameEnc &e) {
    e.type((int32_t)type).src(src);
    if (dst != kBroadcast)
      e.dst(dst);
    e.seq(seq).slot(b.slot > 0 ? (uint32_t)b.slot : 0).payload(b.payload);
    if (b.report)
      e.t_round_b(b.t_round_b).t_reply_b(b.t_reply_b);
    if (b.result)
      e.range_mm(b.range_mm);
    e.advert(b.advert).summary(b.summary).ver(kVersion);
  });
  return r.ok() ? (int)r.size : 0;
}

}  // namespace wire

#endif  // APPS_COMMON_LINK_H
