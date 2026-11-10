// Host test of the link-layer codec (apps/common/link.h): golden vectors from the stdlib
// encoder (swarm/proto/gen_golden.py), the byte budgets of docs/swarm-wire.md, and rejection
// of anything that is not a version-1 swarm frame.
#include "apps/common/link.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace {
int g_failures = 0;
#define CHECK(c)                                                                  \
  do {                                                                            \
    if (!(c)) {                                                                   \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
      g_failures++;                                                               \
    }                                                                             \
  } while (0)

using Bytes = std::vector<uint8_t>;

std::map<std::string, Bytes> Golden() {
  std::map<std::string, Bytes> out;
  std::ifstream f(std::getenv("GOLDEN"));
  std::string line;
  while (std::getline(f, line)) {
    auto eq = line.find(" = ");
    if (eq == std::string::npos) continue;
    Bytes b;
    for (size_t i = eq + 3; i + 1 < line.size() + 1 && i + 1 <= line.size(); i += 2)
      b.push_back((uint8_t)std::stoi(line.substr(i, 2), nullptr, 16));
    out[line.substr(0, eq)] = b;
  }
  return out;
}

pbv::Bytes B(const Bytes& b) { return pbv::Bytes(b.data(), b.size()); }
Bytes V(const uint8_t* p, int n) { return Bytes(p, p + n); }

void GoldenFramesBuildAndParse() {
  auto g = Golden();
  CHECK(g.size() > 10);
  uint8_t buf[wire::kMaxFrame];
  wire::Frame f;

  Bytes advert = {1, 2, 3};
  // summary = golden "summary" bytes
  wire::Body poll;
  poll.slot = 2;
  poll.advert = B(advert);
  poll.summary = B(g["summary"]);
  int n = wire::Build(buf, wire::FrameType::kPoll, 3, wire::kBroadcast, 9, poll);
  CHECK(V(buf, n) == g["link_poll"]);
  CHECK(wire::Parse(g["link_poll"].data(), (int)g["link_poll"].size(), &f));
  CHECK(f.type == wire::FrameType::kPoll && f.src == 3 && f.dst == wire::kBroadcast && f.seq == 9 && f.slot == 2);
  CHECK(f.advert.size == 3 && f.summary.size == g["summary"].size());

  wire::Body resp;
  n = wire::Build(buf, wire::FrameType::kResponse, 20000, 3, 9, resp);
  CHECK(V(buf, n) == g["link_response"]);
  CHECK(wire::Parse(buf, n, &f) && f.src == 20000 && f.dst == 3 && f.advert.size == 0);

  wire::Body rep;
  rep.report = true;
  rep.t_round_b = 0x00123456;
  rep.t_reply_b = 0x00654321;
  n = wire::Build(buf, wire::FrameType::kReport, 20000, 3, 9, rep);
  CHECK(V(buf, n) == g["link_report"]);
  CHECK(wire::Parse(buf, n, &f) && f.t_round_b == 0x00123456 && f.t_reply_b == 0x00654321);

  wire::Body res;
  res.result = true;
  res.range_mm = -1234;
  n = wire::Build(buf, wire::FrameType::kResult, 3, 20000, 9, res);
  CHECK(V(buf, n) == g["link_result"]);
  CHECK(wire::Parse(buf, n, &f) && f.range_mm == -1234 && f.dst == 20000);

  Bytes pl = {0xaa, 0xbb};
  wire::Body mesh;
  mesh.payload = B(pl);
  n = wire::Build(buf, wire::FrameType::kMesh, 3, wire::kBroadcast, 0, mesh);
  CHECK(V(buf, n) == g["link_mesh"]);
  CHECK(wire::Parse(buf, n, &f) && f.payload.size == 2 && f.seq == 0 && f.slot == 0);
}

void ZeroValuesStayDistinguishable() {
  uint8_t buf[wire::kMaxFrame];
  wire::Frame f;
  // dst 0 is a node id (ids come from MACs), not "broadcast"; zero timestamps are still sent.
  wire::Body rep;
  rep.report = true;
  int n = wire::Build(buf, wire::FrameType::kReport, 5, 0, 1, rep);
  CHECK(wire::Parse(buf, n, &f) && f.dst == 0 && f.t_round_b == 0 && f.t_reply_b == 0);
}

void WorstCaseFramesFit() {
  uint8_t buf[wire::kMaxFrame];
  Bytes advert(wire::kMaxAdvert, 0x5a), summary(35, 0x11);
  wire::Body poll;
  poll.slot = 127;
  poll.advert = B(advert);
  poll.summary = B(summary);
  int n = wire::Build(buf, wire::FrameType::kPoll, 65534, 65534, 255, poll);
  CHECK(n > 0 && n <= 120);  // docs/swarm-wire.md: POLL 120 (addressed to a peer: dst present)

  wire::Body resp;
  resp.advert = B(advert);
  n = wire::Build(buf, wire::FrameType::kResponse, 65534, 65534, 255, resp);
  CHECK(n > 0 && n <= 81);

  // A MeshPacket of up to 115 bytes rides a broadcast mesh frame (10 bytes of link header).
  Bytes mesh(115, 0x33);
  wire::Body m;
  m.payload = B(mesh);
  n = wire::Build(buf, wire::FrameType::kMesh, 65534, wire::kBroadcast, 0, m);
  CHECK(n == 125);
  // One byte more does not fit: the budget is exact, and Build refuses instead of truncating.
  Bytes toobig(116, 0x33);
  m.payload = B(toobig);
  CHECK(wire::Build(buf, wire::FrameType::kMesh, 65534, wire::kBroadcast, 0, m) == 0);

  // A manifest frame (no dst, seq, slot): up to 114 bytes of Manifest.
  Bytes manifest(114, 0x77);
  wire::Body mf;
  mf.payload = B(manifest);
  n = wire::Build(buf, wire::FrameType::kFwManifest, 65534, wire::kBroadcast, 0, mf);
  CHECK(n == 124);

  // FwData: 109 bytes of FwData, broadcast; FwRequest unicast.
  Bytes fwdata(109, 0x44);
  wire::Body d;
  d.payload = B(fwdata);
  n = wire::Build(buf, wire::FrameType::kFwData, 65534, wire::kBroadcast, 0, d);
  CHECK(n > 0 && n <= 125);
  Bytes req(18, 0x44);
  d.payload = B(req);
  n = wire::Build(buf, wire::FrameType::kFwRequest, 65534, 65534, 0, d);
  CHECK(n > 0 && n <= 125);
}

void RejectsAnythingThatIsNotASwarmFrame() {
  wire::Frame f;
  uint8_t buf[wire::kMaxFrame];
  wire::Body b;
  int n = wire::Build(buf, wire::FrameType::kFinal, 3, 4, 7, b);
  CHECK(wire::Parse(buf, n, &f) && f.type == wire::FrameType::kFinal);
  for (int k = 0; k < n; k++) (void)wire::Parse(buf, k, &f);  // every truncation: no crash
  CHECK(!wire::Parse(buf, 0, &f));
  CHECK(!wire::Parse(buf, n - 1, &f));  // truncated inside the version field
  // Wrong version.
  uint8_t v2[] = {0x08, 0x03, 0x10, 0x03, 0x78, 0x02};
  CHECK(!wire::Parse(v2, sizeof v2, &f));
  // No version at all.
  uint8_t nover[] = {0x08, 0x03, 0x10, 0x03};
  CHECK(!wire::Parse(nover, sizeof nover, &f));
  // Unknown frame type.
  uint8_t badtype[] = {0x08, 0x2a, 0x10, 0x03, 0x78, 0x01};
  CHECK(!wire::Parse(badtype, sizeof badtype, &f));
  // src out of range (0xFFFF is the broadcast address, never a source).
  uint8_t badsrc[] = {0x08, 0x03, 0x10, 0xff, 0xff, 0x03, 0x78, 0x01};
  CHECK(!wire::Parse(badsrc, sizeof badsrc, &f));
  // Legacy anchor/tag DS-TWR frames and the old packed header (magic 0xA7) are not frames.
  uint8_t legacy[] = {0x01, 0x02, 0x03, 0x00};
  CHECK(!wire::Parse(legacy, sizeof legacy, &f));
  uint8_t old[] = {0xA7, 0x01, 0x03, 0x00, 0xff, 0xff, 0x09, 0x02, 0x00};
  CHECK(!wire::Parse(old, sizeof old, &f));
  // Too long.
  uint8_t big[wire::kMaxFrame + 1] = {};
  CHECK(!wire::Parse(big, sizeof big, &f));
  // Random garbage rarely parses, and never crashes.
  uint64_t x = 0x9e3779b97f4a7c15ULL;
  int accepted = 0;
  for (int i = 0; i < 20000; i++) {
    uint8_t junk[40];
    for (auto& c : junk) {
      x ^= x << 13; x ^= x >> 7; x ^= x << 17;
      c = (uint8_t)x;
    }
    accepted += wire::Parse(junk, 1 + (int)(x % 40), &f);
  }
  CHECK(accepted < 20);  // the version field alone makes a random match ~1 in 1000s
}

}  // namespace

int main() {
  GoldenFramesBuildAndParse();
  ZeroValuesStayDistinguishable();
  WorstCaseFramesFit();
  RejectsAnythingThatIsNotASwarmFrame();
  if (g_failures) return 1;
  std::puts("ok");
  return 0;
}
