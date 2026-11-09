// pbview views of widget.v1: encode a PoseAdvertisement and a WidgetConfig with the in-place
// Writer and read them back in place. Proves the generator, runtime and this schema build
// together under the repo's toolchain; the wire format itself is checked by pbview's
// protoc-golden conformance suite.
#include <cstdio>
#include <string_view>

#include "swarm/proto/widget_config_views.h"

namespace {
int g_failures = 0;
#define CHECK(c)                                                                  \
  do {                                                                            \
    if (!(c)) {                                                                   \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
      g_failures++;                                                               \
    }                                                                             \
  } while (0)
}  // namespace

int main() {
  namespace w = widget::v1;
  uint8_t buf[256];

  pbv::Writer a(buf, sizeof buf);
  CHECK(a.Uint32(1, 7) && a.Uint32(2, 300));
  CHECK(a.Message(3, [](pbv::Writer& v) { return v.Float(1, 1.5f) && v.Float(2, -2.0f) && v.Float(3, 0.25f); }));
  CHECK(a.Float(5, 0.04f) && a.Uint32(6, 5));
  w::PoseAdvertisementView adv;
  CHECK(w::PoseAdvertisementView::Parse(a.bytes(), &adv).ok());
  CHECK(adv.widget_id() == 7 && adv.seq() == 300 && adv.flags() == 5);
  CHECK(adv.p() && adv.p()->x() == 1.5f && adv.p()->y() == -2.0f && adv.p()->z() == 0.25f);
  CHECK(!adv.v());  // absent message: presence is visible

  uint8_t cfg[256];
  pbv::Writer c(cfg, sizeof cfg);
  CHECK(c.Message(1, [](pbv::Writer& e) {
    return e.String(1, "widget-x") && e.Message(2, [](pbv::Writer& s) { return s.Uint32(1, 3) && s.String(2, "uwb"); });
  }));
  w::WidgetConfigView wc;
  CHECK(w::WidgetConfigView::Parse(c.bytes(), &wc).ok());
  CHECK(wc.extrinsics() && wc.extrinsics()->widget_model() == "widget-x");
  auto sensors = wc.extrinsics()->sensors();
  w::SensorExtrinsicView s;
  CHECK(sensors.Next(&s) && s.id() == 3 && s.name() == "uwb");
  CHECK(!sensors.Next(&s));
  CHECK(!wc.intrinsics());

  if (g_failures) return 1;
  std::puts("ok");
  return 0;
}
