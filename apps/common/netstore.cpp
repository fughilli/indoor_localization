// See netstore.h.
#include "apps/common/netstore.h"

#include <esp_timer.h>
#include <string.h>

#include "apps/common/radio.h"
#include "apps/common/store.h"
#include "swarm/net/swarm_net.h"

namespace netstore {
namespace {

constexpr int64_t kQuietUs = 1000000;
constexpr size_t kMaxBlob = 1024;

uint32_t g_dirty = 0;  // SWARM_NET_CHANGED_SWARM / _OVERRIDES
int64_t g_changed_us = 0;
uint8_t g_buf[kMaxBlob];

}  // namespace

void Load()
{
  int n = store::Read(store::kKey, g_buf, sizeof(g_buf));
  if (n == 32)
    swarm_net_set_hmac_key(g_buf);
  n = store::Read(store::kBundle, g_buf, sizeof(g_buf));
  if (n > 0)
    swarm_net_load_bundle(g_buf, n);
  n = store::Read(store::kOverrides, g_buf, sizeof(g_buf));
  if (n > 0)
    swarm_net_load_overrides(g_buf, n);
}

void NoteChanges(uint32_t bits)
{
  bits &= SWARM_NET_CHANGED_SWARM | SWARM_NET_CHANGED_OVERRIDES;
  if (!bits)
    return;
  g_dirty |= bits;
  g_changed_us = esp_timer_get_time();
}

void Idle(int64_t t_us)
{
  if (!g_dirty || t_us - g_changed_us < kQuietUs)
    return;
  if (g_dirty & SWARM_NET_CHANGED_SWARM)
  {
    size_t n = swarm_net_bundle(g_buf, sizeof(g_buf));
    if (n)
      store::WriteAtomic(store::kBundle, g_buf, n);
  }
  if (g_dirty & SWARM_NET_CHANGED_OVERRIDES)
  {
    size_t n = swarm_net_overrides(g_buf, sizeof(g_buf));
    if (n)
      store::WriteAtomic(store::kOverrides, g_buf, n);
  }
  g_dirty = 0;
}

bool SetKey(const uint8_t *key)
{
  static NetCall call;
  call.kind = NetCall::kSetKey;
  call.len = key ? 32 : 0;
  if (key)
    memcpy(call.data, key, 32);
  radio::Call(call);
  if (!key)
  {
    store::Remove(store::kKey);
    return true;
  }
  return store::WriteAtomic(store::kKey, key, 32);
}

}  // namespace netstore
