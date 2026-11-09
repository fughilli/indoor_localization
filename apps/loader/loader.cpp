// Swarm loader: the factory app that owns the boot (docs/swarm-network.md
// "Boot flow"). Rarely changed, so a bad node image can't strand a board.
//
//  1. An install is pending (the node staged a verified image and rebooted
//     here): verify it again, copy staging -> main, verify main, record its
//     version, boot main (pending verify: if main resets before marking
//     itself healthy, the IDF bootloader brings us back here).
//  2. Main is bootable and nobody asked us to stay: boot main.
//  3. Otherwise, recovery: join the swarm like a node without fusion — TDMA
//     slots, ranging (results unused), mesh, settings sync, firmware pages —
//     until a newer verified image arrives; then install it (1).
//
// USB serial (460800 baud, newline JSON): the network commands
// (apps/common/netcmd.h) plus {"cmd":"boot_main"} and {"cmd":"reboot"}.
#include <Arduino.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "apps/common/boot.h"
#include "apps/common/fw_key.h"
#include "apps/common/netcmd.h"
#include "apps/common/netstore.h"
#include "apps/common/radio.h"
#include "apps/common/store.h"
#include "apps/common/swarm_crypto.h"
#include "swarm/net/swarm_net.h"

namespace {

constexpr uint32_t kBaud = 460800;
constexpr uint16_t kAntennaDelayKey = 400;  // swarm/node settings registry
constexpr int kDefaultAntennaDelay = 16350;
constexpr int kBootPin = 0;

uint16_t g_id;
QueueHandle_t g_radio_events;
volatile bool g_fw_ready = false;

uint16_t IdFromMac()
{
  uint64_t mac = ESP.getEfuseMac();
  uint16_t id = (uint16_t)((((mac >> 32) & 0xFF) << 8) | ((mac >> 40) & 0xFF));
  if (id == 0 || id == 0xFFFF)
    id ^= 0x5A5A;
  return id;
}

void Log(const char *type, const char *detail = "")
{
  netcmd::Reply("{\"t\":%.3f,\"type\":\"%s\",\"app\":\"loader\"%s}\n", millis() / 1000.0, type, detail);
}

// Step 1. Returns only if there was nothing to install or it failed.
void InstallIfPending()
{
  if (!store::Exists(store::kInstall))
    return;
  store::Remove(store::kInstall);  // one attempt per request
  // swarm_net owns the manifest format (swarm.v1.Manifest, staged as a length-delimited message).
  uint32_t version = 0, size = 0;
  uint8_t sha256[32];
  if (!swarm_net_staged_manifest_info(&version, &size, sha256) || !swarm_net_verify_staged())
    return Log("install_failed", ",\"stage\":\"verify_staged\"");
  char detail[64];
  snprintf(detail, sizeof(detail), ",\"version\":%lu,\"size\":%lu", (unsigned long)version,
           (unsigned long)size);
  Log("installing", detail);
  if (!boot::InstallStaged(size, sha256))
    return Log("install_failed", ",\"stage\":\"copy\"");
  store::WriteU32(store::kInstalled, version);
  store::Remove(store::kStayInLoader);
  Log("installed", detail);
  Serial.flush();
  boot::RestartIntoMain();
}

void OnNetChanges(uint32_t bits)
{
  netstore::NoteChanges(bits);
  if (bits & SWARM_NET_CHANGED_FW_READY)
    g_fw_ready = true;
}

int AntennaDelay()
{
  uint32_t swarm = 0, over = 0;
  int layers = swarm_net_layer_get(kAntennaDelayKey, &swarm, &over);
  if (layers & 2)
    return (int)over;
  if (layers & 1)
    return (int)swarm;
  return kDefaultAntennaDelay;
}

}  // namespace

void setup()
{
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(8192);
  Serial.begin(kBaud);
  delay(50);
  g_id = IdFromMac();
  netcmd::Init();
  bool lfs = store::Mount();
  swarm_crypto::SelfTest st = swarm_crypto::RunSelfTest();
  SwarmFlashOps staging;
  bool staging_ok = boot::StagingOps(&staging);
  uint32_t running = store::ReadU32(store::kInstalled, 0);
  swarm_net_init(g_id, nullptr, fw_key::kPublic, running, esp_random(), &staging);
  netstore::Load();
  char detail[160];
  snprintf(detail, sizeof(detail),
           ",\"id\":%u,\"installed\":%lu,\"lfs\":%s,\"staging\":%s,\"crypto\":%s,\"main_bootable\":%s",
           g_id, (unsigned long)running, lfs ? "true" : "false", staging_ok ? "true" : "false",
           st.hmac && st.p256 ? "true" : "false", boot::MainBootable() ? "true" : "false");
  Log("boot", detail);

  InstallIfPending();
  pinMode(kBootPin, INPUT_PULLUP);
  bool stay = store::Exists(store::kStayInLoader) || digitalRead(kBootPin) == LOW;
  if (!stay && boot::MainBootable())
  {
    Serial.flush();
    boot::RestartIntoMain();
  }

  // Recovery: join the swarm and wait for an image.
  Log("recovery", stay ? ",\"reason\":\"asked\"" : ",\"reason\":\"no bootable main\"");
  g_radio_events = xQueueCreate(16, sizeof(RadioEvent));
  if (radio::Init(AntennaDelay()))
  {
    radio::SetAdvert(nullptr, 0);
    NetHooks hooks;
    hooks.on_changes = OnNetChanges;
    hooks.on_idle = netstore::Idle;
    radio::Start(g_id, g_radio_events, hooks);
  }
  else
  {
    Log("radio_down");
  }
}

void loop()
{
  static int64_t next_status = 0;
  RadioEvent ev;
  while (xQueueReceive(g_radio_events, &ev, 0) == pdTRUE)
  {
  }  // ranging keeps the schedule going; no fusion here
  while (const char *line = netcmd::ReadLine())
  {
    if (netcmd::Handle(line))
      continue;
    if (netcmd::IsCmd(line, "net"))
    {
      netcmd::EmitStatus(g_id, "loader");
    }
    else if (netcmd::IsCmd(line, "boot_main"))
    {
      store::Remove(store::kStayInLoader);
      bool ok = boot::MainBootable();
      netcmd::Ack("boot_main", ok);
      if (ok)
      {
        Serial.flush();
        boot::RestartIntoMain();
      }
    }
    else if (netcmd::IsCmd(line, "reboot"))
    {
      netcmd::Ack("reboot", true);
      Serial.flush();
      ESP.restart();
    }
    else
    {
      netcmd::Ack("unknown", false);
    }
  }
  NetPacket pk;
  while (radio::NextPacket(&pk))
  {
  }  // no app ports in the loader
  int64_t now = esp_timer_get_time();
  if (now >= next_status)
  {
    next_status = now + 1000000;
    netcmd::EmitStatus(g_id, "loader");
  }
  if (g_fw_ready)
  {
    // Persist settings first (the radio task writes after 1 s of quiet).
    delay(1500);
    store::WriteU32(store::kInstall, 1);
    Log("restart_to_install");
    Serial.flush();
    ESP.restart();
  }
  delay(2);
}
