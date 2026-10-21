// Station-free swarm node firmware: one image for every widget, no roles.
// Runs as the "main" app under the loader (docs/swarm-network.md).
//
//  - radio task (core 0, apps/common/radio.cpp): TDMA schedule; in its own
//    slot each node initiates DS-TWR with the next peer round-robin, then
//    sends its network frames (mesh, firmware pages); otherwise it answers
//    and overhears. Pose advertisements ride on every POLL/RESPONSE. It owns
//    swarm_net (mesh, live settings, firmware distribution).
//  - fusion task (core 1, here): IMU at 200 Hz (or IMU-less propagation),
//    magnetometer, and every advert / range from the radio go into the Rust
//    node runtime (swarm/node: swarm_core estimator + cooperative rules).
//    Settings changes reach it staged by the radio task.
//  - loop task: USB serial commands (newline JSON, see HandleCommand) and
//    network status.
//  - USB serial (460800 baud): the node's own pose at 100 Hz ("self"), the
//    whole swarm as this node knows it at 10 Hz ("swarm"), every range
//    ("range"), stats and network status each second, mesh packets for the
//    host ("mesh").
#include <Arduino.h>
#include <Preferences.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <soc/rtc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <string.h>

#include "apps/common/boot.h"
#include "apps/common/fw_key.h"
#include "apps/common/netcmd.h"
#include "apps/common/netstore.h"
#include "apps/common/pins.h"
#include "apps/common/radio.h"
#include "apps/common/store.h"
#include "apps/common/swarm_crypto.h"
#include "apps/node/imu.h"
#include "swarm/net/swarm_net.h"
#include "swarm/node/swarm_node.h"

namespace {

constexpr uint32_t kBaud = 460800;
constexpr int64_t kFusionPeriodUs = 5000;  // 200 Hz
constexpr int kSelfEvery = 2;              // 100 Hz self pose
constexpr int kSwarmEvery = 20;            // 10 Hz swarm table
constexpr int kAdvertEvery = 4;            // 50 Hz advert refresh for radio
constexpr int kMagEvery = 4;               // 50 Hz magnetometer poll
constexpr int kStatsEvery = 200;           // 1 Hz
// Healthy = up this long with the radio task exchanging and the fusion task
// ticking. Then: cancel the bootloader's rollback, and only then let the
// swarm have this image (canary rule).
constexpr int64_t kHealthyAfterUs = 10000000;
constexpr uint32_t kHealthyFusionTicks = 1500;  // 7.5 s at 200 Hz
// A new image that isn't healthy by then is reset by the RTC watchdog, and
// the reset rolls it back.
constexpr uint32_t kHealthDeadlineS = 30;
constexpr int kBootPin = 0;  // BOOT button: held at reset -> loader

uint16_t g_id;
imu::Present g_imu;
bool g_radio_ok = false;
QueueHandle_t g_radio_events;
bool g_capture = false;  // stream raw imu/mag records (characterization)
uint32_t g_dropped = 0;  // output lines dropped (serial TX buffer full)
// Staged settings hand-off between the radio task and the fusion task.
portMUX_TYPE g_settings_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_fw_ready = false;
volatile uint32_t g_fusion_ticks = 0;

double Secs(int64_t t_us) { return t_us / 1e6; }

// Non-blocking print: drop the line rather than stall the fusion loop.
void Emit(const char *line, int len)
{
  if (!netcmd::WriteLine(line, len, false))
    g_dropped++;
}

#define EMITF(...)                                         \
  do                                                       \
  {                                                        \
    char _buf[512];                                        \
    int _n = snprintf(_buf, sizeof(_buf), __VA_ARGS__);    \
    if (_n > 0 && _n < (int)sizeof(_buf))                  \
      Emit(_buf, _n);                                      \
  } while (0)

uint16_t IdFromMac()
{
  uint64_t mac = ESP.getEfuseMac();  // byte 0 of the MAC in the low byte
  uint16_t id = (uint16_t)((((mac >> 32) & 0xFF) << 8) | ((mac >> 40) & 0xFF));
  if (id == 0 || id == 0xFFFF)
    id ^= 0x5A5A;
  return id;
}

// ---- settings --------------------------------------------------------------

// Radio task: swarm_net changed something.
void OnNetChanges(uint32_t bits)
{
  netstore::NoteChanges(bits);
  if (bits & (SWARM_NET_CHANGED_SWARM | SWARM_NET_CHANGED_OVERRIDES))
  {
    portENTER_CRITICAL(&g_settings_mux);
    int antenna_delay = swarm_node_stage_settings();
    portEXIT_CRITICAL(&g_settings_mux);
    radio::SetAntennaDelay(antenna_delay);
  }
  if (bits & SWARM_NET_CHANGED_FW_READY)
    g_fw_ready = true;
}

// Set (or clear) one override on this node, by registry id.
void LocalOverride(uint16_t id, uint32_t raw, bool set)
{
  static NetCall call;
  call.kind = NetCall::kLocalOverride;
  call.a = id;
  call.value = raw;
  call.flag = set;
  radio::Call(call);
}

uint32_t F32(float f)
{
  uint32_t r;
  memcpy(&r, &f, 4);
  return r;
}

// Earlier firmware kept survey / magcal / antenna delay in NVS: import them
// once as device overrides (the swarm_net layers are the source of truth).
void ImportLegacyNvs()
{
  if (store::Exists(store::kOverrides))
    return;
  Preferences prefs;
  if (!prefs.begin("node", true))
    return;
  if (prefs.isKey("survey"))
  {
    float p[4];
    prefs.getBytes("survey", p, sizeof(p));
    LocalOverride(401, 1, true);
    for (int i = 0; i < 3; i++)
      LocalOverride(402 + i, F32(p[i]), true);
    LocalOverride(405, F32(p[3]), true);
  }
  if (prefs.isKey("magcal"))
  {
    float c[12];
    prefs.getBytes("magcal", c, sizeof(c));
    LocalOverride(410, 1, true);
    for (int i = 0; i < 12; i++)
      LocalOverride(411 + i, F32(c[i]), true);
  }
  if (prefs.isKey("ant_delay"))
    LocalOverride(400, (uint32_t)prefs.getInt("ant_delay", 16350), true);
  prefs.end();
}

// ---- commands (loop task) ---------------------------------------------------

using netcmd::Ack;
using netcmd::Reply;

const char *kKinds[] = {"f32", "i32", "bool"};

void ListSettings()
{
  static NetCall call;
  call.kind = NetCall::kStatus;
  radio::Call(call);
  Reply("{\"type\":\"settings_begin\",\"id\":%u,\"generation\":%lu,\"override_seq\":%lu,\"key\":%s}\n",
        g_id, (unsigned long)call.status.settings_generation, (unsigned long)call.status.override_seq,
        call.status.has_key ? "true" : "false");
  uint16_t id;
  uint8_t name[SWARM_NODE_SETTING_NAME_MAX];
  int32_t kind;
  float mn, mx;
  uint32_t def;
  for (size_t i = 0; swarm_node_setting_info(i, &id, name, &kind, &mn, &mx, &def); i++)
  {
    call.kind = NetCall::kLayerGet;
    call.a = id;
    int layers = radio::Call(call);
    uint32_t over;
    memcpy(&over, call.data, 4);
    uint32_t swarm = call.value;
    uint32_t value = (layers & 2) ? over : (layers & 1) ? swarm : def;
    char v[32], d[32], sw[32] = "null";
    v[swarm_node_setting_format(i, value, (uint8_t *)v, sizeof(v) - 1)] = 0;
    d[swarm_node_setting_format(i, def, (uint8_t *)d, sizeof(d) - 1)] = 0;
    if (layers & 1)
      sw[swarm_node_setting_format(i, swarm, (uint8_t *)sw, sizeof(sw) - 1)] = 0;
    Reply("{\"type\":\"setting\",\"id\":%u,\"name\":\"%s\",\"kind\":\"%s\",\"min\":%g,\"max\":%g,"
          "\"default\":%s,\"swarm\":%s,\"value\":%s,\"source\":\"%s\"}\n",
          id, name, kKinds[kind], mn, mx, d, sw, v,
          (layers & 2) ? "override" : (layers & 1) ? "swarm" : "default");
  }
  Reply("{\"type\":\"settings_end\"}\n");
}

// {"cmd":"set","name":"est.gate","value":5} / {"cmd":"clear","name":..} /
// {"cmd":"clear","all":true}: overrides on this node.
void SetOrClear(const char *line, bool set)
{
  uint16_t id;
  uint8_t nm[SWARM_NODE_SETTING_NAME_MAX];
  int32_t kind;
  float mn, mx;
  uint32_t def, raw = 0;
  if (!set && netcmd::Flag(line, "all"))
  {
    for (size_t i = 0; swarm_node_setting_info(i, &id, nm, &kind, &mn, &mx, &def); i++)
      LocalOverride(id, 0, false);
    return Ack("clear", true);
  }
  char name[SWARM_NODE_SETTING_NAME_MAX], value[40];
  int n = netcmd::String(line, "name", name, sizeof(name));
  int i = n > 0 ? swarm_node_setting_find((const uint8_t *)name, n) : -1;
  if (i < 0)
    return Ack(set ? "set" : "clear", false, ",\"error\":\"unknown setting\"");
  swarm_node_setting_info(i, &id, nm, &kind, &mn, &mx, &def);
  if (set)
  {
    int vn = netcmd::Raw(line, "value", value, sizeof(value));
    double v = 0;
    char *end = value;
    bool parsed = false;
    if (vn > 0 && strcmp(value, "true") == 0)
      v = 1, parsed = true;
    else if (vn > 0 && strcmp(value, "false") == 0)
      v = 0, parsed = true;
    else if (vn > 0)
    {
      v = strtod(value, &end);
      parsed = end != value && *end == 0;
    }
    if (!parsed || !swarm_node_setting_from_number(i, v, &raw))
      return Ack("set", false, ",\"error\":\"bad or out-of-range value\"");
  }
  LocalOverride(id, raw, set);
  Ack(set ? "set" : "clear", true);
}

void HandleCommand(const char *line)
{
  using netcmd::IsCmd;
  if (netcmd::Handle(line))
    return;
  if (IsCmd(line, "settings"))
  {
    ListSettings();
  }
  else if (IsCmd(line, "set") || IsCmd(line, "clear"))
  {
    SetOrClear(line, IsCmd(line, "set"));
  }
  else if (IsCmd(line, "survey"))
  {
    // {"cmd":"survey","p":[x,y,z],"sigma":0.02}: a fixed, known position.
    float p[4] = {0, 0, 0, 0.02f};
    if (netcmd::FloatArray(line, "p", p, 3) != 3)
      return Ack("survey", false, ",\"error\":\"need p:[x,y,z]\"");
    netcmd::Number(line, "sigma", &p[3]);
    for (int i = 0; i < 3; i++)
      LocalOverride(402 + i, F32(p[i]), true);
    LocalOverride(405, F32(p[3]), true);
    LocalOverride(401, 1, true);
    Ack("survey", true);
  }
  else if (IsCmd(line, "unsurvey"))
  {
    LocalOverride(401, 0, true);
    Ack("unsurvey", true);
  }
  else if (IsCmd(line, "magcal"))
  {
    // {"cmd":"magcal","hard":[3],"soft":[9 row-major]} from
    // `characterize mag`; soft defaults to identity.
    float c[12] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
    if (netcmd::FloatArray(line, "hard", c, 3) != 3)
      return Ack("magcal", false, ",\"error\":\"need hard:[x,y,z]\"");
    netcmd::FloatArray(line, "soft", c + 3, 9);
    for (int i = 0; i < 12; i++)
      LocalOverride(411 + i, F32(c[i]), true);
    LocalOverride(410, 1, true);
    Ack("magcal", true);
  }
  else if (IsCmd(line, "antenna_delay"))
  {
    long v;
    if (!netcmd::Int(line, "value", &v) || v < 0 || v > 65535)
      return Ack("antenna_delay", false);
    LocalOverride(400, (uint32_t)v, true);
    Ack("antenna_delay", true);
  }
  else if (IsCmd(line, "net"))
  {
    netcmd::EmitStatus(g_id, "main");
  }
  else if (IsCmd(line, "capture"))
  {
    float on = 1;
    netcmd::Number(line, "on", &on);
    g_capture = on != 0;
    Ack("capture", true);
  }
  else if (IsCmd(line, "reboot_loader"))
  {
    store::WriteU32(store::kStayInLoader, 1);
    Ack("reboot_loader", true);
    Serial.flush();
    boot::RestartIntoLoader();
  }
  else if (IsCmd(line, "reboot"))
  {
    Ack("reboot", true);
    Serial.flush();
    ESP.restart();
  }
  else
  {
    Ack("unknown", false);
  }
}

// ---- output ---------------------------------------------------------------

void EmitSelf(int64_t t)
{
  float s[SWARM_NODE_STATE_LEN];
  if (!swarm_node_state(s))
    return;
  EMITF("{\"t\":%.4f,\"type\":\"self\",\"id\":%u,\"p\":[%.4f,%.4f,%.4f],\"v\":[%.3f,%.3f,%.3f],"
        "\"q\":[%.4f,%.4f,%.4f,%.4f],\"var\":%.4g,\"lock\":%.2f}\n",
        Secs(t), g_id, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11]);
}

void EmitSwarm(int64_t t)
{
  char buf[1536];
  float s[SWARM_NODE_STATE_LEN];
  bool ready = swarm_node_state(s);
  int n = snprintf(buf, sizeof(buf),
                   "{\"t\":%.3f,\"type\":\"swarm\",\"id\":%u,\"nodes\":[{\"id\":%u,\"self\":true,"
                   "\"ready\":%s,\"p\":[%.3f,%.3f,%.3f],\"var\":%.4g,\"lock\":%.2f,\"age\":0}",
                   Secs(t), g_id, g_id, ready ? "true" : "false", s[0], s[1], s[2], s[10], s[11]);
  float pe[SWARM_NODE_PEER_LEN];
  for (int i = 0; i < SWARM_NODE_MAX_PEERS && n < (int)sizeof(buf) - 200; i++)
  {
    uint32_t id = swarm_node_peer(i, t, pe);
    if (!id)
      continue;
    n += snprintf(buf + n, sizeof(buf) - n,
                  ",{\"id\":%u,\"p\":[%.3f,%.3f,%.3f],\"v\":[%.2f,%.2f,%.2f],\"var\":%.4g,"
                  "\"lock\":%.2f,\"age\":%.3f,\"flags\":%d,\"seq\":%u}",
                  id, pe[0], pe[1], pe[2], pe[3], pe[4], pe[5], pe[6], pe[7], pe[8], (int)pe[9],
                  (unsigned)pe[10]);
  }
  n += snprintf(buf + n, sizeof(buf) - n, "]}\n");
  if (n < (int)sizeof(buf))
    Emit(buf, n);
}

void EmitStats(int64_t t, int64_t fusion_max_us)
{
  RadioStats r = radio::Stats();
  EMITF("{\"t\":%.3f,\"type\":\"stats\",\"id\":%u,\"slot\":%u,\"peers\":%u,\"sync_master\":%u,"
        "\"sync_jump_us\":%ld,\"polls\":%lu,\"ranges_init\":%lu,\"ranges_resp\":%lu,"
        "\"timeouts\":%lu,\"rx_errors\":%lu,\"foreign\":%lu,\"beacons\":%lu,"
        "\"fusion_max_us\":%ld,\"dropped\":%lu,\"heap\":%lu}\n",
        Secs(t), g_id, r.slot, r.peers, r.sync_master, (long)r.last_sync_jump_us,
        (unsigned long)r.polls_tx, (unsigned long)r.ranges_init, (unsigned long)r.ranges_resp,
        (unsigned long)r.timeouts, (unsigned long)r.rx_errors, (unsigned long)r.foreign,
        (unsigned long)r.beacons, (long)fusion_max_us, (unsigned long)g_dropped,
        (unsigned long)ESP.getFreeHeap());
}

void EmitPackets()
{
  NetPacket pk;
  while (radio::NextPacket(&pk))
  {
    char hex[2 * sizeof(pk.data) + 1];
    for (int i = 0; i < pk.len; i++)
      snprintf(hex + 2 * i, 3, "%02x", pk.data[i]);
    hex[2 * pk.len] = 0;
    // Blocking (loop task): the host is waiting for these.
    netcmd::Reply("{\"t\":%.3f,\"type\":\"mesh\",\"origin\":%u,\"dest\":%u,\"port\":%u,\"hops\":%u,\"rssi\":%.1f,"
          "\"hex\":\"%s\"}\n",
          Secs(esp_timer_get_time()), pk.origin, pk.dest, pk.port, pk.hops, pk.rssi, hex);
  }
}

// ---- fusion task ----------------------------------------------------------

void FusionTask(void *)
{
  TickType_t wake = xTaskGetTickCount();
  uint32_t tick = 0;
  int64_t fusion_max_us = 0;
  RadioEvent ev;
  uint8_t advert[SWARM_NODE_ADVERT_MAX];
  for (;;)
  {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kFusionPeriodUs / 1000));
    int64_t t0 = esp_timer_get_time();

    portENTER_CRITICAL(&g_settings_mux);
    bool applied = swarm_node_take_settings();
    portEXIT_CRITICAL(&g_settings_mux);
    if (applied)
      EMITF("{\"t\":%.3f,\"type\":\"settings_applied\"}\n", Secs(t0));

    float a[3], g[3];
    if (g_imu.mpu && imu::ReadMotion(a, g))
    {
      swarm_node_imu(t0, a, g);
      if (g_capture)
        EMITF("{\"t\":%.4f,\"type\":\"imu\",\"accel\":[%.4f,%.4f,%.4f],\"gyro\":[%.5f,%.5f,%.5f]}\n",
              Secs(t0), a[0], a[1], a[2], g[0], g[1], g[2]);
    }
    else
    {
      swarm_node_tick(t0);
    }
    if (g_imu.mag && tick % kMagEvery == 0)
    {
      float m[3];
      if (imu::ReadMag(m))
      {
        swarm_node_mag(t0, m);
        if (g_capture)
          EMITF("{\"t\":%.4f,\"type\":\"mag\",\"mag\":[%.1f,%.1f,%.1f]}\n", Secs(t0), m[0], m[1], m[2]);
      }
    }

    while (xQueueReceive(g_radio_events, &ev, 0) == pdTRUE)
    {
      if (ev.kind == RadioEvent::kAdvert)
      {
        swarm_node_advert_rx(ev.t_us, ev.data, ev.len);
      }
      else
      {
        int code = swarm_node_range(ev.t_us, ev.peer, ev.range_m);
        EMITF("{\"t\":%.4f,\"type\":\"range\",\"peer\":%u,\"range_m\":%.4f,\"rssi\":%.1f,"
              "\"fp_rssi\":%.1f,\"init\":%d,\"result\":%d}\n",
              Secs(ev.t_us), ev.peer, ev.range_m, ev.rssi, ev.fp_rssi, ev.initiator, code);
      }
    }

    static bool cal_reported = false;
    float cal[5];
    if (!cal_reported && swarm_node_imu_cal(cal))
    {
      cal_reported = true;
      EMITF("{\"t\":%.3f,\"type\":\"imu_cal\",\"accel_scale\":%.4f,\"gyro_bias\":[%.5f,%.5f,"
            "%.5f],\"still\":%s}\n",
            Secs(t0), cal[0], cal[1], cal[2], cal[3], cal[4] > 0 ? "true" : "false");
    }
    if (tick % kAdvertEvery == 0)
    {
      size_t n = swarm_node_advert_tx(advert, sizeof(advert));
      if (n)
        radio::SetAdvert(advert, n);
    }
    int64_t now = esp_timer_get_time();
    if (now - t0 > fusion_max_us)
      fusion_max_us = now - t0;
    if (tick % kSelfEvery == 0)
      EmitSelf(now);
    if (tick % kSwarmEvery == 0)
      EmitSwarm(now);
    if (tick % kStatsEvery == 0)
    {
      EmitStats(now, fusion_max_us);
      fusion_max_us = 0;
    }
    tick++;
    g_fusion_ticks = tick;
  }
}

void EmitBootInfo(const char *what, bool ok)
{
  netcmd::Reply("{\"t\":%.3f,\"type\":\"%s\",\"ok\":%s}\n", millis() / 1000.0, what, ok ? "true" : "false");
}

}  // namespace

// Arduino's startup marks a pending-verify app valid unless this says
// otherwise; the node marks itself valid only once it's healthy (loop()),
// so a main that crashes early rolls back to the loader.
extern "C" bool verifyRollbackLater() { return true; }

void setup()
{
  // First: a fresh install that hangs anywhere from here on gets reset (and
  // so rolled back) by the RTC watchdog.
  bool pending = boot::PendingVerify();
  if (pending)
    boot::ArmHealthDeadline(kHealthDeadlineS);
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(8192);
  Serial.begin(kBaud);
  delay(50);
  g_id = IdFromMac();
  netcmd::Init();
  netcmd::Reply("{\"t\":%.3f,\"type\":\"health_deadline\",\"pending\":%s,\"armed_s\":%lu,"
                "\"rwdt_config0\":\"0x%08lx\",\"slow_hz\":%lu}\n",
                millis() / 1000.0, pending ? "true" : "false",
                (unsigned long)(pending ? kHealthDeadlineS : 0), (unsigned long)boot::HealthDeadlineConfig(),
                (unsigned long)rtc_clk_slow_freq_get_hz());

  // BOOT held at reset: back to the loader (recovery).
  pinMode(kBootPin, INPUT_PULLUP);
  if (digitalRead(kBootPin) == LOW)
  {
    delay(500);
    if (digitalRead(kBootPin) == LOW)
    {
      store::Mount();
      store::WriteU32(store::kStayInLoader, 1);
      boot::RestartIntoLoader();
    }
  }

  bool lfs = store::Mount();
  swarm_crypto::SelfTest st = swarm_crypto::RunSelfTest();
  Serial.printf("{\"t\":%.3f,\"type\":\"crypto_self_test\",\"hmac\":%s,\"p256\":%s}\n", millis() / 1000.0,
                st.hmac ? "true" : "false", st.p256 ? "true" : "false");
  SwarmFlashOps staging;
  bool staging_ok = boot::StagingOps(&staging);
  uint32_t running = store::ReadU32(store::kInstalled, 0);
  swarm_net_init(g_id, nullptr, fw_key::kPublic, running, esp_random(), &staging);
  netstore::Load();
  ImportLegacyNvs();  // radio not running yet: LocalOverride runs inline

  // Settings before the radio: they carry its antenna delay.
  int antenna_delay = swarm_node_stage_settings();
  // Radio before IMU: the DW3000 samples GPIO straps at reset, and two of
  // its GPIOs share lines with the I2C bus (see pins.h).
  g_radio_events = xQueueCreate(32, sizeof(RadioEvent));
  g_radio_ok = radio::Init(antenna_delay);
  g_imu = imu::Begin();
#if defined(SWARM_FAULT_TEST) && SWARM_FAULT_TEST == 1
  // Test image (//apps/node:node_hang_esp32): wedge the boot.
  netcmd::Reply("{\"type\":\"fault_test\",\"fault\":\"hang\"}\n");
  for (;;)
  {
  }
#elif defined(SWARM_FAULT_TEST) && SWARM_FAULT_TEST == 2
  // Test image (//apps/node:node_crash_esp32): crash during boot.
  netcmd::Reply("{\"type\":\"fault_test\",\"fault\":\"crash\"}\n");
  Serial.flush();
  abort();
#endif
  swarm_node_init(g_id, g_imu.mpu, 0.0f);
  swarm_node_stage_settings();
  swarm_node_take_settings();
  // Persist an NVS import (the radio task's idle hook writes after 1 s).
  netstore::NoteChanges(swarm_net_take_changes());

  uint8_t advert[SWARM_NODE_ADVERT_MAX];
  size_t n = swarm_node_advert_tx(advert, sizeof(advert));
  radio::SetAdvert(advert, n);

  // The radio task busy-polls the DW3000 on core 0 but yields a tick every
  // 100 ms, so the idle task still feeds the watchdog. (disableCore0WDT()
  // makes IDF 5's watchdog idle hook log "task not found" every tick —
  // ~60 kB/s of errors that swamp the serial link.)
  NetHooks hooks;
  hooks.on_changes = OnNetChanges;
  hooks.on_idle = netstore::Idle;
  if (g_radio_ok)
    radio::Start(g_id, g_radio_events, hooks);
  Serial.printf("{\"t\":%.3f,\"type\":\"boot\",\"fw\":\"node\",\"app\":\"main\",\"version\":%lu,\"id\":%u,"
                "\"imu\":%s,\"mag\":%s,\"radio\":%s,\"antenna_delay\":%d,\"lfs\":%s,\"staging\":%s,"
                "\"baud\":%lu}\n",
                millis() / 1000.0, (unsigned long)running, g_id, g_imu.mpu ? "true" : "false",
                g_imu.mag ? "true" : "false", g_radio_ok ? "true" : "false", antenna_delay,
                lfs ? "true" : "false", staging_ok ? "true" : "false", (unsigned long)kBaud);
  xTaskCreatePinnedToCore(FusionTask, "fusion", 12288, nullptr, 10, nullptr, 1);
}

// Arduino loop task (core 1, low priority): command lines, mesh packets for
// the host, network status, health and install handoff.
void loop()
{
  static int64_t next_net = 0;
  static bool valid = false;
  static int64_t fw_ready_at = 0;
  while (const char *line = netcmd::ReadLine())
    HandleCommand(line);
  EmitPackets();
  int64_t now = esp_timer_get_time();
  if (now >= next_net)
  {
    next_net = now + 1000000;
    netcmd::EmitStatus(g_id, "main");
  }
  if (!valid && g_radio_ok && now > kHealthyAfterUs && g_fusion_ticks > kHealthyFusionTicks)
  {
    RadioStats r = radio::Stats();
    if (r.polls_tx + r.ranges_resp + r.beacons > 0)
    {
      valid = true;
      boot::MarkValid();
      boot::DisarmHealthDeadline();
      static NetCall call;
      call.kind = NetCall::kValidated;
      radio::Call(call);
      EmitBootInfo("marked_valid", true);
    }
  }
  if (g_fw_ready && !fw_ready_at)
    fw_ready_at = now;
  // A newer verified image is staged: give the radio a moment to tell the
  // neighbours, then let the loader install it.
  if (fw_ready_at && now - fw_ready_at > 2000000)
  {
    store::WriteU32(store::kInstall, 1);
    EmitBootInfo("installing", true);
    Serial.flush();
    boot::RestartIntoLoader();
  }
  delay(2);
}
