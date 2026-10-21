// Test-mule firmware: bring up the DWM3000 over SPI and run DS-TWR ranging
// between two mules, streaming newline-delimited JSON on the USB serial port.
//
// Role is fixed per binary (BUILD local_defines):
//   MULE_ROLE_INITIATOR — polls the responder every kRangePeriodMs and emits a
//                         "range" record per completed exchange
//   MULE_ROLE_RESPONDER — answers polls addressed to kResponderId
//
// Every JSON line has "t" (seconds since boot) and "type", following the
// capture schema in swarm/tools/records.py, so `swarm/tools/capture.py` output
// feeds `characterize range` directly:
//   boot   — role + pin map, once at startup
//   devid  — raw DEV_ID reads during bring-up (0xDECA0302/0xDECA0312 = OK)
//   ready  — module initialised (with die temperature)
//   range  — one completed exchange (initiator only)
//   stats  — once a second: exchange counts and failures by stage
//   error  — bring-up failure with the stage that failed
// Non-JSON lines come from the shared driver's own logging.
#include <Arduino.h>

#include "apps/mule/pins.h"
#include "libs/dw3000/dw3000.h"

#if defined(MULE_ROLE_INITIATOR) == defined(MULE_ROLE_RESPONDER)
#error "define exactly one of MULE_ROLE_INITIATOR / MULE_ROLE_RESPONDER"
#endif

namespace {

constexpr int kInitiatorId = 10;
constexpr int kResponderId = 1;
// Untuned default (same as anchor/tag); range bias captures calibrate it.
constexpr int kTxAntennaDelay = 16350;
constexpr uint32_t kRangePeriodMs = 50;  // 20 Hz
constexpr uint32_t kRxTimeoutMs = 20;
constexpr uint32_t kStatsPeriodMs = 1000;
// Consecutive failed exchanges before the module is fully re-initialised
// (2 s at 20 Hz): a backstop in case the radio wedges in a way that idling it
// before each exchange does not clear.
constexpr int kRecoverAfterFailures = 40;
constexpr int kDevIdAttempts = 5;

#ifdef MULE_ROLE_INITIATOR
constexpr const char *kRole = "initiator";
constexpr int kSelfId = kInitiatorId;
#else
constexpr const char *kRole = "responder";
constexpr int kSelfId = kResponderId;
#endif

bool g_ready = false;
int g_consecutive_failures = 0;

// Per-second counters, reset after each stats line.
struct Stats
{
  uint32_t ok = 0;
  uint32_t timeout = 0;     // no frame within kRxTimeoutMs
  uint32_t rx_error = 0;    // receiver error (bad frame)
  uint32_t protocol = 0;    // error frame / unexpected stage
} g_stats;

double Now() { return millis() / 1000.0; }

bool DevIdOk(uint32_t id) { return id == 0xDECA0302 || id == 0xDECA0312; }

uint32_t ReadDevId() { return DWM3000.read(GEN_CFG_AES_LOW_REG, NO_OFFSET); }

void EmitBoot()
{
  Serial.printf(
      "{\"t\":%.3f,\"type\":\"boot\",\"fw\":\"mule\",\"role\":\"%s\",\"id\":%d,"
      "\"pins\":{\"sck\":%d,\"miso\":%d,\"mosi\":%d,\"cs\":%d,\"rst\":%d,\"irq\":%d,"
      "\"exton\":%d}}\n",
      Now(), kRole, kSelfId, MULE_PIN_SCK, MULE_PIN_MISO, MULE_PIN_MOSI, MULE_PIN_CS,
      MULE_PIN_RST, MULE_PIN_IRQ, MULE_PIN_EXTON);
}

void EmitError(const char *stage, uint32_t devid)
{
  Serial.printf("{\"t\":%.3f,\"type\":\"error\",\"stage\":\"%s\",\"devid\":\"0x%08lX\"}\n",
                Now(), stage, (unsigned long)devid);
}

void EmitStats()
{
  Serial.printf("{\"t\":%.3f,\"type\":\"stats\",\"role\":\"%s\",\"ok\":%lu,\"timeout\":%lu,"
                "\"rx_error\":%lu,\"protocol\":%lu}\n",
                Now(), kRole, (unsigned long)g_stats.ok, (unsigned long)g_stats.timeout,
                (unsigned long)g_stats.rx_error, (unsigned long)g_stats.protocol);
  g_stats = Stats();
}

// Wake the DW3000 from sleep: it wakes on SPICSn held low for >500 us or on a
// WAKEUP pulse. Seen once on a freshly plugged-in board: DEV_ID read all zeros
// through repeated hard resets until something held CS low.
void WakeModule()
{
  pinMode(MULE_PIN_WAKEUP, OUTPUT);
  digitalWrite(MULE_PIN_WAKEUP, HIGH);
  digitalWrite(MULE_PIN_CS, LOW);
  delay(1);
  digitalWrite(MULE_PIN_CS, HIGH);
  digitalWrite(MULE_PIN_WAKEUP, LOW);
  delay(2);
}

// Hard-reset the module and read DEV_ID until it answers, waking it first on
// retries. Logs every raw read: 0x00000000 / 0xFFFFFFFF point at MISO stuck
// low/high (wiring or CS, or a sleeping module), other garbage at clocking or
// mode problems.
bool ProbeDevId()
{
  for (int attempt = 1; attempt <= kDevIdAttempts; attempt++)
  {
    if (attempt > 1)
      WakeModule();
    DWM3000.hardReset();
    delay(200);
    uint32_t id = ReadDevId();
    Serial.printf("{\"t\":%.3f,\"type\":\"devid\",\"attempt\":%d,\"value\":\"0x%08lX\",\"ok\":%s}\n",
                  Now(), attempt, (unsigned long)id, DevIdOk(id) ? "true" : "false");
    if (DevIdOk(id))
      return true;
  }
  return false;
}

bool BringUp()
{
  if (!ProbeDevId())
  {
    EmitError("devid", ReadDevId());
    return false;
  }
  if (!DWM3000.checkForIDLE())
  {
    EmitError("idle_before_reset", ReadDevId());
    return false;
  }
  DWM3000.softReset();
  delay(200);
  if (!DWM3000.checkForIDLE())
  {
    EmitError("idle_after_reset", ReadDevId());
    return false;
  }
  DWM3000.init();
  DWM3000.setupGPIO();
  DWM3000.setTXAntennaDelay(kTxAntennaDelay);
  DWM3000.setSenderID(kSelfId);
  // Temperature is read once here: the read reconfigures the radio, so it
  // must not interleave with an exchange.
  Serial.printf("{\"t\":%.3f,\"type\":\"ready\",\"devid\":\"0x%08lX\",\"temp_c\":%.2f,"
                "\"antenna_delay\":%d}\n",
                Now(), (unsigned long)ReadDevId(), DWM3000.getTempInC(),
                DWM3000.getTXAntennaDelay());
  DWM3000.configureAsTX();
  DWM3000.clearSystemStatus();
  return true;
}

// Polls for a received frame. Returns 1 on a good frame, -1 on a receiver
// error, 0 if nothing arrived (yet).
int PollRx()
{
  int status = DWM3000.receivedFrameSucc();
  if (status == 0)
    return 0;
  DWM3000.clearSystemStatus();
  if (status != 1)
  {
    g_stats.rx_error++;
    return -1;
  }
  return 1;
}

// Blocks until a frame arrives or kRxTimeoutMs passes; counts failures.
bool AwaitFrame()
{
  uint32_t start = millis();
  while (millis() - start < kRxTimeoutMs)
  {
    int r = PollRx();
    if (r == 1)
      return true;
    if (r < 0)
      return false;
  }
  g_stats.timeout++;
  return false;
}

#ifdef MULE_ROLE_INITIATOR

uint32_t g_seq = 0;

// One full DS-TWR exchange against the responder:
//   Poll(1) -> Response(2) -> Final(3) -> Report(t_roundB, t_replyB)
// Returns false on any failure (already counted in g_stats).
bool RangeOnce()
{
  uint32_t seq = g_seq++;
  // Start from IDLE with clean status: a timed-out exchange leaves the
  // receiver on and stale TX/RX flags set, which otherwise wedges every
  // later exchange (the TX-done check passes on the stale flag).
  DWM3000.forceTRXOff();
  DWM3000.clearSystemStatus();
  DWM3000.setDestinationID(kResponderId);
  DWM3000.ds_sendFrame(1);
  long long tx = DWM3000.readTXTimestamp();  // T1

  if (!AwaitFrame())
    return false;
  if (DWM3000.ds_isErrorFrame() || DWM3000.ds_getStage() != 2)
  {
    g_stats.protocol++;
    return false;
  }
  long long rx = DWM3000.readRXTimestamp();  // T4
  DWM3000.ds_sendFrame(3);
  int t_round_a = rx - tx;
  tx = DWM3000.readTXTimestamp();  // T5
  int t_reply_a = tx - rx;

  if (!AwaitFrame())
    return false;
  if (DWM3000.ds_isErrorFrame())
  {
    g_stats.protocol++;
    return false;
  }
  int clock_offset = DWM3000.getRawClockOffset();
  int t_round_b = DWM3000.read(0x12, 0x04);
  int t_reply_b = DWM3000.read(0x12, 0x08);
  int tof = DWM3000.ds_processRTInfo(t_round_a, t_reply_a, t_round_b, t_reply_b,
                                     clock_offset);
  double range_m = DWM3000.convertToCM(tof) / 100.0;
  double rssi = DWM3000.getSignalStrength();
  double fp_rssi = DWM3000.getFirstPathSignalStrength();
  g_stats.ok++;
  Serial.printf("{\"t\":%.3f,\"type\":\"range\",\"peer\":%d,\"seq\":%lu,\"range_m\":%.4f,"
                "\"rssi\":%.2f,\"fp_rssi\":%.2f,\"clock_offset\":%d,\"tof\":%d,"
                "\"t_round_a\":%d,\"t_reply_a\":%d,\"t_round_b\":%d,\"t_reply_b\":%d}\n",
                Now(), kResponderId, (unsigned long)seq, range_m, rssi, fp_rssi,
                clock_offset, tof, t_round_a, t_reply_a, t_round_b, t_reply_b);
  return true;
}

void RoleLoop()
{
  static uint32_t last = 0;
  if (millis() - last < kRangePeriodMs)
    return;
  last = millis();
  g_consecutive_failures = RangeOnce() ? 0 : g_consecutive_failures + 1;
}

#else  // MULE_ROLE_RESPONDER

// ds_sendRTInfo returns as soon as the TX is queued; wait for TX-done before
// anything idles the radio, or the Report is cut off mid-frame.
void WaitTxDone()
{
  uint32_t start = micros();
  while (!DWM3000.sentFrameSucc() && micros() - start < 5000)
  {
  }
}

void Listen()
{
  DWM3000.forceTRXOff();
  DWM3000.clearSystemStatus();
  DWM3000.standardRX();
}

// Mirror of RangeOnce: answer a Poll, await the Final, send the Report.
void RoleLoop()
{
  static bool listening = false;
  if (!listening)
  {
    Listen();
    listening = true;
  }
  int r = PollRx();
  if (r == 0)
    return;
  listening = false;  // every path below ends with a fresh Listen()
  if (r < 0 || DWM3000.getDestinationID() != kSelfId)
    return;
  if (DWM3000.ds_isErrorFrame() || DWM3000.ds_getStage() != 1)
  {
    g_stats.protocol++;
    return;
  }

  DWM3000.ds_sendFrame(2);
  long long rx = DWM3000.readRXTimestamp();  // T2
  long long tx = DWM3000.readTXTimestamp();  // T3
  int t_reply_b = tx - rx;

  if (!AwaitFrame())
    return;
  if (DWM3000.ds_isErrorFrame() || DWM3000.ds_getStage() != 3)
  {
    g_stats.protocol++;
    return;
  }
  rx = DWM3000.readRXTimestamp();  // T6
  int t_round_b = rx - tx;
  DWM3000.ds_sendRTInfo(t_round_b, t_reply_b);
  WaitTxDone();
  g_stats.ok++;
  g_consecutive_failures = 0;
}

#endif

}  // namespace

void setup()
{
  Serial.begin(115200);
  delay(100);
  pinMode(MULE_PIN_IRQ, INPUT);
  pinMode(MULE_PIN_EXTON, INPUT);
  EmitBoot();

  DWM3000.setPins({MULE_PIN_SCK, MULE_PIN_MISO, MULE_PIN_MOSI, MULE_PIN_CS, MULE_PIN_RST});
  DWM3000.begin();
  g_ready = BringUp();
}

void loop()
{
  static uint32_t last_stats = 0;
  if (!g_ready)
  {
    // Keep retrying so a reseated shield recovers without a reflash.
    delay(1000);
    g_ready = BringUp();
    return;
  }
  RoleLoop();
#ifdef MULE_ROLE_INITIATOR
  // The responder legitimately goes quiet for long stretches (unplugged,
  // out of range), so only the initiator uses the failure backstop.
  if (g_consecutive_failures >= kRecoverAfterFailures)
  {
    Serial.printf("{\"t\":%.3f,\"type\":\"recover\",\"failures\":%d}\n", Now(),
                  g_consecutive_failures);
    g_consecutive_failures = 0;
    g_ready = BringUp();
    return;
  }
#endif
  if (millis() - last_stats >= kStatsPeriodMs)
  {
    last_stats = millis();
    EmitStats();
  }
}
