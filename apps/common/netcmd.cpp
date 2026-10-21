// See netcmd.h.
#include "apps/common/netcmd.h"

#include <Arduino.h>
#include <ctype.h>
#include <esp_timer.h>
#include <freertos/semphr.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "apps/common/netstore.h"
#include "apps/common/radio.h"
#include "swarm/net/swarm_net.h"

namespace netcmd {

namespace {
SemaphoreHandle_t g_serial_lock;
}  // namespace

void Init() { g_serial_lock = xSemaphoreCreateMutex(); }

bool WriteLine(const char *line, int len, bool block)
{
  if (!g_serial_lock)
    return Serial.write((const uint8_t *)line, len) == (size_t)len;
  if (xSemaphoreTake(g_serial_lock, block ? portMAX_DELAY : 0) != pdTRUE)
    return false;
  bool ok = true;
  if (!block && Serial.availableForWrite() < len)
    ok = false;
  else
    Serial.write((const uint8_t *)line, len);
  xSemaphoreGive(g_serial_lock);
  return ok;
}

void Reply(const char *fmt, ...)
{
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0 && n < (int)sizeof(buf))
    WriteLine(buf, n, true);
}

const char *Field(const char *line, const char *key)
{
  char pat[32];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *s = strstr(line, pat);
  if (!s || !(s = strchr(s + strlen(pat), ':')))
    return nullptr;
  s++;
  while (*s == ' ')
    s++;
  return s;
}

bool IsCmd(const char *line, const char *name)
{
  char pat[48];
  snprintf(pat, sizeof(pat), "\"cmd\":\"%s\"", name);
  return strstr(line, pat) != nullptr;
}

int FloatArray(const char *line, const char *key, float *out, int n)
{
  const char *s = Field(line, key);
  if (!s || *s != '[')
    return 0;
  s++;
  int i = 0;
  while (i < n)
  {
    char *end;
    float v = strtof(s, &end);
    if (end == s)
      break;
    out[i++] = v;
    s = end;
    while (*s == ' ' || *s == ',')
      s++;
  }
  return i;
}

bool Number(const char *line, const char *key, float *out)
{
  const char *s = Field(line, key);
  if (!s)
    return false;
  char *end;
  *out = strtof(s, &end);
  return end != s;
}

bool Int(const char *line, const char *key, long *out)
{
  const char *s = Field(line, key);
  if (!s)
    return false;
  char *end;
  *out = strtol(s, &end, 0);
  return end != s;
}

bool Flag(const char *line, const char *key)
{
  const char *s = Field(line, key);
  return s && (strncmp(s, "true", 4) == 0 || *s == '1');
}

int String(const char *line, const char *key, char *out, int cap)
{
  const char *s = Field(line, key);
  if (!s || *s != '"')
    return -1;
  s++;
  const char *e = strchr(s, '"');
  if (!e || e - s >= cap)
    return -1;
  memcpy(out, s, e - s);
  out[e - s] = 0;
  return e - s;
}

int Raw(const char *line, const char *key, char *out, int cap)
{
  const char *s = Field(line, key);
  if (!s)
    return -1;
  int n = 0;
  while (s[n] && s[n] != ',' && s[n] != '}' && s[n] != ' ' && n < cap - 1)
    n++;
  memcpy(out, s, n);
  out[n] = 0;
  return n;
}

int Hex(const char *line, const char *key, uint8_t *out, int cap)
{
  const char *s = Field(line, key);
  if (!s || *s != '"')
    return -1;
  s++;
  int n = 0;
  while (isxdigit((unsigned char)s[0]) && isxdigit((unsigned char)s[1]))
  {
    if (n == cap)
      return -1;
    char b[3] = {s[0], s[1], 0};
    out[n++] = (uint8_t)strtol(b, nullptr, 16);
    s += 2;
  }
  return *s == '"' ? n : -1;
}

void Ack(const char *cmd, bool ok, const char *detail)
{
  Reply("{\"t\":%.3f,\"type\":\"ack\",\"cmd\":\"%s\",\"ok\":%s%s}\n", esp_timer_get_time() / 1e6, cmd,
        ok ? "true" : "false", detail);
}

const char *ReadLine()
{
  static char line[kLineMax];
  static int len = 0;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (len)
      {
        line[len] = 0;
        len = 0;
        return line;
      }
    }
    else if (len < kLineMax - 1)
    {
      line[len++] = c;
    }
  }
  return nullptr;
}

void EmitStatus(uint16_t id, const char *app)
{
  static NetCall call;
  call.kind = NetCall::kStatus;
  radio::Call(call);
  const SwarmNetStatus &s = call.status;
  static const char *kFw[] = {"idle", "receiving", "ready", "serving"};
  RadioStats r = radio::Stats();
  // Neighbours' firmware, from their POLL summaries.
  char nodes[640];
  int n = 0;
  nodes[0] = 0;
  for (int i = 0; i < call.neighbour_count && n < (int)sizeof(nodes) - 96; i++)
  {
    const SwarmNetNeighbour &nb = call.neighbours[i];
    n += snprintf(nodes + n, sizeof(nodes) - n,
                  "%s{\"id\":%u,\"running\":%lu,\"staged\":%lu,\"pages\":%u,\"total\":%u}", i ? "," : "",
                  nb.id, (unsigned long)nb.fw_running, (unsigned long)nb.fw_staged, nb.fw_pages, nb.fw_total);
  }
  Reply("{\"t\":%.3f,\"type\":\"net\",\"id\":%u,\"app\":\"%s\",\"fw\":{\"running\":%lu,\"staged\":%lu,"
        "\"pages\":%u,\"total\":%u,\"status\":\"%s\",\"written\":%lu,\"served\":%lu,\"requests\":%lu,"
        "\"rejected\":%lu},\"settings\":{\"generation\":%lu,\"override_seq\":%lu,\"key\":%s,\"applied\":%lu,"
        "\"rejected_tag\":%lu,\"rejected_replay\":%lu},\"mesh\":{\"sent\":%lu,\"delivered\":%lu,"
        "\"relayed\":%lu,\"relays_cancelled\":%lu,\"duplicates\":%lu,\"retries\":%lu,\"failed\":%lu,"
        "\"frames_tx\":%lu,\"frames_rx\":%lu,\"spare_slots\":%lu},\"peers\":%u,\"nodes\":[%s]}\n",
        esp_timer_get_time() / 1e6, id, app, (unsigned long)s.fw_running, (unsigned long)s.fw_staged,
        s.fw_pages, s.fw_total, kFw[s.fw_status < 4 ? s.fw_status : 0], (unsigned long)s.fw_chunks_written,
        (unsigned long)s.fw_chunks_served, (unsigned long)s.fw_requests_sent, (unsigned long)s.fw_rejected,
        (unsigned long)s.settings_generation, (unsigned long)s.override_seq, s.has_key ? "true" : "false",
        (unsigned long)s.settings_applied, (unsigned long)s.settings_rejected_tag,
        (unsigned long)s.settings_rejected_replay, (unsigned long)s.mesh_originated,
        (unsigned long)s.mesh_delivered, (unsigned long)s.mesh_relayed,
        (unsigned long)s.mesh_relays_cancelled, (unsigned long)s.mesh_duplicates,
        (unsigned long)s.mesh_retries, (unsigned long)s.mesh_failed, (unsigned long)r.net_tx,
        (unsigned long)r.net_rx, (unsigned long)r.spare_slots, r.peers, nodes);
}

bool Handle(const char *line)
{
  static NetCall call;
  if (IsCmd(line, "key"))
  {
    // {"cmd":"key","hex":"<64 hex>"} provisions the swarm HMAC key;
    // {"cmd":"key","remove":true} removes it.
    if (Flag(line, "remove"))
      Ack("key", netstore::SetKey(nullptr));
    else
    {
      uint8_t k[32];
      if (Hex(line, "hex", k, sizeof(k)) != 32)
        Ack("key", false, ",\"error\":\"need 32-byte hex\"");
      else
        Ack("key", netstore::SetKey(k));
    }
  }
  else if (IsCmd(line, "bundle"))
  {
    // A tagged swarm-layer bundle from swarm_ctl.py: apply and spread it.
    call.kind = NetCall::kHostBundle;
    int n = Hex(line, "hex", call.data, sizeof(call.data));
    call.len = n > 0 ? n : 0;
    Ack("bundle", n > 0 && radio::Call(call) == 1);
  }
  else if (IsCmd(line, "send"))
  {
    // {"cmd":"send","dest":id,"port":p,"ack":true,"hex":".."}: a mesh
    // packet the host built (tagged override, settings GET, ...).
    long dest, port;
    call.kind = NetCall::kSend;
    int n = Hex(line, "hex", call.data, 112);
    if (!Int(line, "dest", &dest) || !Int(line, "port", &port) || n < 0)
    {
      Ack("send", false);
      return true;
    }
    call.a = (uint16_t)dest;
    call.b = (uint16_t)port;
    call.flag = Flag(line, "ack");
    call.len = n;
    int id = radio::Call(call);
    char detail[32];
    snprintf(detail, sizeof(detail), ",\"packet\":%d", id);
    Ack("send", id >= 0, detail);
  }
  else if (IsCmd(line, "fw_manifest"))
  {
    call.kind = NetCall::kHostManifest;
    int n = Hex(line, "hex", call.data, sizeof(call.data));
    call.len = n > 0 ? n : 0;
    Ack("fw_manifest", n == SWARM_NET_MANIFEST_LEN && radio::Call(call) == 1);
  }
  else if (IsCmd(line, "fw_page"))
  {
    // {"cmd":"fw_page","page":p,"hex":".."}: one 4 KB page (the last may be
    // short), pages in order.
    static uint8_t page_buf[4096];
    long page;
    int n = Hex(line, "hex", page_buf, sizeof(page_buf));
    if (!Int(line, "page", &page) || n <= 0)
    {
      Ack("fw_page", false);
      return true;
    }
    call.kind = NetCall::kHostPage;
    call.a = (uint16_t)page;
    call.ptr = page_buf;
    call.len = n;
    radio::Call(call);
    Ack("fw_page", true);
  }
  else if (IsCmd(line, "fw_chunk"))
  {
    // {"cmd":"fw_chunk","page":p,"chunk":c,"hex":".."}: pages in order.
    long page, chunk;
    call.kind = NetCall::kHostChunk;
    int n = Hex(line, "hex", call.data, 96);
    if (!Int(line, "page", &page) || !Int(line, "chunk", &chunk) || n <= 0)
    {
      Ack("fw_chunk", false);
      return true;
    }
    call.a = (uint16_t)page;
    call.b = (uint16_t)chunk;
    call.len = n;
    radio::Call(call);
    Ack("fw_chunk", true);
  }
  else
  {
    return false;
  }
  return true;
}

}  // namespace netcmd
