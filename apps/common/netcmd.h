// USB serial commands shared by the node and the loader (newline JSON, run
// on the Arduino loop task): the swarm network ones (HMAC key, settings
// bundle, mesh send, firmware push, status) plus the small JSON field
// helpers both apps' command parsers use.
#ifndef APPS_COMMON_NETCMD_H
#define APPS_COMMON_NETCMD_H

#include <stdint.h>

namespace netcmd {

// A firmware page as hex (fw_page) plus JSON around it.
constexpr int kLineMax = 8400;

// Call once in setup, before the tasks start.
void Init();
// Every serial line from every task goes through here, whole: under
// backpressure a blocking write would otherwise let another task's line
// land in the middle of it. block=false (real-time tasks): drop the line
// (false) rather than wait for the port or for room in its buffer.
bool WriteLine(const char *line, int len, bool block);

// Handle a network command; false if `line` isn't one.
bool Handle(const char *line);
// One "net" status line (app: "main" / "loader").
void EmitStatus(uint16_t id, const char *app);
// Read serial into a line buffer; returns a complete line or nullptr.
const char *ReadLine();

// JSON helpers (flat objects, no escapes).
bool IsCmd(const char *line, const char *name);
const char *Field(const char *line, const char *key);
bool Int(const char *line, const char *key, long *out);
bool Number(const char *line, const char *key, float *out);
bool Flag(const char *line, const char *key);
int String(const char *line, const char *key, char *out, int cap);
int Raw(const char *line, const char *key, char *out, int cap);
int Hex(const char *line, const char *key, uint8_t *out, int cap);
int FloatArray(const char *line, const char *key, float *out, int n);
void Ack(const char *cmd, bool ok, const char *detail = "");
// Blocking printf to serial (command replies).
void Reply(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

}  // namespace netcmd

#endif  // APPS_COMMON_NETCMD_H
