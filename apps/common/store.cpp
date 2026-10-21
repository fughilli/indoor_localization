// See store.h. Same approach as splanc's player firmware: esp_littlefs's VFS
// and stdio, temp file + fsync + rename for atomic replacement (rename is a
// single LittleFS metadata commit).
#include "apps/common/store.h"

#include <esp_littlefs.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

namespace store {
namespace {

bool g_mounted = false;

}  // namespace

bool Mount()
{
  esp_vfs_littlefs_conf_t conf = {};
  conf.base_path = "/lfs";
  conf.partition_label = "lfs";
  conf.format_if_mount_failed = true;
  g_mounted = esp_vfs_littlefs_register(&conf) == ESP_OK;
  return g_mounted;
}

bool Mounted() { return g_mounted; }

int Read(const char *path, uint8_t *out, size_t cap)
{
  if (!g_mounted)
    return -1;
  FILE *f = fopen(path, "rb");
  if (!f)
    return -1;
  size_t n = fread(out, 1, cap, f);
  fclose(f);
  return (int)n;
}

bool WriteAtomic(const char *path, const uint8_t *data, size_t len)
{
  if (!g_mounted)
    return false;
  char tmp[64];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  FILE *f = fopen(tmp, "wb");
  if (!f)
    return false;
  setvbuf(f, nullptr, _IONBF, 0);
  bool ok = fwrite(data, 1, len, f) == len;
  ok &= fflush(f) == 0;
  ok &= fsync(fileno(f)) == 0;
  fclose(f);
  if (!ok)
  {
    remove(tmp);
    return false;
  }
  return rename(tmp, path) == 0;
}

bool Exists(const char *path)
{
  struct stat st;
  return g_mounted && stat(path, &st) == 0;
}

void Remove(const char *path)
{
  if (g_mounted)
    remove(path);
}

uint32_t ReadU32(const char *path, uint32_t fallback)
{
  uint8_t b[4];
  if (Read(path, b, 4) != 4)
    return fallback;
  return b[0] | b[1] << 8 | b[2] << 16 | (uint32_t)b[3] << 24;
}

bool WriteU32(const char *path, uint32_t v)
{
  uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
  return WriteAtomic(path, b, 4);
}

}  // namespace store
