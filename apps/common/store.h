// Persistent files on the "lfs" partition (LittleFS through ESP-IDF's VFS,
// mounted at /lfs), written atomically: a reset mid-write leaves the old
// file, never a torn one. Holds settings layers, the HMAC key and update
// state; see docs/swarm-network.md.
#ifndef APPS_COMMON_STORE_H
#define APPS_COMMON_STORE_H

#include <stddef.h>
#include <stdint.h>

namespace store {

constexpr const char *kKey = "/lfs/hmac.key";            // 32 bytes
constexpr const char *kBundle = "/lfs/settings_swarm.bin";  // tagged bundle
constexpr const char *kOverrides = "/lfs/settings_device.bin";
constexpr const char *kInstalled = "/lfs/fw_installed";  // u32 version
constexpr const char *kInstall = "/lfs/fw_install";      // present: install staged image
constexpr const char *kStayInLoader = "/lfs/stay_in_loader";

// Mount (formatting an unreadable partition). False if there's no "lfs"
// partition (old partition table): every call below then fails quietly.
bool Mount();
bool Mounted();
// Bytes read into `out` (at most `cap`), or -1 if missing.
int Read(const char *path, uint8_t *out, size_t cap);
bool WriteAtomic(const char *path, const uint8_t *data, size_t len);
bool Exists(const char *path);
void Remove(const char *path);
uint32_t ReadU32(const char *path, uint32_t fallback);
bool WriteU32(const char *path, uint32_t value);

}  // namespace store

#endif  // APPS_COMMON_STORE_H
