// swarm_net state <-> LittleFS (store.h): the HMAC key and both settings
// layers load before the radio starts; changes are written back once
// they've been quiet for a second (coalescing bursts, sparing flash).
#ifndef APPS_COMMON_NETSTORE_H
#define APPS_COMMON_NETSTORE_H

#include <stdint.h>

namespace netstore {

// Setup, after swarm_net_init and before radio::Start.
void Load();
// Radio task (NetHooks): note what changed / write it back when quiet.
void NoteChanges(uint32_t bits);
void Idle(int64_t t_us);
// Any task: provision (32 bytes) or remove (nullptr) the HMAC key, in
// swarm_net (through radio::Call) and on flash.
bool SetKey(const uint8_t *key);

}  // namespace netstore

#endif  // APPS_COMMON_NETSTORE_H
