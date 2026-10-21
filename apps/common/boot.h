// Partitions and boot control for the loader / main split
// (docs/swarm-network.md "Boot flow"): the staging partition as swarm_net's
// flash, the IDF OTA calls that pick what boots next, and installing a
// staged image into main.
#ifndef APPS_COMMON_BOOT_H
#define APPS_COMMON_BOOT_H

#include <stdint.h>

#include "swarm/net/swarm_net.h"

namespace boot {

// The staging partition as swarm_net flash ops; false (and capacity 0) if
// the partition table has none.
bool StagingOps(SwarmFlashOps *ops);

// What this image is.
bool RunningLoader();

// Main: running well; cancel the bootloader's rollback.
void MarkValid();
// Main, first thing at boot: the RTC watchdog (independent of both cores)
// resets the chip unless DisarmHealthDeadline() runs within `seconds`. A
// main that hangs before proving healthy therefore resets, and a reset of
// an image still pending verification rolls back to the loader. (Without
// this, only crashes roll back: a hung board just sits there.)
void ArmHealthDeadline(uint32_t seconds);
// Main: is this image still pending verification (first boots after an
// install)? Only then is a missed health deadline a reason to roll back.
bool PendingVerify();
void DisarmHealthDeadline();
// RTC watchdog config register (diagnostics: bit 31 = enabled).
uint32_t HealthDeadlineConfig();
// Main: make the next reset (now) boot the loader.
[[noreturn]] void RestartIntoLoader();
// Loader: is there a main image that isn't marked aborted?
bool MainBootable();
// Loader: boot main next (pending verify until it marks itself valid).
[[noreturn]] void RestartIntoMain();

// Loader: copy the verified staged image (manifest at staging 0, image at
// 4096) into main, verifying main's SHA-256 against `sha256` after.
bool InstallStaged(uint32_t size, const uint8_t sha256[32]);

}  // namespace boot

#endif  // APPS_COMMON_BOOT_H
