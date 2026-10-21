// See boot.h. The stock IDF bootloader decides between "loader" (factory)
// and "main" (ota_0) from otadata, with app rollback enabled in the prebuilt
// bootloader: a newly selected main boots pending verification and, if it
// resets before esp_ota_mark_app_valid_cancel_rollback(), is marked aborted
// and the loader boots instead.
#include "apps/common/boot.h"

#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <hal/wdt_hal.h>
#include <soc/rtc.h>
#include <soc/rtc_cntl_reg.h>
#include <string.h>

#include "apps/common/swarm_crypto.h"

namespace boot {
namespace {

constexpr uint32_t kImageOffset = 4096;  // swarm_net: manifest sector first
constexpr uint32_t kSector = 4096;

const esp_partition_t *Staging()
{
  return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x80, "staging");
}

const esp_partition_t *Main()
{
  return esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
}

const esp_partition_t *Loader()
{
  return esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr);
}

bool Erase(void *ctx, uint32_t offset, uint32_t len)
{
  return esp_partition_erase_range((const esp_partition_t *)ctx, offset, len) == ESP_OK;
}

bool Write(void *ctx, uint32_t offset, const uint8_t *data, size_t len)
{
  return esp_partition_write((const esp_partition_t *)ctx, offset, data, len) == ESP_OK;
}

bool ReadFlash(void *ctx, uint32_t offset, uint8_t *out, size_t len)
{
  return esp_partition_read((const esp_partition_t *)ctx, offset, out, len) == ESP_OK;
}

}  // namespace

bool StagingOps(SwarmFlashOps *ops)
{
  const esp_partition_t *p = Staging();
  ops->ctx = (void *)p;
  ops->capacity = p ? p->size : 0;
  ops->erase = Erase;
  ops->write = Write;
  ops->read = ReadFlash;
  return p != nullptr;
}

bool RunningLoader()
{
  const esp_partition_t *p = esp_ota_get_running_partition();
  return p && p->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY;
}

void MarkValid() { esp_ota_mark_app_valid_cancel_rollback(); }

void ArmHealthDeadline(uint32_t seconds)
{
  wdt_hal_context_t rwdt = RWDT_HAL_CONTEXT_DEFAULT();
  uint32_t ticks = (uint32_t)((uint64_t)seconds * rtc_clk_slow_freq_get_hz());
  wdt_hal_write_protect_disable(&rwdt);
  wdt_hal_init(&rwdt, WDT_RWDT, 0, false);
  wdt_hal_config_stage(&rwdt, WDT_STAGE0, ticks, WDT_STAGE_ACTION_RESET_SYSTEM);
  wdt_hal_enable(&rwdt);
  wdt_hal_write_protect_enable(&rwdt);
}

bool PendingVerify()
{
  esp_ota_img_states_t state;
  const esp_partition_t *p = esp_ota_get_running_partition();
  return p && esp_ota_get_state_partition(p, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;
}

uint32_t HealthDeadlineConfig() { return REG_READ(RTC_CNTL_WDTCONFIG0_REG); }

void DisarmHealthDeadline()
{
  wdt_hal_context_t rwdt = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rwdt);
  wdt_hal_disable(&rwdt);
  wdt_hal_write_protect_enable(&rwdt);
}

void RestartIntoLoader()
{
  // Selecting the factory app erases otadata: the next boot is the loader.
  if (const esp_partition_t *l = Loader())
    esp_ota_set_boot_partition(l);
  esp_restart();
}

bool MainBootable()
{
  const esp_partition_t *m = Main();
  if (!m)
    return false;
  esp_app_desc_t desc;
  if (esp_ota_get_partition_description(m, &desc) != ESP_OK)
    return false;  // no valid image header
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(m, &state) == ESP_OK &&
      (state == ESP_OTA_IMG_ABORTED || state == ESP_OTA_IMG_INVALID))
    return false;
  return true;
}

void RestartIntoMain()
{
  if (const esp_partition_t *m = Main())
    esp_ota_set_boot_partition(m);  // verifies the image; pending verify
  esp_restart();
}

bool InstallStaged(uint32_t size, const uint8_t sha256[32])
{
  const esp_partition_t *s = Staging(), *m = Main();
  if (!s || !m || size == 0 || size > m->size || kImageOffset + size > s->size)
    return false;
  uint32_t span = (size + kSector - 1) / kSector * kSector;
  if (esp_partition_erase_range(m, 0, span) != ESP_OK)
    return false;
  static uint8_t buf[kSector];
  for (uint32_t off = 0; off < size; off += kSector)
  {
    uint32_t n = size - off < kSector ? size - off : kSector;
    if (esp_partition_read(s, kImageOffset + off, buf, n) != ESP_OK ||
        esp_partition_write(m, off, buf, n) != ESP_OK)
      return false;
  }
  // Read main back through the hash.
  void *h = swarm_crypto_sha256_begin();
  if (!h)
    return false;
  for (uint32_t off = 0; off < size; off += kSector)
  {
    uint32_t n = size - off < kSector ? size - off : kSector;
    if (esp_partition_read(m, off, buf, n) != ESP_OK)
    {
      swarm_crypto_sha256_finish(h, buf);
      return false;
    }
    swarm_crypto_sha256_update(h, buf, n);
  }
  uint8_t got[32];
  swarm_crypto_sha256_finish(h, got);
  return memcmp(got, sha256, 32) == 0;
}

}  // namespace boot
