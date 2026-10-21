// Line-level diagnostic for the DWM3000EVB shield, run before any SPI traffic.
// Reports, once a second:
//   rstn, exton  — DW3000 RSTn is pulled up and EXTON driven high by a
//                  powered, awake module; 0 on both means no module power
//   irq          — DW3000 IRQ output level
//   miso_*       — MISO read with pull-up and pull-down, CS deasserted and
//                  asserted. Following the pull = undriven (normal with CS
//                  high); same value under both pulls = actively driven.
//                  Driven low with CS high points at another device on the
//                  pin (e.g. a module whose GPIO16/17 are wired to PSRAM).
//   devid_hw     — DEV_ID read through the SPI peripheral (GPIO matrix)
//   devid_bb     — DEV_ID read by bit-banging the same pins; if this one
//                  works and devid_hw doesn't, the pins are fine and the
//                  peripheral setup is at fault
#include <Arduino.h>
#include <SPI.h>

#include "apps/mule/pins.h"

namespace {

int ReadWithPull(int pin, int mode)
{
  pinMode(pin, mode);
  delayMicroseconds(200);
  return digitalRead(pin);
}

// DW3000 short read of register file 0 (DEV_ID): one header byte 0x00, then
// four bytes LSB first. SPI mode 0, ~100 kHz.
uint32_t BitBangDevId()
{
  pinMode(MULE_PIN_SCK, OUTPUT);
  pinMode(MULE_PIN_MOSI, OUTPUT);
  pinMode(MULE_PIN_MISO, INPUT);
  digitalWrite(MULE_PIN_SCK, LOW);
  digitalWrite(MULE_PIN_CS, LOW);
  delayMicroseconds(10);
  uint8_t bytes[5] = {0};
  for (int i = 0; i < 5; i++)
  {
    uint8_t in = 0;
    for (int bit = 7; bit >= 0; bit--)
    {
      digitalWrite(MULE_PIN_MOSI, 0);  // header 0x00, then dummy 0x00s
      delayMicroseconds(5);
      digitalWrite(MULE_PIN_SCK, HIGH);
      in |= digitalRead(MULE_PIN_MISO) << bit;
      delayMicroseconds(5);
      digitalWrite(MULE_PIN_SCK, LOW);
    }
    bytes[i] = in;
  }
  digitalWrite(MULE_PIN_CS, HIGH);
  return bytes[1] | bytes[2] << 8 | bytes[3] << 16 | (uint32_t)bytes[4] << 24;
}

uint32_t HardwareDevId()
{
  SPI.begin(MULE_PIN_SCK, MULE_PIN_MISO, MULE_PIN_MOSI, MULE_PIN_CS);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MULE_PIN_CS, LOW);
  uint8_t bytes[5];
  for (int i = 0; i < 5; i++)
    bytes[i] = SPI.transfer(0x00);
  digitalWrite(MULE_PIN_CS, HIGH);
  SPI.endTransaction();
  SPI.end();
  return bytes[1] | bytes[2] << 8 | bytes[3] << 16 | (uint32_t)bytes[4] << 24;
}

}  // namespace

void setup()
{
  Serial.begin(115200);
  delay(100);
  pinMode(MULE_PIN_CS, OUTPUT);
  digitalWrite(MULE_PIN_CS, HIGH);
}

void loop()
{
  int rstn = ReadWithPull(MULE_PIN_RST, INPUT);
  int exton = ReadWithPull(MULE_PIN_EXTON, INPUT);
  int irq = ReadWithPull(MULE_PIN_IRQ, INPUT);

  digitalWrite(MULE_PIN_CS, HIGH);
  int up_cs_hi = ReadWithPull(MULE_PIN_MISO, INPUT_PULLUP);
  int dn_cs_hi = ReadWithPull(MULE_PIN_MISO, INPUT_PULLDOWN);
  digitalWrite(MULE_PIN_CS, LOW);
  int up_cs_lo = ReadWithPull(MULE_PIN_MISO, INPUT_PULLUP);
  int dn_cs_lo = ReadWithPull(MULE_PIN_MISO, INPUT_PULLDOWN);
  digitalWrite(MULE_PIN_CS, HIGH);
  pinMode(MULE_PIN_MISO, INPUT);

  uint32_t devid_hw = HardwareDevId();
  pinMode(MULE_PIN_CS, OUTPUT);
  digitalWrite(MULE_PIN_CS, HIGH);
  uint32_t devid_bb = BitBangDevId();

  Serial.printf("{\"t\":%.3f,\"type\":\"diag\",\"rstn\":%d,\"exton\":%d,\"irq\":%d,"
                "\"miso_pu_cs_hi\":%d,\"miso_pd_cs_hi\":%d,\"miso_pu_cs_lo\":%d,"
                "\"miso_pd_cs_lo\":%d,\"devid_hw\":\"0x%08lX\",\"devid_bb\":\"0x%08lX\"}\n",
                millis() / 1000.0, rstn, exton, irq, up_cs_hi, dn_cs_hi, up_cs_lo,
                dn_cs_lo, (unsigned long)devid_hw, (unsigned long)devid_bb);
  delay(1000);
}
