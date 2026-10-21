// Pin-map probe: for each candidate DWM3000 wiring, reset the module and read
// DEV_ID, reporting one JSON line per candidate. Flash this onto an unknown
// board/shield combination to find which map talks to the module (a correct
// map reads 0xDECA0302 / 0xDECA0312). Repeats every few seconds.
#include <Arduino.h>
#include <SPI.h>

#include "libs/dw3000/dw3000.h"

namespace {

struct Candidate
{
  const char *name;
  DWM3000Pins pins;
};

// {sck, miso, mosi, cs, rst}
const Candidate kCandidates[] = {
    // ACEBOTT ESP32-Max-V1.0 + DWM3000EVB (apps/mule/pins.h)
    {"acebott_max_v1", {5, 16, 17, 18, 12}},
    // WeMos D1 R32-style UNO ESP32: D13..D10 = 18/19/23/5, D7 = 14
    {"d1_r32", {18, 19, 23, 5, 14}},
    // CircuitDigest breadboard wiring (driver defaults)
    {"circuitdigest", {18, 19, 23, 4, 27}},
};

}  // namespace

void setup()
{
  Serial.begin(115200);
  delay(100);
}

void loop()
{
  for (const Candidate &c : kCandidates)
  {
    DWM3000.setPins(c.pins);
    SPI.end();
    DWM3000.begin();
    DWM3000.hardReset();
    delay(200);
    uint32_t id = DWM3000.read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
    bool ok = id == 0xDECA0302 || id == 0xDECA0312;
    Serial.printf("{\"t\":%.3f,\"type\":\"probe\",\"map\":\"%s\",\"sck\":%d,\"miso\":%d,"
                  "\"mosi\":%d,\"cs\":%d,\"rst\":%d,\"devid\":\"0x%08lX\",\"ok\":%s}\n",
                  millis() / 1000.0, c.name, c.pins.sck, c.pins.miso, c.pins.mosi,
                  c.pins.cs, c.pins.rst, (unsigned long)id, ok ? "true" : "false");
    // Release the candidate's pins before trying the next map.
    pinMode(c.pins.cs, INPUT);
  }
  delay(3000);
}
