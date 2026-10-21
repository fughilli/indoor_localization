// MPU6050 + QMC5883L over I2C: see imu.h. Register values from the
// MPU-6000/6050 register map rev 4.2 and the QMC5883L datasheet v1.0.
#include "apps/node/imu.h"

#include <Arduino.h>
#include <Wire.h>

#include "apps/common/pins.h"

namespace imu {
namespace {

// MPU6050 registers.
constexpr uint8_t kSmplrtDiv = 0x19;
constexpr uint8_t kConfig = 0x1A;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kIntPinCfg = 0x37;
constexpr uint8_t kAccelXoutH = 0x3B;
constexpr uint8_t kUserCtrl = 0x6A;
constexpr uint8_t kPwrMgmt1 = 0x6B;
constexpr uint8_t kWhoAmI = 0x75;

// Scales for the ranges configured below.
constexpr float kAccelLsbPerG = 8192.0f;   // +-4 g
constexpr float kGyroLsbPerDps = 65.5f;    // +-500 dps
constexpr float kG = 9.80665f;
constexpr float kDegToRad = 0.017453292519943295f;

// QMC5883L registers.
constexpr uint8_t kQmcData = 0x00;
constexpr uint8_t kQmcStatus = 0x06;
constexpr uint8_t kQmcControl1 = 0x09;
constexpr uint8_t kQmcSetReset = 0x0B;
constexpr uint8_t kQmcChipId = 0x0D;

bool Write(uint8_t addr, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Read(uint8_t addr, uint8_t reg, uint8_t *out, int n)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return false;
  if (Wire.requestFrom((int)addr, n) != n)
    return false;
  for (int i = 0; i < n; i++)
    out[i] = Wire.read();
  return true;
}

bool Answers(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void Scan()
{
  Serial.printf("{\"t\":%.3f,\"type\":\"i2c_scan\",\"sda\":%d,\"scl\":%d,\"addrs\":[",
                millis() / 1000.0, NODE_PIN_SDA, NODE_PIN_SCL);
  bool first = true;
  for (uint8_t a = 0x08; a < 0x78; a++)
    if (Answers(a))
    {
      Serial.printf("%s\"0x%02X\"", first ? "" : ",", a);
      first = false;
    }
  Serial.println("]}");
}

bool BeginMpu()
{
  uint8_t who = 0;
  if (!Read(kMpuAddr, kWhoAmI, &who, 1))
    return false;
  // 0x68 on a genuine MPU6050; common clones report 0x70 / 0x72 / 0x98.
  Serial.printf("{\"t\":%.3f,\"type\":\"mpu\",\"who_am_i\":\"0x%02X\"}\n", millis() / 1000.0, who);
  bool ok = Write(kMpuAddr, kPwrMgmt1, 0x80);  // device reset
  delay(100);
  ok &= Write(kMpuAddr, kPwrMgmt1, 0x01);      // wake, PLL on gyro X
  delay(10);
  ok &= Write(kMpuAddr, kConfig, 0x03);        // DLPF 44 Hz -> 1 kHz internal
  ok &= Write(kMpuAddr, kSmplrtDiv, 4);        // 1 kHz / (1+4) = 200 Hz
  ok &= Write(kMpuAddr, kGyroConfig, 0x08);    // +-500 dps
  ok &= Write(kMpuAddr, kAccelConfig, 0x08);   // +-4 g
  ok &= Write(kMpuAddr, kUserCtrl, 0x00);      // aux I2C master off ...
  ok &= Write(kMpuAddr, kIntPinCfg, 0x02);     // ... and bypass on: the aux
  return ok;                                   // bus joins the main bus
}

bool BeginQmc()
{
  uint8_t id = 0;
  if (!Answers(kQmcAddr) || !Read(kQmcAddr, kQmcChipId, &id, 1))
    return false;
  Serial.printf("{\"t\":%.3f,\"type\":\"qmc\",\"chip_id\":\"0x%02X\"}\n", millis() / 1000.0, id);
  bool ok = Write(kQmcAddr, kQmcSetReset, 0x01);  // recommended SET/RESET period
  // OSR 512 (00) | range 8 G (01) | ODR 200 Hz (11) | continuous (01).
  ok &= Write(kQmcAddr, kQmcControl1, 0x1D);
  return ok;
}

int16_t Be16(const uint8_t *b) { return (int16_t)((b[0] << 8) | b[1]); }
int16_t Le16(const uint8_t *b) { return (int16_t)((b[1] << 8) | b[0]); }

// The IMU keeps power across an ESP32 reset; a reset in the middle of a read
// can leave it holding SDA low, waiting for clocks that never come, and the
// bus is dead until power-cycled. Clock SCL until SDA is released, then send
// a STOP (the usual I2C bus-clear sequence).
void RecoverBus()
{
  pinMode(NODE_PIN_SDA, INPUT_PULLUP);
  pinMode(NODE_PIN_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(NODE_PIN_SCL, HIGH);
  delayMicroseconds(10);
  int pulses = 0;
  for (; pulses < 18 && digitalRead(NODE_PIN_SDA) == LOW; pulses++)
  {
    digitalWrite(NODE_PIN_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(NODE_PIN_SCL, HIGH);
    delayMicroseconds(10);
  }
  // STOP: SDA low -> high while SCL is high.
  pinMode(NODE_PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(NODE_PIN_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(NODE_PIN_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(NODE_PIN_SDA, HIGH);
  delayMicroseconds(10);
  pinMode(NODE_PIN_SDA, INPUT_PULLUP);
  Serial.printf("{\"t\":%.3f,\"type\":\"i2c_recover\",\"pulses\":%d,\"sda\":%d}\n",
                millis() / 1000.0, pulses, digitalRead(NODE_PIN_SDA));
}

}  // namespace

Present Begin()
{
  RecoverBus();
  Wire.begin(NODE_PIN_SDA, NODE_PIN_SCL, 400000);
  Scan();
  Present p = {};
  p.mpu = BeginMpu();
  // The QMC is only reachable once the MPU's bypass is on.
  p.mag = BeginQmc();
  if (p.mpu || p.mag)
    Scan();
  return p;
}

bool ReadMotion(float accel[3], float gyro[3])
{
  uint8_t b[14];
  if (!Read(kMpuAddr, kAccelXoutH, b, sizeof(b)))
    return false;
  for (int i = 0; i < 3; i++)
  {
    accel[i] = Be16(b + 2 * i) / kAccelLsbPerG * kG;
    gyro[i] = Be16(b + 8 + 2 * i) / kGyroLsbPerDps * kDegToRad;
  }
  return true;
}

bool ReadMag(float mag[3])
{
  uint8_t status = 0;
  if (!Read(kQmcAddr, kQmcStatus, &status, 1) || !(status & 0x01))
    return false;
  uint8_t b[6];
  if (!Read(kQmcAddr, kQmcData, b, sizeof(b)))
    return false;
  for (int i = 0; i < 3; i++)
    mag[i] = Le16(b + 2 * i);
  return true;
}

}  // namespace imu
