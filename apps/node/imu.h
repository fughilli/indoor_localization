// MPU6050 (accel + gyro) with a QMC5883L magnetometer on its auxiliary bus,
// reached through the MPU's I2C bypass so both sit on the main bus
// (SDA GPIO27, SCL GPIO4). Either may be absent: Begin() reports what answered.
#ifndef APPS_NODE_IMU_H
#define APPS_NODE_IMU_H

#include <stdint.h>

namespace imu {

constexpr uint8_t kMpuAddr = 0x68;
constexpr uint8_t kQmcAddr = 0x0D;

struct Present
{
  bool mpu;
  bool mag;
};

// Starts the I2C bus, prints a scan of every responding address as a JSON
// line, and configures what it finds: MPU6050 at 200 Hz (DLPF 44 Hz,
// +-4 g, +-500 dps), QMC5883L continuous 200 Hz, 8 G.
Present Begin();

// Latest accel (m/s^2, specific force) + gyro (rad/s), sensor frame.
bool ReadMotion(float accel[3], float gyro[3]);
// Latest raw magnetometer counts, sensor frame; false if no new sample.
bool ReadMag(float mag[3]);

}  // namespace imu

#endif  // APPS_NODE_IMU_H
