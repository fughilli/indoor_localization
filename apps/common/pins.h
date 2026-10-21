// Pin map: ACEBOTT ESP32-Max-V1.0 + DWM3000EVB shield (see apps/mule/pins.h
// for the UNO-position -> GPIO derivation), plus the IMU on the board's
// SDA/SCL header. GPIO12 (RSTn) is a flash-voltage strapping pin: the driver
// leaves it floating after reset.
#ifndef APPS_COMMON_PINS_H
#define APPS_COMMON_PINS_H

#define NODE_PIN_SCK 5
#define NODE_PIN_MISO 16
#define NODE_PIN_MOSI 17
#define NODE_PIN_CS 18
#define NODE_PIN_RST 12
#define NODE_PIN_IRQ 23
#define NODE_PIN_WAKEUP 19
#define NODE_PIN_EXTON 32

// MPU6050 (+ QMC5883L via its bypass). GPIO27 (UNO D2) meets DW3000 GPIO4,
// which the driver leaves an input (GPIO_DIR 0xF0); GPIO4 is the ACEBOTT's
// left-header "04", off the shield. Ruled out: 34/35 (input-only), 32/33
// (EXTON / DW GPIO7), 12 (RSTn + strap), 13/14 and 25/26 (DW GPIO0-3,
// which setupGPIO() drives as outputs; 2/3 also carry the RX/TX LEDs).
#define NODE_PIN_SDA 27
#define NODE_PIN_SCL 4

#endif  // APPS_COMMON_PINS_H
