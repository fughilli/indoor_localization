// Pin map: ACEBOTT ESP32-Max-V1.0 + DWM3000EVB shield. The ACEBOTT headers
// are in UNO positions but carry ESP32 GPIO numbers; the shield's signals land
// as follows (UNO position -> GPIO):
//   D13 SPICLK  -> 5    D12 MISO -> 16   D11 MOSI -> 17   D10 SPICSn -> 18
//   D9  WAKEUP  -> 19   D8  IRQ  -> 23   D7  RSTn -> 12
//   A0  EXTON   -> 32   A1 GPIO7 -> 33
// GPIO12 is the ESP32's flash-voltage strapping pin: RSTn must not be held
// high at power-up, so the driver leaves it floating after reset.
#ifndef APPS_MULE_PINS_H
#define APPS_MULE_PINS_H

#define MULE_PIN_SCK 5
#define MULE_PIN_MISO 16
#define MULE_PIN_MOSI 17
#define MULE_PIN_CS 18
#define MULE_PIN_RST 12
#define MULE_PIN_IRQ 23
#define MULE_PIN_WAKEUP 19
#define MULE_PIN_EXTON 32

#endif  // APPS_MULE_PINS_H
