// DWM3000 register-level driver, extracted from the CircuitDigest
// ESP32-DWM3000-UWB-Indoor-RTLS-Tracker reference (GPL-3.0). The class is
// all-static; the DWM3000 instance exists only for the Arduino-style call
// syntax. Anchor and tag firmware share this one copy.
#ifndef LIBS_DW3000_DW3000_H
#define LIBS_DW3000_DW3000_H

#include <Arduino.h>
#include <SPI.h>

// Default pins: the CircuitDigest breadboard wiring (SPI on the ESP32's
// default VSPI pins SCK18/MISO19/MOSI23). Boards that route the module
// elsewhere call DWM3000Class::setPins() before begin().
#define RST_PIN 27
#define CHIP_SELECT_PIN 4

// SPI + control pins; -1 for sck/miso/mosi means the board's default SPI pin.
struct DWM3000Pins
{
  int8_t sck;
  int8_t miso;
  int8_t mosi;
  int8_t cs;
  int8_t rst;
};

#define LEN_RX_CAL_CONF 4
#define LEN_TX_FCTRL_CONF 6
#define LEN_AON_DIG_CFG_CONF 3

#define PMSC_STATE_IDLE 0x3

#define FCS_LEN 2

#define STDRD_SYS_CONFIG 0x188
#define DTUNE0_CONFIG 0x0F

#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000

#define SYS_STATUS_FRAME_TX_SUCC 0x80

#define PREAMBLE_32 4
#define PREAMBLE_64 8
#define PREAMBLE_128 5
#define PREAMBLE_256 9
#define PREAMBLE_512 11
#define PREAMBLE_1024 2
#define PREAMBLE_2048 10
#define PREAMBLE_4096 3
#define PREAMBLE_1536 6

#define CHANNEL_5 0x0
#define CHANNEL_9 0x1

#define PAC4 0x03
#define PAC8 0x00
#define PAC16 0x01
#define PAC32 0x02

#define DATARATE_6_8MB 0x1
#define DATARATE_850KB 0x0

#define PHR_MODE_STANDARD 0x0
#define PHR_MODE_LONG 0x1

#define PHR_RATE_6_8MB 0x1
#define PHR_RATE_850KB 0x0

// Masks
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F

// Registers
#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC_REG 0x4
#define GPIO_CTRL_REG 0x5
#define DRX_REG 0x6
#define RF_CONF_REG 0x7
#define RF_CAL_REG 0x8
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0xC
#define CIA_REG2 0xD
#define CIA_REG3 0xE
#define DIG_DIAG_REG 0xF
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER_REG 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F

#define TRANSMIT_DELAY 0x3B9ACA00 // //0x77359400

#define TRANSMIT_DIFF 0x1FF

#define NS_UNIT 4.0064102564102564  // ns
#define PS_UNIT 15.6500400641025641 // ps

#define SPEED_OF_LIGHT 0.029979245800 // in centimetres per picosecond

#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f

// Offsets
#define NO_OFFSET 0x0

#define DEBUG_OUTPUT 0 // Turn to 1 to get all reads, writes, etc. as info in the console

class DWM3000Class
{
public:
  static int config[9];

  static DWM3000Pins pins;

  // Chip Setup
  static void setPins(const DWM3000Pins &p);
  static void spiSelect(uint8_t cs);

  static void begin();
  static void init();

  static void writeSysConfig();
  static void configureAsTX();
  static void setupGPIO();

  // Double-Sided Ranging
  static void ds_sendFrame(int stage);
  static void ds_sendRTInfo(int t_roundB, int t_replyB);
  static int ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
  static int ds_getStage();
  static bool ds_isErrorFrame();
  static void ds_sendErrorFrame();

  // Radio Settings
  static void setChannel(uint8_t data);
  static void setPreambleLength(uint8_t data);
  static void setPreambleCode(uint8_t data);
  static void setPACSize(uint8_t data);
  static void setDatarate(uint8_t data);
  static void setPHRMode(uint8_t data);
  static void setPHRRate(uint8_t data);

  // Protocol Settings
  static void setMode(int mode);
  static void setTXFrame(unsigned long long frame_data);
  static void setFrameLength(int frame_len);
  static void setTXAntennaDelay(int delay);
  static void setSenderID(int senderID);
  static void setDestinationID(int destID);

  // Status Checks
  static int receivedFrameSucc();
  static int sentFrameSucc();
  static int getSenderID();
  static int getDestinationID();
  static bool checkForIDLE();
  static bool checkSPI();

  // Radio Analytics
  static double getSignalStrength();
  static double getFirstPathSignalStrength();
  static int getTXAntennaDelay();
  static long double getClockOffset();
  static long double getClockOffset(int32_t ext_clock_offset);
  static int getRawClockOffset();
  static float getTempInC();

  static unsigned long long readRXTimestamp();
  static unsigned long long readTXTimestamp();

  // Chip Interaction
  static uint32_t write(int base, int sub, uint32_t data, int data_len);
  static uint32_t write(int base, int sub, uint32_t data);

  static uint32_t read(int base, int sub);
  static uint8_t read8bit(int base, int sub);
  // Bulk transfers (frame payloads). sub < 0x80.
  static void writeBytes(int base, int sub, const uint8_t *data, int len);
  static void readBytes(int base, int sub, uint8_t *out, int len);
  // Length (incl. 2-byte FCS) of the last received frame.
  static int rxFrameLength();
  // SPI clock for all transfers; 0 (default) keeps the Arduino default
  // without per-transfer transactions. The DW3000 needs <= 7 MHz until its
  // PLL locks (init()); it accepts much more afterwards.
  static void setSpiClock(uint32_t hz);
  static uint32_t readOTP(uint8_t addr);

  // Delayed Sending Settings
  static void writeTXDelay(uint32_t delay);
  static void prepareDelayedTX();

  // Radio Stage Settings / Transfer and Receive Modes
  static void delayedTXThenRX();
  static void delayedTX();
  static void standardTX();
  static void standardRX();
  static void TXInstantRX();
  static void forceTRXOff();

  // DWM3000 Firmware Interaction
  static void softReset();
  static void hardReset();
  static void clearSystemStatus();

  // Hardware Status Information
  static void pullLEDHigh(int led);
  static void pullLEDLow(int led);

  // Calculation and Conversion
  static double convertToCM(int DWM3000_ps_units);
  static void calculateTXRXdiff();

  // Printing
  static void printRoundTripInformation();
  static void printDouble(double val, unsigned int precision, bool linebreak);

private:
  // Single Bit Settings
  static void setBit(int reg_addr, int sub_addr, int shift, bool b);
  static void setBitLow(int reg_addr, int sub_addr, int shift);
  static void setBitHigh(int reg_addr, int sub_addr, int shift);

  // Fast Commands
  static void writeFastCommand(int cmd);

  // SPI Interaction
  static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit);
  static uint32_t sendBytes(int b[], int lenB, int recLen);

  // Soft Reset Helper Method
  static void clearAONConfig();

  // Other Helper Methods
  static unsigned int countBits(unsigned int number);
  static int checkForDevID();
};

extern DWM3000Class DWM3000;

#endif  // LIBS_DW3000_DW3000_H
