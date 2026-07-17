// UWB RTLS anchor: responder side of the double-sided ranging exchange.
// App logic from the CircuitDigest reference main_anchor.cpp (GPL-3.0);
// the DWM3000 driver lives in //libs/dw3000.
//
// ANCHOR_ID is injected per-binary via Bazel local_defines (anchor_1/2/3).
#include "libs/dw3000/dw3000.h"

#ifndef ANCHOR_ID
#define ANCHOR_ID 1
#endif
#define RESPONSE_TIMEOUT_MS 10 // Maximum time to wait for a response
unsigned long last_ranging_time = 0;
#define MAX_RETRIES 3
int retry_count = 0;

static int rx_status;

static int curr_stage = 0;

static int t_roundB = 0;
static int t_replyB = 0;

static long long rx = 0;
static long long tx = 0;

void resetRadio()
{
  Serial.println("[INFO] Performing radio reset...");
  DWM3000.softReset();
  delay(100);
  DWM3000.clearSystemStatus();
  DWM3000.configureAsTX();
  DWM3000.standardRX();
}

void setup()
{
  Serial.begin(115200);
  DWM3000.begin();
  DWM3000.hardReset();
  delay(200);

  if (!DWM3000.checkSPI())
  {
    Serial.println("[ERROR] Could not establish SPI Connection to DWM3000!");
    while (1)
      ;
  }

  while (!DWM3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    delay(1000);
  }

  DWM3000.softReset();
  delay(200);

  if (!DWM3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (1)
      ;
  }

  DWM3000.init();
  DWM3000.setupGPIO();

  // Set antenna delay - calibrate this for your hardware!
  DWM3000.setTXAntennaDelay(16350);

  // Set anchor ID
  DWM3000.setSenderID(ANCHOR_ID);

  Serial.print("> ANCHOR ");
  Serial.print(ANCHOR_ID);
  Serial.println(" - Ready for ranging <");
  Serial.print("Antenna delay set to: ");
  Serial.println(DWM3000.getTXAntennaDelay());
  Serial.println("[INFO] Setup finished.");

  DWM3000.configureAsTX();
  DWM3000.clearSystemStatus();
  DWM3000.standardRX();
}

void loop()
{
  if (DWM3000.receivedFrameSucc() == 1 && DWM3000.ds_getStage() == 1 && DWM3000.getDestinationID() == ANCHOR_ID)
  {
    // Reset session if new ranging request arrives
    if (curr_stage != 0)
    {
      Serial.println("[INFO] New request - resetting session");
      curr_stage = 0;
      t_roundB = 0;
      t_replyB = 0;
    }
  }
  switch (curr_stage)
  {
  case 0: // Await ranging
    t_roundB = 0;
    t_replyB = 0;
    last_ranging_time = millis(); // Reset timeout timer

    if (rx_status = DWM3000.receivedFrameSucc())  // Check Poll Received
    {
      DWM3000.clearSystemStatus();
      if (rx_status == 1)
      { // If frame reception was successful
        // Only respond if frame is addressed to us
        if (DWM3000.getDestinationID() == ANCHOR_ID)
        {
          if (DWM3000.ds_isErrorFrame())
          {
            Serial.println("[WARNING] Received error frame!");
            curr_stage = 0;
            DWM3000.standardRX();
          }
          else if (DWM3000.ds_getStage() != 1)
          {
            Serial.print("[WARNING] Unexpected stage: ");
            Serial.println(DWM3000.ds_getStage());
            DWM3000.ds_sendErrorFrame();
            DWM3000.standardRX();
            curr_stage = 0;
          }
          else
          {
            curr_stage = 1; // Move to send response
          }
        }
        else
        {
          // Not for us, go back to RX
          DWM3000.standardRX();
        }
      }
      else
      {
        Serial.println("[ERROR] Receiver Error occurred!");
        DWM3000.clearSystemStatus();
      }
    }
    else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS)
    {
      Serial.println("[WARNING] Timeout waiting for ranging request");
      if (++retry_count > MAX_RETRIES)
      {
        Serial.println("[ERROR] Max retries reached, resetting radio");
        resetRadio();
        retry_count = 0;
      }
      DWM3000.standardRX(); // Reset to listening mode
    }
    break;

  case 1: // Ranging received. Sending response
    DWM3000.ds_sendFrame(2);  // Sends "Response" (Stage 2)

    rx = DWM3000.readRXTimestamp(); // Reads T2 (Time Poll Arrived)
    tx = DWM3000.readTXTimestamp(); // Reads T3 (Time Response Sent)

    t_replyB = tx - rx; // Calculates t_replyB (T3 - T2)
    curr_stage = 2; // Move to wait for Final
    last_ranging_time = millis(); // Reset timeout timer
    break;

  case 2: // Awaiting response
    if (rx_status = DWM3000.receivedFrameSucc())  // Check Final Received
    {
      retry_count = 0; // Reset on successful response
      DWM3000.clearSystemStatus();
      if (rx_status == 1)
      { // If frame reception was successful
        if (DWM3000.ds_isErrorFrame())
        {
          Serial.println("[WARNING] Received error frame!");
          curr_stage = 0;
          DWM3000.standardRX();
        }
        else if (DWM3000.ds_getStage() != 3)
        {
          Serial.print("[WARNING] Unexpected stage: ");
          Serial.println(DWM3000.ds_getStage());
          DWM3000.ds_sendErrorFrame();
          DWM3000.standardRX();
          curr_stage = 0;
        }
        else
        {
          curr_stage = 3; // Move to calculate and report
        }
      }
      else
      {
        Serial.println("[ERROR] Receiver Error occurred!");
        DWM3000.clearSystemStatus();
      }
    }
    else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS)
    {
      Serial.println("[WARNING] Timeout waiting for second response");
      if (++retry_count > MAX_RETRIES)
      {
        Serial.println("[ERROR] Max retries reached, resetting radio");
        resetRadio();
        retry_count = 0;
      }
      curr_stage = 0;
      DWM3000.standardRX();
    }
    break;

  case 3: // Second response received. Sending information frame
    rx = DWM3000.readRXTimestamp(); // Reads T6 (Time Final Arrived)
    t_roundB = rx - tx; // Calculate Anchor's Round Trip: (Time Final Arrived - Time Response Sent)
    DWM3000.ds_sendRTInfo(t_roundB, t_replyB);  // Sends DATA packet containing these two integers

    curr_stage = 0; // Reset for next ranging
    DWM3000.standardRX();
    break;

  default:
    Serial.print("[ERROR] Entered unknown stage (");
    Serial.print(curr_stage);
    Serial.println("). Reverting back to stage 0");

    curr_stage = 0;
    DWM3000.standardRX();
    break;
  }
}
