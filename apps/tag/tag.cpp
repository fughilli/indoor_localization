// UWB RTLS tag: initiator of double-sided ranging against NUM_ANCHORS
// anchors round-robin; streams per-anchor distances as JSON over TCP to the
// host (host/server.py). App logic from the CircuitDigest reference
// main_tag.cpp (GPL-3.0); the DWM3000 driver lives in //libs/dw3000.
#include <WiFi.h>
#include <WiFiClient.h>

#include "apps/tag/config.h"
#include "libs/dw3000/dw3000.h"

// WiFi Configuration
const char *ssid = UWB_WIFI_SSID;
const char *password = UWB_WIFI_PASSWORD;
const char *host = UWB_HOST_IP; // Your PC's IP address
const int port = UWB_HOST_PORT;
WiFiClient client;
bool wifiConnected = false;

// Scalable Anchor Configuration
#define NUM_ANCHORS 3 // Change this to scale the system
#define TAG_ID 10
#define FIRST_ANCHOR_ID 1 // Starting ID for anchors (1, 2, 3, ...)

// Ranging Configuration
#define FILTER_SIZE 30 // For median filter
#define MIN_DISTANCE 0
#define MAX_DISTANCE 1000.0

// Global variables
static int rx_status;
static int current_anchor_index = 0; // Index into anchors array
static int curr_stage = 0;

// Anchor data structure
struct AnchorData
{
    int anchor_id; // Anchor ID

    // Timing measurements
    int t_roundA = 0;
    int t_replyA = 0;
    long long rx = 0;
    long long tx = 0;
    int clock_offset = 0;

    // Distance measurements
    float distance = 0;
    float distance_history[FILTER_SIZE] = {0};
    int history_index = 0;
    float filtered_distance = 0;

    // Signal quality metrics
    float signal_strength = 0;    // RSSI in dBm
    float fp_signal_strength = 0; // First Path RSSI in dBm
};

// Dynamic array of anchor data
AnchorData anchors[NUM_ANCHORS];

// Helper functions for anchor management
void initializeAnchors()
{
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
        // Initialize all other fields to zero (default constructor handles this)
    }
}

AnchorData *getCurrentAnchor()
{
    return &anchors[current_anchor_index];
}

int getCurrentAnchorId()
{
    return anchors[current_anchor_index].anchor_id;
}

void switchToNextAnchor()
{
    current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
}

bool allAnchorsHaveValidData()
{
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        if (anchors[i].filtered_distance <= 0)
        {
            return false;
        }
    }
    return true;
}

// WiFi Functions
void connectToWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        wifiConnected = true;
        Serial.println("\nWiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nFailed to connect to WiFi");
    }
}

void sendDataOverWiFi()
{
    if (!wifiConnected)
    {
        connectToWiFi();
        if (!wifiConnected)
            return;
    }

    if (!client.connected())
    {
        if (!client.connect(host, port))
        {
            Serial.println("Connection to host failed");
            wifiConnected = false;
            return;
        }
    }

    // Create JSON structure dynamically based on number of anchors
    String data = "{\"tag_id\":" + String(TAG_ID) + ",\"anchors\":{";

    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        data += "\"A" + String(anchors[i].anchor_id) + "\":{";
        data += "\"distance\":" + String(anchors[i].filtered_distance, 2) + ",";
        data += "\"raw\":" + String(anchors[i].distance, 2) + ",";
        data += "\"rssi\":" + String(anchors[i].signal_strength, 2) + ",";
        data += "\"fp_rssi\":" + String(anchors[i].fp_signal_strength, 2) + ",";
        data += "\"round_time\":" + String(anchors[i].t_roundA) + ",";
        data += "\"reply_time\":" + String(anchors[i].t_replyA) + ",";
        data += "\"clock_offset\":" + String((double)DWM3000.getClockOffset(anchors[i].clock_offset), 6);
        data += "}";

        // Add comma if not the last anchor
        if (i < NUM_ANCHORS - 1)
        {
            data += ",";
        }
    }

    data += "}}\n";

    client.print(data);

    // For debugging, print the JSON to serial
    Serial.println("Sent JSON data:");
    Serial.println(data);
}

// Helper function to validate distance
bool isValidDistance(float distance)
{
    return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

float calculateMedian(float arr[], int size)
{
    float temp[size];
    for (int i = 0; i < size; i++)
    {
        temp[i] = arr[i];
    }

    for (int i = 0; i < size - 1; i++)
    {
        for (int j = i + 1; j < size; j++)
        {
            if (temp[j] < temp[i])
            {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    if (size % 2 == 0)
    {
        return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
    }
    else
    {
        return temp[size / 2];
    }
}

// updateFilteredDistance function
void updateFilteredDistance(AnchorData &data)
{
    data.distance_history[data.history_index] = data.distance;
    data.history_index = (data.history_index + 1) % FILTER_SIZE;

    float valid_distances[FILTER_SIZE];
    int valid_count = 0;

    for (int i = 0; i < FILTER_SIZE; i++)
    {
        if (isValidDistance(data.distance_history[i]))
        {
            valid_distances[valid_count++] = data.distance_history[i];
        }
    }

    if (valid_count > 0)
    {
        data.filtered_distance = calculateMedian(valid_distances, valid_count);
    }
    else
    {
        data.filtered_distance = 0;
    }
}

// Debug print function
void printDebugInfo(int anchor, long long rx, long long tx, int t_round, int t_reply, int clock_offset)
{
    Serial.print("Anchor ");
    Serial.print(anchor);
    Serial.println(" Debug Info:");
    Serial.print("RX timestamp: ");
    Serial.println(rx);
    Serial.print("TX timestamp: ");
    Serial.println(tx);
    Serial.print("t_round: ");
    Serial.println(t_round);
    Serial.print("t_reply: ");
    Serial.println(t_reply);
    Serial.print("Clock offset: ");
    Serial.println(clock_offset);

    int ranging_time = DWM3000.ds_processRTInfo(t_round, t_reply,
                                                DWM3000.read(0x12, 0x04), DWM3000.read(0x12, 0x08), clock_offset);
    Serial.print("Calculated distance: ");
    Serial.println(DWM3000.convertToCM(ranging_time));
}

void printAllDistances()
{
    Serial.print("Distances - ");
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        Serial.print("A");
        Serial.print(anchors[i].anchor_id);
        Serial.print(": ");
        if (anchors[i].filtered_distance > 0)
        {
            DWM3000.printDouble(anchors[i].filtered_distance, 100, false);
            Serial.print(" cm");
        }
        else
        {
            Serial.print("INVALID");
        }

        if (i < NUM_ANCHORS - 1)
        {
            Serial.print(" | ");
        }
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    // Initialize anchor array
    initializeAnchors();

    Serial.print("Initialized ");
    Serial.print(NUM_ANCHORS);
    Serial.println(" anchors:");
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        Serial.print("  Anchor ");
        Serial.print(i);
        Serial.print(" - ID: ");
        Serial.println(anchors[i].anchor_id);
    }

    // Connect to WiFi first
    connectToWiFi();

    // Initialize UWB
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
    DWM3000.setTXAntennaDelay(16350);
    DWM3000.setSenderID(TAG_ID);

    Serial.println("> TAG - Three Anchor Ranging System <");
    Serial.println("> With WiFi Communication <\n");
    Serial.println("[INFO] Setup is finished.");
    Serial.print("Antenna delay set to: ");
    Serial.println(DWM3000.getTXAntennaDelay());

    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();
}

void loop()
{
    AnchorData *currentAnchor = getCurrentAnchor();
    int currentAnchorId = getCurrentAnchorId();

    switch (curr_stage)
    {
    case 0: // Start ranging with current target
        // Reset timing measurements for current anchor
        currentAnchor->t_roundA = 0;
        currentAnchor->t_replyA = 0;

        DWM3000.setDestinationID(currentAnchorId);
        DWM3000.ds_sendFrame(1);    // Sends "Poll" (Stage 1)
        currentAnchor->tx = DWM3000.readTXTimestamp(); // Saves T1 (Time Sent)
        curr_stage = 1; // Move to wait for response
        break;

    case 1: // Await first response
        if (rx_status = DWM3000.receivedFrameSucc())    // Response Received
        {
            DWM3000.clearSystemStatus();
            if (rx_status == 1)
            {
                if (DWM3000.ds_isErrorFrame())
                {
                    Serial.print("[WARNING] Error frame from Anchor ");
                    Serial.print(currentAnchorId);
                    Serial.print("! Signal strength: ");
                    Serial.print(DWM3000.getSignalStrength());
                    Serial.println(" dBm");
                    curr_stage = 0;
                }
                else if (DWM3000.ds_getStage() != 2)
                {
                    Serial.print("[WARNING] Unexpected stage from Anchor ");
                    Serial.print(currentAnchorId);
                    Serial.print(": ");
                    Serial.println(DWM3000.ds_getStage());
                    DWM3000.ds_sendErrorFrame();
                    curr_stage = 0;
                }
                else
                {
                    curr_stage = 2; // Move to send Final
                }
            }
            else
            {
                Serial.print("[ERROR] Receiver Error from Anchor ");
                Serial.println(currentAnchorId);
                DWM3000.clearSystemStatus();
            }
        }
        break;

    case 2: // Response received. Send second ranging
        currentAnchor->rx = DWM3000.readRXTimestamp();  // Reads T4 (Time Response Arrived)
        DWM3000.ds_sendFrame(3);    // Sends "Final" (Stage 3)
        currentAnchor->t_roundA = currentAnchor->rx - currentAnchor->tx;    // Calculate Tag's Round Trip: (Time Response Arrived - Time Poll Sent)
        currentAnchor->tx = DWM3000.readTXTimestamp();  // Reads T5 (Time Final Sent)
        currentAnchor->t_replyA = currentAnchor->tx - currentAnchor->rx;    // Calculate Tag's Reply Time: (Time Final Sent - Time Response Arrived)

        curr_stage = 3; // Move to wait for Report
        break;

    case 3: // Await second response
        if (rx_status = DWM3000.receivedFrameSucc())    // Report Received
        {
            DWM3000.clearSystemStatus();
            if (rx_status == 1)
            {
                if (DWM3000.ds_isErrorFrame())
                {
                    Serial.print("[WARNING] Error frame from Anchor ");
                    Serial.println(currentAnchorId);
                    curr_stage = 0;
                }
                else
                {
                    currentAnchor->clock_offset = DWM3000.getRawClockOffset();
                    curr_stage = 4; // Move to calculation
                }
            }
            else
            {
                Serial.print("[ERROR] Receiver Error from Anchor ");
                Serial.println(currentAnchorId);
                DWM3000.clearSystemStatus();
            }
        }
        break;

    case 4: // Response received. Calculating results
    {
        int ranging_time = DWM3000.ds_processRTInfo(
            currentAnchor->t_roundA,    // Calculated locally in Phase 3
            currentAnchor->t_replyA,    // Calculated locally in Phase 3
            DWM3000.read(0x12, 0x04),   // Read from Report Packet (Bytes 4-7)
            DWM3000.read(0x12, 0x08),   // Read from Report Packet (Bytes 8-11)
            currentAnchor->clock_offset);

        currentAnchor->distance = DWM3000.convertToCM(ranging_time);
        currentAnchor->signal_strength = DWM3000.getSignalStrength();
        currentAnchor->fp_signal_strength = DWM3000.getFirstPathSignalStrength();
        updateFilteredDistance(*currentAnchor);
    }

        // Print current distances
        printAllDistances();

        // Send data over WiFi if all anchors have valid data
        if (allAnchorsHaveValidData())
        {
            sendDataOverWiFi();
        }

        // Switch to next anchor
        switchToNextAnchor();
        curr_stage = 0;
        break;

    default:
        Serial.print("Entered stage (");
        Serial.print(curr_stage);
        Serial.println("). Reverting back to stage 0");
        curr_stage = 0;
        break;
    }
}