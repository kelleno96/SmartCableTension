/*
 * CAN Bus Sniffer for Arduino with TJA1050
 * 
 * This sketch reads and displays CAN bus messages using:
 * - MCP2515 CAN Controller (SPI interface)
 * - TJA1050 CAN Transceiver
 * 
 * Required Library: mcp2515 by autowp
 * Install via: Arduino IDE -> Tools -> Manage Libraries -> Search "mcp2515"
 */

#include <SPI.h>
#include <mcp2515.h>

// Create MCP2515 object with CS pin 10 (adjust if needed)
MCP2515 mcp2515(10);

unsigned long lastTransmitTime = 0;
const unsigned long transmitInterval = 100; // 10 Hz = 100ms interval
uint8_t messageCounter = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize MCP2515 with 8MHz crystal and 500kbps CAN speed
  // Adjust the CAN speed based on your bus: CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Bus Sniffer -------");
  Serial.println("Listening for CAN messages...");
  Serial.println();
}

void loop() {
  struct can_frame canMsg;
  
  // Send a heartbeat message at 10 Hz
  unsigned long currentTime = millis();
  if (currentTime - lastTransmitTime >= transmitInterval) {
    lastTransmitTime = currentTime;
    
    struct can_frame heartbeat;
    heartbeat.can_id = 0x42;  // Silly ID: Answer to everything
    heartbeat.can_dlc = 8;
    heartbeat.data[0] = 0xCA;  // CA-FE-BA-BE
    heartbeat.data[1] = 0xFE;
    heartbeat.data[2] = 0xBA;
    heartbeat.data[3] = 0xBE;
    heartbeat.data[4] = messageCounter++;
    heartbeat.data[5] = 0x69;  // Nice
    heartbeat.data[6] = 0x42;
    heartbeat.data[7] = 0x00;
    
    mcp2515.sendMessage(&heartbeat);
  }
  
  // Check if there's a message available
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Print CAN ID (in hex)
    Serial.print("ID: 0x");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" | ");
    
    // Print DLC (Data Length Code)
    Serial.print("DLC: ");
    Serial.print(canMsg.can_dlc);
    Serial.print(" | ");
    
    // Print data bytes
    Serial.print("Data: ");
    for (int i = 0; i < canMsg.can_dlc; i++) {
      if (canMsg.data[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }
    
    Serial.println();
  }
}
