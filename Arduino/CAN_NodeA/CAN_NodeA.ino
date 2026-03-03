/*
 * CAN Node A — Sender / Talker
 * 
 * Hardware: Arduino Uno + MCP2515 CAN module (8 MHz oscillator)
 *   INT → D2, CS → D10, SI → D11, SO → D12, SCK → D13
 * 
 * Behavior:
 *   - Sends a heartbeat message (ID 0x100) every 500 ms with a rolling counter
 *   - Sends a ping message (ID 0x200) every 2 seconds
 *   - Listens for any incoming CAN frames and prints them to Serial
 * 
 * Serial: 115200 baud — open the monitor to watch traffic
 * 
 * Library: "mcp_can" by coryjfowler (install via Library Manager)
 */

#include <SPI.h>
#include <mcp_can.h>

// ----- Pin config -----
const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(CAN_CS_PIN);

// ----- State -----
uint8_t heartbeatCounter = 0;
uint32_t lastHeartbeat   = 0;
uint32_t lastPing        = 0;

volatile bool canMsgReceived = false;

void canISR() {
  canMsgReceived = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // wait for serial on USB boards

  Serial.println(F("=== CAN Node A (Sender) ==="));
  Serial.println(F("Initializing MCP2515 (8 MHz, 500 kbps)..."));

  // MCP_8MHZ tells the library our crystal is 8 MHz
  // CAN_500KBPS sets the bus speed
  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("  MCP2515 init failed — retrying in 500 ms"));
    delay(500);
  }

  Serial.println(F("  MCP2515 init OK"));

  // Set normal mode (default after begin is loopback on some libs)
  CAN.setMode(MCP_NORMAL);

  // Attach interrupt for incoming frames
  pinMode(CAN_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);

  Serial.println(F("Ready — sending heartbeats & pings\n"));
}

void loop() {
  uint32_t now = millis();

  // ---- Send heartbeat every 500 ms (ID 0x100) ----
  if (now - lastHeartbeat >= 500) {
    lastHeartbeat = now;

    uint8_t data[8] = {0};
    data[0] = heartbeatCounter++;
    data[1] = (uint8_t)(now >> 24);
    data[2] = (uint8_t)(now >> 16);
    data[3] = (uint8_t)(now >> 8);
    data[4] = (uint8_t)(now);

    byte result = CAN.sendMsgBuf(0x100, 0, 5, data);  // std frame, 5 bytes
    if (result == CAN_OK) {
      Serial.print(F("TX  0x100  cnt="));
      Serial.print(data[0]);
      Serial.print(F("  uptime="));
      Serial.println(now);
    } else {
      Serial.print(F("TX  0x100  FAILED ("));
      Serial.print(result);
      Serial.println(F(")"));
    }
  }

  // ---- Send ping every 2 s (ID 0x200) ----
  if (now - lastPing >= 2000) {
    lastPing = now;

    uint8_t ping[3] = { 'P', 'N', 'G' };
    byte result = CAN.sendMsgBuf(0x200, 0, 3, ping);
    if (result == CAN_OK) {
      Serial.println(F("TX  0x200  PING"));
    } else {
      Serial.print(F("TX  0x200  PING FAILED ("));
      Serial.print(result);
      Serial.println(F(")"));
    }
  }

  // ---- Receive any incoming frames ----
  if (canMsgReceived) {
    canMsgReceived = false;
    readAllFrames();
  }
}

void readAllFrames() {
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    CAN.readMsgBuf(&rxId, &len, buf);

    bool ext = (rxId & 0x80000000) != 0;  // extended frame flag
    bool rtr = (rxId & 0x40000000) != 0;  // remote request flag
    rxId &= 0x1FFFFFFF;                   // mask off flags

    Serial.print(F("RX  0x"));
    Serial.print(rxId, HEX);
    if (ext) Serial.print(F(" EXT"));
    if (rtr) Serial.print(F(" RTR"));
    Serial.print(F("  ["));
    Serial.print(len);
    Serial.print(F("]  "));

    for (byte i = 0; i < len; i++) {
      if (buf[i] < 0x10) Serial.print('0');
      Serial.print(buf[i], HEX);
      if (i < len - 1) Serial.print(' ');
    }
    Serial.println();
  }
}
