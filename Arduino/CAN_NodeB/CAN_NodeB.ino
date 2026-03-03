/*
 * CAN Node B — Receiver / Responder
 * 
 * Hardware: Arduino Uno + MCP2515 CAN module (8 MHz oscillator)
 *   INT → D2, CS → D10, SI → D11, SO → D12, SCK → D13
 * 
 * Behavior:
 *   - Listens for all incoming CAN frames and prints them to Serial
 *   - When it receives a PING (ID 0x200), it replies with PONG (ID 0x201)
 *   - Sends its own status message (ID 0x300) every 3 seconds
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
uint32_t rxCount      = 0;
uint32_t lastStatus   = 0;
uint32_t pongCount    = 0;

volatile bool canMsgReceived = false;

void canISR() {
  canMsgReceived = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("=== CAN Node B (Receiver) ==="));
  Serial.println(F("Initializing MCP2515 (8 MHz, 500 kbps)..."));

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("  MCP2515 init failed — retrying in 500 ms"));
    delay(500);
  }

  Serial.println(F("  MCP2515 init OK"));

  CAN.setMode(MCP_NORMAL);

  pinMode(CAN_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);

  Serial.println(F("Ready — listening for CAN frames\n"));
}

void loop() {
  uint32_t now = millis();

  // ---- Handle received frames ----
  if (canMsgReceived) {
    canMsgReceived = false;
    handleIncoming();
  }

  // ---- Send status every 3 seconds (ID 0x300) ----
  if (now - lastStatus >= 3000) {
    lastStatus = now;

    uint8_t data[8] = {0};
    data[0] = (uint8_t)(rxCount >> 8);
    data[1] = (uint8_t)(rxCount);
    data[2] = (uint8_t)(pongCount >> 8);
    data[3] = (uint8_t)(pongCount);
    data[4] = (uint8_t)(now >> 24);
    data[5] = (uint8_t)(now >> 16);
    data[6] = (uint8_t)(now >> 8);
    data[7] = (uint8_t)(now);

    byte result = CAN.sendMsgBuf(0x300, 0, 8, data);
    if (result == CAN_OK) {
      Serial.print(F("TX  0x300  STATUS  rxCount="));
      Serial.print(rxCount);
      Serial.print(F("  pongCount="));
      Serial.print(pongCount);
      Serial.print(F("  uptime="));
      Serial.println(now);
    } else {
      Serial.print(F("TX  0x300  STATUS FAILED ("));
      Serial.print(result);
      Serial.println(F(")"));
    }
  }
}

void handleIncoming() {
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    CAN.readMsgBuf(&rxId, &len, buf);

    bool ext = (rxId & 0x80000000) != 0;
    bool rtr = (rxId & 0x40000000) != 0;
    rxId &= 0x1FFFFFFF;

    rxCount++;

    // ---- Print the frame ----
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

    // ---- Decode known IDs ----
    if (rxId == 0x100) {
      Serial.print(F("  (heartbeat #"));
      Serial.print(buf[0]);
      Serial.print(F(")"));
    }

    Serial.println();

    // ---- Reply to PING with PONG ----
    if (rxId == 0x200 && len >= 3 && buf[0] == 'P' && buf[1] == 'N' && buf[2] == 'G') {
      pongCount++;
      uint8_t pong[4] = { 'P', 'O', 'N', 'G' };
      byte result = CAN.sendMsgBuf(0x201, 0, 4, pong);

      if (result == CAN_OK) {
        Serial.print(F("TX  0x201  PONG #"));
        Serial.println(pongCount);
      } else {
        Serial.print(F("TX  0x201  PONG FAILED ("));
        Serial.print(result);
        Serial.println(F(")"));
      }
    }
  }
}
