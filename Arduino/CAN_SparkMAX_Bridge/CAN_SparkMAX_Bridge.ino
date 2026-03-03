/*
 * SPARK MAX CAN Bridge (Arduino + MCP2515)
 *
 * Hardware:
 *   - Arduino + MCP2515 (8 MHz crystal)
 *   - CS -> D10, INT -> D2
 *
 * Serial protocol (115200, newline-terminated ASCII):
 *   TX <id_hex> <dlc> <data_hex>
 *      Example: TX 0x2050281 8 0000000000000000
 *
 *   VOLTS <device_id> <voltage>
 *      Example: VOLTS 1 -3.5
 *      Sends SPARK MAX voltage control command (ControlType::kVoltage)
 *
 *   STOP <device_id>
 *      Equivalent to VOLTS <device_id> 0
 *
 *   HB <device_id> <0|1>
 *      Enable/disable automatic heartbeat (20 ms)
 *
 *   RATE <device_id> <status_idx> <period_ms>
 *      Set SPARK MAX status frame period (idx 0..6)
 *
 * CAN RX lines emitted to serial:
 *   RX <id_hex> <dlc> <data_hex>
 */

#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(CAN_CS_PIN);

volatile bool canMsgReceived = false;

bool heartbeatEnabled = true;
uint8_t heartbeatDeviceId = 1;
uint32_t lastHeartbeatMs = 0;
const uint32_t HEARTBEAT_PERIOD_MS = 20;

char lineBuf[128];
uint8_t linePos = 0;

static inline uint32_t makeSparkId(uint8_t devType, uint8_t mfr, uint8_t apiClass, uint8_t apiIndex, uint8_t devId) {
  return ((uint32_t)devType << 24) |
         ((uint32_t)mfr << 16) |
         ((uint32_t)(apiClass & 0x3F) << 10) |
         ((uint32_t)(apiIndex & 0x0F) << 6) |
         ((uint32_t)(devId & 0x3F));
}

void canISR() {
  canMsgReceived = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("BRIDGE BOOT"));
  Serial.println(F("INIT MCP2515 @ 1Mbps (8MHz)"));

  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("ERR INIT"));
    delay(500);
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);

  Serial.println(F("READY"));
  Serial.println(F("CMDS: TX, VOLTS, STOP, HB, RATE"));
}

bool parseHexId(const char* token, uint32_t* out) {
  if (!token || !out) return false;
  char* endp = nullptr;
  unsigned long v = strtoul(token, &endp, 16);
  if (endp == token) return false;
  *out = (uint32_t)v;
  return true;
}

bool parseUInt(const char* token, uint16_t* out) {
  if (!token || !out) return false;
  char* endp = nullptr;
  unsigned long v = strtoul(token, &endp, 10);
  if (endp == token || v > 65535UL) return false;
  *out = (uint16_t)v;
  return true;
}

bool parseFloatVal(const char* token, float* out) {
  if (!token || !out) return false;

  const char* p = token;
  bool neg = false;
  if (*p == '+' || *p == '-') {
    neg = (*p == '-');
    p++;
  }

  bool sawDigit = false;
  unsigned long intPart = 0;
  while (*p >= '0' && *p <= '9') {
    sawDigit = true;
    intPart = intPart * 10UL + (unsigned long)(*p - '0');
    p++;
  }

  float fracPart = 0.0f;
  float scale = 1.0f;
  if (*p == '.') {
    p++;
    while (*p >= '0' && *p <= '9') {
      sawDigit = true;
      fracPart = fracPart * 10.0f + (float)(*p - '0');
      scale *= 10.0f;
      p++;
    }
  }

  if (!sawDigit) return false;
  if (*p != '\0') return false;

  float v = (float)intPart + (fracPart / scale);
  *out = neg ? -v : v;
  return true;
}

void writeHexByte(uint8_t b) {
  const char* h = "0123456789ABCDEF";
  Serial.print(h[(b >> 4) & 0x0F]);
  Serial.print(h[b & 0x0F]);
}

bool sendCanFrame(uint32_t canId, const uint8_t* data, uint8_t dlc) {
  if (dlc > 8) dlc = 8;
  byte result = CAN.sendMsgBuf(canId, 1, dlc, (uint8_t*)data); // ext frame
  return result == CAN_OK;
}

void sendHeartbeat() {
  uint8_t payload[8] = {0};
  uint32_t hbId = makeSparkId(2, 5, 5, 8, heartbeatDeviceId);
  sendCanFrame(hbId, payload, 8);
}

void sendVoltageCommand(uint8_t deviceId, float volts) {
  if (volts > 12.0f) volts = 12.0f;
  if (volts < -12.0f) volts = -12.0f;

  uint32_t canId = makeSparkId(2, 5, 2, 2, deviceId); // ControlType::kVoltage (index 2)
  uint8_t payload[8] = {0};

  union {
    float f;
    uint8_t b[4];
  } u;
  u.f = volts;

  payload[0] = u.b[0];
  payload[1] = u.b[1];
  payload[2] = u.b[2];
  payload[3] = u.b[3];
  payload[4] = 0;
  payload[5] = 0;
  payload[6] = 0;
  payload[7] = 0;

  bool ok = sendCanFrame(canId, payload, 8);
  Serial.print(F("ACK VOLTS "));
  Serial.print(deviceId);
  Serial.print(' ');
  Serial.print(volts, 3);
  Serial.print(' ');
  Serial.println(ok ? F("OK") : F("ERR"));
}

void sendStatusRateCommand(uint8_t deviceId, uint8_t statusIdx, uint16_t periodMs) {
  if (statusIdx > 15) statusIdx = 15;
  uint32_t canId = makeSparkId(2, 5, 0x06, statusIdx, deviceId);
  uint8_t payload[8] = {0};
  payload[0] = (uint8_t)(periodMs & 0xFF);
  payload[1] = (uint8_t)((periodMs >> 8) & 0xFF);
  bool ok = sendCanFrame(canId, payload, 8);

  Serial.print(F("ACK RATE "));
  Serial.print(deviceId);
  Serial.print(' ');
  Serial.print(statusIdx);
  Serial.print(' ');
  Serial.print(periodMs);
  Serial.print(' ');
  Serial.println(ok ? F("OK") : F("ERR"));
}

bool hexToBytes(const char* hex, uint8_t* out, uint8_t maxLen, uint8_t* outLen) {
  if (!hex || !out || !outLen) return false;

  uint8_t n = 0;
  uint8_t highNibble = 0;
  bool haveHigh = false;

  while (*hex) {
    char c = *hex++;
    if (c == ' ' || c == '\t') continue;

    uint8_t val;
    if (c >= '0' && c <= '9') val = c - '0';
    else if (c >= 'a' && c <= 'f') val = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
    else return false;

    if (!haveHigh) {
      highNibble = val;
      haveHigh = true;
    } else {
      if (n >= maxLen) return false;
      out[n++] = (highNibble << 4) | val;
      haveHigh = false;
    }
  }

  if (haveHigh) return false;
  *outLen = n;
  return true;
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " \t\r\n");
  if (!cmd) return;

  if (strcmp(cmd, "TX") == 0) {
    char* idTok = strtok(nullptr, " \t\r\n");
    char* dlcTok = strtok(nullptr, " \t\r\n");
    char* dataTok = strtok(nullptr, "\r\n");

    uint32_t canId = 0;
    uint16_t dlc16 = 0;
    if (!parseHexId(idTok, &canId) || !parseUInt(dlcTok, &dlc16)) {
      Serial.println(F("ERR TX args"));
      return;
    }

    uint8_t dlc = (uint8_t)dlc16;
    if (dlc > 8) dlc = 8;

    uint8_t data[8] = {0};
    uint8_t dataLen = 0;

    if (dataTok && strlen(dataTok) > 0) {
      if (!hexToBytes(dataTok, data, 8, &dataLen)) {
        Serial.println(F("ERR TX data"));
        return;
      }
      if (dataLen < dlc) {
        for (uint8_t i = dataLen; i < dlc; ++i) data[i] = 0;
      }
    }

    bool ok = sendCanFrame(canId, data, dlc);
    Serial.println(ok ? F("ACK TX OK") : F("ACK TX ERR"));
    return;
  }

  if (strcmp(cmd, "VOLTS") == 0) {
    char* devTok = strtok(nullptr, " \t\r\n");
    char* vTok = strtok(nullptr, " \t\r\n");

    uint16_t dev16 = 0;
    float v = 0.0f;
    if (!parseUInt(devTok, &dev16) || !parseFloatVal(vTok, &v)) {
      Serial.println(F("ERR VOLTS args"));
      return;
    }

    sendVoltageCommand((uint8_t)(dev16 & 0x3F), v);
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    char* devTok = strtok(nullptr, " \t\r\n");
    uint16_t dev16 = 0;
    if (!parseUInt(devTok, &dev16)) {
      Serial.println(F("ERR STOP args"));
      return;
    }
    sendVoltageCommand((uint8_t)(dev16 & 0x3F), 0.0f);
    return;
  }

  if (strcmp(cmd, "HB") == 0) {
    char* devTok = strtok(nullptr, " \t\r\n");
    char* enTok = strtok(nullptr, " \t\r\n");

    uint16_t dev16 = 0;
    uint16_t en16 = 0;
    if (!parseUInt(devTok, &dev16) || !parseUInt(enTok, &en16)) {
      Serial.println(F("ERR HB args"));
      return;
    }

    heartbeatDeviceId = (uint8_t)(dev16 & 0x3F);
    heartbeatEnabled = (en16 != 0);

    Serial.print(F("ACK HB "));
    Serial.print(heartbeatDeviceId);
    Serial.print(' ');
    Serial.println(heartbeatEnabled ? F("ON") : F("OFF"));
    return;
  }

  if (strcmp(cmd, "RATE") == 0) {
    char* devTok = strtok(nullptr, " \t\r\n");
    char* idxTok = strtok(nullptr, " \t\r\n");
    char* perTok = strtok(nullptr, " \t\r\n");

    uint16_t dev16 = 0;
    uint16_t idx16 = 0;
    uint16_t per16 = 0;
    if (!parseUInt(devTok, &dev16) || !parseUInt(idxTok, &idx16) || !parseUInt(perTok, &per16)) {
      Serial.println(F("ERR RATE args"));
      return;
    }

    sendStatusRateCommand((uint8_t)(dev16 & 0x3F), (uint8_t)(idx16 & 0x0F), per16);
    return;
  }

  Serial.println(F("ERR unknown cmd"));
}

void readCanFrames() {
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8] = {0};

    CAN.readMsgBuf(&rxId, &len, buf);

    rxId &= 0x1FFFFFFF;
    if (len > 8) len = 8;

    Serial.print(F("RX 0x"));
    Serial.print((uint32_t)rxId, HEX);
    Serial.print(' ');
    Serial.print((uint8_t)len);
    Serial.print(' ');

    for (uint8_t i = 0; i < len; i++) {
      writeHexByte(buf[i]);
    }
    Serial.println();
  }
}

void loop() {
  uint32_t now = millis();

  if (heartbeatEnabled && (now - lastHeartbeatMs >= HEARTBEAT_PERIOD_MS)) {
    lastHeartbeatMs = now;
    sendHeartbeat();
  }

  if (canMsgReceived) {
    canMsgReceived = false;
    readCanFrames();
  }

  // Also poll in case INT is not wired/working.
  readCanFrames();

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      lineBuf[linePos] = '\0';
      handleCommand(lineBuf);
      linePos = 0;
    } else if (c != '\r') {
      if (linePos < sizeof(lineBuf) - 1) {
        lineBuf[linePos++] = c;
      }
    }
  }
}
