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
 *   VEL <device_id> <rpm> [slot]
 *      Example: VEL 1 300.0
 *      Velocity closed-loop (api10=0x000). Default slot=1.
 *
 *   POS <device_id> <rotations> [slot]
 *      Example: POS 1 5.0
 *      Position closed-loop (api10=0x004). Default slot=0.
 *
 *   DUTY <device_id> <-1.0..1.0>
 *      Example: DUTY 1 0.3
 *      Open-loop duty cycle (api10=0x002, no PID gains needed).
 *
 *   VOLTS <device_id> <voltage>
 *      Example: VOLTS 1 -3.5
 *      Voltage control (api10=0x005).
 *
 *   STOP <device_id>
 *      Zeroes all control modes
 *
 *   HB <device_id> <0|1>
 *      Enable/disable automatic heartbeat (20 ms)
 *
 *   RATE <device_id> <status_idx> <period_ms>
 *      Set SPARK MAX status frame period (idx 0..6)
 *      Status frames relevant to encoders:
 *        0: Applied output, faults
 *        1: Faults and warnings
 *        2: Encoder velocity (RPM) + position (rotations)
 *        3: Analog sensor voltage/velocity/position
 *        4: Alternate encoder velocity/position
 *        5: Duty-cycle absolute encoder position + velocity  <-- use this for absolute
 *        6: Duty-cycle absolute encoder velocity + frequency
 *
 * CAN RX lines emitted to serial:
 *   RX <id_hex> <dlc> <data_hex>
 *
 * Decoded status frames (if DECODE is enabled):
 *   POS <device_id> <rotations>        (from Status 2, relative encoder)
 *   ABSPOS <device_id> <0..1>          (from Status 5, absolute encoder)
 *   VEL <device_id> <rpm>              (from Status 2)
 */

#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(CAN_CS_PIN);

volatile bool canMsgReceived = false;

// Set to 1 to emit decoded POS/VEL/ABSPOS lines from status frames
#define DECODE_STATUS 1
// Set to 1 to also print raw "RX 0x... 8 ..." lines for every CAN frame.
// Fine at 500000 baud; disable only if you need to reduce output volume.
#define PRINT_RAW_RX 1

bool heartbeatEnabled = true;
uint8_t heartbeatDeviceId = 1;
uint32_t lastHeartbeatMs = 0;
const uint32_t HEARTBEAT_PERIOD_MS = 20;
bool heartbeatJustEnabled = true; // send zero setpoint on first heartbeat after boot

char lineBuf[128];
uint8_t linePos = 0;

static inline uint32_t makeSparkId(uint8_t devType, uint8_t mfr, uint8_t apiClass, uint8_t apiIndex, uint8_t devId) {
  return ((uint32_t)devType << 24) |
         ((uint32_t)mfr << 16) |
         ((uint32_t)(apiClass & 0x3F) << 10) |
         ((uint32_t)(apiIndex & 0x0F) << 6) |
         ((uint32_t)(devId & 0x3F));
}

static inline uint32_t makeSparkIdApi10(uint16_t api10, uint8_t devId) {
  uint8_t apiClass = (uint8_t)((api10 >> 4) & 0x3F);
  uint8_t apiIndex = (uint8_t)(api10 & 0x0F);
  return makeSparkId(2, 5, apiClass, apiIndex, devId);
}

void canISR() {
  canMsgReceived = true;
}

void setup() {
  Serial.begin(500000);
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
  Serial.println(F("CMDS: TX, VEL, POS, DUTY, VOLTS, STOP, HB, RATE, PING"));
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
  // Retry with timeout in case all TX buffers are busy.
  // mcp_can can spin forever if no buffer is free; cap it to avoid blocking the loop.
  const uint32_t TX_TIMEOUT_MS = 5;
  uint32_t start = millis();
  byte result;
  do {
    result = CAN.sendMsgBuf(canId, 1, dlc, (uint8_t*)data);
    if (result == CAN_OK) return true;
  } while ((millis() - start) < TX_TIMEOUT_MS);
  return false;
}

void sendHeartbeat() {
  uint8_t payload[8] = {0};
  // Non-RIO heartbeat: API10=0x0B2, device ID 0 (broadcast to all devices).
  // Payload is a 64-bit bitfield: bit N = 1 enables device with CAN ID N.
  // Observed from REV Hardware Client: byte0=0x02 enables device ID 1.
  // Set all bits in bytes 0-7 to enable any device ID 1-63.
  // Sending only the specific device bit to match Hardware Client behavior.
  if (heartbeatDeviceId < 8) {
    payload[0] = (uint8_t)(1 << heartbeatDeviceId);
  } else {
    payload[heartbeatDeviceId / 8] = (uint8_t)(1 << (heartbeatDeviceId % 8));
  }
  uint32_t hbId = makeSparkIdApi10(0x0B2, 0); // devId=0 = broadcast
  sendCanFrame(hbId, payload, 8);
}

// api10 values (spark-frames-2.1.0 spec, firmware 26.x):
//   0x000 = Velocity setpoint (RPM, closed-loop)
//   0x002 = Duty-cycle setpoint (-1..1, open-loop)
//   0x004 = Position setpoint (rotations, closed-loop)
//   0x005 = Voltage setpoint (V)
void sendSetpoint(uint8_t deviceId, float value, uint8_t pidSlot,
                  uint16_t api10, const __FlashStringHelper* label) {
  uint8_t payload[8] = {0};
  union { float f; uint8_t b[4]; } u;
  u.f = value;
  payload[0] = u.b[0];
  payload[1] = u.b[1];
  payload[2] = u.b[2];
  payload[3] = u.b[3];
  // payload[6] bits [1:0] = pidSlot (spec: bitPosition 48, 2 bits)
  payload[6] = pidSlot & 0x03;
  uint32_t canId = makeSparkIdApi10(api10, deviceId);
  bool ok = sendCanFrame(canId, payload, 8);
  Serial.print(F("ACK "));
  Serial.print(label);
  Serial.print(' ');
  Serial.print(deviceId);
  Serial.print(' ');
  Serial.print(value, 4);
  Serial.print(' ');
  Serial.println(ok ? F("OK") : F("ERR"));
}

void sendVoltageCommand(uint8_t deviceId, float volts) {
  if (volts > 12.0f) volts = 12.0f;
  if (volts < -12.0f) volts = -12.0f;
  sendSetpoint(deviceId, volts, 0, 0x005, F("VOLTS"));
}


void sendStatusRateCommand(uint8_t deviceId, uint8_t statusIdx, uint16_t periodMs) {
  if (statusIdx > 15) statusIdx = 15;

  uint16_t apiLegacy = (uint16_t)(0x060 + statusIdx);
  uint16_t apiShift  = (uint16_t)(0x2E0 + statusIdx);

  uint32_t canIdLegacy = makeSparkIdApi10(apiLegacy, deviceId);
  uint32_t canIdShift  = makeSparkIdApi10(apiShift, deviceId);

  uint8_t payload[8] = {0};
  payload[0] = (uint8_t)(periodMs & 0xFF);
  payload[1] = (uint8_t)((periodMs >> 8) & 0xFF);
  bool ok1 = sendCanFrame(canIdLegacy, payload, 8);
  bool ok2 = sendCanFrame(canIdShift, payload, 8);
  bool ok = ok1 || ok2;

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

  if (strcmp(cmd, "VEL") == 0) {
    char* devTok  = strtok(nullptr, " \t\r\n");
    char* vTok    = strtok(nullptr, " \t\r\n");
    char* slotTok = strtok(nullptr, " \t\r\n");
    uint16_t dev16 = 0, slot16 = 1; // default slot 1 for velocity
    float v = 0.0f;
    if (!parseUInt(devTok, &dev16) || !parseFloatVal(vTok, &v)) {
      Serial.println(F("ERR VEL args"));
      return;
    }
    if (slotTok) parseUInt(slotTok, &slot16);
    sendSetpoint((uint8_t)(dev16 & 0x3F), v, (uint8_t)(slot16 & 0x03), 0x000, F("VEL"));
    return;
  }

  if (strcmp(cmd, "POS") == 0) {
    char* devTok  = strtok(nullptr, " \t\r\n");
    char* vTok    = strtok(nullptr, " \t\r\n");
    char* slotTok = strtok(nullptr, " \t\r\n");
    uint16_t dev16 = 0, slot16 = 0; // default slot 0 for position
    float v = 0.0f;
    if (!parseUInt(devTok, &dev16) || !parseFloatVal(vTok, &v)) {
      Serial.println(F("ERR POS args"));
      return;
    }
    if (slotTok) parseUInt(slotTok, &slot16);
    sendSetpoint((uint8_t)(dev16 & 0x3F), v, (uint8_t)(slot16 & 0x03), 0x004, F("POS"));
    return;
  }

  if (strcmp(cmd, "DUTY") == 0) {
    char* devTok = strtok(nullptr, " \t\r\n");
    char* vTok   = strtok(nullptr, " \t\r\n");
    uint16_t dev16 = 0;
    float v = 0.0f;
    if (!parseUInt(devTok, &dev16) || !parseFloatVal(vTok, &v)) {
      Serial.println(F("ERR DUTY args"));
      return;
    }
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    sendSetpoint((uint8_t)(dev16 & 0x3F), v, 0, 0x002, F("DUTY"));
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    char* devTok = strtok(nullptr, " \t\r\n");
    uint16_t dev16 = 0;
    if (!parseUInt(devTok, &dev16)) {
      Serial.println(F("ERR STOP args"));
      return;
    }
    // Zero all control modes and all PID slots
    uint8_t d = (uint8_t)(dev16 & 0x3F);
    static const uint16_t stopApis[] = {0x000, 0x002, 0x004, 0x005};
    for (uint8_t a = 0; a < 4; a++)
      for (uint8_t s = 0; s < 4; s++)
        sendSetpoint(d, 0.0f, s, stopApis[a], F("STOP"));
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
    bool wasEnabled = heartbeatEnabled;
    heartbeatEnabled = (en16 != 0);
    if (heartbeatEnabled && !wasEnabled) heartbeatJustEnabled = true;

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

  if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("PONG"));
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

#if PRINT_RAW_RX
    Serial.print(F("RX 0x"));
    Serial.print((uint32_t)rxId, HEX);
    Serial.print(' ');
    Serial.print((uint8_t)len);
    Serial.print(' ');
    for (uint8_t i = 0; i < len; i++) {
      writeHexByte(buf[i]);
    }
    Serial.println();
#endif

#if DECODE_STATUS
    // Extract device ID and API10 from CAN ID.
    // CAN ID layout: [28:24]=devType, [23:16]=mfr, [15:10]=apiClass, [9:6]=apiIndex, [5:0]=devId
    uint8_t devId   = (uint8_t)(rxId & 0x3F);
    uint8_t apiIdx  = (uint8_t)((rxId >> 6) & 0x0F);
    uint8_t apiCls  = (uint8_t)((rxId >> 10) & 0x3F);
    uint8_t devType = (uint8_t)((rxId >> 24) & 0x1F);
    uint8_t mfr     = (uint8_t)((rxId >> 16) & 0xFF);
    uint16_t api10  = (uint16_t)(apiCls << 4) | apiIdx;

    // Only decode SPARK MAX frames (devType=2, mfr=5)
    // SPARK MAX firmware broadcasts using shifted API10 values (+0x280).
    // e.g. Status 2 (legacy 0x062) is broadcast as 0x2E2. Accept both.
    if (devType == 2 && mfr == 5 && len >= 4) {
      union { float f; uint8_t b[4]; } u;
      uint16_t unshifted = (api10 >= 0x280) ? (api10 - 0x280) : api10;

      if (unshifted == 0x062 && len >= 8) {
        // Status 2: bytes 0-3 = velocity (float32, RPM), bytes 4-7 = position (float32, rotations)
        union { float f; uint8_t b[4]; } v;
        v.b[0] = buf[0]; v.b[1] = buf[1]; v.b[2] = buf[2]; v.b[3] = buf[3];
        Serial.print(F("VEL "));
        Serial.print(devId);
        Serial.print(' ');
        Serial.println(v.f, 2);
        u.b[0] = buf[4]; u.b[1] = buf[5]; u.b[2] = buf[6]; u.b[3] = buf[7];
        Serial.print(F("POS "));
        Serial.print(devId);
        Serial.print(' ');
        Serial.println(u.f, 5);
      } else if (unshifted == 0x063 && len >= 8) {
        // Status 3: analog sensor. For analog absolute encoders wired to the
        // SPARK MAX data port analog input: bytes 0-3 = analog velocity (float32),
        // bytes 4-7 = analog position (float32, 0..1 per revolution).
        u.b[0] = buf[4]; u.b[1] = buf[5]; u.b[2] = buf[6]; u.b[3] = buf[7];
        Serial.print(F("ABSPOS "));
        Serial.print(devId);
        Serial.print(' ');
        Serial.println(u.f, 5);
      } else if (unshifted == 0x060) {
        // Status 0: applied output (int16, divide by 32767 for -1..1)
        int16_t rawOut = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        Serial.print(F("OUTPUT "));
        Serial.print(devId);
        Serial.print(' ');
        Serial.println((float)rawOut / 32767.0f, 4);
      }
    }
#endif
  }
}

void loop() {
  // Read serial commands FIRST so they are never starved by CAN frame printing.
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

  uint32_t now = millis();

  if (heartbeatEnabled && (now - lastHeartbeatMs >= HEARTBEAT_PERIOD_MS)) {
    lastHeartbeatMs = now;
    sendHeartbeat();
    if (heartbeatJustEnabled) {
      heartbeatJustEnabled = false;
      // Zero all control modes so motor doesn't resume a stale command.
      static const uint16_t stopApis[] = {0x000, 0x002, 0x004, 0x005};
      for (uint8_t a = 0; a < 4; a++)
        sendSetpoint(heartbeatDeviceId, 0.0f, 0, stopApis[a], F("STOP"));
    }
  }

  // Process CAN receive once per loop (handles both interrupt and polling cases).
  canMsgReceived = false;
  readCanFrames();
}
