/*
  UHF Senior Module - Health Check / "Is it alive?" sketch
  ===========================================================
  Sends RFM_GET_INFO (0x0051) to the module and checks for a valid,
  CRC-correct response with STATUS = 0x00. GET_INFO is a good health
  check because per the guide (2.2.1) it only ever returns success
  (0x00) or nothing at all - so a valid framed reply with the expected
  version/serial fields is solid proof the module is powered, wired
  correctly, and its UART/firmware are working.

  Response payload on success (guide 2.2.1):
    HardVer  32 bytes  ASCII, NUL-padded
    FirmVer  32 bytes  ASCII, NUL-padded
    SN_code  12 bytes  ASCII, NUL-padded

  Wiring / serial port: same as the previous test sketch -
    Module TX -> Arduino RX, Module RX -> Arduino TX, GND -> GND,
    115200 baud, using a hardware serial port (Serial1 on Mega/
    Leonardo/ESP32 - SoftwareSerial is not reliable at 115200).

  Behaviour: runs a health check every CHECK_INTERVAL_MS and prints
  "Module ACTIVE" with version/serial info, or "Module NOT RESPONDING"
  if nothing valid comes back within the retry budget.
*/

#include <Arduino.h>

#define READER_SERIAL Serial1   // change to match your board's spare UART

const uint8_t  HEAD           = 0xCF;
const uint8_t  ADDR_BROADCAST = 0xFF;
const uint16_t CMD_GET_INFO   = 0x0051;

const uint8_t  MAX_RETRIES       = 3;
const uint32_t RESPONSE_TIMEOUT_MS = 800;
const uint32_t CHECK_INTERVAL_MS   = 5000; // how often loop() re-checks

// ---- CRC16, Appendix B of the guide -----------------------------------
uint16_t crc16Cal(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x0001) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
    }
  }
  return crc;
}

// ---- send a command frame: HEAD ADDR CMD(2) LEN Data[] CRC16(2) -------
void sendCommand(uint16_t cmd, const uint8_t *data, uint8_t dataLen) {
  uint8_t frame[8 + 255];
  uint8_t idx = 0;
  frame[idx++] = HEAD;
  frame[idx++] = ADDR_BROADCAST;
  frame[idx++] = (cmd >> 8) & 0xFF;
  frame[idx++] = cmd & 0xFF;
  frame[idx++] = dataLen;
  for (uint8_t i = 0; i < dataLen; i++) frame[idx++] = data[i];

  uint16_t crc = crc16Cal(frame, idx);
  frame[idx++] = (crc >> 8) & 0xFF;
  frame[idx++] = crc & 0xFF;

  // clear any stale bytes before we send, so we don't parse an old reply
  while (READER_SERIAL.available()) READER_SERIAL.read();
  READER_SERIAL.write(frame, idx);
}

// ---- read + validate one response frame --------------------------------
bool readResponse(uint16_t &cmdOut, uint8_t &statusOut,
                   uint8_t *payloadOut, uint8_t &payloadLenOut,
                   uint32_t timeoutMs) {
  uint32_t start = millis();

  while (true) {
    if (millis() - start > timeoutMs) return false;
    if (READER_SERIAL.available()) {
      if (READER_SERIAL.read() == HEAD) break;
    }
  }

  uint8_t header[4]; // ADDR CMD_HI CMD_LO LEN
  for (uint8_t i = 0; i < 4; i++) {
    while (!READER_SERIAL.available()) {
      if (millis() - start > timeoutMs) return false;
    }
    header[i] = READER_SERIAL.read();
  }

  uint16_t cmd = ((uint16_t)header[1] << 8) | header[2];
  uint8_t len  = header[3];
  if (len == 0) return false;

  uint8_t infoField[255];
  for (uint8_t i = 0; i < len; i++) {
    while (!READER_SERIAL.available()) {
      if (millis() - start > timeoutMs) return false;
    }
    infoField[i] = READER_SERIAL.read();
  }

  uint8_t crcBytes[2];
  for (uint8_t i = 0; i < 2; i++) {
    while (!READER_SERIAL.available()) {
      if (millis() - start > timeoutMs) return false;
    }
    crcBytes[i] = READER_SERIAL.read();
  }
  uint16_t receivedCrc = ((uint16_t)crcBytes[0] << 8) | crcBytes[1];

  uint8_t check[6 + 255];
  uint8_t ci = 0;
  check[ci++] = HEAD;
  check[ci++] = header[0];
  check[ci++] = header[1];
  check[ci++] = header[2];
  check[ci++] = header[3];
  for (uint8_t i = 0; i < len; i++) check[ci++] = infoField[i];

  if (crc16Cal(check, ci) != receivedCrc) return false; // corrupted frame

  statusOut = infoField[0];
  payloadLenOut = len - 1;
  for (uint8_t i = 0; i < payloadLenOut; i++) payloadOut[i] = infoField[1 + i];
  cmdOut = cmd;
  return true;
}

// ---- extract a NUL-padded ASCII field into a plain C string -----------
void extractString(const uint8_t *src, uint8_t maxLen, char *dst) {
  uint8_t i = 0;
  for (; i < maxLen; i++) {
    if (src[i] == 0x00) break;
    dst[i] = (char)src[i];
  }
  dst[i] = '\0';
}

// -------------------------------------------------------------------------
// checkModuleStatus() - the actual health check.
// Sends RFM_GET_INFO, retries a few times, returns true/false and (if
// successful) prints hardware/firmware version + serial number.
// -------------------------------------------------------------------------
bool checkModuleStatus() {
  uint16_t cmd;
  uint8_t status;
  uint8_t payload[255];
  uint8_t payloadLen;

  for (uint8_t attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    sendCommand(CMD_GET_INFO, nullptr, 0);

    if (readResponse(cmd, status, payload, payloadLen, RESPONSE_TIMEOUT_MS)) {
      if (cmd == CMD_GET_INFO && status == 0x00 && payloadLen >= 76) {
        char hardVer[33], firmVer[33], sn[13];
        extractString(payload, 32, hardVer);
        extractString(payload + 32, 32, firmVer);
        extractString(payload + 64, 12, sn);

        Serial.println(F("Module ACTIVE - responding correctly."));
        Serial.print(F("  Hardware version: ")); Serial.println(hardVer);
        Serial.print(F("  Firmware version: ")); Serial.println(firmVer);
        Serial.print(F("  Serial number:    ")); Serial.println(sn);
        return true;
      } else {
        Serial.print(F("Module responded but with unexpected data (STATUS=0x"));
        Serial.print(status, HEX);
        Serial.println(F("). Retrying..."));
      }
    } else {
      Serial.print(F("No valid response on attempt "));
      Serial.print(attempt);
      Serial.print(F("/"));
      Serial.println(MAX_RETRIES);
    }
    delay(100);
  }

  Serial.println(F("Module NOT RESPONDING - check power, wiring (TX/RX not swapped), and baud rate (115200)."));
  return false;
}

uint32_t lastCheck = 0;

void setup() {
  Serial.begin(115200);         // USB serial monitor
  READER_SERIAL.begin(115200);  // module UART, per guide section 1.1
  delay(500);

  Serial.println(F("Checking UHF module status..."));
  checkModuleStatus();
  lastCheck = millis();
}

void loop() {
  // Re-run the health check periodically.
  if (millis() - lastCheck >= CHECK_INTERVAL_MS) {
    Serial.println(F("---- re-checking module status ----"));
    checkModuleStatus();
    lastCheck = millis();
  }

  // Also let you trigger an on-demand check from the Serial Monitor.
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') {
      Serial.println(F("---- manual status check ----"));
      checkModuleStatus();
      lastCheck = millis();
    }
  }
}
