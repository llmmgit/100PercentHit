/*
  100 Percent Hit / CODBOT M100
  ESP32 UDP <-> STM32 UART2 bridge + BNO085 UART-RVC + Battery ADC
  + LD14 on hardware UART0 RX remapped to GPIO32

  UART allocation after setup():
    UART0 RX=GPIO32: LD14 115200 input; TX=GPIO1: USB serial diagnostics
    UART1 RX=GPIO26: BNO085 UART-RVC
    UART2 RX/TX=GPIO16/17: STM32

  Safety:
    LD14 missing or invalid does not stop or block M100.
    Existing 500 ms command timeout STOP remains enabled.
*/
#include <WiFi.h>
#include <WiFiUdp.h>

#define USE_RVC_IMU          1
#define USE_BATTERY_ADC      1
#define USE_FAILSAFE_STOP    1
#define FORWARD_STM32_TO_PC  1
#define USE_LD14             1

// ================= WiFi =================
const char* WIFI_SSID = "YOUR_WIFI_NAME_HERE";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD_HERE";
#define USE_STATIC_IP false
IPAddress LOCAL_IP(192,168,1,222);
IPAddress GATEWAY(192,168,1,1);
IPAddress SUBNET(255,255,255,0);
IPAddress DNS1(192,168,1,1);

// ================= UDP =================
const uint16_t UDP_CMD_PORT   = 4210;
const uint16_t UDP_TELEM_PORT = 4211;
const uint16_t UDP_LD14_PORT  = 4212;
WiFiUDP udp;
WiFiUDP lidarUdp;
IPAddress pcIP;
uint16_t pcSourcePort = 0;
bool hasPC = false;
uint8_t udpRxBuf[512];
uint8_t uartRxBuf[256];

// ================= STM32 UART2 =================
HardwareSerial STM32Serial(2);
const int STM32_RX_PIN = 16;
const int STM32_TX_PIN = 17;
const uint32_t STM32_BAUD = 115200;

// ================= BNO085 UART-RVC / UART1 =================
#if USE_RVC_IMU
HardwareSerial BNO085Serial(1);
const int BNO085_RX_PIN = 26;
const int BNO085_TX_PIN = 27;  // not connected to BNO085
const uint32_t BNO085_RVC_BAUD = 115200;
uint8_t rvcFrame[19];
uint8_t rvcPos = 0;
bool bnoOK = false;
uint32_t lastRvcMs = 0;
uint32_t rvcGoodCount = 0;
uint32_t rvcBadCount = 0;
float imuYawDeg = 0.0f;
float imuPitchDeg = 0.0f;
float imuRollDeg = 0.0f;
float linAccX = 0.0f;
float linAccY = 0.0f;
float linAccZ = 0.0f;

static int16_t readLE16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool rvcChecksumOK(const uint8_t* p) {
  uint8_t sum = 0;
  // Debug Fix: AA AA headers are excluded; sum p[2]..p[17].
  for (int i = 2; i < 18; ++i) sum = (uint8_t)(sum + p[i]);
  return sum == p[18];
}

void parseRvcFrame(const uint8_t* p) {
  imuYawDeg   = readLE16(&p[3])  / 100.0f;
  imuPitchDeg = readLE16(&p[5])  / 100.0f;
  imuRollDeg  = readLE16(&p[7])  / 100.0f;
  const float G0 = 9.80665f;
  linAccX = readLE16(&p[9])  / 1000.0f * G0;
  linAccY = readLE16(&p[11]) / 1000.0f * G0;
  linAccZ = readLE16(&p[13]) / 1000.0f * G0;
  bnoOK = true;
  lastRvcMs = millis();
  rvcGoodCount++;
}

void feedRvcByte(uint8_t b) {
  if (rvcPos == 0) {
    if (b == 0xAA) rvcFrame[rvcPos++] = b;
    return;
  }
  if (rvcPos == 1) {
    if (b == 0xAA) rvcFrame[rvcPos++] = b;
    else rvcPos = 0;
    return;
  }
  rvcFrame[rvcPos++] = b;
  if (rvcPos == sizeof(rvcFrame)) {
    if (rvcChecksumOK(rvcFrame)) parseRvcFrame(rvcFrame);
    else rvcBadCount++;
    rvcPos = 0;
  }
}

void setupBNO085Rvc() {
  BNO085Serial.setRxBufferSize(512);
  BNO085Serial.begin(BNO085_RVC_BAUD, SERIAL_8N1, BNO085_RX_PIN, BNO085_TX_PIN);
}

void readBNO085Rvc() {
  while (BNO085Serial.available() > 0) {
    feedRvcByte((uint8_t)BNO085Serial.read());
  }
  if (bnoOK && millis() - lastRvcMs > 500) bnoOK = false;
}
#else
bool bnoOK = false;
uint32_t rvcGoodCount = 0, rvcBadCount = 0;
float imuYawDeg=0, imuPitchDeg=0, imuRollDeg=0;
float linAccX=0, linAccY=0, linAccZ=0;
void setupBNO085Rvc() {}
void readBNO085Rvc() {}
#endif

// ================= LD14 / hardware UART0 =================
#if USE_LD14
const int LD14_RX_PIN = 32;
const int UART0_TX_PIN = 1;  // keep USB serial diagnostics on GPIO1
const uint32_t LD14_BAUD = 115200;
const uint32_t LD14_OFFLINE_MS = 500;
const int LD14_READ_BUDGET = 256;
uint8_t ldFrame[47];
uint8_t ldPos = 0;
uint32_t ldBytes = 0;
uint32_t ldHeader54 = 0;
uint32_t ldHeader542C = 0;
uint32_t ldGood = 0;
uint32_t ldBad = 0;
uint32_t lastLdMs = 0;
bool ldOnline = false;

uint8_t ldCrc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void setupLD14() {
  // UART0 is Serial. Remap RX to GPIO32 and retain TX on GPIO1.
  Serial.begin(LD14_BAUD, SERIAL_8N1, LD14_RX_PIN, UART0_TX_PIN);
  delay(50);
  Serial.println("UART0 RX remapped to GPIO32 for LD14; TX remains GPIO1.");
}

void sendLD14Frame(const uint8_t* p) {
  if (!hasPC) return;
  lidarUdp.beginPacket(pcIP, UDP_LD14_PORT);
  lidarUdp.write((const uint8_t*)"LD14:", 5);
  lidarUdp.write(p, 47);
  lidarUdp.endPacket();
}

void feedLD14Byte(uint8_t b) {
  ldBytes++;
  if (ldPos == 0) {
    if (b == 0x54) {
      ldHeader54++;
      ldFrame[ldPos++] = b;
    }
    return;
  }
  if (ldPos == 1) {
    if (b == 0x2C) {
      ldHeader542C++;
      ldFrame[ldPos++] = b;
    } else {
      ldPos = 0;
      if (b == 0x54) {
        ldHeader54++;
        ldFrame[ldPos++] = b;
      }
    }
    return;
  }
  ldFrame[ldPos++] = b;
  if (ldPos == sizeof(ldFrame)) {
    if (ldCrc8(ldFrame, 46) == ldFrame[46]) {
      ldGood++;
      lastLdMs = millis();
      ldOnline = true;
      sendLD14Frame(ldFrame);
    } else {
      ldBad++;
    }
    ldPos = 0;
  }
}

void readLD14() {
  int budget = LD14_READ_BUDGET;
  while (budget-- > 0 && Serial.available() > 0) {
    feedLD14Byte((uint8_t)Serial.read());
  }
  if (ldOnline && millis() - lastLdMs > LD14_OFFLINE_MS) ldOnline = false;
}
#else
uint32_t ldBytes=0, ldHeader54=0, ldHeader542C=0, ldGood=0, ldBad=0;
bool ldOnline=false;
void setupLD14() { Serial.begin(115200); }
void readLD14() {}
#endif

// ================= Battery ADC =================
#if USE_BATTERY_ADC
const int BAT_ADC_PIN = 34;
const float BAT_ADC_R_TOP = 38700.0f;
const float BAT_ADC_R_BOTTOM = 9660.0f;
uint16_t adcRaw = 0;
uint32_t adcMilliVolts = 0;
float batteryVolts = 0.0f;
void setupBatteryADC() {
  analogReadResolution(12);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
}
void readBatteryADC() {
  adcRaw = analogRead(BAT_ADC_PIN);
  adcMilliVolts = analogReadMilliVolts(BAT_ADC_PIN);
  batteryVolts = (adcMilliVolts / 1000.0f) *
                 (BAT_ADC_R_TOP + BAT_ADC_R_BOTTOM) / BAT_ADC_R_BOTTOM;
}
#else
void setupBatteryADC() {}
void readBatteryADC() {}
#endif

// ================= STOP / failsafe =================
const uint8_t STOP_FRAME[] = {
  0xCD,0x0A,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01
};
const uint32_t COMMAND_TIMEOUT_MS = 500;
const uint32_t STOP_REPEAT_MS = 100;
uint32_t lastCommandMs = 0;
uint32_t lastStopMs = 0;

// ================= Periods and counters =================
const uint32_t IMU_TELEM_PERIOD_MS = 50;
const uint32_t ADC_TELEM_PERIOD_MS = 100;
const uint32_t STATUS_PERIOD_MS = 1000;
const uint32_t LD_DEBUG_PERIOD_MS = 1000;
uint32_t lastImuTelemMs=0, lastAdcTelemMs=0, lastStatusMs=0, lastLdDebugMs=0;
uint32_t udpPacketCount=0, uartPacketCount=0, lastUdpPacketLen=0;

void sendToPC(const uint8_t* data, size_t len) {
  if (!hasPC || len == 0) return;
  udp.beginPacket(pcIP, UDP_TELEM_PORT);
  udp.write(data, len);
  udp.endPacket();
}
void sendTextToPC(const char* text) {
  sendToPC((const uint8_t*)text, strlen(text));
}
void sendStopFrame() {
  STM32Serial.write(STOP_FRAME, sizeof(STOP_FRAME));
}

void sendImuTelemetry() {
  if (!hasPC) return;
  char msg[256];
  snprintf(msg, sizeof(msg),
    "IMU,ms=%lu,bno=%d,yaw=%.2f,pitch=%.2f,roll=%.2f,gx=0.00000,gy=0.00000,gz=0.00000,ax=%.5f,ay=%.5f,az=%.5f\n",
    (unsigned long)millis(), bnoOK?1:0,
    imuYawDeg, imuPitchDeg, imuRollDeg,
    linAccX, linAccY, linAccZ);
  sendTextToPC(msg);
}

void sendAdcTelemetry() {
#if USE_BATTERY_ADC
  if (!hasPC) return;
  char msg[160];
  snprintf(msg, sizeof(msg),
    "ADC,ms=%lu,pin=%d,raw=%u,mv=%lu,vbat=%.3f\n",
    (unsigned long)millis(), BAT_ADC_PIN, adcRaw,
    (unsigned long)adcMilliVolts, batteryVolts);
  sendTextToPC(msg);
#endif
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (USE_STATIC_IP) WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS1);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.printf("\nWiFi connected. ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
}

void handleUdpToStm32() {
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;
    int len = udp.read(udpRxBuf, sizeof(udpRxBuf));
    if (len <= 0) continue;
    pcIP = udp.remoteIP();
    pcSourcePort = udp.remotePort();
    hasPC = true;
    if (len >= 4 && memcmp(udpRxBuf, "PING", 4) == 0) {
      char msg[128];
      snprintf(msg, sizeof(msg), "PONG,ms=%lu,ip=%s\n",
        (unsigned long)millis(), WiFi.localIP().toString().c_str());
      sendTextToPC(msg);
      continue;
    }
    if (len >= 4 && memcmp(udpRxBuf, "STOP", 4) == 0) sendStopFrame();
    else STM32Serial.write(udpRxBuf, len);
    lastCommandMs = millis();
    udpPacketCount++;
    lastUdpPacketLen = len;
  }
}

void handleStm32ToUdp() {
#if FORWARD_STM32_TO_PC
  if (!hasPC) return;
  int availableBytes = STM32Serial.available();
  if (availableBytes <= 0) return;
  int n = STM32Serial.readBytes(uartRxBuf, min(availableBytes, (int)sizeof(uartRxBuf)));
  if (n > 0) {
    udp.beginPacket(pcIP, UDP_TELEM_PORT);
    udp.write((const uint8_t*)"STM:", 4);
    udp.write(uartRxBuf, n);
    udp.endPacket();
    uartPacketCount++;
  }
#endif
}

void sendStatus() {
  if (!hasPC) return;
  char msg[360];
  snprintf(msg, sizeof(msg),
    "ESPSTAT,ms=%lu,ip=%s,wifi_rssi=%d,udp_rx=%lu,uart_tx=%lu,last_len=%lu,bno=%d,rvc_ok=%lu,rvc_bad=%lu,lidar=%d,ld_bytes=%lu,ld_h54=%lu,ld_h542c=%lu,ld_ok=%lu,ld_bad=%lu\n",
    (unsigned long)millis(), WiFi.localIP().toString().c_str(), WiFi.RSSI(),
    (unsigned long)udpPacketCount, (unsigned long)uartPacketCount,
    (unsigned long)lastUdpPacketLen, bnoOK?1:0,
    (unsigned long)rvcGoodCount, (unsigned long)rvcBadCount,
    ldOnline?1:0, (unsigned long)ldBytes,
    (unsigned long)ldHeader54, (unsigned long)ldHeader542C,
    (unsigned long)ldGood, (unsigned long)ldBad);
  sendTextToPC(msg);
}

void printLD14Diagnostic() {
#if USE_LD14
  Serial.printf(
    "LD14 bytes=%lu h54=%lu h542c=%lu good=%lu bad=%lu online=%d avail=%d hasPC=%d pc=%s\n",
    (unsigned long)ldBytes,
    (unsigned long)ldHeader54,
    (unsigned long)ldHeader542C,
    (unsigned long)ldGood,
    (unsigned long)ldBad,
    ldOnline?1:0,
    Serial.available(),
    hasPC?1:0,
    hasPC ? pcIP.toString().c_str() : "--");
#endif
}

void setup() {
  // First map UART0 RX to LD14 GPIO32. TX remains GPIO1 for USB diagnostics.
  setupLD14();
  delay(300);

  STM32Serial.setRxBufferSize(512);
  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  setupBNO085Rvc();
  setupBatteryADC();
  connectWiFi();

  if (!udp.begin(UDP_CMD_PORT)) Serial.println("UDP begin failed.");
#if USE_FAILSAFE_STOP
  sendStopFrame();
#endif
  lastCommandMs = millis();
  Serial.println("M100 + BNO085 + LD14 hardware UART0 diagnostic started.");
}

void loop() {
  handleUdpToStm32();
  handleStm32ToUdp();
  readBNO085Rvc();
  readLD14();
  readBatteryADC();

  uint32_t now = millis();
#if USE_FAILSAFE_STOP
  if (now-lastCommandMs > COMMAND_TIMEOUT_MS && now-lastStopMs > STOP_REPEAT_MS) {
    lastStopMs = now;
    sendStopFrame();
  }
#endif
  if (now-lastImuTelemMs >= IMU_TELEM_PERIOD_MS) {
    lastImuTelemMs = now;
    sendImuTelemetry();
  }
  if (now-lastAdcTelemMs >= ADC_TELEM_PERIOD_MS) {
    lastAdcTelemMs = now;
    sendAdcTelemetry();
  }
  if (now-lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }
  if (now-lastLdDebugMs >= LD_DEBUG_PERIOD_MS) {
    lastLdDebugMs = now;
    printLD14Diagnostic();
  }
}
