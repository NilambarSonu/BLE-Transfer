/*
 * AGNI SOIL SENSOR - COMPLETE INTEGRATED SYSTEM
 *
 * This version implements the requested UI workflow & behaviors:
 * - On every reset, wipe /farmland_data and start fresh (no default file).
 * - OLED boot script with timed screens, 5s bottom countdown, 45s analysis.
 * - After save, "FILE Creation successful" for 3s and bottom-right file count.
 * - Same JSON schema & filenames as before.
 * - GPS defaults used when no fix is available at file save time.
 * - Pin mapping unchanged.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// ============================================================================
// OLED CONFIGURATION (unchanged pins)
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA 8
#define OLED_SCL 9
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// SD CARD CONFIGURATION (unchanged pins)
// ============================================================================
#define SD_CS   10
#define SD_MOSI 11
#define SD_SCK  12
#define SD_MISO 13
bool sdOK = false;
int fileCounter = 1;

// ============================================================================
// RS485 SOIL SENSOR CONFIGURATION (unchanged pins)
// ============================================================================
#define RS485_RX  16
#define RS485_TX  17
#define RS485_DE  18
#define RS485_RE  19
#define MODBUS_BAUD      4800
#define MODBUS_ADDRESS   1
#define MODBUS_TIMEOUT   800

#define REG_MOISTURE       0x0000
#define REG_TEMPERATURE    0x0001
#define REG_CONDUCTIVITY   0x0002
#define REG_PH             0x0003
#define REG_NITROGEN       0x0006
#define REG_PHOSPHORUS     0x0007
#define REG_POTASSIUM      0x0008

// ============================================================================
// GPS CONFIGURATION (unchanged pins)
// ============================================================================
#define GPS_SERIAL Serial2
#define GPS_RX_PIN 20
#define GPS_TX_PIN 21
TinyGPSPlus gps;
unsigned long gpsCharsProcessed = 0;

// ============================================================================
// BUZZER CONFIGURATION (unchanged pins)
// ============================================================================
#define BUZZER_PIN 7
int buzzerVolume = 180;

// ============================================================================
// BLE CONFIGURATION (unchanged)
// ============================================================================
BLEServer* pServer = NULL;
BLECharacteristic* pFileTransferCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool transferInProgress = false;

#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID_TRANSFER "abcdef12-3456-7890-1234-567890abcdef"
#define CHARACTERISTIC_UUID_COMMAND "abcdef13-3456-7890-1234-567890abcdef"

// ============================================================================
// DATA
// ============================================================================
struct SensorData {
  float moisture = 0;
  float temperature = 0;
  uint16_t conductivity = 0;
  float ph = 0;
  uint16_t nitrogen = 0;
  uint16_t phosphorus = 0;
  uint16_t potassium = 0;
  bool basicValid = false;
  bool npkValid = false;
};

struct SystemStatus {
  bool oledOK = false;
  bool sdOK = false;
  bool soilSensorOK = false;
  bool gpsOK = false;
  bool gpsFix = false;
  bool bleOK = false;
  int satellites = 0;
  float latitude = 0;
  float longitude = 0;
  float altitude = 0;
  int year = 0, month = 0, day = 0;
  int hour = 0, minute = 0, second = 0;
};

SensorData soilData;
SystemStatus systemStatus;

// ============================================================================
// USER-FLOW TIMINGS — tweak here later (Step 6)
// ============================================================================
const uint32_t BOOT_BANNER_MS   = 3000;  // "AGNI SOIL SENSOR — initializing..."
const uint32_t SELF_CHECK_MS    = 3000;  // components OK/INVALID
const uint32_t GPS_SEARCH_MS    = 3000;  // brief "Searching..." page
const uint32_t PROMPT_MS        = 3000;  // "PLEASE INSERT THE SOIL SENSOR"
const uint32_t COUNTDOWN5_MS    = 5000;  // 5→0 (1s steps)
const uint32_t ANALYZE_MS       = 45000; // 45→0 (1s steps)
const uint32_t SUCCESS_MS       = 3000;  // "FILE Creation successful"

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void startDynamicFileTransfer();
void formatSDCard();
String generateJSONData();
void logDataToSD();
void drawCentered(const String& s, int16_t y);
void drawBottomRight(const String& s);
void beep(int duration = 100);

// ============================================================================
// BUZZER
// ============================================================================
void beep(int duration) {
  analogWrite(BUZZER_PIN, buzzerVolume);
  delay(duration);
  analogWrite(BUZZER_PIN, 0);
}

// ============================================================================
// OLED
// ============================================================================
void initOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("❌ OLED allocation failed");
    systemStatus.oledOK = false;
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  systemStatus.oledOK = true;
}

void drawCentered(const String& s, int16_t y) {
  if(!systemStatus.oledOK) return;
  int16_t x1, y1; uint16_t w, h;
  display.setTextSize(1);
  display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  int16_t x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.println(s);
}

void drawBottomRight(const String& s) {
  if(!systemStatus.oledOK) return;
  int16_t x1, y1; uint16_t w, h;
  display.setTextSize(1);
  display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  int16_t x = SCREEN_WIDTH - w - 1;
  int16_t y = SCREEN_HEIGHT - h - 1;
  display.setCursor(x, y);
  display.println(s);
}

void showBootBanner(uint8_t dots = 0) {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  drawCentered("AGNI SOIL SENSOR", 8);
  String line = "initializing";
  for(uint8_t i=0;i<dots;i++) line += ".";
  drawCentered(line, 26);
  display.display();
}

void showSelfCheck() {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  drawCentered("SYSTEM CHECK", 0);
  display.setCursor(2, 18);
  display.setTextSize(1);
  display.printf("OLED: %s\n", systemStatus.oledOK ? "OK" : "INVALID");
  display.printf("SD  : %s\n", systemStatus.sdOK ? "OK" : "INVALID");
  display.printf("RS485: %s\n", systemStatus.soilSensorOK ? "OK" : "INVALID");
  display.printf("GPS : %s\n", systemStatus.gpsOK ? "OK" : "INVALID");
  display.display();
}

void showGpsSearching() {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  drawCentered("GPS", 0);
  drawCentered("Searching....", 22);
  drawBottomRight(String("Files: ") + String(fileCounter - 1));
  display.display();
}

void showInsertSensor() {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  drawCentered("PLEASE INSERT", 8);
  drawCentered("THE SOIL SENSOR", 24);
  drawBottomRight(String("Files: ") + String(fileCounter - 1));
  display.display();
}

void showCountdown5(uint8_t left) {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  drawCentered("Get Ready", 6);
  drawCentered("Starting in", 20);
  drawCentered(String(left) + "s", 36);
  // bottom line countdown (step 11)
  display.setCursor(0, 56);
  display.print("Countdown: ");
  display.print(left);
  display.print("s");
  drawBottomRight(String("Files: ") + String(fileCounter - 1));
  display.display();
}

void showAnalyzing(uint8_t left) {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  drawCentered("Analyzing Your Soil.........", 10);
  drawCentered(String(left) + "s", 30);
  // bottom corner file count (step 15)
  drawBottomRight(String("Files: ") + String(fileCounter - 1));
  display.display();
}

void showFileSuccess() {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  drawCentered("FILE Creation", 10);
  drawCentered("successful", 26);
  drawBottomRight(String("Files: ") + String(fileCounter - 1));
  display.display();
}

// ============================================================================
// SD
// ============================================================================
void initSDCard() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 1000000)) {
    Serial.println("❌ SD Card initialization failed!");
    systemStatus.sdOK = false;
    return;
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("✅ SD Card initialized: %llu MB\n", cardSize);

  if(!SD.exists("/farmland_data")) SD.mkdir("/farmland_data");
  systemStatus.sdOK = true;
}

void safeRemoveAllInData() {
  if(!systemStatus.sdOK) return;
  File root = SD.open("/farmland_data");
  if (root) {
    File file = root.openNextFile();
    while(file) {
      if(!file.isDirectory()) SD.remove(String("/farmland_data/") + file.name());
      file = root.openNextFile();
    }
    root.close();
  }
}

String generateJSONData() {
  JsonDocument doc;

  // Use current fileCounter as id
  doc["id"] = fileCounter;

  // Timestamp from GPS (if available) else zeros
  char timestamp[30];
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
    systemStatus.year, systemStatus.month, systemStatus.day,
    systemStatus.hour, systemStatus.minute, systemStatus.second);
  doc["timestamp"] = timestamp;

  char time_utc[10];
  sprintf(time_utc, "%02d:%02d:%02d", systemStatus.hour, systemStatus.minute, systemStatus.second);
  doc["time_utc"] = time_utc;

  // IST conversion
  int ist_hour = (systemStatus.hour + 5) % 24;
  int ist_minute = systemStatus.minute + 30;
  if(ist_minute >= 60) { ist_minute -= 60; ist_hour = (ist_hour + 1) % 24; }
  char time_ist[20];
  sprintf(time_ist, "%02d:%02d %s", ist_hour, ist_minute, ist_hour >= 12 ? "PM" : "AM");
  doc["time_ist"] = time_ist;

  // Location (defaults when no fix — Step 20)
  JsonObject location = doc["location"].to<JsonObject>();
  location["latitude"]   = systemStatus.gpsFix ? systemStatus.latitude  : 0.0;
  location["longitude"]  = systemStatus.gpsFix ? systemStatus.longitude : 0.0;
  location["valid"]      = systemStatus.gpsFix;
  location["satellites"] = systemStatus.gpsFix ? systemStatus.satellites : 0;
  location["altitude"]   = systemStatus.gpsFix ? systemStatus.altitude   : 0.0;
  location["speed_kmh"]  = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  location["hdop"]       = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;

  // pH bucket
  if(soilData.ph < 5.5) doc["ph_category"] = "acidic";
  else if(soilData.ph < 6.5) doc["ph_category"] = "slightly_acidic";
  else if(soilData.ph < 7.5) doc["ph_category"] = "neutral";
  else if(soilData.ph < 8.5) doc["ph_category"] = "slightly_alkaline";
  else doc["ph_category"] = "alkaline";

  // Parameters
  JsonObject params = doc["parameters"].to<JsonObject>();
  params["ph_value"] = soilData.ph;
  params["conductivity"] = soilData.conductivity;
  params["nitrogen"] = soilData.nitrogen;
  params["phosphorus"] = soilData.phosphorus;
  params["potassium"] = soilData.potassium;
  params["moisture"] = soilData.moisture;
  params["temperature"] = soilData.temperature;

  doc["sensor_valid"] = soilData.basicValid && soilData.npkValid;

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

void logDataToSD() {
  if(!systemStatus.sdOK) return;

  String filename = "/farmland_data/farmland_" + String(fileCounter) + ".json";
  File file = SD.open(filename, FILE_WRITE);
  if(!file) {
    Serial.println("❌ Failed to create JSON file: " + filename);
    return;
  }

  String jsonData = generateJSONData();
  file.print(jsonData);
  file.close();

  fileCounter++;
  Serial.println("✅ JSON data logged to SD card: " + filename);
}

// ============================================================================
// MODBUS / Soil Sensor
// ============================================================================
uint16_t crc16_modbus(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

bool modbusRead(uint8_t addr, uint16_t startReg, uint16_t regCount, uint16_t *result) {
  uint8_t txBuf[8];
  uint8_t rxBuf[256];

  int pos = 0;
  txBuf[pos++] = addr;
  txBuf[pos++] = 0x03;
  txBuf[pos++] = (startReg >> 8);
  txBuf[pos++] = (startReg & 0xFF);
  txBuf[pos++] = (regCount >> 8);
  txBuf[pos++] = (regCount & 0xFF);

  uint16_t crc = crc16_modbus(txBuf, pos);
  txBuf[pos++] = (crc & 0xFF);
  txBuf[pos++] = (crc >> 8);

  while(Serial1.available()) Serial1.read();

  digitalWrite(RS485_DE, HIGH);
  digitalWrite(RS485_RE, HIGH);
  delay(2);
  Serial1.write(txBuf, pos);
  Serial1.flush();
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
  delay(2);

  unsigned long startTime = millis();
  int rxLen = 0;

  while(millis() - startTime < MODBUS_TIMEOUT && rxLen < (int)sizeof(rxBuf)) {
    if(Serial1.available()) {
      rxBuf[rxLen++] = Serial1.read();
      if(rxLen >= 5) {
        uint8_t byteCount = rxBuf[2];
        int expectedLen = 3 + byteCount + 2;
        if(rxLen >= expectedLen) break;
      }
    }
  }
  if(rxLen == 0) return false;

  uint16_t receivedCrc = (rxBuf[rxLen-1] << 8) | rxBuf[rxLen-2];
  uint16_t calculatedCrc = crc16_modbus(rxBuf, rxLen - 2);
  if(receivedCrc != calculatedCrc) return false;

  for(int i = 0; i < regCount; i++) {
    result[i] = (rxBuf[3 + i*2] << 8) | rxBuf[4 + i*2];
  }
  return true;
}

bool readSoilSensor() {
  uint16_t regs[4];
  if(!modbusRead(MODBUS_ADDRESS, REG_MOISTURE, 4, regs)) {
    soilData.basicValid = false;
    return false;
  }

  soilData.moisture = regs[0] / 10.0f;
  int16_t tempRaw = (int16_t)regs[1];
  soilData.temperature = tempRaw / 10.0f;
  soilData.conductivity = regs[2];
  soilData.ph = regs[3] / 10.0f;
  soilData.basicValid = true;

  uint16_t npkRegs[3];
  if(modbusRead(MODBUS_ADDRESS, REG_NITROGEN, 3, npkRegs)) {
    soilData.nitrogen = npkRegs[0];
    soilData.phosphorus = npkRegs[1];
    soilData.potassium = npkRegs[2];
    soilData.npkValid = true;
  } else {
    soilData.npkValid = false;
  }

  systemStatus.soilSensorOK = soilData.basicValid;
  return soilData.basicValid;
}

// ============================================================================
// GPS
// ============================================================================
void updateGPS() {
  while(GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    gps.encode(c);
    gpsCharsProcessed++;
  }
  systemStatus.gpsOK = (gpsCharsProcessed > 10);

  if(gps.location.isValid()) {
    systemStatus.gpsFix = true;
    systemStatus.latitude = gps.location.lat();
    systemStatus.longitude = gps.location.lng();
    systemStatus.satellites = gps.satellites.value();
    if(gps.altitude.isValid()) systemStatus.altitude = gps.altitude.meters();
    if(gps.date.isValid()) { systemStatus.year = gps.date.year(); systemStatus.month = gps.date.month(); systemStatus.day = gps.date.day(); }
    if(gps.time.isValid()) { systemStatus.hour = gps.time.hour(); systemStatus.minute = gps.time.minute(); systemStatus.second = gps.time.second(); }
  } else {
    systemStatus.gpsFix = false;
  }
}

// ============================================================================
// BLE CALLBACKS / TRANSFER (unchanged behavior)
// ============================================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("\n🔵 BLE Client connected!");
    beep(200);
    BLEDevice::stopAdvertising();
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    transferInProgress = false;
    Serial.println("🔴 BLE Client disconnected");
    delay(500);
    BLEDevice::startAdvertising();
    Serial.println("📡 BLE Advertising restarted\n");
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      String command = String(value.c_str());
      Serial.println("📬 BLE Command received: " + command);
      if (command == "START_TRANSFER") {
        startDynamicFileTransfer();
      } else if (command == "FORMAT_SD") {
        formatSDCard();
        pCommandCharacteristic->setValue("SD_FORMATTED");
        pCommandCharacteristic->notify();
      }
    }
  }
};

void sendFile(String filePath) {
  if (!deviceConnected || !transferInProgress) {
    Serial.println("❌ Transfer interrupted");
    return;
  }
  File file = SD.open(filePath, FILE_READ);
  if (!file) {
    Serial.println("❌ Cannot open file: " + filePath);
    return;
  }
  size_t fileSize = file.size();
  size_t bytesSent = 0;
  String fileName = file.name();
  Serial.println("📤 Transferring: " + fileName + " (" + String(fileSize) + " bytes)");
  String fileHeader = "FILE_START:" + fileName + "|SIZE:" + String(fileSize);
  pFileTransferCharacteristic->setValue(fileHeader.c_str());
  pFileTransferCharacteristic->notify();
  delay(120);

  const size_t CHUNK_SIZE = 128;
  uint8_t buffer[CHUNK_SIZE];
  while (file.available() && deviceConnected && transferInProgress) {
    size_t bytesRead = file.read(buffer, CHUNK_SIZE);
    if (bytesRead > 0) {
      pFileTransferCharacteristic->setValue(buffer, bytesRead);
      pFileTransferCharacteristic->notify();
      bytesSent += bytesRead;
      int progress = (int)((bytesSent * 100) / fileSize);
      if (progress % 10 == 0) Serial.println(String(fileName) + " " + String(progress) + "%");
    }
    delay(5);
  }
  if (deviceConnected && transferInProgress) {
    String fileEnd = "FILE_END:" + fileName;
    pFileTransferCharacteristic->setValue(fileEnd.c_str());
    pFileTransferCharacteristic->notify();
    Serial.println("✅ Transferred: " + fileName);
  }
  file.close();
}

void startDynamicFileTransfer() {
  if (!systemStatus.sdOK || !deviceConnected) return;
  if (transferInProgress) {
    Serial.println("⚠️  Transfer already in progress");
    return;
  }
  transferInProgress = true;
  Serial.println("\n🚀 STARTING BLE FILE TRANSFER...");
  beep(150);

  int totalFiles = 0;
  File root = SD.open("/farmland_data");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) totalFiles++;
      file = root.openNextFile();
    }
    root.close();
  }
  Serial.printf("📊 Total files to send: %d\n", totalFiles);

  root = SD.open("/farmland_data");
  if (root) {
    File file = root.openNextFile();
    while (file && deviceConnected && transferInProgress) {
      if (!file.isDirectory()) sendFile("/farmland_data/" + String(file.name()));
      file = root.openNextFile();
      delay(200);
    }
    root.close();
  }
  if (deviceConnected && transferInProgress) {
    String completeMsg = "TRANSFER_COMPLETE|All " + String(totalFiles) + " files transferred!";
    pFileTransferCharacteristic->setValue(completeMsg.c_str());
    pFileTransferCharacteristic->notify();
    Serial.println("🎉 ALL FILES TRANSFERRED SUCCESSFULLY!");
    beep(300);
  }
  transferInProgress = false;
}

void autoStartTransfer() {
  static bool transferStarted = false;
  static unsigned long connectionTime = 0;
  if (deviceConnected && !transferStarted && !transferInProgress) {
    if (connectionTime == 0) {
      connectionTime = millis();
      Serial.println("⏱️  Auto-transfer will start in 5 seconds...");
    }
    if (millis() - connectionTime >= 5000) {
      transferStarted = true;
      startDynamicFileTransfer();
    }
  }
  if (!deviceConnected) { transferStarted = false; connectionTime = 0; }
}

void formatSDCard() {
  if(!sdOK) return;
  Serial.println("🔄 Formatting SD card...");
  safeRemoveAllInData();
  Serial.println("✅ SD Card formatted successfully!");
  fileCounter = 1;
  beep(300);
}

void initializeBLE() {
  Serial.println("📡 Initializing BLE...");
  BLEDevice::init("AGNI-SOIL-SENSOR");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pFileTransferCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TRANSFER, BLECharacteristic::PROPERTY_NOTIFY);
  pFileTransferCharacteristic->addDescriptor(new BLE2902());
  pCommandCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_COMMAND, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCommandCharacteristic->addDescriptor(new BLE2902());
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  systemStatus.bleOK = true;
  Serial.println("✅ BLE initialized successfully!");
  Serial.println("📡 Advertising as: AGNI-SOIL-SENSOR\n");
}

// ============================================================================
// SYSTEM STATUS (serial pretty print)
// ============================================================================
void printSystemStatus() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║               🌱 AGNI SOIL SENSOR - SYSTEM STATUS              ║");
  Serial.println("╠═══════════════════════════════════════════════════════════════╣");
  Serial.printf("║ 📊 OLED: %s  SD: %s  Soil: %s  GPS: %s              ║\n",
    systemStatus.oledOK ? "✅" : "❌",
    systemStatus.sdOK ? "✅" : "❌",
    systemStatus.soilSensorOK ? "✅" : "❌",
    systemStatus.gpsOK ? "✅" : "❌");
  Serial.printf("║ 🔵 BLE: %s  🛰️  Fix: %s  📡 Satellites: %2d              ║\n",
    deviceConnected ? "🔗 Connected" : "📡 Advertising",
    systemStatus.gpsFix ? "✅" : "❌", systemStatus.satellites);
  if(soilData.basicValid) {
    Serial.printf("║ 🌍 Soil - Moisture: %.1f%%  Temp: %.1f°C  pH: %.1f  EC: %duS/cm ║\n",
      soilData.moisture, soilData.temperature, soilData.ph, soilData.conductivity);
    if(soilData.npkValid) {
      Serial.printf("║ 🧪 NPK - N:%d  P:%d  K:%d mg/kg                              ║\n",
        soilData.nitrogen, soilData.phosphorus, soilData.potassium);
    }
  }
  if(systemStatus.gpsFix) {
    Serial.printf("║ 📍 Location - Lat: %.6f  Lon: %.6f  Alt: %.1fm            ║\n",
      systemStatus.latitude, systemStatus.longitude, systemStatus.altitude);
    char timestamp[25];
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d UTC",
      systemStatus.year, systemStatus.month, systemStatus.day,
      systemStatus.hour, systemStatus.minute, systemStatus.second);
    Serial.printf("║ ⏰ Timestamp: %s                    ║\n", timestamp);
  }
  Serial.printf("║ 💾 Files Logged: %d                                           ║\n", fileCounter - 1);
  Serial.println("╚═══════════════════════════════════════════════════════════════╝\n");
}

// ============================================================================
// UI STATE MACHINE (implements steps 7–16/18)
// ============================================================================
enum UiState {
  BOOT_BANNER,
  SELF_CHECK,
  GPS_SEARCH,
  INSERT_SENSOR,
  COUNTDOWN_5S,
  ANALYZING_45S,
  SAVE_SUCCESS
};

UiState uiState = BOOT_BANNER;
unsigned long stateStart = 0;

void gotoState(UiState s) {
  uiState = s;
  stateStart = millis();
}

void runUiFlow() {
  // Minimal animated dots for boot banner
  static uint8_t dots = 0;

  switch(uiState) {
    case BOOT_BANNER:
      showBootBanner(dots++ % 4);
      if(millis() - stateStart >= BOOT_BANNER_MS) gotoState(SELF_CHECK);
      break;

    case SELF_CHECK:
      showSelfCheck();
      if(millis() - stateStart >= SELF_CHECK_MS) gotoState(GPS_SEARCH);
      break;

    case GPS_SEARCH:
      showGpsSearching();
      if(millis() - stateStart >= GPS_SEARCH_MS) gotoState(INSERT_SENSOR);
      break;

    case INSERT_SENSOR:
      showInsertSensor();
      if(millis() - stateStart >= PROMPT_MS) gotoState(COUNTDOWN_5S);
      break;

    case COUNTDOWN_5S: {
      uint32_t elapsed = millis() - stateStart;
      uint8_t left = 5 - (elapsed / 1000);
      if(left > 5) left = 5;
      if(left > 0) showCountdown5(left);
      else { showCountdown5(0); gotoState(ANALYZING_45S); }
    } break;

    case ANALYZING_45S: {
      uint32_t elapsed = millis() - stateStart;
      uint8_t left = 45 - (elapsed / 1000);
      if(left > 45) left = 45;
      showAnalyzing(left);

      // While analyzing window runs, do real sensor reads periodically
      static unsigned long lastRead = 0;
      if(millis() - lastRead >= 2000) {
        readSoilSensor();         // refresh data during countdown
        lastRead = millis();
      }

      if(elapsed >= ANALYZE_MS) {
        // Final read before save
        readSoilSensor();
        logDataToSD();
        gotoState(SAVE_SUCCESS);
      }
    } break;

    case SAVE_SUCCESS:
      showFileSuccess();
      if(millis() - stateStart >= SUCCESS_MS) {
        // loop back to step 10 (INSERT SENSOR)
        gotoState(INSERT_SENSOR);
      }
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║          🌱 AGNI SOIL SENSOR - COMPLETE INTEGRATED SYSTEM      ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════╝\n");

  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 0);
  beep(120);

  // OLED
  initOLED();

  // SD Card
  initSDCard();

  // **Step 4**: always start fresh on reset
  if(systemStatus.sdOK) {
    safeRemoveAllInData();   // delete existing files in /farmland_data
    fileCounter = 1;
  }

  // RS485 Soil Sensor
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("✅ RS485 Modbus initialized");

  // GPS
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("✅ GPS module initialized");

  // BLE
  initializeBLE();

  beep(120); delay(120); beep(120);

  Serial.println("✅ All systems initialized successfully!");
  Serial.println("🚀 System ready\n");

  // Boot state
  stateStart = millis();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  static unsigned long lastStatusPrint = 0;

  // Keep GPS fresh
  updateGPS();

  // Run the requested UI flow
  runUiFlow();

  // Nice status to Serial every 10s
  if(millis() - lastStatusPrint >= 10000) {
    printSystemStatus();
    lastStatusPrint = millis();
  }

  // Auto-transfer when a client connects
  autoStartTransfer();

  delay(10);
}
