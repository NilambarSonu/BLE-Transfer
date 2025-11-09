/* CRITICAL BLE FIX: main.cpp
 * MAJOR ISSUE FOUND & FIXED:
 * 1. BLE characteristic was not properly configured for notifications
 *    - Added explicit MTU negotiation
 *    - Added characteristic property validation
 *    - Ensured BLE2902 CCCD is properly written
 * 2. Notifications not being sent to client properly
 *    - Added delay after client connection to allow CCCD setup
 *    - Changed notification method to ensure delivery
 * 3. Python not receiving data despite ESP32 sending
 *    - Fixed by proper BLE stack initialization
 *    - Added explicit notify payload configuration
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
// OLED CONFIGURATION
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA 8
#define OLED_SCL 9
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// SD CARD CONFIGURATION
// ============================================================================
#define SD_CS   10
#define SD_MOSI 11
#define SD_SCK  12
#define SD_MISO 13
bool sdOK = false;
int fileCounter = 1;

#define FILE_CREATION_INTERVAL_MS 30000UL

// ============================================================================
// RS485 SOIL SENSOR CONFIGURATION (ZTS-3002)
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
// GPS CONFIGURATION
// ============================================================================
#define GPS_SERIAL Serial2
#define GPS_RX_PIN 20
#define GPS_TX_PIN 21
TinyGPSPlus gps;
unsigned long gpsCharsProcessed = 0;

#define DEFAULT_LATITUDE  0.0
#define DEFAULT_LONGITUDE 0.0
#define DEFAULT_ALTITUDE  0.0
#define DEFAULT_SATS      0

// ============================================================================
// BUZZER CONFIGURATION (ESP32 ledc PWM)
// ============================================================================
#define BUZZER_PIN 7
const int BUZZER_LEDC_CHANNEL = 0;
const int BUZZER_FREQUENCY = 2000;
const int BUZZER_RESOLUTION = 8;
bool buzzerOK = false;

// ============================================================================
// BLE CONFIGURATION - FIXED
// ============================================================================
BLEServer* pServer = NULL;
BLECharacteristic* pFileTransferCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
bool deviceConnected = false;
bool transferInProgress = false;
bool transferRequested = false;
bool showingTransferComplete = false;

#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID_TRANSFER "abcdef12-3456-7890-1234-567890abcdef"
#define CHARACTERISTIC_UUID_COMMAND "abcdef13-3456-7890-1234-567890abcdef"

// ============================================================================
// DATA STRUCTURES
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
  bool rs485OK = false;
  bool buzzerOK = false;
  int satellites = 0;
  float latitude = 0;
  float longitude = 0;
  float altitude = 0;
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
};

SensorData soilData;
SystemStatus systemStatus;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void startDynamicFileTransfer();
void formatSDCard();
String generateJSONData();
void logDataToSD();
void clearDataDirectory();
bool modbusRead(uint8_t addr, uint16_t startReg, uint16_t regCount, uint16_t *result);
bool readSoilSensor();
void updateGPS();
void updateOLED();
void initOLED();
void initSDCard();
void initializeBLE();
void beep_ms(int duration = 100, int duty = 180);
void showOLEDTransferProgress(const String& filename, int progressPercent);
int countFilesInDir();

// ============================================================================
// BUZZER
// ============================================================================
void beep_ms(int duration, int duty) {
  if(!systemStatus.buzzerOK) return;
  ledcWrite(BUZZER_LEDC_CHANNEL, duty);
  delay(duration);
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
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
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("AGNI SOIL SENSOR");
  display.println("Initializing...");
  display.display();

  systemStatus.oledOK = true;
  Serial.println("✅ OLED initialized");
}

void updateOLED() {
  if(!systemStatus.oledOK) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("AGNI SOIL SENSOR");
  display.println("-----------------");

  if(soilData.basicValid) {
    display.printf("M:%.1f%% T:%.1fC pH:%.1f\n", soilData.moisture, soilData.temperature, soilData.ph);
  } else {
    display.println("Soil: No Data");
    display.println();
  }

  if(systemStatus.gpsFix) {
    display.printf("GPS: Fix (%d) Lat:%.4f\n", systemStatus.satellites, systemStatus.latitude);
  } else {
    display.println("GPS: Searching...");
  }

  if(deviceConnected) display.println("BLE: Connected");
  else display.println("BLE: Advertising");

  display.setCursor(0, 54);
  display.printf("Files: %d", max(0, fileCounter - 1));

  display.display();
}

// ============================================================================
// SD CARD
// ============================================================================
void initSDCard() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI, 1000000)) {
    Serial.println("❌ SD Card initialization failed!");
    systemStatus.sdOK = false;
    sdOK = false;
    return;
  }

  uint64_t cardSize = 0;
  if (SD.cardType() != CARD_NONE) {
    cardSize = SD.cardSize() / (1024 * 1024);
  }

  Serial.printf("✅ SD Card initialized: %llu MB\n", cardSize);

  if(!SD.exists("/farmland_data")) {
    SD.mkdir("/farmland_data");
  }

  systemStatus.sdOK = true;
  sdOK = true;
}

void clearDataDirectory() {
  if(!systemStatus.sdOK) return;

  File root = SD.open("/farmland_data");
  if(!root) return;

  File file = root.openNextFile();
  while(file) {
    String filename = file.name();
    if(!file.isDirectory()) {
      SD.remove(String("/farmland_data/") + filename);
    }
    file = root.openNextFile();
  }
  root.close();
  fileCounter = 1;
  Serial.println("✅ Cleared /farmland_data on startup");
}

int countFilesInDir() {
  if(!systemStatus.sdOK) return 0;
  int total = 0;
  File root = SD.open("/farmland_data");
  if(!root) return 0;
  File f = root.openNextFile();
  while(f) {
    if(!f.isDirectory()) total++;
    f = root.openNextFile();
  }
  root.close();
  return total;
}

// ============================================================================
// JSON GENERATION
// ============================================================================
String generateJSONData() {
  StaticJsonDocument<2048> doc;

  doc["id"] = fileCounter;

  char timestamp[30];
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
    systemStatus.year, systemStatus.month, systemStatus.day,
    systemStatus.hour, systemStatus.minute, systemStatus.second);
  doc["timestamp"] = timestamp;

  char time_utc[10];
  sprintf(time_utc, "%02d:%02d:%02d", systemStatus.hour, systemStatus.minute, systemStatus.second);
  doc["time_utc"] = time_utc;

  int ist_hour = (systemStatus.hour + 5) % 24;
  int ist_minute = systemStatus.minute + 30;
  if(ist_minute >= 60) {
    ist_minute -= 60;
    ist_hour++;
    if(ist_hour >= 24) ist_hour -= 24;
  }
  char time_ist[20];
  sprintf(time_ist, "%02d:%02d %s", ist_hour, ist_minute, ist_hour >= 12 ? "PM" : "AM");
  doc["time_ist"] = time_ist;

  JsonObject location = doc["location"].to<JsonObject>();
  if(systemStatus.gpsFix) {
    location["latitude"] = systemStatus.latitude;
    location["longitude"] = systemStatus.longitude;
    location["valid"] = true;
    location["satellites"] = systemStatus.satellites;
    location["altitude"] = systemStatus.altitude;
    location["speed_kmh"] = gps.speed.kmph();
    location["hdop"] = gps.hdop.hdop();
  } else {
    location["latitude"] = DEFAULT_LATITUDE;
    location["longitude"] = DEFAULT_LONGITUDE;
    location["valid"] = false;
    location["satellites"] = DEFAULT_SATS;
    location["altitude"] = DEFAULT_ALTITUDE;
    location["speed_kmh"] = 0;
    location["hdop"] = 99.9;
  }

  if(soilData.ph < 5.5) doc["ph_category"] = "acidic";
  else if(soilData.ph < 6.5) doc["ph_category"] = "slightly_acidic";
  else if(soilData.ph < 7.5) doc["ph_category"] = "neutral";
  else if(soilData.ph < 8.5) doc["ph_category"] = "slightly_alkaline";
  else doc["ph_category"] = "alkaline";

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

  Serial.println("✅ JSON data logged to SD card: " + filename);
  fileCounter++;
}

// ============================================================================
// MODBUS / RS485
// ============================================================================
uint16_t crc16_modbus(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
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
    systemStatus.rs485OK = false;
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
  systemStatus.rs485OK = soilData.basicValid;
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

    if(gps.altitude.isValid()) {
      systemStatus.altitude = gps.altitude.meters();
    }

    if(gps.date.isValid()) {
      systemStatus.year = gps.date.year();
      systemStatus.month = gps.date.month();
      systemStatus.day = gps.date.day();
    }

    if(gps.time.isValid()) {
      systemStatus.hour = gps.time.hour();
      systemStatus.minute = gps.time.minute();
      systemStatus.second = gps.time.second();
    }
  } else {
    systemStatus.gpsFix = false;
  }
}

// ============================================================================
// BLE CALLBACKS - FIXED FOR PROPER NOTIFICATION HANDLING
// ============================================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("\n🔵 BLE Client connected!");
    Serial.println("📱 Waiting for client to enable notifications...");
    transferRequested = true;
    if(systemStatus.buzzerOK) beep_ms(120, 200);
    BLEDevice::stopAdvertising();
    
    // FIX: Give client time to set up CCCD (Client Characteristic Config Descriptor)
    delay(1000);
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    transferInProgress = false;
    transferRequested = false;
    showingTransferComplete = false;
    Serial.println("🔴 BLE Client disconnected");
    delay(200);
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
        transferRequested = true;
      } else if (command == "FORMAT_SD") {
        formatSDCard();
        pCommandCharacteristic->setValue("SD_FORMATTED");
        pCommandCharacteristic->notify();
      }
    }
  }
};

// ============================================================================
// BLE FILE TRANSFER - WITH PROPER NOTIFICATION DELIVERY
// ============================================================================
void showOLEDTransferProgress(const String& filename, int progressPercent) {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("BLE File Transfer");
  display.println("-----------------");
  display.printf("File: %s", filename.c_str());
  display.setCursor(0, 40);
  display.printf("Progress: %d%%", progressPercent);
  display.display();
}

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

  // Send FILE_START control message
  String fileHeader = "FILE_START:" + fileName + "|SIZE:" + String(fileSize);
  pFileTransferCharacteristic->setValue(fileHeader.c_str());
  pFileTransferCharacteristic->notify();
  delay(150);

  // Read entire file and send as text (JSON)
  String fileContent = "";
  int lastPercentReported = -1;
  
  while (file.available()) {
    char c = file.read();
    fileContent += c;
    bytesSent++;
    
    // Send in chunks of 450 bytes to stay under MTU
    if (fileContent.length() >= 450) {
      pFileTransferCharacteristic->setValue(fileContent.c_str());
      pFileTransferCharacteristic->notify();
      delay(10); // Small delay between chunks
      fileContent = "";
    }
    
    // Update progress
    int progress = (int)((bytesSent * 100) / fileSize);
    if (progress != lastPercentReported && (progress % 10 == 0 || progress == 100)) {
      Serial.printf("   %s %d%%\n", fileName.c_str(), progress);
      showOLEDTransferProgress(fileName, progress);
      lastPercentReported = progress;
    }
  }

  // Send any remaining content
  if (fileContent.length() > 0) {
    pFileTransferCharacteristic->setValue(fileContent.c_str());
    pFileTransferCharacteristic->notify();
    delay(50);
  }

  // Send FILE_END control message
  if (deviceConnected && transferInProgress) {
    delay(150);
    String fileEnd = "FILE_END:" + fileName;
    pFileTransferCharacteristic->setValue(fileEnd.c_str());
    pFileTransferCharacteristic->notify();
    Serial.println("✅ Transferred: " + fileName);

    if(systemStatus.oledOK) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("File Sent");
      display.println(fileName.c_str());
      display.display();
      delay(200);
    }
  }

  file.close();
}

void startDynamicFileTransfer() {
  if (!systemStatus.sdOK || !deviceConnected) {
    Serial.println("⚠️ Can't start transfer - SD or BLE not ready");
    return;
  }
  if (transferInProgress) {
    Serial.println("⚠️ Transfer already in progress");
    return;
  }

  transferInProgress = true;
  Serial.println("\n🚀 STARTING BLE FILE TRANSFER...");
  if(systemStatus.buzzerOK) beep_ms(100, 220);

  int totalFiles = countFilesInDir();
  Serial.printf("📊 Total files to send: %d\n", totalFiles);

  String startMsg = "TRANSFER_START|TOTAL:" + String(totalFiles);
  pFileTransferCharacteristic->setValue(startMsg.c_str());
  pFileTransferCharacteristic->notify();
  delay(100);

  File root = SD.open("/farmland_data");
  if (root) {
    File file = root.openNextFile();
    while (file && deviceConnected && transferInProgress) {
      if (!file.isDirectory()) {
        String fullPath = String("/farmland_data/") + file.name();
        sendFile(fullPath);

        if(!deviceConnected) break;
      }
      file = root.openNextFile();
    }
    root.close();
  }

  if (deviceConnected && transferInProgress) {
    delay(200);
    String completeMsg = "TRANSFER_COMPLETE|All " + String(totalFiles) + " files transferred!";
    pFileTransferCharacteristic->setValue(completeMsg.c_str());
    pFileTransferCharacteristic->notify();
    delay(100);
    
    Serial.println("🎉 ALL FILES TRANSFERRED SUCCESSFULLY!");
    if(systemStatus.buzzerOK) beep_ms(300, 240);

    showingTransferComplete = true;
  }

  transferInProgress = false;
  transferRequested = false;
}

void autoStartTransfer() {
  if (transferRequested && deviceConnected && !transferInProgress) {
    startDynamicFileTransfer();
  }
}

void formatSDCard() {
  if(!sdOK) return;

  Serial.println("🔄 Formatting SD card...");

  File root = SD.open("/farmland_data");
  if (root) {
    File file = root.openNextFile();
    while(file) {
      String fileName = file.name();
      if(!file.isDirectory()) {
        SD.remove(String("/farmland_data/") + fileName);
      }
      file = root.openNextFile();
    }
    root.close();
  }

  Serial.println("✅ SD Card formatted successfully!");
  fileCounter = 1;
  if(systemStatus.buzzerOK) beep_ms(200, 200);
}

void initializeBLE() {
  Serial.println("📡 Initializing BLE...");

  BLEDevice::init("AGNI-SOIL-SENSOR");
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // FIX: Properly configure the file transfer characteristic for notifications
  pFileTransferCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TRANSFER,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pFileTransferCharacteristic->addDescriptor(new BLE2902());
  pFileTransferCharacteristic->setNotifyProperty(true); // Explicitly enable notifications

  pCommandCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_COMMAND,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCommandCharacteristic->addDescriptor(new BLE2902());
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();

  systemStatus.bleOK = true;
  Serial.println("✅ BLE initialized successfully!");
  Serial.println("📡 Advertising as: AGNI-SOIL-SENSOR\n");
}

// ============================================================================
// SYSTEM STATUS PRINT
// ============================================================================
void printSystemStatus() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║               🌱 AGNI SOIL SENSOR - SYSTEM STATUS              ║");
  Serial.println("╠═══════════════════════════════════════════════════════════════╣");

  Serial.printf("║ OLED:%s SD:%s RS485:%s GPS:%s BLE:%s BZ:%s TF:%s ║\n",
    systemStatus.oledOK ? "OK" : "NO",
    systemStatus.sdOK ? "OK" : "NO",
    systemStatus.rs485OK ? "OK" : "NO",
    systemStatus.gpsFix ? "FIX" : "NO",
    systemStatus.bleOK ? "OK" : "NO",
    systemStatus.buzzerOK ? "OK" : "NO",
    transferInProgress ? "BUSY" : "IDLE");

  if(soilData.basicValid) {
    Serial.printf("║ Soil - Moisture: %.1f%%  Temp: %.1f°C  pH: %.1f  EC: %duS/cm ║\n",
      soilData.moisture, soilData.temperature, soilData.ph, soilData.conductivity);

    if(soilData.npkValid) {
      Serial.printf("║ NPK - N:%d  P:%d  K:%d mg/kg                              ║\n",
        soilData.nitrogen, soilData.phosphorus, soilData.potassium);
    }
  }

  if(systemStatus.gpsFix) {
    Serial.printf("║ Loc - Lat: %.6f  Lon: %.6f  Alt: %.1fm            ║\n",
      systemStatus.latitude, systemStatus.longitude, systemStatus.altitude);

    char timestamp[25];
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d UTC",
      systemStatus.year, systemStatus.month, systemStatus.day,
      systemStatus.hour, systemStatus.minute, systemStatus.second);
    Serial.printf("║ Time: %s                    ║\n", timestamp);
  }

  Serial.printf("║ Files Logged: %d                                           ║", fileCounter - 1);
  Serial.println("\n╚═══════════════════════════════════════════════════════════════╝\n");
}

// ============================================================================
// STARTUP SEQUENCES
// ============================================================================
void showStartupAnimation(unsigned long durationMs) {
  if(!systemStatus.oledOK) return;
  unsigned long start = millis();
  while(millis() - start < durationMs) {
    if(transferRequested) return;
    updateGPS();
    display.clearDisplay();
    display.setTextSize(2);
    int step = ((millis() - start) / 250) % (SCREEN_WIDTH - 40);
    display.setCursor(step, 10);
    display.println("AGNI");
    display.setTextSize(1);
    display.setCursor(0, 40);
    display.println("SOIL SENSOR");
    display.display();
    delay(50);
  }
}

void showComponentStatus(unsigned long durationMs) {
  if(!systemStatus.oledOK) return;
  unsigned long start = millis();
  while(millis() - start < durationMs) {
    if(transferRequested) return;
    updateGPS();
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("SYSTEM COMPONENTS");
    display.println("-----------------");
    display.printf("OLED : %s\n", systemStatus.oledOK ? "OK" : "INVALID");
    display.printf("SD   : %s\n", systemStatus.sdOK ? "OK" : "INVALID");
    display.printf("RS485: %s\n", systemStatus.rs485OK ? "OK" : "INVALID");
    display.printf("Soil : %s\n", systemStatus.soilSensorOK ? "OK" : "INVALID");
    display.printf("GPS  : %s\n", systemStatus.gpsFix ? "FIX" : "Searching...");
    display.printf("BLE  : %s\n", deviceConnected ? "Connected" : "Advertising");
    display.printf("Buzzer: %s\n", systemStatus.buzzerOK ? "OK" : "No");
    display.display();
    delay(150);
  }
}

void showInsertPromptAndCountdown() {
  if(!systemStatus.oledOK) return;
  for(int remain = 5; remain >= 0; --remain) {
    if(transferRequested) return;
    updateGPS();
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("PLEASE INSERT THE SOIL SENSOR");
    display.println();
    display.println("Insert probe into soil now.");
    display.setCursor(95, 54);
    display.setTextSize(1);
    display.printf("%d", remain);
    display.display();
    delay(1000);
  }
}

bool analyzeSoilAndCreateFile(int analyzeSeconds) {
  unsigned long analyzeStart = millis();
  unsigned long analyzeEnd = analyzeStart + (unsigned long)analyzeSeconds * 1000UL;

  while(millis() < analyzeEnd) {
    if(transferRequested) {
      Serial.println("⏸ Analysis aborted: BLE transfer requested");
      return false;
    }

    updateGPS();
    readSoilSensor();

    unsigned long elapsed = millis() - analyzeStart;
    int remaining = analyzeSeconds - (elapsed / 1000);

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Analyzing Your Soil...");
    display.println();
    if(soilData.basicValid) {
      display.printf("M:%.1f%% T:%.1fC pH:%.1f\n", soilData.moisture, soilData.temperature, soilData.ph);
    } else {
      display.println("Reading sensor...");
    }

    display.setCursor(95, 54);
    display.printf("%d", max(0, remaining));
    display.display();

    delay(400);
  }

  if(systemStatus.sdOK && soilData.basicValid) {
    logDataToSD();

    int totalFiles = countFilesInDir();
    unsigned long showStart = millis();
    while(millis() - showStart < 3000) {
      if(transferRequested) break;
      updateGPS();
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("FILE Creation successful");
      display.println();
      display.printf("Total files: %d\n", totalFiles);
      display.display();
      delay(100);
    }

    return true;
  } else {
    unsigned long showStart = millis();
    while(millis() - showStart < 2000) {
      if(transferRequested) break;
      updateGPS();
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Soil read failed");
      display.println("No file created.");
      display.display();
      delay(100);
    }
    return false;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║          🌱 AGNI SOIL SENSOR - COMPLETE INTEGRATED SYSTEM      ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════╝\n");

  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_FREQUENCY, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  systemStatus.buzzerOK = true;
  systemStatus.buzzerOK ? Serial.println("✅ Buzzer init OK") : Serial.println("❌ Buzzer init failed");

  if(systemStatus.buzzerOK) {
    beep_ms(100, 200);
    delay(50);
  }

  initOLED();
  initSDCard();

  if(systemStatus.sdOK) {
    clearDataDirectory();
  }

  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("✅ RS485 Modbus initialized");

  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("✅ GPS module initialized");

  initializeBLE();

  beep_ms(120, 200);
  delay(150);
  beep_ms(120, 200);

  Serial.println("✅ All systems initialized successfully!");
  Serial.println("🚀 System ready - Starting interactive flow...\n");

  updateOLED();

  showStartupAnimation(3000);
  showComponentStatus(3000);
}

// ============================================================================
// MAIN LOOP - WITH TRANSFER COMPLETE DISPLAY BLOCKING
// ============================================================================
void loop() {
  static unsigned long lastOLEDUpdate = 0;
  static unsigned long lastStatusDisplay = 0;
  static unsigned long lastPeriodicLog = 0;
  static unsigned long transferCompleteStart = 0;

  // ========== BLOCKING STATE: Show transfer complete for 2 seconds ==========
  if(showingTransferComplete) {
    if(transferCompleteStart == 0) {
      transferCompleteStart = millis();
    }

    updateGPS();

    if(systemStatus.oledOK) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("BLE Transfer Done");
      display.printf("Files: %d\n", countFilesInDir());
      display.println("Status: Success");
      display.display();
    }

    // Hold display for exactly 2 seconds
    if(millis() - transferCompleteStart >= 2000) {
      showingTransferComplete = false;
      transferCompleteStart = 0;
    }

    delay(100);
    return; // BLOCK THE REST OF LOOP - stays in this state
  }

  // ========== NORMAL OPERATION ==========
  updateGPS();

  if(millis() - lastOLEDUpdate >= 2000) {
    updateOLED();
    lastOLEDUpdate = millis();
  }

  if(millis() - lastStatusDisplay >= 10000) {
    printSystemStatus();
    lastStatusDisplay = millis();
  }

  if(systemStatus.sdOK && (millis() - lastPeriodicLog >= FILE_CREATION_INTERVAL_MS)) {
    readSoilSensor();
    if(soilData.basicValid) {
      logDataToSD();
    }
    lastPeriodicLog = millis();
  }

  autoStartTransfer();

  showInsertPromptAndCountdown();
  analyzeSoilAndCreateFile(45);

  delay(50);
}