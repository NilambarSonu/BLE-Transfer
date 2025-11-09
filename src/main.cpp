/*
 * AGNI SOIL SENSOR - COMPLETE INTEGRATED SYSTEM
 * 
 * Features:
 * - BLE File Transfer with real sensor data
 * - RS485 Modbus soil sensor integration
 * - GPS location tracking
 * - SD card data logging with JSON format
 * - OLED display
 * - Beautiful serial monitor output
 * 
 * Pin Configuration for ESP32-S3-DevKit-C1
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

// ============================================================================
// BUZZER CONFIGURATION
// ============================================================================
#define BUZZER_PIN 7
int buzzerVolume = 180;

// ============================================================================
// BLE CONFIGURATION
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
void createInitialDataFile();
void logDataToSD();

// ============================================================================
// BUZZER FUNCTIONS
// ============================================================================
void beep(int duration = 100) {
  analogWrite(BUZZER_PIN, buzzerVolume);
  delay(duration);
  analogWrite(BUZZER_PIN, 0);
}

// ============================================================================
// OLED DISPLAY FUNCTIONS
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
  display.setCursor(0,0);
  display.setTextSize(1);
  
  display.println("AGNI SOIL SENSOR");
  display.println("===============");
  
  if(soilData.basicValid) {
    display.printf("M:%.1f%% T:%.1fC\n", soilData.moisture, soilData.temperature);
    display.printf("pH:%.1f C:%duS\n", soilData.ph, soilData.conductivity);
    
    if(soilData.npkValid) {
      display.printf("N:%d P:%d K:%d mg/kg\n", soilData.nitrogen, soilData.phosphorus, soilData.potassium);
    } else {
      display.println("NPK: Reading...");
    }
  } else {
    display.println("Soil: No Data");
  }
  
  display.println("===============");
  
  if(systemStatus.gpsFix) {
    display.printf("GPS: Fix OK (%d Sats)\n", systemStatus.satellites);
  } else {
    display.println("GPS: Searching...");
  }
  
  if(deviceConnected) {
    display.println("BLE: Connected");
  } else {
    display.println("BLE: Advertising");
  }
  
  display.display();
}

// ============================================================================
// SD CARD FUNCTIONS
// ============================================================================
void initSDCard() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI, 1000000)) {
    Serial.println("❌ SD Card initialization failed!");
    systemStatus.sdOK = false;
    return;
  }

  uint8_t cardType = SD.cardType();
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);

  Serial.printf("✅ SD Card initialized: %llu MB\n", cardSize);
  
  if(!SD.exists("/farmland_data")) {
    SD.mkdir("/farmland_data");
  }
  
  systemStatus.sdOK = true;
}

String generateJSONData() {
  JsonDocument doc;
  
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
  }
  char time_ist[20];
  sprintf(time_ist, "%02d:%02d %s", ist_hour, ist_minute, ist_hour >= 12 ? "PM" : "AM");
  doc["time_ist"] = time_ist;
  
  // Location data
  JsonObject location = doc["location"].to<JsonObject>();
  location["latitude"] = systemStatus.latitude;
  location["longitude"] = systemStatus.longitude;
  location["valid"] = systemStatus.gpsFix;
  location["satellites"] = systemStatus.satellites;
  location["altitude"] = systemStatus.altitude;
  location["speed_kmh"] = gps.speed.kmph();
  location["hdop"] = gps.hdop.hdop();
  
  // pH category
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

void createInitialDataFile() {
  if(!systemStatus.sdOK) {
    Serial.println("❌ SD Card not available - skipping initial file creation");
    return;
  }
  
  String filename = "/farmland_data/farmland_" + String(fileCounter) + ".json";
  File file = SD.open(filename, FILE_WRITE);
  
  if(!file) {
    Serial.println("❌ Failed to create initial JSON file");
    return;
  }
  
  String jsonData = generateJSONData();
  file.print(jsonData);
  file.close();
  
  fileCounter++;
  Serial.println("✅ Initial data file created: " + filename);
}

// ============================================================================
// MODBUS/RS485 FUNCTIONS
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
  
  while(millis() - startTime < MODBUS_TIMEOUT && rxLen < sizeof(rxBuf)) {
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
// GPS FUNCTIONS
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
// BLE CALLBACKS
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

// ============================================================================
// BLE FILE TRANSFER
// ============================================================================
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

      if (progress % 10 == 0) {
        Serial.println(String(fileName) + " " + String(progress) + "%");
      }
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
      if (!file.isDirectory()) {
        totalFiles++;
      }
      file = root.openNextFile();
    }
    root.close();
  }

  Serial.printf("📊 Total files to send: %d\n", totalFiles);

  root = SD.open("/farmland_data");
  if (root) {
    File file = root.openNextFile();
    while (file && deviceConnected && transferInProgress) {
      if (!file.isDirectory()) {
        sendFile("/farmland_data/" + String(file.name()));
        delay(200);
      }
      file = root.openNextFile();
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
  if (!deviceConnected) {
    transferStarted = false;
    connectionTime = 0;
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
        SD.remove("/farmland_data/" + fileName);
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
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
    CHARACTERISTIC_UUID_TRANSFER,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pFileTransferCharacteristic->addDescriptor(new BLE2902());

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
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  systemStatus.bleOK = true;
  Serial.println("✅ BLE initialized successfully!");
  Serial.println("📡 Advertising as: AGNI-SOIL-SENSOR\n");
}

// ============================================================================
// SYSTEM STATUS DISPLAY
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
    systemStatus.gpsFix ? "✅" : "❌",
    systemStatus.satellites);
  
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
// MAIN SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║          🌱 AGNI SOIL SENSOR - COMPLETE INTEGRATED SYSTEM      ║");
  Serial.println("║                    All Systems Working Together                ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════╝\n");
  
  // Buzzer init
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 0);
  beep(100);
  
  Serial.println("🔧 Initializing components...\n");
  
  // OLED
  initOLED();
  
  // SD Card
  initSDCard();
  
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
  
  beep(200);
  delay(200);
  beep(200);
  
  Serial.println("✅ All systems initialized successfully!");
  Serial.println("🚀 System ready - Starting sensor readings...\n");
  
  // Create initial data file so BLE has something to transfer
  delay(1000);
  if(systemStatus.sdOK) {
    createInitialDataFile();
  }
  
  updateOLED();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  static unsigned long lastSoilRead = 0;
  static unsigned long lastStatusDisplay = 0;
  static unsigned long lastDataLog = 0;
  static unsigned long lastOLEDUpdate = 0;
  static bool initialReadDone = false;
  
  // Continuous GPS update
  updateGPS();
  
  // Do initial soil sensor read immediately
  if(!initialReadDone && millis() < 2000) {
    if(readSoilSensor()) {
      Serial.println("✅ Initial soil sensor data read");
      initialReadDone = true;
    }
  }
  
  // Read soil sensor every 5 seconds
  if(millis() - lastSoilRead >= 5000) {
    if(readSoilSensor()) {
      Serial.println("✅ Soil sensor data updated");
    } else {
      Serial.println("⚠️  Soil sensor reading failed");
    }
    lastSoilRead = millis();
  }
  
  // Update OLED every 2 seconds
  if(millis() - lastOLEDUpdate >= 2000) {
    updateOLED();
    lastOLEDUpdate = millis();
  }
  
  // Log data to SD every 30 seconds
  if(systemStatus.sdOK && millis() - lastDataLog >= 30000) {
    logDataToSD();
    lastDataLog = millis();
  }
  
  // Display system status every 10 seconds
  if(millis() - lastStatusDisplay >= 10000) {
    printSystemStatus();
    lastStatusDisplay = millis();
  }
  
  // Handle BLE auto-transfer
  autoStartTransfer();
  
  delay(10);
}