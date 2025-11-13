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
#include <esp_task_wdt.h>
#include<time.h>
// ============================================================================
// CONFIGURABLE SETTINGS
// ============================================================================
#define DATA_LOG_INTERVAL 45000  
#define WATCHDOG_TIMEOUT 30      
#define JSON_DOC_SIZE 1024
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
#define BUZZER_CHANNEL 0
#define BUZZER_RESOLUTION 8
#define BUZZER_VOLUME 185  
#define BUZZER_DEFAULT_FREQ 1000
// --- Note Frequencies (for melodies) ---
#define NOTE_C4 262
#define NOTE_E4 330
#define NOTE_G4 392
#define NOTE_C5 523

// ============================================================================
// DISPLAY STATES
// ============================================================================
enum DisplayState {
  STATE_INITIAL,
  STATE_COMPONENT_CHECK,
  STATE_PLACE_SENSOR,
  STATE_ANALYZING,
  STATE_FILE_CREATED,
  STATE_BLE_TRANSFER
};

DisplayState currentState = STATE_INITIAL;
unsigned long stateStartTime = 0;
unsigned long countdownStartTime = 0;
int countdownValue = 0;

// ============================================================================
// BLE CONFIGURATION
// ============================================================================
BLEServer* pServer = NULL;
BLECharacteristic* pFileTransferCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool transferInProgress = false;
bool transferPending = false;
DisplayState previousStateBeforeTransfer = STATE_PLACE_SENSOR;

#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID_TRANSFER "abcdef12-3456-7890-1234-567890abcdef"
#define CHARACTERISTIC_UUID_COMMAND "abcdef13-3456-7890-1234-567890abcdef"

// ============================================================================
// NON-BLOCKING TRANSFER VARIABLES
// ============================================================================
File currentTransferFile;
File transferRoot;
String currentTransferFileName;
size_t currentTransferBytesSent = 0;
size_t currentTransferFileSize = 0;
unsigned long lastTransferChunkTime = 0;
const size_t TRANSFER_CHUNK_SIZE = 256;   // changed from 128 for faster transfer

DisplayState previousStateBeforeTransfer = STATE_PLACE_SENSOR;
volatile int g_bleCommandToProcess = 0; // 0=None, 1=Start_Transfer, 2=Format, 3=Reset
// ============================================================================
// ERROR RECOVERY VARIABLES
// ============================================================================
int soilSensorFailureCount = 0;
const int MAX_SENSOR_FAILURES = 5;
unsigned long lastSensorReset = 0;
const unsigned long SENSOR_RESET_COOLDOWN = 10000;

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
  int satellites = 7;
  float latitude = 21.06631583;
  float longitude = 86.48895417;
  float altitude = 14.7;
  int year = 2025;
  int month = 11;
  int day = 10;
  int hour = 12;
  int minute = 15;
  int second = 28;
};

SensorData soilData;
SystemStatus systemStatus;
TaskHandle_t SoilSensorTask;
QueueHandle_t soilDataQueue;
// ============================================================================
// ANIMATION CODES
// ============================================================================
#define FRAME_FIRE_DELAY (42)
#define FRAME_FIRE_WIDTH (64)
#define FRAME_FIRE_HEIGHT (64)
#define FRAME_FIRE_COUNT (sizeof(frames_fire) / sizeof(frames_fire[0]))
const byte PROGMEM frames_fire[][512] = {
  {0,0,0,24,0,0,0,0,0,0,0,30,0,0,0,0,0,0,0,31,0,0,0,0,0,0,0,63,128,0,0,0,0,0,0,59,224,0,0,0,0,0,0,56,240,0,0,0,0,0,0,48,120,0,0,0,0,0,0,112,60,0,0,0,0,0,0,112,14,0,0,0,0,0,0,224,7,128,0,0,0,0,0,224,3,192,0,0,0,0,1,192,1,224,0,0,0,0,1,192,0,240,0,0,0,0,3,128,0,120,0,0,0,0,3,128,0,60,0,0,0,0,7,0,0,30,0,0,0,0,14,0,0,15,0,0,0,0,30,0,0,7,0,0,0,0,28,0,0,3,128,0,0,0,56,0,0,1,192,0,0,0,112,0,192,0,224,0,0,0,224,0,224,0,240,0,0,1,224,0,240,0,112,0,0,3,192,0,252,0,56,0,0,7,128,0,252,0,28,0,0,7,0,0,238,0,28,0,0,14,0,0,231,0,14,0,0,28,0,0,227,128,14,0,0,60,0,0,227,128,7,0,0,56,0,0,225,192,7,0,0,112,0,0,224,224,3,128,0,112,0,0,96,224,3,128,0,224,0,0,224,112,1,128,0,192,3,0,224,112,1,192,1,192,7,0,224,48,1,192,1,192,7,128,192,56,1,192,3,128,15,193,192,56,0,192,3,128,14,225,128,56,0,192,3,0,28,255,128,56,0,224,3,0,24,63,0,24,0,224,3,0,24,30,0,24,0,224,7,0,56,0,0,24,0,224,7,0,56,0,0,56,0,224,7,0,56,0,0,56,0,224,7,0,56,0,0,56,0,224,7,0,56,0,0,48,0,224,7,0,56,0,0,112,0,192,7,0,24,0,0,112,1,192,3,0,28,0,0,96,1,192,3,128,14,0,0,224,1,192,3,128,15,0,1,192,3,128,1,128,7,128,1,192,3,128,1,192,3,192,3,128,7,0,0,224,0,128,1,0,7,0,0,224,0,0,0,0,14,0,0,112,0,0,0,0,14,0,0,56,0,0,0,0,28,0,0,60,0,0,0,0,56,0,0,30,0,0,0,0,112,0,0,7,128,0,0,0,224,0,0,3,224,0,0,1,224,0,0,1,255,0,0,15,128,0,0,0,127,128,0,15,0,0,0,0,15,128,0,12,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,48,0,0,0,0,0,0,0,120,0,0,0,0,0,0,0,124,0,0,0,0,0,0,0,127,0,0,0,0,0,0,0,103,128,0,0,0,0,0,0,227,192,0,0,0,0,0,0,224,240,0,0,0,0,0,1,192,120,0,0,0,0,0,1,192,60,0,0,0,0,0,3,128,30,0,0,0,0,0,3,128,15,0,0,0,0,0,7,0,3,128,0,0,0,0,7,0,1,192,0,0,0,0,14,0,0,224,0,0,0,0,14,0,0,112,0,0,0,0,28,0,0,56,0,0,0,0,56,0,0,28,0,0,0,0,120,0,0,14,0,0,0,0,112,0,0,7,0,0,0,0,224,1,0,7,128,0,0,1,224,3,128,3,192,0,0,1,192,3,192,1,192,0,0,3,128,3,224,0,224,0,0,7,0,3,248,0,112,0,0,14,0,1,188,0,112,0,0,30,0,1,156,0,56,0,0,28,0,1,142,0,28,0,0,56,0,1,207,0,28,0,0,112,0,1,199,128,14,0,0,112,0,1,195,128,14,0,0,224,0,1,193,192,7,0,0,192,0,1,193,224,7,0,1,192,0,1,128,224,3,0,1,192,6,1,128,96,3,128,3,128,15,3,128,112,3,128,3,128,15,131,128,112,1,128,3,0,29,231,0,48,1,192,7,0,25,255,0,48,1,192,7,0,56,124,0,56,1,192,7,0,56,16,0,56,1,192,7,0,48,0,0,56,0,192,6,0,48,0,0,56,0,192,6,0,48,0,0,48,1,192,6,0,48,0,0,48,1,192,6,0,48,0,0,112,1,192,6,0,56,0,0,112,1,192,7,0,56,0,0,96,1,128,7,0,28,0,0,224,1,128,7,0,30,0,0,224,3,128,3,0,15,0,1,192,3,128,3,128,7,128,3,128,3,0,1,128,3,192,3,128,7,0,1,192,0,128,1,0,14,0,0,224,0,0,0,0,14,0,0,240,0,0,0,0,28,0,0,112,0,0,0,0,60,0,0,56,0,0,0,0,56,0,0,30,0,0,0,0,112,0,0,15,0,0,0,1,224,0,0,3,192,0,0,3,192,0,0,1,255,0,0,15,128,0,0,0,127,128,0,31,0,0,0,0,7,128,0,28,0,0},
  // ... Include all 16 frames from your fire animation here
  // (I've shown first 2 frames, include the remaining 14 frames)
};

#define FRAME_LEAF_DELAY (42)
#define FRAME_LEAF_WIDTH (64)
#define FRAME_LEAF_HEIGHT (64)
#define FRAME_LEAF_COUNT (sizeof(frames_leaf) / sizeof(frames_leaf[0]))
const byte PROGMEM frames_leaf[][512] = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,120,0,0,0,0,0,0,0,248,0,0,0,0,0,0,1,248,0,0,0,0,0,0,7,220,0,0,0,0,0,0,63,156,0,0,0,0,0,1,254,12,0,0,0,0,0,31,248,14,0,0,0,0,1,255,192,14,0,0,0,0,15,248,0,6,0,0,0,0,127,128,0,7,0,0,0,0,252,0,0,7,0,0,0,3,224,0,0,3,0,0,0,15,128,0,0,3,128,0,0,30,0,0,0,3,128,0,0,60,0,0,0,1,128,0,0,240,0,0,0,1,128,0,0,224,0,0,0,1,192,0,1,192,0,0,6,1,192,0,3,128,0,0,7,1,192,0,7,0,0,0,14,0,192,0,14,0,0,0,14,0,192,0,14,0,0,0,28,0,192,0,28,0,0,0,28,0,224,0,28,0,0,0,56,0,224,0,56,0,0,0,56,0,224,0,56,0,0,0,112,0,224,0,48,0,0,0,112,0,224,0,48,0,0,0,224,0,224,0,112,0,0,0,224,0,224,0,112,0,0,1,192,0,224,0,112,0,0,3,128,0,224,0,112,0,0,3,128,0,224,0,112,0,0,7,0,0,224,0,48,0,0,15,0,0,224,0,56,0,0,14,0,0,192,0,56,0,0,28,0,0,192,0,28,0,0,56,0,1,192,0,28,0,0,120,0,1,192,0,14,0,0,240,0,1,192,0,7,0,0,224,0,1,128,0,7,128,1,192,0,3,128,0,3,192,3,128,0,3,128,0,1,224,7,0,0,7,0,0,0,248,14,0,0,7,0,0,0,126,30,0,0,14,0,0,0,31,252,0,0,14,0,0,0,7,248,0,0,28,0,0,0,1,240,0,0,60,0,0,0,3,240,0,0,120,0,0,0,7,184,0,0,240,0,0,0,15,24,0,1,224,0,0,0,62,28,0,3,192,0,0,0,252,15,0,15,128,0,0,3,240,7,128,31,0,0,0,31,224,3,255,252,0,0,15,255,128,0,255,240,0,0,7,252,0,0,31,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,120,0,0,0,0,0,0,0,248,0,0,0,0,0,0,1,248,0,0,0,0,0,0,7,220,0,0,0,0,0,0,63,156,0,0,0,0,0,1,254,12,0,0,0,0,0,31,248,14,0,0,0,0,1,255,192,14,0,0,0,0,15,248,0,6,0,0,0,0,127,128,0,7,0,0,0,0,252,0,0,7,0,0,0,3,224,0,0,3,0,0,0,15,128,0,0,3,128,0,0,30,0,0,0,3,128,0,0,60,0,0,0,1,128,0,0,240,0,0,0,1,128,0,0,224,0,0,0,1,192,0,1,192,0,0,6,1,192,0,3,128,0,0,7,1,192,0,7,0,0,0,14,0,192,0,14,0,0,0,14,0,192,0,14,0,0,0,28,0,192,0,28,0,0,0,28,0,224,0,28,0,0,0,56,0,224,0,56,0,0,0,56,0,224,0,56,0,0,0,112,0,224,0,48,0,0,0,112,0,224,0,48,0,0,0,224,0,224,0,112,0,0,0,224,0,224,0,112,0,0,1,192,0,224,0,112,0,0,3,128,0,224,0,112,0,0,3,128,0,224,0,112,0,0,7,0,0,224,0,48,0,0,15,0,0,224,0,56,0,0,14,0,0,192,0,56,0,0,28,0,0,192,0,28,0,0,56,0,1,192,0,28,0,0,120,0,1,192,0,14,0,0,240,0,1,192,0,7,0,0,224,0,1,128,0,7,128,1,192,0,3,128,0,3,192,3,128,0,3,128,0,1,224,7,0,0,7,0,0,0,248,14,0,0,7,0,0,0,126,30,0,0,14,0,0,0,31,252,0,0,14,0,0,0,7,248,0,0,28,0,0,0,1,240,0,0,60,0,0,0,3,240,0,0,120,0,0,0,7,184,0,0,240,0,0,0,15,24,0,1,224,0,0,0,62,28,0,3,192,0,0,0,252,15,0,15,128,0,0,3,240,7,128,31,0,0,0,31,224,3,255,252,0,0,15,255,128,0,255,240,0,0,7,252,0,0,31,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};
// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void playIntroAnimation();
void startDynamicFileTransfer();
void processTransferChunk();
void formatSDCard();
String generateJSONData();
void logDataToSD();
void changeState(DisplayState newState);
bool isValidStateTransition(DisplayState from, DisplayState to);
void updateDisplayState();
void showInitialScreen();
void showComponentCheckScreen();
void showPlaceSensorScreen();
void showAnalyzingScreen();
void showFileCreatedScreen();
void showBLETransferScreen();
void clearSDCardData();
void resetToNormalOperation();
void recoverFromSoilSensorFailure();
bool checkSDHealth();
void monitorSystemHealth();
void resetSoilSensor();
// ============================================================================
// BUZZER FUNCTIONS
// ============================================================================
/**
 * @brief The low-level helper function that plays any note.
 */
void playNote(int frequency, int duration) {
  ledcSetup(BUZZER_CHANNEL, frequency, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, BUZZER_VOLUME);
  delay(duration);
  ledcWrite(BUZZER_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
}
/**
 * @brief Your chosen success sound (Pattern 4: C-E-G-C5)
 */
void playSuccessSound() {
  Serial.println("🔊 Playing Success Sound!");
  playNote(NOTE_C4, 120);
  delay(40);
  playNote(NOTE_E4, 120);
  delay(40);
  playNote(NOTE_G4, 120);
  delay(40);
  playNote(NOTE_C5, 200);
}
/**
 * @brief A simple beep for all other notifications.
 */
void beep(int duration = 100) {
  playNote(BUZZER_DEFAULT_FREQ, duration);
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
  systemStatus.oledOK = true;
  Serial.println("✅ OLED initialized");
}
void showInitialScreen() {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("  AGNI");
  display.println("  SOIL");
  display.println(" SENSOR");
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.println("Initializing...");
  display.display();
}
void showComponentCheckScreen() {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("COMPONENT CHECK");
  display.println("===============");
  display.printf("OLED: %s\n", systemStatus.oledOK ? "OK" : "INVALID");
  display.printf("SD: %s\n", systemStatus.sdOK ? "OK" : "INVALID");
  display.printf("SOIL: %s\n", systemStatus.soilSensorOK ? "OK" : "INVALID");
  display.printf("GPS: %s\n", systemStatus.gpsOK ? "OK" : "INVALID");
  display.printf("BLE: %s\n", systemStatus.bleOK ? "OK" : "INVALID");
  display.display();
}
void showPlaceSensorScreen() {
  if(!systemStatus.oledOK) return;
  unsigned long currentTime = millis();
  int remainingTime = 5 - ((currentTime - countdownStartTime) / 1000);
  if (remainingTime < 0) remainingTime = 0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("PLACE RECENT THE");
  display.println("SOIL SENSOR");
  display.println();
  if (systemStatus.gpsFix) {
    display.printf("GPS: Fix OK (%d Sats)\n", systemStatus.satellites);
  } else {
    display.println("GPS: Searching...");
  }
  display.println();
  display.printf("Countdown: %d\n", remainingTime);
  display.display();
}
void showAnalyzingScreen() {
  if(!systemStatus.oledOK) return;
  unsigned long currentTime = millis();
  int remainingTime = (DATA_LOG_INTERVAL / 1000) - ((currentTime - countdownStartTime) / 1000);
  if (remainingTime < 0) remainingTime = 0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Analyzing Your");
  display.println("Soil...");
  display.println();
  if (soilData.basicValid) {
    display.printf("M:%.1f%% T:%.1fC\n", soilData.moisture, soilData.temperature);
    display.printf("pH:%.1f C:%duS\n", soilData.ph, soilData.conductivity);
  } else {
    display.println("Reading sensors...");
  }
  display.println();
  display.printf("Countdown: %d\n", remainingTime);
  display.display();
}
void showFileCreatedScreen() {
  if(!systemStatus.oledOK) return;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("FILE CREATION");
  display.println("SUCCESSFUL");
  display.println();
  display.printf("Total Files: %d\n", fileCounter - 1);
  display.println();
  display.println("Data saved to SD card");
  display.display();
}
void showBLETransferScreen() {
  if(!systemStatus.oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("BLE FILE TRANSFER");
  display.println("IN PROGRESS...");
  display.println();
  display.println("Do not disconnect!");
  display.println();
  display.println("Sensors still reading");
  display.println("in background");
  display.display();
}
void updateDisplayState() {
  static unsigned long lastUpdate = 0;
  static DisplayState lastState = (DisplayState)99;
  // Only update display every second unless in analyzing state
  if (millis() - lastUpdate < 1000 && currentState != STATE_ANALYZING) {
    return;
  }
  if (currentState != lastState) {
    lastState = currentState; // Update the last known state
  } else if (currentState != STATE_PLACE_SENSOR && currentState != STATE_ANALYZING) {
    // If state is same, and not a countdown screen, skip redraw
    return;
  }
  lastUpdate = millis();
  switch(currentState) {
    case STATE_INITIAL:
      showInitialScreen();
      break;
    case STATE_COMPONENT_CHECK:
      showComponentCheckScreen();
      break;
    case STATE_PLACE_SENSOR:
      showPlaceSensorScreen();
      break;
    case STATE_ANALYZING:
      showAnalyzingScreen();
      break;
    case STATE_FILE_CREATED:
      showFileCreatedScreen();
      break;
    case STATE_BLE_TRANSFER:
      showBLETransferScreen();
      break;
  }
}
bool isValidStateTransition(DisplayState from, DisplayState to) {
  switch (from) {
    case STATE_INITIAL:
      return to == STATE_COMPONENT_CHECK;
    case STATE_COMPONENT_CHECK:
      return to == STATE_PLACE_SENSOR;
    case STATE_PLACE_SENSOR:
      return to == STATE_ANALYZING || to == STATE_BLE_TRANSFER;
    case STATE_ANALYZING:
      return to == STATE_FILE_CREATED || to == STATE_BLE_TRANSFER;
    case STATE_FILE_CREATED:
      return to == STATE_PLACE_SENSOR || to == STATE_BLE_TRANSFER;
    case STATE_BLE_TRANSFER:
      return to == STATE_PLACE_SENSOR;
    default:
      return false;
  }
}
void changeState(DisplayState newState) {
  if (!isValidStateTransition(currentState, newState)) {
    Serial.printf("⚠️ Invalid state transition: %d -> %d\n", currentState, newState);
    return;
  }
  currentState = newState;
  stateStartTime = millis();
  countdownStartTime = millis();
  updateDisplayState();
  
  Serial.printf("🔄 State changed to: %d\n", newState);
}

// ============================================================================
// ANIMATION FUNCTION
// ============================================================================
void playIntroAnimation() {
  if(!systemStatus.oledOK) return;

  unsigned long animationStartTime = 0;
  unsigned long lastFrameTime = 0;
  int currentFrame = 0;

  // --- 1. Play "Agni" (Fire) Animation for 2 seconds ---
  Serial.println("▶️  Playing intro animation 1 (Agni)...");
  animationStartTime = millis(); 
  
  // This 'while' loop will run for 2000ms (2 seconds)

  while(millis() - animationStartTime < 2000) { 
    if (millis() - lastFrameTime > FRAME_FIRE_DELAY) {
      lastFrameTime = millis();
      
      display.clearDisplay();
      display.drawBitmap(32, 0, frames_fire[currentFrame], FRAME_FIRE_WIDTH, FRAME_FIRE_HEIGHT, SSD1306_WHITE);
      display.display();
      currentFrame = (currentFrame + 1) % FRAME_FIRE_COUNT; 
    }
    
    // Pet the watchdog and yield to the OS
    esp_task_wdt_reset(); 
    delay(1); 
  }

  // --- 2. Play "Leaf" Animation for 1.5 seconds ---
  Serial.println("▶️  Playing intro animation 2 (Leaf)...");
  animationStartTime = millis(); 
  currentFrame = 0; 
  lastFrameTime = 0; 

  // This 'while' loop will run for 1500ms (1.5 seconds)
  while(millis() - animationStartTime < 1500) { 
    
    if (millis() - lastFrameTime > FRAME_LEAF_DELAY) {
      lastFrameTime = millis();
      
      display.clearDisplay();
      display.drawBitmap(32, 0, frames_leaf[currentFrame], FRAME_LEAF_WIDTH, FRAME_LEAF_HEIGHT, SSD1306_WHITE);
      display.display();
      
      currentFrame = (currentFrame + 1) % FRAME_LEAF_COUNT; 
    }
    
    esp_task_wdt_reset(); 
    delay(1); 
  }
  
  Serial.println("✅ Intro animations completed");
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
  // clearSDCardData();
  if(!SD.exists("/farmland_data")) {
    SD.mkdir("/farmland_data");
  }
  systemStatus.sdOK = true;
  findLastFileCounter();
}
bool checkSDHealth() {
  if (!systemStatus.sdOK) return false;
  
  uint64_t freeBytes = SD.totalBytes() - SD.usedBytes();
  if (freeBytes < 1024 * 1024) {
    Serial.println("⚠️ SD card running low on space!");
    return false;
  }
  return true;
}

/**
 * @brief Scans the /farmland_data directory to find the highest file number.
 * Sets the global 'fileCounter' to the next available number.
 */
void findLastFileCounter() {
  if (!systemStatus.sdOK) return;

  File root = SD.open("/farmland_data");
  if (!root) {
    Serial.println("❌ Failed to open /farmland_data to find last file.");
    fileCounter = 1;
    return;
  }

  int maxFileNum = 0;
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      // file.name() returns the full path, e.g., /farmland_data/farmland_12.json
      String fileName = file.name(); 
      // Get just the filename part: farmland_12.json
      String baseName = fileName.substring(fileName.lastIndexOf('/') + 1); 

      if (baseName.startsWith("farmland_") && baseName.endsWith(".json")) {
        // Extract the number part: "12"
        String numStr = baseName.substring(
          baseName.indexOf('_') + 1, 
          baseName.lastIndexOf('.')
        );

        int fileNum = numStr.toInt();
        if (fileNum > maxFileNum) {
          maxFileNum = fileNum;
        }
      }
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();

  fileCounter = maxFileNum + 1; // Start at the next number
  Serial.printf("✅ SD Scan: Resuming from file number %d\n", fileCounter);
}


// ============================================================================
// TIME CONVERSION HELPER
// ============================================================================
/**
 * @brief Converts the current systemStatus UTC time to IST (UTC+5:30).
 * Handles all date, month, and year rollovers correctly.
 * @param ist_year Output for IST year
 * @param ist_month Output for IST month (1-12)
 * @param ist_day Output for IST day (1-31)
 * @param ist_hour Output for IST hour (0-23)
 * @param ist_minute Output for IST minute (0-59)
 */
void getISTDateTime(int &ist_year, int &ist_month, int &ist_day, int &ist_hour, int &ist_minute) {
  // 1. Create a struct tm for the UTC time
  struct tm utc_tm;
  utc_tm.tm_year = systemStatus.year - 1900;  
  utc_tm.tm_mon  = systemStatus.month - 1;
  utc_tm.tm_mday = systemStatus.day;
  utc_tm.tm_hour = systemStatus.hour;
  utc_tm.tm_min  = systemStatus.minute;
  utc_tm.tm_sec  = systemStatus.second;
  utc_tm.tm_isdst = 0; 
  // setenv("TZ", "UTC", 1);  suggesstion from chatGPT
  // tzset();
  time_t utc_time = mktime(&utc_tm);
  // 2. Define the IST offset in seconds (5 hours * 3600s/hr) + (30 minutes * 60s/min)
  const long IST_OFFSET_SECONDS = 19800; // (5 * 3600) + (30 * 60)
  // 3. Add the offset to get the IST time
  time_t ist_time = utc_time + IST_OFFSET_SECONDS;
  // 4. Convert the IST timestamp back into a struct tm
  struct tm ist_tm;
  gmtime_r(&ist_time, &ist_tm);
  // 5. Assign the correct values to the output variables
  ist_year   = ist_tm.tm_year + 1900;
  ist_month  = ist_tm.tm_mon + 1;
  ist_day    = ist_tm.tm_mday;
  ist_hour   = ist_tm.tm_hour;
  ist_minute = ist_tm.tm_min;
}

/**
 * @brief Recursively deletes all files and sub-folders from a given directory.
 * @param dir The directory File object to start from.
 */
void deleteRecursive(File dir) {
  while(true) {
    File entry = dir.openNextFile();
    if (! entry) {
      // no more files or directories in this directory
      break;
    }

    String entryPath = entry.name();

    if (entry.isDirectory()) {
      // It's a directory. Recurse into it to delete its contents.
      Serial.println("  Entering dir: " + entryPath);
      File subDir = SD.open(entryPath);
      deleteRecursive(subDir);
      subDir.close();

      // Now that the directory is empty, remove it.
      Serial.println("  Removing dir: " + entryPath);
      SD.rmdir(entryPath);
    } else {
      // It's a file. Delete it.
      Serial.println("  Deleting file: " + entryPath);
      SD.remove(entryPath);
    }
    entry.close();
  }
}

void clearSDCardData() {
  if(!systemStatus.sdOK) return;
  Serial.println("🗑️  WIPING ENTIRE SD CARD (as requested on reset)...");

  File root = SD.open("/");
  if (root) {
    deleteRecursive(root); // <-- Call the new helper
    root.close();
  } else {
    Serial.println("❌ Failed to open root directory to wipe.");
  }

  fileCounter = 1;
  Serial.println("✅ SD Card Wiped!");

  // Re-create the data directory since we just deleted it
  if(!SD.exists("/farmland_data")) {
    SD.mkdir("/farmland_data");
    Serial.println("✅ Re-created /farmland_data directory.");
  }
}

String generateJSONData() {
  StaticJsonDocument<JSON_DOC_SIZE> doc;
  doc["id"] = fileCounter;
  
  if(systemStatus.gpsFix) {
    // --- Log UTC Data ---
    char timestamp[30];
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
      systemStatus.year, systemStatus.month, systemStatus.day,
      systemStatus.hour, systemStatus.minute, systemStatus.second);
    doc["timestamp"] = timestamp;
    
    char time_utc[10];
    sprintf(time_utc, "%02d:%02d:%02d", systemStatus.hour, systemStatus.minute, systemStatus.second);
    doc["time_utc"] = time_utc;
    int ist_year, ist_month, ist_day, ist_hour, ist_minute;
    getISTDateTime(ist_year, ist_month, ist_day, ist_hour, ist_minute);
    char date_ist[12];
    sprintf(date_ist, "%04d-%02d-%02d", ist_year, ist_month, ist_day);
    doc["date_ist"] = date_ist;
    char time_ist[20];
    int ist_hour_12 = ist_hour % 12;
    if (ist_hour_12 == 0) ist_hour_12 = 12;
    
    sprintf(time_ist, "%02d:%02d %s", ist_hour_12, ist_minute, ist_hour >= 12 ? "PM" : "AM");
    doc["time_ist"] = time_ist;

  } else {
    doc["timestamp"] = "0000-00-00T00:00:00Z";
    doc["time_utc"] = "00:00:00";
    doc["date_ist"] = "0000-00-00";
    doc["time_ist"] = "00:00 AM";
  }
  JsonObject location = doc.createNestedObject("location");
  location["latitude"] = systemStatus.gpsFix ? systemStatus.latitude : 0.0;
  location["longitude"] = systemStatus.gpsFix ? systemStatus.longitude : 0.0;
  location["valid"] = systemStatus.gpsFix;
  location["satellites"] = systemStatus.gpsFix ? systemStatus.satellites : 0;
  location["altitude"] = systemStatus.gpsFix ? systemStatus.altitude : 0.0;
  location["speed_kmh"] = systemStatus.gpsFix ? gps.speed.kmph() : 0.0;
  location["hdop"] = systemStatus.gpsFix ? gps.hdop.hdop() : 0.0;
  
  if(soilData.ph < 5.5) doc["ph_category"] = "acidic";
  else if(soilData.ph < 6.5) doc["ph_category"] = "slightly_acidic";
  else if(soilData.ph < 7.5) doc["ph_category"] = "neutral";
  else if(soilData.ph < 8.5) doc["ph_category"] = "slightly_alkaline";
  else doc["ph_category"] = "alkaline";
  
  JsonObject params = doc.createNestedObject("parameters");
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
  if(!systemStatus.sdOK || !checkSDHealth()) return;
  String filename = "/farmland_data/farmland_" + String(fileCounter) + ".json";
  File file = SD.open(filename, FILE_WRITE);
  if(!file) {
    Serial.println("❌ Failed to create JSON file: " + filename);
    return;
  }
  
  String jsonData = generateJSONData();
  file.print(jsonData);
  file.close();
  playSuccessSound();
  fileCounter++;
  Serial.println("✅ JSON data logged to SD card: " + filename);
  changeState(STATE_FILE_CREATED);
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

void resetSoilSensor() {
  Serial1.end();
  delay(100);
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  soilSensorFailureCount = 0;
  Serial.println("🔄 Soil sensor reset");
}

void recoverFromSoilSensorFailure() {
  if (millis() - lastSensorReset < SENSOR_RESET_COOLDOWN) return;
  soilSensorFailureCount++;
  Serial.printf("⚠️ Soil sensor failure count: %d/%d\n", soilSensorFailureCount, MAX_SENSOR_FAILURES);
  if (soilSensorFailureCount >= MAX_SENSOR_FAILURES) {
    Serial.println("🔄 Attempting sensor recovery...");
    resetSoilSensor();
    lastSensorReset = millis();
  }
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
  delay(10); // FIX: Was 2
  Serial1.write(txBuf, pos);
  Serial1.flush();
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
  delay(10);  // FIX: Was 2
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

bool readSoilSensor(SensorData &soilData) {
  uint16_t regs[4];
  if(!modbusRead(MODBUS_ADDRESS, REG_MOISTURE, 4, regs)) {
    soilData.basicValid = false;
    recoverFromSoilSensorFailure();
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
  soilSensorFailureCount = 0;
  return soilData.basicValid;
}

/**
 * @brief This is the dedicated task that runs on Core 0
 * to read the soil sensor without blocking the main loop.
 */
void soilSensorTaskLoop(void * pvParameters) {
  Serial.println("✅ Soil Sensor Task started on Core 0");
  SensorData localSensorData;
  for(;;) {
    esp_task_wdt_reset(); // Reset watchdog timer
    bool readOK = readSoilSensor(localSensorData);
    if(readOK) {
      Serial.println("✅ (Core 0) Soil sensor data updated");
      xQueueOverwrite(soilDataQueue, &localSensorData);
    } else {
      Serial.println("⚠️  (Core 0) Soil sensor reading failed");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// ============================================================================
// GPS FUNCTIONS
// ============================================================================
void updateGPS() {
  while(GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    gps.encode(c);
    gpsCharsProcessed++;
    if (gpsCharsProcessed > 10000) {
      gpsCharsProcessed = 100;
    }
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
    transferPending = false;
    Serial.println("🔴 BLE Client disconnected");
    delay(500);
    BLEDevice::startAdvertising();
    Serial.println("📡 BLE Advertising restarted\n");
    
    if (currentState == STATE_BLE_TRANSFER) {
      resetToNormalOperation();
    }
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      String command = String(value.c_str());
      Serial.println("📬 BLE Command received: " + command);
      // Set a flag instead of calling function directly
      if (command == "START_TRANSFER") {
        g_bleCommandToProcess = 1;
      } else if (command == "FORMAT_SD") {
        g_bleCommandToProcess = 2;
      } else if (command == "RESET_SYSTEM") {
        g_bleCommandToProcess = 3;
      }
    }
  }
};
// ============================================================================
// BLE FILE TRANSFER (NON-BLOCKING)
// ============================================================================
void startDynamicFileTransfer() {
  if (!systemStatus.sdOK || !deviceConnected) return;
  if (transferInProgress || transferPending) {
    Serial.println("⚠️  Transfer already in progress");
    return;
  }
  transferRoot = SD.open("/farmland_data");
  if (!transferRoot){
    Serial.println("❌ Failed to open /farmland_data directory");
    return;
  }

  previousStateBeforeTransfer = currentState;
  transferPending = true;
  Serial.println("\n🚀 STARTING BLE FILE TRANSFER...");
  beep(150);
  changeState(STATE_BLE_TRANSFER);
}

void processTransferChunk() {
  if (!transferInProgress && transferPending) {
    if(currentTransferFile) {
      currentTransferFile.close();
    }
    // Start new transfer
    transferInProgress = true;
    transferPending = false;
    if (transferRoot) {
      File file = transferRoot.openNextFile();
      if (file && !file.isDirectory()) {
        currentTransferFile = file;
        currentTransferFileName = file.name();
        currentTransferFileSize = file.size();
        currentTransferBytesSent = 0;
        
        String fileHeader = "FILE_START:" + currentTransferFileName + "|SIZE:" + String(currentTransferFileSize);
        pFileTransferCharacteristic->setValue(fileHeader.c_str());
        pFileTransferCharacteristic->notify();
        Serial.println("📤 Starting transfer: " + currentTransferFileName);
      } else {
        // No more files
        transferInProgress = false;
        if(transferRoot) transferRoot.close();
        String completeMsg = "TRANSFER_COMPLETE|All files transferred!";
        pFileTransferCharacteristic->setValue(completeMsg.c_str());
        pFileTransferCharacteristic->notify();
        Serial.println("🎉 ALL FILES TRANSFERRED SUCCESSFULLY!");
        playSuccessSound();
        resetToNormalOperation();
      }
    } else {
      Serial.println("❌ Transfer error: transferRoot is not open!");
      transferInProgress = false;
      resetToNormalOperation();
    }
    lastTransferChunkTime = millis();
    return;
  }

  if (!transferInProgress || !currentTransferFile) return;

  if (millis() - lastTransferChunkTime < 5) return; // Throttle transfers
  
  if (currentTransferBytesSent < currentTransferFileSize) {
    uint8_t buffer[TRANSFER_CHUNK_SIZE];
    size_t bytesRead = currentTransferFile.read(buffer, TRANSFER_CHUNK_SIZE);
    if (bytesRead > 0) {
      pFileTransferCharacteristic->setValue(buffer, bytesRead);
      pFileTransferCharacteristic->notify();
      currentTransferBytesSent += bytesRead;
      
      int progress = (int)((currentTransferBytesSent * 100) / currentTransferFileSize);
      if (progress % 20 == 0) {
        Serial.println(String(currentTransferFileName) + " " + String(progress) + "%");
      }
    }
  } else {
    // File transfer complete
    currentTransferFile.close();
    String fileEnd = "FILE_END:" + currentTransferFileName;
    pFileTransferCharacteristic->setValue(fileEnd.c_str());
    pFileTransferCharacteristic->notify();
    Serial.println("✅ Transferred: " + currentTransferFileName);
    
    // Move to next file
    transferInProgress = false;
    transferPending = true; 
  }
  
  lastTransferChunkTime = millis();
}

void autoStartTransfer() {
  static bool transferStarted = false;
  static unsigned long connectionTime = 0;

  if (deviceConnected && !transferStarted && !transferInProgress && !transferPending) {
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
  if(!systemStatus.sdOK) return;
  Serial.println("🔄 Formatting SD card...");
  File root = SD.open("/");
  if (root) {
    deleteRecursive(root); // <-- Call the new helper
    root.close();
  } else {
    Serial.println("❌ Failed to open root directory to format.");
  }

  Serial.println("✅ SD Card formatted successfully!");
  fileCounter = 1;
  beep(300);

  // Re-create the data directory since we just deleted it
  if(!SD.exists("/farmland_data")) {
    SD.mkdir("/farmland_data");
    Serial.println("✅ Re-created /farmland_data directory.");
  }
}

void resetToNormalOperation() {
  transferInProgress = false;
  transferPending = false;
  if (currentTransferFile) {
    currentTransferFile.close();
  }
  if (transferRoot){
    transferRoot.close();
  }
  changeState(STATE_PLACE_SENSOR);
  Serial.println("🔄 System reset to normal operation");
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
// SYSTEM HEALTH MONITORING
// ============================================================================
void monitorSystemHealth() {
  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck < 30000) return;
  
  Serial.printf("📊 Free heap: %d bytes\n", esp_get_free_heap_size());
  
  if (!checkSDHealth()) {
    Serial.println("⚠️  SD Card health check failed!");
  }
  
  lastHealthCheck = millis();
}

/**
 * @brief Checks the FreeRTOS queue for new sensor data from Core 0.
 * This is non-blocking and 100% thread-safe.
 */
void checkSoilSensorQueue() {
  SensorData newData;
  // Check if there is data in the queue (non-blocking)
  if (xQueueReceive(soilDataQueue, &newData, 0) == pdPASS) {
    soilData = newData; // This is a safe copy on Core 1
    systemStatus.soilSensorOK = soilData.basicValid;
    Serial.println("✅ (Core 1) Received new soil data from queue.");
  }
}

/**
 * @brief Safely handles commands from the BLE task in the main loop.
 */
void handleBleCommands() {
  if (g_bleCommandToProcess == 0) return; // No command

  // Safely copy the command and clear the flag
  int command = g_bleCommandToProcess;
  g_bleCommandToProcess = 0;

  Serial.printf("⚡ Executing BLE command: %d\n", command);

  switch (command) {
    case 1: // START_TRANSFER
      if (!transferInProgress && !transferPending) {
        startDynamicFileTransfer();
      }
      break;
    case 2: // FORMAT_SD
      formatSDCard();
      if(pCommandCharacteristic) {
        pCommandCharacteristic->setValue("SD_FORMATTED");
        pCommandCharacteristic->notify();
      }
      break;
    case 3: // RESET_SYSTEM
      resetToNormalOperation();
      if(pCommandCharacteristic) {
        pCommandCharacteristic->setValue("SYSTEM_RESET");
        pCommandCharacteristic->notify();
      }
      break;
  }
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
  
  Serial.printf("║ 🔵 BLE: %s  🛰️  Fix: %s  📡 Satellites: %2d                            ║\n",
    deviceConnected ? "🔗 Connected" : "📡 Advertising",
    systemStatus.gpsFix ? "✅" : "❌",
    systemStatus.satellites);
  
  if(soilData.basicValid) {
    Serial.printf("║ 🌍 Soil - Moisture: %.1f%%  Temp: %.1f°C  pH: %.1f  EC: %duS/cm       ║\n",
      soilData.moisture, soilData.temperature, soilData.ph, soilData.conductivity);
    
    if(soilData.npkValid) {
      Serial.printf("║ 🧪 NPK - N:%d  P:%d  K:%d mg/kg                                     ║\n",
        soilData.nitrogen, soilData.phosphorus, soilData.potassium);
    }
  }
  
  if(systemStatus.gpsFix) {
    Serial.printf("║ 📍 Location - Lat: %.6f  Lon: %.6f  Alt: %.1fm                        ║\n",
      systemStatus.latitude, systemStatus.longitude, systemStatus.altitude);
    
    char timestamp[25];
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d UTC",
      systemStatus.year, systemStatus.month, systemStatus.day,
      systemStatus.hour, systemStatus.minute, systemStatus.second);
    Serial.printf("║ ⏰ Timestamp: %s                    ║\n", timestamp);
  }
  
  Serial.printf("║ 💾 Files Logged: %d                                                     ║\n", fileCounter - 1);
  Serial.printf("║ 🔄 Transfer State: %s                                                   ║\n", 
    transferInProgress ? "IN PROGRESS" : (transferPending ? "PENDING" : "IDLE"));
  Serial.printf("║ 📊 Heap: %d bytes  Failures: %d                                         ║\n", 
    esp_get_free_heap_size(), soilSensorFailureCount);
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
  Serial.println("║                  With Enhanced Reliability                     ║");
  Serial.println("╚═════════════════════════════════════════════════════=══════════╝\n");
  
  // Initialize watchdog timer
  esp_task_wdt_init(WATCHDOG_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // Buzzer init
  Serial.println("🔊 Initializing Buzzer...");
  beep(100);
  Serial.println("🔧 Initializing components...\n");

  // OLED - Initialize FIRST
  initOLED();
  if (systemStatus.oledOK) {
    Serial.println("▶️  Playing intro animation...");
    playIntroAnimation();
  }

  // Start with initial display state
  // changeState(STATE_INITIAL);

  // SD Card
  initSDCard();

  // RS485 Soil Sensor
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("✅ RS485 Modbus initialized");
  // Create a queue to safely pass sensor data from Core 0 to Core 1
soilDataQueue = xQueueCreate(1, sizeof(SensorData));
if (soilDataQueue == NULL) {
  Serial.println("❌ Failed to create soilDataQueue!");
} else {
  Serial.println("✅ soilDataQueue created successfully");
}
  // Create the dedicated task for the blocking sensor
  xTaskCreatePinnedToCore(
      soilSensorTaskLoop,   /* Function to implement the task */
      "SoilSensorTask",     /* Name of the task */
      4096,                 /* Stack size in words */
      NULL,                 /* Task input parameter */
      1,                    /* Priority of the task */
      &SoilSensorTask,      /* Task handle. */
      0);   
    if (SoilSensorTask) {
      esp_task_wdt_add(SoilSensorTask); // <-- ADD THIS LINE
      }                 
  delay(500); // Give the task a moment to start
  // GPS
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("✅ GPS module initialized");
  // BLE
  initializeBLE();
  playSuccessSound();
  Serial.println("✅ All systems initialized successfully!");
  Serial.println("🚀 System ready - Starting sensor readings...\n");
  setenv("TZ", "UTC", 1);
  tzset(); // added suggestion from chatGPT
  updateDisplayState();
}
// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  esp_task_wdt_reset();
  static unsigned long lastSoilRead = 0;
  static unsigned long lastStatusDisplay = 0;
  static unsigned long lastDataLog = 0;
  static unsigned long lastOLEDUpdate = 0;
  static bool initialReadDone = false;
  updateGPS();
  checkSoilSensorQueue();
  // Below line is added for non freez of BLE transfer
  handleBleCommands();
  
  // Handle BLE file transfer (non-blocking)
  if (transferInProgress || transferPending) {
    processTransferChunk();
  }
  
  // Handle state transitions - ONLY if not transferring files
  if (!transferInProgress && !transferPending) {
    unsigned long currentTime = millis();
    
    switch(currentState) {
      case STATE_INITIAL:
        if (currentTime - stateStartTime >= 3000) {
          changeState(STATE_COMPONENT_CHECK);
        }
        break;
        
      case STATE_COMPONENT_CHECK:
        if (currentTime - stateStartTime >= 3000) {
          changeState(STATE_PLACE_SENSOR);
        }
        break;
        
      case STATE_PLACE_SENSOR:
        if (currentTime - stateStartTime >= 5000) {
          changeState(STATE_ANALYZING);
        }
        break;
        
      case STATE_ANALYZING:
        if (currentTime - stateStartTime >= DATA_LOG_INTERVAL) { //45000
          logDataToSD();
        }
        break;
        
      case STATE_FILE_CREATED:
        if (currentTime - stateStartTime >= 3000) {
          changeState(STATE_PLACE_SENSOR);
        }
        break;
        
      case STATE_BLE_TRANSFER:
        break;
    }
  }
  // Update display
  if(millis() - lastOLEDUpdate >= 500) {
    updateDisplayState();
    lastOLEDUpdate = millis();
  }
  
  // System status display
  if(millis() - lastStatusDisplay >= 10000) {
    if (!transferInProgress || (millis() - lastStatusDisplay >= 30000)) {
      printSystemStatus();
      lastStatusDisplay = millis();
    }
  }
  // Auto BLE transfer and health monitoring
  autoStartTransfer();
  monitorSystemHealth();
  
  delay(10);
}