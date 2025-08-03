#include <Arduino.h>
#include "LCD_Test.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include <BLE2902.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define DATA_CHAR_UUID      "abcdef12-3456-789a-bcde-123456789abc"
#define CONTROL_CHAR_UUID   "12345678-1234-1234-1234-123456789013"

// Hardware-spezifische Pin-Definitionen (Waveshare ESP32-S3-LCD-1.28)
#define BATTERY_ADC_PIN     1     // GPIO1 - Batteriespannungs-ADC
#define DISPLAY_BL_PIN      40    // GPIO40 - LCD Backlight (corrected from 0)
#define BOOT_PIN           0     // GPIO0 - Boot-Button

// Sampling control (default 100Hz = 10ms interval)
volatile uint32_t samplingInterval = 10;  // ms
bool displayEnabled = true;

// Batterie-Kalibrierung (Waveshare spezifisch)
// Spannungsteiler: 200K + 100K = Faktor 3.0
// Laut Waveshare: Voltage = 3.3 / (1<<12) * 3 * ADC_Value
const float BATTERY_MAX_VOLTAGE = 4.2;   // Vollgeladene Li-Ion Batterie
const float BATTERY_MIN_VOLTAGE = 3.0;   // "Leere" Batterie
const float ADC_VOLTAGE_FACTOR = 3.3 / 4096.0 * 3.0; // Waveshare Formel

// Batterie-Status Variablen
float batteryVoltage = 0.0;
int batteryPercentage = 0;
bool batteryCharging = false;
unsigned long lastBatteryCheck = 0;
bool batteryAutoUpdate = false;
unsigned long lastDisplayUpdate = 0;

BLEServer *pServer = nullptr;
BLECharacteristic *pDataCharacteristic = nullptr;
BLECharacteristic *pControlCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
hw_timer_t *samplingTimer = nullptr;

// Use a 32‑bit type to avoid overflow on the image buffer
uint32_t ImageSize = (uint32_t)LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
uint16_t *BlackImage = nullptr;

// Forward declarations
void updateDisplay();
void updateBatteryStatus();
void sendBatteryStatus();
void drawBatteryIcon(int x, int y);
void checkLowBattery();

class ControlCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        std::string value = pChar->getValue();
        if (value.length() == 0) return;

        String command = String(value.c_str());
        Serial.println("Received command: " + command);
        
        if (command.startsWith("RATE:")) {
            uint32_t newRate = command.substring(5).toInt();
            if (newRate > 0 && newRate <= 500) {
                samplingInterval = 1000 / newRate;
                timerAlarmWrite(samplingTimer, samplingInterval * 1000, true);
                Serial.printf("Sampling rate set to: %u Hz (interval: %u ms)\n", newRate, samplingInterval);
            }
        }
        else if (command == "DISPLAY:ON") {
            displayEnabled = true;
            digitalWrite(DISPLAY_BL_PIN, HIGH);
            ::updateDisplay();
            Serial.println("Display turned ON");
        }
        else if (command == "DISPLAY:OFF") {
            displayEnabled = false;
            digitalWrite(DISPLAY_BL_PIN, LOW);
            Serial.println("Display turned OFF");
        }
        else if (command == "BATTERY:REQUEST") {
            updateBatteryStatus();
            sendBatteryStatus();
            Serial.println("Battery status requested");
        }
        else if (command == "BATTERY_AUTO:ON") {
            batteryAutoUpdate = true;
            lastBatteryCheck = millis();
            Serial.println("Battery auto-update: ON");
        }
        else if (command == "BATTERY_AUTO:OFF") {
            batteryAutoUpdate = false;
            Serial.println("Battery auto-update: OFF");
        }
        else {
            Serial.println("Unknown command: " + command);
        }
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        displayEnabled = true;
        digitalWrite(DISPLAY_BL_PIN, HIGH);
        Serial.println("BLE Client connected");
        ::updateDisplay();
        
        // Send initial battery status
        updateBatteryStatus();
        sendBatteryStatus();
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("BLE Client disconnected");
        pServer->startAdvertising();
        ::updateDisplay();
    }
};

void IRAM_ATTR onTimer() {
    // ISR stub: do minimal work here if needed
}

void updateBatteryStatus() {
    // Mehrere ADC-Messungen für Stabilität
    long adcSum = 0;
    const int numReadings = 20;
    
    for (int i = 0; i < numReadings; i++) {
        adcSum += analogRead(BATTERY_ADC_PIN);
        delay(5);
    }
    
    int adcValue = adcSum / numReadings;
    
    // Spannung berechnen (Waveshare Formel)
    batteryVoltage = adcValue * ADC_VOLTAGE_FACTOR;
    
    // Prozentsatz berechnen
    if (batteryVoltage >= BATTERY_MAX_VOLTAGE) {
        batteryPercentage = 100;
    } else if (batteryVoltage <= BATTERY_MIN_VOLTAGE) {
        batteryPercentage = 0;
    } else {
        batteryPercentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                           (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100;
    }
    
    // Ladestatus heuristisch bestimmen
    static float lastVoltage = batteryVoltage;
    static int voltageReadings = 0;
    static float voltageSum = 0;
    
    voltageSum += batteryVoltage - lastVoltage;
    voltageReadings++;
    
    if (voltageReadings >= 5) {
        float averageChange = voltageSum / voltageReadings;
        batteryCharging = (averageChange > 0.01); // Spannung steigt
        voltageSum = 0;
        voltageReadings = 0;
    }
    
    lastVoltage = batteryVoltage;
    
    Serial.printf("Battery: %.2fV (%d%%) %s\n", 
                  batteryVoltage, batteryPercentage, 
                  batteryCharging ? "Charging" : "Discharging");
}

void sendBatteryStatus() {
    if (!deviceConnected) return;
    
    StaticJsonDocument<200> doc;
    doc["type"] = "battery_status";
    doc["voltage"] = round(batteryVoltage * 100) / 100.0;
    doc["percentage"] = batteryPercentage;
    doc["charging"] = batteryCharging;
    doc["timestamp"] = millis();
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Via BLE senden
    pDataCharacteristic->setValue(jsonString.c_str());
    pDataCharacteristic->notify();
    
    Serial.println("Battery status sent: " + jsonString);
    
    // Warnung bei niedrigem Batteriestand
    if (batteryPercentage <= 15) {
        checkLowBattery();
    }
}

void drawBatteryIcon(int x, int y) {
    const int width = 24;
    const int height = 12;
    
    // Batterie-Rahmen
    Paint_DrawRectangle(x, y, x + width, y + height, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(x + width, y + 3, x + width + 2, y + height - 3, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    
    // Füllstand basierend auf Prozentsatz
    int fillWidth = (width - 2) * batteryPercentage / 100;
    
    if (fillWidth > 0) {
        Paint_DrawRectangle(x + 1, y + 1, x + 1 + fillWidth, y + height - 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    
    // Blitz-Symbol für Ladung (vereinfacht)
    if (batteryCharging) {
        int flashX = x + width/2 - 1;
        int flashY = y + 3;
        Paint_DrawLine(flashX, flashY, flashX + 2, flashY + 6, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }
}

void updateDisplay() {
    if (!displayEnabled) return;
    
    Paint_Clear(WHITE);
    
    // Titel
    Paint_DrawString_EN(50, 10, "ESP32-S3", &Font20, BLACK, WHITE);
    Paint_DrawString_EN(60, 35, "IMU Monitor", &Font16, BLACK, WHITE);
    
    // Verbindungsstatus
    if (deviceConnected) {
        Paint_DrawString_EN(70, 60, "CONNECTED", &Font16, BLACK, WHITE);
        Paint_DrawCircle(220, 15, 5, GREEN, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    } else {
        Paint_DrawString_EN(85, 60, "READY", &Font16, BLACK, WHITE);
        Paint_DrawCircle(220, 15, 5, RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    
    // Batterie-Symbol und Info
    drawBatteryIcon(10, 90);
    
    // Batterie-Spannungsanzeige
    char voltageStr[10];
    sprintf(voltageStr, "%.2fV", batteryVoltage);
    Paint_DrawString_EN(45, 90, voltageStr, &Font16, BLACK, WHITE);
    
    // Batterie-Prozentsatz
    char percentStr[10];
    sprintf(percentStr, "%d%%", batteryPercentage);
    Paint_DrawString_EN(45, 110, percentStr, &Font16, BLACK, WHITE);
    
    // Ladestatus
    if (batteryCharging) {
        Paint_DrawString_EN(120, 95, "Charging", &Font12, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(120, 95, "Battery", &Font12, BLACK, WHITE);
    }
    
    // Sampling Rate Info
    char rateStr[20];
    sprintf(rateStr, "%dHz", 1000/samplingInterval);
    Paint_DrawString_EN(10, 130, "Rate:", &Font12, BLACK, WHITE);
    Paint_DrawString_EN(60, 130, rateStr, &Font12, BLACK, WHITE);
    
    // Uptime
    unsigned long uptime = millis() / 1000;
    char uptimeStr[20];
    sprintf(uptimeStr, "Up: %lus", uptime);
    Paint_DrawString_EN(120, 130, uptimeStr, &Font12, BLACK, WHITE);
    
    // IMU Status (vereinfacht)
    Paint_DrawString_EN(10, 150, "IMU: Active", &Font12, BLACK, WHITE);
    
    // Warnung bei niedrigem Batteriestand
    if (batteryPercentage <= 15) {
        Paint_DrawString_EN(60, 170, "LOW BATTERY!", &Font16, RED, WHITE);
    }
    
    // Frame auf Display ausgeben
    LCD_1IN28_Display(BlackImage);
    
    lastDisplayUpdate = millis();
}

void checkLowBattery() {
    static unsigned long lastWarning = 0;
    
    if (batteryPercentage <= 15 && millis() - lastWarning > 300000) { // Alle 5 Minuten
        // Warnung senden
        if (deviceConnected) {
            StaticJsonDocument<100> doc;
            doc["type"] = "warning";
            doc["message"] = "Low battery: " + String(batteryPercentage) + "%";
            
            String jsonString;
            serializeJson(doc, jsonString);
            
            pDataCharacteristic->setValue(jsonString.c_str());
            pDataCharacteristic->notify();
        }
        
        // Display-Warnung (blinken)
        if (displayEnabled) {
            Paint_Clear(RED);
            Paint_DrawString_EN(40, 100, "LOW", &Font24, WHITE, RED);
            Paint_DrawString_EN(30, 130, "BATTERY", &Font24, WHITE, RED);
            LCD_1IN28_Display(BlackImage);
            delay(1000);
            updateDisplay();
        }
        
        lastWarning = millis();
        Serial.println("LOW BATTERY WARNING sent");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32-S3-LCD-1.28 IMU with Battery Monitoring starting...");

    // Initialize display backlight (GPIO40 für Waveshare Board)
    pinMode(DISPLAY_BL_PIN, OUTPUT);
    digitalWrite(DISPLAY_BL_PIN, HIGH);
    
    // Initialize Boot button
    pinMode(BOOT_PIN, INPUT_PULLUP);
    
    // Initialize Battery ADC
    pinMode(BATTERY_ADC_PIN, INPUT);

    // Initialize display hardware
    if (psramInit()) {
        Serial.println("PSRAM initialized successfully");
    } else {
        Serial.println("PSRAM initialization failed!");
    }
    
    BlackImage = (uint16_t *)ps_malloc(ImageSize);
    if (!BlackImage) {
        Serial.println("PSRAM allocation failed!");
        while (1) delay(1000);
    }
    
    Serial.println("Initializing LCD...");
    DEV_Module_Init();
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(WHITE);
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_SetRotate(ROTATE_0);

    // Splash Screen
    Paint_Clear(BLACK);
    Paint_DrawString_EN(60, 80, "ESP32-S3", &Font24, WHITE, BLACK);
    Paint_DrawString_EN(90, 110, "IMU", &Font24, WHITE, BLACK);
    Paint_DrawString_EN(85, 140, "v1.0", &Font16, WHITE, BLACK);
    LCD_1IN28_Display(BlackImage);
    delay(2000);

    // Initial battery reading
    updateBatteryStatus();
    updateDisplay();

    // BLE Initialization
    Serial.println("Initializing BLE...");
    BLEDevice::init("ESP32-IMU");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pDataCharacteristic = pService->createCharacteristic(
        DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pDataCharacteristic->addDescriptor(new BLE2902());

    pControlCharacteristic = pService->createCharacteristic(
        CONTROL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pControlCharacteristic->setCallbacks(new ControlCallbacks());

    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();
    Serial.println("BLE Advertising started");

    // IMU initialization
    Serial.println("Initializing QMI8658 IMU...");
    QMI8658_init();

    // Setup hardware timer for sampling
    samplingTimer = timerBegin(0, 80, true);  // 1 MHz base
    timerAttachInterrupt(samplingTimer, &onTimer, true);
    timerAlarmWrite(samplingTimer, samplingInterval * 1000, true);
    timerAlarmEnable(samplingTimer);
    
    Serial.println("System initialized successfully!");
    Serial.printf("Sampling rate: %d Hz\n", 1000/samplingInterval);
    Serial.printf("Battery: %.2fV (%d%%)\n", batteryVoltage, batteryPercentage);
}

void loop() {
    static uint32_t lastUpdate = 0;
    static uint32_t frameCount = 0;
    static uint32_t lastHeartbeat = 0;
    static bool lastBootState = HIGH;

    // Handle BLE connection state transitions
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // Boot button handling (toggle display)
    bool bootState = digitalRead(BOOT_PIN);
    if (lastBootState == HIGH && bootState == LOW) {
        displayEnabled = !displayEnabled;
        digitalWrite(DISPLAY_BL_PIN, displayEnabled ? HIGH : LOW);
        if (displayEnabled) {
            updateDisplay();
        }
        Serial.println(displayEnabled ? "Display enabled by button" : "Display disabled by button");
        delay(200); // Debounce
    }
    lastBootState = bootState;

    // Automatic battery updates (every 60 seconds)
    if (batteryAutoUpdate && (millis() - lastBatteryCheck > 60000)) {
        updateBatteryStatus();
        sendBatteryStatus();
        lastBatteryCheck = millis();
    }

    // Simple delay-based IMU sampling
    delay(samplingInterval);

    // Read IMU data
    float acc[3], gyro[3];
    unsigned int tim_count;
    QMI8658_read_xyz(acc, gyro, &tim_count);

    // Build JSON payload for IMU data
    StaticJsonDocument<300> doc;
    doc["type"] = "imu_data";  // Added type field for new protocol
    doc["frame"] = frameCount++;
    doc["timestamp"] = millis();
    
    JsonArray accArr = doc.createNestedArray("accel");
    JsonArray gyroArr = doc.createNestedArray("gyro");
    for (int i = 0; i < 3; i++) {
        accArr.add(acc[i]);
        gyroArr.add(gyro[i]);
    }

    // Send IMU data via BLE
    if (deviceConnected) {
        String jsonString;
        serializeJson(doc, jsonString);
        pDataCharacteristic->setValue(jsonString.c_str());
        pDataCharacteristic->notify();

        // Update display at ~1 Hz
        if (millis() - lastUpdate > 1000) {
            updateDisplay();
            lastUpdate = millis();
        }
    }

    // Heartbeat every 5 seconds
    if (millis() - lastHeartbeat > 5000) {
        Serial.printf("Heartbeat - Frames: %u, Battery: %.2fV (%d%%), Connected: %s\n", 
                     frameCount, batteryVoltage, batteryPercentage, 
                     deviceConnected ? "Yes" : "No");
        lastHeartbeat = millis();
    }

    // Check for low battery warning
    checkLowBattery();
}