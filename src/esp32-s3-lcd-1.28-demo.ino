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

// Display control (adjust pin if needed)
#define DISPLAY_BL_PIN      0
bool displayEnabled = true;

// Sampling control (default 100Hz = 10ms interval)
volatile uint32_t samplingInterval = 10;  // ms

BLEServer *pServer = nullptr;
BLECharacteristic *pDataCharacteristic = nullptr;
BLECharacteristic *pControlCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
hw_timer_t *samplingTimer = nullptr;

// Use a 32‑bit type to avoid overflow on the image buffer
uint32_t ImageSize = (uint32_t)LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
uint16_t *BlackImage = nullptr;

// Forward declaration
void updateDisplay();

class ControlCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        std::string value = pChar->getValue();
        if (value.length() == 0) return;

        String command = String(value.c_str());
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
            ::updateDisplay();               // refresh display text
            Serial.println("Display turned OFF");
        }
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        displayEnabled = true;
        digitalWrite(DISPLAY_BL_PIN, HIGH);
        ::updateDisplay();
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        displayEnabled = false;
        digitalWrite(DISPLAY_BL_PIN, LOW);
        pServer->startAdvertising();
    }
};

void IRAM_ATTR onTimer() {
    // ISR stub: do minimal work here if needed
}

void updateDisplay() {
    if (!displayEnabled) return;
    Paint_Clear(WHITE);
    if (deviceConnected) {
        Paint_DrawString_EN(60, 100, "CONNECTED", &Font24, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(60, 100, "READY", &Font24, BLACK, WHITE);
    }
    LCD_1IN28_Display(BlackImage);
}

void setup() {
    Serial.begin(115200);

    // Initialize display backlight
    pinMode(DISPLAY_BL_PIN, OUTPUT);
    digitalWrite(DISPLAY_BL_PIN, HIGH);

    // Initialize display hardware
    if (psramInit()) Serial.println("PSRAM initialized");
    BlackImage = (uint16_t *)ps_malloc(ImageSize);
    if (!BlackImage) {
        Serial.println("PSRAM allocation failed!");
        while (1);
    }
    DEV_Module_Init();
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(WHITE);
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_SetRotate(ROTATE_0);

    // Initial display
    updateDisplay();

    // BLE Initialization
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
    QMI8658_init();

    // Setup hardware timer for sampling
    samplingTimer = timerBegin(0, 80, true);  // 1 MHz base
    timerAttachInterrupt(samplingTimer, &onTimer, true);
    timerAlarmWrite(samplingTimer, samplingInterval * 1000, true);
    timerAlarmEnable(samplingTimer);
}

void loop() {
    static uint32_t lastUpdate = 0;
    static uint32_t frameCount = 0;

    // Handle BLE connection state transitions
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // Simple delay-based sampling
    delay(samplingInterval);

    // Read IMU data
    float acc[3], gyro[3];
    unsigned int tim_count;
    QMI8658_read_xyz(acc, gyro, &tim_count);

    // Build JSON payload
    StaticJsonDocument<256> doc;
    JsonObject root = doc.to<JsonObject>();
    root["frame"] = frameCount++;
    root["timestamp"] = micros();
    JsonArray accArr = root.createNestedArray("accel");
    JsonArray gyroArr = root.createNestedArray("gyro");
    for (int i = 0; i < 3; i++) {
        accArr.add(acc[i]);
        gyroArr.add(gyro[i]);
    }

    if (deviceConnected) {
        char payload[256];
        size_t len = serializeJson(doc, payload);
        pDataCharacteristic->setValue((uint8_t*)payload, len);
        pDataCharacteristic->notify();

        // Update display at ~5 Hz
        if (millis() - lastUpdate > 200) {
            updateDisplay();
            lastUpdate = millis();
        }
    }
}
