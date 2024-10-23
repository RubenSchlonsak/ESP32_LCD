#include <Arduino.h>
#include "LCD_Test.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE Service und Characteristics UUIDs
#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define CHAR_UUID "87654321-4321-4321-4321-210987654321"

// Variable für Sensordaten
float acc[3], gyro[3];
UWORD *BlackImage = NULL;
BLECharacteristic *sensorCharacteristic;

// BLE Server Callback
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE Client connected.");
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("BLE Client disconnected.");
    }
};

void setup() {
    Serial.begin(115200);

    // PSRAM Initialisierung
    if(psramInit()){
      Serial.println("\nPSRAM is correctly initialized");
    } else {
      Serial.println("PSRAM not available");
    }

    // Speicher für das Display
    if ((BlackImage = (UWORD *)ps_malloc(LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2)) == NULL){
        Serial.println("Failed to allocate memory...");
        exit(0);
    }

    // LCD Initialisierung
    if (DEV_Module_Init() != 0) {
      Serial.println("GPIO Init Fail!");
    } else {
      Serial.println("GPIO Init successful!");
      LCD_1IN28_Init(HORIZONTAL);
      LCD_1IN28_Clear(WHITE);
      Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
      Paint_SetScale(65);
    }

    // BLE Initialisierung
    BLEDevice::init("ESP32_Sensor_Device");
    BLEServer *bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new MyServerCallbacks());

    // BLE Service
    BLEService *bleService = bleServer->createService(SERVICE_UUID);

    // Sensor characteristic zum Senden von Daten
    sensorCharacteristic = bleService->createCharacteristic(
        CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    sensorCharacteristic->addDescriptor(new BLE2902());

    // Starte den BLE Service
    bleService->start();
    bleServer->getAdvertising()->start();
    Serial.println("Waiting for BLE client connection...");

    // Hier kannst du deine Display-Logik fortsetzen oder erweitern.
}

void loop() {
    // Beispielhafte Sensordaten-Erfassung (hier kannst du deine IMU-Daten einsetzen)
    acc[0] = random(100) / 10.0;
    acc[1] = random(100) / 10.0;
    acc[2] = random(100) / 10.0;

    // Konvertiere die Sensordaten in ein String-Format, das über BLE gesendet werden kann
    char sensorData[50];
    snprintf(sensorData, sizeof(sensorData), "Acc: %.2f, %.2f, %.2f", acc[0], acc[1], acc[2]);

    // Setze die Sensordaten und sende sie über BLE
    sensorCharacteristic->setValue(sensorData);
    sensorCharacteristic->notify(); // Notify für das Senden der Daten an verbundene BLE Clients

    // Ein Delay zwischen den Datenübertragungen
    delay(1000);
}
