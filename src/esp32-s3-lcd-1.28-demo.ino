#include <Arduino.h>
#include "LCD_Test.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include <BLE2902.h>

// Define constants and global variables
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "abcdef12-3456-789a-bcde-123456789abc"

UWORD Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
UWORD *BlackImage = NULL;
float acc[3], gyro[3];
unsigned int tim_count = 0;
uint16_t result;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

#define BATCH_SIZE 10  // Anzahl der Messwerte pro Batch

// Server callback to track connection state
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("Device disconnected");
    pServer->startAdvertising(); // Restart advertising
  }
};

void bleTask(void *pvParameters) {
  // Wähle eine ausreichende Kapazität für das JSON-Dokument. 
  // Je nach Batchgröße muss dieser Wert ggf. angepasst werden.
  StaticJsonDocument<1024> doc;
  char dataStr[512];
  static uint32_t frame_id = 0;
  
  while (true) {
    if (deviceConnected) {
      // Erstelle ein JSON-Objekt mit einem Array "batch", in dem mehrere Messungen gesammelt werden
      JsonObject root = doc.to<JsonObject>();
      JsonArray batch = root.createNestedArray("batch");

      // Batch sammeln: Bei BATCH_SIZE Messungen, jeweils mit 10ms Verzögerung (insgesamt ca. 100ms)
      for (int i = 0; i < BATCH_SIZE; i++) {
        JsonObject measurement = batch.createNestedObject();
        measurement["f"] = frame_id++;
        measurement["timestamp"] = micros();

        const char* status = "OK";
        bool dataError = false;
        for (int j = 0; j < 3; j++) {
          if (isnan(acc[j]) || isnan(gyro[j])) {
            dataError = true;
            break;
          }
        }
        if (dataError) {
          status = "ERROR";
        }
        measurement["status"] = status;

        JsonArray accArray = measurement.createNestedArray("acceleration");
        accArray.add(acc[0]);
        accArray.add(acc[1]);
        accArray.add(acc[2]);

        JsonArray gyroArray = measurement.createNestedArray("gyroscope");
        gyroArray.add(gyro[0]);
        gyroArray.add(gyro[1]);
        gyroArray.add(gyro[2]);

        // Warte 10ms bis zur nächsten Messung
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      // Serialisiere das gesammelte JSON-Dokument in einen String
      size_t n = serializeJson(doc, dataStr, sizeof(dataStr));
      // Sende den Batch per BLE-Notification
      pCharacteristic->setValue((uint8_t*)dataStr, n);
      pCharacteristic->notify();

      // Leere das JSON-Dokument für den nächsten Batch
      doc.clear();
    }
    else {
      // Falls kein Gerät verbunden ist, warte etwas länger
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");
  esp_log_level_set("*", ESP_LOG_ERROR);

  // BLE initialisieren
  BLEDevice::init("ESP32-BLE-AccelGyro");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ  |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY 
                    );
  pCharacteristic->setValue("AccelGyro Data");
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE advertising started");

  // PSRAM initialisieren
  if (psramInit()) {
    Serial.println("PSRAM is correctly initialized");
  } else {
    Serial.println("PSRAM not available");
  }
  BlackImage = (UWORD *)ps_malloc(Imagesize);
  if (BlackImage == NULL) {
    Serial.println("Failed to allocate memory for BlackImage");
    while (true) { delay(1000); }
  }
  
  // Display initialisieren
  if (DEV_Module_Init() != 0) {
    Serial.println("GPIO Init Fail!");
    while (true) { delay(1000); }
  } else {
    Serial.println("GPIO Init successful!");
  }
  LCD_1IN28_Init(HORIZONTAL);
  LCD_1IN28_Clear(WHITE);
  Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
  Paint_SetScale(65);
  Paint_SetRotate(ROTATE_0);
  Paint_Clear(WHITE);
  
  // Nur eine einfache Anzeige "Device ON"
  Paint_DrawString_EN(60, 100, "Device ON", &Font24, BLACK, WHITE);
  LCD_1IN28_Display(BlackImage);
  
  // Sensoren initialisieren
  QMI8658_init();
  Serial.println("QMI8658 initialized");

  // BLE-Task erstellen
  xTaskCreate(bleTask, "BLE Task", 4096, NULL, 1, NULL);
  
  Serial.println("Setup complete");
}

void loop() {
  // Sensor-Daten lesen und die Zeitbasis aktualisieren
  TickType_t xLastWakeTime = xTaskGetTickCount();
  result = DEC_ADC_Read();
  QMI8658_read_xyz(acc, gyro, &tim_count);
  
  // Keine Display-Aktualisierung, um 100Hz Sensordaten zu gewährleisten
  vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);
}
