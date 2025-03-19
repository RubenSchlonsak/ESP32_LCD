#include <Arduino.h>
#include "LCD_Test.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include <BLE2902.h>

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
bool previousConnectionState = false;

#define BATCH_SIZE 1

const int touchPins[] = {2, 3, 4, 5, 13, 14};
const int numTouchPins = sizeof(touchPins) / sizeof(touchPins[0]);
int touchValues[numTouchPins];

const char* touchPinNames[] = {"toe", "ball_inside", "ball_middle", "heel", "arch", "ball_outside"};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("🔵 Device connected!");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("🔴 Device disconnected!");
    delay(1000);
    pServer->startAdvertising();
  }
};

void updateDisplayStatus() {
  Paint_Clear(WHITE);
  if (deviceConnected) {
    Paint_DrawString_EN(60, 100, "CONNECTED", &Font24, BLACK, WHITE);
  } else {
    Paint_DrawString_EN(60, 100, "READY", &Font24, BLACK, WHITE);
  }
  LCD_1IN28_Display(BlackImage);
}

void bleTask(void *pvParameters) {
  StaticJsonDocument<2048> doc;
  char dataStr[512];
  static uint32_t frame_id = 0;

  while (true) {
    if (previousConnectionState != deviceConnected) {
      updateDisplayStatus();
      previousConnectionState = deviceConnected;
    }

    if (deviceConnected) {
      JsonObject root = doc.to<JsonObject>();
      JsonArray batch = root.createNestedArray("batch");

      JsonObject measurement = batch.createNestedObject();
      measurement["f"] = frame_id++;
      measurement["timestamp"] = micros();

      bool dataError = false;
      for (int j = 0; j < 3; j++) {
        if (isnan(acc[j]) || isnan(gyro[j])) {
          dataError = true;
          break;
        }
      }
      measurement["status"] = dataError ? "ERROR" : "OK";

      JsonArray accArray = measurement.createNestedArray("acceleration");
      for (int j = 0; j < 3; j++) accArray.add(acc[j]);

      JsonArray gyroArray = measurement.createNestedArray("gyroscope");
      for (int j = 0; j < 3; j++) gyroArray.add(gyro[j]);

      JsonObject touchData = measurement.createNestedObject("touch");
      for (int t = 0; t < numTouchPins; t++) {
        touchValues[t] = touchRead(touchPins[t]);
        delayMicroseconds(10);
        touchData[touchPinNames[t]] = touchValues[t];
      }

      size_t n = serializeJson(doc, dataStr, sizeof(dataStr));
      pCharacteristic->setValue((uint8_t*)dataStr, n);
      pCharacteristic->notify();

      Serial.print("📤 Sending data: ");
      Serial.println(dataStr);

      doc.clear();
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    else {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < numTouchPins; i++) pinMode(touchPins[i], INPUT);

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

  BLE2902* desc = new BLE2902();
  desc->setNotifications(true);
  pCharacteristic->addDescriptor(desc);
  pCharacteristic->setValue("AccelGyro Data");
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("📡 BLE Advertising started");

  if (psramInit()) Serial.println("PSRAM initialized");

  BlackImage = (UWORD *)ps_malloc(Imagesize);
  if (!BlackImage) {
    Serial.println("❌ PSRAM allocation failed!");
    while (1);
  }

  DEV_Module_Init();
  LCD_1IN28_Init(HORIZONTAL);
  LCD_1IN28_Clear(WHITE);
  Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
  Paint_SetScale(65);
  Paint_SetRotate(ROTATE_0);

  updateDisplayStatus();
  QMI8658_init();
  xTaskCreate(bleTask, "BLE Task", 8192, NULL, 1, NULL);
}

void loop() {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  result = DEC_ADC_Read();
  QMI8658_read_xyz(acc, gyro, &tim_count);
  vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);
}