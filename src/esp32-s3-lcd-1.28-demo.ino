#include <Arduino.h>
#include "LCD_Test.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h> 
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

// Server callback to track connection state
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
        pServer->startAdvertising(); // Restart advertising
    }
};

void bleTask(void *pvParameters) {
    // Initialisierung kann hier erfolgen, falls notwendig

    // JSON-Dokument vorbereiten
    const size_t capacity = JSON_OBJECT_SIZE(5) + 100;
    StaticJsonDocument<capacity> doc;

    while (true) {
        if (deviceConnected) {
            // Erfasse den aktuellen Zeitstempel in Mikrosekunden seit Programmstart
            unsigned long timestamp = micros();

            // Fehlerüberprüfung der Sensordaten
            const char* status = "OK";
            bool dataError = false;
            for (int i = 0; i < 3; i++) {
                if (isnan(acc[i]) || isnan(gyro[i])) {
                    dataError = true;
                    break;
                }
            }
            if (dataError) {
                status = "ERROR";
            }

            // JSON-Daten strukturieren
            doc["timestamp"] = timestamp; // Mikrosekunden
            doc["status"] = status;

            // Beschleunigungsdaten hinzufügen
            JsonArray accArray = doc.createNestedArray("acceleration");
            accArray.add(acc[0]);
            accArray.add(acc[1]);
            accArray.add(acc[2]);

            // Gyroskopdaten hinzufügen
            JsonArray gyroArray = doc.createNestedArray("gyroscope");
            gyroArray.add(gyro[0]);
            gyroArray.add(gyro[1]);
            gyroArray.add(gyro[2]);

            // JSON in String umwandeln
            char dataStr[256];
            size_t n = serializeJson(doc, dataStr);

            // BLE-Daten senden
            pCharacteristic->setValue((uint8_t*)dataStr, n);
            pCharacteristic->notify();

            // JSON-Dokument zurücksetzen für den nächsten Durchlauf
            doc.clear();
        }

        // Anpassung der Verzögerung für höhere Sampling-Rate (ca. 100 Hz)
        vTaskDelay(10 / portTICK_PERIOD_MS); // 10 ms Verzögerung
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting setup...");

    // Set log level to ERROR to reduce verbosity
    esp_log_level_set("*", ESP_LOG_ERROR);

    // Initialize BLE
    BLEDevice::init("ESP32-BLE-AccelGyro");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );
    pCharacteristic->setValue("AccelGyro Data");
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->start();
    Serial.println("BLE advertising started");

    // Initialize PSRAM
    if(psramInit()){
        Serial.println("PSRAM is correctly initialized");
    } else {
        Serial.println("PSRAM not available");
    }

    BlackImage = (UWORD *)ps_malloc(Imagesize);
    if (BlackImage == NULL){
        Serial.println("Failed to allocate memory for BlackImage");
        while (true) { delay(1000); } // Halt
    }

    // Initialize display
    if (DEV_Module_Init() != 0) {
        Serial.println("GPIO Init Fail!");
        while (true) { delay(1000); } // Halt
    } else {
        Serial.println("GPIO Init successful!");
    }

    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(WHITE);
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);

    // Initial drawing (optional)
    // ...

    LCD_1IN28_Display(BlackImage);
    DEV_Delay_ms(1000);

    // Initialize sensors
    QMI8658_init();
    Serial.println("QMI8658 initialized");

    // Initial display setup
    Paint_Clear(WHITE);
    Paint_DrawRectangle(0, 00, 240, 47, 0XF410, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 47, 240, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 120, 240, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 195, 240, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);

    Paint_DrawString_EN(45, 30, "LongPress Quit", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(45, 50, "ACC_X = ", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(45, 75, "ACC_Y = ", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(45, 100, "ACC_Z = ", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(45, 125, "GYR_X = ", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(45, 150, "GYR_Y = ", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(45, 175, "GYR_Z = ", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(45, 200, "BAT(V)=", &Font16, WHITE, BLACK);
    LCD_1IN28_Display(BlackImage);

    // Create BLE task
    xTaskCreate(bleTask, "BLE Task", 4096, NULL, 1, NULL);

    Serial.println("Setup complete");
}

void loop()
{
    // Handle sensor reading and display updates here or in another FreeRTOS task
    result = DEC_ADC_Read();
    QMI8658_read_xyz(acc, gyro, &tim_count);

    const float conversion_factor = 3.3f / (1 << 12) * 3;

    // Update display
    Paint_Clear(WHITE);
    Paint_DrawRectangle(120, 47, 220, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(120, 120, 220, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(120, 195, 220, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawNum(120, 50, acc[0], &Font16, 2, BLACK, WHITE);
    Paint_DrawNum(120, 75, acc[1], &Font16, 2, BLACK, WHITE);
    Paint_DrawNum(120, 100, acc[2], &Font16, 2, BLACK, WHITE);
    Paint_DrawNum(120, 125, gyro[0], &Font16, 2, BLACK, WHITE);
    Paint_DrawNum(120, 150, gyro[1], &Font16, 2, BLACK, WHITE);
    Paint_DrawNum(120, 175, gyro[2], &Font16, 2, BLACK, WHITE);
    Paint_DrawNum(130, 200, result * conversion_factor, &Font16, 2, BLACK, WHITE);
    LCD_1IN28_DisplayWindows(120, 50, 210, 200, BlackImage);
    LCD_1IN28_DisplayWindows(130, 200, 220, 220, BlackImage);

    // Optional: Monitor heap
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());

    delay(100); // 100ms delay
}
