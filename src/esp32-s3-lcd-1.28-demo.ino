#include <Arduino.h>
#include "LCD_Test.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Wire.h>

// ===== UUIDs =====
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define DATA_CHAR_UUID      "abcdef12-3456-789a-bcde-123456789abc"
#define CONTROL_CHAR_UUID   "12345678-1234-1234-1234-123456789013"

// ===== Pins (Waveshare ESP32-S3-LCD-1.28) =====
#define BATTERY_ADC_PIN     1
#define DISPLAY_BL_PIN      40
#define BOOT_PIN            0

// ===== Battery calib =====
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.0;
const float ADC_VOLTAGE_FACTOR  = 3.3 / 4096.0 * 3.0;

// ===== Sampling / transport =====
volatile uint32_t samplingIntervalMs = 10; // 100 Hz default
// Transport format v2 (compact int16), magic 0xB0B0
static const uint16_t MAGIC_V2 = 0xB0B0;
static const uint8_t  VER_V2   = 2;
// legacy float batch (kept for compatibility, magic 0xBEEF)
static const uint16_t MAGIC_V1 = 0xBEEF;
static const uint8_t  VER_V1   = 1;

// negotiated payload capacity (ATT_MTU - 3). Default safe=20 bytes if GUI doesn't tell us.
volatile uint16_t g_maxPayload = 20;

// per-sample packed v2 size: dt_ms(u16) + 6*int16 = 14 bytes
static const uint8_t V2_REC_SIZE = 14;

// Ring buffer of float samples; quantized at send time
struct Sample {
  uint32_t t_ms;
  float ax, ay, az;
  float gx, gy, gz;
};
static const size_t RING_N = 4096;
volatile size_t rb_wr = 0, rb_rd = 0;
Sample ringbuf[RING_N];

// Display / state
bool displayEnabled = true;
bool deviceConnected = false;
bool oldDeviceConnected = false;

float batteryVoltage = 0.0f;
int   batteryPercentage = 0;
bool  batteryCharging = false;
unsigned long lastBatteryCheck = 0;
bool batteryAutoUpdate = true;

const unsigned long BATTERY_UPDATE_INTERVAL_CONNECTED = 60000;
const unsigned long BATTERY_UPDATE_INTERVAL_READY     = 30000;
const unsigned long DISPLAY_UPDATE_INTERVAL_CONNECTED = 1000;
const unsigned long DISPLAY_UPDATE_INTERVAL_READY     = 5000;

unsigned long lastDisplayUpdate = 0;

uint32_t ImageSize = (uint32_t)LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
uint16_t *BlackImage = nullptr;

BLEServer *pServer = nullptr;
BLECharacteristic *pDataCharacteristic = nullptr;
BLECharacteristic *pControlCharacteristic = nullptr;

// Forward decls
void updateDisplay();
void updateBatteryStatus();
void sendBatteryStatus();
void drawBatteryIcon(int x, int y);
void checkLowBattery();
void sendBatchesV2();
void imuSamplerTask(void*);

// ===== BLE callbacks =====
class ControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    std::string v = pChar->getValue();
    if (v.empty()) return;
    String cmd = String(v.c_str());
    Serial.println("CMD: " + cmd);

    if (cmd.startsWith("RATE:")) {
      uint32_t hz = cmd.substring(5).toInt();
      if (hz >= 1 && hz <= 1000) {
        samplingIntervalMs = 1000 / hz;
        Serial.printf("Rate -> %u Hz\n", hz);
      }
    } else if (cmd.startsWith("MTU:")) {
      uint32_t att = cmd.substring(4).toInt();
      // GUI sends ATT_MTU - 3 (usable payload) or full MTU; accept both
      if (att >= 20 && att <= 244) {
        g_maxPayload = att; // treat as payload directly
      } else if (att > 23 && att <= 247) {
        g_maxPayload = att - 3;
      }
      if (g_maxPayload < 20) g_maxPayload = 20;
      Serial.printf("Set max payload = %u bytes\n", g_maxPayload);
    } else if (cmd == "DISPLAY:ON") {
      displayEnabled = true; digitalWrite(DISPLAY_BL_PIN, HIGH); updateDisplay();
    } else if (cmd == "DISPLAY:OFF") {
      displayEnabled = false; digitalWrite(DISPLAY_BL_PIN, LOW);
    } else if (cmd == "BATTERY:REQUEST") {
      updateBatteryStatus(); sendBatteryStatus();
    } else if (cmd == "BATTERY_AUTO:ON") {
      batteryAutoUpdate = true; lastBatteryCheck = millis();
    } else if (cmd == "BATTERY_AUTO:OFF") {
      batteryAutoUpdate = false;
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* srv) override {
    deviceConnected = true;
    displayEnabled = true; digitalWrite(DISPLAY_BL_PIN, HIGH);
    Serial.println("BLE connected");
    updateDisplay();
    updateBatteryStatus(); sendBatteryStatus();
  }
  void onDisconnect(BLEServer* srv) override {
    deviceConnected = false;
    Serial.println("BLE disconnected");
    srv->startAdvertising();
    updateDisplay();
  }
};

// ===== Battery / display =====
void updateBatteryStatus() {
  long sum=0; const int N=20;
  for (int i=0;i<N;i++){ sum+=analogRead(BATTERY_ADC_PIN); delay(3); }
  int adc = sum/N;
  batteryVoltage = adc * ADC_VOLTAGE_FACTOR;

  if (batteryVoltage >= BATTERY_MAX_VOLTAGE) batteryPercentage = 100;
  else if (batteryVoltage <= BATTERY_MIN_VOLTAGE) batteryPercentage = 0;
  else batteryPercentage = (int)((batteryVoltage - BATTERY_MIN_VOLTAGE) * 100.0f /
                                (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE));

  static float lastV=batteryVoltage; static int cnt=0; static float acc=0;
  acc += batteryVoltage - lastV; cnt++;
  if (cnt>=5){ batteryCharging = (acc/cnt > 0.01f); acc=0; cnt=0; }
  lastV=batteryVoltage;

  Serial.printf("Battery %.2fV %d%% %s\n", batteryVoltage, batteryPercentage, batteryCharging?"chg":"dis");
}

void sendBatteryStatus() {
  if (!deviceConnected) return;
  StaticJsonDocument<200> doc;
  doc["type"]="battery_status";
  doc["voltage"]= roundf(batteryVoltage*100.f)/100.f;
  doc["percentage"]= batteryPercentage;
  doc["charging"]= batteryCharging;
  doc["timestamp"]= millis();
  char buf[192]; size_t n=serializeJson(doc, buf, sizeof(buf));
  pDataCharacteristic->setValue((uint8_t*)buf, n);
  pDataCharacteristic->notify();
  if (batteryPercentage<=15) checkLowBattery();
}

void drawBatteryIcon(int x,int y){
  const int w=24,h=12;
  Paint_DrawRectangle(x,y,x+w,y+h,BLACK,DOT_PIXEL_1X1,DRAW_FILL_EMPTY);
  Paint_DrawRectangle(x+w,y+3,x+w+2,y+h-3,BLACK,DOT_PIXEL_1X1,DRAW_FILL_FULL);
  int fill=(w-2)*batteryPercentage/100;
  if (fill>0) Paint_DrawRectangle(x+1,y+1,x+1+fill,y+h-1,BLACK,DOT_PIXEL_1X1,DRAW_FILL_FULL);
  if (batteryCharging){int fx=x+w/2-1,fy=y+3; Paint_DrawLine(fx,fy,fx+2,fy+6,BLACK,DOT_PIXEL_1X1,LINE_STYLE_SOLID);}
}

void updateDisplay(){
  if (!displayEnabled) return;
  Paint_Clear(WHITE);
  Paint_DrawString_EN(50,10,(char*)"ESP32-S3",&Font20,BLACK,WHITE);
  Paint_DrawString_EN(60,35,(char*)"IMU Monitor",&Font16,BLACK,WHITE);
  if (deviceConnected){
    Paint_DrawString_EN(70,60,(char*)"CONNECTED",&Font16,BLACK,WHITE);
    Paint_DrawCircle(220,15,5,GREEN,DOT_PIXEL_1X1,DRAW_FILL_FULL);
  } else {
    Paint_DrawString_EN(85,60,(char*)"READY",&Font16,BLACK,WHITE);
    Paint_DrawCircle(220,15,5,RED,DOT_PIXEL_1X1,DRAW_FILL_FULL);
  }
  drawBatteryIcon(10,90);
  char vstr[16]; sprintf(vstr,"%.2fV",batteryVoltage); Paint_DrawString_EN(45,90,vstr,&Font16,BLACK,WHITE);
  char pstr[16]; sprintf(pstr,"%d%%",batteryPercentage); Paint_DrawString_EN(45,110,pstr,&Font16,BLACK,WHITE);
  Paint_DrawString_EN(120,95, batteryCharging?(char*)"Charging":(char*)"Battery", &Font12, BLACK, WHITE);
  char rstr[20]; sprintf(rstr,"%luHz",(unsigned long)(1000/samplingIntervalMs));
  Paint_DrawString_EN(10,130,(char*)"Rate:",&Font12,BLACK,WHITE);
  Paint_DrawString_EN(60,130,rstr,&Font12,BLACK,WHITE);
  unsigned long up=millis()/1000; char ustr[24]; sprintf(ustr,"Up: %lus",up);
  Paint_DrawString_EN(120,130,ustr,&Font12,BLACK,WHITE);
  Paint_DrawString_EN(10,150, deviceConnected?(char*)"IMU: Active":(char*)"IMU: Standby", &Font12, BLACK, WHITE);
  if (batteryPercentage<=15) Paint_DrawString_EN(60,170,(char*)"LOW BATTERY!",&Font16,RED,WHITE);
  LCD_1IN28_Display(BlackImage);
  lastDisplayUpdate = millis();
}

void checkLowBattery(){
  static unsigned long lastWarn=0;
  if (batteryPercentage>15) return;
  if (millis()-lastWarn<300000) return;
  if (deviceConnected){
    StaticJsonDocument<100> doc; doc["type"]="warning";
    String msg = "Low battery: " + String(batteryPercentage) + "%";
    doc["message"]=msg;
    char buf[128]; size_t n=serializeJson(doc,buf,sizeof(buf));
    pDataCharacteristic->setValue((uint8_t*)buf,n); pDataCharacteristic->notify();
  }
  if (displayEnabled){
    Paint_Clear(RED);
    Paint_DrawString_EN(40,100,(char*)"LOW",&Font24,WHITE,RED);
    Paint_DrawString_EN(30,130,(char*)"BATTERY",&Font24,WHITE,RED);
    LCD_1IN28_Display(BlackImage);
    delay(800);
    updateDisplay();
  }
  lastWarn=millis();
}

// ===== V2 sender: MTU-aware, compact int16 =====
static inline int16_t q16(float v, float scale){ // v [phys], scale [units per LSB]
  long s = lroundf(v * scale);
  if (s > 32767) s = 32767;
  if (s < -32768) s = -32768;
  return (int16_t)s;
}

void sendBatchesV2(){
  if (!deviceConnected) return;

  // compute how many samples fit: header 4 bytes + N*14 <= g_maxPayload
  uint16_t payloadMax = g_maxPayload;
  if (payloadMax < 20) payloadMax = 20;
  uint8_t maxCount = (payloadMax - 4) / V2_REC_SIZE;
  if (maxCount < 1) maxCount = 1;
  if ((rb_wr - rb_rd) < 1) return;

  // choose count = min(available, maxCount), but don't loop forever; send multiple packets if needed
  for (int burst=0; burst<8 && (rb_wr - rb_rd) >= 1; ++burst) {
    uint8_t count = (rb_wr - rb_rd) < maxCount ? (rb_wr - rb_rd) : maxCount;
    uint8_t pkt[244]; // maximum payload we’ll ever use
    uint8_t* p = pkt;

    auto put8  = [&](uint8_t v){ *p++ = v; };
    auto put16 = [&](uint16_t v){ *p++ = v & 0xFF; *p++ = (v>>8) & 0xFF; };
    auto puti16= [&](int16_t v){ *p++ = v & 0xFF; *p++ = (v>>8) & 0xFF; };

    put16(MAGIC_V2);
    put8(VER_V2);
    put8(count);

    static uint32_t last_t = 0;
    for (uint8_t i=0;i<count;i++){
      Sample s = ringbuf[rb_rd % RING_N]; rb_rd++;
      uint16_t dt = (last_t==0) ? 0 : (uint16_t)min<uint32_t>(65535, s.t_ms - last_t);
      last_t = s.t_ms;

      // quantize: accel in mg, gyro in mdps (1e3)
      put16(dt);
      puti16(q16(s.ax, 1000.0f)); puti16(q16(s.ay, 1000.0f)); puti16(q16(s.az, 1000.0f));
      puti16(q16(s.gx, 1000.0f)); puti16(q16(s.gy, 1000.0f)); puti16(q16(s.gz, 1000.0f));
    }

    int nbytes = (int)(p - pkt);
    pDataCharacteristic->setValue(pkt, nbytes);
    pDataCharacteristic->notify();
  }
}

// ===== IMU sampler task =====
void imuSamplerTask(void*){
  Wire.begin();
  Wire.setClock(400000); // 400 kHz; raise to 1 MHz if your wiring & QMI8658 allow
  QMI8658_init();        // ensure ODR >= target

  TickType_t last = xTaskGetTickCount();
  const TickType_t ms2tick = pdMS_TO_TICKS(1);

  for(;;){
    vTaskDelayUntil(&last, samplingIntervalMs * ms2tick);

    float acc[3], gyro[3]; unsigned int tim_count;
    QMI8658_read_xyz(acc, gyro, &tim_count);

    size_t i = rb_wr;
    ringbuf[i % RING_N] = { millis(), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2] };
    rb_wr = i + 1;
  }
}

// ===== Setup / loop =====
void setup(){
  Serial.begin(115200);
  Serial.println("ESP32-S3 IMU (MTU-aware compact transport)");

  pinMode(DISPLAY_BL_PIN, OUTPUT); digitalWrite(DISPLAY_BL_PIN, HIGH);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(BATTERY_ADC_PIN, INPUT);

  if (!psramInit()) Serial.println("PSRAM init FAILED");
  BlackImage = (uint16_t*)ps_malloc(ImageSize);
  if (!BlackImage){ Serial.println("PSRAM alloc failed"); while(1) delay(1000); }

  DEV_Module_Init();
  LCD_1IN28_Init(HORIZONTAL);
  LCD_1IN28_Clear(WHITE);
  Paint_NewImage((UBYTE*)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
  Paint_SetScale(65);
  Paint_SetRotate(ROTATE_0);

  Paint_Clear(BLACK);
  Paint_DrawString_EN(60, 80, (char*)"ESP32-S3", &Font24, WHITE, BLACK);
  Paint_DrawString_EN(90, 110, (char*)"IMU", &Font24, WHITE, BLACK);
  Paint_DrawString_EN(75, 140, (char*)"v2.1 compact",&Font16, WHITE, BLACK);
  LCD_1IN28_Display(BlackImage);
  delay(1000);

  updateBatteryStatus();
  updateDisplay();

  BLEDevice::setMTU(247);
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  BLEDevice::init("ESP32-IMU");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *svc = pServer->createService(SERVICE_UUID);

  pDataCharacteristic = svc->createCharacteristic(
    DATA_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pDataCharacteristic->addDescriptor(new BLE2902());

  pControlCharacteristic = svc->createCharacteristic(
    CONTROL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE
  );
  pControlCharacteristic->setCallbacks(new ControlCallbacks());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("Advertising...");

  xTaskCreatePinnedToCore(imuSamplerTask, "imuSampler", 4096, nullptr, configMAX_PRIORITIES-2, nullptr, APP_CPU_NUM);

  Serial.printf("Init: %lu Hz, payload cap=%u\n", (unsigned long)(1000/samplingIntervalMs), g_maxPayload);
}

void loop(){
  static unsigned long lastHeartbeat=0;
  static bool lastBoot=HIGH;

  if (!deviceConnected && oldDeviceConnected){ delay(300); oldDeviceConnected = deviceConnected; }
  if ( deviceConnected && !oldDeviceConnected){ oldDeviceConnected = deviceConnected; }

  bool b = digitalRead(BOOT_PIN);
  if (lastBoot==HIGH && b==LOW){
    displayEnabled = !displayEnabled;
    digitalWrite(DISPLAY_BL_PIN, displayEnabled?HIGH:LOW);
    if (displayEnabled) updateDisplay();
    delay(180);
  }
  lastBoot=b;

  unsigned long bi = deviceConnected ? BATTERY_UPDATE_INTERVAL_CONNECTED : BATTERY_UPDATE_INTERVAL_READY;
  if (batteryAutoUpdate && (millis() - lastBatteryCheck > bi)){
    updateBatteryStatus();
    if (deviceConnected) sendBatteryStatus();
    lastBatteryCheck = millis();
  }

  unsigned long di = deviceConnected ? DISPLAY_UPDATE_INTERVAL_CONNECTED : DISPLAY_UPDATE_INTERVAL_READY;
  if (millis() - lastDisplayUpdate > di) updateDisplay();

  if (deviceConnected) sendBatchesV2();
  else delay(2);

  unsigned long hb = deviceConnected ? 5000 : 10000;
  if (millis() - lastHeartbeat > hb){
    Serial.printf("HB rb:%u/%u rate:%luHz batt:%.2fV(%d%%) payload:%u %s\n",
      (unsigned)(rb_wr - rb_rd), (unsigned)RING_N,
      (unsigned long)(1000/samplingIntervalMs),
      batteryVoltage, batteryPercentage, g_maxPayload, deviceConnected?"CONN":"READY");
    lastHeartbeat = millis();
  }

  checkLowBattery();
}
