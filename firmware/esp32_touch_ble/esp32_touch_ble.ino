/*
  Method A: ESP32 smart insole, 4 CNT honeycomb piezocapacitive sensors -> BLE.

  Board : "ESP32 Dev Module" (Arduino-ESP32 core 2.x or 3.x)
  Wiring: one copper-tape face of each sensor -> touch pin, the other face -> GND.
            Heel     T0  GPIO4
            Instep   T4  GPIO13
            Toe-ball T6  GPIO14
            Toes     T7  GPIO27
          (GPIO12 / T5 is avoided: it is a boot strapping pin.)

  RC fallback (set USE_RC 1): the Uno-style charge-time circuit.
          drive pin --[1..10 Mohm]--+-- sensor -- GND
                                    |
                               sense pin
            Heel     drive GPIO25  sense GPIO34
            Instep   drive GPIO26  sense GPIO35
            Toe-ball drive GPIO18  sense GPIO36 (VP)
            Toes     drive GPIO19  sense GPIO39 (VN)

  Output: BLE notify, 50 Hz, 20 bytes little-endian =
          uint32 time ms, then float32 dC/C0 for Heel, Instep, Toe-ball, Toes.
          Same packet as the Nano 33 BLE + FDC1004 firmware, so one dashboard reads both.

  Keep the foot OFF the insole for the first 2 s after power-up (baseline).
  Serial (115200): 'z' = re-take baseline, 'p' = toggle CSV print (Serial Plotter).
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#ifndef USE_RC
#define USE_RC 0                    // 0 = touch pins, 1 = RC charge-time fallback
#endif

#define SVC_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

const uint8_t TOUCH_PIN[4] = {4, 13, 14, 27};
const uint8_t RC_DRIVE[4]  = {25, 26, 18, 19};
const uint8_t RC_SENSE[4]  = {34, 35, 36, 39};

const uint32_t PERIOD_MS   = 20;    // 50 Hz output
const uint8_t  READS       = 4;     // reads averaged per output sample
const float    EMA         = 0.5f;  // extra smoothing, 1 = off
const uint16_t BASE_N      = 100;   // 100 x 20 ms = 2 s baseline

struct __attribute__((packed)) Packet { uint32_t ms; float v[4]; };
static_assert(sizeof(Packet) == 20, "packet must be 20 bytes");

BLECharacteristic *dataChr = nullptr;
volatile bool bleConnected = false;
float base[4] = {0}, filt[4] = {0};
bool printCsv = false;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }
  void onDisconnect(BLEServer *s) override { bleConnected = false; s->startAdvertising(); }
};

// --------------------------------------------------------------- raw reads
#if USE_RC
// Charge time in CPU cycles; proportional to R * C (time grows with C).
float readRaw(uint8_t i) {
  const uint32_t TIMEOUT = 240 * 2000;          // 2 ms at 240 MHz
  digitalWrite(RC_DRIVE[i], LOW);
  delayMicroseconds(800);                       // discharge (>5 RC for 10 Mohm, 15 pF)
  uint32_t t0 = ESP.getCycleCount();
  digitalWrite(RC_DRIVE[i], HIGH);
  while (!digitalRead(RC_SENSE[i])) {
    if (ESP.getCycleCount() - t0 > TIMEOUT) break;
  }
  uint32_t dt = ESP.getCycleCount() - t0;
  digitalWrite(RC_DRIVE[i], LOW);
  return (float)dt;
}
// value grows with C: dC/C0 = raw/base - 1
inline float toRatio(float raw, float b) { return raw / b - 1.0f; }
#else
float readRaw(uint8_t i) { return (float)touchRead(TOUCH_PIN[i]); }
  #if CONFIG_IDF_TARGET_ESP32
// ESP32 classic: count falls as C rises (count ~ 1/C): dC/C0 = base/raw - 1
inline float toRatio(float raw, float b) { return raw > 0 ? b / raw - 1.0f : 0.0f; }
  #else
// ESP32-S2/S3: count rises with C
inline float toRatio(float raw, float b) { return raw / b - 1.0f; }
  #endif
#endif

void readAll(float out[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    float s = 0;
    for (uint8_t k = 0; k < READS; k++) s += readRaw(i);
    out[i] = s / READS;
  }
}

void takeBaseline() {
  Serial.println("Baseline: keep the foot off the insole (2 s)...");
  double acc[4] = {0};
  float r[4];
  for (uint16_t n = 0; n < BASE_N; n++) {
    readAll(r);
    for (uint8_t i = 0; i < 4; i++) acc[i] += r[i];
    delay(PERIOD_MS);
  }
  for (uint8_t i = 0; i < 4; i++) { base[i] = acc[i] / BASE_N; filt[i] = base[i]; }
  Serial.printf("Baseline raw: %.1f %.1f %.1f %.1f\n", base[0], base[1], base[2], base[3]);
}

void setup() {
  Serial.begin(115200);
#if USE_RC
  for (uint8_t i = 0; i < 4; i++) { pinMode(RC_DRIVE[i], OUTPUT); digitalWrite(RC_DRIVE[i], LOW); pinMode(RC_SENSE[i], INPUT); }
#endif
  takeBaseline();

  BLEDevice::init("GAIT-ESP32");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(SVC_UUID);
  dataChr = svc->createCharacteristic(CHR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  dataChr->addDescriptor(new BLE2902());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);         // name goes in the scan response (128-bit UUID fills the advert)
  BLEDevice::startAdvertising();
  Serial.println("Advertising as GAIT-ESP32");
}

void loop() {
  static uint32_t next = millis();
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') takeBaseline();
    if (c == 'p') printCsv = !printCsv;
  }
  if ((int32_t)(millis() - next) < 0) return;
  next += PERIOD_MS;

  float r[4];
  readAll(r);
  Packet pkt;
  pkt.ms = millis();
  for (uint8_t i = 0; i < 4; i++) {
    filt[i] = EMA * r[i] + (1 - EMA) * filt[i];
    float v = toRatio(filt[i], base[i]);
    pkt.v[i] = v > 0 ? v : 0;          // unloading below baseline is noise
  }
  if (bleConnected) {
    dataChr->setValue((uint8_t *)&pkt, sizeof(pkt));
    dataChr->notify();
  }
  if (printCsv) Serial.printf("%.3f,%.3f,%.3f,%.3f\n", pkt.v[0], pkt.v[1], pkt.v[2], pkt.v[3]);
}
