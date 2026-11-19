/*
  Method B: Arduino Nano 33 BLE + TI FDC1004, 4 CNT honeycomb piezocapacitive sensors -> BLE.

  Board  : "Arduino Nano 33 BLE" (Arduino Mbed OS Nano Boards core)
  Library: ArduinoBLE (Library Manager). The FDC1004 is driven with Wire, no extra library.

  Wiring : FDC1004 VDD -> 3.3V, GND -> GND, SDA -> A4, SCL -> A5 (address 0x50)
           CIN1 Heel, CIN2 Instep, CIN3 Toe-ball, CIN4 Toes (top electrode of each sensor)
           bottom electrode of every sensor -> GND
           SHLD1 -> shield of each lead (shielded cable), if used

  Ranging: at boot each channel's C0 is measured and its CAPDAC (3.125 pF steps, 0..31)
           is set so the unloaded reading sits about 10 pF below zero. The FDC1004 window is
           +/-15 pF around the CAPDAC offset, so about 24 pF is left for loading.

  Output : BLE notify, 50 Hz, 20 bytes little-endian =
           uint32 time ms, then float32 dC/C0 for Heel, Instep, Toe-ball, Toes.
           Same packet as the ESP32 firmware, so one dashboard reads both.

  Keep the foot OFF the insole for the first ~3 s after power-up (ranging + baseline).
  Serial (115200): 'z' = re-take baseline, 'p' = toggle CSV print, 'c' = print C0 per sensor (pF).
*/
#include <Wire.h>
#include <ArduinoBLE.h>

const uint8_t FDC = 0x50;
const uint8_t REG_CONF_MEAS1 = 0x08;     // ..0x0B
const uint8_t REG_FDC_CONF   = 0x0C;
const uint8_t REG_MANUF_ID   = 0xFE;     // 0x5449
const uint8_t REG_DEVICE_ID  = 0xFF;     // 0x1004
const float   CAPDAC_PF      = 3.125f;
const float   HEADROOM_PF    = 12.0f;    // offset = C0 + this, rounded down to a CAPDAC step
const uint16_t RATE_200SPS   = 0b10;     // 4 channels x 50 S/s each in repeat mode

const uint32_t PERIOD_MS = 20;           // 50 Hz output
const uint16_t BASE_N    = 100;          // 2 s baseline

const char *NAMES[4] = {"Heel", "Instep", "Toe-ball", "Toes"};

BLEService gaitSvc("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
BLECharacteristic dataChr("6e400003-b5a3-f393-e0a9-e50e24dcca9e", BLERead | BLENotify, 20, true);

struct __attribute__((packed)) Packet { uint32_t ms; float v[4]; };
static_assert(sizeof(Packet) == 20, "packet must be 20 bytes");

uint8_t capdac[4] = {0};
float c0[4] = {0};
bool printCsv = false;

// ------------------------------------------------------------------- I2C
void wr16(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(FDC);
  Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF);
  Wire.endTransmission();
}
uint16_t rd16(uint8_t reg) {
  Wire.beginTransmission(FDC);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(FDC, (uint8_t)2);
  uint16_t v = (uint16_t)Wire.read() << 8;
  v |= Wire.read();
  return v;
}

// ---------------------------------------------------------------- FDC1004
// CONF_MEASn: CHA [15:13] = CINn, CHB [12:10] = 0b100 (CAPDAC), CAPDAC [9:5]
void setChannel(uint8_t i, uint8_t dac) {
  wr16(REG_CONF_MEAS1 + i, ((uint16_t)i << 13) | (0b100 << 10) | ((uint16_t)(dac & 0x1F) << 5));
}
// FDC_CONF: RATE [11:10], REPEAT [8], MEAS1..4 enable [7:4]
void startRepeat() { wr16(REG_FDC_CONF, (RATE_200SPS << 10) | (1 << 8) | 0x00F0); }
void stopAll()     { wr16(REG_FDC_CONF, 0); }

// Reading relative to the CAPDAC offset, in pF (+/-15 pF window). sat = true at the rails.
float readRel(uint8_t i, bool *sat = nullptr) {
  uint16_t msb = rd16(0x00 + 2 * i), lsb = rd16(0x01 + 2 * i);
  int32_t raw = (int32_t)(((uint32_t)msb << 16) | lsb) >> 8;   // signed 24-bit
  if (sat) *sat = (raw >= 0x7FFFF0 || raw <= -0x7FFFF0);
  return raw / 524288.0f;                                     // 2^19 counts per pF
}
float readPF(uint8_t i, bool *sat = nullptr) { return readRel(i, sat) + capdac[i] * CAPDAC_PF; }

// Measure C0 on every channel, then choose CAPDAC so the unloaded reading sits ~10 pF below zero.
void autoRange() {
  for (uint8_t i = 0; i < 4; i++) { capdac[i] = 0; setChannel(i, 0); }
  startRepeat();
  delay(60);
  for (uint8_t i = 0; i < 4; i++) {
    bool sat;
    readRel(i, &sat);
    while (sat && capdac[i] < 31) {                 // above 15 pF: step the offset up
      capdac[i] += 4; if (capdac[i] > 31) capdac[i] = 31;
      stopAll(); setChannel(i, capdac[i]); startRepeat(); delay(60);
      readRel(i, &sat);
    }
    float c = readPF(i);
    int d = (int)((c + HEADROOM_PF) / CAPDAC_PF);
    if (d > 31) d = 31;
    if (d < 0) d = 0;
    capdac[i] = d;
  }
  stopAll();
  for (uint8_t i = 0; i < 4; i++) setChannel(i, capdac[i]);
  startRepeat();
  delay(60);
}

void takeBaseline() {
  Serial.println("Baseline: keep the foot off the insole (2 s)...");
  double acc[4] = {0};
  for (uint16_t n = 0; n < BASE_N; n++) {
    for (uint8_t i = 0; i < 4; i++) acc[i] += readPF(i);
    delay(PERIOD_MS);
  }
  for (uint8_t i = 0; i < 4; i++) c0[i] = acc[i] / BASE_N;
  printC0();
}

void printC0() {
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(NAMES[i]); Serial.print(": C0 = "); Serial.print(c0[i], 3);
    Serial.print(" pF, CAPDAC = "); Serial.println(capdac[i]);
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 2000) {}        // wait for USB, but run on battery too
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);
  if (rd16(REG_MANUF_ID) != 0x5449 || rd16(REG_DEVICE_ID) != 0x1004) {
    Serial.println("FDC1004 not found: check 3.3V, GND, SDA=A4, SCL=A5");
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
  }
  wr16(REG_FDC_CONF, 0x8000);                      // reset
  delay(10);
  autoRange();
  takeBaseline();

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(500); }
  }
  BLE.setLocalName("GAIT-FDC");
  BLE.setDeviceName("GAIT-FDC");
  BLE.setAdvertisedService(gaitSvc);
  gaitSvc.addCharacteristic(dataChr);
  BLE.addService(gaitSvc);
  BLE.advertise();
  Serial.println("Advertising as GAIT-FDC");
}

void loop() {
  static uint32_t next = millis();
  BLE.poll();
  digitalWrite(LED_BUILTIN, BLE.connected());

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') takeBaseline();
    if (c == 'p') printCsv = !printCsv;
    if (c == 'c') printC0();
  }
  if ((int32_t)(millis() - next) < 0) return;
  next += PERIOD_MS;

  Packet pkt;
  pkt.ms = millis();
  for (uint8_t i = 0; i < 4; i++) {
    // c0 is the total C at CINn (sensor + leads), so this ratio is a little below the bare-sensor
    // dC/C0 from the LCR meter; the dashboard calibration absorbs that.
    float v = c0[i] > 0.1f ? (readPF(i) - c0[i]) / c0[i] : 0.0f;   // 0 if a sensor is unplugged
    pkt.v[i] = v > 0 ? v : 0;
  }
  if (BLE.connected()) dataChr.writeValue((uint8_t *)&pkt, sizeof(pkt));
  if (printCsv) {
    for (uint8_t i = 0; i < 4; i++) { Serial.print(pkt.v[i], 4); Serial.print(i < 3 ? ',' : '\n'); }
  }
}
