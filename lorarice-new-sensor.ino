// T114 + SHT31 + VL53L1X + GPS (L76K) -> LoRaWAN (OTAA) -> TTN
#ifndef REGION_AS923
#define REGION_AS923
#endif

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "Adafruit_VL53L1X.h"
#include "heltec_nrf_lorawan.h"

// =====================================================================
// ตั้งค่าที่ปรับได้
// =====================================================================
#define TX_INTERVAL_MIN        1       // ช่วงส่ง (นาที) ตอนทดสอบ 1 นาที / ตอนใช้งานจริงแนะนำ 15 นาที
#define GPS_FIX_TIMEOUT_MS     15000   // รอ GPS fix สูงสุดต่อรอบ (ถ้าไม่ได้จะใช้พิกัดล่าสุดที่เคยได้)
#define GPS_REFRESH_EVERY_N_CYCLES 1   // ขอ GPS ทุก N รอบ (1 = ทุกรอบ) รอบที่ข้ามจะไม่รอ GPS เลย
                                       // ถ้ายังไม่เคยได้ fix จะลองทุกรอบจนกว่าจะได้
#define SENSOR_HEIGHT_MM       500.0   // ความสูงเซนเซอร์วัดระยะจากฐาน (มม.)
#define SENSOR_BOOT_MS         500     // รอเซนเซอร์บูตหลังเปิดไฟ
#define FORCE_RX2_923_2        1       // ตั้ง RX2 = 923.2 MHz DR2 (ตามโค้ดชุดเดิมของคุณ)

// ถ้าบอร์ดมีวงจร NMOS คุมไฟเซนเซอร์แยกที่ GPIO13 ให้เปลี่ยนเป็น 1
#define USE_NMOS_SENSOR_POWER  0
#define PIN_POWER_CTL          13

// ===== Pin / Address =====
#define I2C_SDA     29
#define I2C_SCL     31
#define SHT31_ADDR  0x44
#define VL53_ADDR   0x29

// ===== Battery (ค่าเดิมจากโค้ดชุดแรกของคุณ) =====
#define PIN_BAT_ADC      4    // GPIO4
#define PIN_BAT_ADC_CTL  6    // GPIO6
#define MY_BAT_AMPLIFY   4.9    // ตัวคูณวงจรแบ่งแรงดันของบอร์ด
#define BATT_CTL_ENABLE  HIGH   // ระดับที่เปิดวงจรวัดแบต (ถ้าได้ค่า 0 ลองเปลี่ยนเป็น LOW)
#define BATT_ADC_REF_MV  3000.0 // แรงดันอ้างอิง ADC ภายใน (mV)
#define BATT_CAL_FACTOR  1.0    // ตัวปรับเทียบ: ค่าจริงจากมัลติมิเตอร์ / ค่าที่บอร์ดอ่านได้
#define BATT_MIN_VALID_MV 1000  // ต่ำกว่านี้ถือว่าอ่านไม่ได้ (ส่ง 0 ให้ decoder แปลงเป็น null)

// GPS L76K ใช้ Serial2 (9600 baud)
#define gpsSerial Serial2

// ค่าที่ใช้บอกว่า "อ่านไม่ได้" ใน payload (decoder จะแปลงเป็น null)
#define INVALID_TEMP   ((int16_t)-32768)
#define INVALID_HUMI   ((uint16_t)0xFFFF)
#define INVALID_DIST   ((int16_t)-99)

// =====================================================================
// LoRaWAN OTAA Credentials  (ต้องตรงกับ end device ที่ลงทะเบียนใน TTN)
// =====================================================================
uint8_t devEui[] = {};
uint8_t appEui[] = {};
uint8_t appKey[] = {};

/* ABP (ไม่ได้ใช้เมื่อ overTheAirActivation = true แต่ไลบรารีต้องมีตัวแปรนี้) */
uint8_t nwkSKey[] = {0x69, 0x1D, 0xD6, 0x27, 0x44, 0x33, 0xEC, 0x69, 0x8F, 0xE2, 0xFD, 0xF9, 0x42, 0x51, 0x7E, 0xF4};
uint8_t appSKey[] = {0xFD, 0xF8, 0x17, 0x47, 0x02, 0x45, 0xDF, 0xEB, 0xCF, 0x4F, 0x81, 0xD6, 0xEC, 0x23, 0x23, 0x58};
uint32_t devAddr = (uint32_t)0x27FC5D83;

// ===== LoRaWAN Settings =====
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AS923;
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;      // TTN fair use: ไม่แนะนำ confirmed uplink ถี่ ๆ (ทำให้เกิด downlink ACK)
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;
uint32_t appTxDutyCycle = (uint32_t)TX_INTERVAL_MIN * 60UL * 1000UL;
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

#define APP_TX_DUTYCYCLE_RND 1000

// =====================================================================
// Global
// =====================================================================
TwoWire *wi = &Wire;
TinyGPSPlus gps;
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

uint16_t batteryVoltage = 0;   // mV (0 = อ่านไม่ได้)

bool shtOk = false;
bool vl53Ok = false;

float temperatureC = 0;
float humidityPct = 0;

float distanceVL = 0;        // ระยะที่ปรับแล้ว = SENSOR_HEIGHT_MM - distance
float distanceLF = 0;        // ผ่าน low-pass filter
bool lfInitialized = false;

// พิกัดล่าสุดที่เคยได้ (เก็บไว้ข้ามรอบ เพราะ GPS ถูกตัดไฟทุกรอบ)
double lastLat = 0;
double lastLon = 0;
bool gpsEverFixed = false;

uint32_t cycleCount = 0;     // นับรอบการส่ง ใช้กำหนดว่ารอบไหนต้องขอ GPS

// =====================================================================
// Helpers
// =====================================================================
void sensorPower(bool on) {
  digitalWrite(PIN_VEXT_CTL, on ? VEXT_ENABLE : (VEXT_ENABLE ? LOW : HIGH));
#if USE_NMOS_SENSOR_POWER
  digitalWrite(PIN_POWER_CTL, on ? HIGH : LOW);
#endif
}

void feedGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

bool i2cPresent(uint8_t addr) {
  wi->beginTransmission(addr);
  return wi->endTransmission() == 0;
}

uint8_t sht31Crc(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

bool readSHT31(float &t, float &h) {
  wi->beginTransmission(SHT31_ADDR);
  wi->write(0x24);
  wi->write(0x00);
  if (wi->endTransmission() != 0) {
    Serial.println("SHT31: ส่งคำสั่งไม่สำเร็จ");
    return false;
  }
  delay(20);

  if (wi->requestFrom((uint8_t)SHT31_ADDR, (uint8_t)6) != 6) {
    Serial.println("SHT31: อ่านข้อมูลไม่ครบ 6 byte");
    return false;
  }
  uint8_t buf[6];
  for (int i = 0; i < 6; i++) buf[i] = wi->read();

  if (sht31Crc(buf, 2) != buf[2] || sht31Crc(buf + 3, 2) != buf[5]) {
    Serial.println("SHT31: CRC ไม่ตรง");
    return false;
  }
  uint16_t rawT = ((uint16_t)buf[0] << 8) | buf[1];
  uint16_t rawH = ((uint16_t)buf[3] << 8) | buf[4];
  t = -45.0f + 175.0f * rawT / 65535.0f;
  h = 100.0f * rawH / 65535.0f;
  return true;
}

bool readVL53(int16_t &raw) {
  uint32_t start = millis();
  while (!vl53.dataReady()) {
    if (millis() - start > 3000) {
      Serial.println("VL53L1X Timeout.");
      return false;
    }
    delay(5);
  }
  raw = vl53.distance();
  vl53.clearInterrupt();

  if (raw < 0) {
    Serial.print(F("VL53L1X: อ่านระยะไม่ได้, status: "));
    Serial.println(vl53.vl_status);
    return false;
  }

  distanceVL = SENSOR_HEIGHT_MM - raw;
  if (!lfInitialized) {
    distanceLF = distanceVL;  // ครั้งแรกใช้ค่าจริงเลย ไม่เริ่มจาก 0
    lfInitialized = true;
  } else {
    distanceLF = (distanceLF * 0.5f) + (distanceVL * 0.5f);
  }
  return true;
}

// เปิดไฟเซนเซอร์ แล้วเริ่มต้น SHT31 / VL53L1X ใหม่ (ไฟถูกตัดทุกรอบ)
void startSensors() {
  sensorPower(true);
  delay(SENSOR_BOOT_MS);

  shtOk = false;
  vl53Ok = false;

  if (i2cPresent(SHT31_ADDR)) {
    wi->beginTransmission(SHT31_ADDR);  // soft reset
    wi->write(0x30);
    wi->write(0xA2);
    wi->endTransmission();
    delay(10);
    shtOk = true;
  } else {
    Serial.println("ไม่เจอ SHT31 ที่ 0x44");
  }

  // เช็ค 0x29 ก่อน ไม่งั้น vl53.begin() อาจค้าง
  if (!i2cPresent(VL53_ADDR)) {
    Serial.println("ไม่เจอ VL53L1X ที่ 0x29");
  } else if (!vl53.begin(VL53_ADDR, wi)) {
    Serial.print(F("VL53L1X init failed, status: "));
    Serial.println(vl53.vl_status);
  } else {
    vl53.VL53L1X_SetROI(5, 5);
    if (vl53.startRanging()) {
      vl53Ok = true;
    } else {
      Serial.print(F("VL53L1X startRanging failed, status: "));
      Serial.println(vl53.vl_status);
    }
  }
}

// รอ GPS fix สดใหม่ (อายุไม่เกิน 2 วินาที) ภายในเวลาที่กำหนด
bool waitGPSFix(uint32_t timeoutMs) {
  uint32_t start = millis();
  uint32_t lastLog = 0;
  while (millis() - start < timeoutMs) {
    feedGPS();
    if (gps.location.isValid() && gps.location.age() < 2000) {
      lastLat = gps.location.lat();
      lastLon = gps.location.lng();
      gpsEverFixed = true;
      return true;
    }
    if (millis() - lastLog > 5000) {
      lastLog = millis();
      Serial.printf("รอ GPS fix... %lu s (chars: %lu, sats: %u)\n",
                    (millis() - start) / 1000, gps.charsProcessed(),
                    gps.satellites.isValid() ? gps.satellites.value() : 0);
    }
    delay(2);
  }
  return false;
}

// อ่านแรงดันแบตเตอรี่ (mV) โดยไม่ใช้ไลบรารี BatteryMonitor
uint16_t readBatteryMv() {
  pinMode(PIN_BAT_ADC_CTL, OUTPUT);
  digitalWrite(PIN_BAT_ADC_CTL, BATT_CTL_ENABLE);  // เปิดวงจรแบ่งแรงดัน
  delay(10);

  analogReadResolution(12);
  analogReference(AR_INTERNAL_3_0);
  analogRead(PIN_BAT_ADC);  // อ่านทิ้ง 1 ครั้งให้ ADC นิ่ง

  const int N = 8;
  uint32_t sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(PIN_BAT_ADC);
    delay(2);
  }

  digitalWrite(PIN_BAT_ADC_CTL, !BATT_CTL_ENABLE);  // ปิดวงจรเพื่อประหยัดไฟ

  float raw = (float)sum / N;
  float mv = raw / 4095.0f * BATT_ADC_REF_MV * MY_BAT_AMPLIFY * BATT_CAL_FACTOR;
  return (uint16_t)lroundf(mv);
}

void put16(int idx, uint16_t v) {
  appData[idx] = (v >> 8) & 0xFF;
  appData[idx + 1] = v & 0xFF;
}

void put32(int idx, uint32_t v) {
  appData[idx] = (v >> 24) & 0xFF;
  appData[idx + 1] = (v >> 16) & 0xFF;
  appData[idx + 2] = (v >> 8) & 0xFF;
  appData[idx + 3] = v & 0xFF;
}

// =====================================================================
// Payload 18 byte (รูปแบบเดียวกับโค้ดเดิม)
//  0-1  temp   int16  x100 °C        (-32768 = อ่านไม่ได้)
//  2-3  humi   uint16 x100 %         (0xFFFF = อ่านไม่ได้)
//  4-5  dist   int16  mm             (-99 = อ่านไม่ได้)
//  6-7  distLF int16  mm             (-99 = อ่านไม่ได้)
//  8-9  batt   uint16 mV/10          (0 = อ่านไม่ได้)
//  10-13 lat   int32  x1e6
//  14-17 lon   int32  x1e6
// =====================================================================
void prepareTxFrame(uint8_t port) {
  appDataSize = 18;
  cycleCount++;

  Serial.println("Reading sensors...");
  startSensors();

  int16_t rawDist = 0;
  bool distOk = vl53Ok && readVL53(rawDist);
  if (distOk) {
    Serial.printf("Distance: %d mm, DistanceVL: %.2f, DistanceLF: %.2f\n", rawDist, distanceVL, distanceLF);
  }

  bool tempOk = shtOk && readSHT31(temperatureC, humidityPct);
  if (tempOk) {
    Serial.printf("Temp: %.2f C, Humidity: %.2f %%\n", temperatureC, humidityPct);
  }

  // Battery
  uint16_t mv = readBatteryMv();
  batteryVoltage = (mv >= BATT_MIN_VALID_MV) ? mv : 0;
  Serial.printf("Battery: %u mV\n", batteryVoltage);

  // GPS: ขอ fix เฉพาะรอบที่ต้องการ (ยังไม่เคยได้ fix หรือถึงรอบ refresh)
  bool wantGps = !gpsEverFixed || (((cycleCount - 1) % GPS_REFRESH_EVERY_N_CYCLES) == 0);
  if (wantGps) {
    bool fix = waitGPSFix(GPS_FIX_TIMEOUT_MS);
    if (fix) {
      Serial.printf("GPS fix: %.6f, %.6f (sats: %u)\n", lastLat, lastLon, gps.satellites.value());
    } else if (gpsEverFixed) {
      Serial.println("GPS ไม่ได้ fix รอบนี้ ใช้พิกัดล่าสุด");
    } else {
      Serial.println("GPS ยังไม่เคยได้ fix ส่งพิกัด 0,0");
    }
  } else {
    Serial.println("ข้าม GPS รอบนี้ ใช้พิกัดล่าสุด");
  }

  int16_t temp = tempOk ? (int16_t)lroundf(temperatureC * 100.0f) : INVALID_TEMP;
  uint16_t humi = tempOk ? (uint16_t)lroundf(humidityPct * 100.0f) : INVALID_HUMI;
  int16_t dist = distOk ? (int16_t)lroundf(distanceVL) : INVALID_DIST;
  int16_t distLF = distOk ? (int16_t)lroundf(distanceLF) : INVALID_DIST;
  uint16_t batt = batteryVoltage / 10;   // mV/10 (เช่น 3700 mV -> 370)
  int32_t lat = (int32_t)lround(lastLat * 1e6);
  int32_t lon = (int32_t)lround(lastLon * 1e6);

  put16(0, (uint16_t)temp);
  put16(2, humi);
  put16(4, (uint16_t)dist);
  put16(6, (uint16_t)distLF);
  put16(8, batt);
  put32(10, (uint32_t)lat);
  put32(14, (uint32_t)lon);

  Serial.printf("Payload: Temp=%d Humi=%u Dist=%d DistLF=%d Batt=%u Lat=%ld Lon=%ld\n",
                temp, humi, dist, distLF, batt, (long)lat, (long)lon);
}

// =====================================================================
void setup() {
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, 115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) delay(10);
  Serial.println("Booting T114 TTN node...");

  pinMode(PIN_VEXT_CTL, OUTPUT);
#if USE_NMOS_SENSOR_POWER
  pinMode(PIN_POWER_CTL, OUTPUT);
#endif
  sensorPower(false);

  gpsSerial.begin(9600);

  wi->setPins(I2C_SDA, I2C_SCL);
  wi->begin();
  wi->setClock(100000);

  deviceState = DEVICE_STATE_INIT;
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
#if FORCE_RX2_923_2
      {
        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_RX2_CHANNEL;
        mibReq.Param.Rx2Channel.Frequency = 923200000;
        mibReq.Param.Rx2Channel.Datarate = DR_2;
        LoRaMacMibSetRequestConfirm(&mibReq);
      }
#endif
      deviceState = DEVICE_STATE_JOIN;
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      break;

    case DEVICE_STATE_SEND:
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;

    case DEVICE_STATE_CYCLE:
      sensorPower(false);  // ตัดไฟเซนเซอร์/GPS ก่อนหลับ
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;

    case DEVICE_STATE_SLEEP:
      LoRaWAN.sleep(loraWanClass);
      break;

    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}
