#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*
 * SARA Arduino Uno + PCA9685 guvenli donanim yazilimi
 *
 * Donanim:
 *   SEN0257 sari -> A0
 *   PCA9685 SDA/SCL -> A4/A5
 *   CH1 pitch, CH2 yaw, CH3 burun kapagi, CH4 ESC, CH5 siringa
 *   D2 -> fiziksel atesleme inhibit butonu (INPUT_PULLUP)
 *   D7 -> LM2596 enable (aktif LOW varsayimi)
 *   USB serial -> Jetson Orin Nano, 115200 baud
 *
 * Jetson yalniz safety_node tarafindan onaylanmis komutlari yollar.
 * Herhangi bir heartbeat/komut kaybinda motor notr, kanatlar merkez,
 * siringa durur ve atesleme cikisi kapanir.
 */

// ---------------- Basinc / SEN0257 ----------------
const uint8_t BASINC_PINI = A0;
const float ADC_REFERANS_V = 5.000f;
const float ADC_MAKS_DEGER = 1023.0f;
const float SENSOR_KATSAYISI_KPA_V = 400.0f;
const float SU_YOGUNLUGU_KG_M3 = 1000.0f;
const float YERCEKIMI_M_S2 = 9.80665f;
const uint16_t ORNEK_SAYISI = 16;
const uint16_t KALIBRASYON_ORNEK_SAYISI = 300;
const float FILTRE_ALFA = 0.50f;
const unsigned long TELEMETRI_ARALIGI_MS = 200;

// ---------------- PCA9685 ----------------
const uint8_t PCA9685_ADRESI = 0x40;
const uint8_t PITCH_SERVO_KANALI = 1;
const uint8_t YAW_SERVO_KANALI = 2;
const uint8_t BURUN_SERVO_KANALI = 3;
const uint8_t ESC_KANALI = 4;
const uint8_t YUZEY_SERVO_KANALI = 5;

const uint16_t SERVO_MIN_US = 500;
const uint16_t SERVO_MAX_US = 2500;
const float KANAT_SERVO_MIN_DERECE = 45.0f;
const float KANAT_SERVO_MAX_DERECE = 135.0f;

// 45A cift yonlu sualti ESC:
//   1500 us = arm/dur
//   +1 = 1500..2000 us yonu (aracta dogrulanan itme yonu)
//   -1 = 1500..1000 us yonu (aracta dogrulanan cekme yonu)
const uint16_t ESC_NOTR_US = 1500;
const int8_t ESC_ITME_YONU = +1;
const uint16_t ESC_TAM_ITKI_FARK_US = 500;
const uint16_t ESC_MUTLAK_MIN_US = 1000;
const uint16_t ESC_MUTLAK_MAX_US = 2000;
const uint16_t ESC_RAMP_ADIM_US = 20;
const unsigned long ESC_RAMP_ARALIGI_MS = 100;
const unsigned long ARM_BEKLEME_MS = 3000;

// CH5 siringa servosu. Normal gorev komutlari korunur; yalnizca
// SURFACE_SWEEP_START test komutu 0-180 derece arasinda tarama yapar.
const uint16_t YUZEY_SERVO_DUR_US = 1500;
const uint16_t YUZEY_SERVO_CALIS_US = 1700;
const uint16_t YUZEY_TARAMA_MIN_US = SERVO_MIN_US;
const uint16_t YUZEY_TARAMA_MAX_US = SERVO_MAX_US;
const uint16_t YUZEY_TARAMA_ADIM_US = 10;
const unsigned long YUZEY_TARAMA_ARALIGI_MS = 20;

// ---------------- Guvenlik ----------------
const unsigned long HEARTBEAT_ZAMAN_ASIMI_MS = 1000;
const unsigned long SERVO_KOMUT_ZAMAN_ASIMI_MS = 500;
const unsigned long ATESLEME_MAKS_SURE_MS = 1000;
const uint8_t ATESLEME_INHIBIT_PINI = 2;
const uint8_t LM2596_ENABLE_PINI = 7;  // aktif LOW
const uint8_t DURUM_LED_PINI = LED_BUILTIN;

Adafruit_PWMServoDriver pwm(PCA9685_ADRESI);

enum MotorDurumu : uint8_t {
  MOTOR_DISARMED = 0,
  MOTOR_ARMING = 1,
  MOTOR_ARMED = 2,
  MOTOR_RUNNING = 3,
  MOTOR_FAULT = 4
};

MotorDurumu motorDurumu = MOTOR_DISARMED;
uint16_t mevcutEscUs = ESC_NOTR_US;
uint16_t hedefEscUs = ESC_NOTR_US;
unsigned long armBaslangicMs = 0;
unsigned long sonRampMs = 0;
unsigned long sonHeartbeatMs = 0;
bool heartbeatAlindi = false;

float pitchServoAcisi = 90.0f;
float yawServoAcisi = 90.0f;
float burunServoAcisi = 0.0f;
unsigned long sonServoKomutMs = 0;
bool kanatKomutuAlindi = false;
bool yuzeyServoCalisiyor = false;
bool yuzeyServoTaramaAktif = false;
uint16_t yuzeyTaramaUs = YUZEY_TARAMA_MIN_US;
int8_t yuzeyTaramaYonu = 1;
unsigned long sonYuzeyTaramaMs = 0;

bool ateslemeIstegi = false;
unsigned long ateslemeBaslangicMs = 0;

float sifirSensorGerilimi = 0.0f;
float filtreliSensorGerilimi = 0.0f;
unsigned long sonTelemetriMs = 0;

char komutTamponu[48];
uint8_t komutUzunlugu = 0;

bool heartbeatGuncel()
{
  return heartbeatAlindi &&
         (millis() - sonHeartbeatMs <= HEARTBEAT_ZAMAN_ASIMI_MS);
}

void servoAciYaz(uint8_t kanal, float aci)
{
  aci = constrain(aci, 0.0f, 180.0f);
  uint16_t darbeUs = (uint16_t)(
      SERVO_MIN_US + aci / 180.0f * (SERVO_MAX_US - SERVO_MIN_US));
  pwm.writeMicroseconds(kanal, darbeUs);
}

void kanatlariMerkezle()
{
  pitchServoAcisi = 90.0f;
  yawServoAcisi = 90.0f;
  servoAciYaz(PITCH_SERVO_KANALI, pitchServoAcisi);
  servoAciYaz(YAW_SERVO_KANALI, yawServoAcisi);
  kanatKomutuAlindi = false;
}

void burunKapaginiKapat()
{
  burunServoAcisi = 0.0f;
  servoAciYaz(BURUN_SERVO_KANALI, burunServoAcisi);
}

void motorNotr(MotorDurumu durum)
{
  hedefEscUs = ESC_NOTR_US;
  mevcutEscUs = ESC_NOTR_US;
  pwm.writeMicroseconds(ESC_KANALI, mevcutEscUs);
  motorDurumu = durum;
}

void yuzeyServosunuDurdur()
{
  yuzeyServoTaramaAktif = false;
  pwm.writeMicroseconds(YUZEY_SERVO_KANALI, YUZEY_SERVO_DUR_US);
  yuzeyServoCalisiyor = false;
}

void ateslemeyiKapat()
{
  ateslemeIstegi = false;
  digitalWrite(LM2596_ENABLE_PINI, HIGH);
  digitalWrite(DURUM_LED_PINI, LOW);
}

void tumunuGuvenliDurdur()
{
  ateslemeyiKapat();
  yuzeyServosunuDurdur();
  motorNotr(MOTOR_DISARMED);
  kanatlariMerkezle();
  burunKapaginiKapat();
}

bool komuttanFloatOku(const char *komut, const char *onek, float &sonuc)
{
  size_t uzunluk = strlen(onek);
  if (strncmp(komut, onek, uzunluk) != 0) return false;
  char *bitis;
  sonuc = strtod(komut + uzunluk, &bitis);
  return bitis != komut + uzunluk && *bitis == '\0' && isfinite(sonuc);
}

float sensorGerilimiOku()
{
  return ((float)analogRead(BASINC_PINI) * ADC_REFERANS_V) /
         ADC_MAKS_DEGER;
}

float ortalamaSensorGerilimiOku(uint16_t adet)
{
  float toplam = 0.0f;
  for (uint16_t i = 0; i < adet; ++i) {
    toplam += sensorGerilimiOku();
    delayMicroseconds(500);
  }
  return toplam / (float)adet;
}

void sifirKalibrasyonuYap()
{
  tumunuGuvenliDurdur();
  Serial.println("INFO,CALIBRATION_START");
  delay(3000);
  sifirSensorGerilimi =
      ortalamaSensorGerilimiOku(KALIBRASYON_ORNEK_SAYISI);
  filtreliSensorGerilimi = sifirSensorGerilimi;
  Serial.println("INFO,CALIBRATION_DONE");
}

void itkiKomutunuUygula(float normalizeItki)
{
  normalizeItki = constrain(normalizeItki, 0.0f, 1.0f);
  if (normalizeItki <= 0.001f) {
    hedefEscUs = ESC_NOTR_US;
    return;
  }
  if (!heartbeatGuncel()) {
    motorNotr(MOTOR_FAULT);
    Serial.println("ERR,HEARTBEAT_REQUIRED");
    return;
  }
  if (motorDurumu != MOTOR_ARMED && motorDurumu != MOTOR_RUNNING) {
    motorNotr(MOTOR_FAULT);
    Serial.println("ERR,MOTOR_NOT_ARMED");
    return;
  }
  // Yon isaretli tutulur; bu sayede -500 us hesabi Uno'da uint16_t
  // tasmasina donusmez. Sonuc ayrıca ESC'nin guvenli araliginda sinirlanir.
  const float escAralikUs =
      (float)ESC_ITME_YONU * (float)ESC_TAM_ITKI_FARK_US;
  const long hesaplananEscUs = lroundf(
      (float)ESC_NOTR_US + normalizeItki * escAralikUs);
  hedefEscUs = (uint16_t)constrain(
      hesaplananEscUs,
      (long)ESC_MUTLAK_MIN_US,
      (long)ESC_MUTLAK_MAX_US);
  motorDurumu = MOTOR_RUNNING;
}

void komutuIsle(char *komut)
{
  for (char *p = komut; *p; ++p) *p = toupper(*p);

  float deger;
  if (komuttanFloatOku(komut, "THRUST=", deger)) {
    itkiKomutunuUygula(deger);
    return;
  }
  if (komuttanFloatOku(komut, "PITCH=", deger)) {
    pitchServoAcisi = constrain(
        deger, KANAT_SERVO_MIN_DERECE, KANAT_SERVO_MAX_DERECE);
    servoAciYaz(PITCH_SERVO_KANALI, pitchServoAcisi);
    sonServoKomutMs = millis();
    kanatKomutuAlindi = true;
    return;
  }
  if (komuttanFloatOku(komut, "YAW=", deger)) {
    yawServoAcisi = constrain(
        deger, KANAT_SERVO_MIN_DERECE, KANAT_SERVO_MAX_DERECE);
    servoAciYaz(YAW_SERVO_KANALI, yawServoAcisi);
    sonServoKomutMs = millis();
    kanatKomutuAlindi = true;
    return;
  }
  if (komuttanFloatOku(komut, "NOSE=", deger)) {
    burunServoAcisi = constrain(deger, 0.0f, 180.0f);
    servoAciYaz(BURUN_SERVO_KANALI, burunServoAcisi);
    return;
  }
  if (komuttanFloatOku(komut, "IGNITION=", deger)) {
    if (deger >= 0.5f) {
      if (!heartbeatGuncel()) {
        ateslemeyiKapat();
        Serial.println("ERR,HEARTBEAT_REQUIRED");
      } else if (digitalRead(ATESLEME_INHIBIT_PINI) == LOW) {
        ateslemeyiKapat();
        Serial.println("ERR,IGNITION_PHYSICAL_INHIBIT");
      } else {
        ateslemeIstegi = true;
        ateslemeBaslangicMs = millis();
        Serial.println("OK,IGNITION_ACTIVE");
      }
    } else {
      ateslemeyiKapat();
      Serial.println("OK,IGNITION_OFF");
    }
    return;
  }

  if (strcmp(komut, "HB") == 0) {
    heartbeatAlindi = true;
    sonHeartbeatMs = millis();
    return;
  }
  if (strcmp(komut, "MOTOR_ARM") == 0) {
    if (!heartbeatGuncel()) {
      motorNotr(MOTOR_FAULT);
      Serial.println("ERR,HEARTBEAT_REQUIRED");
    } else if (motorDurumu == MOTOR_DISARMED ||
               motorDurumu == MOTOR_FAULT) {
      motorNotr(MOTOR_ARMING);
      armBaslangicMs = millis();
      Serial.println("OK,MOTOR_ARMING");
    }
    return;
  }
  if (strcmp(komut, "MOTOR_DISARM") == 0) {
    motorNotr(MOTOR_DISARMED);
    Serial.println("OK,MOTOR_DISARMED");
    return;
  }
  if (strcmp(komut, "MOTOR_STOP") == 0) {
    hedefEscUs = ESC_NOTR_US;
    Serial.println("OK,MOTOR_STOPPING");
    return;
  }
  if (strcmp(komut, "SERVO_CENTER") == 0) {
    kanatlariMerkezle();
    Serial.println("OK,SERVO_CENTER");
    return;
  }
  if (strcmp(komut, "SURFACE_START") == 0) {
    if (!heartbeatGuncel()) {
      yuzeyServosunuDurdur();
      Serial.println("ERR,HEARTBEAT_REQUIRED");
    } else {
      yuzeyServoTaramaAktif = false;
      pwm.writeMicroseconds(
          YUZEY_SERVO_KANALI, YUZEY_SERVO_CALIS_US);
      yuzeyServoCalisiyor = true;
      Serial.println("OK,SURFACE_SERVO_RUNNING");
    }
    return;
  }
  if (strcmp(komut, "SURFACE_SWEEP_START") == 0) {
    if (!heartbeatGuncel()) {
      yuzeyServosunuDurdur();
      Serial.println("ERR,HEARTBEAT_REQUIRED");
    } else {
      yuzeyTaramaUs = YUZEY_TARAMA_MIN_US;
      yuzeyTaramaYonu = 1;
      sonYuzeyTaramaMs = millis();
      yuzeyServoTaramaAktif = true;
      yuzeyServoCalisiyor = true;
      pwm.writeMicroseconds(YUZEY_SERVO_KANALI, yuzeyTaramaUs);
      Serial.println("OK,SURFACE_SWEEP_RUNNING");
    }
    return;
  }
  if (strcmp(komut, "SURFACE_SWEEP_STOP") == 0) {
    yuzeyServosunuDurdur();
    Serial.println("OK,SURFACE_SWEEP_STOPPED");
    return;
  }
  if (strcmp(komut, "SURFACE_STOP") == 0) {
    yuzeyServosunuDurdur();
    Serial.println("OK,SURFACE_SERVO_STOPPED");
    return;
  }
  if (strcmp(komut, "ALL_STOP") == 0) {
    tumunuGuvenliDurdur();
    Serial.println("OK,ALL_STOPPED");
    return;
  }
  if (strcmp(komut, "Z") == 0) {
    sifirKalibrasyonuYap();
    return;
  }
  Serial.println("ERR,UNKNOWN_COMMAND");
}

void seriKomutlariOku()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (komutUzunlugu > 0) {
        komutTamponu[komutUzunlugu] = '\0';
        komutuIsle(komutTamponu);
        komutUzunlugu = 0;
      }
    } else if (komutUzunlugu < sizeof(komutTamponu) - 1) {
      komutTamponu[komutUzunlugu++] = c;
    } else {
      komutUzunlugu = 0;
      tumunuGuvenliDurdur();
      Serial.println("ERR,COMMAND_TOO_LONG");
    }
  }
}

void motoruGuncelle()
{
  unsigned long simdi = millis();
  if ((motorDurumu == MOTOR_ARMING || motorDurumu == MOTOR_ARMED ||
       motorDurumu == MOTOR_RUNNING) && !heartbeatGuncel()) {
    tumunuGuvenliDurdur();
    motorDurumu = MOTOR_FAULT;
    Serial.println("ERR,HEARTBEAT_LOST");
    return;
  }

  if (motorDurumu == MOTOR_ARMING &&
      simdi - armBaslangicMs >= ARM_BEKLEME_MS) {
    motorDurumu = MOTOR_ARMED;
    Serial.println("OK,MOTOR_ARMED");
  }

  if (simdi - sonRampMs < ESC_RAMP_ARALIGI_MS) return;
  sonRampMs = simdi;
  if (mevcutEscUs < hedefEscUs) {
    mevcutEscUs = min(
        (uint16_t)(mevcutEscUs + ESC_RAMP_ADIM_US), hedefEscUs);
  } else if (mevcutEscUs > hedefEscUs) {
    mevcutEscUs = max(
        (uint16_t)(mevcutEscUs - ESC_RAMP_ADIM_US), hedefEscUs);
  }
  pwm.writeMicroseconds(ESC_KANALI, mevcutEscUs);
  if (hedefEscUs == ESC_NOTR_US && mevcutEscUs == ESC_NOTR_US &&
      motorDurumu == MOTOR_RUNNING) {
    motorDurumu = MOTOR_ARMED;
  }
}

void guvenligiGuncelle()
{
  if (kanatKomutuAlindi &&
      millis() - sonServoKomutMs > SERVO_KOMUT_ZAMAN_ASIMI_MS) {
    kanatlariMerkezle();
    Serial.println("ERR,SERVO_COMMAND_TIMEOUT_CENTERED");
  }
  if (yuzeyServoCalisiyor && !heartbeatGuncel()) {
    yuzeyServosunuDurdur();
    Serial.println("ERR,HEARTBEAT_LOST_SURFACE_STOPPED");
  }

  bool fizikselInhibit = digitalRead(ATESLEME_INHIBIT_PINI) == LOW;
  bool ateslemeZamanAsimi =
      millis() - ateslemeBaslangicMs > ATESLEME_MAKS_SURE_MS;
  if (ateslemeIstegi &&
      (!heartbeatGuncel() || fizikselInhibit || ateslemeZamanAsimi)) {
    ateslemeyiKapat();
    Serial.println("ERR,IGNITION_SAFETY_CUTOFF");
  }
  bool cikisAktif =
      ateslemeIstegi && heartbeatGuncel() && !fizikselInhibit;
  digitalWrite(LM2596_ENABLE_PINI, cikisAktif ? LOW : HIGH);
  digitalWrite(DURUM_LED_PINI, cikisAktif ? HIGH : LOW);
}

void yuzeyServoTaramasiniGuncelle()
{
  if (!yuzeyServoTaramaAktif) return;

  unsigned long simdi = millis();
  if (simdi - sonYuzeyTaramaMs < YUZEY_TARAMA_ARALIGI_MS) return;
  sonYuzeyTaramaMs = simdi;

  long sonrakiUs =
      (long)yuzeyTaramaUs +
      (long)yuzeyTaramaYonu * (long)YUZEY_TARAMA_ADIM_US;
  if (sonrakiUs >= (long)YUZEY_TARAMA_MAX_US) {
    sonrakiUs = YUZEY_TARAMA_MAX_US;
    yuzeyTaramaYonu = -1;
  } else if (sonrakiUs <= (long)YUZEY_TARAMA_MIN_US) {
    sonrakiUs = YUZEY_TARAMA_MIN_US;
    yuzeyTaramaYonu = 1;
  }

  yuzeyTaramaUs = (uint16_t)sonrakiUs;
  pwm.writeMicroseconds(YUZEY_SERVO_KANALI, yuzeyTaramaUs);
}

void basinciOkuVeYayinla()
{
  float gerilim = ortalamaSensorGerilimiOku(ORNEK_SAYISI);
  filtreliSensorGerilimi +=
      FILTRE_ALFA * (gerilim - filtreliSensorGerilimi);
  float gaugeKpa =
      (filtreliSensorGerilimi - sifirSensorGerilimi) *
      SENSOR_KATSAYISI_KPA_V;
  if (gaugeKpa < 0.0f) gaugeKpa = 0.0f;
  float derinlik =
      gaugeKpa * 1000.0f / (SU_YOGUNLUGU_KG_M3 * YERCEKIMI_M_S2);

  if (millis() - sonTelemetriMs < TELEMETRI_ARALIGI_MS) return;
  sonTelemetriMs = millis();
  Serial.print("voltage_v=");
  Serial.print(filtreliSensorGerilimi, 6);
  Serial.print(",pressure_kpa=");
  Serial.print(gaugeKpa, 3);
  Serial.print(",depth_m=");
  Serial.print(derinlik, 3);
  Serial.print(",esc_us=");
  Serial.print(mevcutEscUs);
  Serial.print(",motor_state=");
  Serial.print((uint8_t)motorDurumu);
  Serial.print(",heartbeat_ok=");
  Serial.print(heartbeatGuncel() ? 1 : 0);
  Serial.print(",pitch_deg=");
  Serial.print(pitchServoAcisi, 1);
  Serial.print(",yaw_deg=");
  Serial.print(yawServoAcisi, 1);
  Serial.print(",nose_deg=");
  Serial.print(burunServoAcisi, 1);
  Serial.print(",surface_running=");
  Serial.print(yuzeyServoCalisiyor ? 1 : 0);
  Serial.print(",surface_sweep=");
  Serial.print(yuzeyServoTaramaAktif ? 1 : 0);
  Serial.print(",ignition_active=");
  Serial.println(ateslemeIstegi ? 1 : 0);
}

void setup()
{
  pinMode(LM2596_ENABLE_PINI, OUTPUT);
  digitalWrite(LM2596_ENABLE_PINI, HIGH);
  pinMode(ATESLEME_INHIBIT_PINI, INPUT_PULLUP);
  pinMode(DURUM_LED_PINI, OUTPUT);
  digitalWrite(DURUM_LED_PINI, LOW);
  pinMode(BASINC_PINI, INPUT);

  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  tumunuGuvenliDurdur();
  sifirKalibrasyonuYap();
  Serial.println("READY,SARA_UNO_CONTROLLER_V2");
}

void loop()
{
  seriKomutlariOku();
  motoruGuncelle();
  yuzeyServoTaramasiniGuncelle();
  guvenligiGuncelle();
  basinciOkuVeYayinla();
}
