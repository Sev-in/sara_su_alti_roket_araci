#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

/*
 * SARA Arduino Uno + PCA9685 guvenli donanim yazilimi
 *
 * Donanim:
 *   SEN0257 sari -> A0
 *   PCA9685 SDA/SCL -> A4/A5
 *   PCA9685 CH1 pitch, CH2 yaw, CH3 burun kapagi, CH4 ESC
 *   D6 -> siringa servosu SINYAL hatti
 *         (servo gucu harici 5-6 V kaynaktan, GND ortak)
 *   D5 -> ana guc kesme karti (LOW=enerji var, HIGH=enerji kesildi)
 *   D2 -> yalniz olcum modu butonu (INPUT_PULLUP, basili=3 V)
 *   D7 -> LM2596 enable (aktif LOW varsayimi)
 *   D8/D9/D10 -> X9C103S INC/U-D/CS
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

const uint16_t SERVO_MIN_US = 500;
const uint16_t SERVO_MAX_US = 2500;
const float KANAT_SERVO_MIN_DERECE = 45.0f;
const float KANAT_SERVO_MAX_DERECE = 135.0f;

// 45A cift yonlu sualti ESC:
//   1500 us = arm/dur
//   Yazilimla secilen itme yonu: 1500 us'den 1000 us'ye azalan PWM.
//   ESC veri araligi 1-2 ms'dir; 1500 us acilis/self-test notrudur.
//   Rampa 20 us / 100 ms oldugu icin tam araliga yaklasik 2.5 sn'de ulasir.
const uint16_t ESC_NOTR_US = 1500;
const int8_t ESC_ITME_YONU = -1;
const uint16_t ESC_TAM_ITKI_FARK_US = 500;
const uint16_t ESC_MUTLAK_MIN_US = 1000;
const uint16_t ESC_MUTLAK_MAX_US = 2000;
const uint16_t ESC_RAMP_ADIM_US = 20;
const unsigned long ESC_RAMP_ARALIGI_MS = 100;
const unsigned long ARM_BEKLEME_MS = 3000;

// D6 siringa servosu. D6 yalnizca sinyal uretir; servo kesinlikle
// Uno 5 V pininden beslenmemelidir. Gercek gorevlerde yalnizca suyu
// disari itme yonu kullanilir:
//   1800 us / 500 ms calis -> 1500 us / 200 ms notr
// Test modunda ise mekanik yon kontrolu icin 1200 ve 1800 us sirayla denenir.
const uint8_t YUZEY_SERVO_PINI = 6;
const uint16_t YUZEY_SERVO_DUR_US = 1500;
const uint16_t YUZEY_SERVO_SUYU_ICERI_CEK_US = 1200;
const uint16_t YUZEY_SERVO_SUYU_DISARI_IT_US = 1800;
const unsigned long YUZEY_SERVO_CALIS_SURESI_MS = 500;
const unsigned long YUZEY_SERVO_DUR_SURESI_MS = 200;

// ---------------- Guvenlik ----------------
const unsigned long HEARTBEAT_ZAMAN_ASIMI_MS = 1000;
const unsigned long SERVO_KOMUT_ZAMAN_ASIMI_MS = 500;
const unsigned long ATESLEME_MAKS_SURE_MS = 1000;
const uint8_t ANA_GUC_KESME_PINI = 5;  // LOW=enerji var, HIGH=kes
const uint8_t ATESLEME_FIZIKSEL_PINI = 2;
const uint8_t LM2596_ENABLE_PINI = 7;  // aktif LOW
const uint8_t X9C_INC_PINI = 8;
const uint8_t X9C_UD_PINI = 9;
const uint8_t X9C_CS_PINI = 10;
const int X9C_3V_ADIMI = 35;
const bool ATESLEME_OLCUM_TEST_MODU = true;
const uint8_t DURUM_LED_PINI = LED_BUILTIN;

Adafruit_PWMServoDriver pwm(PCA9685_ADRESI);
Servo yuzeyServo;

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
bool anaGucKesildi = false;

float pitchServoAcisi = 90.0f;
float yawServoAcisi = 90.0f;
float burunServoAcisi = 0.0f;
unsigned long sonServoKomutMs = 0;
bool kanatKomutuAlindi = false;
bool yuzeyServoCalisiyor = false;
bool yuzeyServoDarbeAktif = false;
bool yuzeyServoHareketFazi = false;
bool yuzeyServoTestModu = false;
bool yuzeyServoTestDisariYonu = false;
uint16_t yuzeyServoAktifUs = YUZEY_SERVO_SUYU_DISARI_IT_US;
unsigned long yuzeyServoFazBaslangicMs = 0;

bool ateslemeIstegi = false;
unsigned long ateslemeBaslangicMs = 0;
int x9cMevcutAdim = 99;
bool ateslemeButonuBasili = false;
bool ateslemeManuelTestModu = false;
bool ateslemeOlcumBirakmaBekliyor = true;

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
  yuzeyServoDarbeAktif = false;
  yuzeyServoHareketFazi = false;
  yuzeyServoTestModu = false;
  yuzeyServoAktifUs = YUZEY_SERVO_DUR_US;
  yuzeyServo.writeMicroseconds(YUZEY_SERVO_DUR_US);
  yuzeyServoCalisiyor = false;
}

void yuzeyServosunuBaslat(uint16_t baslangicUs, bool ikiYonTesti)
{
  yuzeyServoTestModu = ikiYonTesti;
  yuzeyServoTestDisariYonu = false;
  yuzeyServoAktifUs = baslangicUs;
  yuzeyServoDarbeAktif = true;
  yuzeyServoHareketFazi = true;
  yuzeyServoFazBaslangicMs = millis();
  yuzeyServo.writeMicroseconds(yuzeyServoAktifUs);
  yuzeyServoCalisiyor = true;
}

void ateslemeyiKapat()
{
  ateslemeIstegi = false;
  digitalWrite(LM2596_ENABLE_PINI, HIGH);
  digitalWrite(DURUM_LED_PINI, LOW);
  if (ateslemeManuelTestModu) {
    ateslemeOlcumBirakmaBekliyor = true;
  }
}

void x9cKalibreEt()
{
  // LM2596 bu islem boyunca kapali kalir.
  digitalWrite(LM2596_ENABLE_PINI, HIGH);
  digitalWrite(X9C_UD_PINI, HIGH);
  delayMicroseconds(5);
  digitalWrite(X9C_CS_PINI, LOW);
  delayMicroseconds(5);

  for (int i = 0; i < 100; ++i) {
    digitalWrite(X9C_INC_PINI, LOW);
    delayMicroseconds(5);
    digitalWrite(X9C_INC_PINI, HIGH);
    delayMicroseconds(5);
  }
  digitalWrite(X9C_CS_PINI, HIGH);
  x9cMevcutAdim = 99;
}

void x9cHedefAdimaGit(int hedefAdim)
{
  hedefAdim = constrain(hedefAdim, 0, 99);
  if (hedefAdim == x9cMevcutAdim) return;

  int fark = hedefAdim - x9cMevcutAdim;
  if (fark > 0) {
    digitalWrite(X9C_UD_PINI, HIGH);
  } else {
    digitalWrite(X9C_UD_PINI, LOW);
    fark = -fark;
  }

  delayMicroseconds(5);
  digitalWrite(X9C_CS_PINI, LOW);
  delayMicroseconds(5);
  for (int i = 0; i < fark; ++i) {
    digitalWrite(X9C_INC_PINI, LOW);
    delayMicroseconds(5);
    digitalWrite(X9C_INC_PINI, HIGH);
    delayMicroseconds(5);
  }
  digitalWrite(X9C_CS_PINI, HIGH);
  x9cMevcutAdim = hedefAdim;
}

void ateslemeManuelTestiniGuncelle()
{
  ateslemeButonuBasili =
      digitalRead(ATESLEME_FIZIKSEL_PINI) == LOW;

  if (!heartbeatGuncel()) {
    ateslemeyiKapat();
    return;
  }
  if (!ateslemeButonuBasili) {
    ateslemeIstegi = false;
    digitalWrite(LM2596_ENABLE_PINI, HIGH);
    digitalWrite(DURUM_LED_PINI, LOW);
    ateslemeOlcumBirakmaBekliyor = false;
    return;
  }
  if (ateslemeOlcumBirakmaBekliyor) {
    ateslemeIstegi = false;
    digitalWrite(LM2596_ENABLE_PINI, HIGH);
    digitalWrite(DURUM_LED_PINI, LOW);
    return;
  }

  ateslemeIstegi = true;
  digitalWrite(LM2596_ENABLE_PINI, LOW);
  digitalWrite(DURUM_LED_PINI, HIGH);
}

void tumunuGuvenliDurdur()
{
  ateslemeyiKapat();
  yuzeyServosunuDurdur();
  motorNotr(MOTOR_DISARMED);
  kanatlariMerkezle();
  burunKapaginiKapat();
}

void anaGucuKesVeKilitle()
{
  if (anaGucKesildi) return;

  // Once tum enerji tuketen/tehlikeli cikislari guvenli duruma getir.
  // Ardindan D5 HIGH yapilir. Bu fonksiyondan sonra firmware'in hicbir
  // yolu D5'i tekrar LOW yapmaz; yeniden acma ancak Uno reset/guc dongusu
  // ile mumkundur.
  tumunuGuvenliDurdur();
  anaGucKesildi = true;
  Serial.println(F("OK,POWER_CUTOFF_LATCHED"));
  Serial.flush();
  digitalWrite(ANA_GUC_KESME_PINI, HIGH);
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
  Serial.println(F("INFO,CALIBRATION_START"));
  delay(3000);
  sifirSensorGerilimi =
      ortalamaSensorGerilimiOku(KALIBRASYON_ORNEK_SAYISI);
  filtreliSensorGerilimi = sifirSensorGerilimi;
  Serial.println(F("INFO,CALIBRATION_DONE"));
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
    Serial.println(F("ERR,HEARTBEAT_REQUIRED"));
    return;
  }
  if (motorDurumu != MOTOR_ARMED && motorDurumu != MOTOR_RUNNING) {
    motorNotr(MOTOR_FAULT);
    Serial.println(F("ERR,MOTOR_NOT_ARMED"));
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

  if (anaGucKesildi) {
    if (strcmp(komut, "POWER_CUTOFF") == 0) {
      Serial.println(F("OK,POWER_CUTOFF_LATCHED"));
    } else {
      Serial.println(F("ERR,POWER_CUTOFF_LATCHED"));
    }
    return;
  }

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
  if (komuttanFloatOku(komut, "IGNITION_TEST_MODE=", deger)) {
    ateslemeyiKapat();
    if (deger >= 0.5f) {
      if (!ATESLEME_OLCUM_TEST_MODU) {
        Serial.println(F("ERR,IGNITION_TEST_MODE_DISABLED"));
      } else {
        ateslemeManuelTestModu = true;
        ateslemeOlcumBirakmaBekliyor = true;
        Serial.println(F("OK,IGNITION_MANUAL_TEST_ON"));
      }
    } else {
      ateslemeManuelTestModu = false;
      Serial.println(F("OK,IGNITION_MANUAL_TEST_OFF"));
    }
    return;
  }
  if (komuttanFloatOku(komut, "IGNITION=", deger)) {
    if (ateslemeManuelTestModu) {
      ateslemeyiKapat();
      Serial.println(F("ERR,IGNITION_MANUAL_TEST_MODE"));
      return;
    }
    if (deger >= 0.5f) {
      if (!heartbeatGuncel()) {
        ateslemeyiKapat();
        Serial.println(F("ERR,HEARTBEAT_REQUIRED"));
      } else if (digitalRead(ATESLEME_FIZIKSEL_PINI) == LOW) {
        ateslemeyiKapat();
        Serial.println(F("ERR,IGNITION_PHYSICAL_INHIBIT"));
      } else {
        ateslemeIstegi = true;
        ateslemeBaslangicMs = millis();
        if (ATESLEME_OLCUM_TEST_MODU) {
          Serial.println(F("OK,IGNITION_MEASUREMENT_ACTIVE"));
        } else {
          Serial.println(F("OK,IGNITION_ACTIVE"));
        }
      }
    } else {
      ateslemeyiKapat();
      Serial.println(F("OK,IGNITION_OFF"));
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
      Serial.println(F("ERR,HEARTBEAT_REQUIRED"));
    } else if (motorDurumu == MOTOR_DISARMED ||
               motorDurumu == MOTOR_FAULT) {
      motorNotr(MOTOR_ARMING);
      armBaslangicMs = millis();
      Serial.println(F("OK,MOTOR_ARMING"));
    }
    return;
  }
  if (strcmp(komut, "MOTOR_DISARM") == 0) {
    motorNotr(MOTOR_DISARMED);
    Serial.println(F("OK,MOTOR_DISARMED"));
    return;
  }
  if (strcmp(komut, "MOTOR_STOP") == 0) {
    hedefEscUs = ESC_NOTR_US;
    Serial.println(F("OK,MOTOR_STOPPING"));
    return;
  }
  if (strcmp(komut, "SERVO_CENTER") == 0) {
    kanatlariMerkezle();
    Serial.println(F("OK,SERVO_CENTER"));
    return;
  }
  if (strcmp(komut, "SURFACE_START") == 0) {
    if (!heartbeatGuncel()) {
      yuzeyServosunuDurdur();
      Serial.println(F("ERR,HEARTBEAT_REQUIRED"));
    } else {
      // Gercek gorev: yalnizca 1800 us ile suyu disari it.
      yuzeyServosunuBaslat(YUZEY_SERVO_SUYU_DISARI_IT_US, false);
      Serial.println(F("OK,SURFACE_PULSE_RUNNING"));
    }
    return;
  }
  if (strcmp(komut, "SURFACE_RETRACT_START") == 0) {
    if (!heartbeatGuncel()) {
      yuzeyServosunuDurdur();
      Serial.println(F("ERR,HEARTBEAT_REQUIRED"));
    } else {
      // Yalnizca test: 1200 us ile suyu iceri cek.
      yuzeyServosunuBaslat(YUZEY_SERVO_SUYU_ICERI_CEK_US, false);
      Serial.println(F("OK,SURFACE_RETRACT_RUNNING"));
    }
    return;
  }
  if (strcmp(komut, "SURFACE_TEST_START") == 0 ||
      strcmp(komut, "SURFACE_SWEEP_START") == 0) {
    if (!heartbeatGuncel()) {
      yuzeyServosunuDurdur();
      Serial.println(F("ERR,HEARTBEAT_REQUIRED"));
    } else {
      // Yalnizca test: 1200 ve 1800 us yonlerini sirayla dene.
      yuzeyServosunuBaslat(YUZEY_SERVO_SUYU_ICERI_CEK_US, true);
      Serial.println(F("OK,SURFACE_TEST_RUNNING"));
    }
    return;
  }
  if (strcmp(komut, "SURFACE_SWEEP_STOP") == 0) {
    yuzeyServosunuDurdur();
    Serial.println(F("OK,SURFACE_SWEEP_STOPPED"));
    return;
  }
  if (strcmp(komut, "SURFACE_STOP") == 0) {
    yuzeyServosunuDurdur();
    Serial.println(F("OK,SURFACE_SERVO_STOPPED"));
    return;
  }
  if (strcmp(komut, "ALL_STOP") == 0) {
    tumunuGuvenliDurdur();
    Serial.println(F("OK,ALL_STOPPED"));
    return;
  }
  if (strcmp(komut, "POWER_CUTOFF") == 0) {
    anaGucuKesVeKilitle();
    return;
  }
  if (strcmp(komut, "Z") == 0) {
    sifirKalibrasyonuYap();
    return;
  }
  Serial.println(F("ERR,UNKNOWN_COMMAND"));
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
      Serial.println(F("ERR,COMMAND_TOO_LONG"));
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
    Serial.println(F("ERR,HEARTBEAT_LOST"));
    return;
  }

  if (motorDurumu == MOTOR_ARMING &&
      simdi - armBaslangicMs >= ARM_BEKLEME_MS) {
    motorDurumu = MOTOR_ARMED;
    Serial.println(F("OK,MOTOR_ARMED"));
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
  if (anaGucKesildi) {
    // Olcum modu dahil hicbir arka plan yolu D5'i veya eyleyicileri
    // yeniden etkinlestiremez.
    digitalWrite(ANA_GUC_KESME_PINI, HIGH);
    return;
  }
  if (kanatKomutuAlindi &&
      millis() - sonServoKomutMs > SERVO_KOMUT_ZAMAN_ASIMI_MS) {
    kanatlariMerkezle();
    Serial.println(F("ERR,SERVO_COMMAND_TIMEOUT_CENTERED"));
  }
  if (yuzeyServoCalisiyor && !heartbeatGuncel()) {
    yuzeyServosunuDurdur();
    Serial.println(F("ERR,HEARTBEAT_LOST_SURFACE_STOPPED"));
  }

  if (ATESLEME_OLCUM_TEST_MODU && ateslemeManuelTestModu) {
    ateslemeManuelTestiniGuncelle();
    return;
  }

  ateslemeButonuBasili =
      digitalRead(ATESLEME_FIZIKSEL_PINI) == LOW;
  bool fizikselInhibit = ateslemeButonuBasili;
  bool ateslemeZamanAsimi =
      millis() - ateslemeBaslangicMs > ATESLEME_MAKS_SURE_MS;
  if (ateslemeIstegi &&
      (!heartbeatGuncel() || fizikselInhibit || ateslemeZamanAsimi)) {
    ateslemeyiKapat();
    Serial.println(F("ERR,IGNITION_SAFETY_CUTOFF"));
  }
  bool cikisAktif =
      ateslemeIstegi && heartbeatGuncel() && !fizikselInhibit;
  digitalWrite(LM2596_ENABLE_PINI, cikisAktif ? LOW : HIGH);
  digitalWrite(DURUM_LED_PINI, cikisAktif ? HIGH : LOW);
}

void yuzeyServoDarbeDeseniniGuncelle()
{
  if (!yuzeyServoDarbeAktif) return;

  unsigned long simdi = millis();
  unsigned long fazSuresi =
      yuzeyServoHareketFazi
          ? YUZEY_SERVO_CALIS_SURESI_MS
          : YUZEY_SERVO_DUR_SURESI_MS;
  if (simdi - yuzeyServoFazBaslangicMs < fazSuresi) return;

  yuzeyServoFazBaslangicMs = simdi;
  yuzeyServoHareketFazi = !yuzeyServoHareketFazi;
  if (yuzeyServoHareketFazi && yuzeyServoTestModu) {
    yuzeyServoTestDisariYonu = !yuzeyServoTestDisariYonu;
    yuzeyServoAktifUs = yuzeyServoTestDisariYonu
        ? YUZEY_SERVO_SUYU_DISARI_IT_US
        : YUZEY_SERVO_SUYU_ICERI_CEK_US;
  }
  yuzeyServo.writeMicroseconds(
      yuzeyServoHareketFazi
          ? yuzeyServoAktifUs
          : YUZEY_SERVO_DUR_US);
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
  Serial.print(F("voltage_v="));
  Serial.print(filtreliSensorGerilimi, 6);
  Serial.print(F(",pressure_kpa="));
  Serial.print(gaugeKpa, 3);
  Serial.print(F(",depth_m="));
  Serial.print(derinlik, 3);
  Serial.print(F(",esc_us="));
  Serial.print(mevcutEscUs);
  Serial.print(F(",motor_state="));
  Serial.print((uint8_t)motorDurumu);
  Serial.print(F(",heartbeat_ok="));
  Serial.print(heartbeatGuncel() ? 1 : 0);
  Serial.print(F(",pitch_deg="));
  Serial.print(pitchServoAcisi, 1);
  Serial.print(F(",yaw_deg="));
  Serial.print(yawServoAcisi, 1);
  Serial.print(F(",nose_deg="));
  Serial.print(burunServoAcisi, 1);
  Serial.print(F(",surface_running="));
  Serial.print(yuzeyServoCalisiyor ? 1 : 0);
  Serial.print(F(",surface_pulse="));
  Serial.print(yuzeyServoDarbeAktif ? 1 : 0);
  Serial.print(F(",surface_motion_phase="));
  Serial.print(yuzeyServoHareketFazi ? 1 : 0);
  Serial.print(F(",ignition_measurement_mode="));
  Serial.print(ATESLEME_OLCUM_TEST_MODU ? 1 : 0);
  Serial.print(F(",ignition_manual_test_mode="));
  Serial.print(ateslemeManuelTestModu ? 1 : 0);
  Serial.print(F(",ignition_button_pressed="));
  Serial.print(ateslemeButonuBasili ? 1 : 0);
  Serial.print(F(",ignition_active="));
  Serial.print(ateslemeIstegi ? 1 : 0);
  Serial.print(F(",power_cutoff_latched="));
  Serial.println(anaGucKesildi ? 1 : 0);
}

void setup()
{
  // Acilista enerji vardir. Bir kez HIGH yapildiktan sonra bu calisma
  // oturumunda D5'i LOW yapan baska hicbir kod yolu yoktur.
  pinMode(ANA_GUC_KESME_PINI, OUTPUT);
  digitalWrite(ANA_GUC_KESME_PINI, LOW);
  pinMode(LM2596_ENABLE_PINI, OUTPUT);
  digitalWrite(LM2596_ENABLE_PINI, HIGH);
  pinMode(ATESLEME_FIZIKSEL_PINI, INPUT_PULLUP);
  pinMode(X9C_INC_PINI, OUTPUT);
  pinMode(X9C_UD_PINI, OUTPUT);
  pinMode(X9C_CS_PINI, OUTPUT);
  digitalWrite(X9C_INC_PINI, HIGH);
  digitalWrite(X9C_UD_PINI, HIGH);
  digitalWrite(X9C_CS_PINI, HIGH);
  pinMode(DURUM_LED_PINI, OUTPUT);
  digitalWrite(DURUM_LED_PINI, LOW);
  pinMode(BASINC_PINI, INPUT);

  Serial.begin(115200);
  if (ATESLEME_OLCUM_TEST_MODU) {
    x9cKalibreEt();
    x9cHedefAdimaGit(X9C_3V_ADIMI);
    Serial.println(F("INFO,IGNITION_MEASUREMENT_MODE_ROS_READY"));
  }
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  yuzeyServo.attach(YUZEY_SERVO_PINI, SERVO_MIN_US, SERVO_MAX_US);
  delay(10);

  tumunuGuvenliDurdur();
  sifirKalibrasyonuYap();
  Serial.println(F("READY,SARA_UNO_CONTROLLER_V2"));
}

void loop()
{
  seriKomutlariOku();
  motoruGuncelle();
  yuzeyServoDarbeDeseniniGuncelle();
  guvenligiGuncelle();
  basinciOkuVeYayinla();
}
