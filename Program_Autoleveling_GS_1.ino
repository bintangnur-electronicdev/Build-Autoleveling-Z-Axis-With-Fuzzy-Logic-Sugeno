#include <Wire.h>

// ================= EEPROM & STORAGE CONFIG =================
#define EEPROM_ADDR 0x50
#define MAX_POINTS 12
#define MAX_SAMPLES 5
#define ADDR_SAMPLE_INDEX 0  // Alamat menyimpan index sampel aktif (1-5)

// ================= PIN CONFIGURATION =================
const int pinTrigger = 18;
const int pinOUT_X = 35;
const int pinOUT_Y = 32;
const int pinOUT_Z = 34;
const int pinDIR_X = 26; 
const int pinDIR_Y = 25; 
const int pinDIR_Z = 27;
const int pinSTEP_Z_AL  = 23;  
const int pinDIR_Z_AL   = 17;  
const int pinEN_Z_AL    = 16;  
const int pinTombolAL   = 13;  
const int pinRelay      = 33;  

// ================= KONSTANTA MEKANIK =================
#define ADC_MAX 4095.0
#define MM_PER_REV_X 8.0
#define MM_PER_REV_Y 50.4
#define MM_PER_REV_Z 8.0
#define DEADZONE_ANGLE 1.2 
#define HOLD_STEP 1

// ================= VARIABEL SISTEM =================
float posisiX = 0, posisiY = 0, posisiZ = 0, posisiZFiltered = 0;
float lastAngleX, lastAngleY, lastAngleZ;
int currentPoint = 1;
int activeSampleIdx = 1; 
int lastFinishedSample = 0;
bool isLockedEngraving = false;
bool showTableStatus = false;

unsigned long previousMillis = 0;
const unsigned long interval = 20;
unsigned long lastSaveTime = 0;
const unsigned long saveDelay = 1500;

// Multi-Pass (Deepening)
float zOffset = 0.7;
float targetTotalDepth = 1.4;
float incrementPerPass = 0.7;
bool hasLeftGate = false;
unsigned long lastPassTime = 0;

// ================= FUNGSI EEPROM I2C =================
void writeEEPROM(int addr, byte data) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(addr >> 8));
  Wire.write((int)(addr & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

byte readEEPROM(int addr) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(addr >> 8));
  Wire.write((int)(addr & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

void writeFloat(int addr, float value) {
  byte *data = (byte*)(void*)&value;
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(addr >> 8));
  Wire.write((int)(addr & 0xFF));
  for (int i = 0; i < 4; i++) { Wire.write(data[i]); }
  Wire.endTransmission();
  delay(10);
}

float readFloat(int addr) {
  byte data[4];
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(addr >> 8));
  Wire.write((int)(addr & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 4);
  for (int i = 0; i < 4; i++) { data[i] = Wire.available() ? Wire.read() : 0; }
  float value; memcpy(&value, data, 4);
  return value;
}

int getSampleBaseAddr(int sampleIdx) {
  return 10 + ((sampleIdx - 1) * 144);
}

// ================= FUZZY LOGIC SUGENO =================
float hitungFuzzySugeno(float error) {
  float muTC=0, muTL=0, muTT=0, muNL=0, muNC=0;
  if (error <= -1.0) muTC = 1.0; else if (error < -0.5) muTC = (-error - 0.5) / 0.5;
  if (error > -0.6 && error <= -0.25) muTL = (error + 0.6) / 0.35; else if (error < 0 && error > -0.25) muTL = (-error) / 0.25;
  if (error >= -0.15 && error <= 0.15) muTT = 1.0; 
  if (error > 0 && error <= 0.25) muNL = error / 0.25; else if (error < 0.6 && error > 0.25) muNL = (0.6 - error) / 0.35;
  if (error >= 1.0) muNC = 1.0; else if (error > 0.5 && error < 1.0) muNC = (error - 0.5) / 0.5;
  
  float pb = (muTC*-2500) + (muTL*-600) + (muTT*0) + (muNL*600) + (muNC*2500);
  float py = muTC + muTL + muTT + muNL + muNC;
  return (py == 0) ? 0 : (pb / py);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  pinMode(pinTrigger, INPUT_PULLUP);
  pinMode(pinTombolAL, INPUT_PULLUP);
  pinMode(pinRelay, OUTPUT);
  pinMode(pinSTEP_Z_AL, OUTPUT);
  pinMode(pinDIR_Z_AL, OUTPUT);
  pinMode(pinEN_Z_AL, OUTPUT);

  pinMode(pinDIR_X, OUTPUT); digitalWrite(pinDIR_X, LOW);
  pinMode(pinDIR_Y, OUTPUT); digitalWrite(pinDIR_Y, LOW);
  pinMode(pinDIR_Z, OUTPUT); digitalWrite(pinDIR_Z, LOW);

  analogReadResolution(12);
  lastAngleX = (analogRead(pinOUT_X) * 360.0) / ADC_MAX;
  lastAngleY = (analogRead(pinOUT_Y) * 360.0) / ADC_MAX;
  lastAngleZ = (analogRead(pinOUT_Z) * 360.0) / ADC_MAX;
  
  // Ambil index slot mana yang akan diisi
  activeSampleIdx = readEEPROM(ADDR_SAMPLE_INDEX);
  if (activeSampleIdx < 1 || activeSampleIdx > 5) activeSampleIdx = 1;

  Serial.println("\n========================================");
  Serial.printf(" SYSTEM READY | SLOT AKTIF: SAMPEL %d\n", activeSampleIdx);
  Serial.println("========================================\n");
}

// ================= LOOP UTAMA =================
void loop() {
  unsigned long currentMillis = millis();

  // 1. TOMBOL START (IO13)
  if (digitalRead(pinTombolAL) == LOW) {
    delay(200);
    if (showTableStatus) { // Hanya bisa mulai jika sudah deteksi
      isLockedEngraving = true;
      showTableStatus = false;
      digitalWrite(pinRelay, HIGH);
      Serial.println("\n[!] MODE AUTO-LEVELING AKTIF.");
    } else {
      Serial.println("[!] Selesaikan deteksi 12 titik dulu!");
    }
    while(digitalRead(pinTombolAL) == LOW);
  }

  // 2. SAMPLING 12 TITIK (IO18)
  if (!isLockedEngraving && currentPoint <= MAX_POINTS) {
    if (digitalRead(pinTrigger) == HIGH && (currentMillis - lastSaveTime >= saveDelay)) {
      int baseAddr = getSampleBaseAddr(activeSampleIdx);
      int pointOffset = (currentPoint - 1) * 12;
      
      writeFloat(baseAddr + pointOffset + 0, posisiX);
      writeFloat(baseAddr + pointOffset + 4, posisiY);
      writeFloat(baseAddr + pointOffset + 8, posisiZ);
      
      Serial.printf("Sampel %d - Titik %d Tersimpan!\n", activeSampleIdx, currentPoint);
      currentPoint++;
      lastSaveTime = currentMillis;

      if (currentPoint > MAX_POINTS) {
        lastFinishedSample = activeSampleIdx;
        showTableStatus = true;
        
        // Simpan index untuk sesi berikutnya (Circular)
        int nextIdx = activeSampleIdx + 1;
        if (nextIdx > MAX_SAMPLES) nextIdx = 1;
        writeEEPROM(ADDR_SAMPLE_INDEX, (byte)nextIdx);

        // TAMPILKAN TABEL DATA SHEET (Maks 5 Sampel)
        Serial.println("\n\n========== DATASET FUZZY GRID SEARCH ==========");
        for (int s = 1; s <= MAX_SAMPLES; s++) {
          int base = getSampleBaseAddr(s);
          // Cek apakah sampel ini ada datanya (X != 0 atau Y != 0)
          if (readFloat(base) != 0 || readFloat(base+4) != 0) {
            Serial.printf("\n[ SAMPEL %d ] %s\n", s, (s == lastFinishedSample) ? "(DATA BARU)" : "(SAMPEL LAMA)");
            Serial.println("----------------------------------------------");
            for (int p = 1; p <= MAX_POINTS; p++) {
              int pOff = (p - 1) * 12;
              Serial.printf("Titik %02d | X:%6.1f | Y:%6.1f | Z:%6.1f\n", p, readFloat(base+pOff), readFloat(base+pOff+4), readFloat(base+pOff+8));
            }
          }
        }
        Serial.println("================================================");
        Serial.println("STATUS: SELESAI. Tekan IO13 untuk Mulai.");
      }
    }
  }

  // 3. TRACKING POSISI (SERIAL REAL-TIME) & AUTO-LEVELING
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float aX = (analogRead(pinOUT_X) * 360.0) / ADC_MAX;
    float aY = (analogRead(pinOUT_Y) * 360.0) / ADC_MAX;
    float aZ = (analogRead(pinOUT_Z) * 360.0) / ADC_MAX;

    auto deltaFunc = [](float cur, float last) {
      float d = cur - last;
      if (d > 180) d -= 360; if (d < -180) d += 360;
      return d;
    };

    float dX = deltaFunc(aX, lastAngleX);
    float dY = deltaFunc(aY, lastAngleY);
    float dZ = deltaFunc(aZ, lastAngleZ);

    if (abs(dX) >= DEADZONE_ANGLE) posisiX += dX / 360.0 * MM_PER_REV_X;
    if (abs(dY) >= DEADZONE_ANGLE) posisiY += dY / 360.0 * MM_PER_REV_Y;
    if (abs(dZ) >= 1.5) posisiZ += dZ / 360.0 * MM_PER_REV_Z;

    posisiZFiltered = (posisiZFiltered * 0.7) + (posisiZ * 0.3);
    lastAngleX = aX; lastAngleY = aY; lastAngleZ = aZ;

    // --- AUTO-LEVELING ---
    if (isLockedEngraving) {
      // Multi-pass Logic (X0 Y0 gate)
      float gateX = 0.0, gateY = 0.0;
      if (abs(posisiX - gateX) > 10.0 || abs(posisiY - gateY) > 10.0) hasLeftGate = true;
      if (hasLeftGate && abs(posisiX - gateX) < 3.0 && abs(posisiY - gateY) < 3.0) {
          if (currentMillis - lastPassTime > 5000) {
              if (zOffset < targetTotalDepth) zOffset += incrementPerPass;
              hasLeftGate = false; lastPassTime = currentMillis;
          }
      }

      // Referensi Z diambil dari sampel yang barusan dideteksi
      int baseAddr = getSampleBaseAddr(lastFinishedSample);
      float minDist = 9999.0, targetZSurface = 0;
      for (int i = 0; i < MAX_POINTS; i++) {
        float ex = readFloat(baseAddr + (i * 12) + 0);
        float ey = readFloat(baseAddr + (i * 12) + 4);
        float dist = abs(posisiX - ex) + abs(posisiY - ey);
        if (dist < minDist) {
          minDist = dist;
          targetZSurface = readFloat(baseAddr + (i * 12) + 8);
        }
      }

      float errorZ = (targetZSurface - zOffset) - posisiZFiltered;
      float fOut = hitungFuzzySugeno(errorZ);

      if (abs(errorZ) > 0.15) {
        digitalWrite(pinDIR_Z_AL, fOut > 0 ? HIGH : LOW);
        int speedHz = (int)abs(fOut);
        if (speedHz < 200) speedHz = 200; 
        tone(pinSTEP_Z_AL, speedHz);
      } else {
        noTone(pinSTEP_Z_AL);
      }
    }

    // --- SERIAL MONITOR REAL-TIME ---
    if (!showTableStatus || isLockedEngraving) {
      static int lastOutX, lastOutY, lastOutZ;
      int curX = (int)posisiX; int curY = (int)posisiY; int curZ = (int)posisiZ;
      if (abs(curX - lastOutX) >= HOLD_STEP || abs(curY - lastOutY) >= HOLD_STEP || abs(curZ - lastOutZ) >= HOLD_STEP) {
        Serial.printf("X: %d | Y: %d | Z: %d\n", curX, curY, curZ);
        lastOutX = curX; lastOutY = curY; lastOutZ = curZ;
      }
    }
  }
}