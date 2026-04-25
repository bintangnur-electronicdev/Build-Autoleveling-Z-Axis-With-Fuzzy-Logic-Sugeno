#include <Wire.h>

// ================= EEPROM & KONFIGURASI =================
#define EEPROM_ADDR 0x50
#define MAX_COUNTER 12
const int pinTrigger = 18;

// ================= PIN SENSOR ENCODER (STRICT: JANGAN DIRUBA) =================
const int pinOUT_X = 35;
const int pinOUT_Y = 32;
const int pinOUT_Z = 34;

// PIN DIR ENCODER (TETAP OUTPUT LOW)
const int pinDIR_X = 26; 
const int pinDIR_Y = 25; 
const int pinDIR_Z = 27;

// ================= PIN AUTO-LEVELING (TB6600) =================
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

// ================= VARIABEL MULTI-PASS =================
float zOffset = 0.8;            // Kedalaman awal
float targetTotalDepth = 1.6;   // Batas maksimal kedalaman
float incrementPerPass = 0.8;  // Tambahan kedalaman tiap putaran
bool hasLeftGate = false;       // Flag deteksi mesin sudah jalan
unsigned long lastPassTime = 0; // Debounce trigger layer

// ================= VARIABEL SISTEM =================
float lastAngleX, lastAngleY, lastAngleZ;
float posisiX = 0, posisiY = 0, posisiZ = 0;
float posisiZFiltered = 0; 
int outX = 0, outY = 0, outZ = 0; 

int currentCounter = 1;
bool isLockedEngraving = false; 
bool showTableStatus = false; 

unsigned long previousMillis = 0;
const unsigned long interval = 20;
unsigned long lastSaveTime = 0;
const unsigned long saveDelay = 1500; 

// ================= FUNGSI EEPROM =================
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
  float value;
  memcpy(&value, data, 4);
  return value;
}

// ================= FUZZY SUGENO (STABIL & HALUS) =================
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

  // KUNCI PIN DIR ENCODER (OUTPUT LOW)
  pinMode(pinDIR_X, OUTPUT);
  pinMode(pinDIR_Y, OUTPUT);
  pinMode(pinDIR_Z, OUTPUT);
  digitalWrite(pinDIR_X, LOW);
  digitalWrite(pinDIR_Y, LOW);
  digitalWrite(pinDIR_Z, LOW);

  analogReadResolution(12);
  lastAngleX = (analogRead(pinOUT_X) * 360.0) / ADC_MAX;
  lastAngleY = (analogRead(pinOUT_Y) * 360.0) / ADC_MAX;
  lastAngleZ = (analogRead(pinOUT_Z) * 360.0) / ADC_MAX;
  
  digitalWrite(pinEN_Z_AL, LOW); 
  digitalWrite(pinRelay, LOW);    
  Serial.println("=== SYSTEM READY: SILAKAN DETEKSI 12 TITIK ===");
}

// ================= LOOP UTAMA =================
void loop() {
  unsigned long currentMillis = millis();

  // 1. TOMBOL LOCK (IO13) - MULAI ENGRAVING
  if (digitalRead(pinTombolAL) == LOW) {
    delay(200); 
    if (currentCounter > MAX_COUNTER) {
      isLockedEngraving = true;
      showTableStatus = false;
      digitalWrite(pinRelay, HIGH);    
      Serial.println("\n[!] MODE AUTO-LEVELING AKTIF.");
      Serial.printf("STARTING PASS 1 - OFFSET: %.2f mm\n", zOffset);
    }
    while(digitalRead(pinTombolAL) == LOW);
  }

  // 2. DETEKSI DATA (IO18)
  if (!isLockedEngraving && currentCounter <= MAX_COUNTER) {
    if (digitalRead(pinTrigger) == HIGH && (currentMillis - lastSaveTime >= saveDelay)) {
      int base = (currentCounter - 1) * 12;
      writeFloat(base + 0, posisiX);
      writeFloat(base + 4, posisiY);
      writeFloat(base + 8, posisiZ);
      Serial.printf("\n>>> TITIK %d TERSIMPAN", currentCounter);
      currentCounter++;
      lastSaveTime = currentMillis; 
      
      if (currentCounter > MAX_COUNTER) {
        showTableStatus = true; 
        delay(500);
        Serial.println("\n\n========================================");
        Serial.println("     DATA HASIL AUTLEVELING (12 TITIK)   ");
        Serial.println("========================================");
        for (int i = 1; i <= MAX_COUNTER; i++) {
          int addr = (i - 1) * 12;
          Serial.printf("Titik %02d | X:%6.1f | Y:%6.1f | Z:%6.1f\n", i, readFloat(addr), readFloat(addr+4), readFloat(addr+8));
        }
        Serial.println("========================================");
        Serial.println("STATUS: SELESAI. TEKAN IO13 UNTUK MULAI");
      }
    }
  }

  // 3. TRACKING POSISI & AUTO-LEVELING
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
    if (abs(dZ) >= 1.5) { // Filter Noise Z
      posisiZ += dZ / 360.0 * MM_PER_REV_Z;
    }

    // Filter Filter (LPF) agar stabil
    posisiZFiltered = (posisiZFiltered * 0.7) + (posisiZ * 0.3);

    lastAngleX = aX; lastAngleY = aY; lastAngleZ = aZ;

    // --- LOGIKA AUTO-LEVELING & MULTI-PASS ---
    if (isLockedEngraving) {
      
      // A. Logika Deteksi Layer (Kembali ke X0 Y0)
      float gateX = 0.0, gateY = 0.0, tolerance = 3.0;
      if (abs(posisiX - gateX) > 10.0 || abs(posisiY - gateY) > 10.0) {
          hasLeftGate = true; 
      }
      if (hasLeftGate && abs(posisiX - gateX) < tolerance && abs(posisiY - gateY) < tolerance) {
          if (currentMillis - lastPassTime > 5000) { // Debounce 5 detik
              if (zOffset < targetTotalDepth) {
                  zOffset += incrementPerPass;
                  Serial.printf("\n>>> LAYER SELESAI. Turun ke: %.2f mm\n", zOffset);
              }
              hasLeftGate = false; 
              lastPassTime = currentMillis;
          }
      }

      // B. Cari Titik Terdekat di EEPROM
      float minDist = 9999.0;
      float targetZSurface = 0;
      for (int i = 0; i < MAX_COUNTER; i++) {
        float ex = readFloat(i * 12 + 0);
        float ey = readFloat(i * 12 + 4);
        float dist = abs(posisiX - ex) + abs(posisiY - ey);
        if (dist < minDist) {
          minDist = dist;
          targetZSurface = readFloat(i * 12 + 8);
        }
      }

      // C. Eksekusi Koreksi Motor
      float errorZ = (targetZSurface - zOffset) - posisiZFiltered;
      float fOut = hitungFuzzySugeno(errorZ);

      if (abs(errorZ) > 0.15) { // Toleransi stabil
        digitalWrite(pinDIR_Z_AL, fOut > 0 ? HIGH : LOW);
        int speedHz = (int)abs(fOut);
        if (speedHz < 200) speedHz = 200; 
        tone(pinSTEP_Z_AL, speedHz);
      } else {
        noTone(pinSTEP_Z_AL);
      }
    }

    // --- TAMPILAN SERIAL MONITOR ---
    if (!showTableStatus || isLockedEngraving) {
      int newX = (int)posisiX;
      int newY = (int)posisiY;
      int newZ = (int)posisiZ;
      
      if (abs(newX - outX) > HOLD_STEP) outX = newX;
      if (abs(newY - outY) > HOLD_STEP) outY = newY;
      if (abs(newZ - outZ) > HOLD_STEP) outZ = newZ;

      Serial.print("X: "); Serial.print(outX); 
      Serial.print(" Y: "); Serial.print(outY); 
      Serial.print(" Z: "); Serial.println(outZ);
    }
  }
}