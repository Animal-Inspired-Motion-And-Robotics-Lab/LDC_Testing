#include <Arduino.h>
#include "ldc1101.h"

static constexpr float kSensorL_H = 11.8e-6f; //uH = 1e-6H
static constexpr float kSensorC_F = 220e-12f; //pF = 1e-12F
static constexpr float kSensorQ = 30.0f; // Quality factor (unused)

static uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(9600);
  delay(200);

  ldc1101_init();
  ldc1101_configure(
      kSensorL_H,
      kSensorC_F,
      kSensorQ,
      LDC1101_MODE_RP_L,
      LDC_SPEED_BALANCED_1,
      0,
      -1);

  Serial.println("LDC1101 initialized");
}

void loop() {
  ldc1101_measurement_t m = ldc1101_read(kSensorC_F);

  uint32_t now = millis();
  if (now - lastPrintMs >= 25) {
    lastPrintMs = now;
    Serial.print(m.Rp_ohms, 3); Serial.print('\t'); 
    Serial.println(m.L_uH, 6);
  }

  delay(50);
}