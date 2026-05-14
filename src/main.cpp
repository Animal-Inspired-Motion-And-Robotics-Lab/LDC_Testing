#include <Arduino.h>
#include "ldc1101.h"
#include "measurement_arrays.h"

const char* fw_version = "0.1.0";

static constexpr int reading_delay_ms = 25;

//For the stacked inductors, 11.8, 42.6, 90.0
static constexpr float kSensorL_H = 12e-6f; //uH = 1e-6H

static constexpr float kSensorC_F = 220e-12f; //pF = 1e-12F

//For the stacked inductors, 1.7, 4.6, 6.2
static constexpr float kSensorQ = 30.0f; // Quality factor (unused)

static uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.print("LDC Testing, FW Version: ");Serial. println(fw_version);
  
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
  //appendMeasurement(m.Rp_ohms, m.L_uH); //Add the new measurements to their arrays

  uint32_t now = millis();
  if (now - lastPrintMs >= reading_delay_ms) {
    lastPrintMs = now;
    Serial.print(">Rp:"); Serial.print(m.Rp_ohms, 3);
    Serial.print(">L:"); Serial.print(m.L_uH, 6);
    Serial.print(">t:"); Serial.print(now);
    Serial.println("|xy"); //Indicates x-y values for Teleplot
    //Serial.println(calculateDominantAngle()); //Print the dominant angle in radians
    //printMeasurementArrays();
  }

}