#include <Arduino.h>
#include "ldc1101.h"
#include "measurement_arrays.h"

const char* fw_version = "0.1.0";

static constexpr int reading_delay_ms = 25;

//For the stacked inductors, L = 11.8, 42.6, 90.0 uH
static constexpr float kSensorL_H = 42.6e-6f; //uH = 1e-6H
static constexpr float kSensorC_F = 660e-12f; //pF = 1e-12F

//For the stacked inductors, modeled Q values are 23.6, 24.6, 25.6
//with a 220pF capacitor
static constexpr float kSensorQ = 15.0f; // Quality factor

static uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(9600); //Serial connection
  delay(5000); //Startup delay
  Serial.print("LDC Testing, FW Version: ");Serial. println(fw_version);
  
  //Initialize the LDC1101
  ldc1101_init();
  ldc1101_configure(
      kSensorL_H,
      kSensorC_F,
      kSensorQ,
      LDC1101_MODE_RP_L,
      LDC_SPEED_BALANCED_1,
      0,
      -1);

  //Add a filter window for incoming data
  setFilterWindow(10); //Set to 1 for raw data pass-through

  Serial.println("LDC1101 initialized");
}

void loop() {
  ldc1101_measurement_t m = ldc1101_read(kSensorC_F);
  appendMeasurement(m.Rp_ohms, m.L_uH); //Add the new measurements to their arrays

  uint32_t now = millis();
  if (now - lastPrintMs >= reading_delay_ms) {
    lastPrintMs = now;
    Serial.print(">Rp:"); Serial.print(getLatestFilteredRp(), 3);
    Serial.print(">L:"); Serial.print(getLatestFilteredL(), 6);
    Serial.print(">t:"); Serial.print(now);
    Serial.println("|xy"); //Indicates x-y values for Teleplot
    //Serial.println(calculateDominantAngle()); //Print the dominant angle in radians
    //printMeasurementArrays();
  }

}