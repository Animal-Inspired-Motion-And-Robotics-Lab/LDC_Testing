#include <Arduino.h>
#include "LED.h"
#include "ldc1101.h"
#include "measurement_arrays.h"
#include "serial_commands.h"

const char* fw_version = "0.2.1";

static constexpr uint32_t kDefaultReadingDelayMs = 250;

//For the stacked inductors, L = 11.8, 42.6, 90.0 uH
static constexpr float kSensorL_H = 42.6e-6f; //uH = 1e-6H
static constexpr float kSensorC_F = 100e-12f; //pF = 1e-12F

//For the stacked inductors, modeled Q values are 23.6, 24.6, 25.6
//with a 220pF capacitor
static constexpr float kSensorQ = 30.0f; // Quality factor
static constexpr int kSwitchEnable = 0;
static constexpr int kSwitchGpio = -1;
static constexpr int kLedPin = LED_BUILTIN;
static constexpr bool kLedActiveHigh = false;

static uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(9600); //Serial connection
  ledInit(kLedPin, kLedActiveHigh);
  ledFlash(10, 150); // Quick boot indication
  delay(5000); //Startup delay
  Serial.print("LDC Testing, FW Version: ");Serial. println(fw_version);

  //Set up the serial command interface
  serial_command_config_t commandConfig = {kSensorL_H, kSensorC_F, kSensorQ,
      kSwitchEnable, kSwitchGpio};
  serial_command_state_t initialState = {LDC1101_MODE_RP_L, LDC_SPEED_BALANCED_1,
      true, false, kDefaultReadingDelayMs};
  
  //Initialize the LDC1101
  ldc1101_init();
  ldc1101_configure(kSensorL_H, kSensorC_F, kSensorQ,
      LDC1101_MODE_RP_L, LDC_SPEED_BALANCED_1,
      0, -1);
  serialCommandsInit(&commandConfig, &initialState);

  //Add smoothing for incoming data
  setFilterWindow(10); //Set to 1 for raw data pass-through

  Serial.println("LDC1101 initialized");
}

void loop() {

  serialCommandsPoll(); //Check for incoming serial data
  serial_command_state_t state = serialCommandsGetState();
  if (!state.streaming_enabled) {delay(2); return;} //Skip if streaming is disabled

  uint32_t now = millis(); //Timestamp

  //If enough time has passed, print the latest filtered measurements
  if (now - lastPrintMs >= state.reading_delay_ms) {
    lastPrintMs = now;
      ldc1101_measurement_t m = ldc1101_read(kSensorC_F);
      appendMeasurement(m.Rp_ohms, m.L_uH); //Add the new measurements to their arrays

    //Print out either rotated or unrotated values
    float rpToPrint = state.rotated ? getLatestRotatedRp() : getLatestFilteredRp();
    float lToPrint = state.rotated ? getLatestRotatedL() : getLatestFilteredL();
    Serial.print(">Rp:"); Serial.print(rpToPrint, 3);
    Serial.print(">L:"); Serial.print(lToPrint, 6);
    Serial.print(">t:"); Serial.print(now);
    Serial.println("|xy"); //Indicates x-y values for Teleplot
  }

}