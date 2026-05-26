#include <Arduino.h>
#include "LED.h"
#include "crack_detection.h"
#include "ldc1101.h"
#include "measurement_arrays.h"
#include "serial_commands.h"

const char* fw_version = "0.2.2";

//Default delay between readings (reconfigure over serial)
static constexpr uint32_t kDefaultReadingDelayMs = 25;

//DEFAULTS (reconfigure over serial)
//For the stacked inductors, L = 11.8, 42.6, 90.0 uH
static constexpr float kSensorL_H = 11.84e-6f; //uH = 1e-6H
static constexpr float kSensorC_F = 220e-12f; //pF = 1e-12F

//For the stacked inductors, modeled Q values are 23.6, 24.6, 25.6
//with a 220pF capacitor
static constexpr float kSensorQ = 20.0f; // Quality factor

//Switch configuration
static constexpr int kSwitchEnable = 0;
static constexpr int kSwitchGpio = -1;

//LED Feedback
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
      true, false, false, kDefaultReadingDelayMs};
  
  //Initialize the LDC1101
  ldc1101_init();
  ldc1101_configure(kSensorL_H, kSensorC_F, kSensorQ,
      LDC1101_MODE_RP_L, LDC_SPEED_BALANCED_1,
      0, -1);
  serialCommandsInit(&commandConfig, &initialState);

  //Add smoothing for incoming data
  setFilterWindow(10); //Set to 1 for raw data pass-through

  crack_detection_config_t crackConfig = {
      5.0f,  // min_vector_magnitude
      1.57f, // min_phase_angle_rad (pi/2)
      3.14f, // max_phase_angle_rad (pi)
      1000,  // cooldown_ms
      100,   // window_samples
      1.0f   // length_estimate_scale
  };
  crackDetectionInit(&crackConfig);

  Serial.println("LDC1101 initialized");
}

void loop() {

  serialCommandsPoll(); //Check for incoming serial data
  serial_command_state_t state = serialCommandsGetState();
  serial_command_config_t config = serialCommandsGetConfig();
  if (!state.streaming_enabled) {delay(2); return;} //Skip if streaming is disabled

  uint32_t now = millis(); //Timestamp

  //If enough time has passed, print the latest filtered measurements
  if (now - lastPrintMs >= state.reading_delay_ms) {
    lastPrintMs = now;
      ldc1101_measurement_t m = ldc1101_read(config.sensor_c_f);
      appendMeasurement(m.Rp_ohms, m.L_uH); //Add the new measurements to their arrays

    crack_detection_result_t crackResult = {};
    bool crackDetected = crackDetectionCheck(now, &crackResult);

    if (state.rotated) {
      if (crackDetected) {
        ledFlash(3, 20);
      }
    }

    //Print out either rotated or unrotated values
    float rpToPrint = state.rotated ? getLatestRotatedRp() : getLatestFilteredRp();
    float lToPrint = state.rotated ? getLatestRotatedL() : getLatestFilteredL();
    float crackToPrint = crackDetected ? crackResult.total_length_estimate : 0.0f;
    if (state.mode == LDC1101_MODE_LHR) { rpToPrint = 0.0f; }
    Serial.print(">Rp:"); Serial.print(rpToPrint, 3);
    Serial.print(">L:"); Serial.print(lToPrint, 6);
    Serial.print(">crack:"); Serial.print(crackToPrint, 6);
    Serial.print(">t:"); Serial.print(now);
    Serial.println("|xy"); //Indicates x-y values for Teleplot

    if (state.crack_debug_output) {
      Serial.print("crack det="); Serial.print(crackResult.detected ? 1 : 0);
      Serial.print(" mag="); Serial.print(crackResult.vector_magnitude, 6);
      Serial.print(" phase="); Serial.print(crackResult.phase_angle_rad, 6);
      Serial.print(" vrp="); Serial.print(crackResult.vector_rp_ohms, 3);
      Serial.print(" vl="); Serial.print(crackResult.vector_l_uH, 6);
      Serial.print(" crack_total="); Serial.print(crackResult.total_length_estimate, 6);
      Serial.print(" threshold="); Serial.print(crackDetectionGetMinVectorMagnitude(), 6);
      Serial.print(" window="); Serial.print((unsigned int)crackDetectionGetWindowSamples());
      Serial.print(" crack_scale="); Serial.print(crackDetectionGetLengthEstimateScale(), 6);
      Serial.print(" rotated="); Serial.println(state.rotated ? "on" : "off");
    }
  }

}