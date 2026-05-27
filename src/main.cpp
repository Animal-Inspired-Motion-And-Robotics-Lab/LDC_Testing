#include <Arduino.h>
#include "LED.h"
#include "crack_detection.h"
#include "ldc1101.h"
#include "measurement_arrays.h"
#include "serial_commands.h"
#include "telemetry.h"

const char* fw_version = "0.2.5";

//Default delay between readings (reconfigure over serial)
static constexpr uint32_t kDefaultReadingDelayMs = 25;

//DEFAULTS (reconfigure over serial)
//For the stacked inductors, L = 11.8, 42.6, 90.0 uH
static constexpr float kSensorL_H = 90.00e-6f; //uH = 1e-6H
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
  ledInit(kLedPin, kLedActiveHigh); //LED initialization
  ledFlash(10, 150); // Quick boot indication
  delay(5000); //Startup delay
  Serial.print("LDC Testing, FW Version: ");Serial. println(fw_version);

  //Set up the serial command interface
  serial_command_config_t commandConfig = {kSensorL_H, kSensorC_F, kSensorQ,
      kSwitchEnable, kSwitchGpio};
  serial_command_state_t initialState = {LDC1101_MODE_RP_L, LDC_SPEED_BALANCED_1,
      true, false, true, false, kDefaultReadingDelayMs};
  
  //Initialize the LDC1101; serialCommandsInit() pushes the seeded
  //sensor/mode/speed values to the chip via ldc1101_configure().
  ldc1101_init();
  serialCommandsInit(&commandConfig, &initialState);

  //Add smoothing for incoming data
  setFilterWindow(25); //Set to 1 for raw data pass-through

  crack_detection_config_t crackConfig = {
      0.01f,  // threshold above rotated x-axis
      110,   // window_samples (change as robot speed changes)
      0.5f, // min_parabola_r2 (goodness of fit)
      0.785f, // min_phase_angle_rad (pi/4)
      3.14f,  // max_phase_angle_rad (pi)
      220.0f   // length_estimate_scale (thou per uH)
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
    bool crackDetected = crackDetectionCheck(&crackResult);

    //Check for cracks only if calibrated and rotated
    if (state.rotated) {
      if (crackDetected) {
        ledFlash(3, 20);
      }
    }

    telemetryEmitSample(now, &state, crackDetected, &crackResult);
    
  }

}