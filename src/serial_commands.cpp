#include "serial_commands.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "LED.h"
#include "calibration.h"
#include "crack_detection.h"
#include "measurement_arrays.h"

namespace {

static serial_command_config_t gConfig;
static serial_command_state_t gState;
static bool gInitialized = false;

static constexpr size_t kCommandBufferLen = 96;
static constexpr long kMaxCalibrationSamples = 50000;
static constexpr long kMaxReadingDelayMs = 60000;
static constexpr float kMicroToBase = 1.0e-6f;
static constexpr float kPicoToBase = 1.0e-12f;
static char gCommandBuffer[kCommandBufferLen];
static size_t gCommandLength = 0;

const char* modeToString(ldc1101_mode_t mode) {
  return mode == LDC1101_MODE_LHR ? "lhr" : "lrp";
}

const char* speedToString(ldc_speed_mode_t speed) {
  switch (speed) {
    case LDC_SPEED_ACCURACY_MAX: return "accuracy";
    case LDC_SPEED_BALANCED_1: return "balanced1";
    case LDC_SPEED_BALANCED_2: return "balanced2";
    case LDC_SPEED_FAST:
    default:
      return "fast";
  }
}

void configureSensor() {
  ldc1101_configure(gConfig.sensor_l_h, gConfig.sensor_c_f, gConfig.sensor_q,
      gState.mode, gState.speed_mode,
      gConfig.switch_enable, gConfig.switch_gpio);
}

// Help and status mirror each other: same fields, same order, same names.
// Each `<name>` command shown here corresponds 1:1 to a `<name>=...` token
// printed by `status`.
void printHelp() {
  Serial.println("commands:");
  Serial.println("  help                       // show this list");
  Serial.println("  status                     // print all current values");

  // Sensor parameters.
  Serial.println("  l_h [uH]                   // sensor inductance");
  Serial.println("  c_f [pF]                   // sensor capacitance");
  Serial.println("  q [ratio]                  // sensor quality factor");

  // Reading mode.
  Serial.println("  mode lrp|lhr               // measurement mode");
  Serial.println("  speed accuracy|balanced1|balanced2|fast");
  Serial.println("  stream on|off              // streaming output on/off");
  Serial.println("  delay <ms>                 // sample period");

  // Signal processing / calibration.
  Serial.println("  smoothing <n>              // moving-average window");
  Serial.println("  rotated on|off             // apply calibrated rotation");
  Serial.println("  angle [radians]            // get/set rotation angle");
  Serial.println("  calibrate [samples]        // run PCA calibration on recent samples");

  // Crack detection tuning.
  Serial.println("  crack_window [n]           // parabola fit window samples");
  Serial.println("  crack_threshold [val]      // min parabola peak above baseline");
  Serial.println("  crack_r2 [0..1]            // min parabola R^2 fit");
  Serial.println("  crack_phase [min max]      // allowed phase angle range (rad)");
  Serial.println("  crack_scale [scale]        // peak->length scale factor");
  Serial.println("  crack_output on|off        // emit per-crack >mag >crack_x >crack_size >width");
  Serial.println("  crack_debug on|off         // print crack detector debug state");
}

void printStatus() {
  // Sensor parameters. Print in the same units the commands accept.
  Serial.print("l_h=");
  Serial.print(gConfig.sensor_l_h / kMicroToBase, 6);
  Serial.print(" c_f=");
  Serial.print(gConfig.sensor_c_f / kPicoToBase, 6);
  Serial.print(" q=");
  Serial.println(gConfig.sensor_q, 6);

  // Reading mode.
  Serial.print("mode=");
  Serial.print(modeToString(gState.mode));
  Serial.print(" speed=");
  Serial.print(speedToString(gState.speed_mode));
  Serial.print(" stream=");
  Serial.print(gState.streaming_enabled ? "on" : "off");
  Serial.print(" delay=");
  Serial.println((unsigned long)gState.reading_delay_ms);

  // Signal processing.
  Serial.print("smoothing=");
  Serial.print((unsigned int)getFilterWindow());
  Serial.print(" rotated=");
  Serial.print(gState.rotated ? "on" : "off");
  Serial.print(" angle=");
  Serial.println(getRotationAngle(), 6);

  // Crack detection.
  Serial.print("crack_window=");
  Serial.print((unsigned int)crackDetectionGetWindowSamples());
  Serial.print(" crack_threshold=");
  Serial.print(crackDetectionGetThreshold(), 6);
  Serial.print(" crack_r2=");
  Serial.println(crackDetectionGetMinParabolaR2(), 6);

  Serial.print("crack_phase_min=");
  Serial.print(crackDetectionGetMinPhaseAngleRad(), 6);
  Serial.print(" crack_phase_max=");
  Serial.print(crackDetectionGetMaxPhaseAngleRad(), 6);
  Serial.print(" crack_scale=");
  Serial.println(crackDetectionGetLengthEstimateScale(), 6);

  Serial.print("crack_output=");
  Serial.print(gState.crack_output ? "on" : "off");
  Serial.print(" crack_debug=");
  Serial.println(gState.crack_debug_output ? "on" : "off");
}

void processCommand(char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {return;}

  char* token = strtok(line, " \t");
  if (token == nullptr) {return;}

  if (strcmp(token, "help") == 0) {printHelp(); return;}

  if (strcmp(token, "status") == 0) {printStatus(); return;}

  if (strcmp(token, "l_h") == 0 || strcmp(token, "lh") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Serial.println("ERR l_h must be > 0 (uH)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: l_h [uH]");
        return;
      }
      gConfig.sensor_l_h = parsed * kMicroToBase;
      configureSensor();
    }

    Serial.print("l_h=");
    Serial.println(gConfig.sensor_l_h / kMicroToBase, 6);
    return;
  }

  if (strcmp(token, "c_f") == 0 || strcmp(token, "cf") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Serial.println("ERR c_f must be > 0 (pF)");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: c_f [pF]");
        return;
      }
      gConfig.sensor_c_f = parsed * kPicoToBase;
      configureSensor();
    }

    Serial.print("c_f=");
    Serial.println(gConfig.sensor_c_f / kPicoToBase, 6);
    return;
  }

  if (strcmp(token, "q") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed <= 0.0f) {
        Serial.println("ERR q must be > 0");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: q [value]");
        return;
      }
      gConfig.sensor_q = parsed;
      configureSensor();
    }

    Serial.print("q=");
    Serial.println(gConfig.sensor_q, 6);
    return;
  }

  if (strcmp(token, "angle") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0') {
        Serial.println("ERR usage: angle [radians]");
        return;
      }
      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: angle [radians]");
        return;
      }
      setRotationAngle(parsed);
    }

    Serial.print("angle=");
    Serial.println(getRotationAngle(), 6);
    return;
  }

  if (strcmp(token, "stream") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: stream on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.streaming_enabled = true;
      Serial.println("OK stream on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.streaming_enabled = false;
      Serial.println("OK stream off");
      return;
    }
    Serial.println("ERR usage: stream on|off");
    return;
  }

  if (strcmp(token, "delay") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: delay <ms>");
      return;
    }

    long parsed = strtol(value, nullptr, 10);
    if (parsed <= 0 || parsed > kMaxReadingDelayMs) {
      Serial.print("ERR delay must be 1..");
      Serial.println(kMaxReadingDelayMs);
      return;
    }

    gState.reading_delay_ms = (uint32_t)parsed;
    Serial.print("OK delay ");
    Serial.println((unsigned long)gState.reading_delay_ms);
    return;
  }

  if (strcmp(token, "smoothing") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: smoothing <n>");
      return;
    }

    long parsed = strtol(value, nullptr, 10);
    if (parsed <= 0) {
      Serial.println("ERR smoothing must be >= 1");
      return;
    }

    setFilterWindow((size_t)parsed);
    Serial.print("OK smoothing ");
    Serial.println((unsigned int)getFilterWindow());
    return;
  }

  if (strcmp(token, "crack_window") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      long parsed = strtol(value, nullptr, 10);
      if (parsed <= 0) {
        Serial.println("ERR crack_window must be >= 1");
        return;
      }
      crackDetectionSetWindowSamples((size_t)parsed);
    }

    Serial.print("crack_window=");
    Serial.println((unsigned int)crackDetectionGetWindowSamples());
    return;
  }

  if (strcmp(token, "crack_threshold") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Serial.println("ERR crack_threshold must be >= 0");
        return;
      }
      crackDetectionSetThreshold(parsed);
    }

    Serial.print("crack_threshold=");
    Serial.println(crackDetectionGetThreshold(), 6);
    return;
  }

  if (strcmp(token, "crack_r2") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f || parsed > 1.0f) {
        Serial.println("ERR crack_r2 must be 0..1");
        return;
      }
      crackDetectionSetMinParabolaR2(parsed);
    }

    Serial.print("crack_r2=");
    Serial.println(crackDetectionGetMinParabolaR2(), 6);
    return;
  }

  if (strcmp(token, "crack_phase") == 0) {
    char* minValue = strtok(nullptr, " \t");
    if (minValue != nullptr) {
      char* maxValue = strtok(nullptr, " \t");
      if (maxValue == nullptr) {
        Serial.println("ERR usage: crack_phase [min max]");
        return;
      }

      char* minEnd = nullptr;
      char* maxEnd = nullptr;
      float parsedMin = strtof(minValue, &minEnd);
      float parsedMax = strtof(maxValue, &maxEnd);
      if (minEnd == minValue || *minEnd != '\0' ||
          maxEnd == maxValue || *maxEnd != '\0') {
        Serial.println("ERR usage: crack_phase [min max]");
        return;
      }

      char* extra = strtok(nullptr, " \t");
      if (extra != nullptr) {
        Serial.println("ERR usage: crack_phase [min max]");
        return;
      }

      crackDetectionSetPhaseAngleRange(parsedMin, parsedMax);
    }

    Serial.print("crack_phase_min=");
    Serial.print(crackDetectionGetMinPhaseAngleRad(), 6);
    Serial.print(" crack_phase_max=");
    Serial.println(crackDetectionGetMaxPhaseAngleRad(), 6);
    return;
  }

  if (strcmp(token, "crack_scale") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value != nullptr) {
      char* end = nullptr;
      float parsed = strtof(value, &end);
      if (end == value || *end != '\0' || parsed < 0.0f) {
        Serial.println("ERR crack_scale must be >= 0");
        return;
      }
      crackDetectionSetLengthEstimateScale(parsed);
    }

    Serial.print("crack_scale=");
    Serial.println(crackDetectionGetLengthEstimateScale(), 6);
    return;
  }

  if (strcmp(token, "crack_debug") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: crack_debug on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.crack_debug_output = true;
      Serial.println("OK crack_debug on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.crack_debug_output = false;
      Serial.println("OK crack_debug off");
      return;
    }
    Serial.println("ERR usage: crack_debug on|off");
    return;
  }

  if (strcmp(token, "crack_output") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: crack_output on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.crack_output = true;
      Serial.println("OK crack_output on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.crack_output = false;
      Serial.println("OK crack_output off");
      return;
    }
    Serial.println("ERR usage: crack_output on|off");
    return;
  }

  if (strcmp(token, "mode") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: mode lrp|lhr");
      return;
    }

    ldc1101_mode_t newMode;
    if (strcmp(value, "lrp") == 0) {
      newMode = LDC1101_MODE_RP_L;
    } else if (strcmp(value, "lhr") == 0) {
      newMode = LDC1101_MODE_LHR;
    } else {
      Serial.println("ERR usage: mode lrp|lhr");
      return;
    }

    gState.mode = newMode;
    configureSensor();
    Serial.print("OK mode ");
    Serial.println(modeToString(gState.mode));
    return;
  }

  if (strcmp(token, "speed") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: speed accuracy|balanced1|balanced2|fast");
      return;
    }

    ldc_speed_mode_t newSpeed;
    if (strcmp(value, "accuracy") == 0) {
      newSpeed = LDC_SPEED_ACCURACY_MAX;
    } else if (strcmp(value, "balanced1") == 0 || strcmp(value, "bal1") == 0) {
      newSpeed = LDC_SPEED_BALANCED_1;
    } else if (strcmp(value, "balanced2") == 0 || strcmp(value, "bal2") == 0) {
      newSpeed = LDC_SPEED_BALANCED_2;
    } else if (strcmp(value, "fast") == 0) {
      newSpeed = LDC_SPEED_FAST;
    } else {
      Serial.println("ERR usage: speed accuracy|balanced1|balanced2|fast");
      return;
    }

    gState.speed_mode = newSpeed;
    configureSensor();
    Serial.print("OK speed ");
    Serial.println(speedToString(gState.speed_mode));
    return;
  }

  if (strcmp(token, "calibrate") == 0) {
    char* value = strtok(nullptr, " \t");
    size_t sampleCount = 40;
    if (value != nullptr) {
      long parsed = strtol(value, nullptr, 10);
      if (parsed <= 0 || parsed > kMaxCalibrationSamples) {
        Serial.print("ERR calibrate samples must be 1..");
        Serial.println(kMaxCalibrationSamples);
        return;
      }
      sampleCount = (size_t)parsed;
    }
    ledFlash(10, 30);
    Serial.print("calibrate start samples=");
    Serial.println((unsigned int)sampleCount);
    calibration_result_t result = calibrationRun(gConfig.sensor_c_f, sampleCount);
    calibrationPrintResult(&result);
    return;
  }

    if (strcmp(token, "rotate") == 0 ||
        strcmp(token, "rotated") == 0 ||
        strcmp(token, "rotation") == 0) {
    char* value = strtok(nullptr, " \t");
    if (value == nullptr) {
      Serial.println("ERR usage: rotated on|off");
      return;
    }
    if (strcmp(value, "on") == 0) {
      gState.rotated = true;
      setRotationEnabled(true);
      Serial.println("OK rotated on");
      return;
    }
    if (strcmp(value, "off") == 0) {
      gState.rotated = false;
      setRotationEnabled(false);
      Serial.println("OK rotated off");
      return;
    }
    Serial.println("ERR usage: rotated on|off");
    return;
  }

  Serial.print("ERR unknown command: ");
  Serial.println(token);
}

}  // namespace

void serialCommandsInit(const serial_command_config_t* config,
                        const serial_command_state_t* initial_state) {
  if (config == nullptr || initial_state == nullptr) {
    return;
  }

  gConfig = *config;
  gState = *initial_state;
  setRotationEnabled(gState.rotated);
  gCommandLength = 0;
  gInitialized = true;

  configureSensor();

  printHelp();
}

void serialCommandsPoll(void) {
  if (!gInitialized) {
    return;
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      gCommandBuffer[gCommandLength] = '\0';
      processCommand(gCommandBuffer);
      gCommandLength = 0;
      continue;
    }

    if (gCommandLength < (kCommandBufferLen - 1)) {
      gCommandBuffer[gCommandLength++] = c;
    }
  }
}

serial_command_state_t serialCommandsGetState(void) {
  return gState;
}

serial_command_config_t serialCommandsGetConfig(void) {
  return gConfig;
}
