#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "crack_detection.h"
#include "serial_commands.h"

void telemetryEmitSample(uint32_t now_ms,
                         const serial_command_state_t* state,
                         bool crack_detected,
                         const crack_detection_result_t* crack);

#endif