#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <stdbool.h>

#include "ldc1101.h"

typedef struct {
  float sensor_l_h;
  float sensor_c_f;
  float sensor_q;
  int switch_enable;
  int switch_gpio;
} serial_command_config_t;

typedef struct {
  ldc1101_mode_t mode;
  ldc_speed_mode_t speed_mode;
  bool streaming_enabled;
  bool output_rotated;
  uint32_t reading_delay_ms;
} serial_command_state_t;

void serialCommandsInit(const serial_command_config_t* config,
                        const serial_command_state_t* initial_state);

void serialCommandsPoll(void);

serial_command_state_t serialCommandsGetState(void);

#endif
