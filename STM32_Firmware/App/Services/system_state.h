#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>

typedef enum
{
  WARNING_LEVEL_SAFE = 0,
  WARNING_LEVEL_WARNING,
  WARNING_LEVEL_DANGER
} warning_level_t;

typedef enum
{
  SYSTEM_STATUS_OK = 0,
  SYSTEM_STATUS_SENSOR_TIMEOUT,
  SYSTEM_STATUS_SENSOR_FAULT,
  SYSTEM_STATUS_SENSOR_BELOW_MIN,
  SYSTEM_STATUS_UART_ERROR
} system_status_t;

typedef struct
{
  uint32_t timestamp_ms;
  float distance_cm;
  uint8_t is_object_detected;
  system_status_t system_status;
} distance_data_t;

typedef struct
{
  uint32_t timestamp_ms;
  float distance_cm;
  warning_level_t warning_level;
  system_status_t system_status;
  uint8_t is_object_detected;
} system_state_t;

void system_state_init(void);
void system_state_update(const system_state_t *new_state);
void system_state_get(system_state_t *current_state);
const char *system_state_get_warning_text(warning_level_t warning_level);
const char *system_state_get_status_text(system_status_t system_status);

#endif /* SYSTEM_STATE_H */
