#include "system_state.h"

#include "stm32f1xx_hal.h"

static system_state_t xCurrentSystemState;

void system_state_init(void)
{
  xCurrentSystemState.timestamp_ms = 0U;
  xCurrentSystemState.distance_cm = 0.0f;
  xCurrentSystemState.warning_level = WARNING_LEVEL_SAFE;
  xCurrentSystemState.system_status = SYSTEM_STATUS_OK;
  xCurrentSystemState.is_object_detected = 0U;
}

void system_state_update(const system_state_t *new_state)
{
  uint32_t primask;

  if (new_state == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  xCurrentSystemState = *new_state;
  __set_PRIMASK(primask);
}

void system_state_get(system_state_t *current_state)
{
  uint32_t primask;

  if (current_state == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *current_state = xCurrentSystemState;
  __set_PRIMASK(primask);
}

const char *system_state_get_warning_text(warning_level_t warning_level)
{
  switch (warning_level)
  {
    case WARNING_LEVEL_SAFE:
      return "SAFE";

    case WARNING_LEVEL_WARNING:
      return "WARNING";

    case WARNING_LEVEL_DANGER:
      return "DANGER";

    default:
      return "UNKNOWN";
  }
}

const char *system_state_get_status_text(system_status_t system_status)
{
  switch (system_status)
  {
    case SYSTEM_STATUS_OK:
      return "OK";

    case SYSTEM_STATUS_SENSOR_TIMEOUT:
      return "TIMEOUT";

    case SYSTEM_STATUS_SENSOR_FAULT:
      return "SENSOR_FAULT";

    case SYSTEM_STATUS_SENSOR_BELOW_MIN:
      return "UNDER_MIN";

    case SYSTEM_STATUS_UART_ERROR:
      return "UART_ERROR";

    default:
      return "UNKNOWN";
  }
}
