#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdbool.h>
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
  SYSTEM_STATUS_SENSOR_BELOW_MIN
} system_status_t;

typedef struct
{
  uint32_t timestamp_ms;
  uint16_t distance_mm;            /* đơn vị milimét */
  warning_level_t warning_level;
  system_status_t system_status;
  uint8_t is_object_detected;
} system_state_t;

bool lcd_display_init(void);
void lcd_display_show_boot(void);
void lcd_display_show_state(const system_state_t *sensor1_state,
                            const system_state_t *sensor2_state);

#endif /* LCD_DISPLAY_H */