#include "lcd_display.h"

#include <stdio.h>
#include <string.h>

#include "main.h"

#define LCD_I2C_ADDRESS             (0x27U << 1)
#define LCD_COLUMN_COUNT            16U
#define LCD_ROW_COUNT                2U
#define LCD_BOOT_DELAY_MS           50U

#define LCD_ENABLE_BIT              0x04U
#define LCD_REGISTER_SELECT_BIT     0x01U
#define LCD_BACKLIGHT_BIT           0x08U

#define LCD_CMD_CLEAR               0x01U
#define LCD_CMD_HOME                0x02U
#define LCD_CMD_ENTRY_MODE          0x06U
#define LCD_CMD_DISPLAY_ON          0x0CU
#define LCD_CMD_FUNCTION_SET        0x28U
#define LCD_CMD_SET_DDRAM           0x80U

#define LCD_MODE_COMMAND            0U
#define LCD_MODE_DATA               1U

typedef struct
{
  I2C_HandleTypeDef *i2c;
  uint8_t address;
  uint8_t columns;
  uint8_t rows;
  uint8_t backlight;
} lcd_i2c_t;

static lcd_i2c_t lcd;
static uint8_t lcd_ready;

static void lcd_i2c_write(uint8_t value, uint8_t mode)
{
  uint8_t high = (uint8_t)(value & 0xF0U);
  uint8_t low = (uint8_t)((value << 4) & 0xF0U);
  uint8_t frame[4];

  if (lcd.backlight != 0U)
  {
    high |= LCD_BACKLIGHT_BIT;
    low |= LCD_BACKLIGHT_BIT;
  }

  if (mode == LCD_MODE_DATA)
  {
    high |= LCD_REGISTER_SELECT_BIT;
    low |= LCD_REGISTER_SELECT_BIT;
  }

  frame[0] = (uint8_t)(high | LCD_ENABLE_BIT);
  frame[1] = high;
  frame[2] = (uint8_t)(low | LCD_ENABLE_BIT);
  frame[3] = low;

  (void)HAL_I2C_Master_Transmit(lcd.i2c, lcd.address, frame, sizeof(frame), 100U);
}

static void lcd_command(uint8_t command)
{
  lcd_i2c_write(command, LCD_MODE_COMMAND);
}

static void lcd_data(char data)
{
  lcd_i2c_write((uint8_t)data, LCD_MODE_DATA);
}

static void lcd_set_cursor(uint8_t column, uint8_t row)
{
  static const uint8_t row_offset[] = {0x00U, 0x40U, 0x14U, 0x54U};

  if (column >= lcd.columns)
  {
    column = (uint8_t)(lcd.columns - 1U);
  }

  if (row >= lcd.rows)
  {
    row = (uint8_t)(lcd.rows - 1U);
  }

  lcd_command((uint8_t)(LCD_CMD_SET_DDRAM | (row_offset[row] + column)));
}

static void lcd_write_line(uint8_t row, const char *text)
{
  char line[LCD_COLUMN_COUNT + 1U];
  uint8_t column;

  memset(line, ' ', LCD_COLUMN_COUNT);
  line[LCD_COLUMN_COUNT] = '\0';

  if (text != NULL)
  {
    for (column = 0U; (column < LCD_COLUMN_COUNT) && (text[column] != '\0'); column++)
    {
      line[column] = text[column];
    }
  }

  lcd_set_cursor(0U, row);

  for (column = 0U; column < LCD_COLUMN_COUNT; column++)
  {
    lcd_data(line[column]);
  }
}

static void lcd_copy_slot(char slot[9], const char *text)
{
  uint8_t i;

  memset(slot, ' ', 8U);
  slot[8] = '\0';

  if (text == NULL)
  {
    return;
  }

  for (i = 0U; (i < 8U) && (text[i] != '\0'); i++)
  {
    slot[i] = text[i];
  }
}

/* HÀM FORMAT LCD DÙNG MEMCPY */
static void lcd_format_distance_slot(char slot[9],
                                     char sensor_name,
                                     const system_state_t *state)
{
  unsigned long distance_cm;
  char value[6];

  memset(slot, ' ', 8U);
  slot[8] = '\0';
  slot[0] = 'S';
  slot[1] = sensor_name;
  slot[2] = ':';

  if (state == NULL)
  {
    memcpy(&slot[3], "---  ", 5U);
  }
  else if (state->system_status == SYSTEM_STATUS_OK)
  {
    distance_cm = (unsigned long)(state->distance_mm / 10U);
    if (distance_cm > 999UL) distance_cm = 999UL;
    
    // Ép in đúng 3 số 
    (void)snprintf(value, sizeof(value), "%03lu  ", distance_cm); 
    memcpy(&slot[3], value, 5U);
  }
  else if (state->system_status == SYSTEM_STATUS_SENSOR_BELOW_MIN)
  {
    memcpy(&slot[3], "<20cm", 5U);
  }
  else if (state->system_status == SYSTEM_STATUS_SENSOR_TIMEOUT)
  {
    memcpy(&slot[3], "NO   ", 5U); 
  }
  else
  {
    memcpy(&slot[3], "ERR  ", 5U);
  }
}

static const char *lcd_state_text(const system_state_t *state)
{
  if (state == NULL)
  {
    return "---";
  }

  switch (state->system_status)
  {
    case SYSTEM_STATUS_OK:
      if (state->warning_level == WARNING_LEVEL_DANGER) return "DANGER ";
      if (state->warning_level == WARNING_LEVEL_WARNING) return "WARN   ";
      return "SAFE   ";

    case SYSTEM_STATUS_SENSOR_TIMEOUT:
      return "TIMEOUT";

    case SYSTEM_STATUS_SENSOR_FAULT:
      return "FAULT  ";

    case SYSTEM_STATUS_SENSOR_BELOW_MIN:
      return "DANGER ";

    default:
      return "UNKNOWN";
  }
}

bool lcd_display_init(void)
{
  lcd.i2c = &hi2c1;
  lcd.address = LCD_I2C_ADDRESS;
  lcd.columns = LCD_COLUMN_COUNT;
  lcd.rows = LCD_ROW_COUNT;
  lcd.backlight = LCD_BACKLIGHT_BIT;

  HAL_Delay(LCD_BOOT_DELAY_MS);

  if (HAL_I2C_IsDeviceReady(lcd.i2c, lcd.address, 5U, 20U) != HAL_OK)
  {
    lcd_ready = 0U;
    return false;
  }

  HAL_Delay(50U);
  lcd_command(0x33U);
  lcd_command(0x32U);
  lcd_command(LCD_CMD_FUNCTION_SET);
  lcd_command(LCD_CMD_DISPLAY_ON);
  lcd_command(LCD_CMD_ENTRY_MODE);
  lcd_command(LCD_CMD_CLEAR);
  HAL_Delay(2U);
  lcd_command(LCD_CMD_HOME);
  HAL_Delay(2U);

  lcd_ready = 1U;
  lcd_display_show_boot();
  return true;
}

void lcd_display_show_boot(void)
{
  if (lcd_ready == 0U)
  {
    return;
  }
}

void lcd_display_show_state(const system_state_t *sensor1_state,
                            const system_state_t *sensor2_state)
{
  char line0[LCD_COLUMN_COUNT + 1U];
  char line1[LCD_COLUMN_COUNT + 1U];
  char left[9];
  char right[9];

  if (lcd_ready == 0U)
  {
    return;
  }

  lcd_format_distance_slot(left, '1', sensor1_state);
  lcd_format_distance_slot(right, '2', sensor2_state);
  memcpy(&line0[0], left, 8U);
  memcpy(&line0[8], right, 8U);
  line0[LCD_COLUMN_COUNT] = '\0';

  lcd_copy_slot(left, lcd_state_text(sensor1_state));
  lcd_copy_slot(right, lcd_state_text(sensor2_state));
  memcpy(&line1[0], left, 8U);
  memcpy(&line1[8], right, 8U);
  line1[LCD_COLUMN_COUNT] = '\0';

  lcd_write_line(0U, line0);
  lcd_write_line(1U, line1);
}