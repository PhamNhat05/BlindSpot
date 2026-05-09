#include "lcd_display.h"

#include <stdio.h>
#include <string.h>

#include "system_config.h"
#include "main.h"

/* ---- Minimal CLCD I2C implementation (linker fix) ---- */
#ifndef CLCD_I2C_MINIMAL_IMPL
#define CLCD_I2C_MINIMAL_IMPL

#define CLCD_COMMAND 0x00U
#define CLCD_DATA    0x01U

#define LCD_EN 0x04U
#define LCD_RW 0x02U
#define LCD_RS 0x01U

/* commands */
#define LCD_CLEARDISPLAY  0x01U
#define LCD_RETURNHOME   0x02U

/* function set / entry mode / display control / cursor shift */
#define LCD_ENTRYMODESET      0x04U
#define LCD_ENTRYLEFT         0x02U
#define LCD_ENTRYSHIFTDECREMENT 0x00U
#define LCD_4BITMODE          0x00U
#define LCD_2LINE             0x08U
#define LCD_5x8DOTS           0x00U
#define LCD_FUNCTIONSET      0x20U
#define LCD_DISPLAYCONTROL   0x08U
#define LCD_DISPLAYON        0x04U
#define LCD_CURSOROFF        0x00U
#define LCD_BLINKOFF         0x00U
#define LCD_CURSORSHIFT      0x10U
#define LCD_CURSORMOVE       0x00U
#define LCD_MOVERIGHT        0x04U

#define LCD_SETDDRAMADDR     0x80U
#define LCD_SETDDRAMADDR	    0x80U

#define LCD_BACKLIGHT 0x08U

#define LCD_CURSORON  0x02U
#define LCD_BLINKON   0x01U

static void CLCD_Delay(uint16_t t)
{
  HAL_Delay(t);
}

static void CLCD_WriteI2C(CLCD_I2C_Name* LCD, uint8_t Data, uint8_t Mode)
{
  uint8_t Data_H;
  uint8_t Data_L;
  uint8_t Data_I2C[4];

  Data_H = (uint8_t)(Data & 0xF0U);
  Data_L = (uint8_t)((Data << 4) & 0xF0U);

  if (LCD->BACKLIGHT != 0U)
  {
    Data_H |= LCD_BACKLIGHT;
    Data_L |= LCD_BACKLIGHT;
  }

  if (Mode == CLCD_DATA)
  {
    Data_H |= LCD_RS;
    Data_L |= LCD_RS;
  }
  else
  {
    Data_H &= (uint8_t)~LCD_RS;
    Data_L &= (uint8_t)~LCD_RS;
  }

  Data_I2C[0] = (uint8_t)(Data_H | LCD_EN);
  /* Giảm delay để tránh chiếm CPU quá lâu khi cập nhật LCD liên tục */
  CLCD_Delay(0U);
  Data_I2C[1] = Data_H;
  Data_I2C[2] = (uint8_t)(Data_L | LCD_EN);
  CLCD_Delay(0U);
  Data_I2C[3] = Data_L;

  (void)HAL_I2C_Master_Transmit(LCD->I2C, LCD->ADDRESS, Data_I2C, sizeof(Data_I2C), 1000U);
}

void CLCD_I2C_Init(CLCD_I2C_Name* LCD,
                    I2C_HandleTypeDef* hi2c_CLCD,
                    uint8_t Address,
                    uint8_t Colums,
                    uint8_t Rows)
{
  if ((LCD == 0) || (hi2c_CLCD == 0)) return;

  LCD->I2C = hi2c_CLCD;
  LCD->ADDRESS = Address;
  LCD->COLUMS = Colums;
  LCD->ROWS = Rows;

  LCD->FUNCTIONSET = (uint8_t)(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
  LCD->ENTRYMODE = (uint8_t)(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
  LCD->DISPLAYCTRL = (uint8_t)(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
  LCD->CURSORSHIFT = (uint8_t)(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT);
  LCD->BACKLIGHT = LCD_BACKLIGHT;

  CLCD_Delay(50U);
  CLCD_WriteI2C(LCD, 0x33U, CLCD_COMMAND);
  CLCD_WriteI2C(LCD, 0x33U, CLCD_COMMAND);
  CLCD_Delay(5U);
  CLCD_WriteI2C(LCD, 0x32U, CLCD_COMMAND);
  CLCD_Delay(5U);
  CLCD_WriteI2C(LCD, 0x20U, CLCD_COMMAND);
  CLCD_Delay(5U);

  CLCD_WriteI2C(LCD, LCD->ENTRYMODE, CLCD_COMMAND);
  CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);
  CLCD_WriteI2C(LCD, LCD->CURSORSHIFT, CLCD_COMMAND);
  CLCD_WriteI2C(LCD, LCD->FUNCTIONSET, CLCD_COMMAND);

  CLCD_WriteI2C(LCD, LCD_CLEARDISPLAY, CLCD_COMMAND);
  CLCD_WriteI2C(LCD, LCD_RETURNHOME, CLCD_COMMAND);
}

void CLCD_I2C_SetCursor(CLCD_I2C_Name* LCD, uint8_t Xpos, uint8_t Ypos)
{
  uint8_t DRAM_ADDRESS = 0x00U;

  if (LCD == 0) return;

  if (Xpos >= LCD->COLUMS) Xpos = (uint8_t)(LCD->COLUMS - 1U);
  if (Ypos >= LCD->ROWS)  Ypos = (uint8_t)(LCD->ROWS - 1U);

  if (Ypos == 0U)       DRAM_ADDRESS = (uint8_t)(0x00U + Xpos);
  else if (Ypos == 1U)  DRAM_ADDRESS = (uint8_t)(0x40U + Xpos);
  else if (Ypos == 2U)  DRAM_ADDRESS = (uint8_t)(0x14U + Xpos);
  else if (Ypos == 3U)  DRAM_ADDRESS = (uint8_t)(0x54U + Xpos);

  CLCD_WriteI2C(LCD, (uint8_t)(LCD_SETDDRAMADDR | DRAM_ADDRESS), CLCD_COMMAND);
}

void CLCD_I2C_WriteChar(CLCD_I2C_Name* LCD, char character)
{
  if (LCD == 0) return;
  CLCD_WriteI2C(LCD, (uint8_t)character, CLCD_DATA);
}

#endif /* CLCD_I2C_MINIMAL_IMPL */

/* Timeout for I2C device ready check */
#define LCD_I2C_PROBE_TRIALS          5U
#define LCD_I2C_PROBE_TIMEOUT_MS     20U

static uint8_t ucLcdIsReady;
static CLCD_I2C_Name lcd_device;

static void lcd_display_write_line(uint8_t row, const char *text)
{
  uint8_t column;
  char output_text[LCD_COLUMN_COUNT + 1U];

  memset(output_text, ' ', LCD_COLUMN_COUNT);
  output_text[LCD_COLUMN_COUNT] = '\0';

  if (text != 0)
  {
    for (column = 0U; (column < LCD_COLUMN_COUNT) && (text[column] != '\0'); ++column)
    {
      output_text[column] = text[column];
    }
  }

  CLCD_I2C_SetCursor(&lcd_device, 0, row);

  for (column = 0U; column < LCD_COLUMN_COUNT; ++column)
  {
    CLCD_I2C_WriteChar(&lcd_device, output_text[column]);
  }
}

bool lcd_display_init(void)
{
  if (APP_LCD_ENABLED == 0)
  {
    ucLcdIsReady = 0U;
    return false;
  }

  HAL_Delay(LCD_BOOT_DELAY_MS);

  if (HAL_I2C_IsDeviceReady(&hi2c1,
                            LCD_I2C_ADDRESS,
                            LCD_I2C_PROBE_TRIALS,
                            LCD_I2C_PROBE_TIMEOUT_MS) != HAL_OK)
  {
    ucLcdIsReady = 0U;
    return false;
  }

  CLCD_I2C_Init(&lcd_device, &hi2c1, LCD_I2C_ADDRESS, LCD_COLUMN_COUNT, LCD_ROW_COUNT);

  ucLcdIsReady = 1U;
  lcd_display_show_boot();
  return true;
}

void lcd_display_show_boot(void)
{
  if (ucLcdIsReady == 0U)
  {
    return;
  }

  lcd_display_write_line(0U, "S1:AAA S2:BBB");
  lcd_display_write_line(1U, "STATE S1 STATE S2");
}

/* line0 (8 chars): "Sx: AAA"
 *    col0='S', col1='1'/'2', col2=':', col3=' ', col4-6=AAA, col7=' '
 */
static void lcd_display_format_sensor_top(char out8[9],
                                          const char *label2,
                                          const system_state_t *sensor_state)
{
  unsigned long distance_cm_rounded;
  char value3[4];

  memset(out8, ' ', 8U);
  out8[8] = '\0';

  out8[0U] = label2[0U];
  out8[1U] = label2[1U];
  out8[2U] = ':';
  out8[3U] = ' ';

  value3[0] = value3[1] = value3[2] = ' ';
  value3[3] = '\0';

  if (sensor_state == 0)
  {
    (void)snprintf(value3, sizeof(value3), "---");
  }
  else if (sensor_state->system_status == SYSTEM_STATUS_OK)
  {
    distance_cm_rounded = (unsigned long)(sensor_state->distance_cm + 0.5f);
    /* lấy 3 chữ số cuối để hiển thị vừa 3 ký tự */
    distance_cm_rounded %= 1000UL;
    (void)snprintf(value3, sizeof(value3), "%03lu", distance_cm_rounded);
  }
  else if (sensor_state->system_status == SYSTEM_STATUS_SENSOR_BELOW_MIN)
  {
    /* hiển thị "<20" nhưng format 3 ký tự -> "20" hoặc "<2?"
     * ưu tiên hiển thị "<20" => chuyển sang "20 " */
    (void)snprintf(value3, sizeof(value3), "20 ");
    value3[0] = '<';
    value3[1] = '2';
    value3[2] = '0';
  }
  else if (sensor_state->system_status == SYSTEM_STATUS_SENSOR_TIMEOUT)
  {
    (void)snprintf(value3, sizeof(value3), "NO ");
  }
  else
  {
    /* fault/unknown */
    (void)snprintf(value3, sizeof(value3), "ERR");
  }

  out8[4U] = value3[0];
  out8[5U] = value3[1];
  out8[6U] = value3[2];
  /* out8[7] là ' ' */
}

/* line1 (8 chars): hiển thị status/warning level
 *    Format: "TIMEOUT " hoặc "SAFE    " hoặc "WARNING " hoặc "DANGER  "
 *    Chỉ hiển thị trạng thái, không có S1/S2
 */
static void lcd_display_format_sensor_bottom(char out8[9], const char *label2,
                                             const system_state_t *sensor_state)
{
  const char *status_text;
  uint8_t idx;
  
  memset(out8, ' ', 8U);
  out8[8] = '\0';

  if (sensor_state == 0)
  {
    /* Nếu không có state, hiển thị "---     " */
    out8[0U] = '-';
    out8[1U] = '-';
    out8[2U] = '-';
    return;
  }

  /* Ưu tiên hiển thị system_status nếu có lỗi, nếu không hiển thị warning_level */
  if (sensor_state->system_status != SYSTEM_STATUS_OK)
  {
    status_text = system_state_get_status_text(sensor_state->system_status);
  }
  else
  {
    status_text = system_state_get_warning_text(sensor_state->warning_level);
  }

  /* Hiển thị status text đầy đủ (8 ký tự), phần còn lại sẽ là spaces */
  for (idx = 0U; (idx < 8U) && (status_text[idx] != '\0'); ++idx)
  {
    out8[idx] = status_text[idx];
  }
}

void lcd_display_show_state(const system_state_t *sensor1_state, const system_state_t *sensor2_state)
{
  char line0[LCD_COLUMN_COUNT + 1U];
  char line1[LCD_COLUMN_COUNT + 1U];
  char left_top[9];
  char right_top[9];
  char left_bottom[9];
  char right_bottom[9];

  if (ucLcdIsReady == 0U)
  {
    return;
  }

  memset(line0, ' ', LCD_COLUMN_COUNT);
  memset(line1, ' ', LCD_COLUMN_COUNT);
  line0[LCD_COLUMN_COUNT] = '\0';
  line1[LCD_COLUMN_COUNT] = '\0';

  lcd_display_format_sensor_top(left_top, "S1", sensor1_state);
  lcd_display_format_sensor_top(right_top, "S2", sensor2_state);
  lcd_display_format_sensor_bottom(left_bottom, "S1", sensor1_state);
  lcd_display_format_sensor_bottom(right_bottom, "S2", sensor2_state);

  memcpy(&line0[0], left_top, 8U);
  memcpy(&line0[8], right_top, 8U);
  memcpy(&line1[0], left_bottom, 8U);
  memcpy(&line1[8], right_bottom, 8U);

  lcd_display_write_line(0U, line0);
  lcd_display_write_line(1U, line1);
}
