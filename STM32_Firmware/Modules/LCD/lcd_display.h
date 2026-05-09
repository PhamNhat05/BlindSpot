#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

/* lcd_display.c phụ thuộc vào CLCD I2C (type/prototypes).
 * Để project không cần include CLCD_I2C.h từ thư mục khác, khai báo tối thiểu ngay tại đây.
 */
#include "stm32f1xx_hal.h"
#include "system_state.h"

typedef struct
{
  I2C_HandleTypeDef* I2C;
  uint8_t ADDRESS;
  uint8_t COLUMS;
  uint8_t ROWS;
  uint8_t FUNCTIONSET;
  uint8_t ENTRYMODE;
  uint8_t DISPLAYCTRL;
  uint8_t CURSORSHIFT;
  uint8_t BACKLIGHT;
} CLCD_I2C_Name;

void CLCD_I2C_Init(CLCD_I2C_Name* LCD,
                    I2C_HandleTypeDef* hi2c_CLCD,
                    uint8_t Address,
                    uint8_t Colums,
                    uint8_t Rows);

void CLCD_I2C_SetCursor(CLCD_I2C_Name* LCD, uint8_t Xpos, uint8_t Ypos);
void CLCD_I2C_WriteChar(CLCD_I2C_Name* LCD, char character);

bool lcd_display_init(void);
void lcd_display_show_boot(void);
void lcd_display_show_state(const system_state_t *sensor1_state,
                            const system_state_t *sensor2_state);

#ifdef __cplusplus
extern "C" {
#endif

/* (không cần gì thêm; đảm bảo linker C khi build C++) */

#ifdef __cplusplus
}
#endif

#endif /* LCD_DISPLAY_H */
