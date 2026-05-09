#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#define SENSOR_SAMPLE_PERIOD_MS          100U
/* Nghi giua 2 lan do (giam nhieu cheo giua 2 cam bien sieu am) */
#define SENSOR_INTER_SAMPLE_GAP_MS       50U
#define SENSOR_ECHO_TIMEOUT_MS            60U
#define SENSOR_TRIGGER_PULSE_US           10U
#define SENSOR_STARTUP_DELAY_MS          500U
/* JSN-SR04T: khoang do tin cay theo datasheet */
#define SENSOR_MIN_VALID_CM              20.0f
#define SENSOR_MAX_VALID_CM             600.0f
/* Median (so mau le) + EMA; bo mau nhay dot bien so voi bo loc truoc */
#define SENSOR_FILTER_MEDIAN_SIZE          5U
#define SENSOR_FILTER_EMA_ALPHA         0.35f
#define SENSOR_FILTER_MAX_STEP_CM       120.0f
#define SENSOR_NEAR_SPIKE_CONFIRM_COUNT    2U
#define SENSOR_TIMEOUT_HOLD_COUNT          4U
#define SENSOR_TIMEOUT_HOLD_MS           800U

#define ALERT_WARNING_BLINK_MS          500U
#define ALERT_WARNING_BEEP_WINDOW_MS    120U
#define ALERT_WARNING_BEEP_PERIOD_MS    700U
#define ALERT_ERROR_BLINK_MS            200U

/* 0: khong dung LCD (khong goi I2C). Dat 1 khi co module I2C LCD */
#define APP_LCD_ENABLED                 1

#define LCD_I2C_ADDRESS                (0x27U << 1) /* Dia chi I2C cua LCD, shift len 1 bit de phu hop HAL */
#define LCD_COLUMN_COUNT                16U
#define LCD_ROW_COUNT                    2U
#define LCD_BOOT_DELAY_MS               50U

#define UART_TX_TIMEOUT_MS              50U
#define UART_FRAME_MAX_LENGTH           220U

#define SENSOR_TASK_STACK_SIZE         256U
#define PROCESSING_TASK_STACK_SIZE     256U
#define ALERT_TASK_STACK_SIZE          192U
#define DISPLAY_TASK_STACK_SIZE        256U
#define UART_TASK_STACK_SIZE           256U

#define SENSOR_TASK_PRIORITY               4U
#define PROCESSING_TASK_PRIORITY           5U
#define ALERT_TASK_PRIORITY                3U
#define DISPLAY_TASK_PRIORITY              1U
#define UART_TASK_PRIORITY                 2U

#endif /* SYSTEM_CONFIG_H */
