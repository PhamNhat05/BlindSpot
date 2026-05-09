/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "lcd_display.h"
#include "sensor_filter.h"
#include "system_config.h"
#include "system_state.h"
#include "thresholds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
// So tick CPU ung voi 1 micro giay (dung cho delay us)
static uint32_t cpu_ticks_per_us;

typedef struct
{
  GPIO_TypeDef *trig_port;
  uint16_t trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t echo_pin;
} sensor_pin_map_t;

typedef struct
{
  volatile uint32_t echo_start_us;
  volatile uint32_t echo_pulse_us;
  volatile uint8_t waiting_fall_edge;
  volatile uint8_t echo_ready;
  volatile uint8_t measurement_active;
} sensor_capture_t;

typedef struct
{
  system_state_t sensor1_state;
  system_state_t sensor2_state;
  system_state_t combined_state;
} state_snapshot_t;

typedef struct
{
  uint32_t last_valid_tick_ms;
  uint8_t timeout_streak;
} sensor_runtime_state_t;

static const sensor_pin_map_t sensor_pin_map[SENSOR_FILTER_INSTANCE_COUNT] = {
  {TRig_GPIO_Port, TRig_Pin, Echo_GPIO_Port, Echo_Pin},
  {trig2_GPIO_Port, trig2_Pin, echo2_GPIO_Port, echo2_Pin}
};
static sensor_capture_t sensor_capture[SENSOR_FILTER_INSTANCE_COUNT];

typedef struct
{
  uint8_t sensor;
  distance_data_t data;
} data_t;

static QueueHandle_t dataQueue;
static QueueHandle_t alertQueue;
#if (APP_LCD_ENABLED != 0)
static QueueHandle_t displayQueue;
#endif
static SemaphoreHandle_t echoSem[SENSOR_FILTER_INSTANCE_COUNT];
#if (APP_LCD_ENABLED != 0)
static SemaphoreHandle_t lcdMutex;
#endif
static SemaphoreHandle_t trigMutex;

static SemaphoreHandle_t sampleSem;
static TimerHandle_t sampleTimer;
/* 0: dang do Echo cam bien 1 (PA2); 1: dang do echo2 — loai bo xung nham tu cam con lai */
// Du lieu / trang thai theo tung cam bien
static distance_data_t distance1;
static distance_data_t distance2;
static system_state_t state1;
static system_state_t state2;
static system_state_t state;
static sensor_runtime_state_t sensor_runtime[SENSOR_FILTER_INSTANCE_COUNT];

/* Heuristic reject echo giả:
 * Khi sensor bị rút vẫn tạo pulse giả, giá trị distance thường dao động kiểu "bó hẹp".
 * Ta theo dõi độ thay đổi so với lần trước; nếu OK liên tục nhưng chỉ đổi rất ít trong nhiều mẫu
 * -> coi là FAULT (mất kết nối / echo giả).
 */
static float s_last_ok_distance[SENSOR_FILTER_INSTANCE_COUNT];
static uint8_t s_has_last_ok[SENSOR_FILTER_INSTANCE_COUNT];
static uint8_t s_stable_ok_count[SENSOR_FILTER_INSTANCE_COUNT];

#define ECHO_FAKE_DELTA_CM_THRESHOLD   (10.0f)
#define ECHO_FAKE_STABLE_OK_MIN_COUNT  (4U)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void sensor_reset_capture(uint8_t sensor_index);
static void sensor_fill_timeout(distance_data_t *distance_data, uint32_t timestamp_ms);
static void sensor_start_measurement(uint8_t sensor_index, uint32_t now_ms);
static void app_init_runtime_objects(void);
static void app_init_default_state(void);
static void vSensorTask(void *argument);
static void vProcessingTask(void *argument);
static void vAlertTask(void *argument);
#if (APP_LCD_ENABLED != 0)
static void vDisplayTask(void *argument);
#endif
static void taskMakeState(const distance_data_t *distance_data, system_state_t *system_state);
static void sample_timer_cb(TimerHandle_t timer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Khoi tao cac module can dung truoc vong lap chinh */
  app_support_init();
  system_state_init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|led_Pin|buzz_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Echo_Pin echo2_Pin */
  GPIO_InitStruct.Pin = Echo_Pin|echo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 led_Pin buzz_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|led_Pin|buzz_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* CubeMX: Echo thuong la Input thuong — can EXTI (cang len/xuong) de do do rong xung.
   * Trig1/Trig2 da la Output trong MX_GPIO_Init phia tren (PA1, PA3).
   * Su dung PULLDOWN de tranh floating signal khi cam bien bi rut ra. */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = echo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(echo2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void app_delay_counter_init(void)
{
  // Lay tan so HCLK de quy doi so tick ve micro giay
  cpu_ticks_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t app_get_microseconds(void)
{
  if (cpu_ticks_per_us == 0U)
  {
    app_delay_counter_init();
  }

  return (DWT->CYCCNT / cpu_ticks_per_us);
}

void app_support_init(void)
{
  uint8_t sensor_index;

  for (sensor_index = 0U; sensor_index < SENSOR_FILTER_INSTANCE_COUNT; sensor_index++)
  {
    sensor_reset_capture(sensor_index);
  }

  sensor_filter_reset_all();
  app_delay_counter_init();
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(buzz_GPIO_Port, buzz_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TRig_GPIO_Port, TRig_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(trig2_GPIO_Port, trig2_Pin, GPIO_PIN_RESET);
  (void)lcd_display_init();
}

void app_delay_us(uint32_t microseconds)
{
  uint32_t start_tick;
  uint32_t wait_tick;

  if (cpu_ticks_per_us == 0U)
  {
    app_delay_counter_init();
  }

  start_tick = DWT->CYCCNT;
  wait_tick = microseconds * cpu_ticks_per_us;

  while ((DWT->CYCCNT - start_tick) < wait_tick)
  {
  }
}

static void sensor_reset_capture(uint8_t sensor_index)
{
  if (sensor_index >= SENSOR_FILTER_INSTANCE_COUNT)
  {
    return;
  }

  sensor_capture[sensor_index].echo_start_us = 0U;
  sensor_capture[sensor_index].echo_pulse_us = 0U;
  sensor_capture[sensor_index].waiting_fall_edge = 0U;
  sensor_capture[sensor_index].echo_ready = 0U;
  sensor_capture[sensor_index].measurement_active = 0U;
}

static void sensor_fill_timeout(distance_data_t *distance_data, uint32_t timestamp_ms)
{
  if (distance_data == 0)
  {
    return;
  }

  distance_data->timestamp_ms = timestamp_ms;
  distance_data->distance_cm = 0.0f;
  distance_data->is_object_detected = 0U;
  distance_data->system_status = SYSTEM_STATUS_SENSOR_TIMEOUT;
}

static void sensor_start_measurement(uint8_t sensor_index, uint32_t now_ms)
{
  (void)now_ms;

  if (sensor_index >= SENSOR_FILTER_INSTANCE_COUNT)
  {
    return;
  }

  if ((trigMutex != NULL) && (xSemaphoreTake(trigMutex, pdMS_TO_TICKS(2U)) != pdPASS))
  {
    return;
  }

  sensor_capture[sensor_index].echo_pulse_us = 0U;
  sensor_capture[sensor_index].waiting_fall_edge = 0U;
  sensor_capture[sensor_index].echo_ready = 0U;
  sensor_capture[sensor_index].measurement_active = 1U;

  HAL_GPIO_WritePin(TRig_GPIO_Port, TRig_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(trig2_GPIO_Port, trig2_Pin, GPIO_PIN_RESET);
  app_delay_us(2U);
  HAL_GPIO_WritePin(sensor_pin_map[sensor_index].trig_port,
                    sensor_pin_map[sensor_index].trig_pin,
                    GPIO_PIN_SET);
  app_delay_us(SENSOR_TRIGGER_PULSE_US);
  HAL_GPIO_WritePin(sensor_pin_map[sensor_index].trig_port,
                    sensor_pin_map[sensor_index].trig_pin,
                    GPIO_PIN_RESET);

  if (trigMutex != NULL)
  {
    xSemaphoreGive(trigMutex);
  }
}

bool taskReadDistanceSensor(uint8_t sensor_index, distance_data_t *distance_data)
{
  BaseType_t wait_result;
  sensor_capture_t *capture;

  if ((distance_data == 0) || (sensor_index >= SENSOR_FILTER_INSTANCE_COUNT))
  {
    return false;
  }

  capture = &sensor_capture[sensor_index];
  sensor_fill_timeout(distance_data, HAL_GetTick());

  if (echoSem[sensor_index] == NULL)
  {
    return false;
  }

  while (xSemaphoreTake(echoSem[sensor_index], 0U) == pdPASS)
  {
  }

  sensor_start_measurement(sensor_index, HAL_GetTick());

  if (capture->measurement_active == 0U)
  {
    distance_data->system_status = SYSTEM_STATUS_SENSOR_FAULT;
    distance_data->timestamp_ms = HAL_GetTick();
    return false;
  }

  wait_result =
      xSemaphoreTake(echoSem[sensor_index], pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS));

  if (wait_result != pdPASS)
  {
    capture->measurement_active = 0U;
    capture->waiting_fall_edge = 0U;
    capture->echo_ready = 0U;

    /* Đảm bảo status không bị giữ giá trị cũ khi timeout */
    distance_data->system_status = SYSTEM_STATUS_SENSOR_TIMEOUT;
    distance_data->is_object_detected = 0U;
    distance_data->distance_cm = 0.0f;
    distance_data->timestamp_ms = HAL_GetTick();
    return false;
  }

  distance_data->timestamp_ms = HAL_GetTick();

  if ((capture->echo_ready == 0U) || (capture->echo_pulse_us == 0U))
  {
    distance_data->distance_cm = 0.0f;
    distance_data->is_object_detected = 0U;
    distance_data->system_status = SYSTEM_STATUS_SENSOR_FAULT;
    capture->waiting_fall_edge = 0U;
    capture->echo_ready = 0U;
    capture->measurement_active = 0U;
    return false;
  }

  distance_data->distance_cm = ((float)capture->echo_pulse_us / 58.0f);
  distance_data->is_object_detected = (uint8_t)(distance_data->distance_cm > 0.0f);

  /* Lọc nhiễu/echo giả:
   * Nếu khoảng cách suy ra quá nhỏ (gần như 0) hoặc vượt quá giới hạn cảm nhận => coi là fault.
   * Mục tiêu: khi rút sensor2 mà chân echo bị nhiễu/float, LCD không còn hiển thị SAFE/WARN/DANG cho S2.
   */
  if ((distance_data->distance_cm < (SENSOR_MIN_VALID_CM * 0.5f)) ||
      (distance_data->distance_cm > SENSOR_MAX_VALID_CM))
  {
    distance_data->distance_cm = 0.0f;
    distance_data->is_object_detected = 0U;
    distance_data->system_status = SYSTEM_STATUS_SENSOR_FAULT;
  }
  else
  {
    distance_data->system_status = SYSTEM_STATUS_OK;
  }

  capture->waiting_fall_edge = 0U;
  capture->echo_ready = 0U;
  capture->measurement_active = 0U;
  return true;
}

warning_level_t taskPickWarningLevel(float distance_cm, system_status_t system_status)
{
  if (system_status == SYSTEM_STATUS_SENSOR_BELOW_MIN)
  {
    return WARNING_LEVEL_DANGER;
  }

  /* Cac loi khac: khong tin vao khoang cach, canh bao qua taskUpdateAlert */
  if (system_status != SYSTEM_STATUS_OK)
  {
    return WARNING_LEVEL_SAFE;
  }

  if (distance_cm < WARNING_DISTANCE_THRESHOLD_CM)
  {
    return WARNING_LEVEL_DANGER;
  }

  if (distance_cm <= SAFE_DISTANCE_THRESHOLD_CM)
  {
    return WARNING_LEVEL_WARNING;
  }

  return WARNING_LEVEL_SAFE;
}

static uint8_t compute_led_for_sensor(const system_state_t *st, uint32_t tick_ms)
{
  if (st == 0)
  {
    return 0U;
  }

  if (st->system_status == SYSTEM_STATUS_SENSOR_BELOW_MIN)
  {
    return 1U;
  }

  if (st->system_status != SYSTEM_STATUS_OK)
  {
    return (uint8_t)(((tick_ms / ALERT_ERROR_BLINK_MS) % 2U) != 0U);
  }

  if (st->warning_level == WARNING_LEVEL_DANGER)
  {
    return 1U;
  }

  if (st->warning_level == WARNING_LEVEL_WARNING)
  {
    return (uint8_t)(((tick_ms / ALERT_WARNING_BLINK_MS) % 2U) != 0U);
  }

  return 0U;
}

static void compute_buzzer_from_combined(const system_state_t *combined,
                                         uint32_t tick_ms,
                                         uint8_t *out_buzzer_on)
{
  uint32_t pattern_tick_ms;

  if ((combined == 0) || (out_buzzer_on == 0))
  {
    return;
  }

  if (combined->system_status == SYSTEM_STATUS_SENSOR_BELOW_MIN)
  {
    *out_buzzer_on = 1U;
    return;
  }

  if (combined->system_status != SYSTEM_STATUS_OK)
  {
    *out_buzzer_on = (uint8_t)(((tick_ms / ALERT_ERROR_BLINK_MS) % 2U) != 0U);
    return;
  }

  if (combined->warning_level == WARNING_LEVEL_DANGER)
  {
    *out_buzzer_on = 1U;
    return;
  }

  if (combined->warning_level == WARNING_LEVEL_WARNING)
  {
    pattern_tick_ms = tick_ms % ALERT_WARNING_BEEP_PERIOD_MS;
    *out_buzzer_on = (uint8_t)(pattern_tick_ms < ALERT_WARNING_BEEP_WINDOW_MS);
    return;
  }

  *out_buzzer_on = 0U;
}

void taskUpdateAlert(const system_state_t *combined_state,
                     const system_state_t *sensor1_state,
                     const system_state_t *sensor2_state)
{
  uint32_t tick_ms = HAL_GetTick();
  uint8_t buzz_on = 0U;

  if ((combined_state == 0) || (sensor1_state == 0) || (sensor2_state == 0))
  {
    return;
  }

  compute_buzzer_from_combined(combined_state, tick_ms, &buzz_on);

  HAL_GPIO_WritePin(led_GPIO_Port,
                    led_Pin,
                    compute_led_for_sensor(sensor1_state, tick_ms) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(led2_GPIO_Port,
                    led2_Pin,
                    compute_led_for_sensor(sensor2_state, tick_ms) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(buzz_GPIO_Port, buzz_Pin, buzz_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void taskUpdateDisplay(const system_state_t *sensor1_state,
                       const system_state_t *sensor2_state)
{
#if (APP_LCD_ENABLED != 0)
  if ((sensor1_state == 0) || (sensor2_state == 0))
  {
    return;
  }

  if (lcdMutex != NULL)
  {
    if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(10U)) != pdPASS)
    {
      return;
    }
  }

  /* Disconnect robust theo thời gian:
   * Nếu state của sensor không được cập nhật trong quá lâu -> ép về TIMEOUT
   * để LCD không còn hiển thị "OK/Safe" hoặc nhảy số khi sensor bị rút.
   */
  system_state_t s1 = *sensor1_state;
  system_state_t s2 = *sensor2_state;
  const uint32_t now_ms = HAL_GetTick();

  if ((now_ms - s1.timestamp_ms) > SENSOR_TIMEOUT_HOLD_MS)
  {
    s1.timestamp_ms = now_ms;
    s1.system_status = SYSTEM_STATUS_SENSOR_TIMEOUT;
    s1.distance_cm = 0.0f;
    s1.is_object_detected = 0U;
    /* warning_level sẽ được lcd_display_format_sensor_bottom xử lý từ system_status */
  }

  if ((now_ms - s2.timestamp_ms) > SENSOR_TIMEOUT_HOLD_MS)
  {
    s2.timestamp_ms = now_ms;
    s2.system_status = SYSTEM_STATUS_SENSOR_TIMEOUT;
    s2.distance_cm = 0.0f;
    s2.is_object_detected = 0U;
  }

  // Hien thi tach rieng S1/S2 de de debug cam bien
  lcd_display_show_state(&s1, &s2);

  if (lcdMutex != NULL)
  {
    xSemaphoreGive(lcdMutex);
  }
#else
  (void)sensor1_state;
  (void)sensor2_state;
#endif
}

void taskHandleEchoEdgeIsr(uint16_t gpio_pin)
{
  uint32_t now_us;
  uint8_t sensor_index;
  sensor_capture_t *capture;
  BaseType_t higher_priority_woken;

  if (gpio_pin == Echo_Pin)
  {
    sensor_index = 0U;
  }
  else if (gpio_pin == echo2_Pin)
  {
    sensor_index = 1U;
  }
  else
  {
    return;
  }

  if (sensor_index >= SENSOR_FILTER_INSTANCE_COUNT)
  {
    return;
  }

  capture = &sensor_capture[sensor_index];

  if (capture->measurement_active == 0U)
  {
    return;
  }

  now_us = app_get_microseconds();

  if (HAL_GPIO_ReadPin(sensor_pin_map[sensor_index].echo_port,
                       sensor_pin_map[sensor_index].echo_pin) == GPIO_PIN_SET)
  {
    capture->echo_start_us = now_us;
    capture->waiting_fall_edge = 1U;
  }
  else if (capture->waiting_fall_edge != 0U)
  {
    higher_priority_woken = pdFALSE;

    capture->echo_pulse_us = now_us - capture->echo_start_us;
    capture->waiting_fall_edge = 0U;
    capture->echo_ready = 1U;

    if (echoSem[sensor_index] != NULL)
    {
      xSemaphoreGiveFromISR(echoSem[sensor_index], &higher_priority_woken);
      portYIELD_FROM_ISR(higher_priority_woken);
    }
  }
}

static void sample_timer_cb(TimerHandle_t timer)
{
  (void)timer;

  if (sampleSem != NULL)
  {
    (void)xSemaphoreGive(sampleSem);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  taskHandleEchoEdgeIsr(gpio_pin);
}

static warning_level_t worst_warning_level(warning_level_t a, warning_level_t b)
{
  if ((a == WARNING_LEVEL_DANGER) || (b == WARNING_LEVEL_DANGER))
  {
    return WARNING_LEVEL_DANGER;
  }

  if ((a == WARNING_LEVEL_WARNING) || (b == WARNING_LEVEL_WARNING))
  {
    return WARNING_LEVEL_WARNING;
  }

  return WARNING_LEVEL_SAFE;
}

static uint8_t system_status_priority(system_status_t s)
{
  switch (s)
  {
    case SYSTEM_STATUS_SENSOR_BELOW_MIN:
      return 5U;

    case SYSTEM_STATUS_SENSOR_TIMEOUT:
      return 4U;

    case SYSTEM_STATUS_SENSOR_FAULT:
    case SYSTEM_STATUS_UART_ERROR:
      return 3U;

    default:
      return 0U;
  }
}

static system_status_t worst_system_status(system_status_t a, system_status_t b)
{
  if (system_status_priority(a) >= system_status_priority(b))
  {
    return a;
  }

  return b;
}

static void dual_state_combine(const system_state_t *s1,
                               const system_state_t *s2,
                               system_state_t *out)
{
  out->timestamp_ms = s1->timestamp_ms;

  if (s2->timestamp_ms > out->timestamp_ms)
  {
    out->timestamp_ms = s2->timestamp_ms;
  }

  out->system_status = worst_system_status(s1->system_status, s2->system_status);
  out->warning_level = worst_warning_level(s1->warning_level, s2->warning_level);

  if ((s1->system_status == SYSTEM_STATUS_OK) && (s2->system_status == SYSTEM_STATUS_OK))
  {
    out->distance_cm = (s1->distance_cm < s2->distance_cm) ? s1->distance_cm : s2->distance_cm;
  }
  else if (s1->system_status == SYSTEM_STATUS_OK)
  {
    out->distance_cm = s1->distance_cm;
  }
  else if (s2->system_status == SYSTEM_STATUS_OK)
  {
    out->distance_cm = s2->distance_cm;
  }
  else
  {
    out->distance_cm = 0.0f;
  }

  out->is_object_detected =
      (uint8_t)((s1->is_object_detected != 0U) || (s2->is_object_detected != 0U));
}

static void taskMakeState(const distance_data_t *distance_data,
                          system_state_t *system_state)
{
  if ((distance_data == 0) || (system_state == 0))
  {
    return;
  }

  // Copy du lieu goc tu task doc cam bien
  system_state->timestamp_ms = distance_data->timestamp_ms;
  system_state->distance_cm = distance_data->distance_cm;
  system_state->system_status = distance_data->system_status;
  system_state->is_object_detected = distance_data->is_object_detected;

  if ((system_state->system_status == SYSTEM_STATUS_OK) &&
      (system_state->distance_cm < SENSOR_MIN_VALID_CM))
  {
    system_state->system_status = SYSTEM_STATUS_SENSOR_BELOW_MIN;
    system_state->distance_cm = SENSOR_MIN_VALID_CM;
    system_state->is_object_detected = 1U;
  }

  if (system_state->system_status == SYSTEM_STATUS_SENSOR_BELOW_MIN)
  {
    system_state->warning_level = taskPickWarningLevel(system_state->distance_cm,
                                                       system_state->system_status);
    return;
  }

  if (system_state->system_status != SYSTEM_STATUS_OK)
  {
    system_state->distance_cm = 0.0f;
    system_state->is_object_detected = 0U;
    system_state->warning_level = taskPickWarningLevel(system_state->distance_cm,
                                                       system_state->system_status);
    return;
  }

  /* Chuan hoa lai du lieu de tranh gia tri loi */
  if (system_state->distance_cm > SENSOR_MAX_VALID_CM)
  {
    system_state->system_status = SYSTEM_STATUS_SENSOR_FAULT;
    system_state->distance_cm = 0.0f;
    system_state->is_object_detected = 0U;
  }

  system_state->warning_level = taskPickWarningLevel(system_state->distance_cm,
                                                     system_state->system_status);
}

static void app_init_runtime_objects(void)
{
  uint8_t sensor_index;

  if (dataQueue == NULL)
  {
    dataQueue = xQueueCreate(12U, sizeof(data_t));
  }

  if (alertQueue == NULL)
  {
    alertQueue = xQueueCreate(1U, sizeof(state_snapshot_t));
  }

#if (APP_LCD_ENABLED != 0)
  if (displayQueue == NULL)
  {
    displayQueue = xQueueCreate(1U, sizeof(state_snapshot_t));
  }
#endif

#if (APP_LCD_ENABLED != 0)
  if (lcdMutex == NULL)
  {
    lcdMutex = xSemaphoreCreateMutex();
  }
#endif

  if (trigMutex == NULL)
  {
    trigMutex = xSemaphoreCreateMutex();
  }

  for (sensor_index = 0U; sensor_index < SENSOR_FILTER_INSTANCE_COUNT; sensor_index++)
  {
    if (echoSem[sensor_index] == NULL)
    {
      echoSem[sensor_index] = xSemaphoreCreateBinary();
    }
  }

  if (sampleSem == NULL)
  {
    sampleSem = xSemaphoreCreateBinary();
  }

  if (sampleTimer == NULL)
  {
    sampleTimer = xTimerCreate("sample_timer",
                               pdMS_TO_TICKS(SENSOR_SAMPLE_PERIOD_MS),
                               pdTRUE,
                               NULL,
                               sample_timer_cb);
    if (sampleTimer != NULL)
    {
      (void)xTimerStart(sampleTimer, 0U);
    }
  }
}

static void app_init_default_state(void)
{
  uint8_t sensor_index;

  distance1.timestamp_ms = HAL_GetTick();
  distance1.distance_cm = 0.0f;
  distance1.is_object_detected = 0U;
  distance1.system_status = SYSTEM_STATUS_SENSOR_TIMEOUT;
  distance2 = distance1;

  taskMakeState(&distance1, &state1);
  taskMakeState(&distance2, &state2);
  dual_state_combine(&state1, &state2, &state);
  system_state_update(&state);

  for (sensor_index = 0U; sensor_index < SENSOR_FILTER_INSTANCE_COUNT; sensor_index++)
  {
    sensor_runtime[sensor_index].last_valid_tick_ms = 0U;
    sensor_runtime[sensor_index].timeout_streak = 0U;
  }
}

static void vSensorTask(void *argument)
{
  data_t data;
  TickType_t gap_ticks = pdMS_TO_TICKS(SENSOR_INTER_SAMPLE_GAP_MS);

  (void)argument;

  if ((dataQueue == NULL) || (sampleSem == NULL))
  {
    vTaskDelete(NULL);
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(SENSOR_STARTUP_DELAY_MS));

  for (;;)
  {
    (void)xSemaphoreTake(sampleSem, portMAX_DELAY);

    // Do cam bien 1 -> nghi -> do cam bien 2 (tranh nhiễu chéo)
    data.sensor = 0U;
    (void)taskReadDistanceSensor(0U, &data.data);
    (void)xQueueSendToBack(dataQueue, &data, pdMS_TO_TICKS(5U));

    vTaskDelay(gap_ticks);

    data.sensor = 1U;
    (void)taskReadDistanceSensor(1U, &data.data);
    (void)xQueueSendToBack(dataQueue, &data, pdMS_TO_TICKS(5U));
  }
}

static void vProcessingTask(void *argument)
{
  data_t data;
  state_snapshot_t snapshot;
  distance_data_t normalized_data;
  sensor_runtime_state_t *runtime;
  distance_data_t *target_distance;
  system_state_t *target_state;

  (void)argument;

  if ((dataQueue == NULL) || (alertQueue == NULL))
  {
    vTaskDelete(NULL);
    return;
  }

  app_init_default_state();

  for (;;)
  {
    if (xQueueReceive(dataQueue, &data, portMAX_DELAY) != pdPASS)
    {
      continue;
    }

    if (data.sensor >= SENSOR_FILTER_INSTANCE_COUNT)
    {
      continue;
    }

    normalized_data = data.data;
    runtime = &sensor_runtime[data.sensor];

    if (data.sensor == 0U)
    {
      target_distance = &distance1;
      target_state = &state1;
    }
    else
    {
      target_distance = &distance2;
      target_state = &state2;
    }

    if (normalized_data.system_status == SYSTEM_STATUS_OK)
    {
      sensor_filter_commit(data.sensor, &normalized_data);

      if ((normalized_data.system_status == SYSTEM_STATUS_OK) ||
          (normalized_data.system_status == SYSTEM_STATUS_SENSOR_BELOW_MIN))
      {
        runtime->last_valid_tick_ms = normalized_data.timestamp_ms;
        runtime->timeout_streak = 0U;
      }
    }
    else if (normalized_data.system_status == SYSTEM_STATUS_SENSOR_TIMEOUT)
    {
      runtime->timeout_streak++;
      normalized_data.distance_cm = 0.0f;
      normalized_data.is_object_detected = 0U;
      normalized_data.system_status = SYSTEM_STATUS_SENSOR_TIMEOUT;
    }
    else
    {
      runtime->timeout_streak = 0U;
    }

    *target_distance = normalized_data;
    taskMakeState(target_distance, target_state);

    dual_state_combine(&state1, &state2, &state);
    system_state_update(&state);

    snapshot.sensor1_state = state1;
    snapshot.sensor2_state = state2;
    snapshot.combined_state = state;

    (void)xQueueOverwrite(alertQueue, &snapshot);
#if (APP_LCD_ENABLED != 0)
    if (displayQueue != NULL)
    {
      (void)xQueueOverwrite(displayQueue, &snapshot);
    }
#endif
  }
}

static void vAlertTask(void *argument)
{
  state_snapshot_t snapshot;

  (void)argument;

  snapshot.sensor1_state = state1;
  snapshot.sensor2_state = state2;
  snapshot.combined_state = state;

  for (;;)
  {
    if (alertQueue != NULL)
    {
      (void)xQueueReceive(alertQueue, &snapshot, pdMS_TO_TICKS(5U));
    }

    taskUpdateAlert(&snapshot.combined_state, &snapshot.sensor1_state, &snapshot.sensor2_state);
    vTaskDelay(pdMS_TO_TICKS(5U));
  }
}

#if (APP_LCD_ENABLED != 0)
static void vDisplayTask(void *argument)
{
  state_snapshot_t snapshot;

  (void)argument;

  if (displayQueue == NULL)
  {
    vTaskDelete(NULL);
    return;
  }

  snapshot.sensor1_state = state1;
  snapshot.sensor2_state = state2;
  snapshot.combined_state = state;

  for (;;)
  {
    taskUpdateDisplay(&state1, &state2);
    vTaskDelay(pdMS_TO_TICKS(300U));
  }
}
#endif

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  BaseType_t create_result;

  /* USER CODE BEGIN 5 */
  (void)argument;
  app_init_runtime_objects();
  app_init_default_state();

  if ((dataQueue == NULL) || (alertQueue == NULL) ||
      (echoSem[0U] == NULL) || (echoSem[1U] == NULL) ||
      (trigMutex == NULL) || (sampleSem == NULL) || (sampleTimer == NULL))
  {
    Error_Handler();
  }

  create_result = xTaskCreate(vSensorTask,
															"sensor_task",
                              SENSOR_TASK_STACK_SIZE,
                              NULL,
                              SENSOR_TASK_PRIORITY,
                              NULL);
  if (create_result != pdPASS)
  {
    Error_Handler();
  }

  create_result = xTaskCreate(vProcessingTask,
                              "processing_task",
                              PROCESSING_TASK_STACK_SIZE,
                              NULL,
                              PROCESSING_TASK_PRIORITY,
                              NULL);
  if (create_result != pdPASS)
  {
    Error_Handler();
  }

  create_result = xTaskCreate(vAlertTask,
                              "alert_task",
                              ALERT_TASK_STACK_SIZE,
                              NULL,
                              ALERT_TASK_PRIORITY,
                              NULL);
  if (create_result != pdPASS)
  {
    Error_Handler();
  }

#if (APP_LCD_ENABLED != 0)
  create_result = xTaskCreate(vDisplayTask,
                              "display_task",
                              DISPLAY_TASK_STACK_SIZE,
                              NULL,
                              DISPLAY_TASK_PRIORITY,
                              NULL);
  if (create_result != pdPASS)
  {
    Error_Handler();
  }
#endif

  vTaskDelete(NULL);
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
