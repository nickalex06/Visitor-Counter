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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
#include "vl53l0x_api.h"
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DISTANCE_NO_TARGET 65535

#define WALL_DISTANCE_MM 1200
#define SENSOR_TOLERANCE_MM 30

#define BLOCK_DELTA_MM 200
#define CLEAR_DELTA_MM 80

#define BLOCK_THRESHOLD_MM (WALL_DISTANCE_MM - BLOCK_DELTA_MM)
#define CLEAR_THRESHOLD_MM (WALL_DISTANCE_MM - CLEAR_DELTA_MM)

#define SAMPLE_DELAY_MS 300
#define LOG_EVERY_MS 1000
#define PASS_COOLDOWN_MS 1000

typedef enum {
    STATE_EMPTY = 0,
    STATE_BLOCKED = 1
} PassageState;

static VL53L0X_Dev_t vl53_dev;
static VL53L0X_DEV vl53 = &vl53_dev;

static void led_on(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

static void led_off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

static int key_is_pressed(void)
{
    return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;
}

static void wait_for_key_press(void)
{
    led_off();

    while (!key_is_pressed()) {
        HAL_Delay(20);
    }

    HAL_Delay(50);

    while (key_is_pressed()) {
        led_on();
        HAL_Delay(20);
    }

    HAL_Delay(50);

    led_off();
}
static int vl53_init_sensor(void)
{
    VL53L0X_Error status;
    uint32_t ref_spad_count;
    uint8_t is_aperture_spads;
    uint8_t vhv_settings;
    uint8_t phase_cal;

    vl53->I2cDevAddr = 0x29 << 1;
    vl53->comms_type = 1;
    vl53->comms_speed_khz = 100;

    status = VL53L0X_DataInit(vl53);

    if (status != VL53L0X_ERROR_NONE) {
        return 1;
    }

    status = VL53L0X_StaticInit(vl53);

    if (status != VL53L0X_ERROR_NONE) {
        return 2;
    }

    status = VL53L0X_PerformRefSpadManagement(vl53, &ref_spad_count, &is_aperture_spads);

    if (status != VL53L0X_ERROR_NONE) {
        return 3;
    }

    status = VL53L0X_PerformRefCalibration(vl53, &vhv_settings, &phase_cal);

    if (status != VL53L0X_ERROR_NONE) {
        return 4;
    }

    status = VL53L0X_SetDeviceMode(vl53, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    if (status != VL53L0X_ERROR_NONE) {
        return 5;
    }

    return 0;
}

static int is_person_detected(uint16_t distance)
{
    if (distance == DISTANCE_NO_TARGET) {
        return 0;
    }

    if (distance < BLOCK_THRESHOLD_MM) {
        return 1;
    }

    return 0;
}

static int is_area_clear(uint16_t distance)
{
    if (distance == DISTANCE_NO_TARGET) {
        return 1;
    }

    if (distance > CLEAR_THRESHOLD_MM) {
        return 1;
    }

    return 0;
}

static int vl53_read_distance(uint16_t *distance_mm)
{
    VL53L0X_Error status;
    VL53L0X_RangingMeasurementData_t measurement;

    status = VL53L0X_PerformSingleRangingMeasurement(vl53, &measurement);

    if (status != VL53L0X_ERROR_NONE) {
        return 1;
    }

    if (measurement.RangeStatus != 0) {
        *distance_mm = DISTANCE_NO_TARGET;
        return 0;
    }

    *distance_mm = measurement.RangeMilliMeter;

    return 0;
}

static int append_line_to_file(const char *filename, const char *line)
{
    FIL file;
    FRESULT res;
    UINT written;

    res = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);

    if (res != FR_OK) {
        return 1;
    }

    res = f_lseek(&file, f_size(&file));

    if (res != FR_OK) {
        f_close(&file);
        return 2;
    }

    res = f_write(&file, line, strlen(line), &written);

    if (res != FR_OK || written != strlen(line)) {
        f_close(&file);
        return 3;
    }

    res = f_close(&file);

    if (res != FR_OK) {
        return 4;
    }

    return 0;
}

static int write_count_file(uint32_t count)
{
    FIL file;
    FRESULT res;
    UINT written;
    char line[64];

    snprintf(line, sizeof(line), "count=%lu\r\n", (unsigned long)count);

    res = f_open(&file, "COUNT.TXT", FA_CREATE_ALWAYS | FA_WRITE);

    if (res != FR_OK) {
        return 1;
    }

    res = f_write(&file, line, strlen(line), &written);

    if (res != FR_OK || written != strlen(line)) {
        f_close(&file);
        return 2;
    }

    res = f_sync(&file);

    if (res != FR_OK) {
        f_close(&file);
        return 3;
    }

    res = f_close(&file);

    if (res != FR_OK) {
        return 4;
    }

    return 0;
}

static int write_startup_files(void)
{
    int error;

    error = append_line_to_file("LOG.TXT", "\r\n--- START ---\r\n");

    if (error != 0) {
        return 1;
    }

    error = append_line_to_file("EVENTS.TXT", "\r\n--- START ---\r\n");

    if (error != 0) {
        return 2;
    }

    error = write_count_file(0);

    if (error != 0) {
        return 3;
    }

    return 0;
}

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
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  FATFS fs;
  FRESULT mount_res;

  uint32_t count = 0;
  uint32_t time_ms = 0;
  uint32_t last_log_ms = 0;
  uint32_t last_pass_ms = 0;

  uint16_t distance = 0;
  int sensor_error = 0;
  int system_error = 0;

  PassageState state = STATE_EMPTY;

  char line[128];

  HAL_Delay(500);

  led_off();

  wait_for_key_press();

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(200);

  mount_res = f_mount(&fs, "", 1);

  if (mount_res != FR_OK) {
      system_error = 1;
  }

  if (system_error == 0) {
      system_error = vl53_init_sensor();

      if (system_error != 0) {
          system_error = 2;
      }
  }

  if (system_error == 0) {
      count = 0;
      state = STATE_EMPTY;
      last_pass_ms = 0;
      last_log_ms = HAL_GetTick();

      system_error = write_startup_files();

      if (system_error != 0) {
          system_error = 3;
      }
  }

  if (system_error == 0) {
      system_error = write_count_file(count);

      if (system_error != 0) {
          system_error = 4;
      }
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if (system_error != 0) {
	      led_off();
	      HAL_Delay(1000);

	      for (int i = 0; i < system_error; i++) {
	          led_on();
	          HAL_Delay(300);
	          led_off();
	          HAL_Delay(300);
	      }

	      HAL_Delay(2000);
	      continue;
	  }

      time_ms = HAL_GetTick();
      sensor_error = vl53_read_distance(&distance);

      if (sensor_error == 0) {
          if (state == STATE_EMPTY && is_person_detected(distance)) {
              state = STATE_BLOCKED;
              led_on();

              snprintf(line, sizeof(line),
                       "time_ms=%lu, event=BLOCKED, distance=%u, count=%lu\r\n",
                       (unsigned long)time_ms,
                       distance,
                       (unsigned long)count);

              append_line_to_file("EVENTS.TXT", line);
              append_line_to_file("LOG.TXT", line);
          }

          if (state == STATE_BLOCKED && is_area_clear(distance)) {
              if ((time_ms - last_pass_ms) >= PASS_COOLDOWN_MS) {
                  state = STATE_EMPTY;
                  led_off();

                  count++;
                  last_pass_ms = time_ms;

                  if (distance == DISTANCE_NO_TARGET) {
                      snprintf(line, sizeof(line),
                               "time_ms=%lu, event=PASS, distance=NO_TARGET, count=%lu\r\n",
                               (unsigned long)time_ms,
                               (unsigned long)count);
                  } else {
                      snprintf(line, sizeof(line),
                               "time_ms=%lu, event=PASS, distance=%u, count=%lu\r\n",
                               (unsigned long)time_ms,
                               distance,
                               (unsigned long)count);
                  }

                  append_line_to_file("EVENTS.TXT", line);
                  append_line_to_file("LOG.TXT", line);
                  write_count_file(count);
              }
          }

          if ((time_ms - last_log_ms) >= LOG_EVERY_MS) {
              last_log_ms = time_ms;

              if (distance == DISTANCE_NO_TARGET) {
                  snprintf(line, sizeof(line),
                           "time_ms=%lu, distance=NO_TARGET, count=%lu, state=%s\r\n",
                           (unsigned long)time_ms,
                           (unsigned long)count,
                           state == STATE_EMPTY ? "EMPTY" : "BLOCKED");
              } else {
                  snprintf(line, sizeof(line),
                           "time_ms=%lu, distance=%u, count=%lu, state=%s\r\n",
                           (unsigned long)time_ms,
                           distance,
                           (unsigned long)count,
                           state == STATE_EMPTY ? "EMPTY" : "BLOCKED");
              }

              append_line_to_file("LOG.TXT", line);
          }
      } else {
          if ((time_ms - last_log_ms) >= LOG_EVERY_MS) {
              last_log_ms = time_ms;

              snprintf(line, sizeof(line),
                       "time_ms=%lu, sensor_error=%d, count=%lu, state=%s\r\n",
                       (unsigned long)time_ms,
                       sensor_error,
                       (unsigned long)count,
                       state == STATE_EMPTY ? "EMPTY" : "BLOCKED");

              append_line_to_file("LOG.TXT", line);
          }
      }

      HAL_Delay(SAMPLE_DELAY_MS);

	}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
