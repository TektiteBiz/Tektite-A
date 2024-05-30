/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "bmi08x.h"
#include "bmi088.h"
#include "bmi088_stm32.h"
#include "MS5607SPI.h"
#include "spif.h"
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
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// BMI088
struct bmi08x_dev dev = {
        .accel_id = ACCEL_CS_Pin,
        .gyro_id = GYRO_CS_Pin,
        .intf = BMI08X_SPI_INTF,
        .read = &stm32_spi_read,//user_spi_read
        .write = &stm32_spi_write,//user_spi_write
        .delay_ms = &HAL_Delay,//user_delay_milli_sec
		//.accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ,
		//.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL,
		//.accel_cfg.range = BMI088_ACCEL_RANGE_24G
};

struct bmi08x_int_cfg int_config;
struct bmi08x_data_sync_cfg sync_cfg;
int8_t rslt;
struct bmi08x_sensor_data accel;
struct bmi08x_sensor_data gyro;
uint8_t data = 0;
int count2=0;

//MS5607
struct MS5607UncompensatedValues baroRaw;
struct MS5607Readings baro;
double alt;

// W25Q128
uint8_t testWrite[4] = {1, 2, 3, 4};
uint8_t testRead[4];
SPIF_HandleTypeDef spif;

void LEDWrite(int r, int g, int b) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, r);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, b);
}
void Error(char* err) {
	while (1) {
		LEDWrite(0, 0, 0);
		HAL_Delay(1000);
		LEDWrite(255, 0, 0);
		HAL_Delay(1000);
		printf("%s\n", err);
	}
}


void BMI088Init() {
	/* Initialize bmi088 sensors (accel & gyro)*/
	rslt = bmi088_init(&dev);
	if (rslt != BMI08X_OK) {
		Error("BMI088 Initialization Failure");
	}

	// Reset the accelerometer and wait for 1 ms - delay taken care inside the function
	rslt = bmi08a_soft_reset(&dev);
	if (rslt != BMI08X_OK) {
		Error("BMI088 Initialization Failure");
	}

	// ! Max read/write length (maximum supported length is 32). To be set by the user
	dev.read_write_len = 32;
	// set accel power mode
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	rslt = bmi08a_set_power_mode(&dev);
	if (rslt != BMI08X_OK) {
		Error("BMI088 Initialization Failure\n");
	}
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	bmi08g_set_power_mode(&dev);
	// API uploads the bmi08x config file onto the device and wait for 150ms to enable the data synchronization - delay taken care inside the function
	rslt = bmi088_apply_config_file(&dev);
	if (rslt != BMI08X_OK) {
		Error("BMI088 Initialization Failure\n");
	}
	// assign accel range setting
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
	// assign gyro range setting
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
	// ! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz)
	sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;
	rslt = bmi088_configure_data_synchronization(sync_cfg, &dev);
	if (rslt != BMI08X_OK) {
		Error("BMI088 Initialization Failure\n");
	}

	/*// set accel interrupt pin configuration
	// configure host data ready interrupt
	int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	//configure Accel syncronization input interrupt pin
	int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	//set gyro interrupt pin configuration
	int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
	int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

	int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

	// Enable synchronization interrupt pin
	rslt = bmi088_set_data_sync_int_config(&int_config, &dev);
	if (rslt != BMI08X_OK) {
		char v[4];
		sprintf(&v, "%d", rslt);
		Error(&v[0]);
		Error("BMI088 Initialization Failure\n");
	}*/
}

void MS5607Init() {
	if (MS5607_Init(&hspi1, BARO_CS_GPIO_Port, BARO_CS_Pin) == MS5607_STATE_FAILED) {
		Error("MS5607 Initialization Failure");
	}
}

void SPIFInit() {
	if (!SPIF_Init(&spif, &hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin)) {
		Error("FLASH Initialization Failure");
	}
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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  LEDWrite(255, 0, 0); // Initialize phase

  MS5607Init();
  BMI088Init();
  SPIFInit();

	if (!SPIF_EraseSector(&spif, 0)) {
		Error("FLASH Fail");
	}
	if (!SPIF_WritePage(&spif, 0, testWrite, 4, 0)) {
		Error("FLASH Fail");
	}
	if (!SPIF_ReadPage(&spif, 0, testRead, 4, 0)) {
		Error("FLASH Fail");
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 LEDWrite(0, 0, 0);
	 //bmi088_init(&dev);
	 rslt = bmi088_get_synchronized_data(&accel, &gyro, &dev);
	 if (rslt != BMI08X_OK) {
		 Error("BMI088 Read Failure\n");
	 }

	 MS5607UncompensatedRead(&baroRaw);
	 MS5607Convert(&baroRaw, &baro);
	 alt = (44330.0f * (1.0f - pow((double)baro.pressure / 101325.0f, 0.1902949f)));


	 printf("ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d,alt:%f,temp:%d,pressure:%d\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, alt, baro.temperature, baro.pressure);

	 HAL_Delay(25);
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @PAram  None
  * @retval None
  */
int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*) ptr, len); return len;
}
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

#ifdef  USE_FULL_ASSERT
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
