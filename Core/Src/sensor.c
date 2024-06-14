/*
 * sensor.c
 *
 *  Created on: Jun 13, 2024
 *      Author: nv
 */

#include "sensor.h"

// BMI088
BMI088 imu;

//MS5607
struct MS5607UncompensatedValues baroRaw;
struct MS5607Readings baro;

// W25Q128
SPIF_HandleTypeDef spif;

void LEDWrite(int r, int g, int b) {
	htim1.Instance->CCR1 = b;
	htim1.Instance->CCR2 = r;
	htim1.Instance->CCR3 = g;
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

void LEDInit() {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

double BattVoltage() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return ((double)HAL_ADC_GetValue(&hadc1))*0.00251984291; // x/4096 * (100+47)/47 [voltage resistor] * 3.3 [vref]
}

void BMI088Init() {
	int res = BMI088_Init(&imu, &hspi1, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin);
	if (res != 0) {
		Error("BMI088 Initialization Failure");
	}
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

void ServoInit() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

// 0-120deg => 900-2100uS
// Divide value by 10 to get % of cycle that is spent on this, each cycle is 20ms since 50Hz
// 0.9ms pwm = 0.9/20 * 1000 = 45, 2.1 = 105
void ServoWriteS1(float angle) {
	htim2.Instance->CCR1 = (int)(angle/120.0f * (float)(105-45)) + 45;
}
void ServoWriteS2(float angle) {
	htim2.Instance->CCR2 = (int)(angle/120.0f * (float)(105-45)) + 45;
}
void ServoWriteS3(float angle) {
	htim2.Instance->CCR3 = (int)(angle/120.0f * (float)(105-45)) + 45;
}
void ServoDetach() {
	htim2.Instance->CCR1 = 0;
	htim2.Instance->CCR2 = 0;
	htim2.Instance->CCR3 = 0;
}

void SensorInit() {
	LEDInit();
	LEDWrite(128, 128, 128); // Initialize phase

	MS5607Init();
	BMI088Init();
	SPIFInit();
	ServoInit();

	LEDWrite(0, 0, 0);
}



// Sensor update
uint32_t start;
void ResetTime() {
	start = HAL_GetTick();
}

State state;
void SensorUpdate() {
	BMI088_ReadAccelerometer(&imu);
	BMI088_ReadGyroscope(&imu);
	MS5607UncompensatedRead(&baroRaw);
	MS5607Convert(&baroRaw, &baro);

	state.altr = (44330.0f * (1.0f - pow((double)baro.pressure / 101325.0f, 0.1902949f)));
	estimate(imu.acc_mps2, imu.gyr_rps, state.altr);

	// Copy to state
	state.time = HAL_GetTick() - start;

	state.axr = imu.acc_mps2[0];
	state.ayr = imu.acc_mps2[1];
	state.azr = imu.acc_mps2[2];
	state.gxr = imu.gyr_rps[0];
	state.gyr = imu.gyr_rps[1];
	state.gzr = imu.gyr_rps[2];
	state.baro = (float)baro.pressure;
	state.temp = 0.0001f * (float)baro.temperature;

	state.ax = 0; // TODO: Make ax and ay actually get computed
	state.ay = 0;
	state.az = getZAccel();

	state.vx = 0;
	state.vy = 0;
	state.vz = getZVel();

	state.alt = getZAlt();
}
