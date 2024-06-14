/*
 * sensor.h
 *
 *  Created on: Jun 13, 2024
 *      Author: nv
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "adc.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>

#include "BMI088.h"
#include "MS5607SPI.h"
#include "spif.h"


void LEDWrite(int r, int g, int b);
void Error(char* err);
void SensorInit();
double BattVoltage();
void ServoWriteS1(float angle);
void ServoWriteS2(float angle);
void ServoWriteS3(float angle);
void ServoDetach();
void SensorUpdate();
void ResetTime();

typedef struct {
	uint32_t time;

	// Unfiltered
	float axr;
	float ayr;
	float azr;
	float gxr;
	float gyr;
	float gzr;

	float altr;
	float baro;
	float temp;

	// Filtered
	float ax;
	float ay;
	float az;

	float vx;
	float vy;
	float vz;

	float alt;
} State;


extern State state;

float getZAlt();
float getZVel();
float getZAccel();
void estimate(float accel[3], float gyro[3], float baroHeight);

#endif /* INC_SENSOR_H_ */
