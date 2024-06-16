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
void StoreConfig();
uint32_t GetTime();

#pragma pack(1)
typedef struct {
	uint32_t time;
	// Useful
	float alt;
	float vz;
	float vx;
	float vy;
	float az;
	float pre;
	float s1;

	// Filtered
	float ax;
	float ay;

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

	// Controller
	float s2;
	float s3;

	uint8_t state;
} State;

extern State state;

float getZAlt();
float getZVel();
float getZAccel();
void estimate(float accel[3], float gyro[3], float baroHeight);

#pragma pack(1)
typedef struct {
	uint32_t init; // If 0, then it has been initialized

	// Servo config
	int s1min;
	int s2min;
	int s3min;

	int s1max;
	int s2max;
	int s3max;

	// Flight mode
	bool control;
	float param; // If not under control, use this as the angle, if under control, use this as the target altitude
	uint32_t burntime; // Milliseconds
	float alpha; // Rho * A
	float mass; // kg
} Config;

extern Config config;

typedef struct {
	uint32_t zero; // If this is zero, then that means that there is data (because NOR flash stores as 1s)
	uint8_t sampleCount;
	State buf[42]; // I chose 42 because the max that could fit is 47 and 42 is nice
} SensorBuf;
extern SensorBuf sensorBuf;

void WriteState(bool finished);
void SendData();

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#endif /* INC_SENSOR_H_ */
