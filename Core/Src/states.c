/*
 * states.c
 *
 *  Created on: Jun 13, 2024
 *      Author: nv
 */


#include "states.h"
#include <stdlib.h>

enum State currentState = STANDBY;
Command command;
bool commandAvailable;


float totalAccelHistory[12]; // Standby data
int histIdx = 0;

int noDetachUntil = 0; // For ServoMin/ServoMax command
double battVoltage = 0;
void StandbyUpdate() {
	if (sensorBuf.zero == 0) { // Has data!
		LEDWrite(255, 128, 0); // Orange
	} else if (HAL_GetTick() < noDetachUntil) {
		int val = (noDetachUntil - HAL_GetTick())/2;
		LEDWrite(val, val, val);
	} else if (battVoltage < 5.1) { // Powered off of USB
			LEDWrite(32, 32, 32); // White for USB
	} else if (battVoltage < 7.4) {
		LEDWrite(0, 0, 128); // Blue for low battery
	} else {
		LEDWrite(0, 128, 0); // Green
	}

	if (HAL_GetTick() > noDetachUntil) {
		ServoDetach();
	}

	if (commandAvailable) {
		switch (command.commandType) {
			case ServoMin:
				if (command.config.s1min != 0) {
					int s1min = config.s1min;
					config.s1min = command.config.s1min;
					ServoWriteS1(0);
					config.s1min = s1min;
				} else if (command.config.s2min != 0) {
					int s2min = config.s2min;
					config.s2min = command.config.s2min;
					ServoWriteS2(0);
					config.s2min = s2min;
				} else if (command.config.s3min != 0) {
					int s3min = config.s3min;
					config.s3min = command.config.s3min;
					ServoWriteS3(0);
					config.s3min = s3min;
				}
				noDetachUntil = HAL_GetTick() + 255 * 2;
				break;
			case ServoMax:
				if (command.config.s1max != 0) {
					int s1max = config.s1max;
					config.s1max = command.config.s1max;
					ServoWriteS1(90);
					config.s1max = s1max;
				} else if (command.config.s2max != 0) {
					int s2max = config.s2max;
					config.s2max = command.config.s2max;
					ServoWriteS2(90);
					config.s2max = s2max;
				} else if (command.config.s3max != 0) {
					int s3max = config.s3max;
					config.s3max = command.config.s3max;
					ServoWriteS3(90);
					config.s3max = s3max;
				}
				noDetachUntil = HAL_GetTick() + 255 * 2;
				break;
			case Status:
				StatusData status;
				status.hasData = sensorBuf.zero == 0;
				status.config = config;
				CDC_Transmit_FS((uint8_t*)(&status), sizeof(status));
				break;

			case ConfigWrite:
				config = command.config;
				StoreConfig();
				break;

			case DataRead:
				SendData();
				break;
		}
		commandAvailable = false;
	}

	// Next state
	if (state.azr < -8 && sensorBuf.zero != 0 && battVoltage > 5.1) { // Flipped upside down, no flight data, and powered off of battery
		LEDWrite(0, 0, 0);
		HAL_Delay(1000);
		for (int i = 0; i < 12; i++) {
			totalAccelHistory[i] = 100;
		}
		histIdx = 0;
		currentState = ARMED;
		return;
	}
}

void ArmedUpdate() {
	LEDWrite(255, 0, 0); // Red

	float totalAccel = sqrt(pow(state.axr, 2) + pow(state.ayr, 2) + pow(state.azr, 2));
	totalAccelHistory[histIdx] = totalAccel;

	histIdx++;
	if (histIdx >= 12) {
		histIdx = 0;
	}

	float accelSum = 0;
	for (int i = 0; i < 12; i++) {
		accelSum += totalAccelHistory[i];
	}
	accelSum /= 12.0f;
	//printf("%f\n", accelSum);
	if (accelSum < 9.85 && accelSum > 9.75) {
		SensorFilterReset(); // Reset filter if on launchpad
		//LEDWrite(255, 255, 255);
	} else {
		SensorFilterUpdate(); // Update filter when starting to accelerate
		//LEDWrite(255, 0, 0); // Red
	}
	ServoWriteS1(0);
	ServoWriteS2(0);
	ServoWriteS3(0);

	if (state.azr > 30) { // >4G acceleration = liftoff!
		ResetTime();
		currentState = BURN;
		return;
	}
}

void BurnUpdate() {
	LEDWrite(128, 0, 255); // Purple
	SensorFilterUpdate();
	//state.alt = baroAlt;
	WriteState(false);

	ServoWriteS1(0);
	ServoWriteS2(0);
	ServoWriteS3(0);

	if (GetTime() >= config.starttime) {
		currentState = CONTROL;
		return;
	}
}

void ControlUpdate() {
	LEDWrite(0, 255, 128); // Teal
	SensorFilterUpdate();
	float Cd = fabsf(-2*(state.az + 9.81)/(config.alpha*pow(state.vz, 2)));
	state.pre = getApogee(((float)GetTime())/1000.0f, state.alt, state.vz, Cd);
	WriteState(false);

	if (config.control) {
		float ang = state.s1 + config.P*(state.pre - config.param);
		if (ang < 0.0f) {
			ang = 0.0f;
		} else if (ang > 90.0f) {
			ang = 90.0f;
		}
		ServoWriteS1(ang);
		ServoWriteS2(ang);
		ServoWriteS3(ang);
	} else {
		ServoWriteS1(config.param);
		ServoWriteS2(config.param);
		ServoWriteS3(config.param);
	}

	if (state.vz < -3.0f) {
		currentState = DESCENT;
		return;
	}
}

void DescentUpdate() {
	LEDWrite(160, 32, 240); // Purple
	SensorFilterUpdate();
	WriteState(false);

	ServoDetach();

	if (abs(state.vz) < 1.5 && abs(state.alt) < 2) {
		if (sensorBuf.sampleCount > 0) {
			WriteState(true);
		}
		currentState = STANDBY;
		battVoltage = BattVoltage();
		return;
	}
}

void StateUpdate() {
	state.state = currentState;
	switch (currentState) {
		case STANDBY:
			StandbyUpdate();
			break;

		case ARMED:
			ArmedUpdate();
			break;

		case BURN:
			BurnUpdate();
			break;

		case CONTROL:
			ControlUpdate();
			break;

		case DESCENT:
			DescentUpdate();
			break;
	}
}
