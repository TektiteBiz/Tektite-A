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


int noDetachUntil = 0; // For ServoMin/ServoMax command
void StandbyUpdate() {
	if (sensorBuf.zero == 0) { // Has data!
		LEDWrite(255, 128, 0); // Orange
	} else if (HAL_GetTick() < noDetachUntil) {
		int val = (noDetachUntil - HAL_GetTick())/2;
		LEDWrite(val, val, val);
	} else {
		LEDWrite(0, 128, 0); // Green
	}

	if (HAL_GetTick() > noDetachUntil) {
		ServoDetach();
	}

	if (commandAvailable) {
		switch (command.commandType) {
			case ServoMin:
				if (command.config.s1min != -1) {
					config.s1min = command.config.s1min;
					ServoWriteS1(0);
				} else if (command.config.s2min != -1) {
					config.s2min = command.config.s2min;
					ServoWriteS2(0);
				} else if (command.config.s3min != -1) {
					config.s3min = command.config.s3min;
					ServoWriteS3(0);
				}
				noDetachUntil = HAL_GetTick() + 255 * 2;
				break;
			case ServoMax:
				if (command.config.s1max != -1) {
					config.s1max = command.config.s1max;
					ServoWriteS1(90);
				} else if (command.config.s2max != -1) {
					config.s2max = command.config.s2max;
					ServoWriteS2(90);
				} else if (command.config.s3max != -1) {
					config.s3max = command.config.s3max;
					ServoWriteS3(90);
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
	if (state.azr < 0.8 && sensorBuf.zero != 0) { // Flipped upside down and no data
		currentState = ARMED;
		return;
	}
}

void ArmedUpdate() {
	LEDWrite(255, 0, 0); // Red

	if (state.azr > 3) { // >3G acceleration = liftoff!
		ResetTime();
		currentState = BURN;
		return;
	}
}

void BurnUpdate() {
	LEDWrite(128, 0, 255); // Purple
	WriteState(false);

	if (GetTime() >= config.burntime) {
		currentState = CONTROL;
		return;
	}
}

void ControlUpdate() {
	LEDWrite(0, 255, 128); // Teal
	WriteState(false);

	if (state.vz < -1.5f) {
		currentState = DESCENT;
		return;
	}
}

void DescentUpdate() {
	LEDWrite(0, 0, 255); // Blue
	WriteState(false);

	if (abs(state.vz) < 1.5 && abs(state.alt) < 2) {
		currentState = STANDBY;
		if (sensorBuf.sampleCount > 0) {
			WriteState(true);
		}
		return;
	}
}

void StateUpdate() {
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
