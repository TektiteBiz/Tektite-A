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

ReplayData replayPacket[8];
void ReplayFlight() {
	LEDWrite(115, 3, 170);
	replayPacket[0].delay = 1;
	uint8_t ack = 1;
	while (1) {
		CDC_Transmit_FS(&ack, sizeof(ack));

		while (!commandAvailable) {
			HAL_Delay(1);
		}

		for (int i = 0; i < 8; i++) {
			if (replayPacket[i].delay < 0) {
				ServoWriteS1(0);
				ServoWriteS2(0);
				ServoWriteS3(0);
				HAL_Delay(250);
				return;
			}

			HAL_Delay(replayPacket[i].delay);
			ServoWriteS1(replayPacket[i].servo);
			ServoWriteS2(replayPacket[i].servo);
			ServoWriteS3(replayPacket[i].servo);
		}
	}
	LEDWrite(0, 0, 0);
}



float totalAccelHistory[12]; // Standby data
int histIdx = 0;
int sampleCount = 0;
uint32_t lastWrite = 0;

int noDetachUntil = 0; // For ServoMin/ServoMax command
float battVoltage = 0;
void StandbyUpdate() {
	if (sensorBuf.zero == 0) { // Has data!
		LEDWrite(255, 128, 0); // Orange
	} else if (HAL_GetTick() < noDetachUntil) {
		int val = (noDetachUntil - HAL_GetTick())/2;
		LEDWrite(val, val, val);
	} else if (battVoltage < 5.1) { // Powered off of USB
		#ifdef LANDING
			LEDWrite(32, 0, 0); // Red for landing rocket mode
		#else
			LEDWrite(32, 32, 32); // White for USB
		#endif
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

			case FlightReplay:
				ReplayFlight();
				break;

		}
		commandAvailable = false;
	}

	// Next state
	if (state.azr < -8 && sensorBuf.zero != 0 && battVoltage > 5.1) { // Flipped upside down, no flight data, and powered off of battery
		LEDWrite(0, 0, 0);
		uint32_t start = HAL_GetTick();
		while (HAL_GetTick() - start < 1000) { // Wait for 1 second of holding upside down
			SensorRawUpdate();
			if (state.azr > -8) {
				return;
			}
			HAL_Delay(1);
		}

		// Arm rocket
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
	state.servo = 0;
	state.target = 0;

	if (state.azr > 30) { // >4G acceleration = liftoff!
		ResetTime();
		sampleCount = 0;
		currentState = BURN;
		return;
	}
}


void BurnUpdate() {
	LEDWrite(128, 0, 255); // Purple
	SensorFilterUpdate();

	sampleCount++;
	if (HAL_GetTick() - lastWrite > 10) { // Write data every 10 milliseconds on ascent
		state.samples = sampleCount;
		WriteState(false);
		lastWrite = HAL_GetTick();
		sampleCount = 0;
	}

	ServoWriteS1(0);
	ServoWriteS2(0);
	ServoWriteS3(0);
	state.servo = 0;

	if (GetTime() >= config.starttime) {
		currentState = CONTROL;
		return;
	}
}

#ifdef LANDING
uint32_t pyroStart = 0;
#endif

void ControlUpdate() {
	LEDWrite(0, 255, 128); // Teal
	SensorFilterUpdate();
	float Cd = fabsf(-2*config.mass*(state.az + 9.81)/(config.alpha*pow(state.vz, 2)));
	state.pre = getApogee(((float)GetTime())/1000.0f, state.alt, delayedVel, Cd);
	//float target = (state.alt/(GetUncompensatedAlt(state.baro) - uncompensatedAltOffset))*config.param; // Un-temperature compensate the target altitude
	float target = ((state.temp + 273.15)/286.65f)*config.param; // Un-temperature compensate the target altitude (using algebra), https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure
	state.target = target;

	sampleCount++;
	if (HAL_GetTick() - lastWrite > 10) { // Write data every 10 milliseconds on ascent
		state.samples = sampleCount;
		WriteState(false);
		lastWrite = HAL_GetTick();
		sampleCount = 0;
	}

	#ifdef LANDING // Need pyro channel disabled on ascent when in landing mode
	ServoWriteS2(0);
	#endif

	if (config.control) {
		float ang = state.servo + config.P*(state.pre - target);
		if (ang < 0.0f) {
			ang = 0.0f;
		} else if (ang > 90.0f) {
			ang = 90.0f;
		}
		ServoWriteS1(ang);
		#ifndef LANDING
		ServoWriteS2(ang);
		#endif
		ServoWriteS3(ang);
		state.servo = ang;
	} else {
		ServoWriteS1(config.param);
		#ifndef LANDING
		ServoWriteS2(config.param);
		#endif
		ServoWriteS3(config.param);
		state.servo = config.param;
	}

	if (state.vz < -3.0f) {
		currentState = DESCENT;
		#ifdef LANDING
		pyroStart = 0;
		#endif
		return;
	}
}

void DescentUpdate() {
	LEDWrite(160, 32, 240); // Purple
	SensorFilterUpdate();

	sampleCount++;
	if (HAL_GetTick() - lastWrite > 20) { // Write data every 20 milliseconds on descent
		state.samples = sampleCount;
		WriteState(false);
		lastWrite = HAL_GetTick();
		sampleCount = 0;
	}

	state.servo = 0;

	#ifdef LANDING
	// Servo output 2 used for pyro channel
	if (state.vz > -52.5 && state.alt > 88.5) {
		ServoWriteS1(0);
		ServoWriteS2(0);
		ServoWriteS3(0);
	} else if (HAL_GetTick() - pyroStart > 1000 && pyroStart != 0) {
		ServoDetach();
	} else { // Fire pyro channel
		if (pyroStart == 0) {
			pyroStart = HAL_GetTick();
		}
		ServoWriteS2(90);
	}
	#else
	if (state.alt > 50) { // Make canards vertical
		ServoWriteS1(0);
		ServoWriteS2(0);
		ServoWriteS3(0);
	} else {
		ServoDetach();
	}

	if (abs(state.az) > 20 && abs(state.alt) < 10) { // Detect shaking on impact with the ground
		if (sensorBuf.sampleCount > 0) {
			WriteState(true);
		}
		currentState = STANDBY;
		battVoltage = BattVoltage();
		return;
	}
	#endif
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
