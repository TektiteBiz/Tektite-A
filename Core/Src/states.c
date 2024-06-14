/*
 * states.c
 *
 *  Created on: Jun 13, 2024
 *      Author: nv
 */


#include "states.h"
#include "sensor.h"

enum State currentState;

void StandbyUpdate() {
	LEDWrite(0, 128, 0); // Green
	ServoDetach();

	// Next state
	if (state.azr < 0.8) { // Flipped upside down
		currentState = ARMED;
		return;
	}
}

void ArmedUpdate() {
	LEDWrite(255, 0, 0); // Red
}

void BurnUpdate() {
	LEDWrite(128, 0, 255); // Purple
}

void ControlUpdate() {
	LEDWrite(0, 255, 128); // Teal
}

void DescentUpdate() {
	LEDWrite(0, 0, 255); // Blue
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
