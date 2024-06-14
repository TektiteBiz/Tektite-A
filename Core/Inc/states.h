/*
 * states.h
 *
 *  Created on: Jun 13, 2024
 *      Author: nv
 */

#ifndef INC_STATES_H_
#define INC_STATES_H_


enum State {
	STANDBY, // Ground, unarmed, can connect to computer
	ARMED, // Armed, ready to launch
	BURN, // Motor burning
	CONTROL, // Active control (after motor burn)
	DESCENT, // Parachute descent
};

extern enum State currentState;
void StateUpdate();

#endif /* INC_STATES_H_ */
