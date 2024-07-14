/*
 * filter.c
 *
 *  Created on: Jun 19, 2024
 *      Author: nv
 */


#include "algebra.h"

float C[3][3];
uint32_t prev;

float globalAccel[3] = {0.0, 0.0, 0.0}; // Global acceleration
float velocity[3] = {0.0, 0.0, 0.0};
float altitude = 0;


uint32_t startTime;
const uint32_t delay = 100; // ms
float timeBuffer[40];
float velBuffer[40];
float accelBuffer[40];
int bufferIdx = 0;

float delayedVel = 0.0f;

// After this, acceleration will be 1 vertical and 0 every other way and velocity and altitude will be 0
void FilterInit(float a[3], float alt) {
	// Initialize rotation matrix
	if (a[2] == 0) {
		a[2] = 0.001;
	}
	float tmp[3];
	copyVector(tmp, a);
	normalizeVector(tmp);

	float cx = atanf(tmp[1]/tmp[2]);
	float cy = -asinf(tmp[0]);

	if (isnan(cx) || isnan(cy)) {
		return;
	}

	float scx = sin(cx);
	float ccx = cos(cx);
	float scy = sin(cy);
	float ccy = cos(cy);
	C[0][0] = ccy;
	C[0][1] = scx*scy;
	C[0][2] = ccx*scy;

	C[1][0] = 0;
	C[1][1] = ccx;
	C[1][2] = -scx;

	C[2][0] = -scy;
	C[2][1] = ccy*scx;
	C[2][2] = ccx*ccy;

	velocity[0] = 0.0f;
	velocity[1] = 0.0f;
	velocity[2] = 0.0f;

	// Initialize barometer filter
	altitude = 0;

	prev = GetMicros();
	bufferIdx = 0;
	startTime = HAL_GetTick();
}

void FilterUpdate(float g[3], float a[3], float alt) {
	// Update IMU
	if (g[0] == 0) {
		g[0] = 0.00000001;
	}
	if (g[1] == 0) {
		g[1] = 0.00000001;
	}
	if (g[2] == 0) {
		g[2] = 0.00000001;
	}

	uint32_t time = GetMicros();
	if ((time - prev) == 0) {
		//return;
		time++;
	}
	float dt = ((float)(time - prev))/1000000.0f;
	prev = time;


	// Rotation matrix code is from https://huskiecommons.lib.niu.edu/cgi/viewcontent.cgi?article=1032&context=allgraduate-thesesdissertations

	// Calculate sigma
	float sig;
	vectorLength(&sig, g);
	sig = dt*sig;
	float csig = sinf(sig)/sig;
	float ssig = (1-cosf(sig))/pow(sig, 2);

	// Calculate B
	float B[3][3];

	float phi[3][3]; // "B"
	skew(phi, g);
	scaleMatrix3x3(phi, dt, phi);

	float tmp[3][3]; // "B^2"
	matrixProduct3x3(tmp, phi, phi);

	identityMatrix3x3(B);
	scaleAndAccumulateMatrix3x3(B, csig, phi);
	scaleAndAccumulateMatrix3x3(B, ssig, tmp);

	// Save
	matrixProduct3x3(tmp, C, B);
	copyMatrix3x3(C, tmp);

	// Update velocity
	matrixDotVector3x3(globalAccel, C, a);
	velocity[0] = velocity[0] + dt*globalAccel[0];
	velocity[1] = velocity[1] + dt*globalAccel[1];

	float zAcc = globalAccel[2] - 9.81;
	globalAccel[2] = zAcc;
	velocity[2] = velocity[2] + dt*globalAccel[2];

	// Calculate time delay
	uint32_t tick = HAL_GetTick();
	if (tick - timeBuffer[bufferIdx] > 10) {
		bufferIdx++;
		if (bufferIdx >= 40) {
			bufferIdx = 0;
		}
		timeBuffer[bufferIdx] = tick;
		accelBuffer[bufferIdx] = zAcc;
		velBuffer[bufferIdx] = velocity[2];
	}

	if (tick - startTime < delay) {
		altitude = alt;
		return;
	}


	// Find the value from times
	int timeInd = bufferIdx;
	for (int i = 0; i < 40; i++) {
		if (tick - timeBuffer[timeInd] > delay) {
			break;
		}
		timeInd--;
		if (timeInd < 0) {
			timeInd = 39;
		}
	}


	// Barometer filter
	float prevAlt = altitude;
	altitude = 0.99f*(prevAlt + dt*velBuffer[timeInd] +
			accelBuffer[timeInd]*pow(dt, 2)/2) + (0.01f * alt); // Why is it 0.99 and 0.01?
	delayedVel = velBuffer[timeInd];
}


