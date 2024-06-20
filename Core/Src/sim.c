/*
 * sim.c
 *
 *  Created on: Jun 20, 2024
 *      Author: nv
 */




#include "sim.h"
#include "sensor.h"

// res is in the format az, ax
void calcAccel(float ti, float vzi, float vxi, float Cd, float res[2]) {
	res[0] = -0.5*config.alpha*Cd*pow(vzi, 2)/config.mass - 9.81;
	res[1] = -0.5*config.alpha*Cd*pow(vxi, 2)/config.mass;
}

const float h = 0.1;

// res is x, vz, vx
void solveIter(float ti, float xi, float vzi, float vxi, float Cd, float res[3]) {
	float k0z = h*vzi;
	float k0x = h*vxi;
	float l0[2];
	calcAccel(ti, vzi, vxi, Cd, l0);

	float k1z = h*(vzi+0.5*k0z);
	float k1x = h*(vxi+0.5*k0x);
	float l1[2];
	calcAccel(ti+0.5*h, vzi+0.5*k0z, vxi+0.5*k0x, Cd, l1);

	float k2z = h*(vzi + 0.5*k1z);
	float k2x = h*(vxi + 0.5*k1x);
	float l2[2];
	calcAccel(ti+0.5*h, vzi+0.5*k1z, vxi+0.5*k1x, Cd, l2);

	float k3z = h*(vzi+k2z);
	float l3[2];
	calcAccel(ti+h, vzi+k2z, vxi+k2x, Cd, l3);

	res[0] = xi + (1.0 / 6.0) * (k0z + 2.0 * k1z + 2.0 * k2z + k3z);
	res[1] = vzi + (1.0 / 6.0) * (l0[0] * h + 2.0 * l1[0] * h + 2.0 * l2[0] * h + l3[0] * h);
	res[2] = vxi + (1.0 / 6.0) * (l0[1] * h + 2.0 * l1[1] * h + 2.0 * l2[1] * h + l3[1] * h);
}

float getApogee(float ti, float xi, float vzi, float vxi, float Cd) {
	float vx = vxi;
	float vz = vzi;
	float x = xi;
	float t = ti;

	while (t < 0.001f || vz > 0.0f) {
		float res[3];
		solveIter(t, x, vz, vx, Cd, res);
		x = res[0];
		vz = res[1];
		vx = res[2];
		t += h;
	}

	return x;
}
