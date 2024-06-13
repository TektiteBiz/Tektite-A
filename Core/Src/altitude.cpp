/*
    altitude.cpp: Altitude estimation via barometer/accelerometer fusion
*/

#include "filters.h"
#include "algebra.h"
#include "altitude.h"

AltitudeEstimator::AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro,
                                     float ca, float accelThreshold)
:kalman(ca, sigmaGyro, sigmaAccel), complementary(sigmaAccel, sigmaBaro, accelThreshold)
{
      this->sigmaAccel = sigmaAccel;
      this->sigmaGyro = sigmaGyro;
      this->sigmaBaro = sigmaBaro;
      this->ca = ca;
      this->accelThreshold = accelThreshold;
}

void AltitudeEstimator::estimate(float accel[3], float gyro[3], float baroHeight, uint32_t timestamp)
{
        float deltat = (float)(timestamp-previousTime)/1000000.0f;
        float verticalAccel = kalman.estimate(pastGyro,
                                              pastAccel,
                                              deltat);
        complementary.estimate(& estimatedVelocity,
                               & estimatedAltitude,
                               baroHeight,
                               pastAltitude,
                               pastVerticalVelocity,
                               pastVerticalAccel,
                               deltat);
        // update values for next iteration
        copyVector(pastGyro, gyro);
        copyVector(pastAccel, accel);
        pastAltitude = estimatedAltitude;
        pastVerticalVelocity = estimatedVelocity;
        pastVerticalAccel = verticalAccel;
        previousTime = timestamp;
}

float AltitudeEstimator::getAltitude()
{
        // return the last estimated altitude
        return estimatedAltitude;
}

float AltitudeEstimator::getVerticalVelocity()
{
        // return the last estimated vertical velocity
        return estimatedVelocity;
}

float AltitudeEstimator::getVerticalAcceleration()
{
        // return the last estimated vertical acceleration
        return pastVerticalAccel;
}


// Altitude estimator
AltitudeEstimator altitude = AltitudeEstimator(0.0005, 	// sigma Accel
                                               0.0005, 	// sigma Gyro
                                               0.018,   // sigma Baro
                                               0.5, 	// ca
                                               0.1);	// accelThreshold

void estimate(float accel[3], float gyro[3], float baroHeight) {
	altitude.estimate(accel, gyro, baroHeight, GetMicros());
}
float getZAlt() {
	return altitude.getAltitude();
}
float getZVel() {
	return altitude.getVerticalVelocity();
}
float getZAccel() {
	return altitude.getVerticalAcceleration();
}
