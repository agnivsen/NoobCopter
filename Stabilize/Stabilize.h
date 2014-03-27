/*
  Stabilize.h:
 
 Kinda implements a PID function, along with some cheesy complementary filters, LPF etc.
 
*/
#ifndef Stabilize_h
#define Stabilize_h

#include "Arduino.h"

#define IMUPIN 53          //I receive the IMU reading on this pin, for APM 2.6. For other kits, change the value shrewdly

#define RollSetValue 0    //might be changed to a non-zero value in future
#define PitchSetValue 0    //right now, I just want this piece of crap to hover horizintally

#define ROLL_ERROR(roll) (roll - RollSetValue)
#define PITCH_ERROR(pitch) (pitch - PitchSetValue)

#define COMPLEMENTARY(angle, gyroAngle, previousGyroAngle, accelAngle) ((0.97 * (angle + ((gyroAngle - previousGyroAngle)))) + (0.03 * accelAngle))  //the magical complementary filter, which is touted as the easier brethren of Kalman filter

#define LPF(val,prevVal,gainFactor) ((gainFactor*val) + ((1 - gainFactor)*prevVal))

#define DerivativeGainFactor 0.85

class Stabilize
{
  public:
    Stabilize(double kP, double kI, double kD);
	double compute(double error);
	void errorCorrection(double proportionalErrorNow,double proportionalErrorBefore,long int timeElapsedInMicros);
	double getDerivativeError();
	double getIntegralError();
	void updateGain(double kP, double kI, double kD);

  private:
	double _kP, _kD, _kI;
	double proportionalError, derivativeError, integralError;
	double previousDerivativeError;
	
};

#endif