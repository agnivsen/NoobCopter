/*
  Stabilize.h 
*/
#ifndef Stabilize_h
#define Stabilize_h

#include "Arduino.h"

#define IMUPIN 53          //you can also try PIN 42, 38 and 25, but they won't work

#define RollSetValue 0    //might be changed to a non-zero value in future
#define PitchSetValue 0    //however, the likelihood of these two remaining zero for eternity is very high

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