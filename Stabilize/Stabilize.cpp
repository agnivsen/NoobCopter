/*
  Stabilize.cpp 	
*/

#include "Arduino.h"
#include "Stabilize.h"

Stabilize::Stabilize(double kP, double kI, double kD)
{
	_kP = kP;
	_kI = kI;
	_kD = kD;
	
	previousDerivativeError = 0;
	
}

void Stabilize::updateGain(double kP, double kI, double kD)
{
	_kP = kP;
	_kI = kI;
	_kD = kD;
}

double Stabilize::compute(double error)
{

/* Serial.print((((proportionalError = abs(error)) * _kP) + (integralError * _kI) + (derivativeError * _kD)));
Serial.print(",");
Serial.print((((proportionalError = abs(error)) * _kP) + (integralError * _kI)));
Serial.println(); */
	return (((proportionalError = abs(error)) * _kP) + (abs(integralError) * _kI) + (derivativeError * _kD));
}


double Stabilize::getDerivativeError()
{
	return derivativeError;
}

double Stabilize::getIntegralError()
{
	return integralError;
}

void Stabilize::errorCorrection(double proportionalErrorNow,double proportionalErrorBefore,long int timeElapsedInMicros)
{

	
	if(abs(proportionalErrorNow)>1)
	{
	//if(abs(proportionalErrorNow) > abs(proportionalErrorBefore))
		{
			integralError += proportionalErrorNow;
		}
	derivativeError = ((abs(proportionalErrorNow) - abs(proportionalErrorBefore))/timeElapsedInMicros) * 1000000;
	derivativeError = (DerivativeGainFactor * derivativeError) + ((1 - DerivativeGainFactor) * previousDerivativeError);
	
	}
	else
	{
		derivativeError = 0;
		integralError = 0;
	}
	derivativeError<0?derivativeError = 0:true;
	previousDerivativeError = derivativeError;
	
}