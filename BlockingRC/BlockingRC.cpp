/*
  IMU.cpp 	
*/

#include "Arduino.h"
#include "BlockingRC.h"

BlockingRC::BlockingRC()
{
}


int BlockingRC::setPin(int inputPin)
{
lastReadTime = micros();
if(globalPinCounter<6)
{
	pin[globalPinCounter++] = inputPin;
	 pinMode(inputPin, INPUT);
	return (globalPinCounter - 1);
}
else
	return -1;
}


bool BlockingRC::removePin(int pinIndex)
{
	if (pinIndex>=globalPinCounter)
	{
		return false;
	}
	else if (pinIndex>=0)
	{
		if(pinIndex == (globalPinCounter - 1))
		{
			pin[--globalPinCounter] = 0;
			return true;
		}
		else if (pinIndex < (globalPinCounter - 1))
		{
			for(int i = pinIndex; i<globalPinCounter-1; i++)
			{
				pin[i] = pin[i+1];
			}
			globalPinCounter--;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

float BlockingRC::getValue(int pinIndex)
{
	//lastReadTime = micros();
	return pulseIn(pin[pinIndex], HIGH, 25000);
}

float BlockingRC::getValueWithDelay(int pinIndex, long int delayInMicros)
{
	long int currentTime = micros();
	if((currentTime - lastReadTime)>delayInMicros)
	{
	lastReadTime = currentTime;
	return lastValue = pulseIn(pin[pinIndex], HIGH, 25000);
	}
	else
	{
	return lastValue;
	}
}