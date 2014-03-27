/*
  BlockingRC.cpp
*/

#include "Arduino.h"
#include "BlockingRC.h"

BlockingRC::BlockingRC()
{
}


int BlockingRC::setPin(int inputPin)    //set the pin number of the RC object. goes into a global array
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


bool BlockingRC::removePin(int pinIndex)    //remove the pin from global array
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

float BlockingRC::getValue(int pinIndex)        //get the current RC reading for the channel 'pinIndex'
{
	//lastReadTime = micros();
	return pulseIn(pin[pinIndex], HIGH, 25000);
}

float BlockingRC::getValueWithDelay(int pinIndex, long int delayInMicros)   //get the current RC reading for the channel 'pinIndex', provided that the time elapsed since the last reading was taken is greater than delayInMicros
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