/*
  IMU.h 
*/
#ifndef BlockingRC_h
#define BlockingRC_h

#include "Arduino.h"

#define MAXPIN 7


class BlockingRC
{
  public:
    
	BlockingRC();
	int setPin(int pin);
	bool removePin(int pinIndex);
	float getValue(int pinIndex);
	float getValueWithDelay(int pinIndex, long int delayInMicros);
	
	

  private:
	int globalPinCounter;
	long int lastReadTime;
	int pin[7];
	int lastValue;
	
};

#endif