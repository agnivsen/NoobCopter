

#ifndef LandMe_h
#define LandMe_h

#include "Arduino.h"

#define CUBIC_EXP_DECAY 1			//follows the equation y = exp(-x^3)
#define QUADRIATIC_EXP_DECAY 2		//follows equation y = exp(-x^2)
#define LINEAR_DECAY 3				//follows equation y = (1 - x)

class LandMe
{
	public:	
	LandMe(int modeSelect, float maxValue, float minValue);
	float landingValue (float totalTimeInMicros, float elapsedTimeInMicros);
	
	private:
	int landingMode;
	float maxVal, minVal;
};

#endif