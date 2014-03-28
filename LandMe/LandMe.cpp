

#include "Arduino.h"

#include "LandMe.h"

	LandMe::LandMe(int modeSelect, float maxValue, float minValue)
	{
		landingMode = modeSelect;
		maxVal = maxValue;
		minVal = minValue;
	}
	
	float LandMe::landingValue (float totalTimeInMicros, float elapsedTimeInMicros)
	{
	//Serial.print("dfk");
		switch(landingMode)
		{
			case CUBIC_EXP_DECAY:
			{
			/*
			Serial.print("elapsed time : ");
			Serial.print(elapsedTimeInMicros);
			Serial.print("total time : ");
			Serial.print(totalTimeInMicros);
			Serial.println();
			Serial.println((elapsedTimeInMicros/totalTimeInMicros));
			Serial.println();
			Serial.print(",");
			Serial.print(totalTimeInMicros);
			Serial.println();
			*/
				return	((exp(-pow((2*(float)(elapsedTimeInMicros/totalTimeInMicros)),3)) * (maxVal - minVal)) + minVal);
			}	
			break;
			case QUADRIATIC_EXP_DECAY:
			{
				return	((exp(-pow((2*(float)(elapsedTimeInMicros/totalTimeInMicros)),4)) * (maxVal - minVal)) + minVal);
			}
			break;
			case LINEAR_DECAY:
			{
				return (((1 - ((float)(elapsedTimeInMicros/totalTimeInMicros))) * (maxVal - minVal)) + minVal);
			}
			break;
			default: break;
		}
		
		return 0.0000;
	}