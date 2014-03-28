/*
  IMU.h 
*/
#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include "SPI.h"

#define ToD(x) (x/131)
#define ToG(x) (x*9.80665/16384)

class IMU
{
  public:
    
	IMU(int chipSelectPin);
	
	void initialize();
	int acceX();
	int acceY();
	int acceZ();

	int gyroX();
	int gyroY();
	int gyroZ();

	float acceDeg(int axisSelect); // 0 - X axis, 1 - Y axis, 2 - Z axis
	float gyroDeg(int axisSelect); // 0 - X axis, 1 - Y axis, 2 - Z axis

  private:
	uint8_t SPIread(byte reg);
	
	void SPIwrite(byte reg, byte data);
	
	void ConfigureMPU6000();

	int _chipSelectPin;
	
	int time;
    int time_old;
	
	float angleX;
	float angleY;
	float angleZ;
	
	
};

#endif