/*
  IMU.cpp 
 */

#include "Arduino.h"
#include "SPI.h"

#include "IMU.h"

IMU::IMU(int chipSelectPin)
{
_chipSelectPin = chipSelectPin;
}

void IMU::initialize()
{
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH); 	//Oh God, Why????????

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);


  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);


  pinMode(_chipSelectPin, OUTPUT);

  ConfigureMPU6000();
  
  time = time_old = 0;
  angleX = angleY = angleZ = 0;

}

/*All the private methods are given below:*/

uint8_t IMU::SPIread(byte reg) {
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr=reg|0x80;
  digitalWrite(_chipSelectPin,LOW);
  dump=SPI.transfer(addr);
  return_value=SPI.transfer(0x00);
  digitalWrite(_chipSelectPin,HIGH);
  return(return_value);
}



void IMU::SPIwrite(byte reg, byte data) {
  uint8_t dump;
  digitalWrite(_chipSelectPin,LOW);
  dump=SPI.transfer(reg);
  dump=SPI.transfer(data);
  digitalWrite(_chipSelectPin,HIGH);
}

void IMU::ConfigureMPU6000()
{
  // DEVICE_RESET @ PWR_MGMT_1, reset device
  SPIwrite(0x6B,0x80);
  delay(150);

  // TEMP_DIS @ PWR_MGMT_1, wake device and select GyroZ clock
  SPIwrite(0x6B,0x03);
  delay(150);

  // I2C_IF_DIS @ USER_CTRL, disable I2C interface
  SPIwrite(0x6A,0x10);
  delay(150);

  // SMPRT_DIV @ SMPRT_DIV, sample rate at 1000Hz
  SPIwrite(0x19,0x00);
  delay(150);

  // DLPF_CFG @ CONFIG, digital low pass filter at 42Hz
  SPIwrite(0x1A,0x03);
  delay(150);

  // FS_SEL @ GYRO_CONFIG, gyro scale at 250dps
  SPIwrite(0x1B,0x00);
  delay(150);

  // AFS_SEL @ ACCEL_CONFIG, accel scale at 2g (1g=8192)
  SPIwrite(0x1C,0x00);
  delay(150);
}

/*Private Methods End, public methods begin here : */

//--- Function to obtain angles based on accelerometer readings ---//
float IMU::acceDeg(int axisSelect) {
  float Ax=ToG(acceX());
  float Ay=ToG(acceY());
  float Az=ToG(acceZ());
  
  float ADegX, ADegY, ADegZ;
  
  
  switch (axisSelect)
  {
    case 0:
	ADegX=((atan(Ax/(sqrt((Ay*Ay)+(Az*Az)))))/PI)*180;
    return ADegX;
    break;
    case 1:
	ADegY=((atan(Ay/(sqrt((Ax*Ax)+(Az*Az)))))/PI)*180;
    return ADegY;
    break;
    case 2:
	ADegZ=((atan((sqrt((Ax*Ax)+(Ay*Ay)))/Az))/PI)*180;
    return ADegZ;
    break;
  }
}


//--- Function to obtain angles based on gyroscope readings ---//
float IMU::gyroDeg(int axisSelect) {
  time_old=time;
  time=millis();
  float dt=time-time_old;
  if (dt>=1000)
  {
    dt=0;
  }
  float Gx;
  float Gy;
  float Gz;
  
  
  
  switch (axisSelect)
  {
    case 0:
	{
		Gx=ToD(gyroX());
		if (Gx>0 && Gx<1.4)
		{
			Gx=0;
		}
		angleX+=Gx*(dt/1000);
		return angleX;
	}
    break;
    case 1:
	{
		Gy=ToD(gyroY());
		angleY+=Gy*(dt/1000);
		return angleY;
	}
    break;
    case 2:
	{
		Gz=ToD(gyroZ());
		angleZ+=Gz*(dt/1000);
		return angleZ;
	}
    break;
  }
}



int IMU::acceX() {   //this returns data along X axis

  uint8_t AcceY_H=SPIread(0x3D);
  uint8_t AcceY_L=SPIread(0x3E);
  int16_t AcceY=AcceY_H<<8|AcceY_L;
  return(AcceY); 
}

int IMU::acceY() {    //this return data along Y axis
  uint8_t AcceX_H=SPIread(0x3B);
  uint8_t AcceX_L=SPIread(0x3C);
  int16_t AcceX=AcceX_H<<8|AcceX_L;
  return(AcceX);
}


int IMU::acceZ() {
  uint8_t AcceZ_H=SPIread(0x3F);
  uint8_t AcceZ_L=SPIread(0x40);
  int16_t AcceZ=AcceZ_H<<8|AcceZ_L;
  return(AcceZ);
}


int IMU::gyroX() {
  uint8_t GyroY_H=SPIread(0x45);
  uint8_t GyroY_L=SPIread(0x46);
  int16_t GyroY=GyroY_H<<8|GyroY_L;
  return(GyroY);
}


int IMU::gyroY() {
  uint8_t GyroX_H=SPIread(0x43);
  uint8_t GyroX_L=SPIread(0x44);
  int16_t GyroX=GyroX_H<<8|GyroX_L;
  return(GyroX);
}


int IMU::gyroZ() {
  uint8_t GyroZ_H=SPIread(0x47);
  uint8_t GyroZ_L=SPIread(0x48);
  int16_t GyroZ=GyroZ_H<<8|GyroZ_L;
  return(GyroZ);
}


