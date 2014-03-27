/*
Status - Work in Progress.
Not to be released to the public.*/

/*Version - 0.1.1
Date - 27-MAR-2014, 12:40 PM

FlightStableNew.pde*/

#include <Servo.h>
#include <math.h>
#include <IMU.h>
#include <Stabilize.h>
#include <BlockingRC.h>


//hard coded values for roll and pitch gains
#define Kp_Roll  0.13//0.0069
#define Ki_Roll  0.00//0.000008
#define Kd_Roll  0.00014//0.015

#define Kp_Pitch  0.12//0.16 
#define Ki_Pitch  0.0000//20//1//5//9//1//1
#define Kd_Pitch  0.0001//14//19 //0.012

#define MAXTHROTTLE 70//75    //maximum angle at which the motors can spin
#define MINTHROTTLE 20        //minimum angles for the motors

//float pitch_calib = 0.00001;  //comment this variable when not using semi-manual auto-tuning\
//float roll_calib = 0.0001;  //comment this variable when not using semi-manual auto-tuning

//max and min voltages for reading the RC input
#define MINVOLT 1100
#define MAXVOLT 1900

//#define RollLPFGain 0.2
//#define PitchLPFGain 0.2

//three channels assigned for RC input
int RC1 = A6;
int RC2 = A7;
int RC3 = A8;



float throttle;

IMU imu(IMUPIN);


Stabilize rollStabilize (Kp_Roll, Ki_Roll, Kd_Roll);
Stabilize pitchStabilize (Kp_Pitch, Ki_Pitch, Kd_Pitch);

BlockingRC rc;

double  prevRollGyro, prevPitchGyro, rollGyro,  
  rollAcc,  pitchGyro,  pitchAcc,_roll,_pitch, pitchError, rollError; 

int globalLoopCounter;
long startTime, endTime;
long throttleStart, throttleEnd;

float tempThrottle;
int throttleCutOff = 1750;

long int delayN = 500;

//long int debugStart, debugEnd;

float motor1Correction, motor2Correction, motor3Correction, motor4Correction;  
float motor1Offset = 0, motor2Offset = 0, motor3Offset = 0, motor4Offset = 0;

float prevRoll, prevPitch;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


double rCorrection1, rCorrection2, pCorrection1, pCorrection2;

//int wildCounter = 0;

void setup()
{
//  Serial.begin(19200);
//  Serial.flush();
  imu.initialize();  //initialize the IMU library
  
  //set the initial values for roll and pitch
  prevRollGyro = rollGyro = imu.gyroDeg(0);  
  prevPitchGyro = pitchGyro = imu.gyroDeg(1);
  rollAcc = imu.acceDeg(0);
  pitchAcc = imu.acceDeg(1);
  
  attachMotors();
  
  RC1 = rc.setPin(RC1);    //sets the throttle for the motors. all motors are fed the same input (excluding the PID corrections)
  RC2 = rc.setPin(RC2);    //triggers and halts the motors.
  RC2 = rc.setPin(RC3);    //does nothing
  
  tempThrottle = 0;
  
  globalLoopCounter = 0;
  throttleStart = endTime = micros();
  
  throttle = MINTHROTTLE;    //set initial throttle to minimum value
  
  prevRoll = prevPitch = 0;

}

void loop()
{
  
  while(true)
  {
 
  startTime = micros();
  rollPitchPID();      //PID for roll and pitch only
  endTime = micros();
  
  rollCorrection(rCorrection1, rCorrection2);  //feed the roll correction values to the motors
  pitchCorrection(pCorrection1, pCorrection2); ////feed the pitch correction values to the motors
  
  globalLoopCounter++;
  
 // wildCounter++;  //delete this shit in final build

  
  if(rc.getValueWithDelay(RC2,delayN)>throttleCutOff)    //check channel RC2 for triggering criterion
  {
    
    if ((endTime - throttleStart)>delayN)
    {
      float rcThrottleInput = rc.getValue(RC1);          //check the value for throttle coming in through RC1
      
      delayN = 3000000;                                  //increase the delay, since we don't want to check RC input on every loop.
      //We will check for RC input after every delayN microsecs
      
      /*Auto tuning block : BEGIN*/
      /*Use this only when you're fed up with current PID settings*/
      
//      pitchStabilize.updateGain(Kp_Pitch,Ki_Pitch,(pitch_calib+=0.00001));
//      Serial.println(pitch_calib,6);

      /*Auto tuning block : END*/
      
      throttle = (MINTHROTTLE + (((rcThrottleInput - (float)MINVOLT)/((float)MAXVOLT - (float)MINVOLT))*((float)MAXTHROTTLE - (float)MINTHROTTLE)));  //scale RC input voltage to throttle value
      
      throttleStart = micros();
      
//      Serial.println(wildCounter);
//      wildCounter= 0;
    }
    
    //tempThrottle<throttle?motorSpin(tempThrottle+=1):motorSpin(throttle);    //throttle value increments slowly to attain saturation at max value
    motorSpin(throttle);                                                       //le wild hard coding of throttle value, smoothness be damned.
  }
  else
  {
    allMotorsDeadStop();    //stop all motors
  }
  
  }
  
}


/*Roll & Pitch update logic*/

void rollPitchPID()      //compute the PID values for the roll & pitch only
{
  updateRoll();  //set global variables with updated values for roll
  updatePitch();  //set global variables with updated values for pitch

  double rollErrorNow = ROLL_ERROR(_roll);
  double pitchErrorNow = PITCH_ERROR(_pitch);
  
  long int duration;
  rollStabilize.errorCorrection(rollErrorNow, rollError, duration = (startTime - endTime));
  pitchStabilize.errorCorrection(pitchErrorNow, pitchError, duration);


  if(rollErrorNow>0)
  {
    rCorrection1 = rollStabilize.compute(rollError = rollErrorNow);
    rCorrection2 = -rCorrection1;

  }
  else if (rollErrorNow<0)
  {
    rCorrection1 = -rollStabilize.compute(rollError = rollErrorNow);
    rCorrection2 = -rCorrection1;
  }
  
  if(pitchErrorNow>0)
  {
  pCorrection1 = pitchStabilize.compute(pitchError = pitchErrorNow);
  pCorrection2 = -pCorrection1;
  }
  else if(pitchErrorNow<0)
  {
  pCorrection1 = -pitchStabilize.compute(pitchError = pitchErrorNow);
  pCorrection2 = -pCorrection1;
  }
  
   
}
void updateRoll()
{
  /*Complementary Filter implemented here*/
  prevRollGyro = rollGyro;
  
  rollGyro = imu.gyroDeg(0);
  rollAcc = imu.acceDeg(0);
  
  _roll = COMPLEMENTARY(_roll, rollGyro, prevRollGyro, rollAcc);

}

void updatePitch()
{
  /*Complementary Filter implemented here*/
  prevPitchGyro = pitchGyro;
  
  pitchGyro = imu.gyroDeg(1);
  pitchAcc = imu.acceDeg(1);
  
  _pitch = COMPLEMENTARY(_pitch, pitchGyro, prevPitchGyro, pitchAcc); //complementary filter

}


/*Flight Parameters and Control*/

void attachMotors()
{
motor1.attach(7);
motor2.attach(8);
motor3.attach(11);
motor4.attach(12);
}

void allMotorsDeadStop()
{
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);
  tempThrottle = 0;
  delayN = 500;
  //delay(1000);
}

void motorSpin (int value)
{
  motor1.write(value + motor1Correction + motor1Offset);
  motor2.write(value + motor2Correction + motor2Offset);
  motor3.write(value + motor3Correction + motor3Offset);
  motor4.write(value + motor4Correction + motor4Offset);
 

}

void rollCorrection (float correction1, float correction2)
{
  motor3Correction = correction2;
  motor4Correction = correction1;
}

void pitchCorrection (float correction1, float correction2)
{
  motor1Correction = correction1;
  motor2Correction = correction2;
}






