/*Flight Stabilization mechanism
primary implementation of PID controller...
*/

/*Version - 0.0.4
Date - 28-MAR-2014, 05:47 PM

FlightStableNew.pde*/

#include <Servo.h>
#include <math.h>
#include <IMU.h>
#include <Stabilize.h>
#include <BlockingRC.h>
#include <LandMe.h>


//hard coded values for roll and pitch gains
#define Kp_Roll  0.13//0.0069
#define Ki_Roll  0.000002//0.000008
#define Kd_Roll  0.000007//0.015

#define Kp_Pitch  0.12//0.16 
#define Ki_Pitch  0.000002//20//1//5//9//1//1
#define Kd_Pitch  0.000004//14//19 //0.012

#define MAXTHROTTLE 70//75    //maximum angle at which the motors can spin
#define MINTHROTTLE 20        //minimum angles for the motors

//float pitch_calib = 0.000002;  //comment this variable when not using semi-manual auto-tuning\
//float roll_calib = 0.000002;  //comment this variable when not using semi-manual auto-tuning

//max and min voltages for reading the RC input
#define MINVOLT 1100
#define MAXVOLT 1900

//#define RollLPFGain 0.2
//#define PitchLPFGain 0.2


IMU imu(IMUPIN);

LandMe landingParams(QUADRIATIC_EXP_DECAY, MAXTHROTTLE, MINTHROTTLE);

Stabilize rollStabilize (Kp_Roll, Ki_Roll, Kd_Roll);
Stabilize pitchStabilize (Kp_Pitch, Ki_Pitch, Kd_Pitch);

BlockingRC rc;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

//three channels assigned for RC input
int RC1 = A6;
int RC2 = A7;
int RC3 = A8;
int globalLoopCounter;
int throttleCutOff = 1750;
int landingTrigger = 1200;

long landingTimeInMicros = 6000000;
long int delayN = 500;

long startTime, endTime;
long throttleStart, throttleEnd, landingStart;

float throttle;
float tempThrottle;
float motor1Correction, motor2Correction, motor3Correction, motor4Correction;  
float motor1Offset = 0, motor2Offset = 0, motor3Offset = 0, motor4Offset = 0;
float prevRoll, prevPitch;

double  prevRollGyro, prevPitchGyro, rollGyro,  
  rollAcc,  pitchGyro,  pitchAcc,_roll,_pitch, pitchError, rollError; 
double rCorrection1, rCorrection2, pCorrection1, pCorrection2;

boolean landingFlag, landingAccomplished;


void setup()
{
  Serial.begin(19200);
  Serial.flush();
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
  
  
  landingAccomplished = true;
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
  
  //free(correction);
  globalLoopCounter++;
  
 // wildCounter++;  //delete this shit in final build

  float rcVal = rc.getValueWithDelay(RC2,delayN);
  if(rcVal>throttleCutOff)    //check channel RC2 for triggering criterion
  {
    
    if (((endTime - throttleStart)>delayN)&&landingAccomplished)
    {
      landingFlag = false; 
      landingAccomplished = false;
      float rcThrottleInput = rc.getValue(RC1);          //check the value for throttle coming in through RC1
      
      delayN = 6000000;                                  //increase the delay, since we don't want to check RC input on every loop.
      //We will check for RC input after every delayN microsecs
      
      /*Auto tuning block : BEGIN*/
//      pitchStabilize.updateGain(Kp_Pitch,(pitch_calib+=0.000001),Kd_Pitch);
//      Serial.println(pitch_calib,6);
      /*Auto tuning block : END*/
      
      throttle = (MINTHROTTLE + (((rcThrottleInput - (float)MINVOLT)/((float)MAXVOLT - (float)MINVOLT))*((float)MAXTHROTTLE - (float)MINTHROTTLE)));  //scale RC input voltage to throttle value
      
      throttleStart = micros();
      
//      Serial.println(wildCounter);
//      wildCounter= 0;
    }
    
    //tempThrottle<throttle?motorSpin(tempThrottle+=1):motorSpin(throttle);
    motorSpin(throttle);
  }
  else if((rcVal<landingTrigger) && (!landingAccomplished))
  {
    if(!landingFlag)
    {
    landingFlag=!landingFlag; 
    landingStart = micros(); 
    landingAccomplished = false;
    }
    //(!landingFlag)?(landingFlag!=landingFlag,landingStart = micros(),landingAccomplished = false):true;
    float tempLandingVal = landingParams.landingValue(landingTimeInMicros,(micros() - landingStart));
    tempLandingVal<(MINTHROTTLE + 0.0001)?landingAccomplished = true:false;
    
    //Serial.println(tempLandingVal,6);
    //float avgSpeed = readMotorsAverage();
    motorSpin(tempLandingVal>throttle?throttle:tempLandingVal);
  }
  else
  {
    allMotorsDeadStop();    //stop all motors
  }
  
  }
  
}


/*Roll & Pitch update logic*/

void rollPitchPID()
{
  updateRoll();  //set global variables with updated values for roll
  updatePitch();  //set global variables with updated values for pitch

  double rollErrorNow = ROLL_ERROR(_roll);
  double pitchErrorNow = PITCH_ERROR(_pitch);
  
  long int duration;
  rollStabilize.errorCorrection(rollErrorNow, rollError, duration = (startTime - endTime));
  pitchStabilize.errorCorrection(pitchErrorNow, pitchError, duration);
  
//  Serial.print(rollErrorNow);
//  Serial.print(",");
//  Serial.print(rollStabilize.getDerivativeError());
//  Serial.print(",");
//  Serial.print(rollStabilize.getIntegralError());
//  Serial.println();

 // Correction *correction = (Correction *) malloc (sizeof(Correction));
  if(rollErrorNow>0)
  {
//  correction->rollCorrection1 = rollStabilize.compute(rollError = rollErrorNow);
//  correction->rollCorrection2 = 0;//-rollStabilize.compute(rollError);
    rCorrection1 = rollStabilize.compute(rollError = rollErrorNow);
    rCorrection2 = -rCorrection1;

  }
  else if (rollErrorNow<0)
  {
//  correction->rollCorrection1 = 0;//-rollStabilize.compute(rollError = rollErrorNow);
//  correction->rollCorrection2 = rollStabilize.compute(rollError = rollErrorNow);
    rCorrection1 = -rollStabilize.compute(rollError = rollErrorNow);
    rCorrection2 = -rCorrection1;
  }
  
  if(pitchErrorNow>0)
  {
//  correction->pitchCorrection1 = pitchStabilize.compute(pitchError = pitchErrorNow);
//  correction->pitchCorrection2 = 0;//-pitchStabilize.compute(pitchError);
  pCorrection1 = pitchStabilize.compute(pitchError = pitchErrorNow);
  pCorrection2 = -pCorrection1;
  }
  else if(pitchErrorNow<0)
  {
//  correction->pitchCorrection1 = 0;//-pitchStabilize.compute(pitchError = pitchErrorNow);
//  correction->pitchCorrection2 = pitchStabilize.compute(pitchError = pitchErrorNow);
  pCorrection1 = -pitchStabilize.compute(pitchError = pitchErrorNow);
  pCorrection2 = -pCorrection1;
  }
  
 // return correction;
   
}
void updateRoll()
{
  /*Complementary Filter implemented here*/
  prevRollGyro = rollGyro;
  
  rollGyro = imu.gyroDeg(0);
  rollAcc = imu.acceDeg(0);
  
  _roll = COMPLEMENTARY(_roll, rollGyro, prevRollGyro, rollAcc);
//  _roll = LPF(_roll, prevRoll, RollLPFGain);
//  prevRoll = _roll;
}

void updatePitch()
{
  /*Complementary Filter implemented here*/
  prevPitchGyro = pitchGyro;
  
  pitchGyro = imu.gyroDeg(1);
  pitchAcc = imu.acceDeg(1);
  
  _pitch = COMPLEMENTARY(_pitch, pitchGyro, prevPitchGyro, pitchAcc); //complementary filter
//  Serial.print(_pitch);
//  Serial.print(", ");
//  _pitch = LPF(_pitch, prevPitch, PitchLPFGain);
//  prevPitch = _pitch;
}


/*Flight Parameters and Control*/

void attachMotors()
{
//motor1.attach(7);
//motor2.attach(8);
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
 
//  Serial.print(_pitch);
//  Serial.print(" , "); 
//  Serial.print(value + motor3Correction);
//  Serial.print(" , ");
//  Serial.print(value + motor4Correction);
//  Serial.println();
}

float readMotorsAverage()
{
  return (motor1.read() + motor2.read() + motor3.read() + motor4.read())/4;
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






