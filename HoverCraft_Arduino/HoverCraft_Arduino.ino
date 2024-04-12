#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <util/twi.h> //status codes of TWI for AVR
#include <stdlib.h>
#include <Arduino.h>
#include <Servo.h>

//personally made classes and helper functions
//#include "UART.h"
#include "Fan.h"
#include "UsSensor.h"
#include "MPU6050.h"
#include "Utilities.h"
#include "I2C.h"
#include "TimerTwo.h"
#include "TimerZero.h"


/* ---===       PIN DECLARATION      ===--- */

  //define pins for the US sensor
  #define US_TRIG 12
  #define US_ECHO 8

  //define IMU
  #define I2C_DATA PC4
  #define I2C_CLOCK PC5
  #define MPU6050_ADDRESS 0x68

  //define Servo pins
  #define SERVO_PIN PB1
    //uses timer 1 for PWM

  //define lift fan pins
  #define L_FAN_PIN 5 //lift
  #define T_FAN_PIN 6 //thrust
    //uses timer 0 for PWM

/* ---===   END OF PIN DECLARATION   ===--- */

/* ---===   RESOURCES   ===--- /
  Timer 0 is for general timing (1 second outputs), Lift and thrust fan PWM
  Timer 1 is being used by servo for PWM
  Timer 2 is being used for IMU motion tracking, the US Sensor and the LED's brightness.
/  ---===               ===--- */

TimerTwo timerTwo;
//UART uart;
Servo servo;
Fan liftFan(L_FAN_PIN);
Fan thrustFan(T_FAN_PIN);


double TargetLeft = 0.0;
double TargetUp = 90.0;
double TargetRight = 180.0;
double TargetDown = 270.0;

///
/// Function to return distance read from US sensor
///
double getUSdistance()
{
  // Send a pulse to the trigger pin to initiate the measurement
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(US_ECHO, HIGH);
  double distance = (duration * 0.034) / 2;

  return distance;

}

double turningPoint(MPU6050* iMpu, double iYaw, int flag)
{
  // turn off fans
  thrustFan.setSpeed(0);
  liftFan.setSpeed(0);
  _delay_ms(300);

  double targetD = 0.0;
  if(flag == 0){ targetD = 180.0;} 
  if(flag == 1){ targetD = 270.0;}
  if(flag == 2){ targetD = 180.0;}
  if(flag == 3){ targetD = 90.0;}

  //double fanServo = 90.0 - ((180 - iYaw) - targetD);
  double fanServo = 270 - iYaw;
  if(fanServo >= 180) {fanServo = 180;}
  double rightFanServo = fanServo + 90.0;
  double leftFanServo = fanServo - 90.0;
  
  if(flag == 0 || flag == 1)
  {
    //scan left
    servo.write(leftFanServo);
    _delay_ms(3000);
  }
  else
  {
    //scan right
    servo.write(rightFanServo);
    _delay_ms(3000);

  }

  if (flag == 0 || flag == 1) //right turn
  {
    double fanServo = 270 - iYaw;
    if(fanServo >= 180) {fanServo = 180;}

   double turnDist = 0.0;

    liftFan.setSpeed(255);
    _delay_ms(50);
    thrustFan.setSpeed(140);
    _delay_ms(50);

   while(true){

    timerTwo.start();
    turnDist = getUSdistance();
    iMpu->readSensor();
    
    timerTwo.read();
    timerTwo.stop();

    double turningYawChange = iMpu->getGyroZ_degPerSec() * timerTwo.timeInSeconds;
    iYaw -= turningYawChange*1.3;
    servo.write(fanServo + turningYawChange*1.3);  //try +-
    _delay_ms(30);

    if(iYaw > 170 && flag == 0) {
      liftFan.setSpeed(0);
      _delay_ms(50);
      thrustFan.setSpeed(0);
      _delay_ms(50);
      break;
    }
    else if(iYaw > 260 && flag == 1)
    {
      liftFan.setSpeed(0);
      _delay_ms(50);
      thrustFan.setSpeed(0);
      _delay_ms(50);
      break; 
    }
   }

  }
  else if(flag == 2 || flag == 3)
  {
    double fanServo = 270 - iYaw;
    if(fanServo >= 180) {fanServo = 180;}

    //during all this we need to calibrate
   servo.write(fanServo); //fly
   
   liftFan.setSpeed(255);
   thrustFan.setSpeed(120);
   
   double turnDist = 0.0;
   while(true){
    timerTwo.start();
    turnDist = getUSdistance();
    iMpu->readSensor();
    
    timerTwo.read();
    timerTwo.stop();

    double turningYawChange = iMpu->getGyroZ_degPerSec() * timerTwo.timeInSeconds;
    iYaw -= turningYawChange*1.3;
    servo.write(fanServo + turningYawChange*1.3);
    _delay_ms(100);

    if(iYaw > 170 && flag == 2) { //to be changed
      liftFan.setSpeed(0);
      _delay_ms(50);
      thrustFan.setSpeed(0);
      _delay_ms(50);
      break;
    }
    else if(iYaw > 260 && flag == 3)
    {
      liftFan.setSpeed(0);
      _delay_ms(50);
      thrustFan.setSpeed(0);
      _delay_ms(50);
      break; 
    }
   }
  }
  else
  {
    //looking for the hole
  }
  return iYaw;
}

///
/// Entry point
///
int main() {

  // Initialize serial communication
  Serial.begin(9600);

  //UsSensor usSensor(US_ECHO, US_TRIG);
  pinMode(US_TRIG,OUTPUT);
  pinMode(US_ECHO,INPUT);

  MPU6050 mpu(MPU6050_ADDRESS, 0b00000000, 0b00000000);

  // Servo initalization
  servo.attach(9);
  servo.write(90);
  _delay_ms(500);

  sei();  //Activate interrupts

  // Yaw initialization
  float yaw = 90.0; 
  float yawChange;
  int flag = 0;
  // Void loop
  while (true) {

    // Serial.print("Yaw Value: ");   // Print a label
    // Serial.println(yaw); 

    double distance = getUSdistance();
 

    // Threshold distance for Checking
    if(distance < 50)
    {
      yaw = turningPoint( &mpu , yaw , flag ); // make the turn
      flag++;
    }
    else // Normal state of hoverCraft i.e. not at an intersection
    {
      
      // Activate lift fan
      liftFan.setSpeed(255);
      
      //Activate thrust fan
      thrustFan.setSpeed(200);

      // Start IMU timer      
      timerTwo.start();
      mpu.readSensor();

      //end of printing for debugging

      if((int)yaw > 0 && (int)yaw < 180)
      {
        double adjServoYaw = (double) yaw;
        double standard180 = 180.0;
        servo.write(standard180 - adjServoYaw*1.0);
        _delay_ms(100);
      }
      else if((int)yaw > 180 && (int)yaw < 360)
      {
        double adjServoYaw = (double) yaw - 180;
        double standard180 = 180.0;
        servo.write(standard180 - adjServoYaw*1.0);
        _delay_ms(100);
      }

      timerTwo.read();
      timerTwo.stop();

      //important, do not remove
      yawChange = mpu.getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      yaw -= yawChange*1.3; //correction factor, can change the scale for precision.

      if(yaw > 360 )
      {
        yaw -= 360;
      }
      else if(yaw < 0)
      {
        yaw += 360;
      }
    } // else statment i.e. normal state of HoverCraft
  } // void loop
  return 0;
} // int main
