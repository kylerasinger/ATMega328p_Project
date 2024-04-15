#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <util/twi.h> //status codes of TWI for AVR
#include <stdlib.h>
#include <Servo.h>

//personally made classes and helper functions
#include "UART.h"
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
UART uart;
Servo servo;
Fan liftFan(L_FAN_PIN);
Fan thrustFan(T_FAN_PIN);

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

///
/// Helper function to make the turn of the hoverCraft
///
double turningPoint(MPU6050* iMpu, double iYaw)
{
  double wYaw = iYaw;
  double turningYawChange = 0.0;

  double wYawLeft = iYaw - 90;
  if(wYawLeft <= 0){wYawLeft = 0;}

  double wYawRight = iYaw + 90;
  if(wYawRight >= 180){wYawRight = 180;}

  // turn off fans
  thrustFan.setSpeed(0);
  liftFan.setSpeed(0);
  _delay_ms(100);

  //Check left
  servo.write(wYawLeft);
  _delay_ms(500);
  double leftDistance = getUSdistance();
  _delay_ms(500);

  //Check right
  servo.write(wYawRight);
  _delay_ms(500);
  double rightDistance = getUSdistance();
  _delay_ms(500);

  ///
  /// Right Turn
  ///
  if(leftDistance < rightDistance) 
  {

    while(true){ 

      double servoAngleAtTurns = 270 - wYaw;
      if(servoAngleAtTurns >= 180.0) {servoAngleAtTurns = 180.0;}
      else if(wYaw > 180 ) {servoAngleAtTurns = 360 - wYaw;}

      liftFan.setSpeed(255); // Lift the hovercraft
      thrustFan.setSpeed(160); // Make the turn

      timerTwo.start();
      iMpu->readSensor();

      //Serial.println(wYaw);
      _delay_ms(50);
      servo.write(servoAngleAtTurns + turningYawChange*1.3);
      _delay_ms(50);

      timerTwo.read();
      timerTwo.stop();

      turningYawChange = iMpu->getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      wYaw -= turningYawChange*1.3;

      if(260 < wYaw)
      {
        _delay_ms(1000);
        // turn off fans
        thrustFan.setSpeed(0);
        liftFan.setSpeed(0);
        break;
      }
    }

    return wYaw;
  }

  ///
  /// Left Turn
  ///
  else if(leftDistance > rightDistance)
  {


    // loop for turning
    while(true){ 

      double servoAngleAtTurns = 250 - wYaw;
      if(servoAngleAtTurns < 0) {servoAngleAtTurns = 0;}

      liftFan.setSpeed(255); // Lift the hovercraft
      thrustFan.setSpeed(200); // Make the turn

      timerTwo.start();
      iMpu->readSensor();


      _delay_ms(50);
      servo.write(servoAngleAtTurns + turningYawChange*1.3);
      _delay_ms(50);

      timerTwo.read();
      timerTwo.stop();

      turningYawChange = iMpu->getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      wYaw -= turningYawChange*1.3;

      if(110 < wYaw && wYaw < 130)
      {
        // turn off fans
        thrustFan.setSpeed(0);
        liftFan.setSpeed(0);
        break;
      }
    }
    return wYaw; 
  }
  return wYaw; 
}
///
/// Searching for the whole at the end
///
double searchingForDream(MPU6050* iMpu, double iYaw)
{
  double wYaw = iYaw;
  double turningYawChange = 0.0;

  double wYawLeft = iYaw - 90;
  if(wYawLeft <= 0){wYawLeft = 0;}

  double wYawRight = iYaw + 90;
  if(wYawRight >= 180){wYawRight = 180;}

  // turn off fans
  thrustFan.setSpeed(0);
  liftFan.setSpeed(0);
  _delay_ms(100);

  //Check right
  servo.write(wYawRight);
  _delay_ms(500);
  double rightDistance = getUSdistance();
  _delay_ms(500);

  if (rightDistance > 30)
  {
    double servoAngleAtTurns = 250 - wYaw;
    if(servoAngleAtTurns >= 180.0) {servoAngleAtTurns = 180.0;}

    while(true){ 

      liftFan.setSpeed(255); // Lift the hovercraft
      thrustFan.setSpeed(180); // Make the turn

      timerTwo.start();
      iMpu->readSensor();

      //Serial.println(wYaw);
      _delay_ms(50);
      servo.write(servoAngleAtTurns + turningYawChange*1.3);
      _delay_ms(50);

      timerTwo.read();
      timerTwo.stop();

      turningYawChange = iMpu->getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      wYaw -= turningYawChange*1.3;

      if(230 < wYaw && wYaw < 260)
      {
        // turn off fans
        thrustFan.setSpeed(0);
        liftFan.setSpeed(0);
        _delay_ms(500);
        break;
      }
    }
  }
  else
  {
    servo.write(wYaw);
    _delay_ms(500);

    liftFan.setSpeed(255); // Lift the hovercraft
    thrustFan.setSpeed(100); // Make the turn
    _delay_ms(1000);
  }

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
  _delay_ms(50);

  sei();  //Activate interrupts

  // Yaw initialization
  float yaw = 90.0; 
  float yawChange;

  // Void loop
  while (true) {

    double distance = getUSdistance();
    int counter = 0;

    // Threshold distance for Checking
    if(distance < 80 && counter == 0) // distance for right turn
    {

      yaw = turningPoint( &mpu , yaw ); // make the turn
      counter++;
    }
    else if (distance > 0 && distance < 5 && counter == 1) // distance before left turn
    {
      yaw = turningPoint( &mpu , yaw ); // make the turn
      counter++;
    }
    else if(counter > 1)
    {
      yaw = searchingForDream( &mpu , yaw ); //looking for the whole
    }
    else // Normal state of hoverCraft i.e. not at an intersection
    {
      
      // Activate lift fan
      liftFan.setSpeed(255);
      
      //Activate thrust fan
      thrustFan.setSpeed(240);

      // Start IMU timer      
      timerTwo.start();
      mpu.readSensor();

      if((int)yaw > 0 && (int)yaw < 180)
      {
        double adjServoYaw = (double) yaw;
        double standard180 = 180.0;
        servo.write(standard180 - adjServoYaw);
        _delay_ms(50);
      }
      else if((int)yaw > 180 && (int)yaw < 360)
      {
        double adjServoYaw = (double) yaw - 290;
        double standard90 = 90.0;
        servo.write(standard90 - adjServoYaw);
        _delay_ms(50);
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
