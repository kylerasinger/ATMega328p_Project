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
  // turn off fans
  thrustFan.setSpeed(0);
  liftFan.setSpeed(0);

  double frontDistance = getUSdistance();
  _delay_ms(100);

  //Check left
  servo.write(0);
  _delay_ms(500);
  double leftDistance = getUSdistance();
  _delay_ms(500);

  //Check right
  servo.write(180);
  _delay_ms(500);
  double rightDistance = getUSdistance();
  _delay_ms(500);

  if(leftDistance < rightDistance && rightDistance > frontDistance)  //turn right 
  {
    servo.write(150); // we can modify this to go at an angle not strictly 180 degrees
    _delay_ms(500);
    //add loop here
    liftFan.setSpeed(255); // Lift the hovercraft
    _delay_ms(500); //track yaw here
    thrustFan.setSpeed(190); // Make the turn

    //3 second delay for the turn while tracking the yaw
    for(int i = 0; i< 300; i++){ //equivalent to _delay_ms(3000);
      timerTwo.start();
      iMpu->readSensor();
      _delay_ms(10);
      timerTwo.read();
      timerTwo.stop();

      double turningYawChange = iMpu->getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      iYaw -= turningYawChange*1.3;
    }
    
    servo.write(90); //we want this to set it back on the opposite x axis direction
    _delay_ms(500);
    
    //printing for debugging purposes
      // unsigned long intPart = static_cast<unsigned long>(iYaw);
      // unsigned long fracPart = static_cast<unsigned long>(fabs(iYaw - intPart) * 100000);

      // char buffer[25];
      // sprintf(buffer, "Yaw (degrees): %lu.%05lu", intPart, fracPart);
      // uart.println(buffer);
    //end of printing

    return iYaw;
  }
  else if(leftDistance > rightDistance && leftDistance > frontDistance) //turn left
  {
    servo.write(30); // we can modify this to go at an angle not strictly 0 degrees
    _delay_ms(500);
    //add loop here
    liftFan.setSpeed(255); // Lift the hovercraft
    _delay_ms(500);
    thrustFan.setSpeed(190); // Make the turn

    for(int i = 0; i< 300; i++){ //equivalent to _delay_ms(3000);
      timerTwo.start();
      iMpu->readSensor();
      _delay_ms(10);
      timerTwo.read();
      timerTwo.stop();

      double turningYawChange = iMpu->getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      iYaw -= turningYawChange*1.3;
    }

    servo.write(90); //sets servo to middle of opposite x axis direction
    _delay_ms(500);

    //printing for debugging purposes
      // unsigned long intPart = static_cast<unsigned long>(iYaw);
      // unsigned long fracPart = static_cast<unsigned long>(fabs(iYaw - intPart) * 100000);

      // char buffer[25];
      // sprintf(buffer, "Yaw (degrees): %lu.%05lu", intPart, fracPart);
      // uart.println(buffer);
    //end of printing

    return iYaw;
  }
  
  // // Check front
  // servo.write(90);
  // _delay_ms(500);
  // double frontDistance = getUSdistance();
  // _delay_ms(500);

  servo.write(90); // Recenter the servo if neither side is open and go forward

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
  _delay_ms(1000);
  servo.write(180);
  _delay_ms(1000);

  sei();  //Activate interrupts

  // Yaw initialization
  float yaw = 90.0; 
  float yawChange;

  // Void loop
  while (true) {

    double distance = getUSdistance();
    int counter = 0;

    // Threshold distance for Checking
    if(distance < 50)
    {
      yaw = turningPoint( &mpu , yaw ); // make the turn
      counter++;
    }
    else // Normal state of hoverCraft i.e. not at an intersection
    {
      
      // Activate lift fan
      liftFan.setSpeed(255);
      
      //Activate thrust fan
      thrustFan.setSpeed(220);

      // Start IMU timer      
      timerTwo.start();
      mpu.readSensor();

      // //This is all printing for debugging
        // unsigned long intPart = static_cast<unsigned long>(yaw);
        // unsigned long fracPart = static_cast<unsigned long>(fabs(yaw - intPart) * 100000);

        // char buffer[25];
        // sprintf(buffer, "Yaw (degrees): %lu.%05lu", intPart, fracPart);
        // uart.println(buffer);
      //end of printing for debugging

      if((int)yaw > 0 && (int)yaw < 180)
      {
        double adjYaw = (double) yaw;
        double standard180 = 180.0;
        servo.write(standard180 - adjYaw*1.0);
        _delay_ms(100);
      }

      timerTwo.read();
      timerTwo.stop();

      //important, do not remove
      yawChange = mpu.getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      yaw -= yawChange*1.3; //correction factor, can change the scale for precision.
    } // else statment i.e. normal state of HoverCraft
  } // void loop
  return 0;
} // int main
