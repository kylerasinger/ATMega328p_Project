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
  TimerZero timerZero;
  TimerTwo timerTwo;
  UART uart;
  Servo servo;
  Fan liftFan(L_FAN_PIN);
  Fan thrustFan(T_FAN_PIN);

int main() {

  // Initialize serial communication
  Serial.begin(9600);

  //UsSensor usSensor(US_ECHO, US_TRIG);
  pinMode(US_TRIG,OUTPUT);
  pinMode(US_ECHO,INPUT);

  MPU6050 mpu(MPU6050_ADDRESS, 0b00000000, 0b00000000);

  servo.attach(9);
  servo.write(0);


  sei(); 

  float yaw = 90.0;
  float yawChange;

  // Void loop
  while (true) {

    liftFan.setSpeed(255);
    thrustFan.setSpeed(255);

    // Send a pulse to the trigger pin to initiate the measurement
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);
  
    // Measure the duration of the echo pulse
    long duration = pulseIn(US_ECHO, HIGH);
    double distance = (duration * 0.034) / 2;
    
    // Print the distance reading to the serial monitor
    Serial.print("US Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    timerTwo.start();
    mpu.readSensor();

    //This is all printing for debugging
    unsigned long intPart = static_cast<unsigned long>(yaw);
    unsigned long fracPart = static_cast<unsigned long>(fabs(yaw - intPart) * 100000);

    char buffer[25];
    sprintf(buffer, "Yaw (degrees): %lu.%05lu", intPart, fracPart);
    uart.println(buffer);
    //end of printing for debugging

      if((int)yaw > 5 && (int)yaw < 200)
      {
        servo.write(180 - (int) yaw);
      }
  
      timerTwo.read();
      timerTwo.stop();
  
      yawChange = mpu.getGyroZ_degPerSec() * timerTwo.timeInSeconds;
      yaw -= yawChange*1.3;
  
      timerZero.read();
      if(timerZero.timeInSeconds > 1.0)
      {
        mpu.printSensorReadings(&uart);
        timerZero.stop();
        timerZero.start();
      }
  }
  
  return 0;
}
