#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <util/twi.h> //status codes of TWI for AVR
#include <stdlib.h>

//personally made classes and helper functions
#include "Communication/UART.h"
#include "Communication/LED.h"
#include "Sensors/UsSensor.h"
#include "Sensors/IrSensor.h"
#include "Sensors/MPU6050.h"
#include "Servo/Servo.h"
#include "Utilities/Utilities.h"
#include "Utilities/I2C.h"
#include "Timer/8BitTimer.h"



/* ---===       PIN DECLARATION      ===--- */

  //define pins for the US sensor
  #define US_TRIG PB4
  #define US_ECHO PB0

  //define pins for the IR
  #define IR_IN PC0

  //define LED pin
  #define LED_PIN PB3
  #define Y_LED_PIN PB5

  //define IMU 
  #define I2C_DATA PC4
  #define I2C_CLOCK PC5
  #define MPU6050_ADDRESS 0x68

  //define Servo pins
  #define SERVO_PIN PB1

/* ---===   END OF PIN DECLARATION   ===--- */



int main(void) {
  EightBitTimer smallTimer;
  UART uart;
  UsSensor usSensor(US_ECHO, US_TRIG, &uart);
  IrSensor irSensor(IR_IN, &uart);
  LED largeLED(LED_PIN);
  MPU6050 mpu(MPU6050_ADDRESS, 0b00011000, 0b00000000, &largeLED);
  Servo servo(SERVO_PIN);

  sei(); //enable interrupts

  float yaw = 90.0;
  float yawChange;

  while(true){
    smallTimer.start();

    mpu.readSensor(&largeLED);

    unsigned long intPart = static_cast<unsigned long>(yaw);
    unsigned long fracPart = static_cast<unsigned long>(fabs(yaw - intPart) * 100000);
    char buffer[25];
    sprintf(buffer, "Yaw (degrees): %lu.%05lu", intPart, fracPart);
    uart.println(buffer);

    if((int)yaw > 5 && (int)yaw < 175){
      servo.setServoAngle((int)yaw);
      turnOffYellowLED();
    }else{
      turnOnYellowLED();
    }
    smallTimer.read();
    smallTimer.stop();

    yawChange = mpu.getGyroZ_degPerSec() * smallTimer.timeInSeconds;
    yaw -= yawChange;

   // mpu.printSensorReadings(&largeLED, &uart);

  }
}

