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
#include "Timer/TimerTwo.h"
#include "Timer/TimerZero.h"



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

/* ---===   RESOURCES   ===--- /

  Timer 0 is for general timing (1 second outputs)
  Timer 1 is being used by servo for PWM
  Timer 2 is being used for IMU motion tracking, the US Sensor and the LED's brightness.

/  ---===               ===--- */

int main(void) {
  TimerZero timerZero;
  TimerTwo timerTwo;
  UART uart;
  UsSensor usSensor(US_ECHO, US_TRIG, &uart);
  IrSensor irSensor(IR_IN, &uart);
  LED largeLED(LED_PIN);
  MPU6050 mpu(MPU6050_ADDRESS, 0b00011000, 0b00000000, &largeLED);
  Servo servo(SERVO_PIN);

  sei(); //enable interrupts

  float yaw = 90.0;
  float yawChange;

  float velocityX = 0.0;
  float velocityXChange;

  float distanceX = 0.0;
  float distanceXChange;

  timerZero.start();

  /*
    Two major questions for the POD session
      1. How can we convert the accel to distance traveled
      2. How can we run all this which measures yaw without having different readings depending on the time spent in other code.
  
  */

  while(true){
    timerTwo.start();

    mpu.readSensor(&largeLED);

    //This is all printing for debugging
      // unsigned long intPart = static_cast<unsigned long>(yaw);
      // unsigned long fracPart = static_cast<unsigned long>(fabs(yaw - intPart) * 100000);
      char buffer[25];
      // sprintf(buffer, "Yaw (degrees): %lu.%05lu", intPart, fracPart);
      // uart.println(buffer);

      // unsigned long intPart2 = static_cast<unsigned long>(distanceX);
      // unsigned long fracPart2 = static_cast<unsigned long>(fabs(distanceX - intPart2) * 100000);
      // sprintf(buffer, "DistanceX (m): %lu.%05lu", intPart2, fracPart2);
      // uart.println(buffer);
    // end of printing for debugging

    if((int)yaw > 5 && (int)yaw < 175){
      servo.setServoAngle((int)yaw);
      turnOffYellowLED();
    }else{
      turnOnYellowLED();
    }

    int intAccelX = (int)((mpu.getAccelX_g()) * 100);
    int absIntAccelX = abs(intAccelX);

    if(absIntAccelX < 8 ){
      largeLED.setBrightness(0);
    }else if(absIntAccelX > 108){
      largeLED.setBrightness(255);
    }else if(absIntAccelX > 8 && absIntAccelX < 108){
      int mappedBrightness = map(absIntAccelX, 8, 108, 0, 255);
      largeLED.setBrightness(mappedBrightness);
    }

    timerTwo.read();
    timerTwo.stop();

    yawChange = mpu.getGyroZ_degPerSec() * timerTwo.timeInSeconds;
    yaw -= yawChange*1.3;
    
    float accelX_ms2 = mpu.getAccelX_g() * 9.81;
    velocityX += accelX_ms2 * timerTwo.timeInSeconds;
    distanceXChange = velocityX * timerTwo.timeInSeconds;
    distanceX += distanceXChange;

    timerZero.read();
    if(timerZero.timeInSeconds > 1.0){
      mpu.printSensorReadings(&largeLED, &uart);
      timerZero.stop();
      timerZero.start();
    }
          // mpu.printSensorReadings(&largeLED, &uart);

  }
}

