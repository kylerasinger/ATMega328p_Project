#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <util/twi.h> //status codes of TWI for AVR

//personally made classes and helper functions
#include "Communication/UART.h"
#include "Communication/LED.h"
#include "Sensors/UsSensor.h"
#include "Sensors/IrSensor.h"
#include "Sensors/MPU6050.h"
#include "Utilities/Utilities.h"
#include "Utilities/I2C.h"
#include <stdlib.h>


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

/* ---===   END OF PIN DECLARATION   ===--- */

bool ReadSensor(LED* led, UART* uart) {
  uint8_t accel[6];
  uint8_t gyro[6];

  // Read accelerometer data
  for (uint8_t i = 0; i < 6; ++i) {
    accel[i] = I2CReadFromReg(0x68, 0x3B + i, led);
    // Check for error in reading (use your error handling method here)
    // if (accel[i] == 0xFF) { // Assuming 0xFF indicates an error
    //   led->setBrightness(255);
    // }
  }

  // Read gyroscope data
  for (uint8_t i = 0; i < 6; ++i) {
    gyro[i] = I2CReadFromReg(0x68, 0x43 + i, led);
    // Check for error in reading (use your error handling method here)
    // if (gyro[i] == 0xFF) { // Assuming 0xFF indicates an error
    //   led->setBrightness(255);
    // }
  }

  // Combine high and low bytes to form the full values
  int16_t AcX = (int16_t)(accel[0] << 8 | accel[1]);
  int16_t AcY = (int16_t)(accel[2] << 8 | accel[3]);
  int16_t AcZ = (int16_t)(accel[4] << 8 | accel[5]);
  int16_t GyX = (int16_t)(gyro[0] << 8 | gyro[1]);
  int16_t GyY = (int16_t)(gyro[2] << 8 | gyro[3]);
  int16_t GyZ = (int16_t)(gyro[4] << 8 | gyro[5]);

  float gAcX = (float)AcX / 16384.0f;
  float gAcY = (float)AcY / 16384.0f;
  float gAcZ = (float)AcZ / 16384.0f;

  int intPart;
  int fracPart;

  //print
  if(true){
    char bufferAcX[12];
    intPart = (int)gAcX; // Get the integer part
    fracPart = abs((int)((gAcX - intPart) * 100)); // Get the fractional part as an integer
    snprintf(bufferAcX, sizeof(bufferAcX), "AcX: %d.%02d", intPart, fracPart);
    uart->println(bufferAcX);

    char bufferAcY[12];
    intPart = (int)gAcY; // Get the integer part
    fracPart = abs((int)((gAcY - intPart) * 100)); // Get the fractional part as an integer
    snprintf(bufferAcY, sizeof(bufferAcY), "AcY: %d.%02d", intPart, fracPart);
    uart->println(bufferAcY);

    char bufferAcZ[12];
    intPart = (int)gAcZ; // Get the integer part
    fracPart = abs((int)((gAcZ - intPart) * 100)); // Get the fractional part as an integer
    snprintf(bufferAcZ, sizeof(bufferAcZ), "AcZ: %d.%02d", intPart, fracPart);
    uart->println(bufferAcZ);

    char bufferGyX[12];
    sprintf(bufferGyX, "GyX: %d", GyX);
    uart->println(bufferGyX);

    char bufferGyY[12];
    sprintf(bufferGyY, "GyY: %d", GyY);
    uart->println(bufferGyY);

    char bufferGyZ[12];
    sprintf(bufferGyZ, "GyZ: %d", GyZ);
    uart->println(bufferGyZ);
  }

  return true;
}


int main(void) {
  setUp8BitTimer();

  UART uart;
  UsSensor usSensor(US_ECHO, US_TRIG, &uart);
  IrSensor irSensor(IR_IN, &uart);
  LED largeLED(LED_PIN);
  MPU6050 mpu(MPU6050_ADDRESS, 0b00000000, 0b00000000, &largeLED);
  // mpu.calibrateGyro(100);

  largeLED.setBrightness(0);


  while(true){
    _delay_ms(2000);
    mpu.readSensor(&largeLED, &uart);
  }
}
