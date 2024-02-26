#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <util/twi.h> //status codes of TWI for AVR

//personally made classes and helper functions
#include "Communication/UART.h"
#include "Sensors/UsSensor.h"
#include "Sensors/IrSensor.h"
#include "Utilities/Utilities.h"
#include "Communication/LED.h"
#include "Utilities/I2C.h"


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


int main(void) {
  UART uart;
  UsSensor usSensor(US_ECHO, US_TRIG, &uart);
  IrSensor irSensor(IR_IN, &uart);
  LED largeLED(LED_PIN);
  setUp8BitTimer();

  largeLED.setBrightness(0);

  //MPU set up
    I2CInit(400000);
    
    // bool i2cConnection = detectMPU(&largeLED);
    // _delay_us(10);
    // char buffer[48];
    // sprintf(buffer, "MPU Connection status: %s", i2cConnection ? "Success" : "Failed");
    // uart.println(buffer);

    //test if write works
    uint8_t data = 0x01;
    //power management
    /*
    * range of gyroscope
    * 0b11100000 for 250 deg/s
    * 0b11101000 for 500 deg/s
    * 0b11110000 for 1000 deg/s
    * 0b11111000 for 2000 deg/ s
    */
   
    I2CWriteToReg(MPU6050_ADDRESS, 0x6B, 0x00, &largeLED); //clear the sleep bit of the MPU
    data = I2CReadFromReg(MPU6050_ADDRESS, 0x6B, &largeLED);
    
    char buffer2[32];
    sprintf(buffer2, "Data read: %02X", data);
    uart.println(buffer2);

  while(true){
  }
}
