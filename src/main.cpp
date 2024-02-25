#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>

//personally made classes and helper functions
#include "Communication/UART.h"
#include "Sensors/UsSensor.h"
#include "Sensors/IrSensor.h"
#include "Utilities/Utilities.h"
#include "Communication/LED.h"


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

void I2C_init(uint32_t i2cBaudRate) {
  TWSR = 0b00000000; // Set prescaler to 1
  TWBR = ((F_CPU / i2cBaudRate) - 16) / 2; // Set SCL frequency
  TWCR |= (1 << TWEN); // Enable I2C
}

bool detectMPU(){
  uint8_t START = 0x08; //success code for i2c start
  uint8_t MT_SLA_ACK = 0x18; //success code for i2c connection

  //detect if MPU is set up
  //start condition
  TWCR = (1 << TWINT)|(1 << TWSTA)|(1 << TWEN);
  // busy wait for interrupt back from MPU (its done)
  while (!(TWCR & (1 << TWINT)));
  // Check if the start condition was successfully transmitted
  if ((TWSR & 0xF8) != START) return false;
  //load slave address into register, start transmission of address
  TWDR = (MPU6050_ADDRESS << 1) | 0; //shift address and append write bit (0)
  TWCR = (1 << TWINT)|(1 << TWEN);
  //busy wait for interrupt, meaning address is transmitted and we receive data
  while (!(TWCR & (1<<TWINT)));

  //check for success of i2c connection
  if ((TWSR & 0xF8) != MT_SLA_ACK) return false;
  //stop condition
  TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
  return true;
}

int main(void) {
  UART uart;
  UsSensor usSensor(US_ECHO, US_TRIG, &uart);
  IrSensor irSensor(IR_IN, &uart);
  LED largeLED(LED_PIN);
  setUp8BitTimer();

  //MPU set up
  I2C_init(400000);
  bool i2cConnection = detectMPU();

  char buffer[48];
  sprintf(buffer, "MPU Connection status: %s", i2cConnection ? "Success" : "Failed");

  uart.println(buffer);

  while(true){
    
  }
}
