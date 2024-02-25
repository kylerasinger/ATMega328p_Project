#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>

#include <util/twi.h>

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



void I2CInit(uint32_t i2cBaudRate) {
  TWSR = 0b00000000; // Set prescaler to 1
  TWBR = ((F_CPU / i2cBaudRate) - 16) / 2; // Set SCL frequency
  TWCR |= (1 << TWEN); // Enable I2C
}

void I2CStart(){
  //start condition
  TWCR = (1 << TWINT)|(1 << TWSTA)|(1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void I2CStop(){
  //stop condition
  TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

void I2CWrite(uint8_t data) {
  TWDR = data; // Load data into TWDR register
  TWCR = (1 << TWINT) | (1 << TWEN); // Begin transmission
  while (!(TWCR & (1 << TWINT))); // Wait for transmission to complete
}

bool I2CWriteToReg(uint8_t deviceAddr, uint8_t regAddr, uint8_t data){
  I2CStart();
  
  //add write bit (0)
  I2CWrite(deviceAddr<<1);
  I2CWrite(regAddr);
  I2CWrite(data);

  I2CStop();
  return true;
}

uint8_t I2CReadFromReg(uint8_t deviceAddr, uint8_t regAddr){
  uint8_t data = 0x02;
  I2CStart();
  I2CWrite(deviceAddr<<1); //write mode
  I2CWrite(regAddr);

  //repeated start because we want to read from the register
  I2CStart();
  I2CWrite(deviceAddr << 1 | 0x01); //read mode


  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT))); // Wait for transmission to complete

  data = TWDR;

  I2CStop();
  return data;
}

uint8_t I2CReadFromReg(uint8_t deviceAddr, uint8_t regAddr, LED* led) {
  //if debug light is on, this will return the status of the TWSR register
  uint8_t data = 0x02;
  uint8_t status;
  uint8_t START_CONDITION_ACK = 0x08;  
  uint8_t REPEATED_START_ACK = 0x10; 
  uint8_t SLA_W_ACK = 0x18;
  uint8_t MT_DATA_ACK = 0x28;
  uint8_t SLA_R_ACK = 0x40;
  uint8_t MR_DATA_ACK = 0x50; 
  uint8_t NACK = 0x58;

  _delay_us(10);
  I2CStart();
  status = TWSR & 0xF8; // Mask out prescaler bits
  if (status != START_CONDITION_ACK) { //successful start condition
    led->setBrightness(255); 
    return status;
  }

  _delay_us(10);
  I2CWrite(deviceAddr << 1); //write mode
  status = TWSR & 0xF8;
  if (status != SLA_W_ACK) { //successful SLA+W
    led->setBrightness(255);
    return status;
  }

  _delay_us(10);
  I2CWrite(regAddr);
  status = TWSR & 0xF8;
  if (status != MT_DATA_ACK) { //successful data transmission
    led->setBrightness(255);
    return status;
  }

  //repeated start because we want to read from the register
  _delay_us(10);
  I2CStart();
  status = TWSR & 0xF8;
  if (status != REPEATED_START_ACK) { //successful repeated start
    led->setBrightness(255);
    return status;
  }

  _delay_us(10);
  I2CWrite(deviceAddr << 1 | 0x01); // Read mode
  status = TWSR & 0xF8;
  if (status != SLA_R_ACK) { //successful SLA+R
    led->setBrightness(0);
    return status;
  }

  //!!! PROBLEM HERE !!!
  _delay_us(10);
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT))); // Wait for transmission to complete
  status = TWSR & 0xF8;
  if (status != MR_DATA_ACK && status != NACK) { //successful data receive
    led->setBrightness(2);
    return status;
  }

  data = TWDR;

  I2CStop();
  return data;
}


bool detectMPU(LED* led){
  uint8_t START = 0x08; //success code for i2c start
  uint8_t MT_SLA_ACK = 0x18; //success code for i2c connection
  uint8_t status;

  //detect if MPU is set up
  I2CStart();
  status = TWSR & 0xF8;
  if (status != START) { // Replace START_CONDITION_ACK with the actual status code for a successful start condition
    led->setBrightness(255);
  }

  // Check if the start condition was successfully transmitted
  if ((TWSR & 0xF8) != START) return false;
  //load slave address into register, start transmission of address
  //shift address and append write bit (0)
  TWDR = (MPU6050_ADDRESS << 1) | 0; 
  TWCR = (1 << TWINT)|(1 << TWEN);
  //busy wait for interrupt, meaning address is transmitted and we receive data
  while (!(TWCR & (1<<TWINT)));
  //check for success of i2c connection
  if ((TWSR & 0xF8) != MT_SLA_ACK) return false;
  
  I2CStop();
  return true;
}

// bool MPUStart(){

// }

int main(void) {
  UART uart;
  UsSensor usSensor(US_ECHO, US_TRIG, &uart);
  IrSensor irSensor(IR_IN, &uart);
  LED largeLED(LED_PIN);
  setUp8BitTimer();

  largeLED.setBrightness(0);

  //MPU set up
    I2CInit(400000);
    _delay_ms(100);
    // bool i2cConnection = detectMPU(&largeLED);
    // _delay_ms(100);
    // char buffer[48];
    // sprintf(buffer, "MPU Connection status: %s", i2cConnection ? "Success" : "Failed");
    // uart.println(buffer);

    //test if write works
    uint8_t data = 0x01;
    //power management
    I2CWriteToReg(MPU6050_ADDRESS, 0x6B, 0xF0);
    data = I2CReadFromReg(MPU6050_ADDRESS, 0x6B, &largeLED);
    
    char buffer2[32];
    sprintf(buffer2, "Data read: %02X", data);
    uart.println(buffer2);

    // MPUStart();

  while(true){
  }
}
