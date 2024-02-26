#include "I2C.h"

void I2CInit(long i2cBaudRate) {
  TWSR = 0b00000000; // Set prescaler to 1
  TWBR = ((F_CPU / i2cBaudRate) - 16) / 2; // Set SCL frequency
  TWCR |= (1 << TWEN); // Enable I2C
  _delay_us(10);
}

void I2CStart(){
  //start condition
  _delay_us(10);
  TWCR = (1 << TWINT)|(1 << TWSTA)|(1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  
}

void I2CStop(){
  //stop condition
  TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

bool I2CWrite(uint8_t data) {
  TWDR = data; // Load data into TWDR register
  TWCR = (1 << TWINT) | (1 << TWEN); // Begin transmission
  while (!(TWCR & (1 << TWINT))); // Wait for transmission to complete

  uint8_t twst = TW_STATUS & 0xF8;
	if ((twst != TW_MT_SLA_ACK) && (twst != TW_MT_DATA_ACK)) {
    return false;
  }

  _delay_us(10);
  return true;
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
    led->setBrightness(255);
    return status;
  }

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

bool detectMPU(LED* led){ //THIS IS PURELY FOR DEBUGGING
  uint8_t START = 0x08; //success code for i2c start
  uint8_t MT_SLA_ACK = 0x18; //success code for i2c connection
  uint8_t status;
  uint8_t MPU6050_ADDRESS = 0x68;

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

bool I2CWriteToReg(uint8_t deviceAddr, uint8_t regAddr, uint8_t data, LED* led){
  I2CStart();
  
  //add write bit (0)
  if(!I2CWrite(deviceAddr<<1)){
    led->setBrightness(255);
    return false;
  };

  if(!I2CWrite(regAddr)){
    led->setBrightness(255);
    return false;
  };

  if(!I2CWrite(data)){
    led->setBrightness(255);
    return false;
  };
  I2CStop();
  return true;
}