#ifndef I2C_H
#define I2C_H

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <Communication/LED.h>

void I2CInit(uint32_t i2cBaudRate);
void I2CStart();
void I2CStop();
bool I2CWrite(uint8_t data);
uint8_t I2CReadFromReg(uint8_t deviceAddr, uint8_t regAddr, LED* led);
bool I2CWriteToReg(uint8_t deviceAddr, uint8_t regAddr, uint8_t data, LED* led); 
bool detectMPU(LED* led);

#endif