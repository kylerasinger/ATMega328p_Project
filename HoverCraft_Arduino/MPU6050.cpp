#include "MPU6050.h"
#include <stdlib.h>
#include <stdio.h>



MPU6050::MPU6050(uint8_t mpuAddress, uint8_t gyro, uint8_t accel){
    long baudRate = 400000;
    //gyro sensitivity
    switch (gyro)
    {
    case 0b00000000:
        gyroSens = 131.0;
        break;
    case 0b00001000:
        gyroSens = 65.5;
        break;
    case 0b00010000:
        gyroSens = 32.8;
        break;
    case 0b00011000:
        gyroSens = 16.4;
        break;
    
    default:
        break;
    }

    switch (accel)
    {
    case 0b00000000:
        accelSens = 16384;
        break;
    case 0b00001000:
        accelSens = 8192;
        break;
    case 0b00010000:
        accelSens = 4096;
        break;
    case 0b00011000:
        accelSens = 2048;
        break;
    
    default:
        break;
    }

    I2CInit(baudRate);

    I2CWriteToReg(mpuAddress, 0x6B, 0b10000000);
    _delay_ms(100);

    //clear the sleep bit of the MPU 
    I2CWriteToReg(mpuAddress, 0x6B, 0x00);


    /* range of gyroscope, set in register 27 (0x1B)
    * 0bxxx00xxx for 250  deg/s, sens 131
    * 0bxxx01xxx for 500  deg/s, sens 65.5
    * 0bxxx10xxx for 1000 deg/s, sens 32.8
    * 0bxxx11xxx for 2000 deg/s, sens 16.4                  */
    I2CWriteToReg(mpuAddress, 0x1B, gyro);

    /* full scale range of accelerometer, set in register 28 (0x1C)
    * 0bxxx00xxx for +/- 2  deg/s
    * 0bxxx01xxx for +/- 4  deg/s
    * 0bxxx10xxx for +/- 8  deg/s
    * 0bxxx11xxx for +/- 16 deg/s                                */
    I2CWriteToReg(mpuAddress, 0x1C, accel);

    //set the PLL to the X axis gyroscope
    I2CWriteToReg(mpuAddress, 0x6B, 0b00000001);
}

bool MPU6050::readSensor() {
  //Possible problem, the accelZ_g value goes negative. pretty sure this is normal, the X and Y dont.

  uint8_t accel[6];
  uint8_t gyro[6];

  //read accelerometer data
  for (uint8_t i = 0; i < 6; ++i) {
    accel[i] = I2CReadFromReg(0x68, 0x3B + i);
  }

  //read gyroscope data
  for (uint8_t i = 0; i < 6; ++i) {
    gyro[i] = I2CReadFromReg(0x68, 0x43 + i);
  }

  //combine high and low bytes to form the full values
  int16_t AcY = (int16_t)(accel[2] << 8 | accel[3]);
  int16_t AcX = (int16_t)(accel[0] << 8 | accel[1]);
  int16_t AcZ = (int16_t)(accel[4] << 8 | accel[5]);
  int16_t GyX = (int16_t)(gyro[0] << 8 | gyro[1]);
  int16_t GyY = (int16_t)(gyro[2] << 8 | gyro[3]);
  int16_t GyZ = (int16_t)(gyro[4] << 8 | gyro[5]);

  //convert raw accel to accel in g's
  accelX_g = (float)AcX / accelSens;
  accelY_g = (float)AcY / accelSens;
  accelZ_g = (float)AcZ / accelSens;

  //convert raw gyro to gyro in 
  gyroX_degPerSec = ((float)GyX / gyroSens) - gyroBiasX;
  gyroY_degPerSec = ((float)GyY / gyroSens) - gyroBiasY;
  gyroZ_degPerSec = ((float)GyZ / gyroSens) - gyroBiasZ;

  return true;
}

void MPU6050::printSensorReadings(UART* uart){

  int intPart;
  int fracPart;

  char buffer[20] = "-----=========-----";
  uart->println(buffer);

  intPart = (int)accelX_g; // Get the integer part
  fracPart = abs((int)((accelX_g - intPart) * 100)); // Get the fractional part as an integer
  snprintf(buffer, sizeof(buffer), "AcX: %d.%02d", intPart, fracPart);
  uart->println(buffer);

  intPart = (int)accelY_g; // Get the integer part
  fracPart = abs((int)((accelY_g - intPart) * 100)); // Get the fractional part as an integer
  snprintf(buffer, sizeof(buffer), "AcY: %d.%02d", intPart, fracPart);
  uart->println(buffer);

  intPart = (int)accelZ_g; // Get the integer part
  fracPart = abs((int)((accelZ_g - intPart) * 100)); // Get the fractional part as an integer
  snprintf(buffer, sizeof(buffer), "AcZ: %d.%02d", intPart, fracPart);
  uart->println(buffer);

  //print gyroscope data
  intPart = (int)gyroX_degPerSec; // Get the integer part
  fracPart = abs((int)((gyroX_degPerSec - intPart) * 100)); // Get the fractional part as an integer
  snprintf(buffer, sizeof(buffer), "GyX: %d.%02d", intPart, fracPart);
  uart->println(buffer);

  intPart = (int)gyroY_degPerSec; // Get the integer part
  fracPart = abs((int)((gyroY_degPerSec - intPart) * 100)); // Get the fractional part as an integer
  snprintf(buffer, sizeof(buffer), "GyY: %d.%02d", intPart, fracPart);
  uart->println(buffer);

  intPart = (int)gyroZ_degPerSec; // Get the integer part
  fracPart = abs((int)((gyroZ_degPerSec - intPart) * 100)); // Get the fractional part as an integer
  snprintf(buffer, sizeof(buffer), "GyZ: %d.%02d", intPart, fracPart);
  uart->println(buffer);
}