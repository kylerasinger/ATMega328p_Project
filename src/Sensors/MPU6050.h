#ifndef MPU6050_H
#define MPU6050_H

#include "../Utilities/I2C.h"
#include "../Communication/UART.h"

class MPU6050 {
public: 
    MPU6050(uint8_t mpuAddressm, uint8_t gyro, uint8_t accel, LED* led);
    bool readSensor(LED* led, UART* uart);

private: 
    float gyroSens = 0;
    uint16_t accelSens = 0;
    LED* debugLed;
    
    float accelX_g = 0;
    float accelY_g = 0;
    float accelZ_g = 0;
    float gyroX_degPerSec = 0;
    float gyroY_degPerSec = 0;
    float gyroZ_degPerSec = 0;

    //these are currently done at home, should be done in the room where we test.
    float accelBiasX = 0;
    float accelBiasY = 0;
    float accelBiasZ = 0;
    float gyroBiasX = -1.65;
    float gyroBiasY = 0.20;
    float gyroBiasZ = -1.35;
};

#endif