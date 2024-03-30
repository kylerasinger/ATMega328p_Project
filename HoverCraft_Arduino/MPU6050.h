#ifndef MPU6050_H
#define MPU6050_H

#include "I2C.h"
#include "UART.h"

/*
    Accelerometer: Measures the force in g's (9.82m/s^2)
        X axis:
            !!!always postive 
        Y axis:
            !!!always positive
        Z axis:
            positive for upwards, negative upside down

    Gyroscope: Measures the rotational motion in degree's per second (deg/s)
        X axis:
            Looking from behind, roll to the right is +, roll to the left is -
        Y axis:
            Forward tilt is +, Backwards tilt is -
        Z axis: 
            From top down, clockwise is a - value, counter clock is +.
*/
class MPU6050 {
public: 
    MPU6050(uint8_t mpuAddressm, uint8_t gyro, uint8_t accel);
    bool readSensor();
    void printSensorReadings( UART* uart);

    //getters setters
    float getAccelX_g(){ return accelX_g; };
    float getAccelY_g(){ return accelY_g; };
    float getAccelZ_g(){ return accelZ_g; };
    float getGyroX_degPerSec(){ return gyroX_degPerSec; };
    float getGyroY_degPerSec(){ return gyroY_degPerSec; };
    float getGyroZ_degPerSec(){ return gyroZ_degPerSec; };

private: 
    float gyroSens = 0;
    uint16_t accelSens = 0;
    
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