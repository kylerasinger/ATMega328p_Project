#ifndef SERVO_H
#define SERVO_H

#include <avr/io.h>

class Servo {
public:
    Servo(uint8_t servoPin);
    void setServoAngle(uint8_t angle);

private:
};

#endif