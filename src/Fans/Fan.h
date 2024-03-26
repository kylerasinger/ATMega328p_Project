#ifndef FAN_H
#define FAN_H

#include <avr/io.h>

class Fan {
    public:
        Fan(uint8_t fanPin);
        void setSpeed(uint8_t speed);
    private:
        uint8_t fanPin;
};

#endif