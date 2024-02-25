#include <avr/io.h>
#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
    public:
        virtual uint16_t getDistance() = 0;
};

#endif