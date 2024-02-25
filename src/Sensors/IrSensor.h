#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Sensor.h" // Include the base class 'Sensor'
#include "../Communication/UART.h"
#include <stdint.h> // For uint8_t, uint16_t types

class IrSensor : public Sensor{
public:
    IrSensor(int irInPin, UART* uart); // Constructor with pin assignments
    uint16_t getDistance() override;

private:
    int irInPin;
    UART* uart;
    


};

#endif