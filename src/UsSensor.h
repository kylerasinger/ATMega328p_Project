#ifndef US_SENSOR_H
#define US_SENSOR_H

#include "Sensor.h" // Include the base class 'Sensor'
#include "UART.h"
#include <stdint.h> // For uint8_t, uint16_t types

class UsSensor : public Sensor {
public:
    UsSensor(int echoPin, int trigPin, UART* uart); // Constructor with pin assignments
    uint16_t getDistance() override; // Override the getDistance() method from Sensor

private:
    int echoPin; // Pin used to receive the echo signal
    int trigPin; // Pin used to trigger the ultrasonic pulse
    UART* uart;

    int getUsPulse(); // Method to measure the duration of the echo pulse
};

#endif // US_SENSOR_H