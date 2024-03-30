#ifndef US_SENSOR_H
#define US_SENSOR_H

#include <stdint.h> // For uint16_t type

/*
    Class for an ultrasonic sensor.
    This class provides functionality to measure distance using an ultrasonic sensor.
    It inherits from a base Sensor class.
*/

class UsSensor {
public:
    // Constructor: Initializes the ultrasonic sensor with pin assignments
    UsSensor(int echoPin, int trigPin);

    // Method to get the distance measured by the ultrasonic sensor
    uint16_t getDistance();

private:
    int echoPin; // Pin used to receive the echo signal
    int trigPin; // Pin used to trigger the ultrasonic pulse

    // Private method to measure the duration of the echo pulse
    float getUsPulse();
};

#endif // US_SENSOR_H
