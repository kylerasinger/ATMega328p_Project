#include <avr/io.h>
#include <util/delay.h>
#include "UsSensor.h" // Assuming UsSensor.h contains the declaration of UsSensor class

// Constructor
UsSensor::UsSensor(int echoPin, int trigPin) : echoPin(echoPin), trigPin(trigPin) {
    // Set echoPin as input and trigPin as output
    DDRB &= ~(1 << echoPin);
    DDRB |= (1 << trigPin);
}

// Function to get distance measured by the ultrasonic sensor
uint16_t UsSensor::getDistance() {
    const int maxDistCM = 400;
    const int minDistCM = 2;

    // Speed of sound depending on temperature
    const float speedOfSoundCMperMicroSec = 0.01029f; //0.0343 is speed of sound and we do x3

    // Trigger the ultrasonic sensor's cycle
    PORTB &= ~(1 << trigPin); // Set trigPin LOW
    _delay_us(2);
    PORTB |= (1 << trigPin); // Set trigPin HIGH
    _delay_us(10);
    PORTB &= ~(1 << trigPin); // Set trigPin LOW

    // Calculate ultrasound pulse duration
    float pulseUS = getUsPulse();

    // Convert pulse duration to distance in centimeters
    uint16_t distCM = (pulseUS * speedOfSoundCMperMicroSec) / 2;

    // Out-of-range error catch
    if (distCM < minDistCM || distCM > maxDistCM) {
        return 0;
    }

    return distCM;
}

// Function to get ultrasound pulse duration in microseconds
float UsSensor::getUsPulse() {
    unsigned short pulseDuration = 0;

    // Stop Timer2 and reset its count
    TCCR2B = 0;
    TCNT2 = 0;

    // Wait for echo pin to go HIGH
    while (!(PINB & (1 << echoPin)));

    // Start Timer2 (no prescaler)
    TCCR2B |= (1 << CS20);

    // Wait for echo pin to go LOW and measure pulse duration
    while (PINB & (1 << echoPin)) {
        if (TCNT2 == 255) { // Check for overflow
            pulseDuration += 255; // Increment pulse duration on overflow
            TCNT2 = 0; // Reset Timer2 count
        }
        pulseDuration++;
    }

    // Stop Timer2
    TCCR2B &= ~(1 << CS20);

    return pulseDuration;
}
