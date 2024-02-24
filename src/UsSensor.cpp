// #include "Sensor.h"
#include "UsSensor.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

UsSensor::UsSensor(int echoPin, int trigPin, UART* uart) : echoPin(echoPin), trigPin(trigPin){
    DDRB &= ~(1 << this->echoPin); // Set echoPin as input
    DDRB |= (1 << this->trigPin);  // Set trigPin as output
}

uint16_t UsSensor::getDistance() {
    /*
     * There are two values we can use to calibrate. The division at the end of distCM
     * and the calibrationFloat. The division should be 2 for the time of the pulse to 
     * send, hit and come back. It is 4 at the moment, not sure why its more accurate.
    */

    int maxDistCM = 400;
    int minDistCM = 2;
    int temp = 22.8;

    float speedOfSoundCMperMicroSec = 0.03313 + (0.0000606 * temp);  //speed of sound depending on temperature

    //pulse the US sensors input, triggering the US's cycle
    PORTB &= ~(1 << trigPin);
    _delay_us(2);
    PORTB |= (1 << trigPin);
    _delay_us(10);
    PORTB &= ~(1 << trigPin);

    //calculate ultrasound pulse
    int pulseUS = getUsPulse();

    char buffer2[32];
    sprintf(buffer2, "PulseUS: %u us", pulseUS);
    uart->println(buffer2);

    float calibrationFloat = 1.30400;
    uint16_t distCM = ((pulseUS * calibrationFloat) * speedOfSoundCMperMicroSec)/2; //pulseUS is the pulse length in microseconds
    
    //out of range error catch
    if(distCM < minDistCM || distCM > maxDistCM){
        return 0;
    }
    return distCM;
}

int UsSensor::getUsPulse(){
        //this function returns the length of teh ultrasound pulse but in microseconds.
    uint16_t pulseDuration = 0;
    
    TCCR2B = 0; //stop timer 2
    TCNT2 = 0;  //reset timer 2

    //while echo pin is high
    while (!(PINB & (1 << echoPin))); 

    //start timer (no prescaler because of the reset done 6 lines up)
    TCCR2B |= (1 << CS20);

    //while echo pin is low
    while (PINB & (1 << echoPin)) {
        if (TCNT2 == 255) {  //counts and checks for overflow TRY REMOVING THIS, I SHOULDNT HAVE ANY OVERFLOW REMOVE
        pulseDuration += 255;  //on each overflow
        TCNT2 = 0;  // Reset Timer2 to count from 0 again
        }
    }
    pulseDuration += TCNT2;

    //stop the timer
    TCCR2B &= ~(1 << CS20);

    pulseDuration = pulseDuration/4;
    return pulseDuration;
}