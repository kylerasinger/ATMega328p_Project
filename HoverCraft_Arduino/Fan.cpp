#include <avr/io.h>
#include "Fan.h"

Fan::Fan(uint8_t fanPin){
    Fan::fanPin = fanPin;
    // Set fan pin as output
    DDRD |= (1 << fanPin);

    //set Timer0 to fast PWM 
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    
    // Choose non-inverting mode
    // COM0A1:COM0A0 = 10 for non-inverting mode if fanPin is PD6 (OC0A)
    // COM0B1:COM0B0 = 10 for non-inverting mode if fanPin is PD5 (OC0B)
    if(fanPin == PD6) {
        TCCR0A |= (1 << COM0A1);
    } else if(fanPin == PD5) {
        TCCR0A |= (1 << COM0B1);
    }

    // Set the prescaler and start PWM
    // CS02:CS00 = 001 for no prescaling, 010 for clk/8, etc.
    TCCR0B |= (1 << CS00); // No prescaler for maximum frequency
}

void Fan::setSpeed(uint8_t speed){
    // Adjust PWM duty cycle for speed control
    // OCR0A for PD6, OCR0B for PD5
    if(Fan::fanPin == PD6) {
        OCR0A = speed;
    } else if(fanPin == PD5) {
        OCR0B = speed;
    }
}