#include <avr/io.h>
#include "LED.h"

LED::LED(int ledPin){
    DDRB |= (1 << ledPin);
    PORTB |= (1 << ledPin);    
    OCR2A = 0;
}

void LED::setBrightness(int brightness){
  OCR2A = brightness;
}