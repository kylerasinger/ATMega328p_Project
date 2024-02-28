#include <avr/io.h>
#include "LED.h"

LED::LED(int ledPin){
    DDRB |= (1 << ledPin);
    PORTB |= (1 << ledPin);    
    OCR2A = 0;
    setBrightness(0);
}

void LED::setBrightness(int brightness){ //0 to 255
  //its backwards in the conu board.
  brightness = 255 - brightness;
  OCR2A = brightness;
}