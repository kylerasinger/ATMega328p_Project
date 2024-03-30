#include "Utilities.h"
#include <avr/io.h>

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t map(uint8_t x, uint16_t inMin, uint16_t inMax, uint16_t outMin, uint16_t outMax){
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void turnOnYellowLED(){

  PORTB |= (1 << PB5);
}

void turnOffYellowLED(){
  PORTB &= ~(1 << PB5);
}