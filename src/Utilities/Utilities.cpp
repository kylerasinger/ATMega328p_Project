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

/* Legacy Code */

/* technical asmt 1
  distance = (MODE) ? usSensor.getDistance() : irSensor.getDistance();

  if(distance < 15){
    largeLED.setBrightness(255);
    turnOnYellowLED();
  }else if(distance >= 15 && distance < 40){
    //set led brightness to a linear percent
    largeLED.setBrightness(map(distance, 15, 45, 255, 0));
    turnOffYellowLED();
  }else if(distance >= 40){
    largeLED.setBrightness(0);
    turnOnYellowLED();
  }

  char buffer[32];
  sprintf(buffer, "Distance: %u cm", distance);
  uart.println(buffer);

  _delay_ms(250);
*/