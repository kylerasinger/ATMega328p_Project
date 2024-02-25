#include "Utilities.h"
#include <avr/io.h>

void setUp8BitTimer(){
    /* --===-- Setup Timers --===-- */
  //this is used in ADC aswell as the US
  //using 8 bit timer with fast PWM, we can use it for US without worrying about
    //an overflow because the max distance that can be read from the timer is approx 550cm
    //and our sensor can only go to 400cm
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); 
  TCCR2B = (1 << CS22); //prescaler at 64, 128 is "slowmode" MIGHT BE ABLE TO REMOVE IF IR AND US BOTH STILL WORK
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void turnOnYellowLED(){

  PORTB |= (1 << PB5);
}

void turnOffYellowLED(){
  PORTB &= ~(1 << PB5);
}

/* Legacy Code */

/*
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