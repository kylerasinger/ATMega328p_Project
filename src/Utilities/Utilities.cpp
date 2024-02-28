#include "Utilities.h"
#include <avr/io.h>

/*




*/

void setUp8BitTimer(){
    /* --===-- Setup Timers --===-- */
  //this is used in ADC aswell as the US
  //using 8 bit timer with fast PWM, we can use it for US without worrying about
    //an overflow because the max distance that can be read from the timer is approx 550cm
    //and our sensor can only go to 400cm
      //COM2A1 clears the OC2A on compare match
      //COM2B1 clears the OC2B on compare match
      //WGM21 and WGM20 set FAST PWM with top of 0xff (255)
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); 
  TCCR2B = (1 << CS22); //prescaler at 64, 128 is "slowmode" MIGHT BE ABLE TO REMOVE IF IR AND US BOTH STILL WORK
}



int read8BitTimer(){
  //time is store in TCNT2
  int time = 0;
  while(true){ //!!!CHANGE!!!
    if(TCNT2 == 255){
      time += 255;
    }
  }
  return (time + TCNT2);
}

void stop8BitTimer(){
  TCCR2B &= ~(1 << CS20); //stop timer
}

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