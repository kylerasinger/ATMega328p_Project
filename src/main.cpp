#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "UART.h"
#include "UsSensor.h"
#include "IrSensor.h"
#include "Utilities.h"


/* ---===       PIN DECLARATION      ===--- */

  //define pins for the US sensor
  #define US_TRIG PB4
  #define US_ECHO PB0

  //define pins for the IR
  #define IR_IN PC0

  //define LED pin
  #define LED_PIN PB3
  #define Y_LED_PIN PB5

/* ---===   END OF PIN DECLARATION   ===--- */


//Initialize objects
UART uart;
UsSensor usSensor(US_ECHO, US_TRIG, &uart);
IrSensor irSensor(IR_IN, &uart);

void set_brightness(int brightness){
  OCR2A = brightness;
}

int main(void)
{
  //true for US, false for IR
  const bool MODE = false;

  /* --===-- Setup LED --===-- */
  //sets the LED pin as an output
  DDRB |= (1 << LED_PIN);
  PORTB |= (1 << LED_PIN);    
  
  /* --===-- Setup Timers --===-- */
  //this is used in ADC aswell as the US
  //using 8 bit timer with fast PWM, we can use it for US without worrying about
    //an overflow because the max distance that can be read from the timer is approx 550cm
    //and our sensor can only go to 400cm
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); 
  TCCR2B = (1 << CS22); //prescaler at 64, 128 is "slowmode" MIGHT BE ABLE TO REMOVE IF IR AND US BOTH STILL WORK

  //we use uint16_t because uint8_t wont fit the entire range of sensors.
  uint16_t distance = 0;

  while(true){
    distance = (MODE) ? usSensor.getDistance() : irSensor.getDistance();

    if(distance < 15){
      //set led brightness to 100
      set_brightness(255);
      //turn on yellow led
      PORTB |= (1 << Y_LED_PIN);
    }else if(distance >= 15 && distance < 40){
      //set led brightness to a linear percent
      set_brightness(map(distance, 15, 45, 255, 0));
      PORTB &= ~(1 << Y_LED_PIN);
    }else if(distance >= 40){
      set_brightness(0);
      PORTB |= (1 << Y_LED_PIN);
    }

    char buffer[32];
    sprintf(buffer, "Distance: %u cm", distance);
    uart.println(buffer);

    _delay_ms(250);
  }
}
