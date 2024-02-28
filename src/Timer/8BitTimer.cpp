#include "8BitTimer.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile long EightBitTimer::time = 0;

ISR(TIMER2_OVF_vect) {
  EightBitTimer::time += 255;
  TCNT2 = 0;
}

EightBitTimer::EightBitTimer() {
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

void EightBitTimer::start(){
  TCCR2B = 0; // Stop timer 2
  TCNT2 = 0;  // Reset timer 2
  TIMSK2 |= (1 << TOIE2); // Enable Timer2 Overflow Interrupt
  TCCR2B |= (1 << CS20); // Start timer 2 with no prescaling
  time = 0;
  timerRunning = true;

}

long EightBitTimer::read(){
  timerRunning = false;
  return time;
}

void EightBitTimer::stop(){
  TCCR2B &= ~(1 << CS20); // Stop timer
  TIMSK2 &= ~(1 << TOIE2); // Disable Timer2 Overflow Interrupt
}
