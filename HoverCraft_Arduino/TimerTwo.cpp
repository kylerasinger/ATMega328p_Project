#include "TimerTwo.h"
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

volatile long TimerTwo::time = 0;

//THIS IS TIMER 2 and it is 8 bit

ISR(TIMER2_OVF_vect) {
  TimerTwo::time += 255;
  TCNT2 = 0;
}

TimerTwo::TimerTwo() {
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

void TimerTwo::start(){
  TCCR2B = 0; // Stop timer 2
  TCNT2 = 0;  // Reset timer 2
  TIMSK2 |= (1 << TOIE2); // Enable Timer2 Overflow Interrupt
  TCCR2B |= (1 << CS20); // Start timer 2 with no prescaling
  time = 0;
  timerRunning = true;
}

long TimerTwo::read(){
  unsigned long correctedTime = static_cast<unsigned long>(time * 1.0); //apply a correction here if needed
  TimerTwo::correctedTime = correctedTime;
  
  TimerTwo::timeInSeconds = correctedTime/16000000.0;

  timerRunning = false;

  return time;
}

void TimerTwo::stop(){
  TCCR2B &= ~(1 << CS20); // Stop timer
  TIMSK2 &= ~(1 << TOIE2); // Disable Timer2 Overflow Interrupt
}

void TimerTwo::printTime(UART *uart) {
  // float timeInSeconds = correctedTime / 16000000.0;

  unsigned long intPart = static_cast<unsigned long>(TimerTwo::timeInSeconds);
  unsigned long fracPart = static_cast<unsigned long>(fabs(TimerTwo::timeInSeconds - intPart) * 100000);

  char buffer[40];

  sprintf(buffer, "Time (cycles): %lu", correctedTime);
  uart->println(buffer);

  sprintf(buffer, "Time (seconds): %lu.%05lu", intPart, fracPart);
  uart->println(buffer);
}