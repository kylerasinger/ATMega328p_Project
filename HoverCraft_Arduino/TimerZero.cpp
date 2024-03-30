#include "TimerZero.h"
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

volatile long TimerZero::time = 0;

// ISR(TIMER0_OVF_vect) {
//   TimerZero::time += 255;
//   TCNT0 = 0;
// }

TimerZero::TimerZero() {
/* --===-- Setup Timers --===-- */

  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS02);
}

void TimerZero::start(){
  TCCR0B = 0; // Stop timer 0
  TCNT0 = 0;  // Reset timer 0
  TIMSK0 |= (1 << TOIE0); // Enable Timer0 Overflow Interrupt
  TCCR0B |= (1 << CS00); // Start timer0 with no prescaling
  time = 0;
  timerRunning = true;
}

long TimerZero::read(){
  unsigned long correctedTime = static_cast<unsigned long>(time * 1.0); //apply a correction here if needed
  TimerZero::correctedTime = correctedTime;
  
  TimerZero::timeInSeconds = correctedTime/16000000.0;

  timerRunning = false;

  return time;
}

void TimerZero::stop(){
  TCCR0B &= ~(1 << CS00); // Stop timer
  TIMSK0 &= ~(1 << TOIE0); // Disable Timer2 Overflow Interrupt
}

void TimerZero::printTime(UART *uart) {
  // float timeInSeconds = correctedTime / 16000000.0;

  unsigned long intPart = static_cast<unsigned long>(TimerZero::timeInSeconds);
  unsigned long fracPart = static_cast<unsigned long>(fabs(TimerZero::timeInSeconds - intPart) * 100000);

  char buffer[40];

  sprintf(buffer, "Time (cycles): %lu", correctedTime);
  uart->println(buffer);

  sprintf(buffer, "Time (seconds): %lu.%05lu", intPart, fracPart);
  uart->println(buffer);
}