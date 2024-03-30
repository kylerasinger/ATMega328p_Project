#ifndef TIMERZERO_H
#define TIMERZERO_H

#include <stdio.h>
#include "Communication/UART.h"

/*
    Order of operations:

    1. start the timer
    2. read the timer
    3. stop the timer
*/

class TimerZero {
public:
    TimerZero();

    void start();
    long read();
    void stop();
    float getTime() { return time; };
    void printTime(UART *uart);

    volatile static long time;
    unsigned long correctedTime = time; //this is to correct for the ISR
    float timeInSeconds;

private:
    bool timerRunning = false;
};

#endif