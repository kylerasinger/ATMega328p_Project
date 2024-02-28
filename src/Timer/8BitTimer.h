#ifndef EIGHTBITTIMER_H
#define EIGHTBITTIMER_H

#include <stdio.h>

/*
    Order of operations:

    1. start the timer
    2. read the timer
    3. stop the timer
*/

class EightBitTimer {
    public:
            EightBitTimer();
            
            void start();
            long read();
            void stop();
            long getTime() { return time; };

            volatile static long time;

            
        private:
            bool timerRunning = false;

};


#endif