#ifndef UART_H
#define UART_H

class UART {
    public:
        void putChar(char c);
        void print(const char* str);
        void println(const char* str);
};

#endif