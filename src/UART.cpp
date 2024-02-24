#include <avr/io.h>
#include "UART.h"

void UART::putChar(char c) {
    //busy wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    //load char into USART i/o data register
    UDR0 = c;
}

void UART::print(const char *str) {
    while (*str) {
        putChar(*str++);
    }
}

void UART::println(const char *str) {
    print(str);
    putChar('\r');
    putChar('\n');
}
