#include <avr/io.h>
#include "UART.h"

UART::UART(){
    /* --===-- Setup Serial Coms --===-- */
    //enable transmitter and receiver for USART
    UCSR0B = (1 << RXEN0)|(1 << TXEN0);
    //Sets the frame format as 8 bits for serial comms
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    //we are using a baud rate of 9600 bps at 16mhz because it is standar
    UBRR0H = 0;
    UBRR0L = 103; 
}

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

