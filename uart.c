#include <stdio.h>
#include <avr/io.h>

#include "uart.h"

int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);

    loop_until_bit_is_set(UCSR1A, UDRE1);
    UDR1 = c;

    return 0;
}

uint8_t uart_getchar(void)
{
    uint16_t errorCounter = 0xFFFF;
    while( !(UCSR1A & (1<<RXC1)) && (errorCounter > 100))
    {
        errorCounter--;
    }
    if(errorCounter <= 101)
    {
        return 0xff;
    } else {
        return UDR1;
    }

}