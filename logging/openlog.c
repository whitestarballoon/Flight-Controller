//*****************************************************************************
//
// File Name		: 'openlog.c'
// Title			: Driver Routines for the Sparkfun Openlog
// Original Author	: Brad Luyster, LVL1 White Star Balloon Project
// Created			: 2010.12.2
// Modified			: I won't keep up with this
// Target MCU		: Atmel AVR series
//
// http://www.whitestarballoon.com/
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

//*****************************************************************************
//
// TODO:
//   - Add Checks and timeouts!
//
//*****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>

#include <util/delay.h>

#include "../dataStructures.h"

#include "openlog.h"

#define OPENLOGUBRR 77 //9600 baud

static FILE olout = FDEV_SETUP_STREAM(ol_putchar, NULL, _FDEV_SETUP_WRITE);
extern int lprintf(char *, ...);

//REMOVE BEFORE FLIGHT, THIS REMOVES THE LOG ON INIT
void initOpenLogTest(void)
{
	
	UBRR0H = OPENLOGUBRR >> 8;
    UBRR0L = OPENLOGUBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	_delay_ms(1500);
	
	fprintf(&olout, "%c%c%c\r", (char)0x1a,(char)0x1a,(char)0x1a);
	while((char)ol_getchar() != '>');
	
	fprintf(&olout, "echo off\r");
	while((char)ol_getchar() != '>');
	
	fprintf(&olout, "\r");
	while((char)ol_getchar() != '>');
	
	fprintf(&olout, "rm SAMPLES.TXT\r");
	while((char)ol_getchar() != '>');
	
	fprintf(&olout, "new SAMPLES.TXT\r");
	while((char)ol_getchar() != '>');
	
}

void initOpenLogFlight(void)
{
	
	UBRR0H = OPENLOGUBRR >> 8;
    UBRR0L = OPENLOGUBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	//Delay for Openlog to start
	_delay_ms(2000);
	//Ensure we're in command mode.
	fprintf(&olout, "%c%c%c\r", (char)0x1a,(char)0x1a,(char)0x1a);

}

void putDataSample(char *string)
{
	//lprintf("h1\n");
	//Open file for writing
	fprintf(&olout, "append SAMPLES.TXT\r");
	while((char)ol_getchar() != '<');
	//write the string (must be null terminated!)
	//lprintf("P: %s\n", string);
	fprintf(&olout, "%s\n", string);
	_delay_ms(100);
	//Back to command mode.
	fprintf(&olout, "%c%c%c\r", (char)0x1a, (char)0x1a, (char)0x1a);
	while((char)ol_getchar() != '>');
	while((char)ol_getchar() != '>');
	//Sync to card in case we lose power or something
	fprintf(&olout, "sync\r");
	while((char)ol_getchar() != '>');

}

//16 bits is fine for speedball-1, check again for future flights, though
//Also, returnString needs to be long enough to hold the output plus null terminator, or bad things happen.
void getDataSample(uint16_t sampleNumber, char *returnString)
{
	uint16_t i;
	
	fprintf(&olout, "\r");
	while((char)ol_getchar() != '>');
	i=0;
	volatile char thisChar;
	fprintf(&olout, "read samples.txt %d %d\r", (sampleNumber * (SAMPLESTRINGSIZEINCHARS + 2)), SAMPLESTRINGSIZEINCHARS);
	ol_getchar(); // discard first char
	ol_getchar(); // discared the second char!
	while( (thisChar = (char)ol_getchar())!='\r')
	{
		returnString[i] = thisChar;
		i++;
	}
	
	returnString[i] = '\0';
	while( (thisChar = (char)ol_getchar())!='>');
	
}

int ol_putchar(char c, FILE *stream)
{
    if (c == '\n') ol_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}

uint8_t ol_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}
