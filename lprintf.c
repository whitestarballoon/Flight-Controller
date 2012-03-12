#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include <avr/pgmspace.h>

#include "lprintf.h"
#include "lib/i2c.h"

//LEDs on PB3 and PB4
#define STATUS_PORT PORTB
#define STATUS_LED1 3
#define STATUS_LED2 4

#define yellow_on()  ( PORTB |= (1 << STATUS_LED1)  )
#define red_on()     ( PORTB |= (1 << STATUS_LED2)  )

#define yellow_off() ( PORTB &= ~(1 << STATUS_LED1) )
#define red_off()    ( PORTB &= ~(1 << STATUS_LED2) )

int lprintf(char *str, ...)
{
	char lstr[301];
	int chars;
	va_list args;

	va_start(args, str);

	lstr[0] = 'F';

	chars = vsnprintf(lstr+1, 300, str, args);

	if(chars > 300)
	{
		va_end(args);
		return 1;
	} else {
		uint8_t retVal = i2cMasterSendNI(0b00001110, chars+1, lstr);
		if(retVal != I2C_OK)
		{
			yellow_on();
		}

		va_end(args);
		return 0;
	}
	_delay_ms(100);
}

int lprintf_P(const char *str, ...)
{
	char lstr[100];
	uint8_t i2cSend[101];
	int chars;
	va_list args;

	va_start(args, str);

	chars = vsnprintf_P(lstr, 100, str, args);

	if(chars > 100)
	{
		va_end(args);
		return 1;
	} else {
		int i=0;
		i2cSend[0] = 'F';
		for(i=0; i <=chars; i++)
		{
			i2cSend[i+1] = (uint8_t)lstr[i];
		}
		uint8_t retVal = i2cMasterSendNI(0b00001110, chars+1, i2cSend);
		if(retVal != I2C_OK)
		{
			yellow_on();
		}

		va_end(args);
		return 0;
	}
	_delay_ms(100);
}