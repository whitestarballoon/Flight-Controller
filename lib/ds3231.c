//*****************************************************************************
//
// File Name		: 'ds3231.c'
// Title			: Driver Functions for DS3231 TXCO Real Time Clock
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

#include <stdio.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>

#include "i2c.h"
#include "ds3231.h"

#define STATUS_LED1 3
#define STATUS_LED2 4
#define red_on()     ( PORTB |= (1 << STATUS_LED2)  )

uint8_t getTime(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *days)
{

	uint8_t error;

	i2cDisableInt();

	i2cSendStart();
	error |= i2cWaitForComplete();

	i2cSendByte(DS3231FC);	// write 0xEE
	error |= i2cWaitForComplete();

	i2cSendByte(0x00);	// write register address
	error |= i2cWaitForComplete();

	i2cSendStart();
	error |= i2cWaitForComplete();

	i2cSendByte(DS3231FC+1);	// rep start
	error |= i2cWaitForComplete();

	i2cReceiveByte(1);
	error |= i2cWaitForComplete();
	*seconds = i2cGetReceivedByte();	// Get seconds
	error |= i2cWaitForComplete();

	i2cReceiveByte(1);
	error |= i2cWaitForComplete();
	*minutes = i2cGetReceivedByte();	// Get minutes
	error |= i2cWaitForComplete();

	i2cReceiveByte(1);
	error |= i2cWaitForComplete();
	*hours = i2cGetReceivedByte();	// Get hours
	error |= i2cWaitForComplete();

	i2cReceiveByte(0);
	error |= i2cWaitForComplete();
	*days = i2cGetReceivedByte();	// Get days
	error |= i2cWaitForComplete();

	i2cSendStop();

	i2cEnableInt();

	*seconds = (*seconds & 0x0F) + ((*seconds & 0xF0) >> 4)*10;
	*minutes = (*minutes & 0x0F) + ((*minutes & 0xF0) >> 4)*10;
	*hours = (*hours & 0x0F) + ((*hours & 0b00110000) >> 4)*10;
	*days = (*days & 0x0F) + ((*days & 0xF0) >> 4)*10;

	return error;

}

inline uint8_t getSeconds(void)
{

	uint8_t temp = ds3231read(0x00);
	return (temp & 0x0F) + ((temp & 0xF0) >> 4)*10;

}

inline uint8_t getMinutes(void)
{
	uint8_t temp = ds3231read(0x01);
	return (temp & 0x0F) + ((temp & 0xF0) >> 4)*10;
}

inline uint8_t getHours(void)
{

	uint8_t temp = ds3231read(0x02);
	return (temp & 0x0F) + ((temp & 0b00110000) >> 4)*10;

}

inline uint8_t getDays(void)
{
	uint8_t temp = ds3231read(0x03);
	return (temp & 0x0F) + ((temp & 0xF0) >> 4)*10;

}


void ds3231write(uint8_t address, uint8_t data)
{

	i2cDisableInt();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(DS3231FC);	// write 0xEE
	i2cWaitForComplete();

	i2cSendByte(address);	// write register address
	i2cWaitForComplete();

	i2cSendByte(data);	// write data address
	i2cWaitForComplete();

	i2cSendStop();

	i2cEnableInt();

}

uint8_t ds3231read(uint8_t address)
{
	uint8_t data;

	i2cDisableInt();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(DS3231FC);	// write 0xEE
	i2cWaitForComplete();

	i2cSendByte(address);	// write register address
	i2cWaitForComplete();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(DS3231FC+1);	// rep start
	i2cWaitForComplete();

	i2cReceiveByte(0);
	i2cWaitForComplete();
	data = i2cGetReceivedByte();	// Get result
	i2cWaitForComplete();

	i2cSendStop();

	i2cEnableInt();

	return data;

}

