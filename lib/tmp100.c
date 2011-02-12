//*****************************************************************************
//
// File Name		: 'tmp100.c'
// Title			: Driver Functions for TMP100 Temperature Sensor
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

#include "i2c.h"
#include "tmp100.h"

uint16_t tmp100rawTemp(uint8_t address)
{
	uint16_t error;

	i2cDisableInt();

	i2cSendStart();
	error |= i2cWaitForComplete();

	i2cSendByte(address);
	error |= i2cWaitForComplete();
	i2cSendByte(0b00000000);
	error |= i2cWaitForComplete();

	i2cSendStart();
	error |= i2cWaitForComplete();

	i2cSendByte(address+1);
	error |= i2cWaitForComplete();

	i2cReceiveByte(1);
	error |= i2cWaitForComplete();
	uint8_t data1 = i2cGetReceivedByte(); //Read the MSB data
	error |= i2cWaitForComplete();

	i2cReceiveByte(0);
	error |= i2cWaitForComplete();
	uint8_t data2 = i2cGetReceivedByte(); //Read the LSB data
	error |= i2cWaitForComplete();

	i2cSendStop();
	i2cEnableInt();

	if(error == 0)
	{
		return data1 << 8 | data2;
	} else {
		return 0xEFFF;
	}

}

uint8_t getTMP100config(uint8_t address)
{

	i2cDisableInt();
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(address);
	i2cWaitForComplete();
	i2cSendByte(0b00000001);
	i2cWaitForComplete();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(address+1);
	i2cWaitForComplete();

	i2cReceiveByte(1);
	i2cWaitForComplete();
	uint8_t data1 = i2cGetReceivedByte(); //Read the MSB data
	i2cWaitForComplete();

	i2cSendStop();
	i2cEnableInt();

	return data1;

}

uint8_t setTMP100config(uint8_t address, uint8_t data)
{
	uint8_t error;
	i2cDisableInt();
	i2cSendStart();
	error |= i2cWaitForComplete();

	i2cSendByte(address);
	error |= i2cWaitForComplete();

	i2cSendByte(0b00000001);
	error |= i2cWaitForComplete();

	i2cSendByte(data);
	error |= i2cWaitForComplete();

	i2cSendStop();
	i2cEnableInt();

	return error;

}

uint8_t setTMP101Thermo(uint8_t address, uint8_t data, uint8_t tHigh)
{

	uint8_t error;
	i2cDisableInt();
	i2cSendStart();
	error |= i2cWaitForComplete();

	i2cSendByte(address);
	error |= i2cWaitForComplete();

	i2cSendByte(0b00000010 + tHigh);
	error |= i2cWaitForComplete();

	i2cSendByte(data);
	error |= i2cWaitForComplete();

	i2cSendStop();
	i2cEnableInt();

	return error;

}

//THIS DOESN'T BELONG HERE, REMOVE IT
int16_t get12bit2scomp(uint16_t value)
{
	if(value > 0x7FF)
	{
		return (value&0x7FF) - 2048;
	} else {
		return (int16_t)value;
	}
}

uint16_t set12bit2scomp(int16_t value)
{
	if(value < 0)
	{
		return (value + 2048) + 0x800;
	} else
	{
		return (uint16_t)value;
	}
}
