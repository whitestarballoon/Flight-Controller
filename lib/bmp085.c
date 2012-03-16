//*****************************************************************************
//
// File Name		: 'bmp085.c'
// Title			: Driver Functions for BMP085 Pressure Sensor
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

//TODO: TEST THE BAROMETER BEHAVIOR IN LOW TEMPERATURES!
//      - Put Calibration Values in EEPROM

#include <stdio.h>
#include <stdint.h>
#include <util/delay.h>

#include <avr/eeprom.h>

#include "../lprintf.h"
#include "i2c.h"
#include "bmp085.h"

//Needed for BMP085
extern int16_t EEMEM EEBMPac1;
extern int16_t EEMEM EEBMPac2;
extern int16_t EEMEM EEBMPac3;
extern uint16_t EEMEM EEBMPac4;
extern uint16_t EEMEM EEBMPac5;
extern uint16_t EEMEM EEBMPac6;
extern int16_t EEMEM EEBMPb1;
extern int16_t EEMEM EEBMPb2;
extern int16_t EEMEM EEBMPmb;
extern int16_t EEMEM EEBMPmc;
extern int16_t EEMEM EEBMPmd;

void BMP085_Calibration(void)
{

	eeprom_write_word((uint16_t *)&EEBMPac1, bmp085ReadShort(0xAA));
	eeprom_write_word((uint16_t *)&EEBMPac2, bmp085ReadShort(0xAC));
	eeprom_write_word((uint16_t *)&EEBMPac3, bmp085ReadShort(0xAE));
	eeprom_write_word((uint16_t *)&EEBMPac4, bmp085ReadShort(0xB0));
	eeprom_write_word((uint16_t *)&EEBMPac5, bmp085ReadShort(0xB2));
	eeprom_write_word((uint16_t *)&EEBMPac6, bmp085ReadShort(0xB4));
	eeprom_write_word((uint16_t *)&EEBMPb1, bmp085ReadShort(0xB6));
	eeprom_write_word((uint16_t *)&EEBMPb2, bmp085ReadShort(0xB8));
	eeprom_write_word((uint16_t *)&EEBMPmb, bmp085ReadShort(0xBA));
	eeprom_write_word((uint16_t *)&EEBMPmc, bmp085ReadShort(0xBC));
	eeprom_write_word((uint16_t *)&EEBMPmd, bmp085ReadShort(0xBE));

}

short bmp085ReadShort(unsigned char address)
{
	char msb, lsb;
	short data;

	i2cDisableInt();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(BMP085FC);	// write 0xEE
	i2cWaitForComplete();

	i2cSendByte(address);	// write register address
	i2cWaitForComplete();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(BMP085FC+1);	// write 0xEF
	i2cWaitForComplete();

	i2cReceiveByte(1);
	i2cWaitForComplete();
	msb = i2cGetReceivedByte();	// Get MSB result
	i2cWaitForComplete();

	i2cReceiveByte(0);
	i2cWaitForComplete();
	lsb = i2cGetReceivedByte();	// Get LSB result
	i2cWaitForComplete();

	i2cSendStop();

	data = msb << 8;
	data |= lsb;

	i2cEnableInt();

	return data;
}

unsigned long bmp085ReadTemp(void)
{
	i2cDisableInt();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(BMP085FC);	// write 0xEE
	i2cWaitForComplete();

	i2cSendByte(0xF4);	// write register address
	i2cWaitForComplete();

	i2cSendByte(0x2E);	// write register data for temp
	i2cWaitForComplete();

	i2cSendStop();

	_delay_ms(10);	// max time is 4.5ms

	i2cEnableInt();

	return (unsigned long) bmp085ReadShort(0xF6);
}

long bmp085ReadPressure(void)
{
	long pressure = 0;

	i2cDisableInt();

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte(BMP085FC);	// write 0xEE
	i2cWaitForComplete();

	i2cSendByte(0xF4);	// write register address
	i2cWaitForComplete();

	i2cSendByte(0x34);	// write register data for pressure
	i2cWaitForComplete();

	i2cSendStop();

	_delay_ms(10);	// max time is 4.5ms

	pressure = bmp085ReadShort(0xF6);
	pressure &= 0x0000FFFF;

	i2cEnableInt();

	return pressure;

	//return (long) bmp085ReadShort(0xF6);
}

void bmp085Convert(long* temperature, long* pressure)
{

	//Globals.  These eat up RAM.  Maybe not global them?
	short ac1;
	short ac2;
	short ac3;
	unsigned short ac4;
	unsigned short ac5;
	unsigned short ac6;
	short b1;
	short b2;
	short mb;
	short mc;
	short md;
	//End Globals

	ac1 = eeprom_read_word((uint16_t *)&EEBMPac1);
	ac2 = eeprom_read_word((uint16_t *)&EEBMPac2);
	ac3 = eeprom_read_word((uint16_t *)&EEBMPac3);
	ac4 = eeprom_read_word((uint16_t *)&EEBMPac4);
	ac5 = eeprom_read_word((uint16_t *)&EEBMPac5);
	ac6 = eeprom_read_word((uint16_t *)&EEBMPac6);
	b1 = eeprom_read_word((uint16_t *)&EEBMPb1);
	b2 = eeprom_read_word((uint16_t *)&EEBMPb2);
	mb = eeprom_read_word((uint16_t *)&EEBMPmb);
	mc = eeprom_read_word((uint16_t *)&EEBMPmc);
	md = eeprom_read_word((uint16_t *)&EEBMPmd);

	//This code is modified from Sparkfun's Example code
	unsigned long ut;
	long up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;

	ut = bmp085ReadTemp();
	ut = bmp085ReadTemp();	// some bug here, have to read twice to get good data
	up = bmp085ReadPressure();
	up = bmp085ReadPressure();

	x1 = (ut - (long)ac6) * (long)ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + (long)md);
	b5 = x1 + x2;
	*temperature = (b5 + 8) >> 4;

	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((int32_t) ac1 * 4 + x3) + 2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	*pressure = p + ((x1 + x2 + 3791) >> 4);
}
