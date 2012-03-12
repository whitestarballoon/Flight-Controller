//*****************************************************************************
//
// File Name		: 'epoch.c'
// Title			: Functions for calculating epoch given a real time
//						and configuring the star of all epochs.
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

#include "../lib/i2c.h"
#include "epoch.h"
#include <avr/eeprom.h>

#include "../eepromVars.h"

void defaultEEPROM(void)
{

//eeprom_write_byte(&EEcruiseVrecovSpeedThresh, 15);
//eeprom_write_word(&EEcruiseMinAltThresh, 7000);

eeprom_write_word(&EEballastSafetyAltThresh, 1500);

eeprom_write_byte(&EEoverOceanFlag, 0);

eeprom_write_word(&EEmaxAllowableTXInterval, 3600);

eeprom_write_byte((uint8_t *)&EEbatteryHeaterSetpoint, -20);

eeprom_write_word(&EEdataCollectionInterval, 60);
eeprom_write_word(&EEdataTransmitInterval, 900);
eeprom_write_word(&EEshortDataTransmitInterval, 300);

eeprom_write_dword(&EEepochOfLastBatchTransmit, 0);

eeprom_write_word(&EEcurrentBatchNumber, 0);
eeprom_write_word(&EEbatchSampleStart, 0);
eeprom_write_word(&EEbatchSampleEnd, 0);

eeprom_write_word(&EEcommPromStart, 0);
eeprom_write_word(&EEcommPromEnd, 0);

eeprom_write_byte(&EEflightComputerResetCount, 0);
eeprom_write_byte(&EEcommModuleResetCount, 0);

eeprom_write_byte(&EEflightPhase, 0x10);

eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);

eeprom_write_word(&EEcurrentTelemetryVersion, 0);

eeprom_write_word(&EEballastTargetAltitude, 10000);
eeprom_write_word((uint16_t *)&EEballastTargetPositiveVSpeed, 5);
eeprom_write_word((uint16_t *)&EEballastTargetNegativeVSpeed, -5);

eeprom_write_word(&EEmaydayAltitude, 1000);
eeprom_write_word((uint16_t *)&EEmaydayVSpeed, -100);

eeprom_write_byte(&EEautoBallastDisable, 1);

eeprom_write_word(&EEhfDataTransmitInterval, 3600);
eeprom_write_byte(&EEhfRapidTransmit, 30);

eeprom_write_byte(&EEEpochLock, 0);

}

uint32_t getEpochSeconds(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t days)
{
	uint8_t eeSeconds = eeprom_read_byte(&EEepochStartSeconds);
	uint8_t eeMinutes = eeprom_read_byte(&EEepochStartMinutes);
	uint8_t eeHours = eeprom_read_byte(&EEepochStartHours);
	uint8_t eeDays = eeprom_read_byte(&EEepochStartDays);

	return (((long int)days - (long int)eeDays) * 86400) + (((long int)hours - (long int)eeHours) * 3600)
			+ (((long int)minutes - (long int)eeMinutes) * 60) + ((long int)seconds - (long int)eeSeconds);
}

void writeEpochStart(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t days)
{
	 eeprom_write_byte(&EEepochStartSeconds, seconds);
	 eeprom_write_byte(&EEepochStartMinutes, minutes);
	 eeprom_write_byte(&EEepochStartHours, hours);
	 eeprom_write_byte(&EEepochStartDays, days);
}

