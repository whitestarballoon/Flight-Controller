//*****************************************************************************
//
// File Name		: 'eepromVars.h'
// Title			: List of EEProm Variables
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

#ifndef EEPROMVARS_H
#define EEPROMVARS_H

#include <avr/eeprom.h>

uint8_t EEMEM EEepochStartSeconds;
uint8_t EEMEM EEepochStartMinutes;
uint8_t EEMEM EEepochStartHours;
uint8_t EEMEM EEepochStartDays;

uint16_t EEMEM EEballastTargetAltitude;
int16_t EEMEM EEballastTargetPositiveVSpeed;
int16_t EEMEM EEballastTargetNegativeVSpeed;

uint16_t EEMEM EEmaydayAltitude;
int16_t EEMEM EEmaydayVSpeed;

uint16_t EEMEM EEballastSafetyAltThresh;
uint8_t EEMEM EEautoBallastDisable;

uint8_t EEMEM EEoverOceanFlag;

int8_t EEMEM EEnightTemperatureForecast;
uint32_t EEMEM EEsunriseAnticipation;

uint16_t EEMEM EEmaxAllowableTXInterval;

int8_t EEMEM EEbatteryHeaterSetpoint;

uint16_t EEMEM EEdataCollectionInterval;  //Var 0x03
uint16_t EEMEM EEdataTransmitInterval;  //Var 0x04
uint16_t EEMEM EEshortDataTransmitInterval;

uint16_t EEMEM EEhfDataTransmitInterval;
uint8_t EEMEM EEhfRapidTransmit;


uint32_t EEMEM EEepochOfLastBatchTransmit;

uint16_t EEMEM EEcurrentBatchNumber;
uint16_t EEMEM EEbatchSampleStart;
uint16_t EEMEM EEbatchSampleEnd;

uint16_t EEMEM EEcommPromStart;
uint16_t EEMEM EEcommPromEnd;

uint8_t EEMEM EEflightComputerResetCount;
uint8_t EEMEM EEcommModuleResetCount;

uint8_t EEMEM EEflightPhase;  //Var 0x05

uint32_t EEMEM EEcurrentTelemetryBitmap[3]; //Var 0x06
uint16_t EEMEM EEcurrentTelemetryVersion; //Var 0x07

uint8_t EEMEM EEEpochLock = 0;

uint16_t EEMEM EEhfTimeToTx;
uint8_t EEMEM EEhfLenngthToTx;

//THIS IS ALSO DEFINED IN MAIN, REMEMBER TO CHANGE THIS DUMMY
#define VSPEEDSAMPLESDESIRED 20

int16_t EEMEM EEvSpeedHolderSamples[VSPEEDSAMPLESDESIRED];

//Needed for BMP085
short EEMEM EEBMPac1;
short EEMEM EEBMPac2;
short EEMEM EEBMPac3;
unsigned short EEMEM EEBMPac4;
unsigned short EEMEM EEBMPac5;
unsigned short EEMEM EEBMPac6;
short EEMEM EEBMPb1;
short EEMEM EEBMPb2;
short EEMEM EEBMPmb;
short EEMEM EEBMPmc;
short EEMEM EEBMPmd;


#endif
