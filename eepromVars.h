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

#ifndef EEPROMVARS_Hf
#define EEPROMVARS_H

#include <avr/eeprom.h>

extern uint8_t EEMEM EEepochStartSeconds;
extern uint8_t EEMEM EEepochStartMinutes;
extern uint8_t EEMEM EEepochStartHours;
extern uint8_t EEMEM EEepochStartDays;

extern uint16_t EEMEM EEballastTargetAltitude;
extern int16_t EEMEM EEballastTargetPositiveVSpeed;
extern int16_t EEMEM EEballastTargetNegativeVSpeed;

extern uint16_t EEMEM EEmaydayAltitude;
extern int16_t EEMEM EEmaydayVSpeed;

extern uint16_t EEMEM EEballastSafetyAltThresh;
extern uint8_t EEMEM EEautoBallastDisable;

extern uint8_t EEMEM EEoverOceanFlag;

extern int8_t EEMEM EEnightTemperatureForecast;
extern uint32_t EEMEM EEsunriseAnticipation;

extern uint16_t EEMEM EEmaxAllowableTXInterval;

extern int8_t EEMEM EEbatteryHeaterSetpoint;

extern uint16_t EEMEM EEdataCollectionInterval;  //Var 0x03
extern uint16_t EEMEM EEdataTransmitInterval;  //Var 0x04
extern uint16_t EEMEM EEshortDataTransmitInterval;

extern uint16_t EEMEM EEhfDataTransmitInterval;
extern uint8_t EEMEM EEhfRapidTransmit;


extern uint32_t EEMEM EEepochOfLastBatchTransmit;

extern uint16_t EEMEM EEcurrentBatchNumber;
extern uint16_t EEMEM EEbatchSampleStart;
extern uint16_t EEMEM EEbatchSampleEnd;

extern uint16_t EEMEM EEcommPromStart;
extern uint16_t EEMEM EEcommPromEnd;

extern uint8_t EEMEM EEflightComputerResetCount;
extern uint8_t EEMEM EEcommModuleResetCount;

extern uint8_t EEMEM EEflightPhase;  //Var 0x05

extern uint32_t EEMEM EEcurrentTelemetryBitmap[3]; //Var 0x06
extern uint16_t EEMEM EEcurrentTelemetryVersion; //Var 0x07

extern uint8_t EEMEM EEEpochLock;

extern uint16_t EEMEM EEhfTimeToTx;
extern uint8_t EEMEM EEhfLenngthToTx;

extern uint32_t EEMEM EEepochOffset;

//THIS IS ALSO DEFINED IN MAIN, REMEMBER TO CHANGE THIS DUMMY
#define VSPEEDSAMPLESDESIRED 20

extern int16_t EEMEM EEvSpeedHolderSamples[VSPEEDSAMPLESDESIRED];

//Needed for BMP085
extern short EEMEM EEBMPac1;
extern short EEMEM EEBMPac2;
extern short EEMEM EEBMPac3;
extern unsigned short EEMEM EEBMPac4;
extern unsigned short EEMEM EEBMPac5;
extern unsigned short EEMEM EEBMPac6;
extern short EEMEM EEBMPb1;
extern short EEMEM EEBMPb2;
extern short EEMEM EEBMPmb;
extern short EEMEM EEBMPmc;
extern short EEMEM EEBMPmd;


#endif
