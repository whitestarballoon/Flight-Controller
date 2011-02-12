//*****************************************************************************
//
// File Name		: 'dataStructuures.h'
// Title			: Data Structures for Balloon project
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

#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#define SAMPLESTRINGSIZEINCHARS 225  //Size of a sample string as stored in the open log
								   //In chars (or bytes), including the terminating \r\n

#define MAXTXSAMPLESIZE 150

#define COMMPROMSIZE 32768

typedef void (*scheduledFunction)(uint32_t);

struct dataSample
{
	uint32_t epochSampled;
	uint16_t flightComputerTemperature;
	uint32_t barometricPressure;
	//struct gpsData sampleGPS;
	
} dataSample;

struct gpsData
{
	uint8_t status;
	float latitude;
	float longitude;
	uint8_t speed;
	uint16_t bearing;
	uint16_t hdop;
	uint16_t vdop;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint16_t altitude;
	uint8_t numberOfSats;
	
} gpsData;


#endif
