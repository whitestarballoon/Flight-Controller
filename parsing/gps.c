//*****************************************************************************
//
// File Name		: 'gps.c'
// Title			: GPS Parsing for NMEA Strings
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
#include <string.h>

#include <avr/pgmspace.h>
#include <avr/wdt.h>


#include "gps.h"

//$GPRMC,040302.663,A,3939.7,N,10506.6,W,0.27,358.86,200804,,*1A
//^Sentence             ^Lat(HHMM.M)      ^Speed (Knots)
//         ^HHMMSS.XXX          ^Lon(HHHMM.M)   ^Bearing
//                  ^GPS Fix (A=active, V=invalid)     ^UTC Date
void getGPS(struct gpsData *outputData)
{
	// Begin GPRMC Acquisition Section
	static char lineBuff[100];
	char tempChar;
	uint8_t i;
	uint8_t errorTracker;

	//Needed so the first comparison works out correctly
	memset(lineBuff, 0x00, 100);

	errorTracker = 0;


	//loads a line into a buffer, then checks the line for the GPS line
	//that we're looking for.  If it takes more than 100 lines to do this,
	//something is wrong.
	do
	{
		i=0;
		//i < 99 to leave room for the null terminator
		while(((tempChar = (char)uart_getchar()) != '\n') && (i < 99) && tempChar!= 0xff)
		{
			lineBuff[i] = tempChar;
			i++;
		}
		lineBuff[i] = '\0';
		errorTracker++;
	} while((strncmp("$GPRMC", lineBuff, 6) != 0) && errorTracker < 100);
	// End GPRMC Acquisition Section

	//lprintf("et: %d\n", errorTracker);
    wdt_reset();
	//If something is wrong, return before we try to parse the data.
	//Set the output status to "Loco".
	if(errorTracker >= 100)
	{
	    lprintf_P(PSTR("ERR\n"));
		outputData->status  = 3;
		return;
	}

	char targetChecksum[3];
	uint8_t  tSum;
	uint8_t checksum = 0;
	targetChecksum[0] = lineBuff[i-3];
	targetChecksum[1] = lineBuff[i-2];
	targetChecksum[2] = '\0';
	sscanf(targetChecksum, "%x", &tSum);
	for(uint8_t j = 1; j < i-4; j++)
	{
		 checksum = checksum ^ lineBuff[j];
	}

	if(checksum != tSum)
	{
		outputData->status = 2;
		return;
	}

	// Start GPRMC Parse Section
	char latdir;
	char londir;
	int latDegrees;
	float latMinutes;
	int lonDegrees;
	float lonMinutes;
	float speed, bearing;
	float time, altitude;
	char tempStatus;

	char localStatus;

	int error;

	error = sscanf( lineBuff,
			"$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f",
			&time, &tempStatus, &outputData->latitude, &latdir,
			&outputData->longitude, &londir, &speed, &bearing);


	outputData->hours = (uint8_t)(time/10000);
	outputData->minutes = (uint8_t)(time - (outputData->hours)*10000)/100;
	outputData->seconds = (uint8_t)(time - (outputData->hours)*10000 - (outputData->seconds)*100);

	if(latdir == 'S')
				outputData->latitude = 0-(outputData->latitude);
	if(londir == 'W')
				outputData->longitude = 0-(outputData->longitude);

	latDegrees = (int)(outputData->latitude/100);
	latMinutes = (float)(outputData->latitude - latDegrees*100);
	outputData->latitude = latDegrees + (latMinutes/60); //Conversion into decimal degrees

	lonDegrees = (int)(outputData->longitude/100);
	lonMinutes = (float)(outputData->longitude - lonDegrees*100);
	outputData->longitude = lonDegrees + (lonMinutes/60); //Conversion into decimal degrees

	outputData->speed = (uint8_t)speed;
	outputData->bearing = (uint16_t)bearing;
	// End GPRMC Parse Section



	// Begin GPGSA Acquisition Section

	//Needed so the first comparison works out correctly
	memset(lineBuff, 0x00, 100);

	errorTracker = 0;
	do
	{
		i=0;
		//i < 99 to leave room for the null terminator
		while(((tempChar = (char)uart_getchar()) != '\n') && i < 99)
		{
			lineBuff[i] = tempChar;
			i++;
		}
		errorTracker++;
		lineBuff[i] = '\0';
	} while((strncmp("$GPGSA", lineBuff, 6) != 0) && errorTracker < 100);
	// End GPGSA Acquisition Section
    wdt_reset();
	if(errorTracker >= 100)
	{
		outputData->status  = 3;
		return;
	}

	checksum = 0;
	targetChecksum[0] = lineBuff[i-3];
	targetChecksum[1] = lineBuff[i-2];
	targetChecksum[2] = '\0';
	sscanf(targetChecksum, "%x", &tSum);
	for(uint8_t j = 1; j < i-4; j++)
	{
		 checksum = checksum ^ lineBuff[j];
	}

	if(checksum != tSum)
	{
		outputData->status = 2;
		return;
	}


	float tempVdop, tempHdop;
	// Start GPGSA Parse Section

	char *token, *lastToken, *beforeThat;
	token = strtok(lineBuff, ",");
	while(token != NULL)
	{
		beforeThat = lastToken;
		lastToken = token;
		token = strtok(NULL,",");
	}
	sscanf(beforeThat, "%f", &tempHdop);
	sscanf(lastToken, "%f*", &tempVdop);

	outputData->hdop = (uint16_t)(tempHdop*10);
	outputData->vdop = (uint16_t)(tempVdop*10);
	// End GPGSA Parse Section

	// Begin GPGGA Acquisition Section

	//Needed so the first comparison works out correctly
	memset(lineBuff, 0x00, 100);

	errorTracker = 0;
	do
	{
		i=0;
		//i < 99 to leave room for the null terminator
		while(((tempChar = (char)uart_getchar()) != '\n') && (i < 99) && tempChar!= 0xff)
		{
			lineBuff[i] = tempChar;
			i++;
		}
		lineBuff[i] = '\0';
		errorTracker++;
	} while((strncmp("$GPGGA", lineBuff, 6) != 0) && errorTracker < 100);
	// End GPGGA Acquisition Section
    wdt_reset();
	if(errorTracker >= 100)
	{
		outputData->status  = 3;
		return;
	}

	checksum = 0;
	targetChecksum[0] = lineBuff[i-3];
	targetChecksum[1] = lineBuff[i-2];
	targetChecksum[2] = '\0';
	sscanf(targetChecksum, "%x", &tSum);
	for(uint8_t j = 1; j < i-4; j++)
	{
		 checksum = checksum ^ lineBuff[j];
	}

	if(checksum != tSum)
	{
		outputData->status = 2;
		return;
	}

	// Start GPGGA Parse Section



	sscanf( lineBuff,
			"$GPGGA,%*f,%*f,%*c,%*f,%*c,%*d,%d,%*f,%f",
			&outputData->numberOfSats, &altitude );

	outputData->altitude = (uint16_t)altitude;
	// End GPGGA Parse Section

	switch(tempStatus)
	{
		case 'A':
			outputData->status = 0;
			break;
		case 'V':
			outputData->status = 1;
			break;
	}

}

//REMOVE BEFORE FLIGHT
void debugPrintRawStrings(void)
{

	static char lineBuff[100];
	char tempChar;
	uint8_t i;
	uint8_t errorTracker;
	//Needed so the first comparison works out correctly
	memset(lineBuff, 0x00, 100);

	lprintf_P(PSTR("In Ur GPS Debug\n"));

	do
	{
		i=0;
		//i < 99 to leave room for the null terminator
		while(((tempChar = (char)uart_getchar()) != '\n') && (i < 99) && tempChar!= 0xff)
		{
			lineBuff[i] = tempChar;
			i++;
		}
		lineBuff[i] = '\0';
		errorTracker++;
	} while((strncmp("$GPRMC", lineBuff, 6) != 0));



	char targetChecksum[3];
	uint8_t  tSum;
	uint8_t checksum = 0;
	targetChecksum[0] = lineBuff[i-3];
	targetChecksum[1] = lineBuff[i-2];
	targetChecksum[2] = '\0';
	sscanf(targetChecksum, "%x", &tSum);
	for(uint8_t j = 1; j < i-4; j++)
	{
		 checksum = checksum ^ lineBuff[j];
	}

	lprintf_P(PSTR("CS: %x TCS: %x\n"), checksum, tSum);

	for(uint8_t j = 0; j < i; j++)
	{
		lprintf("%c", lineBuff[j]);
	}
	if(tSum == checksum)
	{
		lprintf_P(PSTR("Checksum Valid\n"));
	}

	memset(lineBuff, 0x00, 100);

	do
	{
		i=0;
		//i < 99 to leave room for the null terminator
		while(((tempChar = (char)uart_getchar()) != '\n') && i < 99)
		{
			lineBuff[i] = tempChar;
			i++;
		}
		lineBuff[i] = '\0';
	} while((strncmp("$GPGSA", lineBuff, 6) != 0));
	// End GPGSA Acquisition Section

	checksum = 0;
	targetChecksum[0] = lineBuff[i-3];
	targetChecksum[1] = lineBuff[i-2];
	targetChecksum[2] = '\0';
	sscanf(targetChecksum, "%x", &tSum);
	for(uint8_t j = 1; j < i-4; j++)
	{
		 checksum = checksum ^ lineBuff[j];
	}

	lprintf_P(PSTR("CS: %x TCS: %x\n"), checksum, tSum);

	for(uint8_t j = 0; j < i; j++)
	{
		lprintf("%c", lineBuff[j]);
	}
	if(tSum == checksum)
	{
		lprintf_P(PSTR("Checksum Valid\n"));
	}

	memset(lineBuff, 0x00, 100);
	do
	{
		i=0;
		//i < 99 to leave room for the null terminator
		while(((tempChar = (char)uart_getchar()) != '\n') && i < 99)
		{
			lineBuff[i] = tempChar;
			i++;
		}
		lineBuff[i] = '\0';
	} while((strncmp("$GPGGA", lineBuff, 6) != 0));
	// End GPGGA Acquisition Section

	checksum = 0;
	targetChecksum[0] = lineBuff[i-3];
	targetChecksum[1] = lineBuff[i-2];
	targetChecksum[2] = '\0';
	sscanf(targetChecksum, "%x", &tSum);
	for(uint8_t j = 1; j < i-4; j++)
	{
		 checksum = checksum ^ lineBuff[j];
	}

	lprintf_P(PSTR("CS: %x TCS: %x\n"), checksum, tSum);

	for(uint8_t j = 0; j < i; j++)
	{
		lprintf("%c", lineBuff[j]);
	}
	if(tSum == checksum)
	{
		lprintf_P(PSTR("Checksum Valid\n"));
	}

}
