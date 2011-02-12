
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "../dataStructures.h"
//#include "../eepromVars.h"

#include "../progmemVars.h"
#include "../logging/openlog.h"

#include "outputParse.h"

#define opdebug

//if commented out, no eeprom writing
//#define detacheeprom


#define COMPROM 0b10100000


extern int lprintf(char *, ...);
extern uint16_t EEMEM EEcurrentTelemetryVersion;

uint16_t getTxSample(uint8_t *output, uint32_t *bitmask, uint16_t sampleNumber, uint16_t batch)
{

	uint32_t reversedBitmask[3];

	//uint32_t v;     // input bits to be reversed
	//uint32_t r = v; // r will be reversed bits of v; first get LSB of v

	for(int i=0; i < 3; i++)
	{
		uint32_t v = bitmask[i];
		uint32_t r = v;
		int s = sizeof(v) * 8 - 1; // extra shift needed at end
		for (v >>= 1; v; v >>= 1)
		{
		  r <<= 1;
		  r |= v & 1;
		  s--;
		}
		r <<= s; // shift when v's highest bits are zero
		reversedBitmask[i] = r;

	}
	//Bitmask reversal words

	char sampleHolder[SAMPLESTRINGSIZEINCHARS];

	getDataSample(sampleNumber, sampleHolder);
	//Sample Retrieval words
	#ifdef opdebug
		lprintf("ISAMP: %d\n", sampleNumber);
		lprintf("%s\n", sampleHolder);
	#endif

	uint8_t currentTelemetryChannel = 0;
	uint8_t bytesWritten = 0;
	char *token;


	//BEGIN SAMPLE PREAMBLE
	//First token is epoch time:
	token = strtok(sampleHolder, ",");
	uint32_t epochSample;
	sscanf(token, "%ld", &epochSample);

	output[0] = eeprom_read_word(&EEcurrentTelemetryVersion) >> 8;
	output[1] = eeprom_read_word(&EEcurrentTelemetryVersion);

	bytesWritten +=2;
	memcpy(output[2], bitmask, sizeof(uint32_t)*3);

	bytesWritten +=sizeof(uint32_t)*3;

	output[bytesWritten] = epochSample >> 24;
	bytesWritten++;
	output[bytesWritten] = epochSample >> 16;
	bytesWritten++;
	output[bytesWritten] = epochSample >> 8;
	bytesWritten++;
	output[bytesWritten] = epochSample;
	bytesWritten++;
	output[bytesWritten] = batch >> 8;
	bytesWritten++;
	output[bytesWritten] = batch;
	bytesWritten++;

	//END SAMPLE PREAMBLE

	do
	{
		token = strtok(NULL,",");

		lprintf("L: %d V: %lx\n", currentTelemetryChannel/32, reversedBitmask[currentTelemetryChannel/32] );
		if(reversedBitmask[currentTelemetryChannel/32] & 1 == 1)
		{
			//lprintf("Tof: %d\n", pgm_read_byte(&bitmaskTypeOrder[currentTelemetryChannel]));
			//lprintf("Tok: %s\n", token);
			switch(pgm_read_byte(&bitmaskTypeOrder[currentTelemetryChannel]))
			{
				uint16_t holder16;
				uint32_t holder32;
				float holderf;
				case 8:
					sscanf(token, "%d", &output[bytesWritten]);
					bytesWritten+=1;
					break;
				case 16:
					sscanf(token, "%d", &holder16);
					memrcpy(&output[bytesWritten], &holder16, sizeof(uint8_t)*2);
					bytesWritten+=2;
					break;
				case 24:
					sscanf(token, "%ld", &holder32);
					memrcpy(&output[bytesWritten], &holder32, sizeof(uint8_t)*3);
					bytesWritten+=3;
					break;
				case 32:
					sscanf(token, "%ld", &holder32);
					memrcpy(&output[bytesWritten], &holder32, sizeof(uint8_t)*3);
					bytesWritten+=4;
					break;
				case 33: //Float
					sscanf(token, "%f", &holderf);
					memrcpy(&output[bytesWritten], &holderf, sizeof(float));
					bytesWritten+=sizeof(float);
					break;
				case 34: //UTC Date or Time
					break;
				default:
					break;
			}
		}

		currentTelemetryChannel++;
		reversedBitmask[currentTelemetryChannel/32] >>= 1;
	} while(reversedBitmask[0] != 0 || reversedBitmask[1] != 0 || reversedBitmask[2] != 0);

	return bytesWritten;

}

//Before this function is called, CommPromEnd must be set to CommPromStart, or weird things could hapen.
void loadBatch(void)
{

	uint16_t batchSampleStart = eeprom_read_word(&EEbatchSampleStart);
	uint16_t batchSampleEnd = eeprom_read_word(&EEbatchSampleEnd);
	uint16_t batchNumber = eeprom_read_word(&EEcurrentBatchNumber);

	uint32_t currentBitmask[3];
	currentBitmask[0] = eeprom_read_dword(&EEcurrentTelemetryBitmap[0]);
	currentBitmask[1] = eeprom_read_dword(&EEcurrentTelemetryBitmap[1]);
	currentBitmask[2] = eeprom_read_dword(&EEcurrentTelemetryBitmap[2]);

	uint16_t commPromEnd = eeprom_read_word(&EEcommPromEnd);

	for(int i=batchSampleStart; i < batchSampleEnd; i++)
	{
		uint8_t thisSample[MAXTXSAMPLESIZE];
		uint8_t sizeOfSample;
		lprintf("i: %d\n", i);
		sizeOfSample = getTxSample(thisSample, currentBitmask, i, batchNumber);
		#ifdef opdebug
			lprintf("A Sample: ");
		#endif
		for(int j=0; j < sizeOfSample; j++)
		{
			#ifdef opdebug
				lprintf("%x", thisSample[j]);
			#endif
			uint8_t data[3];
			data[0] = commPromEnd >> 8;
			data[1] = commPromEnd;
			data[3] = thisSample;
			#ifdef detacheeprom
			i2cMasterSendNI(COMPROM, 3, &data);
			#endif
			commPromEnd++;
		}
		#ifdef opdebug
			lprintf("\n");
		#endif
	}

	batchNumber++;
	batchSampleStart = batchSampleEnd;

	eeprom_write_word(&EEbatchSampleStart, batchSampleStart);
	eeprom_write_word(&EEbatchSampleEnd, batchSampleEnd);
	eeprom_write_word(&EEcurrentBatchNumber, batchNumber);
	eeprom_write_word(&EEcommPromEnd, commPromEnd);

}

void flushSatQueue(void)
{

	uint16_t commPromEnd = eeprom_read_word(&EEcommPromEnd);
	uint16_t commPromStart = eeprom_read_word(&EEcommPromStart);
	//Send A message to comm module(CommEEPROMstart, CommEEPROMEnd)
	if(COMMPROMSIZE - commPromEnd < 1024)
		commPromStart = commPromEnd = 0;
	else
		commPromStart = commPromEnd;

	eeprom_write_word(&EEcommPromEnd, commPromEnd);
	eeprom_write_word(&EEcommPromStart, commPromStart);
}

void memrcpy(void *dst, const void *src, size_t len)
{
    size_t i;
    char* d = (char*)dst;
    const char* s = (const char*)src;

    for(i=0; i<len; ++i)
    {
        d[len-1-i] = s[i];
    }
}

