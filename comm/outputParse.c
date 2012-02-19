
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "../dataStructures.h"
//#include "../eepromVars.h"

#include "../progmemVars.h"
#include "../logging/openlog.h"

#include "../lib/i2c.h"

#include "outputParse.h"

#define opdebug

//if commented out, no eeprom writing
#define detacheeprom


#define COMPROM 0b10100000


extern int lprintf(char *, ...);
extern int lprintf_P(const char *str, ...);
extern uint16_t EEMEM EEcurrentTelemetryVersion;
extern uint8_t i2cMasterSendNI(uint8_t, uint8_t, uint8_t*);
extern void i2cSendStart(void);
extern uint8_t i2cWaitForComplete(void);
extern void i2cSendByte(uint8_t);
extern void i2cSendStop(void);

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

	char sampleHolder[SAMPLESTRINGSIZEINCHARS+5];

    memset(sampleHolder, 0, SAMPLESTRINGSIZEINCHARS+5);
	getDataSample(sampleNumber, sampleHolder);
	//Sample Retrieval words
	#ifdef opdebug
		lprintf("ISAMP: %d\n", sampleNumber);
        for(int i = 0; i < SAMPLESTRINGSIZEINCHARS; i++)
        {
            lprintf("%c", sampleHolder[i]);
        }
		lprintf("\n");
	#endif

	//lprintf("done\n");

	uint8_t currentTelemetryChannel = 0;
	uint8_t bytesWritten = 0;
	char *token;


	//BEGIN SAMPLE PREAMBLE
	//First token is epoch time:
	token = strtok(sampleHolder, ",");
	uint32_t epochSample;
	sscanf(token, "%ld", &epochSample);

	//lprintf("\nep: %s\n", token);

	output[0] = eeprom_read_word(&EEcurrentTelemetryVersion) >> 8;
	output[1] = eeprom_read_word(&EEcurrentTelemetryVersion);

	bytesWritten +=2;

    output[2] = bitmask[0] >> 24;
    output[3] = bitmask[0] >> 16;
    output[4] = bitmask[0] >> 8;
    output[5] = bitmask[0];
    output[6] = bitmask[1] >> 24;
    output[7] = bitmask[1] >> 16;
    output[8] = bitmask[1] >> 8;
    output[9] = bitmask[1];
    output[10] = bitmask[2] >> 24;
    output[11] = bitmask[2] >> 16;
    output[12] = bitmask[2] >> 8;
    output[13] = bitmask[2];


	bytesWritten +=sizeof(uint32_t)*3;



	output[bytesWritten] = epochSample >> 24;
	//lprintf("%x ", output[bytesWritten]);
	bytesWritten++;
	output[bytesWritten] = epochSample >> 16;
	//lprintf("%x ", output[bytesWritten]);
	bytesWritten++;
	output[bytesWritten] = epochSample >> 8;
	//lprintf("%x ", output[bytesWritten]);
	bytesWritten++;
	output[bytesWritten] = epochSample;
	//lprintf("%x\n", output[bytesWritten]);
	bytesWritten++;
	output[bytesWritten] = batch >> 8;
	bytesWritten++;
	output[bytesWritten] = batch;
	bytesWritten++;

	//END SAMPLE PREAMBLE

	do
	{
		token = strtok(NULL,",");

		//lprintf("L: %d V: %lx\n", currentTelemetryChannel/32, reversedBitmask[currentTelemetryChannel/32] );
        //lprintf("P: %lx\n", reversedBitmask[0]);
        //lprintf("P: %lx\n", reversedBitmask[1]);
        //lprintf("P: %lx\n", reversedBitmask[2]);
		if((reversedBitmask[currentTelemetryChannel/32] & 1) == 1)
		{
			//lprintf("Tof: %d\n", pgm_read_byte(&bitmaskTypeOrder[currentTelemetryChannel]));
			//lprintf("Tok: %s\n", token);
			switch(pgm_read_byte(&bitmaskTypeOrder[currentTelemetryChannel]))
			{
				uint16_t holder16;
				uint32_t holder32;
				float holderf;
				case 8:
					sscanf(token, "%d", (int *)&output[bytesWritten]);
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

        reversedBitmask[currentTelemetryChannel/32] >>= 1;
		currentTelemetryChannel++;

	} while(reversedBitmask[0] != 0 || reversedBitmask[1] != 0 || reversedBitmask[2] != 0);

	return bytesWritten;

}

//Before this function is called, CommPromEnd must be set to CommPromStart, or weird things could hapen.
void loadBatch(void)
{

    //lprintf_P(PSTR("LB\n"));
	uint16_t batchSampleStart = eeprom_read_word(&EEbatchSampleStart);
	uint16_t batchSampleEnd = eeprom_read_word(&EEbatchSampleEnd);
	uint16_t batchNumber = eeprom_read_word(&EEcurrentBatchNumber);

	uint32_t currentBitmask[3];
	currentBitmask[0] = eeprom_read_dword(&EEcurrentTelemetryBitmap[0]);
	currentBitmask[1] = eeprom_read_dword(&EEcurrentTelemetryBitmap[1]);
	currentBitmask[2] = eeprom_read_dword(&EEcurrentTelemetryBitmap[2]);

	uint16_t commPromEnd = eeprom_read_word(&EEcommPromEnd);

	for(uint16_t i=batchSampleStart; i < batchSampleEnd; i++)
	{
		uint8_t thisSample[MAXTXSAMPLESIZE];
		uint8_t sizeOfSample;
		memset(thisSample, 0x00, MAXTXSAMPLESIZE);
		sizeOfSample = getTxSample(thisSample, currentBitmask, i, batchNumber);

		#ifdef opdebug
			lprintf("A Sample: ");
		#endif
		for(int j=0; j < sizeOfSample; j++)
		{
			#ifdef opdebug
				lprintf("%x ", thisSample[j]);
			#endif
			uint8_t data[3];
			uint8_t error;
			data[0] = commPromEnd >> 8;
			data[1] = commPromEnd;
			data[2] = thisSample[j];
			#ifdef detacheeprom
			while((error = i2cMasterSendNI(COMPROM, 3, data)) != I2C_OK);
			#endif
            eeprom_write_word(&EEbatchSampleStart, ++batchSampleStart);
            wdt_reset();
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

    #ifdef opdebug
    lprintf_P(PSTR("Sending Long Report\n"));
    #endif

    uint8_t cps1 = commPromStart >> 8;
    uint8_t cps2 = commPromStart;
    uint8_t cpe1 = commPromEnd >> 8;
    uint8_t cpe2 = commPromEnd;

    #ifdef opdebug
        lprintf_P(PSTR("CPS: %x %x CPE: %x %x\n"), cps1, cps2, cpe1, cpe2);
    #endif

	i2cSendStart();
    i2cWaitForComplete();
    i2cSendByte(0x10);
    i2cWaitForComplete();
    i2cSendByte(0x01);
    i2cWaitForComplete();

    i2cSendByte(cps1);
    i2cWaitForComplete();
    i2cSendByte(cps2);
    i2cWaitForComplete();
    i2cSendByte(cpe1);
    i2cWaitForComplete();
    i2cSendByte(cpe2);
    i2cWaitForComplete();

    i2cSendStop();

    #ifdef opdebug
    lprintf_P(PSTR("Done w/Long Report\n"));
    #endif

	if((COMMPROMSIZE - commPromEnd) < 1024)
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

