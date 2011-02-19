//*****************************************************************************
//
// File Name		: 'main.c'
// Title			: White Star Balloon Speedball-1 Flight Software
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
//*****************************************************************************
//
// TODO: Before flight, have at least 2 people go through this code with a fine
//       tooth comb to root out any "remove or change before flight" comments.
//
//*****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "lib/i2c.h"
#include "lib/tmp100.h"
#include "lib/bmp085.h"
#include "lib/ds3231.h"

#include "parsing/epoch.h"
#include "parsing/gps.h"

#include "comm/outputParse.h"

#include "logging/openlog.h"

//#include "eepromVars.h"
#include "queue.h"
#include "dataStructures.h"

#define MYUBRR 12  //57600 baud

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

//LEDs on PB3 and PB4
#define STATUS_PORT PORTB
#define STATUS_LED1 3
#define STATUS_LED2 4

#define yellow_on()  ( PORTB |= (1 << STATUS_LED1)  )
#define red_on()     ( PORTB |= (1 << STATUS_LED2)  )

#define yellow_off() ( PORTB &= ~(1 << STATUS_LED1) )
#define red_off()    ( PORTB &= ~(1 << STATUS_LED2) )

#define XCLR 4
#define XCLR_P PORTA

#define TAKEOFFPINPORT PORTD
#define TAKEOFFPIN 5

#define TMP100FC 0b10011110
#define TMP101BH 0b10010010
#define TMP100EXT 0b10010110
#define BALLASTMODULE 0b00010010
#define AD7992 0x40
#define INTSENSOR 0x14

#define GSPDEBUG
//#define FCPUDEBUG
//#define OSHITDISABLE

#define CRITCOMFAIL 25

#define CRITGPSFAIL 20


//Define functions
//======================
uint32_t now(void);
void ioinit(void);      // initializes IO
static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);
int lprintf(char *, ...);
int lprintf_P(const char *str, ...);
void receiveCommandHandler(uint8_t receiveDataLength, uint8_t* recieveData);

//Test Vars
void dumpVarsToGSP(void);
void bmpTest(void);
void dumpTemps(void);
void debugBallast(void);
void dumpGPS(void);

//DEFINE SCHEDULED THINGS
void rapidHFXmit(uint32_t);
void timedCutdown(uint32_t);
void processMonitor(uint32_t);
void calculateVspeed(uint32_t);
void timedCutdown(uint32_t);
void autoBallast(uint32_t);
void collectData(uint32_t);
void transmitSamples(uint32_t);
void transmitShortReport(uint32_t);
void updateCommHFTelemetry(uint32_t);
void rapidHFXmit(uint32_t);
void ballastStaticTickle(uint32_t);
void flightPhaseLogic(uint32_t);

//Vspeed Calculation Variables
int16_t vSpeedAvg;
uint8_t numberOfVSpeedSamples=0;
uint16_t lastAltitude=0;
uint16_t lastRunTime=0;
extern int16_t EEMEM EEvSpeedHolderSamples;

//Globals
uint8_t enableReports = 1;
uint8_t reportCounterL=0;
uint8_t reportCounterH=0;
uint16_t statusCode = 0x00;

//HOLY CRAP YOU ARE STUPID PUT THESE SOMEWHERE THAT MAKES SENSE SO YOU DON'T HAVE TO EXTERN THEM
//IN YOUR FREAKING MAIN ROUTINE
extern uint16_t EEMEM EEbatchSampleEnd;
extern int16_t EEMEM EEballastTargetPositiveVSpeed;
extern int16_t EEMEM EEballastTargetNegativeVSpeed;
extern uint16_t EEMEM EEballastTargetAltitude;
extern uint16_t EEMEM EEballastSafetyAltThresh;
extern uint16_t EEMEM EEmaxAllowableTXInterval;
extern uint16_t EEMEM EEdataTransmitInterval;
extern uint16_t EEMEM EEshortDataTransmitInterval;
extern uint8_t EEMEM EEhfRapidTransmit;
extern uint8_t EEMEM EEflightPhase;
extern uint16_t EEMEM EEdataCollectionInterval;
extern uint16_t EEMEM EEmaydayAltitude;
extern int16_t EEMEM EEmaydayVSpeed;
extern uint8_t EEMEM EEoverOceanFlag;
extern uint32_t EEMEM EEsunriseAnticipation;
extern int8_t EEMEM EEnightTemperatureForecast;
extern uint16_t EEMEM EEhfDataTransmitInterval;
extern uint8_t EEMEM EEautoBallastDisable;
extern int8_t EEMEM EEbatteryHeaterSetpoint;
extern uint16_t EEMEM EEcurrentTelemetryVersion;
extern uint8_t EEMEM EEcommModuleResetCount;
extern uint8_t EEMEM EEflightComputerResetCount;
extern uint32_t EEMEM EEepochOfLastBatchTransmit;
extern uint8_t EEMEM EEepochStartSeconds;
extern uint8_t EEMEM EEepochStartMinutes;
extern uint8_t EEMEM EEepochStartHours;
extern uint8_t EEMEM EEepochStartDays;
extern uint8_t EEMEM EEEpochLock;

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
//======================



int main (void)
{
	uint8_t seconds,  minutes, hours, days;
	uint8_t error;

	ioinit(); //Setup IO pins and defaults
	i2cSetTheDamnTWBRMyself(10);

	i2cSetLocalDeviceAddr(0b00001010, 0x00, 0, 0, 0);
	i2cSetSlaveReceiveHandler(receiveCommandHandler);
	i2cEnableInt();
	sei();

	_delay_ms(500);

	lprintf("WSB CPU Alive\n");

	error = getTime(&seconds, &minutes, &hours, &days);
	lprintf("E: %d\n", error);
	lprintf("S: %d M: %d H: %d D: %d\n", seconds, minutes, hours, days);

	if(eeprom_read_byte(&EEEpochLock) == 0)
	{
		#ifdef FCPUDEBUG
			lprintf_P(PSTR("Setting Epoch Start\n"));
		#endif
		writeEpochStart(seconds, minutes, hours, days);
	}

	lprintf("Still Alive\n");

	// TMP100
	setTMP100config(TMP100FC, 0xE0);
	//uint8_t tmpConfig = getTMP100config(TMP100FC);
	// TMP100 END

	// Configure TMP101
	//Set Config 00000000
	/*setTMP100config(TMP101BH, 0x00);
	setTMP101Thermo(TMP101BH, set12bit2scomp(eeprom_read_byte(&EEbatteryHeaterSetpoint))*16, 0);
	setTMP101Thermo(TMP101BH, set12bit2scomp(eeprom_read_byte(&EEbatteryHeaterSetpoint))*16+32, 1);*/
	// TMP101 END

	// BMP085
	//long temperature = 0;
	//long pressure = 0;
	BMP085_Calibration();
	// BMP085 END
	//defaultEEPROM();
	if(eeprom_read_byte(&EEEpochLock) == 0)
	{
		defaultEEPROM();
        initOpenLogTest();
	} else {
		initOpenLogFlight();
	}


	lprintf("Still Still Alive\n");

	//lprintf("psamp\n");
	//Test Routine for data Storage and Retreival
	/*char sample1[SAMPLESTRINGSIZEINCHARS]= "0,583,0,72,0,+18.24";
	char sample2[SAMPLESTRINGSIZEINCHARS]= "0,282,0,80,0,+18.90";
	char sample3[SAMPLESTRINGSIZEINCHARS]= "0,163,0,04,0,-24.20";

	putDataSample(sample1);
	eeprom_write_word(&EEbatchSampleEnd, eeprom_read_word(&EEbatchSampleEnd)+1);

	putDataSample(sample2);
	eeprom_write_word(&EEbatchSampleEnd, eeprom_read_word(&EEbatchSampleEnd)+1);

	putDataSample(sample3);
	eeprom_write_word(&EEbatchSampleEnd, eeprom_read_word(&EEbatchSampleEnd)+1);

	lprintf("SampleStart: %d\n", eeprom_read_word(&EEbatchSampleStart));
	lprintf("SampleEnd: %d\n", eeprom_read_word(&EEbatchSampleEnd));


	loadBatch();
	*/

	/*while(1)
	{
		struct gpsData testStruct;
		getGPS(&testStruct);
		_delay_ms(500);
		lprintf("stat: %c sats: %d\n", testStruct.status, testStruct.numberOfSats);
		lprintf("lat: %f lon: %f\n", testStruct.latitude, testStruct.longitude);
		lprintf("now: %ld\n", now());

	}*/

	uint32_t rnow = now();
	scheduleQueueAdd(&processMonitor, rnow);
	scheduleQueueAdd(&calculateVspeed, rnow);
	scheduleQueueAdd(&collectData, rnow);
	scheduleQueueAdd(&transmitSamples, rnow);
	//scheduleQueueAdd(&transmitShortReport, rnow);
	scheduleQueueAdd(&updateCommHFTelemetry, rnow);
	scheduleQueueAdd(&ballastStaticTickle, rnow);
	scheduleQueueAdd(&autoBallast, rnow);
	scheduleQueueAdd(&flightPhaseLogic, rnow);
	while(1)
	{
		uint32_t scheduleTime;
		scheduledFunction thisFunction;
		volatile int8_t error;
		volatile uint32_t rightNow = now();

		error = scheduleQueueGetTop(&thisFunction, &scheduleTime);
		//lprintf("PTR: %p time: %lud now: %lud\n", ptrToFunction, scheduleTime, rightNow);

		if(error == 0 && scheduleTime <= rightNow)
		{
			/*#ifdef FCPUDEBUG
				lprintf_P(PSTR("Running some function\n"));
			#endif*/
			thisFunction(rightNow);
		} else if (error == 0 && scheduleTime > rightNow)
		{
			/*#ifdef FCPUDEBUG
				lprintf_P(PSTR("ReScheduling some function\n"));
			#endif*/
			scheduleQueueAdd(thisFunction, scheduleTime);
		} else {
			//Error!
		}
		_delay_ms(50);
	}

	while(1)
	{
		red_on();
		_delay_ms(500);
		red_off();
		_delay_ms(500);
	}


    return(0);
}

uint8_t cutdownStatus = 0;
uint8_t rapidHFEnable=0;
void receiveCommandHandler(uint8_t receiveDataLength, uint8_t* recieveData)
{
    uint8_t temp;
	switch(recieveData[0])
	{
		case 0x00:
			//verify we have received the right data
			//if(receiveDataLength == 2)
			{
				eeprom_write_byte(&EEflightPhase, recieveData[1]);
			}
			break;
		case 0x01:
			//ballast altitude target
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
				eeprom_write_word(&EEballastTargetAltitude, holder);
			}
			break;
		case 0x02:
			//Ballast Positive Vertical Speed Target
			//if(receiveDataLength == 3)
			{
				int16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
				eeprom_write_word(&EEballastTargetPositiveVSpeed, holder);
			}
			break;
		case 0x03:
			//set Over Ocean Flag
			//if(receiveDataLength == 2)
			{
				eeprom_write_byte(&EEoverOceanFlag, recieveData[1]);
			}
			break;
		case 0x04:
			//set Sunrise Anticipation Time
			//if(receiveDataLength == 5)
			{
				uint32_t holder = ((uint32_t)recieveData[0] << 24) + ((uint32_t)recieveData[1] << 16) + ((uint32_t)recieveData[2] << 8) + recieveData[3];
				eeprom_write_dword(&EEsunriseAnticipation, holder);
			}
			break;
		case 0x05:
			//set Night Temperature Forecast
			//if(receiveDataLength == 2)
			{
				eeprom_write_byte(&EEnightTemperatureForecast, recieveData[1]);
			}
			break;
		case 0x06:
			//set Data Sample Interval
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
				eeprom_write_word(&EEdataCollectionInterval, holder);
			}
			break;
		case 0x07:
			//set Data Transmit Interval
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
				eeprom_write_word(&EEdataTransmitInterval, holder);
			}
			break;
		case 0x08:
			//set Predefined Bitmask Select
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
				eeprom_write_word(&EEcurrentTelemetryVersion, holder);
			}
			break;
		case 0x09:
			//set Custom Bitmask Select
			//if(receiveDataLength == 13)
			{
				eeprom_write_block((uint16_t*)recieveData[1], &EEcurrentTelemetryBitmap, sizeof(uint32_t)*3);
			}
			break;
		case 0x0A:
			//set Max Allowable TX Interval
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1] << 8) + recieveData[2];
				eeprom_write_word(&EEmaxAllowableTXInterval, holder);
			}
			break;
		case 0x0B:
			//set Battery Heater Setpoint
			//if(receiveDataLength == 2)
			{
				uint8_t error;
				eeprom_write_byte(&EEbatteryHeaterSetpoint, recieveData[1]);
				error |= setTMP101Thermo(TMP101BH, set12bit2scomp(eeprom_read_byte(&EEbatteryHeaterSetpoint))*16, 0);
				error |= setTMP101Thermo(TMP101BH, set12bit2scomp(eeprom_read_byte(&EEbatteryHeaterSetpoint))*16+32, 1);
				#ifdef GSPDEBUG
					if(error != 0)
					{
						lprintf_P(PSTR("Error Setting thermometer\n"));
					}
				#endif
				//Should probably set a status code here  BEFORE FLIGHT
			}
			break;
		case 0x0C:
			//BEFORE FLIGHT ADD STUFF HERE TO TALK TO STROBE
			break;
		case 0x0D:
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1] << 8) + recieveData[2];
				eeprom_write_word(&EEballastSafetyAltThresh, holder);
			}
			break;
		case 0x0E:
			//set HF Transmit Interval
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1] << 8) + recieveData[2];
				eeprom_write_word(&EEhfDataTransmitInterval, holder);
			}
			break;
		case 0x0F:
			//if(receiveDataLength == 1)
			{
				cutdownStatus= 1;
				//enable rapid hf xmit
				rapidHFEnable = 1;
				//schedule rapid hf xmit
				scheduleQueueAdd(&rapidHFXmit, now());
			}
			break;
		case 0x10:
			//if(receiveDataLength == 5)
			{
				//It Looks Like You're trying to Schedule a cutdown!
				uint32_t time = ((uint32_t)recieveData[0] << 24) + ((uint32_t)recieveData[1] << 16) + ((uint32_t)recieveData[2] << 8) + recieveData[3];
				scheduleQueueAdd(&timedCutdown, time);
			}
			break;
		case 0x11:
			//if(receiveDataLength == 3)
			{
				//Send Ballast Controller the ballast command
			}
			break;
		case 0x12:
			//set Disarm Auto Ballast
			lprintf_P(PSTR("bdis\n"));
			temp = eeprom_read_byte(&EEautoBallastDisable);
            lprintf("Vb: %d\n", temp);
			lprintf_P(PSTR("bptr %p\n"), &EEautoBallastDisable);
            eeprom_write_byte(&EEautoBallastDisable, 1);
            temp = eeprom_read_byte(&EEautoBallastDisable);
            lprintf("V: %d\n", temp);
			break;
		case 0x13:
			//if(receiveDataLength == 1)
			{
				eeprom_write_byte(&EEautoBallastDisable, 0);
			}
			break;
		case 0x14:
			//set mayday vspeed threshold
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1] << 8) + recieveData[2];
				eeprom_write_word(&EEmaydayVSpeed, holder);
			}
			break;
		case 0x15:
			//set mayday altitude Thresh
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1] << 8) + recieveData[2];
				eeprom_write_word(&EEmaydayAltitude, holder);
			}
			break;
		case 0x16:
			//set Static Ballast Tickle Mass
			//DEPRECATED
			break;
		case 0x17:
			//set Set Rapid HF Transmit Period
			//if(receiveDataLength == 2)
			{
				eeprom_write_byte(&EEhfRapidTransmit, recieveData[1]);
			}
			break;
		case 0x18:
			//if(receiveDataLength == 3)
			{
				int16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
				eeprom_write_word(&EEballastTargetNegativeVSpeed, holder);
			}
			break;
		case 0x19:
			enableReports = 1;
			break;
		case 0x1A:
			enableReports = 0;
			break;
        case 0x1B:
            {
                int16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
                eeprom_write_word(&EEshortDataTransmitInterval, holder);
            }
        break;
		case 0xF7:
			eeprom_write_byte(&EEEpochLock, recieveData[1]);
			break;
		case 0xF8:
			dumpGPS();
			break;
		case 0xF9:
			debugBallast();
			break;
		case 0xFA:
			debugPrintRawStrings();
			break;
		case 0xFB:
			//lprintf_P(PSTR("Defaulting the EEPROM...\n"));
			defaultEEPROM();
			break;
		case 0xFC:
			collectData(0xFFFFFFFF);
			break;
		case 0xFD:
			dumpTemps();
			break;
		case 0xFE:
			bmpTest();
			break;
		case 0xFF:
			dumpVarsToGSP();
			break;
	}
}

void debugBallast(void)
{
	lprintf_P(PSTR("testing ballast\n"));

	uint8_t data = 19;
	uint8_t retVal;
	while( (retVal = i2cMasterSendNI(BALLASTMODULE, 1, &data)) != I2C_OK)
	{
		_delay_ms(500);
	}
	_delay_ms(1000);
	lprintf(PSTR("turned ballast on.\n"));
	_delay_ms(10000);
	lprintf("Done waiting\n");
	data = 18;
	while( (retVal = i2cMasterSendNI(BALLASTMODULE, 1, &data)) != I2C_OK)
	{
		_delay_ms(500);
	}
	lprintf("out\n");
}

void bmpTest(void)
{
	long myPressure;
	long myTemp;
	bmp085Convert(&myTemp, &myPressure);

	//lprintf_P(PSTR("BMP085 Temp: %ld Pressure %ld\n"), myTemp, myPressure);
	lprintf_P(PSTR("BMP T: %ld\n"), myTemp);
	lprintf_P(PSTR("BMP P: %ld\n"), myPressure);
	lprintf("Test\n");
}

void dumpTemps(void)
{
	//get FC temp
	uint16_t rawFCTemp;
	int8_t internalTemp;
	rawFCTemp = tmp100rawTemp(TMP100FC)>>4;
	int16_t fctinm = get12bit2scomp(rawFCTemp);
	internalTemp = (int8_t)(fctinm/16);
	//Convert to 8 bit

	//get ext temp
	uint16_t rawExtTemp = tmp100rawTemp(TMP100EXT)>>4;
	int16_t externalTemperature = get12bit2scomp(rawExtTemp);

	//get battetry temp
	uint16_t rawBattTemp = tmp100rawTemp(TMP101BH)>>4;
	int16_t btinm = get12bit2scomp(rawBattTemp);
	int8_t batteryTemperature = (int8_t)(btinm/16);
	//conver to 8 bit

	if(rawFCTemp != 0xEFF)
	{
		lprintf_P(PSTR("FC: Raw: %x Calc: %d\n"), rawFCTemp, internalTemp);
	} else {
		lprintf_P(PSTR("Error Reading FC Temp\n"));
	}

	if(rawExtTemp != 0xEFF)
	{
		lprintf_P(PSTR("Ext: Raw: %x Calc: %d\n"), rawExtTemp, externalTemperature);
	} else {
		lprintf_P(PSTR("Error Reading External Temp\n"));
	}

	if(rawBattTemp != 0xEFF)
	{
		lprintf_P(PSTR("Batt: Raw: %x Calc: %d\n"), rawBattTemp, batteryTemperature);
	} else {
		lprintf_P(PSTR("Error Reading Battery Temp\n"));
	}
}

void dumpVarsToGSP(void)
{

	//lprintf_P(PSTR("epochStartSec: %d\n"), eeprom_read_byte(&EEepochStartSeconds));
	//lprintf_P(PSTR("epochStartMin: %d\n"), eeprom_read_byte(&EEepochStartMinutes));
	//lprintf_P(PSTR("epochStartHrs: %d\n"), eeprom_read_byte(&EEepochStartHours));
	//lprintf_P(PSTR("epochStartDays: %d\n"), eeprom_read_byte(&EEepochStartDays));
	_delay_ms(500);

	//lprintf_P(PSTR("ballastTrgtAlt: %d\n"), eeprom_read_word(&EEballastTargetAltitude));
	//lprintf_P(PSTR("ballastTrgt +Vspd: %d\n"), eeprom_read_word(&EEballastTargetPositiveVSpeed));
	//lprintf_P(PSTR("ballastTrgt -Vspd: %d\n"), eeprom_read_word(&EEballastTargetNegativeVSpeed));

	//lprintf_P(PSTR("maydayAlt: %d\n"), eeprom_read_word(&EEmaydayAltitude));
	//lprintf_P(PSTR("maydayVSpd: %d\n"), eeprom_read_word(&EEmaydayVSpeed));
	_delay_ms(500);

	lprintf_P(PSTR("ballastSftyAlt: %d\n"), eeprom_read_word(&EEballastSafetyAltThresh));
	uint8_t variable = (volatile)eeprom_read_byte(&EEautoBallastDisable);
	lprintf_P(PSTR("autoBallast dsbled?: %d\n"), variable);

	//lprintf_P(PSTR("overOcean? %d\n"), eeprom_read_byte(&EEoverOceanFlag));

	//lprintf_P(PSTR("nightTempForecast: %d\n"), eeprom_read_byte(&EEnightTemperatureForecast));
	//lprintf_P(PSTR("sunriseAntcpation: %ld\n"), eeprom_read_dword(&EEsunriseAnticipation));
	_delay_ms(500);

	lprintf_P(PSTR("maxAllowedTXInterval: %d\n"), eeprom_read_word(&EEmaxAllowableTXInterval));

	lprintf_P(PSTR("batteryHeaterSet: %d\n"), eeprom_read_byte(&EEbatteryHeaterSetpoint));

	lprintf_P(PSTR("dataSampleInterval: %d\n"), eeprom_read_word(&EEdataCollectionInterval));
	lprintf_P(PSTR("batchTXInterval: %d\n"), eeprom_read_word(&EEdataTransmitInterval));
    lprintf_P(PSTR("shortTXInterval: %d\n"), eeprom_read_word(&EEshortDataTransmitInterval));
	_delay_ms(500);

	lprintf_P(PSTR("HFdataXmitInterval: %d\n"), eeprom_read_word(&EEhfDataTransmitInterval));
	lprintf_P(PSTR("HFrapidXmitInterval: %d\n"), eeprom_read_byte(&EEhfRapidTransmit));

	lprintf_P(PSTR("epochOfLastBatchTX: %ld\n"), eeprom_read_dword(&EEepochOfLastBatchTransmit));

	lprintf_P(PSTR("curBatchNumber: %d\n"), eeprom_read_word(&EEcurrentBatchNumber));
	lprintf_P(PSTR("batchSampleStart: %d\n"), eeprom_read_word(&EEbatchSampleStart));
	lprintf_P(PSTR("batchSampleEnd: %d\n"), eeprom_read_word(&EEbatchSampleEnd));
	_delay_ms(500);

	lprintf_P(PSTR("commEEPROMStart: %d\n"), eeprom_read_word(&EEcommPromStart));
	lprintf_P(PSTR("commEEPROMEnd: %d\n"), eeprom_read_word(&EEcommPromEnd));

	lprintf_P(PSTR("flightComputerResetCount: %d\n"), eeprom_read_byte(&EEflightComputerResetCount));
	lprintf_P(PSTR("commModuleResetCount: %d\n"), eeprom_read_byte(&EEcommModuleResetCount));

	lprintf_P(PSTR("Phase: %d\n"), eeprom_read_byte(&EEflightPhase));
	_delay_ms(500);

	lprintf_P(PSTR("TelemBitmap: %lx "), eeprom_read_dword(&EEcurrentTelemetryBitmap[0]));
    lprintf_P(PSTR("%lx "), eeprom_read_dword(&EEcurrentTelemetryBitmap[1]));
    lprintf_P(PSTR("%lx\n"), eeprom_read_dword(&EEcurrentTelemetryBitmap[2]));
	lprintf_P(PSTR("telemetrySpeedDial: %d\n"), eeprom_read_word(&EEcurrentTelemetryVersion));

    lprintf_P(PSTR("epLoc: %d\n"), eeprom_read_byte(&EEEpochLock));

	//int16_t EEMEM EEvSpeedHolderSamples[VSPEEDSAMPLESDESIRED];
	//Maybe should print this for debug...

}

struct gpsData currentPositionData;

void dumpGPS(void)
{

	/*lprintf_P(PSTR("Lat: %f Lon: %f\n"), currentPositionData.latitude, currentPositionData.longitude);
	lprintf_P(PSTR("Alt: %d Sats: %d\n"), currentPositionData.altitude, currentPositionData.numberOfSats);
	lprintf_P(PSTR("vdop: %d hdop: %d\n"), currentPositionData.vdop, currentPositionData.hdop);*/

}

uint8_t gpsFailures = 0;

void processMonitor(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In Process Monitor\n"));
	#endif

	uint32_t currentBitmask[3];
	currentBitmask[0] = eeprom_read_dword(&EEcurrentTelemetryBitmap[0]);
	currentBitmask[1] = eeprom_read_dword(&EEcurrentTelemetryBitmap[1]);
	currentBitmask[2] = eeprom_read_dword(&EEcurrentTelemetryBitmap[2]);

	getGPS(&currentPositionData);
	if(currentPositionData.status == 0)
	{
		gpsFailures = 0;
	} else {
		#ifdef FCPUDEBUG
			lprintf_P(PSTR("Bad GPS Data\n"));
		#endif
		#ifndef OSHITDISABLE
		//flush Vspeed
		numberOfVSpeedSamples=0;
		gpsFailures++;
		//turn on GPS status telemetry channel
		lprintf_P(PSTR("CBM1\n"));
		currentBitmask[0] |= _BV(8);
		#endif
	}

	if(gpsFailures >= CRITGPSFAIL)
	{
		//Setting altitude to 0 will kick us into flight phase 3
		#ifdef FCPUDEBUG
			lprintf_P(PSTR("Critical GPS Failure\n"));
		#endif
		gpsFailures = CRITGPSFAIL + 1;
		currentPositionData.altitude = 0;
		vSpeedAvg = 0;
		eeprom_write_byte(&EEautoBallastDisable, 1);
	}

	//get battetry temp
	uint16_t rawBattTemp = tmp100rawTemp(TMP101BH)>>4;
	int16_t btinm = get12bit2scomp(rawBattTemp);
	int8_t batteryTemperature = (int8_t)(btinm/16);
	//conver to 8 bit

	//If battery temp is below setpoint, turn on telemetry channel
	int8_t batterySetpoint = eeprom_read_byte(&EEbatteryHeaterSetpoint);;
	if(batteryTemperature < batterySetpoint)
	{
	    lprintf_P(PSTR("CBM1\n"));
		currentBitmask[0] |= (uint32_t)(1<<24);
	}

	//GET RAW PACK VOLTAGE, If below Nominal, transmit
	//if(batteryVoltage < NominalVoltage)
	//{
		//currentBitmask[0] |= _BV(1);
	//}

    eeprom_write_dword(&EEcurrentTelemetryBitmap[0], currentBitmask[0]);
    eeprom_write_dword(&EEcurrentTelemetryBitmap[1], currentBitmask[1]);
    eeprom_write_dword(&EEcurrentTelemetryBitmap[2], currentBitmask[2]);


	scheduleQueueAdd(&processMonitor, time+5);
}

//Calculate Running Avg. of Vertical Speed

//THIS IS DEFINED IN EEPROMVARS.H, GOD THIS IS STUPID
#define VSPEEDSAMPLESDESIRED 20


void calculateVspeed(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In Vspeed Calculator\n"));
	#endif
	struct gpsData myGPS = currentPositionData;

	//Failure mode: Assume we're at the same altitude we were at.
	//This means
	uint16_t thisAltitude = myGPS.altitude;

	int16_t vSpeedInstant[VSPEEDSAMPLESDESIRED];
	eeprom_read_block(vSpeedInstant, &EEvSpeedHolderSamples, sizeof(int16_t)*VSPEEDSAMPLESDESIRED);
	if(numberOfVSpeedSamples == VSPEEDSAMPLESDESIRED)
	{
		for(int i=1; i <= VSPEEDSAMPLESDESIRED; i++)
		{
			vSpeedInstant[i-1] = vSpeedInstant[i];
		}
	} else {
		numberOfVSpeedSamples++;
	}

	int16_t thisVspeed = (thisAltitude - lastAltitude) / ((float)(time - lastRunTime)/60.);
	vSpeedInstant[numberOfVSpeedSamples-1] = thisVspeed;
	int16_t vSpeedAdder=0;
	for(int i = 0; i < numberOfVSpeedSamples; i++)
	{
		vSpeedAdder += vSpeedInstant[i];
	}
	vSpeedAvg = vSpeedAdder / (int8_t)numberOfVSpeedSamples;

	lastRunTime = time;
	lastAltitude = thisAltitude;
	eeprom_write_block(vSpeedInstant, &EEvSpeedHolderSamples, sizeof(int16_t)*VSPEEDSAMPLESDESIRED);



	scheduleQueueAdd(&calculateVspeed, time+5);
}

void timedCutdown(uint32_t time)
{
	//BEFORE FLIGHT
	//Send Comm Controller the cutdown command
	//In response, will  I receive the Cutdown Now command?
}

uint8_t ballastBabySit;
int16_t babySitVertSpeed;
int16_t currentTargetVspeed;
uint32_t lastBallastTime;
void autoBallast(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In autoBallast\n"));
	#endif
	uint8_t retVal;
	uint16_t targetAltitude = eeprom_read_word(&EEballastTargetAltitude);
	uint16_t ballastSafety = eeprom_read_word(&EEballastSafetyAltThresh);
	uint8_t ballastDisabled = eeprom_read_byte(&EEautoBallastDisable);

	struct gpsData myGPS = currentPositionData;

	//Failure mode: Try to maintain stability (moderate upward float)
	uint16_t thisAltitude = myGPS.altitude;

	if(ballastBabySit == 1)
	{
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Ballast Babysit Enable!\n"));
			#endif
		if(vSpeedAvg > (currentTargetVspeed + babySitVertSpeed)/2)
		{
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Babysit: 1/2 T reached\n"));
			#endif
			//Close ballast
			//Send i2c address 0x09, 0d18
			uint8_t data = 18;
			uint8_t errorTolerance = 0;
			while( ((retVal = i2cMasterSendNI(BALLASTMODULE, 1, &data)) != I2C_OK) && errorTolerance < CRITCOMFAIL)
			{
				_delay_ms(500);
				errorTolerance++;
			}
			if(errorTolerance >= CRITCOMFAIL)
			{
				#ifdef FCPUDEBUG
					lprintf_P(PSTR("Ballast Error\n"));
				#endif
				statusCode = (statusCode & 0xFFFD) | (1 << 1);
			} else {
				statusCode = (statusCode & 0xFFFD);
			}
			scheduleQueueAdd(&autoBallast, time+60);
			ballastBabySit = 0;
		} else {
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Babysit: Still Waiting\n"));
			#endif
			scheduleQueueAdd(&autoBallast, time+10);
		}
	} else {

		#ifdef FCPUDEBUG
			lprintf_P(PSTR("No babysity\n"));
		#endif

		//If we're above the safety threshold
		if(ballastSafety < thisAltitude && ballastDisabled != 1)
		{


			//if current altitude is below target altitude AND vertical Velocity is above target
				//Switch to positive target velocity
			//else if current altitude is above target altitude
				//use negative target velocity
			//else if current altitude is below target altitude AND vertical velocity is below target
				//use zero target
			if(thisAltitude <= targetAltitude && vSpeedAvg > currentTargetVspeed)
			{
				#ifdef FCPUDEBUG
					lprintf_P(PSTR("Ballast: TVSpeed+\n"));
				#endif
				currentTargetVspeed = eeprom_read_word(&EEballastTargetPositiveVSpeed);
			} else if(thisAltitude > targetAltitude)
			{
				#ifdef FCPUDEBUG
					lprintf_P(PSTR("Ballast: TVSpeed-\n"));
				#endif
				currentTargetVspeed = eeprom_read_word(&EEballastTargetNegativeVSpeed);
			} else if(thisAltitude < targetAltitude && vSpeedAvg < currentTargetVspeed)
			{
				#ifdef FCPUDEBUG
					lprintf_P(PSTR("Ballast: TVSpeed0\n"));
				#endif
				currentTargetVspeed = 0;
			}

			//if vertical velocity is below target
			//save VV
			//set "come back and check" flag
			//turn on the ballast
			if(vSpeedAvg < currentTargetVspeed)
			{
				ballastBabySit = 1;
				babySitVertSpeed = vSpeedAvg;
				lastBallastTime = time;
				//turn on the ballast
				//send 0x09 0d19
				uint8_t data = 19;
				uint8_t errorTolerance = 0;
				while( ((retVal = i2cMasterSendNI(BALLASTMODULE, 1, &data)) != I2C_OK) && errorTolerance < CRITCOMFAIL)
				{
					_delay_ms(500);
					errorTolerance++;
				}
				if(errorTolerance >= CRITCOMFAIL)
				{
					#ifdef FCPUDEBUG
						lprintf_P(PSTR("Ballast Error\n"));
					#endif
					statusCode = (statusCode & 0xFFFD) | (1 << 1);
				} else {
					statusCode = (statusCode & 0xFFFD);
				}
				scheduleQueueAdd(&autoBallast, time+10);
			} else {
				scheduleQueueAdd(&autoBallast, time+60);
			}
		} else {

			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Ballast Disabled\n"));
			#endif

			//turn off the ballast
			//send 0x09 0d19
			uint8_t data = 18;
			uint8_t errorTolerance = 0;
			while( ((retVal = i2cMasterSendNI(BALLASTMODULE, 1, &data)) != I2C_OK) && errorTolerance < CRITCOMFAIL)
			{
				_delay_ms(500);
				errorTolerance++;
			}
			if(errorTolerance >= CRITCOMFAIL)
			{
				statusCode = (statusCode & 0xFFFD) | (1 << 1);
			} else {
				statusCode = (statusCode & 0xFFFD);
			}
			scheduleQueueAdd(&autoBallast, time+60);
		}
	}

}


void collectData(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In Data Collector\n"));
	#endif
	char sampleString[SAMPLESTRINGSIZEINCHARS];
	memset(sampleString, 0x00, SAMPLESTRINGSIZEINCHARS);

	//get time
	uint32_t epochNow = now();
	//get ambient pressure
	long myPressure;
	long myTemp;
	bmp085Convert(&myTemp, &myPressure);
	//get FC temp
	uint16_t rawFCTemp;
	int8_t internalTemp;
	rawFCTemp = tmp100rawTemp(TMP100FC)>>4;
	int16_t fctinm = get12bit2scomp(rawFCTemp);
	internalTemp = (int8_t)(fctinm/16);
	//Convert to 8 bit

	//get ext temp
	uint16_t rawExtTemp = tmp100rawTemp(TMP100EXT)>>4;
	int16_t externalTemperature = get12bit2scomp(rawExtTemp);

	//get battetry temp
	uint16_t rawBattTemp = tmp100rawTemp(TMP101BH)>>4;
	int16_t btinm = get12bit2scomp(rawBattTemp);
	int8_t batteryTemperature = (int8_t)(btinm/16);
	//conver to 8 bit

	//get humidity
	//NEED COMMANDS FROM TIM
	uint8_t humidity[2] = {0,0};
	uint16_t humFinal;
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(INTSENSOR);
	i2cWaitForComplete();
	i2cSendByte(0x04);
	i2cWaitForComplete();
	i2cSendStop();

	_delay_ms(10);

	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(INTSENSOR+1);
	i2cWaitForComplete();
	i2cReceiveByte(1);
	i2cWaitForComplete();
	humidity[0] = i2cGetReceivedByte();
	i2cWaitForComplete();
	i2cReceiveByte(0);
	i2cWaitForComplete();
	humidity[1] += i2cGetReceivedByte();
	i2cWaitForComplete();
	i2cSendStop();

	humFinal = ((uint16_t)humidity[0] << 8) | humidity[1];

	//get coud sensor value
	uint8_t cloudVal[2]= {0,0};
	uint8_t cloudFinal;
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(INTSENSOR);
	i2cWaitForComplete();
	i2cSendByte(0x09);
	i2cWaitForComplete();
	i2cSendStop();

	_delay_ms(10);

	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(INTSENSOR+1);
	i2cWaitForComplete();
	i2cReceiveByte(1);
	i2cWaitForComplete();
	cloudVal[0] = i2cGetReceivedByte();
	i2cWaitForComplete();
	i2cReceiveByte(0);
	i2cWaitForComplete();
	cloudVal[1] += i2cGetReceivedByte();
	i2cWaitForComplete();
	i2cSendStop();
	cloudFinal = (cloudVal[0] << 6) | (cloudVal[1] >> 2);
	//NEED COMMANDS FROM TIM

	//get longitude
	//get latitude
	//get altitude
	//get heading
	//get ground speed
	//get gps fix
	//get HDOP
	//get VDOP
	struct gpsData myGPS = currentPositionData;

	//Convert HDOP and VDOP to 16 bits

	//get climb rate
	//This is global variable VSpeedAvg

	//get raw pack voltage
	//AD7998 Interfacing
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(AD7992);
	i2cWaitForComplete();
	i2cSendByte(0x10);
	i2cWaitForComplete();
	i2cSendStop();

	_delay_us(5);

	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(AD7992+1);
	i2cWaitForComplete();
	i2cReceiveByte(1);
	i2cWaitForComplete();
	uint16_t batteryValue = (uint16_t)i2cGetReceivedByte() << 8;
	i2cWaitForComplete();
	i2cReceiveByte(0);
	i2cWaitForComplete();
	batteryValue += i2cGetReceivedByte();
	i2cWaitForComplete();
	i2cSendStop();

	batteryValue &= 0x0FFF;
	//This is not such a magic value.  12 bits AD = 4096
	//Divider network = 4.07 kohms and 20 khoms
	//(3.3 volts / 4096) * (24.07/4.07) = 0.0047647
	//Multiply by 10 to get bigger value.
	batteryValue = (uint16_t)((((float)batteryValue*0.0047647))*10.);
	uint8_t outputVoltage = (uint8_t)batteryValue;

	//get ballast valve state
	uint8_t ballastError = 0;
	i2cSendStart();
	ballastError |= i2cWaitForComplete();
	i2cSendByte(18);
	ballastError |= i2cWaitForComplete();
	i2cSendByte(9);
	ballastError |= i2cWaitForComplete();
	i2cSendByte(1);
	ballastError |= i2cWaitForComplete();
	i2cSendStop();

	_delay_ms(1000);

	i2cSendStart();
	ballastError |= i2cWaitForComplete();
	i2cSendByte(0x13);
	ballastError |= i2cWaitForComplete();
	i2cReceiveByte(1);
	ballastError |= i2cWaitForComplete();
	uint16_t valveStatus = (uint16_t)i2cGetReceivedByte() << 8;
	ballastError |= i2cWaitForComplete();
	i2cReceiveByte(0);
	ballastError |= i2cWaitForComplete();
	valveStatus += i2cGetReceivedByte();
	ballastError |= i2cWaitForComplete();
	i2cSendStop();

	//1 = open, 0 = closed

	//get ballast remaining
	i2cSendStart();
	ballastError |= i2cWaitForComplete();
	i2cSendByte(18);
	ballastError |= i2cWaitForComplete();
	i2cSendByte(9);
	ballastError |= i2cWaitForComplete();
	i2cSendByte(2);
	ballastError |= i2cWaitForComplete();
	i2cSendStop();

	_delay_ms(1000);

	i2cSendStart();
	ballastError |= i2cWaitForComplete();
	i2cSendByte(18+1);
	ballastError |= i2cWaitForComplete();
	i2cReceiveByte(1);
	ballastError |= i2cWaitForComplete();
	uint16_t ballastRemaining = (uint16_t)i2cGetReceivedByte() << 8;
	ballastError |= i2cWaitForComplete();
	i2cReceiveByte(0);
	ballastError |= i2cWaitForComplete();
	ballastRemaining += (uint16_t)i2cGetReceivedByte();
	ballastError |= i2cWaitForComplete();
	i2cSendStop();

	statusCode = (statusCode & 0xFFFD) | (ballastError << 1);
	//16 bit grams remaining

	//get status code
	//variable StatusCode

	//Get Helium temperature
	int16_t heliumTemperature;
	uint8_t helVal[2]= {0,0};
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(INTSENSOR);
	i2cWaitForComplete();
	i2cSendByte(0x0A);
	i2cWaitForComplete();
	i2cSendStop();

	_delay_ms(10);

	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(INTSENSOR+1);
	i2cWaitForComplete();
	i2cReceiveByte(1);
	i2cWaitForComplete();
	helVal[0] = i2cGetReceivedByte();
	i2cWaitForComplete();
	i2cReceiveByte(0);
	i2cWaitForComplete();
	helVal[1] += i2cGetReceivedByte();
	i2cWaitForComplete();
	i2cSendStop();
	heliumTemperature = (int16_t)(((uint16_t)helVal[0] * 256) + (helVal[1]));
	//NEED COMMANDS FROM TIM

	//format into string
	sprintf_P(sampleString, PSTR("%ld," //Epoch
		"%ld,%ld," 	//Ambient Pressure Raw and Calculate
		"%d,%d,"  	//FC Temp Raw and Calculated
		"%d,%d,"   	//Ext Temp raw and Calcuated
		"%d,%d,"   	//Battery Temp Raw and Calculated
		"%d,%d,"   	//Humidity raw and Calculated
		"%d,"		//Cloud Sensor
		"%s,%f,"	//Longitude Raw and Calculated
		"%s,%f,"	//Latitude Raw and Calculated
		"%s,%d,"	//Altitude Raw and Calculated
		"%s,%d,"	//Heading Raw and Calculated
		"%s,%d,"	//Ground Speed raw and calculated
		"%d,%d,"	//UTC Date and Time (in YYMMDD,HHMMSS)
		"%d,"		//GPS Fix
		"%s,%d,"	//HDOP Raw and calc
		"%s,%d,"	//VDOP Raw and calc
		"%d,"		//Climb Rate
		"%d,"		//Voltage
		"%d,%d,"	//Ballast Valve State and Amount Remaining
		"%d,"		//Dewpoint
		"%d,"       //status Code
		"%d,"),		//Helium Temperature
		epochNow,
		myPressure, myPressure/2,  //Ambient Pressure Raw and Calculate, NO RAW YET
		rawFCTemp, internalTemp, //FC Temp Raw and Calculated
		rawExtTemp, externalTemperature,
		rawBattTemp, batteryTemperature,
		//NEED COMMANDS FROM TIM
		humFinal,humFinal,
		//NEED COMMANDS FROM TIM
		cloudFinal,
		"LON",(double)myGPS.longitude, //No Raw Lon Yet
		"LAT",(double)myGPS.latitude,  //No Raw Lat Yet
		"ALT",myGPS.altitude,  //No Raw Altitude Yet
		"HED",myGPS.bearing,   //No Raw Heading Yet
		"GSP",myGPS.speed,		//No Raw Groundspeed
		000000,000000,		//No UTC Date/Time yet
		myGPS.status,
		"HDP",myGPS.hdop/5,	//No Raw HDOP yet
		"VDP",myGPS.vdop/5,	//No Raw VDOP yet
		vSpeedAvg,
		outputVoltage,
		(uint8_t)valveStatus,
		ballastRemaining,
		-14,
		statusCode, heliumTemperature);

	//Pad with spaces
	uint8_t ssLen = strlen(sampleString);
	uint8_t k;
	for(k = ssLen; k < (SAMPLESTRINGSIZEINCHARS - 2); k++)
	{
		sampleString[k] = '.';
	}
	//lprintf("pw: %d\n", k);

	sampleString[SAMPLESTRINGSIZEINCHARS-2] = '\r';
	sampleString[SAMPLESTRINGSIZEINCHARS-1] = '\n';
	sampleString[SAMPLESTRINGSIZEINCHARS] = '\0';

	//store in openlog
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("Sample String: \n"));
		for(int i=0; i < SAMPLESTRINGSIZEINCHARS; i++)
		{
			lprintf("%c", sampleString[i]);
		}
	#endif
	putDataSample(sampleString);



	//Here is where we increment the batch so that things actually get transmitted.
	uint16_t batchSampleEnd = eeprom_read_word(&EEbatchSampleEnd);
	eeprom_write_word(&EEbatchSampleEnd, ++batchSampleEnd);

	//reschedule myself
	if(time != 0xFFFFFFFF)
	{
		scheduleQueueAdd(&collectData, time+eeprom_read_word(&EEdataCollectionInterval));
	} else {
		//Bogus time means testing time!
	}
}

void transmitShortReport(uint32_t time)
{
    if(enableReports)
	{
		#ifdef FCPUDEBUG
			lprintf_P(PSTR("Generating Report\n"));
		#endif
		uint16_t desiredTX = eeprom_read_word(&EEshortDataTransmitInterval);

		uint8_t packet1[6];
		uint8_t packet2[6];

        float tmp1;
        uint32_t lon_code, lat_code;
        if(currentPositionData.longitude < 0.0 )
        {
            /* west */
            tmp1 = (currentPositionData.longitude + 360.0)/360.0;
        }
        else
        {
            /* east */
            tmp1 = currentPositionData.longitude/360.0;
        }
        lon_code = (uint32_t) (tmp1 * 0x0FFFFFF) & 0x0FFFFFF;
        tmp1 = -(currentPositionData.latitude - 90.0)/180.0;
        lat_code = (uint32_t) (tmp1 * 0x0FFFFFF) & 0x0FFFFFF;

        if((reportCounterH & 0x03) == 0)
        {
            reportCounterH++;
        }
        if((reportCounterL & 0x03) == 0)
        {
            reportCounterL++;
        }


		packet1[0] = lon_code>>16;
		packet1[1] = lon_code>>8;
		packet1[2] = lon_code;
		packet1[3] = currentPositionData.bearing >> 1;
		packet1[4] = currentPositionData.altitude >> 8;
		packet1[5] = (currentPositionData.altitude & 0x00F0) | (reportCounterL & 0x0003);

		uint32_t rightNow = now();
		packet2[0] = lat_code>>16;
		packet2[1] = lat_code>>8;
		packet2[2] = lat_code;
		packet2[3] = currentPositionData.speed;
		packet2[4] = (rightNow/60) >> 8;
		packet2[5] = ((rightNow/60) & 0x00F0) | ((reportCounterH<<2) & 0x000C);

        lprintf("Short Report: ");
        for(int i = 0; i < 6; i++)
        {
            lprintf_P(PSTR("%x "), packet1[i]);
        }
        for(int i = 0; i < 6; i++)
        {
            lprintf_P(PSTR("%x "), packet2[i]);
        }
        lprintf("\n");


		reportCounterL++;
		reportCounterH++;

		i2cSendStart();
        i2cWaitForComplete();
        i2cSendByte(0x10);
        i2cWaitForComplete();
        i2cSendByte(0x0);
        i2cWaitForComplete();

        i2cSendByte(packet1[0]);
        i2cWaitForComplete();
        i2cSendByte(packet1[1]);
        i2cWaitForComplete();
        i2cSendByte(packet1[2]);
        i2cWaitForComplete();
        i2cSendByte(packet1[3]);
        i2cWaitForComplete();
        i2cSendByte(packet1[4]);
        i2cWaitForComplete();
        i2cSendByte(packet1[5]);
        i2cWaitForComplete();

        i2cSendByte(packet2[0]);
        i2cWaitForComplete();
        i2cSendByte(packet2[1]);
        i2cWaitForComplete();
        i2cSendByte(packet2[2]);
        i2cWaitForComplete();
        i2cSendByte(packet2[3]);
        i2cWaitForComplete();
        i2cSendByte(packet2[4]);
        i2cWaitForComplete();
        i2cSendByte(packet2[5]);
        i2cWaitForComplete();

        i2cSendStop();

        scheduleQueueAdd(&transmitShortReport, time+desiredTX);
		//BEFORE FLIGHT Send this data to the comm controller
	}
}

void transmitSamples(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In Sample TX\n"));
	#endif
	loadBatch();
	flushSatQueue();
	uint16_t maxTX = eeprom_read_word(&EEmaxAllowableTXInterval);
	uint16_t desiredTX = eeprom_read_word(&EEdataTransmitInterval);

	if(maxTX > desiredTX)
		scheduleQueueAdd(&transmitSamples, time+desiredTX);
	else
		scheduleQueueAdd(&transmitSamples, time+maxTX);


}

void updateCommHFTelemetry(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In HF Telem\n"));
	#endif
	//ALL 8 BITS VALUES!
	//send 8 bits of ballast remaining
	//send raw pack voltage
	//send top temp

	//send to comm controller

	scheduleQueueAdd(&updateCommHFTelemetry, time+60);
}

//Simply add this to the scheduler queue if you want it.
//Note: needs to be able to remove itself from the queue.

void rapidHFXmit(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In Rapid HF TX\n"));
	#endif
	if(rapidHFEnable == 1)
	{
		//send comm controller rapid HF command
		scheduleQueueAdd(&rapidHFXmit, time+eeprom_read_byte(&EEhfRapidTransmit));
	}
}


void ballastStaticTickle(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In ballast tickle\n"));
	#endif
	if((time - lastBallastTime) > 3600)
	{
		uint8_t retVal;
		uint8_t dataToSend[3] = {20,00,01};
		uint8_t errorTolerance = 0;
		while( ((retVal = i2cMasterSendNI(18, 3, dataToSend)) != I2C_OK) && errorTolerance < CRITCOMFAIL)
		{
			_delay_ms(500);
			errorTolerance++;
		}
		if(errorTolerance >= CRITCOMFAIL)
		{
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Ballast Error\n"));
			#endif
			statusCode = (statusCode & 0xFFFD) | (1 << 1);
		} else {
			statusCode = (statusCode & 0xFFFD);
		}
	}
	scheduleQueueAdd(&ballastStaticTickle, time+3600);
}

//1 implies that we have cutdown

void flightPhaseLogic(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In Phase Logic\n"));
	#endif
	uint8_t currentPhase = eeprom_read_byte(&EEflightPhase);
	struct gpsData myGPS = currentPositionData;


	//Failure Condition: We will enter mayday mode.
	uint16_t thisAltitude = myGPS.altitude;

	uint16_t maydayAltitude = eeprom_read_word(&EEmaydayAltitude);
	int16_t maydayVSpeed = eeprom_read_word(&EEmaydayVSpeed);
	uint8_t myPhase = currentPhase & 0x0F;
	uint8_t myFlags = currentPhase >> 4;
	switch(myPhase)
	{

		case 0:
			if(((PIND & _BV(TAKEOFFPIN)) == 1) && (myFlags & 1 == 1))
			{
				#ifdef FCPUDEBUG
					_delay_ms(500);
					//lprintf_P(PSTR("Where am I going?! Entering Phase 1\n"));
				#endif
				myPhase = 1;
				_delay_ms(500);
				//Save time
			}
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 0: Prelaunch\n"));
			#endif
			//reschedule 1 second from now
			scheduleQueueAdd(&flightPhaseLogic, time+1);
			break;
		case 1:
			//change sample time to 30 seconds
			eeprom_write_word(&EEdataCollectionInterval, 30);
			//schedule rapid hf xmit
			if(rapidHFEnable == 0)
			{
				scheduleQueueAdd(&rapidHFXmit, time);
			}
			//enable rapid hf xmit
			rapidHFEnable = 1;


			//reschedule 1 minute from now
			if((myGPS.altitude > 8500) && (vSpeedAvg < 0) && (myFlags & 1 == 1))
			{
				myPhase = 2;
			}
			scheduleQueueAdd(&flightPhaseLogic, time+10);
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 1: Climb\n"));
			#endif
			break;
		case 2:
			//disable rapid hf xmit
			rapidHFEnable = 0;
			//make sure sat is enabled in here! BEFORE FLIGHT
			if((vSpeedAvg < maydayVSpeed) || (thisAltitude < maydayAltitude)  || (cutdownStatus == 1) && (myFlags & 1 == 1))
			{
				myPhase = 3;
			}
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 2: Cruise\n"));
			#endif
			scheduleQueueAdd(&flightPhaseLogic, time+30);
		case 3:
			//disable sat in here! BEFORE FLIGHT
			//enable rapid hf xmit
			if(rapidHFEnable == 0)
			{
				scheduleQueueAdd(&rapidHFXmit, time);
			}
			rapidHFEnable = 1;
			if((vSpeedAvg > maydayVSpeed) && (thisAltitude > maydayAltitude) && (cutdownStatus != 0) && (myFlags & 1 == 1))
			{
					myPhase = 2;
			}
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 3: FALLING OUT OF SKY\n"));
			#endif
			scheduleQueueAdd(&flightPhaseLogic, time+1);
			break;
		case 4:
			//reset HF and Sat Sample and Transmit Intervals to 1 hour
			scheduleQueueAdd(&flightPhaseLogic, time+3600);
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 4: Landed\n"));
			#endif
			break;
		default:
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("ERROR!\n"));
			#endif
			break;
		}

	eeprom_write_byte(&EEflightPhase, (myFlags << 4) + myPhase);

}


inline uint32_t now(void)
{
	uint8_t seconds, minutes, hours, days;
	uint8_t error;
	error = getTime(&seconds, &minutes, &hours, &days);
	while(error != 0)
	{
		_delay_ms(50);
		error = getTime(&seconds, &minutes, &hours, &days);
	}
	return getEpochSeconds(seconds, minutes, hours, days);
}

void ioinit (void)
{
    //1 = output, 0 = input
    DDRA = 0b00010000;
    DDRB = 0b11111111; //PB4 = MISO
    DDRC = 0b11111111; //
    DDRD = 0b11011010; //PORTD (RX on PD0)

    PORTD |= _BV(TAKEOFFPIN);

    XCLR_P |= _BV(XCLR);
    PORTB &= ~_BV(0) & ~_BV(1);

    UBRR1H = MYUBRR >> 8;
    UBRR1L = MYUBRR;
    UCSR1B = (1<<RXEN1)|(1<<TXEN1);


    stdout = &mystdout; //Required for printf init

    i2cInit();
    //i2cSetBitrate(10);

}



static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);

    loop_until_bit_is_set(UCSR1A, UDRE1);
    UDR1 = c;

    return 0;
}

uint8_t uart_getchar(void)
{
	uint16_t errorCounter = 0xFFFF;
    while( !(UCSR1A & (1<<RXC1)) && (errorCounter > 100))
    {
		errorCounter--;
	}
	if(errorCounter <= 101)
	{
        return 0xff;
	} else {
        return UDR1;
	}

}

int lprintf(char *str, ...)
{
	char lstr[100];
	uint8_t i2cSend[101];
	int chars;
	va_list args;

	va_start(args, str);

	chars = vsnprintf(lstr, 100, str, args);

	if(chars > 100)
	{
		va_end(args);
		return 1;
	} else {
		int i=0;
		i2cSend[0] = 0x05;
		for(i=0; i <=chars; i++)
		{
			i2cSend[i+1] = (uint8_t)lstr[i];
		}
		uint8_t retVal = i2cMasterSendNI(0b00001110, chars+1, i2cSend);
		if(retVal != I2C_OK)
		{
			yellow_on();
		}

		va_end(args);
		return 0;
	}
	_delay_ms(100);
}

int lprintf_P(const char *str, ...)
{
	char lstr[100];
	uint8_t i2cSend[101];
	int chars;
	va_list args;

	va_start(args, str);

	chars = vsnprintf_P(lstr, 100, str, args);

	if(chars > 100)
	{
		va_end(args);
		return 1;
	} else {
		int i=0;
		i2cSend[0] = 0x05;
		for(i=0; i <=chars; i++)
		{
			i2cSend[i+1] = (uint8_t)lstr[i];
		}
		uint8_t retVal = i2cMasterSendNI(0b00001110, chars+1, i2cSend);
		if(retVal != I2C_OK)
		{
			yellow_on();
		}

		va_end(args);
		return 0;
	}
	_delay_ms(100);
}
