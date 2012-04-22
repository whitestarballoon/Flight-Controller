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
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/wdt.h>

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "uart.h"

#include "lprintf.h"

#include "lib/i2c.h"
#include "lib/tmp100.h"
#include "lib/bmp085.h"
#include "lib/ds3231.h"

#include "parsing/epoch.h"
#include "parsing/gps.h"

#include "comm/outputParse.h"

#include "logging/openlog.h"

#include "eepromVars.h"
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

#define POWERPORT PORTB
#define I2C9PT  0
#define I2C10PT 1

#define TMP100FC 0b10011110
#define TMP101BH 0b10010010
#define TMP100EXT 0b10010110
#define BALLASTMODULE 0b00010010
#define AD7992 0x40
#define INTSENSOR 0x14

#define GSPDEBUG
#define FCPUDEBUG
//#define OSHITDISABLE

#define CRITCOMFAIL 25

#define CRITGPSFAIL 20


//Define functions
//======================
uint32_t now(void);
void ioinit(void);      // initializes IO
void receiveCommandHandler(uint8_t receiveDataLength, uint8_t* recieveData);

//Test Vars
void dumpVarsToGSP(void);
void bmpTest(void);
void dumpTemps(void);
void debugBallast(void);
void dumpGPS(void);

//DEFINE SCHEDULED THINGS
void timedCutdown(uint32_t);
void processMonitor(uint32_t);
void calculateVspeed(uint32_t);
void timedCutdown(uint32_t);
void autoBallast(uint32_t);
void collectData(uint32_t);
void transmitSamples(uint32_t);
void transmitShortReport(uint32_t);
void ballastStaticTickle(uint32_t);
void flightPhaseLogic(uint32_t);
void resetWatchdog(uint32_t);
void incrementEpoch(uint32_t);
void updateSpeedDial(uint16_t speedDial);

//Vspeed Calculation Variables
int16_t vSpeedAvg = 0;
uint8_t numberOfVSpeedSamples=0;
uint16_t lastAltitude=0;
uint16_t lastRunTime=0;

//Globals
uint8_t enableReports = 1;
uint8_t reportCounterL=0;
uint8_t reportCounterH=0;
uint16_t statusCode = 0x00;

uint32_t epochOffset;

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
//Watchdog Vars
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));

int main (void)
{
	uint8_t seconds,  minutes, hours, days;
	uint8_t error;

    //wdt_disable();

	ioinit(); //Setup IO pins and defaults
	i2cSetTheDamnTWBRMyself(10);

	i2cSetLocalDeviceAddr(0b00001010, 0x00, 0, 0, 0);
	i2cSetSlaveReceiveHandler(receiveCommandHandler);
	i2cEnableInt();
	sei();

	_delay_ms(500);

	lprintf("WSB CPU Alive");
    
    #ifdef FCPUDEBUG
		lprintf_P(PSTR("Set Epoch"));
	#endif
	wdt_enable(WDTO_8S);
	if(eeprom_read_byte(&EEEpochLock) == 0)
	{
        lprintf_P(PSTR("ReSet Epoch"));
		eeprom_write_dword(&EEepochOffset, 0);
	}

	uint32_t offset = eeprom_read_dword(&EEepochOffset);
	uint8_t offsetDays = offset / 86400;
	uint8_t offsetHours = (offset - (offsetDays * 86400)) / 3600;
	uint8_t offsetMinutes = (offset - (offsetDays * 86400) - (offsetHours * 3600)) / 60;
	uint8_t offsetSeconds = (offset - (offsetDays * 86400) - (offsetHours * 3600) - (offsetMinutes * 60));
    ds3231write(0x00, offsetSeconds);
    ds3231write(0x01, offsetMinutes);
    ds3231write(0x02, offsetHours);
    ds3231write(0x03, offsetDays);

    writeEpochStart(0, 0, 0, 0);

	lprintf("Still Alive");

	// TMP100
	setTMP100config(TMP100FC, 0xE0);

	BMP085_Calibration();
	// BMP085 END
	//defaultEEPROM();
	if(eeprom_read_byte(&EEEpochLock) == 0)
	{
		//defaultEEPROM();
        initOpenLogTest();
        eeprom_write_word(&EEcurrentBatchNumber, 0);
        eeprom_write_word(&EEbatchSampleStart, 0);
        eeprom_write_word(&EEbatchSampleEnd, 0);
	} else {
		initOpenLogFlight();
	}

	lprintf("SSAlive");

	uint32_t rnow = now();
	scheduleQueueAdd(&resetWatchdog, rnow);
	scheduleQueueAdd(&incrementEpoch, rnow);
	scheduleQueueAdd(&processMonitor, rnow);
	scheduleQueueAdd(&calculateVspeed, rnow);
	scheduleQueueAdd(&collectData, rnow);
	scheduleQueueAdd(&transmitSamples, rnow);
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
		//lprintf("PTR: %p time: %lud now: %lud", ptrToFunction, scheduleTime, rightNow);

		if(error == 0 && scheduleTime <= rightNow)
		{
			thisFunction(rightNow);
		} else if (error == 0 && scheduleTime > rightNow)
		{
			scheduleQueueAdd(thisFunction, scheduleTime);
		} else {
			//Error!
		}
		_delay_ms(500);
	}

    return(0);
}

uint8_t cutdownStatus = 0;
void receiveCommandHandler(uint8_t receiveDataLength, uint8_t* recieveData)
{
    uint8_t temp;

    lprintf_P(PSTR("CPU I2C"));
    for(int i=0; i < receiveDataLength; i++)
    {
        lprintf("%x ", recieveData[i]);
    }
    lprintf("");

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
				eeprom_write_word((uint16_t *)&EEballastTargetPositiveVSpeed, holder);
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
				eeprom_write_byte((uint8_t *)&EEnightTemperatureForecast, recieveData[1]);
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
				updateSpeedDial(holder);
			}
			break;
		case 0x09:
			//set Custom Bitmask Select
			//if(receiveDataLength == 13)
			{
			    uint32_t holder1 = (((uint32_t)recieveData[1])<<24) + (((uint32_t)recieveData[2])<<16) + (((uint32_t)recieveData[3])<<8) + ((uint32_t)recieveData[4]);
			    uint32_t holder2 = (((uint32_t)recieveData[5])<<24) + (((uint32_t)recieveData[6])<<16) + (((uint32_t)recieveData[7])<<8) + ((uint32_t)recieveData[8]);
			    uint32_t holder3 = (((uint32_t)recieveData[9])<<24) + (((uint32_t)recieveData[10])<<16) + (((uint32_t)recieveData[11])<<8) + ((uint32_t)recieveData[12]);
				//eeprom_write_block((uint8_t*)recieveData[1], &EEcurrentTelemetryBitmap, sizeof(uint32_t)*3);
				lprintf("BM: %lx %lx %lx", holder1, holder2, ((uint32_t)recieveData[1])<<24);
				eeprom_write_dword(&EEcurrentTelemetryBitmap[0], holder1 );
				eeprom_write_dword(&EEcurrentTelemetryBitmap[1], holder2 );
				eeprom_write_dword(&EEcurrentTelemetryBitmap[2], holder3 );
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
				eeprom_write_byte((uint8_t *)&EEbatteryHeaterSetpoint, recieveData[1]);
				error |= setTMP101Thermo(TMP101BH, set12bit2scomp(eeprom_read_byte((uint8_t *)&EEbatteryHeaterSetpoint))*16, 0);
				error |= setTMP101Thermo(TMP101BH, set12bit2scomp(eeprom_read_byte((uint8_t *)&EEbatteryHeaterSetpoint))*16+32, 1);
				#ifdef GSPDEBUG
					if(error != 0)
					{
						//lprintf_P(PSTR("Error Setting thermometer"));
					}
				#endif
				//Should probably set a status code here  BEFORE FLIGHT
			}
			break;
		case 0x0D:
			//if(receiveDataLength == 3)
			{
				uint16_t holder = ((uint16_t)recieveData[1] << 8) + recieveData[2];
				eeprom_write_word(&EEballastSafetyAltThresh, holder);
			}
			break;
		case 0x10:
			//if(receiveDataLength == 5)
			{
				//It Looks Like You're trying to Schedule a cutdown!
				//uint32_t time = ((uint32_t)recieveData[0] << 24) + ((uint32_t)recieveData[1] << 16) + ((uint32_t)recieveData[2] << 8) + recieveData[3];
				uint32_t time = ((((uint32_t)recieveData[1])<<8) +  (recieveData[2]))*60;
				lprintf("Schd 4 %lu", time);
				scheduleQueueAdd(&timedCutdown, time);
			}
			break;
		case 0x11:
			//if(receiveDataLength == 3)
			{
				//Send Ballast Controller the ballast command
				uint8_t retVal;
                uint8_t dataToSend[3] = {20,recieveData[0],recieveData[1]};
                uint8_t errorTolerance = 0;
                while( ((retVal = i2cMasterSendNI(18, 3, dataToSend)) != I2C_OK) && errorTolerance < CRITCOMFAIL)
                {
                    _delay_ms(500);
                    errorTolerance++;
                }
			}
			break;
		case 0x12:
			//set Disarm Auto Ballast
			//lprintf_P(PSTR("bdis"));
			temp = eeprom_read_byte(&EEautoBallastDisable);
			//lprintf_P(PSTR("bptr %p"), &EEautoBallastDisable);
            eeprom_write_byte(&EEautoBallastDisable, 1);
            temp = eeprom_read_byte(&EEautoBallastDisable);
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
				eeprom_write_word((uint16_t *)&EEmaydayVSpeed, holder);
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
		case 0x18:
			//if(receiveDataLength == 3)
			{
				int16_t holder = ((uint16_t)recieveData[1]<<8) + recieveData[2];
				eeprom_write_word((uint16_t *)&EEballastTargetNegativeVSpeed, holder);
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
        case 0xf1:
            collectData(0xFFFFFFFF);
            transmitSamples(0xFFFFFFFF);
            break;
        case 0xf2:
            transmitShortReport(0xFFFFFFFF);
            break;
        case 0xf4:
            while(1);
            break;
		case 0xF7:
			eeprom_write_byte(&EEEpochLock, recieveData[1]);
			break;
		case 0xFB:
			defaultEEPROM();
			break;
		case 0xFC:
			collectData(0xFFFFFFFF);
			break;
		case 0xFF:
			dumpVarsToGSP();
			break;
	}
}

void updateSpeedDial(uint16_t speedDial)
{
    #ifdef FCPUDEBUG
        lprintf_P(PSTR("SpdDial"));
    #endif
    switch(speedDial)
    {
        case 0x00:
        {
            //Default 1 minute collect, 15 minute batch
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01000000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 60);
            eeprom_write_word(&EEdataTransmitInterval, 900);
        }
        break;
        case 0x01:
        {
            //10 minute collect, 20 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 600);
            eeprom_write_word(&EEdataTransmitInterval, 1200);
        }
        break;
        case 0x02:
        {
            //5 minute collect, 15 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 300);
            eeprom_write_word(&EEdataTransmitInterval, 900);
        }
        break;
        case 0x03:
        {
            //15 minute collect, 15 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 900);
            eeprom_write_word(&EEdataTransmitInterval, 900);
        }
        break;
        case 0x04:
        {
            //15 minute collect, 30 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 900);
            eeprom_write_word(&EEdataTransmitInterval, 1800);
        }
        break;
        case 0x05:
        {
            //15 minute collect, 1 hour transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 900);
            eeprom_write_word(&EEdataTransmitInterval, 3600);
        }
        break;
        case 0x06:
        {
            //1 minute collect, 1 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 60);
            eeprom_write_word(&EEdataTransmitInterval, 60);
        }
        break;
        case 0x07:
        {
            //1 minute collect, 5 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010100011010101010100001011001);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 60);
            eeprom_write_word(&EEdataTransmitInterval, 300);
        }
        break;
        case 0x08:
        {
            //1 minute collect, 1 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b00000000000010101010000100001000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b00000000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 60);
            eeprom_write_word(&EEdataTransmitInterval, 60);
        }
        break;
        case 0x09:
        {
            //1 minute collect, 1 minute transmit
            eeprom_write_dword(&EEcurrentTelemetryBitmap[0], 0b01010101011010101010100101011111);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[1], 0b01100000000000000000000000000000);
            eeprom_write_dword(&EEcurrentTelemetryBitmap[2], 0);
            eeprom_write_word(&EEdataCollectionInterval, 60);
            eeprom_write_word(&EEdataTransmitInterval, 60);
        }
        break;

    }
}


void dumpVarsToGSP(void)
{

	lprintf_P(PSTR("ballastTrgtAlt: %u"), eeprom_read_word(&EEballastTargetAltitude));
	lprintf_P(PSTR("ballastTrgt +Vspd: %d"), eeprom_read_word((uint16_t *)&EEballastTargetPositiveVSpeed));
	lprintf_P(PSTR("ballastTrgt -Vspd: %d"), eeprom_read_word((uint16_t *)&EEballastTargetNegativeVSpeed));

	lprintf_P(PSTR("maydayAlt: %ud"), eeprom_read_word(&EEmaydayAltitude));
	lprintf_P(PSTR("maydayVSpd: %d"), eeprom_read_word((uint16_t *)&EEmaydayVSpeed));
	_delay_ms(500);

	lprintf_P(PSTR("ballastSftyAlt: %u"), eeprom_read_word(&EEballastSafetyAltThresh));
	uint8_t variable = (volatile int)eeprom_read_byte(&EEautoBallastDisable);
	lprintf_P(PSTR("autoBallast dsbled?: %d"), variable);

	lprintf_P(PSTR("maxAllowedTXInterval: %u"), eeprom_read_word(&EEmaxAllowableTXInterval));

	lprintf_P(PSTR("batteryHeaterSet: %d"), eeprom_read_byte((uint8_t *)&EEbatteryHeaterSetpoint));

	lprintf_P(PSTR("dataSampleInterval: %u"), eeprom_read_word(&EEdataCollectionInterval));
	lprintf_P(PSTR("batchTXInterval: %u"), eeprom_read_word(&EEdataTransmitInterval));
    lprintf_P(PSTR("shortTXInterval: %u"), eeprom_read_word(&EEshortDataTransmitInterval));
	_delay_ms(500);

	lprintf_P(PSTR("curBatchNumber: %u"), eeprom_read_word(&EEcurrentBatchNumber));
	lprintf_P(PSTR("batchSampleStart: %u"), eeprom_read_word(&EEbatchSampleStart));
	lprintf_P(PSTR("batchSampleEnd: %u"), eeprom_read_word(&EEbatchSampleEnd));
	_delay_ms(500);

	lprintf_P(PSTR("Phase: %d"), eeprom_read_byte(&EEflightPhase));
	lprintf_P(PSTR("TelemBitmap: %lx "), eeprom_read_dword(&EEcurrentTelemetryBitmap[0]));
    lprintf_P(PSTR("%lx "), eeprom_read_dword(&EEcurrentTelemetryBitmap[1]));
    lprintf_P(PSTR("%lx"), eeprom_read_dword(&EEcurrentTelemetryBitmap[2]));
	lprintf_P(PSTR("telemetrySpeedDial: %u"), eeprom_read_word(&EEcurrentTelemetryVersion));

    lprintf_P(PSTR("epLoc: %d"), eeprom_read_byte(&EEEpochLock));

}

struct gpsData currentPositionData;
uint8_t gpsFailures = 0;
uint8_t ballastBabySit =0;
void processMonitor(uint32_t time)
{
	#ifdef FCPUDEBUG
		//lprintf_P(PSTR("In Proc Mon"));
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
			lprintf_P(PSTR("Bad GPS Data"));
		#endif
		#ifndef OSHITDISABLE
		//flush Vspeed
		numberOfVSpeedSamples=0;
		gpsFailures++;
		//turn on GPS status telemetry channel
		//lprintf_P(PSTR("CBM1"));
		currentBitmask[0] |= _BV(8);
		#endif
	}

	if(gpsFailures >= CRITGPSFAIL)
	{
		//Setting altitude to 0 will kick us into flight phase 3
		#ifdef FCPUDEBUG
			lprintf_P(PSTR("Critical GPS Failure"));
		#endif
		gpsFailures = CRITGPSFAIL + 1;
		currentPositionData.altitude = 0;
		vSpeedAvg = 0;
		//eeprom_write_byte(&EEautoBallastDisable, 1);
	}

    eeprom_write_dword(&EEcurrentTelemetryBitmap[0], currentBitmask[0]);
    eeprom_write_dword(&EEcurrentTelemetryBitmap[1], currentBitmask[1]);
    eeprom_write_dword(&EEcurrentTelemetryBitmap[2], currentBitmask[2]);


	scheduleQueueAdd(&processMonitor, time+5);

}

//Calculate Running Avg. of Vertical Speed
void calculateVspeed(uint32_t time)
{
	#ifdef FCPUDEBUG
		//lprintf_P(PSTR("In Vspeed Calculator"));
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

	int16_t thisVspeed = (int16_t)((thisAltitude - lastAltitude) / ((float)(time - lastRunTime)/60.));
	//int16_t thisVspeed = 30*(thisAltitude - lastAltitude);
	vSpeedInstant[numberOfVSpeedSamples-1] = thisVspeed;
	//lprintf_P(PSTR("This Vspeed: %d"), thisVspeed);
	int32_t vSpeedAdder=0;
	for(int i = 0; i < numberOfVSpeedSamples; i++)
	{
		vSpeedAdder += vSpeedInstant[i];
	}
	vSpeedAvg = (int16_t)(vSpeedAdder / (int32_t)numberOfVSpeedSamples);

	lastRunTime = time;
	lastAltitude = thisAltitude;
	eeprom_write_block(vSpeedInstant, &EEvSpeedHolderSamples, sizeof(int16_t)*VSPEEDSAMPLESDESIRED);



	scheduleQueueAdd(&calculateVspeed, time+5);
}

void timedCutdown(uint32_t time)
{
    lprintf_P(PSTR("Cutdown"));
	//BEFORE FLIGHT
	i2cSendStart();
    i2cWaitForComplete();
    i2cSendByte(0x10);
    i2cWaitForComplete();
    i2cSendByte(0x99);
    i2cWaitForComplete();
    i2cSendStop();
	//Send Comm Controller the cutdown command
	//In response, will  I receive the Cutdown Now command?
}


int16_t babySitVertSpeed;
int16_t currentTargetVspeed;
uint32_t lastBallastTime;
void autoBallast(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("In autoBallast"));
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
				lprintf_P(PSTR("Babysit Enable!"));
			#endif
		if(vSpeedAvg > (currentTargetVspeed + babySitVertSpeed)/2)
		{
			#ifdef FCPUDEBUG
				//lprintf_P(PSTR("Babysit: 1/2 T"));
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
					//lprintf_P(PSTR("Blst Error"));
				#endif
				statusCode = (statusCode & 0xFFFD) | (1 << 1);
			} else {
				statusCode = (statusCode & 0xFFFD);
			}
			scheduleQueueAdd(&autoBallast, time+60);
			ballastBabySit = 0;
		} else {
			#ifdef FCPUDEBUG
				//lprintf_P(PSTR("Babysit: Waiting"));
			#endif
			scheduleQueueAdd(&autoBallast, time+10);
		}
	} else {

		#ifdef FCPUDEBUG
			//lprintf_P(PSTR("No babysity"));
		#endif

		//If we're above the safety threshold
		if(ballastDisabled != 1)
		{


			//if current altitude is below target altitude AND vertical Velocity is above target
				//Switch to positive target velocity
			//else if current altitude is above target altitude
				//use negative target velocity
			//else if current altitude is below target altitude AND vertical velocity is below target
				//use zero target
			if(thisAltitude <= targetAltitude)
			{
				#ifdef FCPUDEBUG
					//lprintf_P(PSTR("Ballast: TVSpeed+"));
				#endif
				currentTargetVspeed = eeprom_read_word((uint16_t *)&EEballastTargetPositiveVSpeed);
			} else if(thisAltitude > targetAltitude)
			{
				#ifdef FCPUDEBUG
					//lprintf_P(PSTR("Ballast: TVSpeed-"));
				#endif
				currentTargetVspeed = eeprom_read_word((uint16_t *)&EEballastTargetNegativeVSpeed);
			} /*else if(thisAltitude < targetAltitude && vSpeedAvg < currentTargetVspeed)
			{
				#ifdef FCPUDEBUG
					//lprintf_P(PSTR("Ballast: TVSpeed0"));
				#endif
				currentTargetVspeed = 0;
			}*/

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
						lprintf_P(PSTR("Blst Error"));
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
				lprintf_P(PSTR("Ballast Disabled"));
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
		lprintf_P(PSTR("In Data Collect"));
	#endif
	char sampleString[SAMPLESTRINGSIZEINCHARS+1];
	memset(sampleString, 0x00, SAMPLESTRINGSIZEINCHARS);
	//get time
	uint32_t epochNow = now();
    lprintf_P(PSTR("E: %lu"), epochNow);
	lprintf("1 ");
	//get ambient pressure
	long myPressure;
	long myTemp;
	bmp085Convert(&myTemp, &myPressure);
	lprintf("2 ");
	//get FC temp
	uint16_t rawFCTemp;
	int8_t internalTemp;
	rawFCTemp = tmp100rawTemp(TMP100FC)>>4;
	int16_t fctinm = get12bit2scomp(rawFCTemp);
	internalTemp = (int8_t)(fctinm/16);
	//Convert to 8 bit
	lprintf("3 ");
	//get ext temp
	uint16_t rawExtTemp = tmp100rawTemp(TMP100EXT)>>4;
	int16_t externalTemperature = get12bit2scomp(rawExtTemp);
    lprintf("4 ");
	//get battetry temp
	uint16_t rawBattTemp = tmp100rawTemp(TMP101BH)>>4;
	int16_t btinm = get12bit2scomp(rawBattTemp);
	int8_t batteryTemperature = (int8_t)(btinm/16);
	//conver to 8 bit
    lprintf("5 ");

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
    lprintf("6 ");
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
	lprintf("7 ");
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
	_delay_ms(5);

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
    lprintf("8 ");  //POTENTIAL FREEZE SPOT AFTER THIS.
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
    lprintf("9 ");
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
    lprintf("10 ");
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
	lprintf("11 ");
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
   // lprintf("12 ");
	uint8_t ssLen = strlen(sampleString);
	uint8_t k;
	for(k = ssLen; k < (SAMPLESTRINGSIZEINCHARS - 2); k++)
	{
		sampleString[k] = '.';
	}
	//lprintf("pw: %d", k);
    lprintf("13 ");
	sampleString[SAMPLESTRINGSIZEINCHARS-2] = '\r';
	sampleString[SAMPLESTRINGSIZEINCHARS-1] = '\n';
	sampleString[SAMPLESTRINGSIZEINCHARS] = '\0';

    lprintf("in: ");
    for(int i = 0; i < 10; i++)
    {
        lprintf("%c", sampleString[i]);
    }

	//store in openlog
	#ifdef FCPUDEBUG
	lprintf("SS: ");
	//lprintf("%s", sampleString);
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
			lprintf_P(PSTR("Generating Report"));
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

		uint32_t rightNow = now();
		packet1[0] = lon_code>>16;
		packet1[1] = lon_code>>8;
		packet1[2] = lon_code;
		packet1[3] = (currentPositionData.altitude/50) >> 2;
		packet1[4] = (((currentPositionData.altitude/50) << 6) & 0xC0) | (((rightNow / 60) >> 8) & 0x0F);
		packet1[5] = (rightNow/60);


		packet2[0] = lat_code>>16;
		packet2[1] = lat_code>>8;
		packet2[2] = lat_code;
		packet2[3] = currentPositionData.speed;
		packet2[4] = (1<<4) | (((rightNow / 60) >> 8) & 0x0F);
		packet2[5] = (rightNow/60);

		if(currentPositionData.status != 0)
        {
            packet2[4] |= _BV(5);
        }

        //lprintf("N: %lu", rightNow);
        //lprintf("T1: %u T2: %u", ((packet1[4] & 0xF) <<8) + packet1[5], ((packet2[4] & 0xF)<<8) + packet2[5]);

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
        if(time != 0xFFFFFFFF)
        {
            scheduleQueueAdd(&transmitShortReport, time+desiredTX);
        }
	}
}

void transmitSamples(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("Sample TX"));
	#endif
	loadBatch();
	flushSatQueue();
	uint16_t maxTX = eeprom_read_word(&EEmaxAllowableTXInterval);
	uint16_t desiredTX = eeprom_read_word(&EEdataTransmitInterval);

    if(time == 0xFFFFFFFF)
    {

    } else{
        if(maxTX > desiredTX)
            scheduleQueueAdd(&transmitSamples, time+desiredTX);
        else
            scheduleQueueAdd(&transmitSamples, time+maxTX);
    }

}

void ballastStaticTickle(uint32_t time)
{
	#ifdef FCPUDEBUG
		lprintf_P(PSTR("bllst tckl"));
	#endif
	if((time - lastBallastTime) > 1800)
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
				//lprintf_P(PSTR("Ballast Error"));
			#endif
			statusCode = (statusCode & 0xFFFD) | (1 << 1);
		} else {
			statusCode = (statusCode & 0xFFFD);
		}
	}
	scheduleQueueAdd(&ballastStaticTickle, time+1800);
}

//1 implies that we have cutdown

void flightPhaseLogic(uint32_t time)
{
	#ifdef FCPUDEBUG
		//lprintf_P(PSTR("In Phase Logic"));
	#endif
	uint8_t currentPhase = eeprom_read_byte(&EEflightPhase);
	struct gpsData myGPS = currentPositionData;


	//Failure Condition: We will enter mayday mode.
	uint16_t thisAltitude = myGPS.altitude;

	uint16_t maydayAltitude = eeprom_read_word(&EEmaydayAltitude);
	int16_t maydayVSpeed = eeprom_read_word((uint16_t *)&EEmaydayVSpeed);
	uint8_t myPhase = currentPhase & 0x0F;
	uint8_t myFlags = currentPhase >> 4;
	switch(myPhase)
	{

		case 0:
			if(((PIND & _BV(TAKEOFFPIN))>>TAKEOFFPIN == 1) && ((myFlags & 1) == 1))
			{
				#ifdef FCPUDEBUG
					_delay_ms(500);
					//lprintf_P(PSTR("Where am I going?! Entering Phase 1"));
				#endif
				myPhase = 1;
				_delay_ms(500);
				//Save time
			}
			#ifdef FCPUDEBUG
				//lprintf_P(PSTR("Phase 0: Pre"));

			#endif
			//reschedule 1 second from now
			scheduleQueueAdd(&flightPhaseLogic, time+1);
			break;
		case 1:
			//reschedule 1 minute from now
			if(((myGPS.altitude > 8500) && (vSpeedAvg < 0) && ((myFlags & 1) == 1)) || (cutdownStatus == 1))
			{
				myPhase = 2;
			}
			scheduleQueueAdd(&flightPhaseLogic, time+10);
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 1: Climb"));
			#endif
			break;
		case 2:
			//make sure sat is enabled in here! BEFORE FLIGHT
			if(((vSpeedAvg < maydayVSpeed) || (thisAltitude < maydayAltitude)  || (cutdownStatus == 1)) && ((myFlags & 1) == 1))
			{
			    #ifdef FCPUDEBUG
                    lprintf_P(PSTR("Phase 3 Criteria"));
                    //lprintf_P(PSTR("VS: %d A: %d"), vSpeedAvg, thisAltitude);
                #endif
				myPhase = 3;
			}
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 2: Cruise"));
			#endif
			scheduleQueueAdd(&flightPhaseLogic, time+30);
			break;
		case 3:
			//disable sat in here! BEFORE FLIGHT
			if((vSpeedAvg > maydayVSpeed) && (thisAltitude > maydayAltitude) && (cutdownStatus == 0) && ((myFlags & 1) == 1))
			{
					myPhase = 2;
			}
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 3: FALLING"));
			#endif
			scheduleQueueAdd(&flightPhaseLogic, time+1);
			break;
		case 4:
			//reset HF and Sat Sample and Transmit Intervals to 1 hour
			scheduleQueueAdd(&flightPhaseLogic, time+3600);
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("Phase 4: Land"));
			#endif
			break;
		default:
			#ifdef FCPUDEBUG
				lprintf_P(PSTR("ERR!"));
			#endif
			break;
		}

	eeprom_write_byte(&EEflightPhase, (myFlags << 4) + myPhase);

}

void resetWatchdog(uint32_t time)
{

    wdt_reset();

    #ifdef FCPUDEBUG
        //lprintf_P(PSTR("Reset WDT"));
    #endif

    scheduleQueueAdd(&resetWatchdog, time+1);

}

void incrementEpoch(uint32_t time)
{
    eeprom_write_dword(&EEepochOffset, time);
    scheduleQueueAdd(&incrementEpoch, time+1);
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

}

void get_mcusr(void)
{
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}
