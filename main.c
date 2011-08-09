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
#include <avr/wdt.h>

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
//#define MAKERFAIRE_GPS_DEBUG
#define MAKERFAIRE_DEMO_NOTX
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

enum makerfaireMode { positive, negative };
enum makerfaireMode globalMode;

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
void resetWatchdog(uint32_t);
void turnHfOn(uint32_t);
void turnHfOff(uint32_t);
void incrementEpoch(uint32_t);
void updateSpeedDial(uint16_t speedDial);

//Vspeed Calculation Variables
int16_t vSpeedAvg = 0;
uint8_t numberOfVSpeedSamples=0;
uint16_t lastAltitude=0;
uint16_t lastRunTime=0;
extern int16_t EEMEM EEvSpeedHolderSamples;

//Globals
uint8_t enableReports = 1;
uint8_t reportCounterL=0;
uint8_t reportCounterH=0;
uint16_t statusCode = 0x00;
uint8_t hfSema = 0;

uint32_t epochOffset;

//HOLY CRAP YOU ARE STUPID PUT THESE SOMEWHERE THAT MAKES SENSE SO YOU DON'T HAVE TO EXTERN THEM
//IN YOUR FREAKING MAIN ROUTINE
extern uint32_t EEMEM EEepochOffset;
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
extern uint16_t EEMEM EEhfTimeToTx;
extern uint8_t EEMEM EEhfLenngthToTx;

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
//Watchdog Vars
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));

//======================



int main (void)
{
	uint8_t seconds,  minutes, hours, days;
	uint8_t error;

    //wdt_disable();

	ioinit(); //Setup IO pins and defaults
	i2cSetTheDamnTWBRMyself(10);

	i2cSetLocalDeviceAddr(0b00001010, 0x00, 0, 0, 0);
	i2cEnableInt();
	sei();

	_delay_ms(500);

	lprintf("WSB CPU Alive\n");

    #ifdef FCPUDEBUG
		lprintf_P(PSTR("Set Epoch\n"));
	#endif
	wdt_enable(WDTO_8S);
	if(eeprom_read_byte(&EEEpochLock) == 0)
	{
		eeprom_write_dword(&EEepochOffset, 0);
	}
	//error = getTime(&seconds, &minutes, &hours, &days);

	uint32_t offset = eeprom_read_dword(&EEepochOffset);
	uint8_t offsetDays = offset / 86400;
	uint8_t offsetHours = (offset - (offsetDays * 86400)) / 3600;
	uint8_t offsetMinutes = (offset - (offsetDays * 86400) - (offsetHours * 3600)) / 60;
	uint8_t offsetSeconds = (offset - (offsetDays * 86400) - (offsetHours * 3600) - (offsetMinutes * 60));
    ds3231write(0x00, offsetSeconds);
    ds3231write(0x01,  offsetMinutes);
    ds3231write(0x02,  offsetHours);
    ds3231write(0x03,  offsetDays);

    eeprom_write_word(&EEcurrentBatchNumber, 0);
    eeprom_write_word(&EEbatchSampleStart, 0);
    eeprom_write_word(&EEbatchSampleEnd, 0);
    writeEpochStart(0, 0, 0, 0);

	lprintf("Still Alive\n");

	// TMP100
	setTMP100config(TMP100FC, 0xE0);

	lprintf("TMPCONF Complete\n");
	//BMP085_Calibration();
	// BMP085 END
	//defaultEEPROM();
	/*if(eeprom_read_byte(&EEEpochLock) == 0)
	{
		//defaultEEPROM();
		
	} else {
		lprintf_P(PSTR("OpenLog Flight Init\n"));
		initOpenLogFlight();
	}*/

	lprintf_P(PSTR("OpenLog Reset Init\n"));
    initOpenLogTest();


	lprintf("SSAlive\n");

    if((mcusr_mirror & 0x08) == 0x08)
    {
        lprintf("WDTReset\n");
    }

    #ifdef MAKERFAIRE_GPS_DEBUG
    while(1)
    {
    	wdt_disable();
    	debugPrintRawStrings();
    }
    #endif

	uint32_t rnow = now();

	while(1)
	{

		long myPressure;
		long myTemp;
		bmp085Convert(&myTemp, &myPressure);

		struct gpsData currentPositionData;
		getGPS(&currentPositionData);

		char dataSample[100];

		sprintf_P(PSTR("Temp: %ld Pres: %ld Altitude: %d Lat: %f Lon %f\n"), myTemp, myPressure,
						currentPositionData.altitude, currentPositionData.latitude, 
						currentPositionData.longitude);

		red_on();
		_delay_ms(500);
		red_off();
		_delay_ms(500);
	}


    return(0);
}




void resetWatchdog(uint32_t time)
{

    wdt_reset();

    #ifdef FCPUDEBUG
        //lprintf_P(PSTR("Reset WDT\n"));
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
	//lprintf("Now: %lu\n", getEpochSeconds(seconds, minutes, hours, days));
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

void get_mcusr(void)
{
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}
