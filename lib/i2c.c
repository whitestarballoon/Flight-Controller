//*****************************************************************************
//
// File Name		: 'i2c.c'
// Title			: I2C interface using AVR Two-Wire Interface (TWI) hardware
// Original Author	: Pascal Stang
// Modified By		: Brad Luyster, LVL1 White Star Balloon Project
// Created			: 2002.06.25
// Modified			: 2010.10.21
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
// Improvement Notes:
// Need to tuse the TW Address Mask Register, and double check the address that
// was actually transmitted.
// Make sure the ghetto semaphor (I2cStatus) works.
// Document these functions, and how they MUST be used in order to work
// Properly.
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>

#include <util/delay.h>
#include <util/twi.h>

#include "i2c.h"

// Standard I2C bit rates are:
// 100KHz for slow speed
// 400KHz for high speed

//#define I2C_DEBUG

#define WRITE_sda() DDRC = DDRC | 0b00010000 //SDA must be output when writing
#define READ_sda()  DDRC = DDRC & 0b11101111 //SDA must be input when reading - don't forget the resistor on SDA!!

// I2C state and address variables
static volatile eI2cStateType I2cState;
static uint8_t I2cDeviceAddrRW;
static uint8_t I2cWhiteStarGeneralAddr;
static uint8_t I2cWhiteStarGeneralEn;
// send/transmit buffer (outgoing data)
static uint8_t I2cSendData[I2C_SEND_DATA_BUFFER_SIZE];
static uint8_t I2cSendDataIndex;
static uint8_t I2cSendDataLength;
// receive buffer (incoming data)
static uint8_t I2cReceiveData[I2C_RECEIVE_DATA_BUFFER_SIZE];
static uint8_t I2cReceiveDataIndex;
static uint8_t I2cReceiveDataLength;

// function pointer to i2c receive routine
//! I2cSlaveReceive is called when this processor
// is addressed as a slave for writing
static void (*i2cSlaveReceive)(uint8_t receiveDataLength, uint8_t* recieveData);
//! I2cWSBReceive is called when this processor
// is addressed by the WSB Bus General Call
static void (*i2cWSBReceive)(uint8_t receiveDataLength, uint8_t* recieveData);
//! I2cSlaveTransmit is called when this processor
// is addressed as a slave for reading
static uint8_t (*i2cSlaveTransmit)(uint8_t transmitDataLengthMax, uint8_t* transmitData);

// functions
void i2cInit(void)
{
	/*// set pull-up resistors on I2C bus pins
	// TODO: should #ifdef these
	sbi(PORTC, 0);	// i2c SCL on ATmega163,323,16,32,etc
	sbi(PORTC, 1);	// i2c SDA on ATmega163,323,16,32,etc
	sbi(PORTD, 0);	// i2c SCL on ATmega128,64
	sbi(PORTD, 1);	// i2c SDA on ATmega128,64*/

	// clear SlaveReceive and SlaveTransmit handler to null
	i2cSlaveReceive = 0;
	i2cSlaveTransmit = 0;
	// set i2c bit rate to 100KHz
	i2cSetBitrate(100);
	// enable TWI (two-wire interface)
	TWCR |= _BV(TWEN);
	// set state
	I2cState = I2C_IDLE;
	// enable TWI interrupt and slave address ACK
	TWCR |= _BV(TWIE) | _BV(TWEA);
	//outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	// enable interrupts
	sei();
}

//For some reason, the TWBR calculation doesn't work out properly, so we're
//going to set it ourself, damnit
void i2cSetTheDamnTWBRMyself(uint8_t myOwnTWBR)
{

	TWBR = myOwnTWBR;

}

void i2cSetBitrate(uint16_t bitrateKHz)
{
	uint8_t bitrate_div;
	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*TWBR))
	#ifdef TWPS0
		// for processors with additional bitrate division (mega12)
		// SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
		// set TWPS to zero
		TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
	#endif
	// calculate bitrate division
	//printf("1: %d\n", (uint16_t)(799./100.));
	bitrate_div = (uint8_t)(((float)F_CPU/(float)10001))/(float)bitrateKHz;
	if(bitrate_div >= 16)
		bitrate_div = (bitrate_div-16)/2;
	//printf("bitrate: %d\n", bitrate_div);
	//printf("k: %d\n", bitrateKHz);
	TWBR = bitrate_div;

	//TWSR |= _BV(TWPS0) | _BV(TWPS1);
	//TWBR = 97;

}

//Mask is right-adjusted
void i2cSetLocalDeviceAddr(uint8_t deviceAddr, uint8_t maskAddr, uint8_t genCallEn, uint8_t wsGenCallEn, uint8_t wsGenCallAddr)
{
	// set local device address (used in slave mode only)
	TWAR =  ((deviceAddr&0xFE) | (genCallEn?1:0));
	TWAMR = maskAddr << 1;
	I2cWhiteStarGeneralEn = wsGenCallEn;
	I2cWhiteStarGeneralAddr = wsGenCallAddr;

}

void i2cSetSlaveReceiveHandler(void (*i2cSlaveRx_func)(uint8_t receiveDataLength, uint8_t* recieveData))
{
	i2cSlaveReceive = i2cSlaveRx_func;
}

void i2cSetSlaveTransmitHandler(uint8_t (*i2cSlaveTx_func)(uint8_t transmitDataLengthMax, uint8_t* transmitData))
{
	i2cSlaveTransmit = i2cSlaveTx_func;
}

inline void i2cSendStart(void)
{
	WRITE_sda();
	// send start condition
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
}

inline void i2cSendStop(void)
{
	// transmit stop condition,
	// leave with TWEA on for slave receiving
	TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWSTO) | _BV(TWEN);
}

inline void i2cDisableInt(void)
{
	// disable TWI interrupt
	TWCR &= ~_BV(TWIE);
}

inline void i2cEnableInt(void)
{
	TWCR  |= _BV(TWIE);
}

uint8_t i2cWaitForComplete(void)
{
	// wait for i2c interface to complete operation
	volatile uint8_t i = 0;
	while( !(TWCR & _BV(TWINT) ) && i < 100 )
	{
		i++;
		_delay_us(150);
	}
	if(i >= 98)
	{
		return 1;
	}
	return 0;
}

inline void i2cSendByte(uint8_t data)
{
	WRITE_sda();
	// save data to the TWDR
	TWDR = data;
	// begin send
	TWCR = _BV(TWINT)|_BV(TWEN);
}

inline void i2cReceiveByte(uint8_t ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT) | _BV(TWEA);
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT);
	}
}

inline uint8_t i2cGetReceivedByte(void)
{
	// retieve received data byte from i2c TWDR
	return( TWDR );
}

inline uint8_t i2cGetStatus(void)
{
	// retieve current i2c status from i2c TWSR
	return( TWSR );
}

void i2cMasterSend(uint8_t deviceAddr, uint8_t length, uint8_t* data)
{
	uint8_t i;
	// wait for interface to be ready
	while(I2cState);
	// set state
	I2cState = I2C_MASTER_TX;
	// save data
	I2cDeviceAddrRW = (deviceAddr & 0xFE);	// RW cleared: write operation
	for(i=0; i<length; i++)
		I2cSendData[i] = *data++;
	I2cSendDataIndex = 0;
	I2cSendDataLength = length;
	// send start condition
	i2cSendStart();
}

void i2cMasterReceive(uint8_t deviceAddr, uint8_t length, uint8_t* data)
{
	uint8_t i;
	// wait for interface to be ready
	while(I2cState);
	// set state
	I2cState = I2C_MASTER_RX;
	// save data
	I2cDeviceAddrRW = (deviceAddr|0x01);	// RW set: read operation
	I2cReceiveDataIndex = 0;
	I2cReceiveDataLength = length;
	// send start condition
	i2cSendStart();
	// wait for data
	while(I2cState);
	// return data
	for(i=0; i<length; i++)
		*data++ = I2cReceiveData[i];
}

uint8_t i2cMasterSendNI(uint8_t deviceAddr, uint8_t length, uint8_t* data)
{
	uint8_t retval = I2C_OK;

	// disable TWI interrupt
	TWCR &= ~_BV(TWIE);

	// send start condition
	i2cSendStart();
	i2cWaitForComplete();

	// send device address with write
	i2cSendByte( deviceAddr & 0xFE );
	i2cWaitForComplete();

	// check if device is present and live
	if( TWSR == TW_MT_SLA_ACK )
	{
		// send data
		while(length)
		{
			i2cSendByte( *data++ );
			i2cWaitForComplete();
			length--;
		}
	}
	else
	{
		// device did not ACK it's address,
		// data will not be transferred
		// return error
		retval = I2C_ERROR_NODEV;
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	i2cSendStop();
	while( !(TWCR & _BV(TWSTO)) );

	// enable TWI interrupt
	TWCR |= _BV(TWIE);

	return retval;
}

uint8_t i2cMasterReceiveNI(uint8_t deviceAddr, uint8_t length, uint8_t *data)
{
	uint8_t retval = I2C_OK;

	// disable TWI interrupt
	TWCR &= ~_BV(TWIE);

	// send start condition
	i2cSendStart();
	i2cWaitForComplete();

	// send device address with read
	i2cSendByte( deviceAddr | 0x01 );
	i2cWaitForComplete();

	// check if device is present and live
	if( TWSR == TW_MR_SLA_ACK)
	{
		// accept receive data and ack it
		while(length > 1)
		{
			i2cReceiveByte(TRUE);
			i2cWaitForComplete();
			*data++ = i2cGetReceivedByte();
			// decrement length
			length--;
		}

		// accept receive data and nack it (last-byte signal)
		i2cReceiveByte(FALSE);
		i2cWaitForComplete();
		*data++ = i2cGetReceivedByte();
	}
	else
	{
		// device did not ACK it's address,
		// data will not be transferred
		// return error
		retval = I2C_ERROR_NODEV;
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	i2cSendStop();

	// enable TWI interrupt
	TWCR |= _BV(TWIE);

	return retval;
}
/*
void i2cMasterTransferNI(uint8_t deviceAddr, uint8_t sendlength, uint8_t* senddata, uint8_t receivelength, uint8_t* receivedata)
{
	// disable TWI interrupt
	cbi(TWCR, TWIE);

	// send start condition
	i2cSendStart();
	i2cWaitForComplete();

	// if there's data to be sent, do it
	if(sendlength)
	{
		// send device address with write
		i2cSendByte( deviceAddr & 0xFE );
		i2cWaitForComplete();

		// send data
		while(sendlength)
		{
			i2cSendByte( *senddata++ );
			i2cWaitForComplete();
			sendlength--;
		}
	}

	// if there's data to be received, do it
	if(receivelength)
	{
		// send repeated start condition
		i2cSendStart();
		i2cWaitForComplete();

		// send device address with read
		i2cSendByte( deviceAddr | 0x01 );
		i2cWaitForComplete();

		// accept receive data and ack it
		while(receivelength > 1)
		{
			i2cReceiveByte(TRUE);
			i2cWaitForComplete();
			*receivedata++ = i2cGetReceivedByte();
			// decrement length
			receivelength--;
		}

		// accept receive data and nack it (last-byte signal)
		i2cReceiveByte(TRUE);
		i2cWaitForComplete();
		*receivedata++ = i2cGetReceivedByte();
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	i2cSendStop();
	while( !(inb(TWCR) & BV(TWSTO)) );

	// enable TWI interrupt
	sbi(TWCR, TWIE);
}
*/

//! I2C (TWI) interrupt service routine
ISR(TWI_vect)
{
	//lprintf("In I2C int.\n");
	// read status bits
	uint8_t status = TWSR & TW_STATUS_MASK;

	switch(status)
	{
	// Master General
	case TW_START:						// 0x08: Sent start condition
	case TW_REP_START:					// 0x10: Sent repeated start condition
		#ifdef I2C_DEBUG
		printf("I2C: M->START\r\n");
		#endif
		// send device address
		i2cSendByte(I2cDeviceAddrRW);
		break;

	// Master Transmitter & Receiver status codes
	case TW_MT_SLA_ACK:					// 0x18: Slave address acknowledged
	case TW_MT_DATA_ACK:				// 0x28: Data acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MT->SLA_ACK or DATA_ACK\r\n");
		#endif
		if(I2cSendDataIndex < I2cSendDataLength)
		{
			// send data
			i2cSendByte( I2cSendData[I2cSendDataIndex++] );
		}
		else
		{
			// transmit stop condition, enable SLA ACK
			i2cSendStop();
			// set state
			I2cState = I2C_IDLE;
		}
		break;
	case TW_MR_DATA_NACK:				// 0x58: Data received, NACK reply issued
		#ifdef I2C_DEBUG
		printf("I2C: MR->DATA_NACK\r\n");
		#endif
		// store final received data byte
		I2cReceiveData[I2cReceiveDataIndex++] = TWDR;
		// continue to transmit STOP condition
	case TW_MR_SLA_NACK:				// 0x48: Slave address not acknowledged
	case TW_MT_SLA_NACK:				// 0x20: Slave address not acknowledged
	case TW_MT_DATA_NACK:				// 0x30: Data not acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MTR->SLA_NACK or MT->DATA_NACK\r\n");
		#endif
		// transmit stop condition, enable SLA ACK
		i2cSendStop();
		// set state
		I2cState = I2C_IDLE;
		break;
	case TW_MT_ARB_LOST:				// 0x38: Bus arbitration lost
	//case TW_MR_ARB_LOST:				// 0x38: Bus arbitration lost
		#ifdef I2C_DEBUG
		printf("I2C: MT->ARB_LOST\r\n");
		#endif
		// release bus
		TWCR |= _BV(TWINT);
		// set state
		I2cState = I2C_IDLE;
		// release bus and transmit start when bus is free
		//outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWSTA));
		break;
	case TW_MR_DATA_ACK:				// 0x50: Data acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MR->DATA_ACK\r\n");
		#endif
		// store received data byte
		I2cReceiveData[I2cReceiveDataIndex++] = TWDR;
		// fall-through to see if more bytes will be received
	case TW_MR_SLA_ACK:					// 0x40: Slave address acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MR->SLA_ACK\r\n");
		#endif
		if(I2cReceiveDataIndex < (I2cReceiveDataLength-1))
			// data byte will be received, reply with ACK (more bytes in transfer)
			i2cReceiveByte(TRUE);
		else
			// data byte will be received, reply with NACK (final byte in transfer)
			i2cReceiveByte(FALSE);
		break;

	// Slave Receiver status codes
	case TW_SR_SLA_ACK:					// 0x60: own SLA+W has been received, ACK has been returned
	case TW_SR_ARB_LOST_SLA_ACK:		// 0x68: own SLA+W has been received, ACK has been returned
	case TW_SR_GCALL_ACK:				// 0x70:     GCA+W has been received, ACK has been returned
	case TW_SR_ARB_LOST_GCALL_ACK:		// 0x78:     GCA+W has been received, ACK has been returned
		#ifdef I2C_DEBUG
		printf("I2C: SR->SLA_ACK\r\n");
		#endif
		//If the address sent is the same as the address in our TWAR register, do stuff.
		//Otherwise, maintain the old state.
		if( (TWDR & 0xFE) == (TWAR & 0xFE) )
		{
			#ifdef I2C_DEBUG
			printf("I2C: Slave Address Directly\r\n");
			#endif
			// we are being addressed as slave for writing (data will be received from master)
			// set state
			I2cState = I2C_SLAVE_RX;
			// prepare buffer
			I2cReceiveDataIndex = 0;
			// receive data byte and return ACK
			TWCR |= _BV(TWINT) | _BV(TWEA);
		} else if ( ((TWDR & 0xFE) == I2cWhiteStarGeneralAddr) && I2cWhiteStarGeneralEn)
		{
			#ifdef I2C_DEBUG
			printf("I2C: WSB General Call\r\n");
			#endif
			//White Star Balloon General Call Mode, GO!
			I2cState = I2C_WSB_GENERAL;
			//Prepare Buffer
			I2cReceiveDataIndex = 0;
			//RX data, and ack
			TWCR |= _BV(TWINT) | _BV(TWEA);
		} else {
			#ifdef I2C_DEBUG
			printf("I2C: Not the Intended Recipient\r\n");
			#endif
		}
		break;
	case TW_SR_DATA_ACK:				// 0x80: data byte has been received, ACK has been returned
	case TW_SR_GCALL_DATA_ACK:			// 0x90: data byte has been received, ACK has been returned
		#ifdef I2C_DEBUG
		printf("I2C: SR->DATA_ACK\r\n");
		#endif
		// get previously received data byte
		I2cReceiveData[I2cReceiveDataIndex++] = TWDR;
		// check receive buffer status
		if(I2cReceiveDataIndex < I2C_RECEIVE_DATA_BUFFER_SIZE)
		{
			// receive data byte and return ACK
			i2cReceiveByte(TRUE);
			//outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
		}
		else
		{
			// receive data byte and return NACK
			i2cReceiveByte(FALSE);
			//outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
		}
		break;
	case TW_SR_DATA_NACK:				// 0x88: data byte has been received, NACK has been returned
	case TW_SR_GCALL_DATA_NACK:			// 0x98: data byte has been received, NACK has been returned
		#ifdef I2C_DEBUG
		printf("I2C: SR->DATA_NACK\r\n");
		#endif
		// receive data byte and return NACK
		i2cReceiveByte(FALSE);
		//outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
		break;
	case TW_SR_STOP:					// 0xA0: STOP or REPEATED START has been received while addressed as slave
		#ifdef I2C_DEBUG
		printf("I2C: SR->SR_STOP\r\n");
		#endif
		// switch to SR mode with SLA ACK
		TWCR |= _BV(TWINT) | _BV(TWEA);
		if(I2cState == I2C_WSB_GENERAL)
		{
			if(i2cWSBReceive) i2cWSBReceive(I2cReceiveDataIndex, I2cReceiveData);
		} else {
			// i2c receive is complete, call i2cSlaveReceive
			if(i2cSlaveReceive) i2cSlaveReceive(I2cReceiveDataIndex, I2cReceiveData);
		}
		// set state
		I2cState = I2C_IDLE;
		break;

	// Slave Transmitter
	case TW_ST_SLA_ACK:					// 0xA8: own SLA+R has been received, ACK has been returned
	case TW_ST_ARB_LOST_SLA_ACK:		// 0xB0:     GCA+R has been received, ACK has been returned
		#ifdef I2C_DEBUG
		printf("I2C: ST->SLA_ACK\r\n");
		#endif
		//If the address sent is the same as the address in our TWAR register, do stuff.
		//Otherwise, maintain the old state.
		if( (TWDR & 0xFE) == (TWAR & 0xFE) )
		{
			// we are being addressed as slave for reading (data must be transmitted back to master)
			// set state
			I2cState = I2C_SLAVE_TX;
			// request data from application
			if(i2cSlaveTransmit) I2cSendDataLength = i2cSlaveTransmit(I2C_SEND_DATA_BUFFER_SIZE, I2cSendData);
			// reset data index
			I2cSendDataIndex = 0;
			// fall-through to transmit first data byte
		} else {
			//Otherwise, don't fall through!
			break;
		}
	case TW_ST_DATA_ACK:				// 0xB8: data byte has been transmitted, ACK has been received
		#ifdef I2C_DEBUG
		printf("I2C: ST->DATA_ACK\r\n");
		#endif
		// transmit data byte
		TWDR = I2cSendData[I2cSendDataIndex++];
		if(I2cSendDataIndex < I2cSendDataLength)
			// expect ACK to data byte
			TWCR |= _BV(TWINT) | _BV(TWEA);
		else
			// expect NACK to data byte
			TWCR |= _BV(TWINT);
		break;
	case TW_ST_DATA_NACK:				// 0xC0: data byte has been transmitted, NACK has been received
	case TW_ST_LAST_DATA:				// 0xC8:
		#ifdef I2C_DEBUG
		printf("I2C: ST->DATA_NACK or LAST_DATA\r\n");
		#endif
		// all done
		// switch to open slave
		TWCR |= _BV(TWINT) | _BV(TWEA);
		// set state
		I2cState = I2C_IDLE;
		break;

	// Misc
	case TW_NO_INFO:					// 0xF8: No relevant state information
		// do nothing
		#ifdef I2C_DEBUG
		printf("I2C: NO_INFO\r\n");
		#endif
		break;
	case TW_BUS_ERROR:					// 0x00: Bus error due to illegal start or stop condition
		#ifdef I2C_DEBUG
		printf("I2C: BUS_ERROR\r\n");
		#endif
		// reset internal hardware and release bus
		TWCR |= _BV(TWINT) | _BV(TWSTO) | _BV(TWEA);
		// set state
		I2cState = I2C_IDLE;
		break;
	}
}

eI2cStateType i2cGetState(void)
{
	return I2cState;
}
