//*****************************************************************************
//
// File Name		: 'i2c.h'
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

#ifndef I2C_H
#define I2C_H

#define TRUE 1
#define FALSE 0
//#include "global.h"

// include project-specific configuration
//#include "i2cconf.h"

/*// TWSR values (not bits)
// (taken from avr-libc twi.h - thank you Marek Michalkiewicz)
// Master
#define TW_START					0x08
#define TW_REP_START				0x10
// Master Transmitter
#define TW_MT_SLA_ACK				0x18
#define TW_MT_SLA_NACK				0x20
#define TW_MT_DATA_ACK				0x28
#define TW_MT_DATA_NACK				0x30
#define TW_MT_ARB_LOST				0x38
// Master Receiver
#define TW_MR_ARB_LOST				0x38
#define TW_MR_SLA_ACK				0x40
#define TW_MR_SLA_NACK				0x48
#define TW_MR_DATA_ACK				0x50
#define TW_MR_DATA_NACK				0x58
// Slave Transmitter
#define TW_ST_SLA_ACK				0xA8
#define TW_ST_ARB_LOST_SLA_ACK		0xB0
#define TW_ST_DATA_ACK				0xB8
#define TW_ST_DATA_NACK				0xC0
#define TW_ST_LAST_DATA				0xC8
// Slave Receiver
#define TW_SR_SLA_ACK				0x60
#define TW_SR_ARB_LOST_SLA_ACK		0x68
#define TW_SR_GCALL_ACK				0x70
#define TW_SR_ARB_LOST_GCALL_ACK	0x78
#define TW_SR_DATA_ACK				0x80
#define TW_SR_DATA_NACK				0x88
#define TW_SR_GCALL_DATA_ACK		0x90
#define TW_SR_GCALL_DATA_NACK		0x98
#define TW_SR_STOP					0xA0
// Misc
#define TW_NO_INFO					0xF8
#define TW_BUS_ERROR				0x00
*/
// defines and constants
#define TWCR_CMD_MASK		0x0F
#define TWSR_STATUS_MASK	0xF8

// return values
#define I2C_OK				0x00
#define I2C_ERROR_NODEV		0x01

// define I2C data buffer sizes
// These buffers are used in interrupt-driven Master sending and receiving,
// and in slave sending and receiving.  They must be large enough to store
// the largest I2C packet you expect to send and receive, respectively.
#define I2C_SEND_DATA_BUFFER_SIZE		0x20
#define I2C_RECEIVE_DATA_BUFFER_SIZE	0x20


// types
typedef enum
{
	I2C_IDLE = 0, I2C_BUSY = 1,
	I2C_MASTER_TX = 2, I2C_MASTER_RX = 3,
	I2C_SLAVE_TX = 4, I2C_SLAVE_RX = 5,
	I2C_WSB_GENERAL = 6
} eI2cStateType;

// functions

//! Initialize I2C (TWI) interface
void i2cInit(void);

// Set the I2C transaction bitrate (in KHz)
void i2cSetBitrate(uint16_t bitrateKHz);

// I2C setup and configurations commands
//! Set the local (AVR processor's) I2C device address
void i2cSetLocalDeviceAddr(uint8_t deviceAddr, uint8_t maskAddr, uint8_t genCallEn, uint8_t wsGenCallEn, uint8_t wsGenCallAddr);

//! Set the user function which handles receiving (incoming) data as a slave
void i2cSetSlaveReceiveHandler(void (*i2cSlaveRx_func)(uint8_t receiveDataLength, uint8_t* recieveData));
//! Set the user function which handles transmitting (outgoing) data as a slave
void i2cSetSlaveTransmitHandler(uint8_t (*i2cSlaveTx_func)(uint8_t transmitDataLengthMax, uint8_t* transmitData));

// Low-level I2C transaction commands 
//! Send an I2C start condition in Master mode
void i2cSendStart(void);
//! Send an I2C stop condition in Master mode
void i2cSendStop(void);
//! Wait for current I2C operation to complete
uint8_t i2cWaitForComplete(void);
//! Send an (address|R/W) combination or a data byte over I2C
void i2cSendByte(uint8_t data);
//! Receive a data byte over I2C  
// ackFlag = TRUE if recevied data should be ACK'ed
// ackFlag = FALSE if recevied data should be NACK'ed
void i2cReceiveByte(uint8_t ackFlag);
//! Pick up the data that was received with i2cReceiveByte()
uint8_t i2cGetReceivedByte(void);
//! Get current I2c bus status from TWSR
uint8_t i2cGetStatus(void);
void i2cDisableInt(void);
void i2cEnableInt(void);

void i2cSetTheDamnTWBRMyself(uint8_t myOwnTWBR);

// high-level I2C transaction commands

//! send I2C data to a device on the bus
void i2cMasterSend(uint8_t deviceAddr, uint8_t length, uint8_t *data);
//! receive I2C data from a device on the bus
void i2cMasterReceive(uint8_t deviceAddr, uint8_t length, uint8_t* data);

//! send I2C data to a device on the bus (non-interrupt based)
uint8_t i2cMasterSendNI(uint8_t deviceAddr, uint8_t length, uint8_t* data);
//! receive I2C data from a device on the bus (non-interrupt based)
uint8_t i2cMasterReceiveNI(uint8_t deviceAddr, uint8_t length, uint8_t *data);

//! Get the current high-level state of the I2C interface
eI2cStateType i2cGetState(void);

#endif
