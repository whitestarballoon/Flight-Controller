//*****************************************************************************
//
// File Name		: 'progmemVars.h'
// Title			: List of Program Memory Variables
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

#include <avr/pgmspace.h>

#ifndef PROGMEMVARS_H
#define PROGMEMVARS_H

//const uint8_t bitmaskTypeOrder[] PROGMEM = {32,16,16,8,16,16,16,8,33,33,8,96,33,96,33,64,16,48,16,48,8,34,34,8,48,33,48,33,33,16,8,16,16,16,16};
const uint8_t bitmaskTypeOrder[] PROGMEM = {32,16,16,8,16,16,16,8,16,16,8,96,33,96,33,64,16,48,16,48,8,34,34,8,48,8,48,8,16,8,8,16,16,16,16};

#endif
