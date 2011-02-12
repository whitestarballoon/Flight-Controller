//*****************************************************************************
//
// File Name		: 'ds3231.h'
// Original Author	: Brad Luyster, LVL1 White Star Balloon Project
//
// http://www.whitestarballoon.com/
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#ifndef DS3231_H
#define DS3231_H

#define DS3231FC 0b11010000

uint8_t getTime(uint8_t *, uint8_t *, uint8_t *, uint8_t *);
uint8_t getSeconds(void);
uint8_t getMinutes(void);
uint8_t getHours(void);
uint8_t getDays(void);
void ds3231write(uint8_t, uint8_t);
uint8_t ds3231read(uint8_t);

#endif
