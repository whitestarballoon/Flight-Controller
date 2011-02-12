//*****************************************************************************
//
// File Name		: 'epoch.h'
// Original Author	: Brad Luyster, LVL1 White Star Balloon Project
//
// http://www.whitestarballoon.com/
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#ifndef EPOCH_H
#define EPOCH_H

void defaultEEPROM(void);
uint32_t getEpochSeconds(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t days);
void writeEpochStart(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t days);

#endif
