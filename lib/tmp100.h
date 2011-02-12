//*****************************************************************************
//
// File Name		: 'tmp100.h'
// Original Author	: Brad Luyster, LVL1 White Star Balloon Project
//
// http://www.whitestarballoon.com/
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

uint16_t tmp100rawTemp(uint8_t);
uint8_t getTMP100config(uint8_t);
uint8_t setTMP100config(uint8_t, uint8_t);
uint8_t setTMP101Thermo(uint8_t, uint8_t, uint8_t);
int16_t get12bit2scomp(uint16_t);
uint16_t set12bit2scomp(int16_t);
