//*****************************************************************************
//
// File Name		: 'openlog.h'
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

void initOpenLogTest(void);
void initOpenLogFlight(void);
void putDataSample(char *);
void getDataSample(uint16_t, char *);

int ol_putchar(char c, FILE *stream);
uint8_t ol_getchar(void);
