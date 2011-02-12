//*****************************************************************************
//
// File Name		: 'gps.h'
// Original Author	: Brad Luyster, LVL1 White Star Balloon Project
//
// http://www.whitestarballoon.com/
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

//*****************************************************************************
// TODO
//
// Encapsulate in IFDEF to save from redeclaration.
//*****************************************************************************

#include "../dataStructures.h"

extern uint8_t uart_getchar(void);
extern int lprintf(char *, ...);
extern int lprintf_P(const char *, ...);

void getGPS(struct gpsData *outputData);
void debugPrintRawStrings(void);
