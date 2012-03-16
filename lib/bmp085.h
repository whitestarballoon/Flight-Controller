//*****************************************************************************
//
// File Name		: 'bmp085.h'
// Original Author	: Brad Luyster, LVL1 White Star Balloon Project
//
// http://www.whitestarballoon.com/
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#define BMP085FC 0xEE
#define OSS 0

void BMP085_Calibration(void);
short bmp085ReadShort(unsigned char address);
unsigned long bmp085ReadTemp(void);
long bmp085ReadPressure(void);
void bmp085Convert(long* temperature, long* pressure);

