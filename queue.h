//*****************************************************************************
//
// File Name		: 'queue.h'
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
// - Figure out a Sane Value for the Queuedepth
//*****************************************************************************

#ifndef QUEUE_H
#define QUEUE_H

#include "dataStructures.h"

#define QUEUEDEPTH 12

extern int lprintf(char *, ...);
void scheduleQueueAdd(scheduledFunction thisFunction, uint32_t epochTime);
int8_t scheduleQueueGetTop(scheduledFunction *returnFunction, uint32_t *returnTime);


#endif
