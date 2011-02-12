//*****************************************************************************
//
// File Name		: 'queue.c'
// Title			: Queuing system for executing tasks at a given scheduled
//						time.
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

//*****************************************************************************
// TODO
//
// - Failure mode for full queue
// - Flush Queue function
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <avr/io.h>

#include "dataStructures.h"
#include "queue.h"

scheduledFunction scheduledFunctionArray[QUEUEDEPTH] = {NULL};
uint32_t scheduledFunctionTimes[QUEUEDEPTH];

//This is a circular buffer.  Initialize start and end this way.
uint8_t queueStart=0;
uint8_t queueEnd=0;


void scheduleQueueAdd(scheduledFunction thisFunction, uint32_t epochTime)
{
	//Honestly, this algorithm is straight from wikipedia.
	if ((queueEnd + 1) % QUEUEDEPTH != queueStart) {
                scheduledFunctionArray[queueEnd] = thisFunction;
                scheduledFunctionTimes[queueEnd] = epochTime;
                queueEnd = (queueEnd + 1) % QUEUEDEPTH;
    } else {
		lprintf("Q BUF FULL\n");
	}
	
}

int8_t scheduleQueueGetTop(scheduledFunction *returnFunction, uint32_t *returnTime)
{
        if (queueEnd != queueStart) {
				
                scheduledFunction tempF = scheduledFunctionArray[queueStart];
                uint32_t tempT = scheduledFunctionTimes[queueStart];
                //This Algorithm too.
                queueStart = (queueStart + 1) % QUEUEDEPTH;
                *returnFunction = tempF;
                *returnTime = tempT;
                return 0;
        }
        //otherwise, the buffer is empty; return an error code
        return 1;
}

