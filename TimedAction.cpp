/********************************************************************/
/*    Timed Action Implementation                                   */
/*    TimedAction.cpp                                               */
/*                                                                  */
/*    Author:        Alexander Brevig                               */
/*    Contributors:  Jacob Moroni (jakemoroni@gmail.com)            */
/*                                                                  */
/*    Description:   This file implements the TimedAction           */
/*                   library (C++).                                 */
/*                                                                  */
/*    Revision:      Oct 15, 2015 (Modified by JMoroni)             */
/*                                                                  */
/*    This library is free software; you can redistribute it and/or */
/*    modify it under the terms of the GNU Lesser General Public    */
/*    License as published by the Free Software Foundation; either  */
/*    version 2.1 of the License, or (at your option) any later     */
/*    version.                                                      */
/*    This library is distributed in the hope that it will be       */
/*    useful, but WITHOUT ANY WARRANTY; without even the implied    */
/*    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/*    PURPOSE. See the GNU Lesser General Public License for more   */
/*    details.                                                      */
/*    You should have received a copy of the GNU Lesser General     */
/*    Public License along with this library; if not, write to the  */
/*    Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,  */
/*    Boston, MA 02110-1301 USA                                     */
/********************************************************************/

/********************************************************************/
/*                            INCLUDES                              */
/********************************************************************/

/* Arduino system. (millis). */
#include <Arduino.h>

/* Private header. */
#include "TimedAction.h"

/********************************************************************/
/*                       FUNCTION DEFINITIONS                       */
/********************************************************************/

TimedAction::TimedAction(unsigned long intervl, void (*function)())
{
    active = false;
    previous = 0;
    interval = intervl;
    execute = function;
}

void TimedAction::start()
{
    active = true;
    previous = millis();
}

void TimedAction::stop()
{
    active = false;
}

void TimedAction::check()
{
    unsigned long currTime;
    if(active)
    {
        currTime = millis();
        if((currTime - previous) >= interval)
        {
            /* Maintain frequency stability by just adding one interval rather */
            /* than using the current time. */
            /* If we use the current time, it's possible for delays in the calling */
            /* of the check function to introduce "slip". */
            previous += interval;
            execute();
        }
    }
}
