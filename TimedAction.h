/********************************************************************/
/*    Timed Action Implementation Header                            */
/*    TimedAction.h                                                 */
/*                                                                  */
/*    Author:        Alexander Brevig                               */
/*    Contributors:  Jacob Moroni (jakemoroni@gmail.com)            */
/*                                                                  */
/*    Description:   This file implements the interface for the     */
/*                   TimedAction library (C++).                     */
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

#ifndef _TIMEDACTION_H_
#define _TIMEDACTION_H_

/********************************************************************/
/*                            INCLUDES                              */
/********************************************************************/

/* Standard types. */
#include <stdint.h>

/********************************************************************/
/*                            CLASSES                               */
/********************************************************************/

class TimedAction
{
    public:
        /* This constructor initializes a TimedAction. */
        /* This does NOT start the timer. */
        /* IN: intervl - period of which execution of callback should occur (ms) */
        /* IN: function - pointer to a void function which will be executed */
        TimedAction(unsigned long intervl, void (*function)());

        /* This function starts/resets the timer. */
        void start();

        /* This function stops the timer. */
        void stop();

        /* This function checks the timer and executes the callback if the */
        /* period has elapsed. */
        void check();

    private:
        uint8_t active;
        unsigned long previous;
        unsigned long interval;
        void (*execute)();
};


#endif /* _TIMEDACTION_H_ */
