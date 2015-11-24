/********************************************************************/
/*    Controller Core Implementation Header                         */
/*    ControllerMain.h                                              */
/*                                                                  */
/*    Author:         Jacob Moroni (jakemoroni@gmail.com)           */
/*                                                                  */
/*    Description:    This file implements the core functionality   */
/*                    of the CANbus based meth controller.          */
/*                                                                  */
/*    Revision:       Oct 12, 2015 (First)                          */
/*                                                                  */
/*    Copyright (C) 2015 Jacob Moroni.  All right reserved.         */
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

#ifndef _CONTROLLERMAIN_H_
#define _CONTROLLERMAIN_H_

/********************************************************************/
/*                             DEFINES                              */
/********************************************************************/

/* Enable debug output. */
#define CONTROLLER_MAIN_SERIAL_DEBUG

/* Can controller SPI chip select pin. */
#define CONTROLLER_MAIN_CAN_CHIP_SEL_PIN           10

/* Good status output pin. Can be hooked to an LED. */
/* A better term may have been "armed and ready"... */
/* This basically ends up being the same as the meth controller's */
/* failsafe output pin, but inverted. The reason being that */
/* it's nice to know that the system is powered on and running. */
/* If the failsafe isn't tripped, we don't know if everything is okay */
/* or if the unit is just dead. */
#define CONTROLLER_MAIN_OKAY_OUTPUT_PIN            6


#endif /* _CONTROLLERMAIN_H_ */
