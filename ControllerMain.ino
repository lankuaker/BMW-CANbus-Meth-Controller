/********************************************************************/
/*    Controller Core Implementation Header                         */
/*    ControllerMain.ino (cpp)                                      */
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

/********************************************************************/
/*                            INCLUDES                              */
/********************************************************************/

/* Internal header. */
#include "ControllerMain.h"

/* Include the MCP2515 API. */
#include "MCPCANbusUDS.h"

/* Service headers... */
#include "MethService.h"

/********************************************************************/
/*                     GLOBAL STATIC VARIABLES                      */
/********************************************************************/

/* Initialize a CAN library instance. */
/* Provide chip select on pin 10. */
MCP_CAN CAN(CONTROLLER_MAIN_CAN_CHIP_SEL_PIN);

/* Interrupt flag. Single byte; atomic. */
static volatile unsigned char _intrFlagRecv = 0;

/********************************************************************/
/*                   STATIC FUNCTION PROTOTYPES                     */
/********************************************************************/

/* These prototypes are necessary to avoid compiler warnings for some reason, */
/* even though they're defined before they're called... */
static void _mcp2515ISR();
static void _canRXMsgDispatch(uint32_t id, uint8_t * buf);

/********************************************************************/
/*                   STATIC FUNCTION DEFINITIONS                    */
/********************************************************************/

/* Interrupt service routine for the CAN controller's RX interrupt. */
static void _mcp2515ISR()
{
    _intrFlagRecv = 1;
}

/* This function dispatches received CAN messages to the proper service. */
static void _canRXMsgDispatch(uint32_t id, uint8_t * buf)
{
    if(id == UDS_CAN_RECEIVE_ID)
    {
        /* Fuel load response. Send to meth service. */
        if(!memcmp(buf, UDS_CAN_FUEL_LOAD_RESPONSE_MSG_PTR, UDS_CAN_RX_RESP_COMPARE_LENGTH))
        {
            MethServiceRXFuelLoadMsg(buf);
        }
    }
}

/********************************************************************/
/*                   GLOBAL FUNCTION DEFINITIONS                    */
/********************************************************************/

/* Initialization routine for Arduino. */
void setup()
{
    MCP_CAN_Ret_Val_E canInit = CAN_OK;

    /* Initialize UART. Other services might rely on it. */
    Serial.begin(115200);

    /* Initialize system okay output pin to OFF. */
    pinMode(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, OUTPUT);
    digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, LOW);

    /* Call the early initialization routine for all controllers. */
    MethServiceInitEarly();

    /* Register ISR for CAN message RX event. */
    attachInterrupt(0, _mcp2515ISR, FALLING);

    /* 2011 BMW E92 uses a 500Kbps CANbus on the OBD port. */
    if(CAN.begin(MCP_CAN_SPEED_500KBPS) != CAN_OK) canInit = CAN_FAILINIT;

    /* Configure filters and masks. */
    /* Set both masks for the full 11 bit ID. */
    if(CAN.init_Mask(0, 0, UDS_CAN_ID_MASK) != CAN_OK) canInit = CAN_FAILINIT;
    if(CAN.init_Mask(1, 0, UDS_CAN_ID_MASK) != CAN_OK) canInit = CAN_FAILINIT;

    /* The first two filters apply to the higher priority RX buffer (buffer 0). */
    /* If that buffer is occupied, it will overflow into the second buffer */
    /* regardless of the second buffer's filters. */
    /* However, the second buffer's filters can allow messages to enter directly in to the second buffer. */
    /* In other words, set both masks full on, and configure only the first two filters. */
    /* This essentially means that all frames will try to enter buf 0 first, then ovf in to buf 1. */
    /* Register to receive transmitted messages so we can detect another scanner on the bus......... */
    if(CAN.init_Filt(0, 0, UDS_CAN_TRANSMIT_ID) != CAN_OK) canInit = CAN_FAILINIT;
    if(CAN.init_Filt(1, 0, UDS_CAN_RECEIVE_ID) != CAN_OK) canInit = CAN_FAILINIT;
    if(CAN.init_Filt(2, 0, 0) != CAN_OK) canInit = CAN_FAILINIT;
    if(CAN.init_Filt(3, 0, 0) != CAN_OK) canInit = CAN_FAILINIT;
    if(CAN.init_Filt(4, 0, 0) != CAN_OK) canInit = CAN_FAILINIT;
    if(CAN.init_Filt(5, 0, 0) != CAN_OK) canInit = CAN_FAILINIT;

    if(canInit == CAN_OK)
    {
        /* Call the late initialization routine for all controllers. */
        MethServiceInitLate();

#if defined(CONTROLLER_MAIN_SERIAL_DEBUG)
        Serial.println("System initialized! Entering main loop...");
#endif
    }
    else
    {
#if defined(CONTROLLER_MAIN_SERIAL_DEBUG)
        Serial.println("MCP2515 initialization failed! Aborting controllers...");
#endif
        /* Send the failure signal to all controllers. */
        MethServiceAbort();
    }
}

/* Main execution loop. */
void loop()
{
    /* Buffer used to read messages. */
    static uint8_t rxLen = 0;
    static uint8_t rxBuf[MCP_CAN_MAX_MSG_LEN] = {0};

    if(_intrFlagRecv)
    {
        _intrFlagRecv = 0;

        /* Loop until there are no messages. This ensures that the interrupt will be reset. */
        while(CAN_MSGAVAIL == CAN.checkReceive())
        {
            CAN.readMsgBuf(&rxLen, rxBuf);
            _canRXMsgDispatch(CAN.getCanId(), rxBuf);
        }
    }

    /* Execute each service. */
    MethServiceExec();

    /* Derive the global system status from the controllers' status. */
    /* For now, there's only one controller, so it's easy. */
    /* Decide whether to turn the "okay" LED on or not. */
    switch(MethServiceGetState())
    {
    case METH_SERVICE_STATE_FAILURE:
        digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, LOW);
#if defined(CONTROLLER_MAIN_SERIAL_DEBUG)
        Serial.println("STATE: FAILURE");
#endif
        break;

    case METH_SERVICE_STATE_READY_ARMED:
        digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, HIGH);
        break;

    case METH_SERVICE_STATE_INJECTING:
        digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, HIGH);
        break;

    case METH_SERVICE_STATE_DISABLED:
        digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, LOW);
#if defined(CONTROLLER_MAIN_SERIAL_DEBUG)
        Serial.println("STATE: DISABLED");
#endif
        break;

    case METH_SERVICE_STATE_DISABLED_WARN:
        digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, LOW);
#if defined(CONTROLLER_MAIN_SERIAL_DEBUG)
        Serial.println("STATE: DISABLED-WARN");
#endif
        break;

    case METH_SERVICE_STATE_LOW_TANK:
        digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, LOW);
#if defined(CONTROLLER_MAIN_SERIAL_DEBUG)
        Serial.println("STATE: LOW-TANK");
#endif
        break;

    default:
        digitalWrite(CONTROLLER_MAIN_OKAY_OUTPUT_PIN, LOW);
#if defined(CONTROLLER_MAIN_SERIAL_DEBUG)
        Serial.println("STATE: ERROR - UNKNOWN STATE");
#endif
        break;
    }
}
