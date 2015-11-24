/********************************************************************/
/*    MCP2515 + UDS Protocol Interface Library Header               */
/*    MCPCANbusUDS.h                                                */
/*                                                                  */
/*    Author:         Loovee                                        */
/*    Contributors:   Cory J. Fowler                                */
/*                    Jacob Moroni (jakemoroni@gmail.com)           */
/*                                                                  */
/*    Description:    This file implements the interface methods as */
/*                    well as the UDS protocol definitions required */
/*                    to communicate with a late model (2008+)      */
/*                    BMW using the MCP2515 CANbus controller.      */
/*                                                                  */
/*    Revision:       Oct 12, 2015                                  */
/*                                                                  */
/*    Copyright (C) 2012 Seeed Technology Inc. All right reserved.  */
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

#ifndef _MCPCANBUSUDS_H_
#define _MCPCANBUSUDS_H_

/********************************************************************/
/*                            INCLUDES                              */
/********************************************************************/

/* Chip register definitions. */
#include "MCP2515RegDefs.h"

/* Standard types. */
#include <stdint.h>

/* Arduino system. */
#include <Arduino.h>

/* SPI Interface. */
#include <SPI.h>

/********************************************************************/
/*                             DEFINES                              */
/********************************************************************/

/* ---------- BMW UDS (Extended OBD) Definitions. ---------- */

/* The ID of which we send requests. (Mailbox) */
#define UDS_CAN_TRANSMIT_ID                    0x6F1

/* The ID of which responses are sent. (Mailbox) */
#define UDS_CAN_RECEIVE_ID                     0x612

/* We use an 11 bit ID... */
#define UDS_CAN_ID_MASK                        0x7FF

/* Fuel load request message.                                          Len   SID   DID1  DID2  PAD . . */
#define UDS_CAN_FUEL_LOAD_REQUEST_MSG_PTR      (const uint8_t[]){0x12, 0x03, 0x22, 0x45, 0x21, 0, 0, 0}

/* Fuel load response message. */
#define UDS_CAN_FUEL_LOAD_RESPONSE_MSG_PTR     (const uint8_t[]){0xF1, 0x05, 0x62, 0x45, 0x21, 0, 0, 0}

/* BMW multipler = 0.10000000149011612. */
/* Specific gravity of gasoline = 0.73722 kg per liter. */
/* So, 0.10000000149011612 * (1 / 0.73722). */
#define UDS_CAN_FUEL_LOAD_KG_TO_L_MULT_SCALAR  ((double)0.135645)

/* Comparison depth for qualifying received UDS messages. */
#define UDS_CAN_RX_RESP_COMPARE_LENGTH         5

/* Payload offsets. */
#define UDS_CAN_RX_RESP_DATA_BYTE_0_IDX        5
#define UDS_CAN_RX_RESP_DATA_BYTE_1_IDX        6
#define UDS_CAN_RX_RESP_DATA_BYTE_2_IDX        7

/* ---------- Generic MCP CAN Definitions. ---------- */

/* Maximum message length under all circumstances. */
#define MCP_CAN_MAX_MSG_LEN                    8

/* Arduino SPI abstraction. */
#define spi_readwrite SPI.transfer
#define spi_read() spi_readwrite(0x00)

/* Arduino chip select abstraction. Active LOW. */
#define MCP2515_SELECT()   digitalWrite(SPICS, LOW)
#define MCP2515_UNSELECT() digitalWrite(SPICS, HIGH)

/* Timeout value (actually, retry value). This is used for */
/* instances where we are waiting for transmissions to complete. */
#define TIMEOUTVALUE                           50

/********************************************************************/
/*                            DATATYPES                             */
/********************************************************************/

/* Supported CANbus speeds. */
typedef enum
{
    MCP_CAN_SPEED_250KBPS,
    MCP_CAN_SPEED_500KBPS
} MCP_CAN_Speed_E;

/* MCP CAN API return values. */
typedef enum
{
    CAN_OK,
    CAN_FAILINIT,
    CAN_MSGAVAIL,
    CAN_NOMSG,
    CAN_GETTXBFTIMEOUT
} MCP_CAN_Ret_Val_E;

/* MCP2515 Interface function return value. */
typedef enum
{
    MCP2515_OK,
    MCP2515_FAIL,
    MCP2515_ALLTXBUSY
} MCP2515_Ret_Val_E;

/********************************************************************/
/*                            CLASSES                               */
/********************************************************************/

class MCP_CAN
{
    private:
    
    uint8_t   m_nExtFlg;                        /* Identifier xxxID             */
                                                /* Either extended (the 29 LSB) */
                                                /* or standard (the 11 LSB)     */
    uint32_t  m_nID;                            /* CAN Message ID               */
    uint8_t   m_nDlc;                           /* Message Length               */
    uint8_t   m_nDta[MCP_CAN_MAX_MSG_LEN];      /* Message Data Buffer          */
    uint8_t   m_nRtr;                           /* RTR                          */
    uint8_t   SPICS;                            /* The SPI chip select pin.     */

private:

    uint8_t mcp2515_readRegister(const uint8_t address);
    void mcp2515_readRegisterS(const uint8_t address, uint8_t values[], const uint8_t n);
    void mcp2515_setRegister(const uint8_t address, const uint8_t value);
    void mcp2515_setRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n);
    void mcp2515_modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data);
    uint8_t mcp2515_readStatus(void);
    MCP2515_Ret_Val_E mcp2515_setCANCTRL_Mode(const uint8_t newmode);
    MCP_CAN_Ret_Val_E mcp2515_init(const MCP_CAN_Speed_E canSpeed);
    void mcp2515_write_id( const uint8_t mcp_addr, const uint8_t ext, const uint32_t id );
    void mcp2515_read_id( const uint8_t mcp_addr, uint8_t * ext, uint32_t * id );
    void mcp2515_write_canMsg( const uint8_t buffer_sidh_addr );
    void mcp2515_read_canMsg( const uint8_t buffer_sidh_addr);
    MCP2515_Ret_Val_E mcp2515_getNextFreeTXBuf(uint8_t * txbuf_n);
    void setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t * pData);
    MCP_CAN_Ret_Val_E readMsg();
    MCP_CAN_Ret_Val_E sendMsg();

public:

    MCP_CAN(uint8_t _CS);
    MCP_CAN_Ret_Val_E begin(MCP_CAN_Speed_E speedset);
    MCP_CAN_Ret_Val_E init_Mask(uint8_t num, uint8_t ext, uint32_t ulData);
    MCP_CAN_Ret_Val_E init_Filt(uint8_t num, uint8_t ext, uint32_t ulData);
    MCP_CAN_Ret_Val_E sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);
    MCP_CAN_Ret_Val_E readMsgBuf(uint8_t *len, uint8_t * buf);
    MCP_CAN_Ret_Val_E checkReceive(void);
    uint32_t getCanId(void);
};


#endif /* _MCPCANBUSUDS_H_ */
