/********************************************************************/
/*    MCP2515 + UDS Protocol Interface Library                      */
/*    MCPCANbusUDS.cpp                                              */
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

/********************************************************************/
/*                            INCLUDES                              */
/********************************************************************/

/* Interface header. */
#include "MCPCANbusUDS.h"

/********************************************************************/
/*                       FUNCTION DEFINITIONS                       */
/********************************************************************/

uint8_t MCP_CAN::mcp2515_readRegister(const uint8_t address)
{
    uint8_t ret;
    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    ret = spi_read();
    MCP2515_UNSELECT();
    return ret;
}

void MCP_CAN::mcp2515_readRegisterS(const uint8_t address, uint8_t values[], const uint8_t n)
{
    uint8_t i;
    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    /* Address will auto increment. */
    for (i = 0; i < n; i++)
    {
        values[i] = spi_read();
    }
    MCP2515_UNSELECT();
}

void MCP_CAN::mcp2515_setRegister(const uint8_t address, const uint8_t value)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);
    MCP2515_UNSELECT();
}

void MCP_CAN::mcp2515_setRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n)
{
    uint8_t i;
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    for (i = 0; i < n; i++) 
    {
        spi_readwrite(values[i]);
    }
    MCP2515_UNSELECT();
}

void MCP_CAN::mcp2515_modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_BITMOD);
    spi_readwrite(address);
    spi_readwrite(mask);
    spi_readwrite(data);
    MCP2515_UNSELECT();
}

uint8_t MCP_CAN::mcp2515_readStatus(void)                             
{
    uint8_t i;
    MCP2515_SELECT();
    spi_readwrite(MCP_READ_STATUS);
    i = spi_read();
    MCP2515_UNSELECT();
    return i;
}

MCP2515_Ret_Val_E MCP_CAN::mcp2515_setCANCTRL_Mode(const uint8_t newmode)
{
    uint16_t i;
    mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);
    /* We must check the status bits to make sure the configuration took. */
    /* It may take some time (the MCP2515 waits until all pending TX messages are sent */
    /* before applying the mode change). */
    for(i = 0; i < TIMEOUTVALUE; i++)
    {
        if((mcp2515_readRegister(MCP_CANSTAT) & MODE_MASK) == newmode)
        {
            /* Success. */
            return MCP2515_OK;
        }
    }
    return MCP2515_FAIL;
}

MCP_CAN_Ret_Val_E MCP_CAN::mcp2515_init(const MCP_CAN_Speed_E canSpeed)
{
    uint8_t i;

    /* Set registers to a known initial state. */
    MCP2515_SELECT();
    spi_readwrite(MCP_RESET);
    MCP2515_UNSELECT();
    /* Wait some time for things to settle. */
    delay(10);

    /* Enter configuration mode. */
    if(mcp2515_setCANCTRL_Mode(MODE_CONFIG) != MCP2515_OK)
    {
        /* Failed to set config mode. */
        return CAN_FAILINIT;
    }

    /* Set CAN speed. */
    if(canSpeed == MCP_CAN_SPEED_500KBPS)
    {
        mcp2515_setRegister(MCP_CNF1, MCP_16MHz_500kBPS_CFG1);
        mcp2515_setRegister(MCP_CNF2, MCP_16MHz_500kBPS_CFG2);
        mcp2515_setRegister(MCP_CNF3, MCP_16MHz_500kBPS_CFG3);
    }
    else if(canSpeed == MCP_CAN_SPEED_250KBPS)
    {
        mcp2515_setRegister(MCP_CNF1, MCP_16MHz_250kBPS_CFG1);
        mcp2515_setRegister(MCP_CNF2, MCP_16MHz_250kBPS_CFG2);
        mcp2515_setRegister(MCP_CNF3, MCP_16MHz_250kBPS_CFG3);
    }
    else
    {
        return CAN_FAILINIT;
    }

    /* Initialize buffers. */
    /* Clear and deactivate the three transmit buffers. */
    for (i = 0; i < 14; i++)
    {
        mcp2515_setRegister(MCP_TXB0CTRL + i, 0);
        mcp2515_setRegister(MCP_TXB1CTRL + i, 0);
        mcp2515_setRegister(MCP_TXB2CTRL + i, 0);
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0);
    mcp2515_setRegister(MCP_RXB1CTRL, 0);

    /* Initialize interrupts (both RX buffers). */
    mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

    /* Enable both receive buffers to receive messages with std/ext identifiers */
    /* and enable rollover. */
    /* This means that if there's a match for buffer 1 but it's currently occupied, */
    /* it will overflow in to buffer 2. However, if there's a match for buffer 2 and it's occupied, */
    /* it will drop the message EVEN if buffer 1 is empty. */
    mcp2515_modifyRegister(MCP_RXB0CTRL, 
                           MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK, 
                           MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
    mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_STDEXT);

    /* Set to normal mode. */                                                             
    if(mcp2515_setCANCTRL_Mode(MODE_NORMAL) != MCP2515_OK)
    {
        return CAN_FAILINIT;
    }
    return CAN_OK;
}

void MCP_CAN::mcp2515_write_id(const uint8_t mcp_addr, const uint8_t ext, const uint32_t id)
{
    uint16_t canid;
    uint8_t tbufdata[4];
    canid = (uint16_t)(id & 0x0FFFF);

    if(ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t)(canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t)(canid >> 3 );
        tbufdata[MCP_SIDL] = (uint8_t)((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcp2515_setRegisterS(mcp_addr, tbufdata, 4);
}

void MCP_CAN::mcp2515_read_id(const uint8_t mcp_addr, uint8_t * ext, uint32_t * id)
{
    uint8_t tbufdata[4];
    *ext = 0;
    *id = 0;
    mcp2515_readRegisterS(mcp_addr, tbufdata, 4);
    *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);
    if((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M) 
    {
        *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id << 8) + tbufdata[MCP_EID8];
        *id = (*id << 8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

void MCP_CAN::mcp2515_write_canMsg(const uint8_t buffer_sidh_addr)
{
    uint8_t mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(mcp_addr + 5, m_nDta, m_nDlc);
    if(m_nRtr == 1)
    {
        m_nDlc |= MCP_RTR_MASK;  
    }
    mcp2515_setRegister((mcp_addr + 4), m_nDlc);
    mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID);
}

void MCP_CAN::mcp2515_read_canMsg(const uint8_t buffer_sidh_addr)
{
    uint8_t mcp_addr, ctrl;
    mcp_addr = buffer_sidh_addr;
    mcp2515_read_id(mcp_addr, &m_nExtFlg, &m_nID);
    ctrl = mcp2515_readRegister(mcp_addr - 1);
    m_nDlc = mcp2515_readRegister(mcp_addr + 4);
    if((ctrl & 0x08))
    {
        m_nRtr = 1;
    }
    else
    {
        m_nRtr = 0;
    }
    m_nDlc &= MCP_DLC_MASK;
    mcp2515_readRegisterS(mcp_addr + 5, &(m_nDta[0]), m_nDlc);
}

MCP2515_Ret_Val_E MCP_CAN::mcp2515_getNextFreeTXBuf(uint8_t * txbuf_n)
{
    uint8_t i;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};
    *txbuf_n = 0x00;

    for (i = 0; i < MCP_N_TXBUFFERS; i++)
    {
        if((mcp2515_readRegister(ctrlregs[i]) & MCP_TXB_TXREQ_M) == 0)
        {
            *txbuf_n = ctrlregs[i] + 1;
            return MCP2515_OK;
        }
    }
    return MCP2515_ALLTXBUSY;
}

MCP_CAN::MCP_CAN(uint8_t _CS)
{
    SPICS = _CS;
    pinMode(SPICS, OUTPUT);
    MCP2515_UNSELECT();
}

MCP_CAN_Ret_Val_E MCP_CAN::begin(MCP_CAN_Speed_E speedset)
{
    SPI.begin();
    return mcp2515_init(speedset);
}

MCP_CAN_Ret_Val_E MCP_CAN::init_Mask(uint8_t num, uint8_t ext, uint32_t ulData)
{
    /* Check for validity of input. */
    if((num == 0) || (num == 1))
    {
        /* Good. */
    }
    else
    {
        return CAN_FAILINIT;
    }

    if(mcp2515_setCANCTRL_Mode(MODE_CONFIG) != MCP2515_OK)
    {
        return CAN_FAILINIT;
    }
    if(num == 0)
    {
        mcp2515_write_id(MCP_RXM0SIDH, ext, ulData);
    }
    else
    {
        mcp2515_write_id(MCP_RXM1SIDH, ext, ulData);
    }
    if(mcp2515_setCANCTRL_Mode(MODE_NORMAL) != MCP2515_OK)
    {
        return CAN_FAILINIT;
    }
    return CAN_OK;
}

MCP_CAN_Ret_Val_E MCP_CAN::init_Filt(uint8_t num, uint8_t ext, uint32_t ulData)
{
    /* Check for validity of input. */
    if((num == 0) || 
       (num == 1) || 
       (num == 2) || 
       (num == 3) || 
       (num == 4) || 
       (num == 5))
    {
        /* Good. */
    }
    else
    {
        return CAN_FAILINIT;
    }

    if(mcp2515_setCANCTRL_Mode(MODE_CONFIG) != MCP2515_OK)
    {
        return CAN_FAILINIT;
    }
    switch(num)
    {
    case 0:
        mcp2515_write_id(MCP_RXF0SIDH, ext, ulData);
        break;
    case 1:
        mcp2515_write_id(MCP_RXF1SIDH, ext, ulData);
        break;
    case 2:
        mcp2515_write_id(MCP_RXF2SIDH, ext, ulData);
        break;
    case 3:
        mcp2515_write_id(MCP_RXF3SIDH, ext, ulData);
        break;
    case 4:
        mcp2515_write_id(MCP_RXF4SIDH, ext, ulData);
        break;
    case 5:
        mcp2515_write_id(MCP_RXF5SIDH, ext, ulData);
        break;
    default:
        break;
    }
    if(mcp2515_setCANCTRL_Mode(MODE_NORMAL) != MCP2515_OK)
    {
        return CAN_FAILINIT;
    }
    return CAN_OK;
}

void MCP_CAN::setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t * pData)
{
    uint8_t i;
    m_nExtFlg = ext;
    m_nID = id;
    m_nDlc = len;
    for(i = 0; i < MCP_CAN_MAX_MSG_LEN; i++)
    {
        m_nDta[i] = *(pData + i);
    }
}

MCP_CAN_Ret_Val_E MCP_CAN::sendMsg()
{
    uint8_t txbuf_n;
    uint16_t i = 0;

    /* Wait for a TX buffer. */
    for(i = 0; i < TIMEOUTVALUE; i++)
    {
        if(mcp2515_getNextFreeTXBuf(&txbuf_n) == MCP2515_OK)
        {
            /* We have an open TX buffer. Address is loaded to txbuf_n. */
            /* Write msg to HW. */
            mcp2515_write_canMsg(txbuf_n);
            /* Trigger send. One byte before the buffer is the buffer's control register. */
            mcp2515_modifyRegister(txbuf_n - 1, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);
            return CAN_OK;
        }
    }
    return CAN_GETTXBFTIMEOUT;
}

MCP_CAN_Ret_Val_E MCP_CAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    setMsg(id, ext, len, buf);
    return sendMsg();
}

MCP_CAN_Ret_Val_E MCP_CAN::readMsg()
{
    uint8_t stat;
    MCP_CAN_Ret_Val_E res;

    stat = mcp2515_readStatus();

    if(stat & MCP_STAT_RX0IF)
    {
        mcp2515_read_canMsg(MCP_RXBUF_0);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if(stat & MCP_STAT_RX1IF)
    {
        mcp2515_read_canMsg(MCP_RXBUF_1);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
    {
        res = CAN_NOMSG;
    }
    return res;
}

MCP_CAN_Ret_Val_E MCP_CAN::readMsgBuf(uint8_t * len, uint8_t buf[])
{
    MCP_CAN_Ret_Val_E rc;

    rc = readMsg();

    if (rc == CAN_OK)
    {
        *len = m_nDlc;
        for(int i = 0; i < m_nDlc; i++)
        {
            buf[i] = m_nDta[i];
        }
    }
    else
    {
        *len = 0;
    }
    return rc;
}

MCP_CAN_Ret_Val_E MCP_CAN::checkReceive(void)
{
    uint8_t res;
    res = mcp2515_readStatus();
    if(res & MCP_STAT_RXIF_MASK) 
    {
        return CAN_MSGAVAIL;
    }
    else 
    {
        return CAN_NOMSG;
    }
}

uint32_t MCP_CAN::getCanId(void)
{
    return m_nID;
}
