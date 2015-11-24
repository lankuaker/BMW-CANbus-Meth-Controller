/********************************************************************/
/*    MCP2515 Register and Interface Definitions                    */
/*    MCP2515RegDefs.h                                              */
/*                                                                  */
/*    Author:         Loovee                                        */
/*    Contributors:   Cory J. Fowler                                */
/*                    Jacob Moroni (jakemoroni@gmail.com)           */
/*                                                                  */
/*    Description:    This file implements the register definitions */
/*                    for the MCP2515 CANbus controller.            */
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

#ifndef _MCP2515REGDEFS_H_
#define _MCP2515REGDEFS_H_

/********************************************************************/
/*                             DEFINES                              */
/********************************************************************/

#define MCP_N_TXBUFFERS     (3)
#define MCP_SIDH            0
#define MCP_SIDL            1
#define MCP_EID8            2
#define MCP_EID0            3
#define MCP_TXB_EXIDE_M     0x08
#define MCP_DLC_MASK        0x0F
#define MCP_RTR_MASK        0x40
#define MCP_RXB_RX_STDEXT   0x00
#define MCP_RXB_RX_MASK     0x60
#define MCP_RXB_BUKT_MASK   (1 << 2)
#define MCP_TXB_TXREQ_M     0x08
#define MCP_STAT_RXIF_MASK  (0x03)
#define MCP_STAT_RX0IF      (1 << 0)
#define MCP_STAT_RX1IF      (1 << 1)
#define MCP_RXF0SIDH        0x00
#define MCP_RXF1SIDH        0x04
#define MCP_RXF2SIDH        0x08
#define MCP_RXF3SIDH        0x10
#define MCP_RXF4SIDH        0x14
#define MCP_RXF5SIDH        0x18
#define MCP_RXM0SIDH        0x20
#define MCP_RXM1SIDH        0x24
#define MCP_CANSTAT         0x0E
#define MCP_CANCTRL         0x0F
#define MCP_CNF3            0x28
#define MCP_CNF2            0x29
#define MCP_CNF1            0x2A
#define MCP_CANINTE         0x2B
#define MCP_CANINTF         0x2C
#define MCP_TXB0CTRL        0x30
#define MCP_TXB1CTRL        0x40
#define MCP_TXB2CTRL        0x50
#define MCP_RXB0CTRL        0x60
#define MCP_RXB0SIDH        0x61
#define MCP_RXB1CTRL        0x70
#define MCP_RXB1SIDH        0x71
#define MCP_WRITE           0x02
#define MCP_READ            0x03
#define MCP_BITMOD          0x05
#define MCP_READ_STATUS     0xA0
#define MCP_RESET           0xC0
#define MODE_NORMAL         0x00
#define MODE_CONFIG         0x80
#define MODE_MASK           0xE0
#define MCP_RX0IF           0x01
#define MCP_RX1IF           0x02
#define MCP_16MHz_500kBPS_CFG1   (0x00)
#define MCP_16MHz_500kBPS_CFG2   (0xF0)
#define MCP_16MHz_500kBPS_CFG3   (0x86)
#define MCP_16MHz_250kBPS_CFG1   (0x41)
#define MCP_16MHz_250kBPS_CFG2   (0xF1)
#define MCP_16MHz_250kBPS_CFG3   (0x85)
#define MCP_RXBUF_0         (MCP_RXB0SIDH)
#define MCP_RXBUF_1         (MCP_RXB1SIDH)


#endif /* _MCP2515REGDEFS_H_ */
