/*
  mcp_can.cpp
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  Author:Loovee
  Contributor: Cory J. Fowler
  2014-1-16
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#include "mcp_can.h"
#include <string.h>

#define spi_readwrite pSPI->transfer
#define spi_read() spi_readwrite(0x00)

/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            reset the device
*********************************************************************************************************/
void MCP_CAN::mcp2515_reset(void)
{
  MCP2515_SELECT();
  spi_readwrite(MCP_RESET);
  MCP2515_UNSELECT();
  delay(10);
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            read register
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_readRegister(const INT8U address)
{

  if (smartSPI) {
	  INT8U buf[3];
	  buf[0] = MCP_READ;
	  buf[1] = address;
	  buf[2] = 0xFF; // For return value

	  MCP2515_SELECT();
	  pSPI->transfer(buf, 3);
	  MCP2515_UNSELECT();

	  return buf[2];

  } else {
	  INT8U ret;

	  MCP2515_SELECT();
	  spi_readwrite(MCP_READ);
	  spi_readwrite(address);
	  ret = spi_read();
	  MCP2515_UNSELECT();

	  return ret;
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            read registerS
*********************************************************************************************************/
void MCP_CAN::mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n)
{
  if (smartSPI && n <= 8) {
	  unsigned char buf[10];

	  buf[0] = MCP_READ;
	  buf[1] = address;
	  // reaming bufs are for return values

	  MCP2515_SELECT();
	  pSPI->transfer(buf, static_cast<uint8_t>(2+n));
	  MCP2515_UNSELECT();

	  memcpy(values, &buf[2],n);
  } else {
	  MCP2515_SELECT();
	  spi_readwrite(MCP_READ);
	  spi_readwrite(address);
	  // mcp2515 has auto-increment of address-pointer
	  for (int i = 0; i < n && i < CAN_MAX_CHAR_IN_MESSAGE; i++) {
		values[i] = spi_read();
	  }
	  MCP2515_UNSELECT();
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            set register
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegister(const INT8U address, const INT8U value)
{
  if (smartSPI) {
	  unsigned char buf[3];
	  buf[0] = MCP_WRITE;
	  buf[1] = address;
	  buf[2] = value;

	  MCP2515_SELECT();
	  pSPI->transfer(buf, 3);
	  MCP2515_UNSELECT();
  } else {
	  MCP2515_SELECT();
	  spi_readwrite(MCP_WRITE);
	  spi_readwrite(address);
	  spi_readwrite(value);
	  MCP2515_UNSELECT();
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            set registerS
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n)
{
	if (smartSPI && n <= 8) {
		unsigned char buf[10];

		buf[0] = MCP_WRITE;
		buf[1] = address;
		memcpy(&buf[2], values, n);

		MCP2515_SELECT();
		pSPI->transfer(buf, static_cast<uint8_t>(2+n));
		MCP2515_UNSELECT();
	} else {
		INT8U i;
		MCP2515_SELECT();
		spi_readwrite(MCP_WRITE);
		spi_readwrite(address);

		for (i = 0; i < n; i++)
		{
			spi_readwrite(values[i]);
		}
		MCP2515_UNSELECT();
	}
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            set bit of one register
*********************************************************************************************************/
void MCP_CAN::mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data)
{
	if (smartSPI) {
		unsigned char buf[4];
		buf[0] = MCP_BITMOD;
		buf[1] = address;
		buf[2] = mask;
		buf[3] = data;

		MCP2515_SELECT();
		pSPI->transfer(buf, 4);
		MCP2515_UNSELECT();
	} else {
		MCP2515_SELECT();
		spi_readwrite(MCP_BITMOD);
		spi_readwrite(address);
		spi_readwrite(mask);
		spi_readwrite(data);
		MCP2515_UNSELECT();
	}
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            read mcp2515's Status
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_readStatus(void)
{
	if (smartSPI) {
		unsigned char buf[2];

		buf[0] = MCP_READ_STATUS;
		// buf[1] is read status

		MCP2515_SELECT();
		pSPI->transfer(buf,2);
		MCP2515_UNSELECT();

		return buf[1];

	} else {
		INT8U status;
		MCP2515_SELECT();
		spi_readwrite(MCP_READ_STATUS);
		status = spi_read();
		MCP2515_UNSELECT();

		return status;
	}
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            set control mode
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_setCANCTRL_Mode(const INT8U newmode)
{
  INT8U i;

  mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

  i = mcp2515_readRegister(MCP_CANCTRL);
  i &= MODE_MASK;

  if ( i == newmode )
  {
    return MCP2515_OK;
  }

  return MCP2515_FAIL;

}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            set boadrate
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_configRate(const INT8U canSpeed, const INT8U clock)
{
  INT8U set, cfg1, cfg2, cfg3;
  set = 1;
  switch (clock)
  {
    case (MCP_16MHz) :
      switch (canSpeed)
      {
        case (CAN_5KBPS):
          cfg1 = MCP_16MHz_5kBPS_CFG1;
          cfg2 = MCP_16MHz_5kBPS_CFG2;
          cfg3 = MCP_16MHz_5kBPS_CFG3;
          break;

        case (CAN_10KBPS):
          cfg1 = MCP_16MHz_10kBPS_CFG1;
          cfg2 = MCP_16MHz_10kBPS_CFG2;
          cfg3 = MCP_16MHz_10kBPS_CFG3;
          break;

        case (CAN_20KBPS):
          cfg1 = MCP_16MHz_20kBPS_CFG1;
          cfg2 = MCP_16MHz_20kBPS_CFG2;
          cfg3 = MCP_16MHz_20kBPS_CFG3;
          break;

        case (CAN_31K25BPS):
          cfg1 = MCP_16MHz_31k25BPS_CFG1;
          cfg2 = MCP_16MHz_31k25BPS_CFG2;
          cfg3 = MCP_16MHz_31k25BPS_CFG3;
          break;

        case (CAN_33KBPS):
          cfg1 = MCP_16MHz_33kBPS_CFG1;
          cfg2 = MCP_16MHz_33kBPS_CFG2;
          cfg3 = MCP_16MHz_33kBPS_CFG3;
          break;

        case (CAN_40KBPS):
          cfg1 = MCP_16MHz_40kBPS_CFG1;
          cfg2 = MCP_16MHz_40kBPS_CFG2;
          cfg3 = MCP_16MHz_40kBPS_CFG3;
          break;

        case (CAN_50KBPS):
          cfg1 = MCP_16MHz_50kBPS_CFG1;
          cfg2 = MCP_16MHz_50kBPS_CFG2;
          cfg3 = MCP_16MHz_50kBPS_CFG3;
          break;

        case (CAN_80KBPS):
          cfg1 = MCP_16MHz_80kBPS_CFG1;
          cfg2 = MCP_16MHz_80kBPS_CFG2;
          cfg3 = MCP_16MHz_80kBPS_CFG3;
          break;

        case (CAN_83K3BPS):
          cfg1 = MCP_16MHz_83k3BPS_CFG1;
          cfg2 = MCP_16MHz_83k3BPS_CFG2;
          cfg3 = MCP_16MHz_83k3BPS_CFG3;
          break;

        case (CAN_95KBPS):
          cfg1 = MCP_16MHz_95kBPS_CFG1;
          cfg2 = MCP_16MHz_95kBPS_CFG2;
          cfg3 = MCP_16MHz_95kBPS_CFG3;
          break;

        case (CAN_100KBPS):                                             /* 100KBPS                  */
          cfg1 = MCP_16MHz_100kBPS_CFG1;
          cfg2 = MCP_16MHz_100kBPS_CFG2;
          cfg3 = MCP_16MHz_100kBPS_CFG3;
          break;

        case (CAN_125KBPS):
          cfg1 = MCP_16MHz_125kBPS_CFG1;
          cfg2 = MCP_16MHz_125kBPS_CFG2;
          cfg3 = MCP_16MHz_125kBPS_CFG3;
          break;

        case (CAN_200KBPS):
          cfg1 = MCP_16MHz_200kBPS_CFG1;
          cfg2 = MCP_16MHz_200kBPS_CFG2;
          cfg3 = MCP_16MHz_200kBPS_CFG3;
          break;

        case (CAN_250KBPS):
          cfg1 = MCP_16MHz_250kBPS_CFG1;
          cfg2 = MCP_16MHz_250kBPS_CFG2;
          cfg3 = MCP_16MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS):
          cfg1 = MCP_16MHz_500kBPS_CFG1;
          cfg2 = MCP_16MHz_500kBPS_CFG2;
          cfg3 = MCP_16MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS):
          cfg1 = MCP_16MHz_1000kBPS_CFG1;
          cfg2 = MCP_16MHz_1000kBPS_CFG2;
          cfg3 = MCP_16MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          break;
      }
      break;

    case (MCP_8MHz) :
      switch (canSpeed)
      {
        case (CAN_5KBPS) :
          cfg1 = MCP_8MHz_5kBPS_CFG1;
          cfg2 = MCP_8MHz_5kBPS_CFG2;
          cfg3 = MCP_8MHz_5kBPS_CFG3;
          break;

        case (CAN_10KBPS) :
          cfg1 = MCP_8MHz_10kBPS_CFG1;
          cfg2 = MCP_8MHz_10kBPS_CFG2;
          cfg3 = MCP_8MHz_10kBPS_CFG3;
          break;

        case (CAN_20KBPS) :
          cfg1 = MCP_8MHz_20kBPS_CFG1;
          cfg2 = MCP_8MHz_20kBPS_CFG2;
          cfg3 = MCP_8MHz_20kBPS_CFG3;
          break;

        case (CAN_31K25BPS) :
          cfg1 = MCP_8MHz_31k25BPS_CFG1;
          cfg2 = MCP_8MHz_31k25BPS_CFG2;
          cfg3 = MCP_8MHz_31k25BPS_CFG3;
          break;

        case (CAN_40KBPS) :
          cfg1 = MCP_8MHz_40kBPS_CFG1;
          cfg2 = MCP_8MHz_40kBPS_CFG2;
          cfg3 = MCP_8MHz_40kBPS_CFG3;
          break;

        case (CAN_50KBPS) :
          cfg1 = MCP_8MHz_50kBPS_CFG1;
          cfg2 = MCP_8MHz_50kBPS_CFG2;
          cfg3 = MCP_8MHz_50kBPS_CFG3;
          break;

        case (CAN_80KBPS) :
          cfg1 = MCP_8MHz_80kBPS_CFG1;
          cfg2 = MCP_8MHz_80kBPS_CFG2;
          cfg3 = MCP_8MHz_80kBPS_CFG3;
          break;

        case (CAN_100KBPS) :                                             /* 100KBPS                  */
          cfg1 = MCP_8MHz_100kBPS_CFG1;
          cfg2 = MCP_8MHz_100kBPS_CFG2;
          cfg3 = MCP_8MHz_100kBPS_CFG3;
          break;

        case (CAN_125KBPS) :
          cfg1 = MCP_8MHz_125kBPS_CFG1;
          cfg2 = MCP_8MHz_125kBPS_CFG2;
          cfg3 = MCP_8MHz_125kBPS_CFG3;
          break;

        case (CAN_200KBPS) :
          cfg1 = MCP_8MHz_200kBPS_CFG1;
          cfg2 = MCP_8MHz_200kBPS_CFG2;
          cfg3 = MCP_8MHz_200kBPS_CFG3;
          break;

        case (CAN_250KBPS) :
          cfg1 = MCP_8MHz_250kBPS_CFG1;
          cfg2 = MCP_8MHz_250kBPS_CFG2;
          cfg3 = MCP_8MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS) :
          cfg1 = MCP_8MHz_500kBPS_CFG1;
          cfg2 = MCP_8MHz_500kBPS_CFG2;
          cfg3 = MCP_8MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS) :
          cfg1 = MCP_8MHz_1000kBPS_CFG1;
          cfg2 = MCP_8MHz_1000kBPS_CFG2;
          cfg3 = MCP_8MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          break;
      }
      break;

    default:
      set = 0;
      break;
  }
  if (set) {
    mcp2515_setRegister(MCP_CNF1, cfg1);
    mcp2515_setRegister(MCP_CNF2, cfg2);
    mcp2515_setRegister(MCP_CNF3, cfg3);
    return MCP2515_OK;
  }
  else {
    return MCP2515_FAIL;
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            init canbuffers
*********************************************************************************************************/
void MCP_CAN::mcp2515_initCANBuffers(void)
{
  INT8U i, a1, a2, a3;

  // INT8U std = 0;
  // INT8U ext = 1;
  // INT32U ulMask = 0x00, ulFilt = 0x00;


  //mcp2515_write_id(MCP_RXM0SIDH, ext, ulMask);      /*Set both masks to 0           */
  //mcp2515_write_id(MCP_RXM1SIDH, ext, ulMask);      /*Mask register ignores ext bit */

  /* Set all filters to 0         */
  //mcp2515_write_id(MCP_RXF0SIDH, ext, ulFilt);      /* RXB0: extended               */
  //mcp2515_write_id(MCP_RXF1SIDH, std, ulFilt);      /* RXB1: standard               */
  //mcp2515_write_id(MCP_RXF2SIDH, ext, ulFilt);      /* RXB2: extended               */
  //mcp2515_write_id(MCP_RXF3SIDH, std, ulFilt);      /* RXB3: standard               */
  //mcp2515_write_id(MCP_RXF4SIDH, ext, ulFilt);
  //mcp2515_write_id(MCP_RXF5SIDH, std, ulFilt);

  /* Clear, deactivate the three  */
  /* transmit buffers             */
  /* TXBnCTRL -> TXBnD7           */
  a1 = MCP_TXB0CTRL;
  a2 = MCP_TXB1CTRL;
  a3 = MCP_TXB2CTRL;
  for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
    mcp2515_setRegister(a1, 0);
    mcp2515_setRegister(a2, 0);
    mcp2515_setRegister(a3, 0);
    a1++;
    a2++;
    a3++;
  }
  mcp2515_setRegister(MCP_RXB0CTRL, 0);
  mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            init the device
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_init(const INT8U canSpeed, const INT8U clock)                       /* mcp2515init                  */
{

  INT8U res;

  mcp2515_reset();

  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res != MCP2515_OK)
  {
#if DEBUG_MODE
    Serial.print("Enter setting mode fall\r\n");
#else
    delay(10);
#endif
    return res;
  }
#if DEBUG_MODE
  Serial.print("Enter setting mode success \r\n");
#else
  delay(10);
#endif

  /* set baud rate                 */
  if (MCP2515_OK != mcp2515_configRate(canSpeed, clock))
  {
#if DEBUG_MODE
    Serial.print("set rate fall!!\r\n");
#else
    delay(10);
#endif
    return res;
  }
#if DEBUG_MODE
  Serial.print("set rate success!!\r\n");
#else
  delay(10);
#endif

  if ( res == MCP2515_OK ) {

    /* init canbuffers              */
    mcp2515_initCANBuffers();

#if 0

    /* interrupt mode               */
    enableInterrupt(MCP_RX0IF | MCP_RX1IF);

#if (DEBUG_RXANY==1)
    /* enable both receive-buffers  */
    /* to receive any message       */
    /* and enable rollover          */
    mcp2515_modifyRegister(MCP_RXB0CTRL,
                           MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                           MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
    mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                           MCP_RXB_RX_ANY);
#else
    /* enable both receive-buffers  */
    /* to receive messages          */
    /* with std. and ext. identifie */
    /* rs                           */
    /* and enable rollover          */
    mcp2515_modifyRegister(MCP_RXB0CTRL,
                           MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                           MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
    mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                           MCP_RXB_RX_STDEXT);
#endif
#else
    /* interrupt mode               */
    enableInterrupt(MCP_RX0IF);

    /* Enable just RXB0 - we don't know the order (?)of reception if using more buffers */
    mcp2515_modifyRegister(MCP_RXB0CTRL,
                           MCP_RXB_RX_MASK,
                           MCP_RXB_RX_ANY);
#endif
    /* enter normal mode            */
    res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);
    if (res != MCP2515_OK)
    {
#if DEBUG_MODE
      Serial.print("Enter Normal Mode Fall!!\r\n");
#else
      delay(10);
#endif
      return res;
    }


#if DEBUG_MODE
    Serial.print("Enter Normal Mode Success!!\r\n");
#else
    delay(10);
#endif

  }
  return res;

}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            write can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_id( const INT8U mcp_addr, const INT8U ext, const INT32U id )
{
  uint16_t canid;
  INT8U tbufdata[4];

  canid = (uint16_t)(id & 0x0FFFF);

  if ( ext == 1)
  {
    tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
    tbufdata[MCP_EID8] = (INT8U) (canid >> 8);
    canid = (uint16_t)(id >> 16);
    tbufdata[MCP_SIDL] = (INT8U) (canid & 0x03);
    tbufdata[MCP_SIDL] = (INT8U) (tbufdata[MCP_SIDL]+((canid & 0x1C) << 3));
    tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
    tbufdata[MCP_SIDH] = (INT8U) (canid >> 5 );
  }
  else
  {
    tbufdata[MCP_SIDH] = (INT8U) (canid >> 3 );
    tbufdata[MCP_SIDL] = (INT8U) ((canid & 0x07 ) << 5);
    tbufdata[MCP_EID0] = 0;
    tbufdata[MCP_EID8] = 0;
  }
  mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            read can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_id( const INT8U mcp_addr, INT8U* ext, INT32U* id )
{
  INT8U tbufdata[4];

  *ext = 0;
  *id = 0;

  mcp2515_readRegisterS( mcp_addr, tbufdata, 4 );

  *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

  if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M )
  {
    /* extended id                  */
    *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
    *id = (*id << 8) + tbufdata[MCP_EID8];
    *id = (*id << 8) + tbufdata[MCP_EID0];
    *ext = 1;
  }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            write msg
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_canMsg( const INT8U buffer_sidh_addr)
{
  INT8U mcp_addr;
  mcp_addr = buffer_sidh_addr;
  mcp2515_setRegisterS((INT8U)(mcp_addr + 5), m_nDta,  m_nDlc );                /* write data bytes             */
  mcp2515_setRegister((INT8U)(mcp_addr + 4), (INT8U)(m_nDlc |( m_nRtr ? MCP_RTR_MASK : 0 )));                      /* write the RTR and DLC        */
  mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID );                     /* write CAN id                 */

}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            read message
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_canMsg( const INT8U buffer_sidh_addr)        /* read can msg                 */
{
  INT8U mcp_addr, ctrl;

  mcp_addr = buffer_sidh_addr;

  mcp2515_read_id( mcp_addr, &m_nExtFlg, &m_nID );

  ctrl = mcp2515_readRegister( (INT8U)(mcp_addr - 1) );
  m_nDlc = mcp2515_readRegister( (INT8U)(mcp_addr + 4) );

  if ((ctrl & 0x08)) {
    m_nRtr = 1;
  }
  else {
    m_nRtr = 0;
  }

  m_nDlc &= MCP_DLC_MASK;
  mcp2515_readRegisterS( (INT8U)(mcp_addr + 5), &(m_nDta[0]), m_nDlc );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
void MCP_CAN::mcp2515_start_transmit(const INT8U mcp_addr)              /* start transmit               */
{
  mcp2515_modifyRegister( (INT8U)(mcp_addr - 1) , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
INT8U MCP_CAN::mcp2515_getNextFreeTXBuf(INT8U *txbuf_n)                 /* get Next free txbuf          */
{
  INT8U res, i, ctrlval;
  INT8U ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

  res = MCP_ALLTXBUSY;
  *txbuf_n = 0x00;

  /* check all 3 TX-Buffers       */
  for (i = 0; i < MCP_N_TXBUFFERS; i++) {
    ctrlval = mcp2515_readRegister( ctrlregs[i] );
    if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
      *txbuf_n = (INT8U)(ctrlregs[i] + 1);                                 /* return SIDH-address of Buffe */
      /* r                            */
      res = MCP2515_OK;
      return res;                                                 /* ! function exit              */
    }
  }
  return res;
}

/*********************************************************************************************************
** Function name:           MCP_CAN - constructor
** Descriptions:
*********************************************************************************************************/
MCP_CAN::MCP_CAN(SPIClass &spi, INT8U _CS)
{
  smartSPI = 0;
  pSPI=&spi;
  init_CS(_CS);
}

/*********************************************************************************************************
** Function name:           MCP_CAN - constructor - alternative
** Descriptions:
*********************************************************************************************************/
MCP_CAN::MCP_CAN(SPIClass &spi, void (*csFunc)(INT8U val), INT8U _smartSPI)
{
	smartSPI = _smartSPI;
	pSPI = &spi;
	SPICS = 0;
	chipSelect = csFunc;
	// Don't call chipSelect until begin()
}

/*********************************************************************************************************
** Function name:           set CS
** Descriptions:            init CS pin and set UNSELECTED
*********************************************************************************************************/
void MCP_CAN::init_CS(INT8U _CS)
{
  if (_CS == 0) return;
  SPICS = _CS;
  pinMode(SPICS, OUTPUT);
  MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           init
** Descriptions:            init can and set speed
*********************************************************************************************************/
INT8U MCP_CAN::begin(INT8U speedset, const INT8U clockset)
{
  INT8U res;

  pSPI->begin();
  res = mcp2515_init(speedset, clockset);
  if (res == MCP2515_OK) return CAN_OK;
  else return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            init canid Masks
*********************************************************************************************************/
INT8U MCP_CAN::init_Mask(INT8U num, INT8U ext, INT32U ulData)
{
  INT8U res = MCP2515_OK;
#if DEBUG_MODE
  Serial.print("Begin to set Mask!!\r\n");
#else
  delay(10);
#endif
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0) {
#if DEBUG_MODE
    Serial.print("Enter setting mode fall\r\n");
#else
    delay(10);
#endif
    return res;
  }

  if (num == 0) {
    mcp2515_write_id(MCP_RXM0SIDH, ext, ulData);

  }
  else if (num == 1) {
    mcp2515_write_id(MCP_RXM1SIDH, ext, ulData);
  }
  else res =  MCP2515_FAIL;

  res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);
  if (res > 0) {
#if DEBUG_MODE
    Serial.print("Enter normal mode fall\r\n");
#else
    delay(10);
#endif
    return res;
  }
#if DEBUG_MODE
  Serial.print("set Mask success!!\r\n");
#else
  delay(10);
#endif
  return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            init canid filters
*********************************************************************************************************/
INT8U MCP_CAN::init_Filt(INT8U num, INT8U ext, INT32U ulData)
{
  INT8U res = MCP2515_OK;
#if DEBUG_MODE
  Serial.print("Begin to set Filter!!\r\n");
#else
  delay(10);
#endif
  res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  if (res > 0)
  {
#if DEBUG_MODE
    Serial.print("Enter setting mode fall\r\n");
#else
    delay(10);
#endif
    return res;
  }

  switch ( num )
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
      res = MCP2515_FAIL;
  }

  res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);
  if (res > 0)
  {
#if DEBUG_MODE
    Serial.print("Enter normal mode fall\r\nSet filter fail!!\r\n");
#else
    delay(10);
#endif
    return res;
  }
#if DEBUG_MODE
  Serial.print("set Filter success!!\r\n");
#else
  delay(10);
#endif

  return res;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
INT8U MCP_CAN::setMsg(INT32U id, INT8U ext, INT8U len, INT8U rtr, const INT8U *pData)
{
  int i = 0;
  m_nExtFlg = ext;
  m_nID     = id;
  m_nDlc    = len;
  m_nRtr    = rtr;
  for (i = 0; i < MAX_CHAR_IN_MESSAGE; i++)
  {
    m_nDta[i] = *(pData + i);
  }
  return MCP2515_OK;
}


/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
INT8U MCP_CAN::setMsg(INT32U id, INT8U ext, INT8U len, const INT8U *pData)
{
  int i = 0;
  m_nExtFlg = ext;
  m_nID     = id;
  m_nDlc    = len;
  for (i = 0; i < MAX_CHAR_IN_MESSAGE; i++)
  {
    m_nDta[i] = *(pData + i);
  }
  return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            set all message to zero
*********************************************************************************************************/
INT8U MCP_CAN::clearMsg()
{
  m_nID       = 0;
  m_nDlc      = 0;
  m_nExtFlg   = 0;
  m_nRtr      = 0;
  m_nfilhit   = 0;
  for (int i = 0; i < m_nDlc; i++ )
    m_nDta[i] = 0x00;

  return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
INT8U MCP_CAN::sendMsg(bool wait_sent)
{
  INT8U res, res1, txbuf_n;
  uint16_t uiTimeOut = 0;

  do {
    if (uiTimeOut > 0) {
    	delayMicroseconds(10);
    }
    res = mcp2515_getNextFreeTXBuf(&txbuf_n);                       /* info = addr.                 */
    uiTimeOut++;
  } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

  if (uiTimeOut == TIMEOUTVALUE)
  {
    return CAN_GETTXBFTIMEOUT;                                      /* get tx buff time out         */
  }
  mcp2515_write_canMsg( txbuf_n);
  mcp2515_start_transmit( txbuf_n );
#if DEBUG_MODE
  Serial.print("txbuf:"); Serial.println(txbuf_n);
#endif

  if (wait_sent) {
    uiTimeOut = 0;
    do
    {
      if (uiTimeOut > 0) {
    	  delayMicroseconds(10);
      }
      uiTimeOut++;
      res1 = mcp2515_readRegister((INT8U)(txbuf_n - 1));                     /* read send buff ctrl reg  */
      res1 = res1 & 0x08;
    } while (res1 && (uiTimeOut < TIMEOUTVALUE));

    if (uiTimeOut == TIMEOUTVALUE)                                      /* send msg timeout             */
    {
      return CAN_SENDMSGTIMEOUT;
    }
  }

  return CAN_OK;

}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************************************************************************************************/
INT8U MCP_CAN::sendMsgBuf(INT32U id, INT8U ext, INT8U rtr, INT8U len, const INT8U *buf, bool wait_sent)
{
  setMsg(id, ext, len, rtr, buf);
  return sendMsg(wait_sent);
}

/*********************************************************************************************************
** Function name:           sendMsgBufUnconditional
** Descriptions:            send buf using specific tx buffer
*********************************************************************************************************/
INT8U MCP_CAN::sendMsgBufUnconditional(INT8U txNo, INT32U id, INT8U ext, INT8U len, const INT8U *buf)
{
	if (txNo > 2) return CAN_FAIL;

    const INT8U ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

	INT8U txbuf_n =(INT8U)( ctrlregs[txNo] +1); // Most functions take the CTRL register + 1 (a.k.a. TXBnSIDH)

	setMsg(id,ext,len, buf);

	mcp2515_write_canMsg(txbuf_n);
    mcp2515_start_transmit( txbuf_n );

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************************************************************************************************/
INT8U MCP_CAN::sendMsgBuf(INT32U id, INT8U ext, INT8U len, const INT8U *buf, bool wait_sent)
{
  setMsg(id, ext, len, buf);
  return sendMsg(wait_sent);
}

/*********************************************************************************************************
** Function name:           readMsgUnconditional
** Descriptions:            read message from specific rx buf
*********************************************************************************************************/
INT8U MCP_CAN::readMsgUnconditional(const INT8U rxNo)
{
	if (rxNo > 1) return CAN_NOMSG;

	if (0 == rxNo)
	{
		mcp2515_read_canMsg( MCP_RXBUF_0);
		clearInterrupt(MCP_RX0IF);
		return CAN_OK;
	}
	if (1 == rxNo)
	{
		mcp2515_read_canMsg( MCP_RXBUF_1);
		clearInterrupt(MCP_RX1IF);
		return CAN_OK;
	}
	return CAN_NOMSG;
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            read message
*********************************************************************************************************/
INT8U MCP_CAN::readMsg()
{
  INT8U stat, res;

  stat = mcp2515_readStatus();

  if ( stat & MCP_STAT_RX0IF )                                        /* Msg in Buffer 0              */
  {
	  return readMsgUnconditional(0);
  }
  else if ( stat & MCP_STAT_RX1IF )                                   /* Msg in Buffer 1              */
  {
	  return readMsgUnconditional(1);
  }
  else
  {
    res = CAN_NOMSG;
  }
  return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            read message buf
*********************************************************************************************************/
INT8U MCP_CAN::readMsgBuf(INT8U *len, INT8U buf[])
{
  INT8U  rc;

  rc = readMsg();

  if (rc == CAN_OK) {
    *len = m_nDlc;
    for (int i = 0; i < m_nDlc; i++) {
      buf[i] = m_nDta[i];
    }
  } else {
    *len = 0;
  }
  return rc;
}

/*********************************************************************************************************
** Function name:           readMsgBufID
** Descriptions:            read message buf and can bus source ID
*********************************************************************************************************/
INT8U MCP_CAN::readMsgBufID(INT32U *ID, INT8U *len, INT8U buf[])
{
  INT8U rc;
  rc = readMsg();

  if (rc == CAN_OK) {
    *len = m_nDlc;
    *ID  = m_nID;
    for (int i = 0; i < m_nDlc && i < MAX_CHAR_IN_MESSAGE; i++) {
      buf[i] = m_nDta[i];
    }
  } else {
    *len = 0;
  }
  return rc;
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            check if got something
*********************************************************************************************************/
INT8U MCP_CAN::checkReceive(void)
{
  INT8U res;
  res = mcp2515_readStatus();                                         /* RXnIF in Bit 1 and 0         */
  if ( res & MCP_STAT_RXIF_MASK )
  {
    return CAN_MSGAVAIL;
  }
  else
  {
    return CAN_NOMSG;
  }
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            if something error
*********************************************************************************************************/
INT8U MCP_CAN::checkError(void)
{
  INT8U eflg = mcp2515_readRegister(MCP_EFLG);

  if ( eflg & MCP_EFLG_ERRORMASK )
  {
    return CAN_CTRLERROR;
  }
  else
  {
    return CAN_OK;
  }
}

/*********************************************************************************************************
** Function name:           getCanId
** Descriptions:            when receive something ,u can get the can id!!
*********************************************************************************************************/
INT32U MCP_CAN::getCanId(void)
{
  return m_nID;
}

/*********************************************************************************************************
** Function name:           isRemoteRequest
** Descriptions:            when receive something ,u can check if it was a request
*********************************************************************************************************/
INT8U MCP_CAN::isRemoteRequest(void)
{
  return m_nRtr;
}

/*********************************************************************************************************
** Function name:           isExtendedFrame
** Descriptions:            did we just receive standard 11bit frame or extended 29bit? 0 = std, 1 = ext
*********************************************************************************************************/
INT8U MCP_CAN::isExtendedFrame(void)
{
  return m_nExtFlg;
}

/*********************************************************************************************************
** Function name:           getData
** Descriptions:            Return data from latest received message
*********************************************************************************************************/
void MCP_CAN::getData(INT8U *len, INT8U buf[])
{
	*len = m_nDlc;
	for(int i=0; i < m_nDlc; i++) {
		buf[i] = m_nDta[i];
	}
}

/*********************************************************************************************************
** Function name:           enableInterrupt
** Descriptions:            Enable interrupts defined in mask
*********************************************************************************************************/
void MCP_CAN::enableInterrupt(INT8U mask)
{
	mcp2515_modifyRegister(MCP_CANINTE, mask, 0xFF);
}

/*********************************************************************************************************
** Function name:           disableInterrupt
** Descriptions:            Disable interrupts defined in mask
*********************************************************************************************************/
void MCP_CAN::disableInterrupt(INT8U mask)
{
	mcp2515_modifyRegister(MCP_CANINTE, mask, 0);
}

/*********************************************************************************************************
** Function name:           clearInterrupt
** Descriptions:            Clear interrupts defined in ints
*********************************************************************************************************/
void MCP_CAN::clearInterrupt(INT8U ints)
{
	mcp2515_modifyRegister(MCP_CANINTF, ints, 0);
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
