/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"
#include <stdio.h>


/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     3       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;

volatile unsigned char  ucRTUBuf[MB_SER_PDU_SIZE_MAX];

unsigned char *pucSndBufferCur;
static unsigned short usSndBufferCount;

static volatile unsigned short usRcvBufferPos;

volatile bool data_recieved = false;
volatile bool datacmp_recieved = false;

volatile bool dataMixstation_recieved = false;
volatile bool dataArmCabin_recieved = false;



void ClearBuf(void)
{
	memset((uint8_t*)ucRTUBuf, 0, sizeof(ucRTUBuf));
    usRcvBufferPos = 0;
}

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBRTUInit( unsigned char ucSlaveAddress, unsigned char ucPort, unsigned long ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    unsigned long  usTimerT35_50us;

    ( void )ucSlaveAddress;
    ENTER_CRITICAL_SECTION(  );

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( ucPort, ulBaudRate, 8, eParity ) != true )
    {
        eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
            usTimerT35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
        }
        if( xMBPortTimersInit( ( unsigned short ) usTimerT35_50us ) != true )
        {
            eStatus = MB_EPORTERR;
        }
    }
    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

void
eMBRTUStart( void )
{
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable( TRUE, FALSE );
    vMBPortTimersEnable(  );

    EXIT_CRITICAL_SECTION(  );
}

void
eMBRTUStop( void )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE );
    vMBPortTimersDisable(  );
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBRTUReceive( unsigned char * pucRcvAddress, unsigned char ** pucFrame, unsigned short * pusLength )
{
    bool xFrameReceived = false;
    eMBErrorCode    eStatus = MB_ENOERR;
	// int i;
	
    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

#if 0
//	if ( ucRTUBuf[0] == 35)
		{
	printf("ucRTUBuf = ");
	for (i = 0;i < usRcvBufferPos; i++)
		printf(" %x ", ucRTUBuf[i]);
	printf("\n");
		}
#endif	
	if (ucRTUBuf[0] == 33)
		datacmp_recieved = true;
	if (ucRTUBuf[0] == 32)
		dataMixstation_recieved = true;
/*	
short tst = 0;
	tst = usMBCRC16( ( unsigned char * ) ucRTUBuf, usRcvBufferPos -2);
	printf("CRC =%x\n",tst);	
*/
    /* Length and CRC check */
#if 1
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( unsigned char * ) ucRTUBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( unsigned short )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( unsigned char * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];
        xFrameReceived = true;
		data_recieved = true;
		if (ucRTUBuf[0] == 35)
		dataArmCabin_recieved = true;
	
//		printf("dataok\n");
    }
    else
#endif
    {
    	printf("datafail\n");
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
//	printf("eMBRTUReceive eStatus=%d, MB_ENOERR=%d",eStatus,MB_ENOERR);
    return eStatus;
}


unsigned char	 sendBuffer[256];			
eMBErrorCode									//注意传入参数前面一位必须申请内存
eMBRTUSend( unsigned char ucSlaveAddress, const unsigned char * pFrame, unsigned short usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    unsigned short          usCRC16;
	unsigned char * pucFrame = &sendBuffer[1];

	memcpy(&sendBuffer[1], pFrame, usLength);

    ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
      //  eRcvState = STATE_RX_IDLE;
   // if( eRcvState == STATE_RX_IDLE )
   if (true)
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( unsigned char * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( unsigned char * ) pucSndBufferCur, usSndBufferCount );
        pucSndBufferCur[usSndBufferCount] = ucRTUBuf[usSndBufferCount++] = ( unsigned char )( usCRC16 & 0xFF );
        pucSndBufferCur[usSndBufferCount] = ucRTUBuf[usSndBufferCount++] = ( unsigned char )( usCRC16 >> 8 );
	    //	printf("crc = %x %x\n", pucSndBufferCur[usSndBufferCount-2],pucSndBufferCur[usSndBufferCount-1]);
        /* Activate the transmitter. */
        eSndState = STATE_TX_XMIT;

        vMBPortSerialEnable( false, true );
		
    }
    else
    {
    	printf("STATE_RX_IDLE = false\n");
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
	//while(eSndState != STATE_TX_IDLE) {usleep(10000); }
    return eStatus;
}

bool xMBRTUReceiveFSM( void )
{
    bool  xTaskNeedSwitch = false;
    unsigned char           ucByte;

    assert( eSndState == STATE_TX_IDLE );
    //	printf("in recieveing\n");
    /* Always read the character. */
    ( void )xMBPortSerialGetByte( ( char * ) & ucByte );

    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
    	//	printf("STATE_RX_INIT\n");
        vMBPortTimersEnable(  );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
	    //	printf("STATE_RX_ERROR\n");
        vMBPortTimersEnable(  );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:
	    //	printf("STATE_RX_IDLE\n");
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte;
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        vMBPortTimersEnable(  );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
	    //	printf("STATE_RX_RCV\n");
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
        }
        else
        {
            eRcvState = STATE_RX_ERROR;
        }
        vMBPortTimersEnable(  );
        break;
    }
    return xTaskNeedSwitch;
}


bool xMBRTUTransmitFSM( void )
{
    bool  xNeedPoll = false;

    //   assert( eRcvState == STATE_RX_IDLE );
    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( true, false );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutByte( ( char )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
        }
        else
        {
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable( true, false );
            eSndState = STATE_TX_IDLE;
	        //		printf("disable trans\n");
        }
        break;
    }

    return xNeedPoll;
}

bool xMBRTUTimerT35Expired( void )
{
   bool xNeedPoll = false;

    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost( EV_READY );
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED );
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        assert( ( eRcvState == STATE_RX_INIT ) ||
                ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
    }

    vMBPortTimersDisable(  );
    eRcvState = STATE_RX_IDLE;

    return xNeedPoll;
}


bool read_frame(unsigned char **ucMBFrame, unsigned char *ucRcvAddress,unsigned short *usLength)
{
	// int i;
	unsigned char   *ucMBFrame1;
    unsigned char    ucRcvAddress1;
    // unsigned char    ucFunctionCode1;
    unsigned short   usLength1;
	

	eMBRTUReceive(&ucRcvAddress1, &ucMBFrame1, &usLength1);

//	printf("rcvaddr =%d receive frame = \n", ucRcvAddress1);
//	for(i = 0; i < usLength1; i++)
//		printf(" %x ", ucMBFrame1[i]);
//	printf("\n");
    return true;
}

void get_frame(unsigned char ** rtu, unsigned short * rtu_long)
{
	// *rtu = &ucRTUBuf[0];
    *rtu = &(ucRTUBuf[0]);
	*rtu_long = usRcvBufferPos;
}


