/*
 * FreeModbus Libary: Linux Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.3 2006/10/12 08:35:34 wolti Exp $
 */

/* ----------------------- Standard includes --------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "sttype.h"
#include "port.h"
#include "tpthread.h"
#include "midbus.h"
#include "serailbussfile.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"
#include "mbrtu.h"

/* ----------------------- Defines  -----------------------------------------*/
#if MB_ASCII_ENABLED == 1
#define BUF_SIZE    513         /* must hold a complete ASCII frame. */
#else
#define BUF_SIZE    256         /* must hold a complete RTU frame. */
#endif

/* ----------------------- Static variables ---------------------------------*/
 int      iSerialFd = -1;
// static int 		iSerialFd_minib = -1;
// static int      iSerialFd_temper = -1;
static bool     bRxEnabled;
static bool     bTxEnabled;

static unsigned long    ulTimeoutMs;
static unsigned char    ucBuffer[BUF_SIZE];
static int      uiRxBufferPos;
static int      uiTxBufferPos;

static struct termios xOldTIO;
// static eMBEventType myEvent = EV_READY;


/* ----------------------- Function prototypes ------------------------------*/
 bool     prvbMBPortSerialRead( unsigned char * pucBuffer, unsigned short usNBytes, unsigned short * usNBytesRead );
 bool     prvbMBPortSerialWrite( unsigned char * pucBuffer, unsigned short usNBytes );

/* ----------------------- Begin implementation -----------------------------*/
void
vMBPortSerialEnable( bool bEnableRx, bool bEnableTx )
{
	int fd;
	
    /* it is not allowed that both receiver and transmitter are enabled. */
    assert( !bEnableRx || !bEnableTx );
/*
	if (thread_id_miniboard == (int)pthread_self())///////////////////////////
		fd = iSerialFd_minib;
	else
		fd = iSerialFd_temper;
	*/
	fd = iSerialFd;
    if( bEnableRx )
    {
        ( void )tcflush( fd, TCIFLUSH );
        uiRxBufferPos = 0;
        bRxEnabled = true;
    }
    else
    {
        bRxEnabled = false;
    }
    if( bEnableTx )
    {
        bTxEnabled = true;
        uiTxBufferPos = 0;
    }
    else
    {
        bTxEnabled = false;
    }

}

bool
xMBPortSerialInit( unsigned char ucPort, unsigned long ulBaudRate, unsigned char ucDataBits, eMBParity eParity )
{
    char            szDevice[16];
    bool            status = true;
	int fd;

    struct termios new_opt;
	int baud_rate;

    sprintf( szDevice,  "/dev/ttyS%d", ucPort );

    if( ( fd = open( szDevice, O_RDWR | O_NOCTTY | O_NONBLOCK) ) < 0 )
    {
        vMBPortLog( MB_LOG_ERROR, "SER-INIT", "Can't open serial port %s: %s\n", szDevice,
                    strerror( errno ) );
    }
    else
    {
      //get the current config -> new_opt
        serail_hand[TEM_PORT] = fd;
        tcgetattr(fd,&new_opt);
        bzero( &new_opt, sizeof(new_opt));

        switch ( ulBaudRate )
        {
        case 9600:
            baud_rate = B9600;
            break;
        case 19200:
            baud_rate = B19200;
            break;
        case 38400:
            baud_rate = B38400;
            break;
        case 57600:
            baud_rate = B57600;
            break;
        case 115200:
            baud_rate = B115200;
            break;
        default:
            baud_rate = false;
        }

        tcflush(fd, TCIOFLUSH);
        //setup input/output baudrate
        cfsetispeed(&new_opt,baud_rate);
        //printf("cfsetispeed::c_cflag = %x\r\n", new_opt.c_cflag);
        cfsetospeed(&new_opt,baud_rate);
        //printf("cfsetospeed::c_cflag = %x\r\n", new_opt.c_cflag);

        if (tcsetattr(fd, TCSANOW, &new_opt) != 0)
        {
            perror("tcsetattr::set baud rate failed\n");
            return false;
        }

        //修改控制模式，保证程序不会占用串口？
        new_opt.c_cflag |= CLOCAL;
        //printf("c_cflag |= CLOCAL => %x\r\n", new_opt.c_cflag);

        //修改控制模式，使得能够从串口读取输入数据
        new_opt.c_cflag |= CREAD;
        //printf("c_cflag |= CREAD => %x\r\n", new_opt.c_cflag);

        new_opt.c_cflag |= HUPCL;
        //setup control flow
	
		//no control-flow
		new_opt.c_cflag &= ~CRTSCTS;
		
        //printf("c_cflag(no ctl-flow) = %x\r\n", new_opt.c_cflag);

        //setup bit size
        new_opt.c_cflag &=~CSIZE;
        switch(ucDataBits)
        {
        case '5':
            new_opt.c_cflag |=CS5;
            break;
        case '6':
            new_opt.c_cflag |=CS6;
            break;
        case '7':
            new_opt.c_cflag |=CS7;
            break;
        case '8':
            new_opt.c_cflag |=CS8;
            break;
        default:
            new_opt.c_cflag |=CS8;
        }
        //	printf("c_cflag |= CS8 => %x\r\n", new_opt.c_cflag);

        //setup parity
        switch(eParity)
        {
        
        case MB_PAR_NONE:
            new_opt.c_cflag &= ~PARENB;   /* Clear parity enable */
            new_opt.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;

        case MB_PAR_ODD:
            new_opt.c_cflag |= (PARODD | PARENB);	/* 设置为奇效验*/
            new_opt.c_iflag |= INPCK;				/* Disable parity checking */
            break;

        case MB_PAR_EVEN:
            new_opt.c_cflag |= PARENB;		/* Enable parity */
            new_opt.c_cflag &= ~PARODD;		/* 转换为偶效验*/
            new_opt.c_iflag |= INPCK;       /* Disable parity checking */
            break;

        default:
            fprintf(stderr,"Unsupported parity\n");
            return false;
        }
        //printf("c_cflag &=~PARENB => %x\r\n", new_opt.c_cflag);


        //setup stop-bit 1bit
        //new_opt.c_cflag &=~CSTOPB;
        
        //setup stop-bit 2bit
        new_opt.c_cflag |=CSTOPB;
        
        //printf("c_cflag &=~CSTOPB => %x\r\n", new_opt.c_cflag);

        /* Set input parity option */
        if ((eParity != MB_PAR_NONE))
        {
            new_opt.c_iflag |= INPCK;
        }

        //修改输出模式：原始数据输出(raw 模式)
        new_opt.c_lflag &= ~(ICANON | ECHO | ISIG);				/*Input*/
        new_opt.c_oflag &= ~OPOST;								/*Output*/

        //修改控制字符：读取字符的最少个数为1 ？？？
        new_opt.c_cc[VMIN]=1;

        //修改控制字符：读取第一个字符的超时时间为1×100ms
        new_opt.c_cc[VTIME]=1;

        //试图去掉在接收时必须收到'\n'才返回的问题
        //忽略输入的回车
        //new_opt.c_iflag |= IGNCR;
        //new_opt.c_iflag &= ~(IXON|IXOFF|IXANY);

        //如果发生数据溢出，接收数据，但是不再读取
        tcflush(fd,TCIFLUSH);

        
        if(tcsetattr(fd,TCSANOW,&new_opt) != 0)
        {
            perror("Cannot set the serial port parameters");
            return false;
        }
    }
	iSerialFd = fd;
	/*
	if (thread_id_miniboard == (int)pthread_self())///////////////////////////
		iSerialFd_minib = fd ;
	else
		iSerialFd_temper = fd ;
		*/
	return status; 
}


bool
xMBPortSerialSetTimeout( unsigned long ulNewTimeoutMs )
{
    if( ulNewTimeoutMs > 0 )
    {
        ulTimeoutMs = ulNewTimeoutMs;
    }
    else
    {
        ulTimeoutMs = 1;
    }
    return true;
}

void
vMBPortClose( void )
{
	int fd;
/*	
	if (thread_id_miniboard == (int)pthread_self())///////////////////////////
		fd = iSerialFd_minib;
	else
		fd = iSerialFd_temper;
*/
	fd = iSerialFd;
    if( fd != -1 )
    {
        ( void )tcsetattr( fd, TCSANOW, &xOldTIO );
        ( void )close( fd );
        fd = -1;
    }
}

bool
prvbMBPortSerialRead( unsigned char * pucBuffer, unsigned short usNBytes, unsigned short * usNBytesRead )
{
    bool  bResult = true;
    ssize_t         res;
    fd_set          rfds;
	// int i;
	int fd = -1;
    struct timeval  tv;
    /*
	if (thread_id_miniboard == (int)pthread_self())///////////////////////////
		fd = iSerialFd_minib;
	else
		fd = iSerialFd_temper;
	*/
	fd = iSerialFd;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    FD_ZERO( &rfds );
    FD_SET( fd, &rfds );
    /* Wait until character received or timeout. Recover in case of an
     * interrupted read system call. */
    do
    {
        if( select( fd + 1, &rfds, NULL, NULL, &tv ) == -1 )
        {
            if( errno != EINTR )
            {
            printf("bResult error");
                bResult = false;
            }
        }
        else if( FD_ISSET( fd, &rfds ) )
        {
        	
            if( ( res = read( fd, pucBuffer, usNBytes ) ) == -1 )
            {
            	  printf("bResult error");
                bResult = false;
            }
            else
            {
            //	tcflush(fd,TCIFLUSH);
            #if 0
            	printf("modbus recieved usNBytes=%d\n",usNBytes);
				for (i = 0; i < res;i++)
					printf(" %x ",pucBuffer[i]);
			#endif
                *usNBytesRead = ( unsigned short ) res;
                break;
            }
        }
        else
        {
            *usNBytesRead = 0;
            break;
        }
    }
    while( bResult == true );
    return bResult;
}

bool
prvbMBPortSerialWrite( unsigned char * pucBuffer, unsigned short usNBytes )
{
    ssize_t         res;
    size_t          left = ( size_t ) usNBytes;
    size_t          done = 0;
	int fd;  //,i;
    /*	
        if (thread_id_miniboard == (int)pthread_self())///////////////////////////
                fd = iSerialFd_minib;
            else
                fd = iSerialFd_temper;
    */
	fd = iSerialFd;
    /*
    while(1)
    {
        printf("writing =%x\n",pucBuffer[0]);
        if( ( res = write( fd,pucBuffer, 8) ) == -1 )
    {	
        printf("write error\n");
        
    }
    //	return TRUE;
        usleep(100000);
    }
    */
        
    //printf("writefd = %d\n", fd);
    if (big_version)
 	{
		GPIO_OutClear(RS485RW);
        usleep(10000);
    // printf("writerwing\n");
 	}

    while( left > 0 )
    {
    #if 0
            printf("writing");
            printf("\n");
        for(i = 0; i < left;i++)
            printf(" %x ",*(pucBuffer + done + i));
        printf("\n");
    #endif
        if( ( res = write( fd, pucBuffer + done, left ) ) == -1 )
        {	
        	printf("write error\n");
            if( errno != EINTR )
            {
                break;
            }
            /* call write again because of interrupted system call. */
            continue;
        }
        done += res;
        left -= res;
    }
	if (big_version)
    {
		usleep(10000);
		GPIO_OutSet(RS485RW);
    }
    //	printf("done = %d\n", done);
    return left == 0 ? true : false;
}


bool
xMBPortSerialPoll(  )
{
    bool  bStatus = true;
    unsigned short          usBytesRead;
    int             i;
    //	printf("in xMBPortSerialPoll\n");
    while( bRxEnabled )
    {
        //	if (xMBPortEventGet(&myEvent))
        //	if (myEvent == EV_FRAME_RECEIVED)
        //		break;
        if( prvbMBPortSerialRead( &ucBuffer[0], BUF_SIZE, &usBytesRead ) )
        {
            if( usBytesRead == 0 )
            {
                /* timeout with no bytes. */
                break;
            }
            else if( usBytesRead > 0 )
            {	
                for( i = 0; i < usBytesRead; i++ )
                {
                    /* Call the modbus stack and let him fill the buffers. */
                    ( void )pxMBFrameCBByteReceived(  );	
                }	
                uiRxBufferPos = 0;
            }
        }
        else
        {
            vMBPortLog( MB_LOG_ERROR, "SER-POLL", "read failed on serial device: %s\n",
                        strerror( errno ) );
            bStatus = false;
        }
    }
    if( bTxEnabled )
    {
        while( bTxEnabled )
        {
            ( void )pxMBFrameCBTransmitterEmpty(  );
            /* Call the modbus stack to let him fill the buffer. */
        }
		
        if( !prvbMBPortSerialWrite( &ucBuffer[0], uiTxBufferPos ) )
        {
            vMBPortLog( MB_LOG_ERROR, "SER-POLL", "write failed on serial device: %s\n",
                        strerror( errno ) );
            bStatus = false;
        }	
    }

    return bStatus;
}

bool
xMBPortSerialPutByte(char ucByte )
{
    assert( uiTxBufferPos < BUF_SIZE );
    ucBuffer[uiTxBufferPos] = ucByte;
    uiTxBufferPos++;
    return true;
}

bool
xMBPortSerialGetByte( char * pucByte )
{
    assert( uiRxBufferPos < BUF_SIZE );
    *pucByte = ucBuffer[uiRxBufferPos];
    uiRxBufferPos++;

	
    return true;
}
