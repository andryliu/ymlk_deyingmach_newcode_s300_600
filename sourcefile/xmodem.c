/*
	Copyright 2001, 2002 Georges Menie (www.menie.org)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/* this code needs standard functions memcpy() and memset()
   and input/output functions port_inbyte() and port_outbyte().

   the prototypes of the input/output functions are:
     int port_inbyte(unsigned short timeout); // msec timeout
     void port_outbyte(int c);

 */

#include "crc16.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include "port.h"



#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define XMODENACK  0x06
#define NAK  0x15

#define CAN  0x18

//#define CAN  0x06

#define CTRLZ 0x1A

#define DLY_1S 1000
#define MAXRETRANS 25



extern unsigned char scan_data[];
extern unsigned int scan_data_len;
unsigned char packetno = 1;
unsigned char firstpacket = 1;
int len = 0;






static int last_error = 0;
/****************Portting Start *******************/
#include "string.h"

void port_outbyte(int m_fd, unsigned char trychar)
{
	unsigned char buf[2];
	buf[0] = trychar;
//	lowLevel_write(buf,1);
 //   WritePort(buf,1);
//	printf("port_outbyte send % xm_fd = %d\n",trychar,m_fd);
	tcflush( m_fd,TCOFLUSH);
	
	write(m_fd, buf, 1);
}


unsigned char port_inbyte(int m_fd, unsigned char *dest, unsigned int time_out)
{
	unsigned char ch;
	int i; // len,j;
	last_error = 0;

//	printf("time_out=%d",time_out);
	for(i = 0;i < time_out;i++)
	{
	    if(read( m_fd, &ch, 1) == 1 )
	    {    	
	    //	printf("**** %x*** ",ch);
		//	printf("out of port_inbyte\n");
	        return ch;
	    }
	//	printf("read m_fd again");
		usleep(10000);
	}
	last_error = 1;
//	printf("out of port_inbyte\n");
	return ch;
}


/****************Portting End*******************/
static int check(int crc, const unsigned char *buf, int sz)
{
	if (crc)
	{
		unsigned short crc = crc16_ccitt((const char*)buf, sz);
		unsigned short tcrc = (buf[sz]<<8)+buf[sz+1];
		if (crc == tcrc)
			return 1;
	}
	else
	{
		int i;
		unsigned char cks = 0;
		for (i = 0; i < sz; ++i)
		{
			cks += buf[i];
		}
		if (cks == buf[sz])
		return 1;
	}

	return 0;
}

static void flushinput(int m_fd)
{
	tcflush(m_fd,TCIOFLUSH);
		;
}

int xmodemReceive(int m_fd, const unsigned char *dest, int destsz)
{
    unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
    unsigned char *p;
    int bufsz, crc = 0;
    int retry, retrans = MAXRETRANS;
    unsigned char trychar = 'C';
	unsigned char first_flame = 1;
    int i = 0, c = 0;
	//const char    Buf3[3]={0x16, 0x55, 0x0D};
	
	while(1)
	{
		for( retry = 0; retry < 16; ++retry)
		{
			
			if (trychar)
				port_outbyte(m_fd, trychar);
		//		while(1);
			c = port_inbyte(m_fd, (uint8_t *)dest, DLY_1S);
			if (last_error == 0)
			{
				switch (c)
				{
					case SOH:
						printf("SOH\n");
						bufsz = 128;
						goto start_recv;
					case STX:
						printf("STX\n");
						bufsz = 1024;
					
						goto start_recv;
					case EOT:
						printf("EOT\n");
						flushinput(m_fd);
						port_outbyte(m_fd,XMODENACK);
						packetno = 1;
						firstpacket = 1;
					//	write(m_fd,  Buf3, 3);
						return 2; /* normal end */
					case CAN:
						printf("CAN\n");
						// c = port_inbyte(m_fd, (unsigned char*)(*dest), DLY_1S);
						c = port_inbyte(m_fd, (unsigned char*)dest, DLY_1S);
						if (c == CAN)
						{
							flushinput(m_fd);
							port_outbyte(m_fd,XMODENACK);
							return -1; /* canceled by remote */
						}
						break;
						case 0:
							printf("0\n");
							flushinput(m_fd);
							trychar = XMODENACK;
						break;
						case 0x64:
							printf("0x64\n");
							flushinput(m_fd);
							trychar = XMODENACK;
							break;
					default:
						
						break;
				}
			}
		}
		/*	if (trychar == 'C')
			{
				trychar = NAK;	//超时按基本版本传输
				continue;
			}
		*/
		//	printf("retry agin\n");
		
		#if 1
        port_outbyte(m_fd,CAN);
		port_outbyte(m_fd,CAN);
		port_outbyte(m_fd,CAN);
		#endif
		//		port_outbyte(ACK);
		//		port_outbyte(ACK);
		//		port_outbyte(ACK);
		return -2; /* sync error */

	start_recv:
		//	if (trychar == 'C') crc = 1;
		if (first_flame == 1) crc = 1;
		first_flame = 0;
		trychar = 0;
		p = xbuff;
		*p++ = c;
		//	printf("bufsz+(crc?1:0)+3=%d\n",bufsz+(crc?1:0)+3);
		for (i = 0;  i < (bufsz+(crc?1:0)+3); ++i)
		{
			//c = port_inbyte(m_fd, (unsigned char*)(*dest), DLY_1S * 100);
			c = port_inbyte(m_fd, (unsigned char*)dest, DLY_1S * 100);
			//		printf(" %x",c);
			if (last_error != 0)
				goto reject;
			*p++ = c;
		}
		//	printf("i=%d\n",i);

		if (xbuff[1] == (unsigned char)(~xbuff[2])
			&&	check(crc, &xbuff[3], bufsz)&&
			(xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno-1))
		{	

			if (xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno-1)
			{
	 			//int count = destsz - len;
	 			int count = destsz-1;
				if (count > bufsz)
					count = bufsz;
                if (count > 0)
				{
			//		printf("count=%d\n",count);
					if (firstpacket)
					{
						i = 20;	//30号开始找 ~
						while (1)
						{
							if (xbuff[i] == 0x7e)
							{
								if ((xbuff[i + 1] == 0x1d) && (xbuff[i + 2] == 0x42) && (xbuff[i + 3] == 0x4d))
								{
									printf("find\n");
									i += 2;
									count = count - i + 3;
								
									 memcpy (scan_data, &xbuff[i], count);
									
									 scan_data_len = count;
									 printf("scan_data_len=%d\n",scan_data_len);


		                     	// 	  dest[count] = xbuff[0];
		                     		break;
								}
								else 
									continue;		
							}
							i++;
						}
					/*	
                        for (i = 0;i < count;i++)
						{
							printf(" %x",dest[i]);
						}
					*/
						firstpacket = 0;
					}	
                    else
                    {
                    	
                        memcpy (scan_data + scan_data_len, &xbuff[3], count);
						scan_data_len += count;
                    //    dest[count] = xbuff[0];				
                    }

					
                }

				++packetno;
				retrans = MAXRETRANS+1;
			}
			if (--retrans <= 0)
			{

				flushinput(m_fd);
				port_outbyte(m_fd,CAN);
				port_outbyte(m_fd,CAN);
				port_outbyte(m_fd,CAN);
				return -3; /* too many retry error */
			}
			port_outbyte(m_fd,XMODENACK);
			continue;
		}

		printf("check error\n");
	reject:
	//	printf("i=%d\n",i);
		flushinput(m_fd);
		port_outbyte(m_fd,NAK);
	}
}


