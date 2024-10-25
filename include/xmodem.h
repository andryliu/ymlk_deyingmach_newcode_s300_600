#ifndef _XMODEM_H
#define _XMODEM_H

void port_outbyte(unsigned char trychar);
unsigned char port_inbyte(int m_fd,unsigned char *dest,unsigned int time_out);

int xmodemReceive(int m_fd,const unsigned char *dest, int destsz);

int xmodemTransmit(unsigned char *src, int srcsz);

#endif
