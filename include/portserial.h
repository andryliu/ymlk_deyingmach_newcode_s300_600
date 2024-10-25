/*
 * portserial.h
 *
 *  Created on: 2013-9-10
 *      Author: Administrator
 */

#ifndef PORTSERIAL_H_
#define PORTSERIAL_H_

#include "port.h"


void vMBPortSerialEnable( bool bEnableRx, bool bEnableTx );
bool prvbMBPortSerialRead( unsigned char * pucBuffer, unsigned short usNBytes, unsigned short * usNBytesRead );
bool prvbMBPortSerialWrite( unsigned char * pucBuffer, unsigned short usNBytes );
bool xMBPortSerialPoll(  );
#endif /* PORTSERIAL_H_ */
