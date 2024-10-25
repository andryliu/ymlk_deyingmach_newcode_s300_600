


#ifndef __CANTOOL_H__
#define __CANTOOL_H__

#include "string.h"
#include "port.h"




void can_printframe(const uint32_t frame_id, const char *data, const uint8_t len, 
    const bool extended, 
    const bool ok_flag, 
    const bool sendflag);

int can_sendframe(const int sockfd, const uint8_t* data, const int count);

int recv_frame(const int sockfd, uint8_t* buf, const int count, const int timeout_ms);
int can_recvframe(const int sockfd, uint8_t* buf, const int count, const int timeout_ms);


#endif
