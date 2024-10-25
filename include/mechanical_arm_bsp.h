

#ifndef __MECHANICAL_ARM_BSP_H__
#define __MECHANICAL_ARM_BSP_H__


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include  <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <time.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <assert.h>
#include <pthread.h>  
#include <signal.h>

#include <linux/if.h>
#include <linux/mii.h>
#include <linux/sockios.h>

#include "temperctlmy.h"

#include "common.h"
#include "tpthread.h"
#include "midbus.h"
#include "temper_control.h"
#include "xmodem.h"
#include "mini_board.h"
#include "xmodem.h"
#include "mbcrc.h"
#include "crc16.h"
 #include "port.h"
 
#include "arm_ctl_cabin.h"







#define ARMMOTO_RUNPARA_INI   "armmotor_runpara_ini"




extern void bsp_arm_run(void);




#endif

