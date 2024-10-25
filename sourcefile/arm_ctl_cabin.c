

#include "mb.h"
#include "mbrtu.h"
#include "portserial.h"
#include "midbus.h"
#include "tpthread.h"
#include "arm_ctl_cabin.h"
#include "can.h"
#include "port.h"
#include "cantool.h"




typedef union u_canframe_data
{
    unsigned char            data[8];

    struct
    {
        unsigned int       dl;
        unsigned int       dh;
    } s;
} u_canframe_data_t;




extern pthread_mutex_t mutex_armcabinlock;
volatile bool ArmCabinWork_finished[3] = {FALSE};
volatile bool ArmCabinWork_needreport[3] = {FALSE};
ArmCabin_t ArmCabinCmdList[3];
volatile bool unload[3] = {FALSE};


#define CAN_Id_Standard             ((unsigned int)0x00000000)  /*!< Standard Id */
#define CAN_Id_Extended             ((unsigned int)0x00000004)  /*!< Extended Id */
#define CAN_RTR_Data                ((unsigned int)0x00000000)  /*!< Data frame */


unsigned int LOCAL_CAN_ID  = 0;

#define ArmOFFSET  1 //留一位地址为

//shelf = 0 ,1,2
/***************************************************/ //   
int SendCmdArmCtlCabin(char cmd, uint8_t shelf,char* data)
{
	uint8_t i = 0;
	// unsigned short crc=0;
//	 char cmdbuffer[10] ={0};
	//  char Ackbuffer[6] ={'M', 2, 0, 0X40, 0X46, 0X88};
	// unsigned char * rtu;
	// unsigned short	rtu_long;
	// char serail_recvbuf[50];
	static char seq[3] = {0};
	int eor_cnt=0, timout_cnt=0;
	//  byte *psendframe = (byte *)&sendframe;
 //   u_canframe_data_t *psend_data = (u_canframe_data_t *)sendframe.data;
	//  bool carry_bit = FALSE;// 进位标志
	 int  ret = 0;
	 S_CanFrame sendframe;

	if (seq[shelf] > 7)
		seq[shelf] = 0;	
	
	//sendframe.can_id = ntohl((send_frame_id & CAN_EFF_MASK) | CAN_EFF_FLAG);
//	sendframe.can_id = ((send_frame_id & CAN_EFF_MASK) | CAN_EFF_FLAG);
	sendframe.can_id = 0;//ID 必须为0
//	printf("can_id = %x ", sendframe.can_id);
	sendframe.can_dlc = 8;

	sendframe.data[0] = shelf;
	sendframe.data[1] = 6;
	sendframe.data[2] = seq[shelf];
	sendframe.data[3] = cmd;
	memcpy(&sendframe.data[4], data, 4);

//	if (PRINTF)
	{
		printf("the data send to shelf\n");
		for (i = 0;i < 8;i++)
			printf(" %x",sendframe.data[i]);
	}
	ret = can_sendframe(send_socket_fd, (const uint8_t *)&sendframe, sizeof(sendframe));
	#if(USE_PRINT_LOG == 1)
		can_printframe(sendframe.can_id & CAN_EFF_MASK, sendframe.data, sendframe.can_dlc, 
			((sendframe.can_id & CAN_EFF_FLAG) ? TRUE : FALSE),
			ret > 0 ? TRUE : FALSE, 
			TRUE);
	#endif
	
	while(CanReState[shelf] != ARM_CABIN_ACK)
	{
			if(timout_cnt++ > 1000)
			 {
			 timout_cnt = 0;
			 	if (eor_cnt++ > 9)
			 	{
			 		mb_seterrorcode(CONNECT_ERROR_ARM_CABINA + shelf);
					return -1;
			 	}
			
				
					printf("recieve shelf timeout send again\n");
					sendframe.data[2] |= 0X80;
					for (i = 0;i < 8;i++)
						printf(" %x",sendframe.data[i]);
					ret = can_sendframe(send_socket_fd, (const uint8_t *)&sendframe, sizeof(sendframe));

					#if(USE_PRINT_LOG == 1)
					can_printframe(sendframe.can_id & CAN_EFF_MASK, sendframe.data, sendframe.can_dlc, 
						((sendframe.can_id & CAN_EFF_FLAG) ? TRUE : FALSE),
						ret > 0 ? TRUE : FALSE, 
						TRUE);
					#endif
				
			 }
		 
		usleep(1000);
	}

	timout_cnt = eor_cnt = 0;

	while(CanReState[shelf] != ARM_CABIN_RESP && CanReState[shelf] != ARM_CABIN_EOR && CanReState[shelf] != ARM_CABIN_STATE)
	{
		if(timout_cnt++ > 1000)
		{
			printf("shelf resp timeout\n");
		//	return -1;
		}
	usleep(100000);
	}

	
	return 1;

return 0;

}







