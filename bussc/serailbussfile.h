

#ifndef __SERIAL_BUSSECES_H__
#define __SERAIL_BUSSECES_H__

#include "sttype.h"


//------ Arm Address, Device Address----------------------
#define		ARM_ADDR		0x38
#define		DEVICE_ADDR_8	0x38		/* X, Y, and Z arm commands */
#define		DEVICE_ADDR_9	0x39		/* Interactive motor drive control (Molo-X) */
#define		MASTER_ADDR		0x30		/* The master address for pump */
#define 	PUMP_ADDR  		0X31		/* Address single device */

#define	ACK								0x40
#define ANSWER_OK						0x50
#define	ANSWER_INVAILD_ADDR				0x60
#define	ANSWER_ERROR					0x40
#define SEARIAL_BUFEER_SIZE 100

#define STX_FLAG       		0x02         /* start of text */
#define	ETX_FLAG			0x03         /* end of text */

//------ enum status Defime----------------------
enum emcom_parity { MYPARITY_ODD = 0, MYPARITY_EVEN = 1, MYPARITY_NONE = 2 };
enum emcom_bytesize { COM_BYTESIZE5 = 5, COM_BYTESIZE6 = 6, COM_BYTESIZE7 = 7, COM_BYTESIZE8 = 8};
enum emcom_framestate { INVAILD_FRAME = 0, ACK_FRAME, ANSWER_OK_FRAME, ANSWER_INVAILD_ADDR_FRAME, ANSWER_ERROR_FRAME, DEVICE_BUSY_FRAME};



extern pthread_mutex_t armlock_ack; //主臂ack同步锁
pthread_cond_t armcondtion_ack;  //主臂ack条件变量
pthread_mutex_t armlock_answer;  //主臂answer同步锁
pthread_cond_t armcondtion_answer;

extern int port_arm, port_pump, port_scanner, cabin_port, mix_port;
extern volatile unsigned int ArrayDat;
extern unsigned char serail_recvbuf[SEARIAL_BUFEER_SIZE]; 
extern volatile unsigned int serail_dataarr;
extern volatile bool flg_getedserailarr;
extern volatile unsigned char flg_armack;
extern volatile unsigned char sr_armansercode;
extern bool flg_startrecvpump;
extern bool flg_recvpump;


// #define port_arm port_arm
// #define port_pump port_pump	
// #define port_scanner port_scanner

extern int serail_hand[10];

extern int sb_serailcreat(emb_comid com, int baud, enum emcom_bytesize bytes, enum emcom_parity parity);
extern int sb_frameconver2rsp9000(char* cmdStr, int cmdStrLength, unsigned char DeviceAddr, unsigned char seq, unsigned char isCmdRepeat, unsigned char *WriteBuffer);
extern int sb_parseframe(FRAME_TYPE tecanOEMFrameType, int recvdatlen);
extern int sb_armpumpsend(emb_comid port, uint8_t *cmdbuf, uint8_t devaddr, uint8_t seq, uint8_t flgreport, uint8_t connmod);
extern int sb_waitingframeaswer(sr_cmdstruct_t * CommandElemt);
extern int sb_waitingframeaswer(sr_cmdstruct_t * CommandElemt);
extern void sb_serialclose(uint8_t comnum);
extern int sb_findframeheadtailflg(unsigned char *FrameBuff, int *FrameLen, int ReadLen);



#endif




