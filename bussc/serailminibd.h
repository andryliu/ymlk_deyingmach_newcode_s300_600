



#ifndef __SERAILMINIBD_H__
#define __SERAILMINIBD_H__


#include "port.h"
#include "temper_control.h"
#include "serailminibd.h"
#include "sttype.h"




#define MINI_ACK 0X40

typedef enum 
{
	NOCMD=0,
	INIT_MiniCmd,
	FAN_WORK,
	MINI_CONFIG,
	DO_DISPENSE,
	STRETCH,
	DO_ARM,
	STRETCH_TEST,
	ARM_TEST,
	RELOAD,
	WORK_FINISH,
	PROBE_WORK	,
	SHUTDOWN,
	SETPLATE1,
	PUMP_TEST,
	PERFUSION,
	MAINTAIN_FAN_WORK,
	
}em_minicmd;   //     


typedef enum{	
    NONE_ERROR = 0,
	
    SHELF_UNLOCKED_A = 0XD1,//inform the control board
    SHELF_UNLOCKED_B = 0XD2,
    SHELF_UNLOCKED_C = 0XD3,
    SHELF_LOCKED_A = 0XD4,
    SHELF_LOCKED_B = 0XD5,
    SHELF_LOCKED_C = 0XD7,

    SHELF_LOSE_A = 0XF1,//error code
    SHELF_LOSE_B = 0XF2,
    SHELF_LOSE_C = 0XF3,
    SHELF_UNLOAD_A = 0XF4,
    SHELF_UNLOAD_B = 0XF5,
    SHELF_UNLOAD_C = 0XF6,	
    ERROR_CMD = 0XFF
	
}emminicmd_report;

typedef struct{
	em_minicmd cmd;					//ÃüÁî×Ö
	uint8_t minicmd_buffer[20];	//·¢ËÍ»º´æ
	uint8_t minicmd_num;			//·¢ËÍ³¤¶È
		
}stminibd_sendpacket;

typedef struct stminibd_cmdlist
{
	stminibd_sendpacket cmd;
	struct stminibd_cmdlist* next;
}stminibd_cmdlist;






extern volatile bool mini_finished;
extern volatile bool mini_work_finished[3];
extern volatile unsigned char   mini_recieve_codeACK;
extern volatile char shelf_stirmode[3];
extern unsigned int liquid_valA, liquid_valB, liquid_valC;
extern volatile unsigned char   mini_recieve_code_all[3];
extern unsigned int load_motor[5], load_motor2[5], load_motor3[5];


void sm_parseminibdframe(emb_comid comPort, int ReadLength);
stminibd_sendpacket  sm_minibdrecvdata(stminibd_cmdlist** pcmd_head);
void sm_sendminibdframe (emb_comid comPort, char * frame, uint8_t send_num);
void sm_sendpacket(emb_comid comPort, unsigned int reg_addr, unsigned int send_num, char * data);
void set_minicmd(volatile stminibd_cmdlist* pcmd, stminibd_sendpacket cmd);
void sm_thread_miniarm(void *arg);
bool sm_minisendonecmd(stTemper_cmdframeunion* te_cmd,unsigned char plate_num, unsigned char* recieve_tem);
bool sm_miniwaitinganswer(emb_comid comPort,unsigned char* mini_recieve_code, stminibd_sendpacket cmd);
int sm_serailsenddat(emb_comid comm, sr_cmdstruct_t *sendbuf, unsigned char sq, unsigned char flgresend, unsigned char commod);
void mb_setstatetoshelf(char num, bool IsSc);



#endif


