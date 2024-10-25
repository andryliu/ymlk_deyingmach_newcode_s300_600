
#include "port.h"

#ifndef TEMPERCTLMY_H_
#define TEMPERCTLMY_H_


#define TEMPER_WR  1
#define TEMPER_RD  0


typedef enum {
	SET=0,       //  设置温度
	CUR1,        //  当前1温度
	CUR2,        //  当前2温度
	BIAS,        //  
	KP,
	TI,
	TD,
	Tsam,
	
	RUN_STOP
}emTermper_cmdtype;
	

typedef struct {
	char devaddr;//
	char chanel;
	char temcmd;
	char r_w;//0=r
	unsigned short temvalue;
		
}stTemper_cmdframe;


extern int tp_sendtemperframe(char chanel, emTermper_cmdtype cmd, char r_w, short wdata, short *rdata);
extern int tp_revicetemperframe(unsigned char *pdata,stTemper_cmdframe *temcmd);


#endif



