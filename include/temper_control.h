/*
 * temper_control.h
 *
 *  Created on: 2013-9-6
 *      Author: Administrator
 */

#include "port.h"
#include "temperctlmy.h"


#ifndef TEMPER_CONTROL_H_
#define TEMPER_CONTROL_H_

#define RUN 0
#define STOP 1
#define TEM_NUM 30
typedef enum{
/****************************control operating group**************************/
	CH1_RUN_STOP = 0, CH1_Multi_SV_No,CH1_SV,CH2_RUN_STOP, CH2_Multi_SV_No,CH2_SV,
	CH3_RUN_STOP, CH3_Multi_SV_No,CH3_SV,CH4_RUN_STOP, CH4_Multi_SV_No,CH4_SV,//12

	CH1_AUTO_Tuning,CH1_HeatingPro_Band,CH1_HeatingInt_Time,CH1_HeatingDer_Time,
	CH1__MV_Low,CH1_MV_High,CH1_Ramp_Up,CH1_Ramp_Down,CH1_Ramp_Unit,//21
	CH2_AUTO_Tuning,CH2_HeatingPro_Band,CH2_HeatingInt_Time,CH2_HeatingDer_Time,
	CH2__MV_Low,CH2_MV_High,CH2_Ramp_Up,CH2_Ramp_Down,CH2_Ramp_Unit,//30
	CH3_AUTO_Tuning,CH3_HeatingPro_Band,CH3_HeatingInt_Time,CH3_HeatingDer_Time,
	CH3__MV_Low,CH3_MV_High,CH3_Ramp_Up,CH3_Ramp_Down,CH3_Ramp_Unit,//38
	CH4_AUTO_Tuning,CH4_HeatingPro_Band,CH4_HeatingInt_Time,CH4_HeatingDer_Time,
	CH4__MV_Low,CH4_MV_High,CH4_Ramp_Up,CH4_Ramp_Down,CH4_Ramp_Unit,//48
	/****************************read input register**************************/
	CH1_Present_Value,CH2_Present_Value,CH3_Present_Value,CH4_Present_Value,

	/***************************Initial Setting*******************************/
	CH1_Input_Type,CH1_Input_Bias,CH1_Input_Filter,CH1_SV_Low,CH1_SV_High,
	CH1_Tuning_Type,CH1_Heatingctl_Time,
	CH2_Input_Type,CH2_Input_Bias,CH2_Input_Filter,CH2_SV_Low,CH2_SV_High,
	CH2_Tuning_Type,CH2_Heatingctl_Time,
	CH3_Input_Type,CH3_Input_Bias,CH3_Input_Filter,CH3_SV_Low,CH3_SV_High,
	CH3_Tuning_Type,CH3_Heatingctl_Time,
	CH4_Input_Type,CH4_Input_Bias,CH4_Input_Filter,CH4_SV_Low,CH4_SV_High,
	CH4_Tuning_Type,CH4_Heatingctl_Time,

	/*********LOCK UNLOCK***********/
	LOCK, UNLOCK,
	NOTEMCMD
}emTemp_chopcmd;

typedef enum{
	SV=0,
	MV_Low,
	MV_High,
	Present_Value,
	Input_Type,
	SV_Low,
	SV_High
	
}TEMCMDTYPE;

typedef enum{
	NOTEMPERCMD = 0,
	INIT_TEMPER,
	START_CONTROL1,
	START_CONTROL2,
	START_CONTROL3,
	STOP_CONTROL1,
	STOP_CONTROL2,
	STOP_CONTROL3,
	CONFIG_TEMPER
}emTemper_cmd;

typedef struct {
	unsigned char func;
	unsigned char reg_addrhi;
	unsigned char reg_addrlo;

}stTemper_framedata;

typedef struct {
	char addr;//前一个是地址需要保留位置
	stTemper_framedata cmd;
	unsigned char valuehi;
	unsigned char valuelo;
}stTemper_frame;

typedef struct
{
	union{
		unsigned int devaddr;		//模块地址
		char plate_num;
	};
	union{
		emTemp_chopcmd chopcmd;
		emTermper_cmdtype cmdmy;
	};
	union{
		int temvalue;
		short wdata;
	};
}stTemper_cmdframeunion;

typedef struct tem_cmd_list	//maintian
{
	stTemper_cmdframeunion cmdframeunion;
	struct tem_cmd_list* next;
}tem_cmd_list;

 typedef struct temper_cmd_list //work
{
	emTemper_cmd temcmd;
	struct temper_cmd_list* next;
}temper_cmd_list;

extern volatile temper_cmd_list* ptemcmd_head;
extern unsigned int temp_goalval[];

extern  int temp_Dvalmy[][105];
extern	int temp_setval[][105];

	
void tc_packettempcmd(emTemper_cmd cmd);
emTemper_cmd  tc_readframtemp(void);
void tc_testpacketcmdtemp(stTemper_cmdframeunion cmd);
stTemper_cmdframeunion  tc_testreadtcmdtemp(void);

int tc_gettemtempvalprintf(char sel);

bool tc_temsendcmd(unsigned char dev_addr, emTemp_chopcmd cmd, unsigned int value);
bool tc_recivtempframe(unsigned char *pdata,stTemper_cmdframeunion *cmd);
bool tc_sendcmdshelf(const unsigned int cmd_value, emTemp_chopcmd cmd_type, int* temvalue, char shelf_num);
bool tc_sendcmdchanl(const unsigned int cmd_value, emTemp_chopcmd cmd_type, int* temvalue);
int tc_getfulltempval( int* temvalue);
int tc_getcmtempval(unsigned char plate_num, int* temvalue);
void tc_gettemperaddr(stTemper_cmdframeunion * cmd, unsigned char plate_num);




#endif /* TEMPER_CONTROL_H_ */


