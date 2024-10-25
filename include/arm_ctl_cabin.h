

#ifndef _ARM_CTL_CABIN_H
#define _ARM_CTL_CABIN_H




typedef enum  ArmCabinState_t{
	ARM_CABIN_CONNECT_ERROR = 0X20,
	ARM_CABIN_ACK = 0X40,
	ARM_CABIN_EOR = 0X41,
	ARM_CABIN_RESP = 0X51,
	ARM_CABIN_STATE = 0X20,
	WASH_STATE=0X30,
	ARM_CABIN_FREE = 0X60//询问返回的是当前架次的状态，没完成返回正在运行的命令
		
}ArmCabinState_t;

//错误码直接转换不用定义
/*
typedef enum{
	NO_ERROR=0,
	ARM_LOS,
	PUMP_CONNECT_ERROR,
	INVALIDE_OPERAND,
	PUMP_SYS_LOS,
	PUMP_VALVE_LOS,
	PUMP_INVALID_CMD,
	PUMP_NOT_INITIAL,
	PUMP_CMD_OVERFLOWED,
	PUMP_BUSY,
	LOAD_LOS,
	NOT_LOAD,
	
}ARM_ERROR_T;
*/

typedef enum ArmCabinCmd{
	ArmCabinNo = 0,
	ArmCabinINIT = 1,
	ArmCabinWork,
	ArmCabinPar,
	ArmCabinPumpValve,
	ArmCabinPumpSy,
	ArmCabinArm,
	ArmCabinLoadMotor,
	ArmCabinKey,
	ArmCabinUpMotor,
	ArmCabinFANON,
	ArmCabinFANOFF,
	ArmCabinFANXIUMIAN
}ArmCabinCmd;


	/*

typedef struct ArmCabin_t{
	ArmCabinCmd cmd;
	data0_t data0;
	data1_t data1;
	data2_t data2;
	
}ArmCabin_t;
*/
	typedef struct ArmCabin_t{
		ArmCabinCmd cmd;
		union {
	char init_kind;
	char dispense_pos;
	char dir;
	char key_state;
	char up_state;
	};
	union {
		char reagent_sel;
		char pos_datalo;
		};
	union
		{
		char reagent_val;
		char pos_datahi;
		};
	char par4;
	}ArmCabin_t;



extern volatile bool ArmCabinWork_finished[];
extern volatile bool ArmCabinWork_needreport[];
extern ArmCabin_t ArmCabinCmdList[];
extern volatile bool unload[];



int SendCmdArmCtlCabin(char cmd, uint8_t shelf, char* data );


#endif
