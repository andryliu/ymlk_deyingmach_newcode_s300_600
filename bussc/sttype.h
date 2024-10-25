


#ifndef __STTYPE_H__
#define __STTYPE_H__

#include "pthread.h"
#include "config.h"
#include "port.h"



#define REAGENT_CASE 37
#define MAINTIAN_MAIN_INCREMENT 2
#define MAINTIAN_SELF_INCREMENT 2

#pragma pack(1)

typedef struct stmb_pwmconf
{
	unsigned int		cmd;			// = 0, 1, 2, ....
	unsigned int		freq; 			/* in Hz */
	unsigned int		duty;			/* in % */
	unsigned int		polarity;
}stmb_pwmconf;

typedef enum {
	BIG_COM1 = 1,
	BIG_COM2 = 2, 
	BIG_COM3 = 3, 
	BIG_COM4 = 4, 
	BIG_COM5 = 5, 
	BIG_COM6 = 6, 
	BIG_COM7 = 7,
	BIG_COM8 = 8,
	BIG_COM9 = 9
}emb_bcomid;
	

typedef enum {  
	COM2 = 1, 
	COM3 = 2, 
	COM4 = 3, 
	COM5 = 4, 
	COM6 = 5, 
	COM7 = 6,
	COM8 = 7,
	COM9 = 8
}emb_comid;




typedef enum  { 
	RSP9000_FRAME = 0, 
	XLP6000_FRAME = 1
} FRAME_TYPE;


typedef enum{
	WASH4 = 0, WASH3,WASH2,WASH1

}COORDINATE;

typedef enum {
	NO_EVENT = 0,
	INIT,	//initial the instrument
	MAINTIAN,	//when the MAINTIAN has been recieved the instrument truns to the state of maintianwork
	WORKA,	//when shelf A is going to recieve the operation WORKA should be sented first 
	WORKB,
	WORKC,
	REAGNET_INFO,//if there are some particular reagents was scaned ,PC should send SPECIAL_REAGNET first
	CHECK_REAGENT,//CHECK_REAGENT is sented when someone click the button of check cmd on the PC
	CHECK_PROBLEM,
	WORK_NORMAL,//���������ͳ�ʼ��������PC�˼��û��������͸�����
	PAUSE_NOW,//10
	STOP_NOW,
	STANDBY,
	PROBE_CLR,
	DELAY_START,//0,1,2	
	RUNINGCONFIG,//������ͣ�������
	UpdateSlave,
	UpdateIP,
	ChangeReagentA,
	ChangeReagentB,
	ChangeReagentC,//20
	ChangeReagent2,
	
	/******����Ϊ���巵��PC��********/
	INIT_START,
	INIT_END,
	SHELF_LOCKED,//�Լ���������Ϣ
	SHELF_UNLOCKED,//�Լ��ܽ�����Ϣ
	WORK_STEP,//������Ϣ
	WORK_PECENT_PROBR_CLR,
	WORK_TEMP,//�¶���Ϣ
	CASE_STATE,//�������Լ���Ϣ
	ERROR_WARNING,//����ͱ�����Ϣ 30
	SCANNER_SEND,//ɨ�貣Ƭ����ͷ
	REAGENT_SEND,//�����Լ����Լ�����ͷ
	REAGENTCODE_SEND,//ɨ���Լ��뱨��ͷ
	SHELF_SCAN_END,//��Ƭɨ����� 26
	REAGENT_SCAN_END,//�Լ���������
	SET_REAGENT_DISABLE,//��ĳ���Լ�������͵��Լ�������Ϣ
	REAGENT_REMOVED,//�Լ����Լ����Ƴ�
	HYDRATE_WORK,
	HEART_BEAT,
	SCANNER_IMAGE_SEND,//ɨ�貣Ƭͼ����ͷ 40
	REPLACE_REAGENT,//��PC���͵ļ����Լ���Ϣ
	SHELF_SCAN__START, //34
	REAGENT_SCAN_START,
	WORK_START,
	PARAMETER,
	MIXSTATION_NEED_CLEAR,//�����Ƿ���Ҫ��ϴ������ǿ�ƹػ���ֹͣ ��������վû������ϴ
	REAGENT_SEND_INWORK,//�������й����С������Լ�����Ϣ
	ESTIMATE_TIME,
	UpdateSlaveState,
	AddBatchErrorLog,//50
	SlideCoverOCRCode,
	InstrumentSleep
}Eenetevent;

typedef enum {
	FREE_WORK,
	BUSY_WORK,
	ASPIRATE_WORK,
	DISPENSE_WORK,
	INIT_WORK,
	STOP_WORK,
	STOP_ALL,
	PAUSE,
	MAINTAIN_WORK,
	STANDBY_WORK,
	PROBE_CLR_WORK
}ewkevent;


typedef enum {
	MAIN_ARM = 0,
	SELF_ARM,
	STRECH,
	PUMP,
	OTHER,
	TEMP
}emianid;

typedef enum {
	SHELF = 0,		// + 0~29 / 30~59 /60~89
	REAGENT_STAION,		// + 0~35
	MIX_STATION,		// + 0~5
	CLEAR_STATION,	// + 0~3
	MAIN_X,			//  + 0/1 	0��ʾ������1��ʾ���� 
	MAIN_Y,			//  + 0/1 	0��ʾ������1��ʾ����
	MAIN_Z,			//  + 0/1 	0��ʾ������1��ʾ����
	RELEASE,			//�ͷ�Z��
	MAIN_INIT,		//��ʼ��
	INIT_SET,
	MIAN_SAVE,       //  10  ��λ��У׼ҳ�����½Ǳ��水ť
	SCANER,
	SHELF_CHECK,  // + 0/1 (������ȷ����е�����)
	SHELF_SCAN,
	Z_DOWN,
}ejobid;


typedef enum {
	SELFA = 0, //  0��ʼ����1��ʼλ�ã�2 150ulλ�ã�3 100ulλ��
	SELFB,		// 0��ʼ����1��ʼλ�ã�2 150ulλ�ã�3 100ulλ��
	SELFC,		//  0��ʼ����1��ʼλ�ã�2 150ulλ�ã�3 100ulλ��
	SELFAY,		// + 0/1 0��ʾ������1��ʾ���� 
	SELFBY,		// + 0/1 0��ʾ������1��ʾ���� 
	SELFCY,		// + 0/1 0��ʾ������1��ʾ���� 
	SELFA_INIT,	//��ʼ��A
	SELFB_INIT,	//��ʼ��B
	SELFC_INIT,	//��ʼ��C
	SELFA_UPDOWN,	// + 0/1  0��ʾdown
	SELFB_UPDOWN,
	SELFC_UPDOWN,
	SELF_SAVE
}eselfact;

//����Լ��� �����Լ��� ��ɨ���Լ���֪ͨ
typedef struct stshelfstaut{
	int sen;
	int lock;
	volatile bool NEED_SCAN;
	volatile bool NEED_MIX;
	volatile bool NEED_CHECK;
	volatile bool STATE;//0�ޣ�1��
}stshelfstaut;


typedef enum{
	STRECHA = 0,	// + 0/1/2/3  0��ʾ��λλ�ã�1��ʾ��ʼ��λ�ã�2��ʾ����һ��λ�ã�3��ʾ����ȫ��λ��
	STRECHB,
	STRECHC
}estrechid;

typedef enum emixcmd{
	MIXACK = 0X40,
	MIXRESP = 0X51,
	MIXEOR = 0X41,
	MIXSTATE = 0X20,	
	SENSTATE = 0X21,
	SCANSTATE= 0X22,

	MIXDATA1 = 0X30,
	MIXDATA2 = 0X31,
	MIXDATA3 = 0X32,
	MIXDATA4 = 0X33,
}emixcmd;

typedef enum MIX_EOR{
	MIX_EMP = 0,
	MIX_DIRDY,
	MIX_LIGHT_EOR,
	MIX_RFID_EOR,
}MIX_EOR;

typedef enum{
	PUMPA = 0,	// 0~1000UL(����ֵ)
	PUMPB,
	PUMPC,
	PUMPD,
	PUMP_INITA,//��ʼ��
	PUMP_INITB,
	PUMP_INITC,
	PUMP_INITD
}PUMP_T;


typedef enum{
	MAINTIAN_V1 = 0,MAINTIAN_V2,MAINTIAN_V3,MAINTIAN_V4,MAINTIAN_V5,MAINTIAN_V6,
	MAINTIAN_V7,MAINTIAN_V8,MAINTIAN_V9,MAINTIAN_V10,
	MAINTIAN_V20,MAINTIAN_V21,MAINTIAN_V22,
	MAINTIAN_VP1,
	MAINTIAN_P10,
	MAINTIAN_P11,
	MAINTIAN_FANA,
	MAINTIAN_FANB,
	MAINTIAN_FANC,
	MAINTIAN_BEEP,
	MAINTAIN_SETZERO,
	MAINTAIN_SETFULL,
}OTHER_T;


typedef enum{	
	REAGENT_WATER = 37,
	REAGENT_WASH,
	REAGENT_DEWAX,
	REAGENT_ALCOHOL,//40
	REAGENT_ER1,
	REAGENT_ER2,
	REAGENT_DAB_NULL,	//��Ϻú��DAB ��ʱ��Ч
	NO_REAGENT,	//�濾
	STOP_OPERATE,
	START_OPERATE,
	FINAL_STOP,
	REAGENT_DAB,	//��Ϻú��DAB 
	REAGENT_SECEND,// ˫Ⱦͬһ����
 
    /*	REAGENT_GREEN1,
        REAGENT_GREEN2,
        REAGENT_GREEN3,
        REAGENT_GREEN4,
        REAGENT_FR1,
        REAGENT_FR2,
        REAGENT_FR3,
        REAGENT_FR4
    */	
}emREAGENT;


typedef enum {					//���󣬾�����Ϣ	//�˴ο����з����Ĵ����������˷��� ��Ӧ����������ֽ���1
	NO_ERROR= 0,
	DOOR_OPEN,
	SHELF_UNLOAD_MINIA,//������Ա����A��װ�ذ�ť���ǲ�Ƭû��װ��λ,��������װ�벣Ƭ��A
	SHELF_UNLOAD_MINIB,//						,��������װ�벣Ƭ��B
	SHELF_UNLOAD_MINIC,	//					,��������װ�벣Ƭ��C
	
	MIX_STATION_NOTCLEAR,//��ʼ����⵽���վû�б���ϴ,�������������վ�Ƿ�ɾ�
	MIX_STATION_CLEARERROR,//���վ��ϴʧ��,�������������վ�Ƿ��в���Һ

	ATEMPERL,	//�����¶���Ϣ (��ʱû��)
	ATEMPERH,
	BTEMPERL,
	BTEMPERH,//10
	CTEMPERL,
	CTEMPERH,//12
	SHELF1_ABANDON,// A�� Ѱ������Լ�ʱ�����滻��WASH ��������
	SHELF2_ABANDON,
	SHELF3_ABANDON,
	MIXA_DILUENT_WRONG,//A�� ���ʱ��ȡϡ��Һ����
	MIXB_DILUENT_WRONG,
	MIXC_DILUENT_WRONG,
	MIXA_DAB_WRONG,// A�� ���ʱ��ȡŨ��Һ����
	MIXB_DAB_WRONG,//20
	MIXC_DAB_WRONG,
	ASPIRATE_WRONG,//��������ȡ�Լ����Լ�����,û�м�⵽Һ��
	
	/***********************����Ϊ������Ϣ���û���Ԥ�ų����������*************************/
	MIX_STATION_MOVED,//��ʼ��ʱ��⵽���վ���Ƴ�,��������װ����վ
	WATERPOUR_WRONG,//���� ˮ ��עʱʧ��,������������װ��DIWATER�Լ�ƿ
	WASHPOUR_WRONG,//���� ����Һ ��עʱʧ��,������������װ��WASHBUFFER�Լ�ƿ
	DEWAXPOUR_WRONG,//���� ������ ��עʱʧ��,������������װ��DEWAX�Լ�ƿ
	ALCOHOLPOUR_WRONG,//���� �ƾ� ��עʱʧ��,������������װ��ALCOHOL�Լ�ƿ
	ER1POUR_WRONG,////���� ER1 ��עʱʧ��,������������װ��ER1�Լ�ƿ
	ER2POUR_WRONG,//���� ER2 ��עʱʧ��,������������װ��ER2�Լ�ƿ
	
	WAST_LIUID_LOW_FULL,//�ⲿ��Ũ�ȷ�ҺͰ��,������������ⲿ��Ũ�ȷ�ҺͰ//30
	WAST_LIUID_HIGH_FULL,//�ⲿ��Ũ�ȷ�ҺͰ��,������������ⲿ��Ũ�ȷ�ҺͰ
	/************************************************************************************/
	
	
	/***********************���´������������ϵ����,����������ϵ�ͷ���Ա*************************/
	WAST_LIUID_LOW,//�ŵ�Ũ�ȷ�Һ����
	WAST_LIUID_HIGH,//�Ÿ�Ũ�ȷ�Һ����
	
	ASPIRATE_WRONG_NOT_ENOUGH,//��������ȡ�Լ����Լ�����,�Լ�������
	LargeBottle_outdoor,
	WAST_LIUID_WAST,//�ŷ�Һ�ռ�ƿ����
	CONNECT_ERROR_TEMPERA,
	CONNECT_ERROR_TEMPERB,
	CONNECT_ERROR_TEMPERC,
	CONNECT_ERROR_XIUMIAN,
	ARM_INVALID_COMMAND=0X62,//�����������	
	PUMP_INVALID_OPERAND=0X63,//ע���D�������
	ARM_INVALID_OPERAND=0X64,//�����������
	CONECT_ERROR_MIAN_PUMP = 0X65,//����û�г�ʼ��
	ARM_DEVICE_NOT_INITIALIZED=0X67,//ע���Dû�г�ʼ��
	ARM_DEVICE_COMMEND_OVERFLOW=0X68,//�������г�����Χ
	PUMP_PLUNGER_OVERLOAD = 0X69,//ע���Dע������������
	PUMP_VALVE_OVERLOAD = 0X6A,//ע���Dת������������
	ARM_COLLISION_AVOIDED=0X71,
	Z_INIT_FAILED = 0X73,
	ARM_STEP_LOSS_X=0X74,//����X�����б���
	ARM_STEP_LOSS_Y=0X75,//����Y�����б���
	ARM_STEP_LOSS_Z=0X76,//����Z�����б���
	

	CONNECT_ERROR_TEMPER,//�¿���ͨѶʧ��
	CONNECT_ERROR_TEMPRO,//�¿������߰�ͨѶʧ��
	CONNECT_ERROR_MINIA,//�Կر�AͨѶʧ��
	CONNECT_ERROR_MINIB,//�Կر�BͨѶʧ��
	CONNECT_ERROR_MINIC,//�Կر�CͨѶʧ��
	CONNECT_ERROR_PUMPA,//ע���DͨѶʧ��
	CONNECT_ERROR_PUMPB,//ע���BͨѶʧ��
	CONNECT_ERROR_PUMPC,//ע���CͨѶʧ��
	CONNECT_ERROR_WATER,//ˮ����ư�ͨѶʧ��
	CONNECT_ERROR_WEIGHT,//����������ͨѶʧ��
	CONNECT_ERROR_SCANNER,//ɨ��ͷͨѶʧ��
	CONNECT_ERROR_ARM,//����ͨѶʧ��
	ARM_LOSE_MINIA,//�Կر�A���г���
	ARM_LOSE_MINIB,//�Կر�B���г���
	ARM_LOSE_MINIC,//�Կر�C���г���
	SHELF_LOSE_MINIA,//װ�ص��A���г���
	SHELF_LOSE_MINIB,//װ�ص��B���г���
	SHELF_LOSE_MINIC,//װ�ص��C���г���
	PROBE_LOSS_MINIA,//�Կر�A̽�����г���
	PROBE_LOSS_MINIB,//�Կر�B̽�����г���
	PROBE_LOSS_MINIC,//�Կر�C̽�����г���
	VACUUM_ERROR,//δ�ܴﵽ���Ҫ��
	COMPRESSOR_ERROR,//δ�ܴﵽѹ��Ҫ��
	POWER_ERROR,//��Դ����
	
	CONNECT_ERROR_ARM_CABINA = 700,//�Կرۿ���AͨѶʧ��
	CONNECT_ERROR_ARM_CABINB,//�Կرۿ���BͨѶʧ��
	CONNECT_ERROR_ARM_CABINC,//�Կرۿ���CͨѶʧ��
	CONNECT_ERROR_MIX,//���վͬѶʧ��

	//A�ܳ���
	SHELFA_ERROR_ARMLOS=750,//A�Կر�ʧ��
	SHELFA_ERROR_PUMP_CNT,//A�Կر�ʧ��
	SHELFA_ERROR_SYS_LOS,//A�Կر�ע��������
	SHELFA_ERROR_VALVE_LOS,//A�Կر�ת��������
	SHELFA_ERROR_LOAD_LOS,//A��װ�ص������
	SHELFA_ERROR_INVALIDE_OPERAND = 770,//A�Կ��������
	SHELFA_ERROR_PUMP_INVALID_CMD,//A�Կر��������
	SHELFA_ERROR_PUMP_NOT_INITIAL,//A�Կر�û�г�ʼ��
	SHELFA_ERROR_PUMP_CMD_OVERFLOWED,//A�Կر��������Χ
	SHELFA_ERROR_PUMP_BUSY,//A�Կر�æ
	SHELFA_ERROR_NOT_LOAD,//������Ա����A��װ�ذ�ť���ǲ�Ƭû��װ��λ,��������װ�벣Ƭ��A

	SHELFB_ERROR_ARMLOS=800,
	SHELFB_ERROR_PUMP_CNT,
	SHELFB_ERROR_SYS_LOS,
	SHELFB_ERROR_VALVE_LOS,
	SHELFB_ERROR_LOAD_LOS,
	SHELFB_ERROR_INVALIDE_OPERAND = 820,
	SHELFB_ERROR_PUMP_INVALID_CMD,
	SHELFB_ERROR_PUMP_NOT_INITIAL,
	SHELFB_ERROR_PUMP_CMD_OVERFLOWED,
	SHELFB_ERROR_PUMP_BUSY,
	SHELFB_ERROR_NOT_LOAD,

	SHELFC_ERROR_ARMLOS=850,
	SHELFC_ERROR_PUMP_CNT,
	SHELFC_ERROR_SYS_LOS,
	SHELFC_ERROR_VALVE_LOS,
	SHELFC_ERROR_LOAD_LOS,
	SHELFC_ERROR_INVALIDE_OPERAND = 870,
	SHELFC_ERROR_PUMP_INVALID_CMD,
	SHELFC_ERROR_PUMP_NOT_INITIAL,
	SHELFC_ERROR_PUMP_CMD_OVERFLOWED,
	SHELFC_ERROR_PUMP_BUSY,
	SHELFC_ERROR_NOT_LOAD,
	/********************************************************************/

	/****************************����Ϊ�¶ȿ�����Ӧ��λ�ò�������,����������ϵ�ͷ���Ա****************************************/
	TEMPER1=900,TEMPER2,TEMPER3,TEMPER4,TEMPER5,TEMPER6,TEMPER7,TEMPER8,TEMPER9,TEMPER10,	//30�����ȿ����
	TEMPER11,TEMPER12,TEMPER13,TEMPER14,TEMPER15,TEMPER16,TEMPER17,TEMPER18,TEMPER19,TEMPER20,
	TEMPER21,TEMPER22,TEMPER23,TEMPER24,TEMPER25,TEMPER26,TEMPER27,TEMPER28,TEMPER29,TEMPER30,
	/********************************************************************/
	
	/****************************����Ϊ���ȿ鲻�ܿ��Ƴ���,����������ϵ�ͷ���Ա****************************************/
	TEMPER1_EM,TEMPER2_EM,TEMPER3_EM,TEMPER4_EM,TEMPER5_EM,TEMPER6_EM,TEMPER7_EM,TEMPER8_EM,TEMPER9_EM,TEMPER10_EM,//1��10�������κ�һ������A�����ܲ�Ƭ��������
	TEMPER11_EM,TEMPER12_EM,TEMPER13_EM,TEMPER14_EM,TEMPER15_EM,TEMPER16_EM,TEMPER17_EM,TEMPER18_EM,TEMPER19_EM,TEMPER20_EM,//11��20�������κ�һ������B�����ܲ�Ƭ��������
	TEMPER21_EM,TEMPER22_EM,TEMPER23_EM,TEMPER24_EM,TEMPER25_EM,TEMPER26_EM,TEMPER27_EM,TEMPER28_EM,TEMPER29_EM,TEMPER30_EM//21��30�������κ�һ������C�����ܲ�Ƭ��������
}emERRORCODE;



typedef struct stbeep_state{
	char door;	
	char cabin;
	char error;	
}stbeep_state;

typedef struct stnet_report{
	unsigned char num;	//�Լ���Ϣ37��ʼ
	unsigned char value;
}stnet_report;

typedef struct stnet_reportlist{
	stnet_report report;
	struct stnet_reportlist* next;
}stnet_reportlist;

typedef struct sterrcode_list{
	int error_code;
	struct sterrcode_list* next;
}sterrcode_list;


typedef struct{
	int exchange_reagent_time;
	int reagent_shelf_time;
	int tem_time;
	int max_time;
	int fan_time;
	int slide_cnt;
	int mov_time;
	int discharge_time;
	int all_time;//����5���ӵȴ�ʱ�䲽����
	
}time_cal_t;


typedef enum SPECIAL_REAGENT{
	Antibody = 1,//6ml
	H2O2,//30ml        // ϡ��Һ  
	DAB,//7ml
	RED,        //  ���� DAB��˫Ⱦʱ�п����õ� RED��AP��GREEN��FR ���Լ�����������ʵ��һ�㶼��ʹ�� DAB
	CLEAR,//30ml
	RED1,
	RED2,
	RED3,
	FREE_REAGENT2,//7ml
	Antibody2, //30ml
	GREEN1,//30ml
	GREEN2,//30ml
	FR1,//30ml
	FR2,//30ml
	system7ml,
	FREE_REAGENT3 = 16,//30ml
	EBER6,//Һ��Ƚϳ�//6ml
	EBER7,//7ml
	EBER30,//30ml
	AP1,//7ml
	AP2//30ml
}em_specail_reagentcode;

typedef struct{
	char code[20];		//�����Լ����õ���Ѱ�Ҽ����Լ�ʱ��ͬһ�Լ�ϵͳ ���Լ����õ���Ѱ����λ��
	char reagent_kind[9];	//������������Ѱ������Լ�   // 9 �ֽڵ��ַ������Լ�������������andry
	em_specail_reagentcode special_num;//�����ʶ  1Ϊ���� 2Ϊϡ��Һ(H2O2) 3ΪDAB  4ΪRED 5Ϊ��ϴϵͳ  
	char lot_num[9];	//����
	int val;	//�Լ���
}reagent_t;              //  sizeof(reagnent_t)  = 43 bytes

typedef struct{
	char reagent_bigcode[100];//���ܻ���ֻ��һ������
	bool DataRead;
}reagentdata_t;


typedef struct{
	unsigned char ordArrayA[10];//  Ũ��Һ
	unsigned char ordArrayB[10];// ϡ��Һ
	reagent_t reagentA[10];
	reagent_t reagentB[10];
	char tep_num;
}mix_t;

typedef struct reagentoutside_list{
	reagent_t reagent_info;
	struct reagentoutside_list* next;
}reagentoutside_list;

typedef struct operate_t{			//������Ԫ�ṹ��  
	unsigned char reagent;       // 
	unsigned int time;
	unsigned int temp;
	unsigned char plate_num;	// 0~89  ��Ƭ�μӵľ����ļܣ��ĸ���Ƭλ�� andry
	reagent_t reagent_info;//��������ʱ���� reagent ���Լ�ƽ̨�е���Ϣ���
	struct operate_t* next;
}operate_t;

typedef struct operate_head_list{		//������Ԫͷ�ṹ��
	operate_t operate;
	unsigned int operate_work_time;
	struct operate_head_list* next_head;
}operate_head_list;   //   0x15f = 351

typedef struct{
	unsigned char reagent;
	unsigned char plate_num;
	reagent_t reagent_info;
}dispense_t;



typedef enum sttemstat{
	IN_85 = 0X88,
	IN_85_TIME, 
	SEND_TEMP,
	SENDED_TEMPERCMD,
	REACH_TEMP, 
	WAIT_STRECH,
	START_TIME, 
	START_FAN, 
	END_TIME
}em_status;

typedef struct stfanstruct{
 	bool NORMAL_FAN;
	char time;//s
} stfanstruct;



typedef struct tpool_work{
  void (*handler_routine)();
  void *arg;
  struct tpool_work *next;                  /*��������*/
} sttpwk;

typedef struct tpool{
  int num_threads;
  int max_queue_size;

  int do_not_block_when_full;
  pthread_t *threads;
  int cur_queue_size;
  sttpwk *queue_head;
  sttpwk *queue_tail;
  pthread_mutex_t queue_lock;
  pthread_cond_t queue_not_full;
  pthread_cond_t queue_not_empty;
  pthread_cond_t queue_empty;
  
  int queue_closed;
  unsigned char shutdown;
  //unsigned char suspend;
  //unsigned char resume;
} stthreadpl;


typedef struct temper_control_t
{
	unsigned int time;
	unsigned char plate_num;
	unsigned int temp;// 1Ϊ1�� //�Լ��¿���1Ϊ0.1��
	em_status state;	
}temper_control_t;

typedef struct hydrate_t	//ˮ�ϲ�������
{
	dispense_t hydrate_plate[11]; //ˮ�������ڶ�ȡ����ʱ����
	unsigned int start_time; 
	volatile bool flage;        //ˮ�ϱ�־ �ڽ���ĳ�ܶ���ʱ TRUE 
}hydrate_t;

typedef struct mixstation_clear_state_t
{
	bool AFTER_DISPENSE;  //��ϵ�һ���� ��FALSE  ��ǰ���ƿ�μ���� ��TRUE
	bool NEEDCLEAR;       //��ϵ�һ���� ��TRUE ��ϴ�����FALSE
}mixstation_clear_state_t;

typedef struct parameter_t{
	unsigned int ordArray_wash[3];        //�������ϴվX,Y,Z����
	unsigned int ordArray_reagent[4][3];  // 4���Լ�ƽ̨�ĵ�һ��λ��
	//unsigned int ordArray_mixed_DAB[3];  //���վ�ĵ�һ��λ��
	unsigned int ordArray_plate[9][3];    // 3����Ƭ�ܵĵ�һ��λ�� �� 100ul 150ul
	unsigned int shelf_check[2][3];

	unsigned int ordArray_OW[3];  // ��ʼ��λ������
	unsigned int load_motor1[2]; //�Կر�A��λ װ��
	unsigned int arm_motor1[3];  //��ʼλ�� 150ulλ�ã�100ulλ��
	unsigned int load_motor2[2];  //�Կر�B��λ װ��
	unsigned int arm_motor2[3];  //�Կر�B��һ����Ƭλ�� ��ϴվ
	unsigned int load_motor3[2];  //�Կر�C��λ װ��
	unsigned int arm_motor3[3];  //�Կر�C��һ����Ƭλ�� ��ϴվ
	unsigned int temp_SV_High[30];  //SV����
	unsigned int temp_MV[30];    //MV ���Ƽ������ٷֱ�
	int temp_Input_Bias[60];     //����ֵ
	unsigned int temp_Input_Filter[30];   //���������˲�ֵ
	unsigned int temp_HeatingPro_Band[30];  //��������P
	unsigned int temp_HeatingInt_Time[30];  //���ֿ���I
	unsigned int temp_HeatingDer_Time[30];  //΢�ֿ���D
	unsigned int temp_Heatingctl_Time[30];  //��������T																														
}parameter_t;

//------ struct Defime----------------------
/*
* Emlinix JUN-2-2010: double input parameters can be used in more than one driver
*/
struct double_pars
{
	unsigned int	par1;
	unsigned int	par2;
};

typedef struct sr_tecanoemstruct_t {  
	unsigned char arm_addr;  
	unsigned char device_addr;  
	unsigned char control;  
	unsigned char data[64];
} sr_tecanoemstruct_t;



typedef struct {
	char *str;
	unsigned char device_addr;
}cmdStr_t;

typedef struct {
	char cmdbuf[100];
	unsigned char srdevaddr;
	unsigned int seq;
}sr_cmdstruct_t;





#endif




