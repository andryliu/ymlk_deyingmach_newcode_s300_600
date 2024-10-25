


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
	WORK_NORMAL,//当仪器发送初始化结束后，PC端检测没有问题后发送给仪器
	PAUSE_NOW,//10
	STOP_NOW,
	STANDBY,
	PROBE_CLR,
	DELAY_START,//0,1,2	
	RUNINGCONFIG,//运行暂停相关配置
	UpdateSlave,
	UpdateIP,
	ChangeReagentA,
	ChangeReagentB,
	ChangeReagentC,//20
	ChangeReagent2,
	
	/******以下为主板返回PC的********/
	INIT_START,
	INIT_END,
	SHELF_LOCKED,//试剂架锁定信息
	SHELF_UNLOCKED,//试剂架解锁信息
	WORK_STEP,//步骤信息
	WORK_PECENT_PROBR_CLR,
	WORK_TEMP,//温度信息
	CASE_STATE,//大容量试剂信息
	ERROR_WARNING,//错误和报警信息 30
	SCANNER_SEND,//扫描玻片报文头
	REAGENT_SEND,//返回试剂量试剂报文头
	REAGENTCODE_SEND,//扫描试剂码报文头
	SHELF_SCAN_END,//玻片扫描结束 26
	REAGENT_SCAN_END,//试剂量检测结束
	SET_REAGENT_DISABLE,//当某架试剂抽离后发送的试剂抽离信息
	REAGENT_REMOVED,//试剂架试剂被移出
	HYDRATE_WORK,
	HEART_BEAT,
	SCANNER_IMAGE_SEND,//扫描玻片图像报文头 40
	REPLACE_REAGENT,//向PC发送的兼容试剂信息
	SHELF_SCAN__START, //34
	REAGENT_SCAN_START,
	WORK_START,
	PARAMETER,
	MIXSTATION_NEED_CLEAR,//发送是否需要清洗，避免强制关机或停止 操作后混合站没进行清洗
	REAGENT_SEND_INWORK,//用于运行过程中　返回试剂量信息
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
	MAIN_X,			//  + 0/1 	0表示减量；1表示增量 
	MAIN_Y,			//  + 0/1 	0表示减量；1表示增量
	MAIN_Z,			//  + 0/1 	0表示减量；1表示增量
	RELEASE,			//释放Z轴
	MAIN_INIT,		//初始化
	INIT_SET,
	MIAN_SAVE,       //  10  上位机校准页面右下角保存按钮
	SCANER,
	SHELF_CHECK,  // + 0/1 (两个点确定机械臂误差)
	SHELF_SCAN,
	Z_DOWN,
}ejobid;


typedef enum {
	SELFA = 0, //  0初始化，1初始位置，2 150ul位置，3 100ul位置
	SELFB,		// 0初始化，1初始位置，2 150ul位置，3 100ul位置
	SELFC,		//  0初始化，1初始位置，2 150ul位置，3 100ul位置
	SELFAY,		// + 0/1 0表示减量；1表示增量 
	SELFBY,		// + 0/1 0表示减量；1表示增量 
	SELFCY,		// + 0/1 0表示减量；1表示增量 
	SELFA_INIT,	//初始化A
	SELFB_INIT,	//初始化B
	SELFC_INIT,	//初始化C
	SELFA_UPDOWN,	// + 0/1  0表示down
	SELFB_UPDOWN,
	SELFC_UPDOWN,
	SELF_SAVE
}eselfact;

//检测试剂架 锁定试剂架 和扫描试剂架通知
typedef struct stshelfstaut{
	int sen;
	int lock;
	volatile bool NEED_SCAN;
	volatile bool NEED_MIX;
	volatile bool NEED_CHECK;
	volatile bool STATE;//0无，1有
}stshelfstaut;


typedef enum{
	STRECHA = 0,	// + 0/1/2/3  0表示复位位置；1表示初始化位置；2表示拉伸一半位置；3表示拉伸全部位置
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
	PUMPA = 0,	// 0~1000UL(绝对值)
	PUMPB,
	PUMPC,
	PUMPD,
	PUMP_INITA,//初始化
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
	REAGENT_DAB_NULL,	//混合好后的DAB 暂时无效
	NO_REAGENT,	//烘烤
	STOP_OPERATE,
	START_OPERATE,
	FINAL_STOP,
	REAGENT_DAB,	//混合好后的DAB 
	REAGENT_SECEND,// 双染同一处理
 
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


typedef enum {					//错误，警告信息	//此次开机中发生的错误后来解决了发送 对应错误码最高字节置1
	NO_ERROR= 0,
	DOOR_OPEN,
	SHELF_UNLOAD_MINIA,//操作人员按了A架装载按钮但是玻片没有装到位,处理方法：装入玻片架A
	SHELF_UNLOAD_MINIB,//						,处理方法：装入玻片架B
	SHELF_UNLOAD_MINIC,	//					,处理方法：装入玻片架C
	
	MIX_STATION_NOTCLEAR,//初始化检测到混合站没有被清洗,处理方法：检查混合站是否干净
	MIX_STATION_CLEARERROR,//混合站清洗失败,处理方法：检查混合站是否有残留液

	ATEMPERL,	//加热温度信息 (暂时没用)
	ATEMPERH,
	BTEMPERL,
	BTEMPERH,//10
	CTEMPERL,
	CTEMPERH,//12
	SHELF1_ABANDON,// A架 寻找替代试剂时整架替换成WASH 放弃操作
	SHELF2_ABANDON,
	SHELF3_ABANDON,
	MIXA_DILUENT_WRONG,//A架 混合时吸取稀释液错误
	MIXB_DILUENT_WRONG,
	MIXC_DILUENT_WRONG,
	MIXA_DAB_WRONG,// A架 混合时吸取浓缩液错误
	MIXB_DAB_WRONG,//20
	MIXC_DAB_WRONG,
	ASPIRATE_WRONG,//运行中吸取试剂架试剂出错,没有检测到液体
	
	/***********************以下为警告信息需用户干预排除后才能运行*************************/
	MIX_STATION_MOVED,//初始化时检测到混合站被移出,处理方法：装入混合站
	WATERPOUR_WRONG,//进行 水 灌注时失败,处理方法：重新装载DIWATER试剂瓶
	WASHPOUR_WRONG,//进行 缓冲液 灌注时失败,处理方法：重新装载WASHBUFFER试剂瓶
	DEWAXPOUR_WRONG,//进行 脱蜡剂 灌注时失败,处理方法：重新装载DEWAX试剂瓶
	ALCOHOLPOUR_WRONG,//进行 酒精 灌注时失败,处理方法：重新装载ALCOHOL试剂瓶
	ER1POUR_WRONG,////进行 ER1 灌注时失败,处理方法：重新装载ER1试剂瓶
	ER2POUR_WRONG,//进行 ER2 灌注时失败,处理方法：重新装载ER2试剂瓶
	
	WAST_LIUID_LOW_FULL,//外部低浓度废液桶满,处理方法：清空外部低浓度废液桶//30
	WAST_LIUID_HIGH_FULL,//外部高浓度废液桶满,处理方法：清空外部高浓度废液桶
	/************************************************************************************/
	
	
	/***********************以下错误码需仪器断电操作,处理方法：联系客服人员*************************/
	WAST_LIUID_LOW,//排低浓度废液出错
	WAST_LIUID_HIGH,//排高浓度废液出错
	
	ASPIRATE_WRONG_NOT_ENOUGH,//运行中吸取试剂架试剂出错,试剂量不够
	LargeBottle_outdoor,
	WAST_LIUID_WAST,//排废液收集瓶出错
	CONNECT_ERROR_TEMPERA,
	CONNECT_ERROR_TEMPERB,
	CONNECT_ERROR_TEMPERC,
	CONNECT_ERROR_XIUMIAN,
	ARM_INVALID_COMMAND=0X62,//主臂命令不可用	
	PUMP_INVALID_OPERAND=0X63,//注射泵D命令不可用
	ARM_INVALID_OPERAND=0X64,//主臂命令不可用
	CONECT_ERROR_MIAN_PUMP = 0X65,//主臂没有初始化
	ARM_DEVICE_NOT_INITIALIZED=0X67,//注射泵D没有初始化
	ARM_DEVICE_COMMEND_OVERFLOW=0X68,//主臂运行超出范围
	PUMP_PLUNGER_OVERLOAD = 0X69,//注射泵D注射器运行受阻
	PUMP_VALVE_OVERLOAD = 0X6A,//注射泵D转换阀运行受阻
	ARM_COLLISION_AVOIDED=0X71,
	Z_INIT_FAILED = 0X73,
	ARM_STEP_LOSS_X=0X74,//主臂X轴运行被阻
	ARM_STEP_LOSS_Y=0X75,//主臂Y轴运行被阻
	ARM_STEP_LOSS_Z=0X76,//主臂Z轴运行被阻
	

	CONNECT_ERROR_TEMPER,//温控器通讯失败
	CONNECT_ERROR_TEMPRO,//温控器接线板通讯失败
	CONNECT_ERROR_MINIA,//自控臂A通讯失败
	CONNECT_ERROR_MINIB,//自控臂B通讯失败
	CONNECT_ERROR_MINIC,//自控臂C通讯失败
	CONNECT_ERROR_PUMPA,//注射泵D通讯失败
	CONNECT_ERROR_PUMPB,//注射泵B通讯失败
	CONNECT_ERROR_PUMPC,//注射泵C通讯失败
	CONNECT_ERROR_WATER,//水箱控制板通讯失败
	CONNECT_ERROR_WEIGHT,//大容量检测板通讯失败
	CONNECT_ERROR_SCANNER,//扫描头通讯失败
	CONNECT_ERROR_ARM,//主臂通讯失败
	ARM_LOSE_MINIA,//自控臂A运行出错
	ARM_LOSE_MINIB,//自控臂B运行出错
	ARM_LOSE_MINIC,//自控臂C运行出错
	SHELF_LOSE_MINIA,//装载电机A运行出错
	SHELF_LOSE_MINIB,//装载电机B运行出错
	SHELF_LOSE_MINIC,//装载电机C运行出错
	PROBE_LOSS_MINIA,//自控臂A探针运行出错
	PROBE_LOSS_MINIB,//自控臂B探针运行出错
	PROBE_LOSS_MINIC,//自控臂C探针运行出错
	VACUUM_ERROR,//未能达到真空要求
	COMPRESSOR_ERROR,//未能达到压力要求
	POWER_ERROR,//电源出错
	
	CONNECT_ERROR_ARM_CABINA = 700,//自控臂控制A通讯失败
	CONNECT_ERROR_ARM_CABINB,//自控臂控制B通讯失败
	CONNECT_ERROR_ARM_CABINC,//自控臂控制C通讯失败
	CONNECT_ERROR_MIX,//混合站同讯失败

	//A架出错
	SHELFA_ERROR_ARMLOS=750,//A自控臂失步
	SHELFA_ERROR_PUMP_CNT,//A自控泵失步
	SHELFA_ERROR_SYS_LOS,//A自控泵注射器受阻
	SHELFA_ERROR_VALVE_LOS,//A自控泵转换阀受阻
	SHELFA_ERROR_LOAD_LOS,//A架装载电机受阻
	SHELFA_ERROR_INVALIDE_OPERAND = 770,//A自控命令不可用
	SHELFA_ERROR_PUMP_INVALID_CMD,//A自控泵命令不可用
	SHELFA_ERROR_PUMP_NOT_INITIAL,//A自控泵没有初始化
	SHELFA_ERROR_PUMP_CMD_OVERFLOWED,//A自控泵命令超出范围
	SHELFA_ERROR_PUMP_BUSY,//A自控泵忙
	SHELFA_ERROR_NOT_LOAD,//操作人员按了A架装载按钮但是玻片没有装到位,处理方法：装入玻片架A

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

	/****************************以下为温度块出错对应的位置不能运行,处理方法：联系客服人员****************************************/
	TEMPER1=900,TEMPER2,TEMPER3,TEMPER4,TEMPER5,TEMPER6,TEMPER7,TEMPER8,TEMPER9,TEMPER10,	//30个加热块出错
	TEMPER11,TEMPER12,TEMPER13,TEMPER14,TEMPER15,TEMPER16,TEMPER17,TEMPER18,TEMPER19,TEMPER20,
	TEMPER21,TEMPER22,TEMPER23,TEMPER24,TEMPER25,TEMPER26,TEMPER27,TEMPER28,TEMPER29,TEMPER30,
	/********************************************************************/
	
	/****************************以下为加热块不受控制出错,处理方法：联系客服人员****************************************/
	TEMPER1_EM,TEMPER2_EM,TEMPER3_EM,TEMPER4_EM,TEMPER5_EM,TEMPER6_EM,TEMPER7_EM,TEMPER8_EM,TEMPER9_EM,TEMPER10_EM,//1到10的其中任何一个出错，A架整架玻片不能运行
	TEMPER11_EM,TEMPER12_EM,TEMPER13_EM,TEMPER14_EM,TEMPER15_EM,TEMPER16_EM,TEMPER17_EM,TEMPER18_EM,TEMPER19_EM,TEMPER20_EM,//11到20的其中任何一个出错，B架整架玻片不能运行
	TEMPER21_EM,TEMPER22_EM,TEMPER23_EM,TEMPER24_EM,TEMPER25_EM,TEMPER26_EM,TEMPER27_EM,TEMPER28_EM,TEMPER29_EM,TEMPER30_EM//21到30的其中任何一个出错，C架整架玻片不能运行
}emERRORCODE;



typedef struct stbeep_state{
	char door;	
	char cabin;
	char error;	
}stbeep_state;

typedef struct stnet_report{
	unsigned char num;	//试剂信息37开始
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
	int all_time;//超过5分钟等待时间步骤数
	
}time_cal_t;


typedef enum SPECIAL_REAGENT{
	Antibody = 1,//6ml
	H2O2,//30ml        // 稀释液  
	DAB,//7ml
	RED,        //  类似 DAB，双染时有可能用到 RED，AP，GREEN，FR 等试剂，我们做的实验一般都是使用 DAB
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
	EBER6,//液体比较稠//6ml
	EBER7,//7ml
	EBER30,//30ml
	AP1,//7ml
	AP2//30ml
}em_specail_reagentcode;

typedef struct{
	char code[20];		//用于试剂被拿掉后寻找兼容试剂时找同一试剂系统 或试剂被拿掉后寻找新位置
	char reagent_kind[9];	//种类名字用于寻找替代试剂   // 9 字节的字符串（试剂类型缩略名）andry
	em_specail_reagentcode special_num;//种类标识  1为抗体 2为稀释液(H2O2) 3为DAB  4为RED 5为清洗系统  
	char lot_num[9];	//批号
	int val;	//试剂量
}reagent_t;              //  sizeof(reagnent_t)  = 43 bytes

typedef struct{
	char reagent_bigcode[100];//接受缓存只存一架数据
	bool DataRead;
}reagentdata_t;


typedef struct{
	unsigned char ordArrayA[10];//  浓缩液
	unsigned char ordArrayB[10];// 稀释液
	reagent_t reagentA[10];
	reagent_t reagentB[10];
	char tep_num;
}mix_t;

typedef struct reagentoutside_list{
	reagent_t reagent_info;
	struct reagentoutside_list* next;
}reagentoutside_list;

typedef struct operate_t{			//操作单元结构体  
	unsigned char reagent;       // 
	unsigned int time;
	unsigned int temp;
	unsigned char plate_num;	// 0~89  玻片滴加的具体哪架，哪个玻片位置 andry
	reagent_t reagent_info;//接收流程时根据 reagent 在试剂平台中的信息添加
	struct operate_t* next;
}operate_t;

typedef struct operate_head_list{		//操作单元头结构体
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
  struct tpool_work *next;                  /*任务链表*/
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
	unsigned int temp;// 1为1度 //自己温控器1为0.1°
	em_status state;	
}temper_control_t;

typedef struct hydrate_t	//水合操作链表
{
	dispense_t hydrate_plate[11]; //水合坐标在读取流程时填入
	unsigned int start_time; 
	volatile bool flage;        //水合标志 在结束某架动作时 TRUE 
}hydrate_t;

typedef struct mixstation_clear_state_t
{
	bool AFTER_DISPENSE;  //混合第一步后 置FALSE  当前混合瓶滴加完后 置TRUE
	bool NEEDCLEAR;       //混合第一步后 置TRUE 清洗完后置FALSE
}mixstation_clear_state_t;

typedef struct parameter_t{
	unsigned int ordArray_wash[3];        //最左边清洗站X,Y,Z坐标
	unsigned int ordArray_reagent[4][3];  // 4个试剂平台的第一个位置
	//unsigned int ordArray_mixed_DAB[3];  //混合站的第一个位置
	unsigned int ordArray_plate[9][3];    // 3个玻片架的第一个位置 和 100ul 150ul
	unsigned int shelf_check[2][3];

	unsigned int ordArray_OW[3];  // 初始化位子设置
	unsigned int load_motor1[2]; //自控臂A复位 装载
	unsigned int arm_motor1[3];  //初始位置 150ul位置，100ul位置
	unsigned int load_motor2[2];  //自控臂B复位 装载
	unsigned int arm_motor2[3];  //自控臂B第一个玻片位置 清洗站
	unsigned int load_motor3[2];  //自控臂C复位 装载
	unsigned int arm_motor3[3];  //自控臂C第一个玻片位置 清洗站
	unsigned int temp_SV_High[30];  //SV上限
	unsigned int temp_MV[30];    //MV 控制加热最大百分比
	int temp_Input_Bias[60];     //修正值
	unsigned int temp_Input_Filter[30];   //输入数字滤波值
	unsigned int temp_HeatingPro_Band[30];  //比例控制P
	unsigned int temp_HeatingInt_Time[30];  //积分控制I
	unsigned int temp_HeatingDer_Time[30];  //微分控制D
	unsigned int temp_Heatingctl_Time[30];  //控制周期T																														
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




