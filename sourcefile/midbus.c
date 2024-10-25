 /*
* Copyright(C), 2007-2008, Red Hat Inc.	
* File name: 		funcgpio.c							
* Author:    		Frank Chelle  							
* Version:   		v1.0					
* Date: 			2013.06.14					
* Description:		This provid the gpio control.					
* 						
* History:    							
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
// #include <sys/socket.h>
#include <time.h>
#include <sys/select.h>
#include <sys/wait.h>
// #include <netinet/in.h>
// #include <netdb.h>
// #include <arpa/inet.h>
#include <assert.h>
#include <pthread.h>  
#include <signal.h>
// #include <linux/if.h>
#include <linux/mii.h>
#include <linux/sockios.h>
#include "temperctlmy.h"
 #include "netbuss.h"
#include "config.h"
#include "common.h"
#include "tpthread.h"
#include "midbus.h"
#include "temper_control.h"
#include "serailbussfile.h"
#include "xmodem.h"
#include "serailminibd.h"
#include "xmodem.h"
#include "mbcrc.h"
#include "crc16.h"
#include "port.h"
#include "arm_ctl_cabin.h"
#include "netbuss.h"
#include "buss.h"
#include "readinifile.h"
#include "../caculateProceTime/caculteprocetime.h"
#include "termperbusses.h"
#include "scaner.h"




volatile unsigned int serail_dataarr = 0;        //主臂Z轴下降的高度
unsigned volatile char pump_readbuf[21] = {0};  //读取PUMP

char start_pump_index = 0;

volatile bool flg_lockdischargliquit = false;//防止清洗时试剂瓶拿出后进行灌注时锁死
 
//#define __DEBUG__
//#define _PRT_PAR_

#define ASPIRATE_T 500000
#define ASPIRATE_AIR_T 500000
#define DISPENSE_T 800000
#define ARMSTABLE_T 300000

int MIX_WASH_DISTANCE;

/*  ZWASH_STEP4 : 4 号清洗槽Z坐标  */
int OFFSET_STEP, SHELF_OFFSET, MOV_ZH, ZSPEED_SLOW, ZWASH_STEP1, ZWASH_STEP2_1, ZWASH_STEP2_2, ZWASH_STEP2_ZX;
int ZWASH_STEP3, ZWASH_STEP4, ZWASH_STEP4_SLOW, ZDAB_STEP, LIQUID_ZMAX, DAB_ZMAX, POUR_LIQUID_ZMAX;
int OPEN_ARMZMAX, SCAN_DISTANC_MIXSTATION;
int PULL_DISTANC_INIT, PULL_DISTANC_LT, PULL_DISTANC_HALF, PULL_DISTANC_FULL;

volatile int  new_scaner = 0, new_temper = 0, honey_scaner = 0;
unsigned short reagent_cabin_zero[6];
unsigned short reagent_cabin_full[6];

int big_version = 0, self_icovert = 0, self_pump = 0, new_mixstation = 1, shelf_lock_new = 0, new_outwast_sen = 0;
int new_inwast_sen = 0, new_version=0;
//**方便校准变量**/
int reagent_distance = REAGENT_DISTANCE;
int slide_distance = SLIDE_DISTANCE;
int distance_scan = DISTANCE_SCAN;
int dispense_spd = 0;
stminibd_sendpacket mini_cmd;
int litst = 15;
int conf_zdab_addstep = 50;
/****/



//#include "em9280_drivers.h"

/****************量设置全局变量*******************/
/****************量设置全局变量*******************/
unsigned int zspeed_slow = 0;
unsigned int zwash_step1 = 0;
unsigned int zwash_step2_1 = 0;
unsigned int zwash_step2_2 = 0;
unsigned int zwash_step3 = 0;
unsigned int zwash_step4 = 0; 
unsigned int zDAB_step = 900;
unsigned int zwash_step2_waste = 20;//吸废液步数

unsigned int er2port = 0;
unsigned int er1port = 0;
unsigned int dewaxport = 0;
unsigned int alcoholport = 0;
unsigned int waterport = 0;	
unsigned int washport = 0;
unsigned int probeport = 0;
unsigned int liquid_speed = 0;
unsigned int liquid_step_glb = 2; 
unsigned int liquid_mstep = 0; 
unsigned int liquid_zmax = 1320;//1336;
unsigned int DAB_zmax = 900;
unsigned int pour_liquid_zmax = 0;
unsigned int dispense_stepA[3] = {0};//初始位置，150ul，100ul
unsigned int dispense_stepB[3] = {0};
unsigned int dispense_stepC[3] = {0};
unsigned int open_armstep = 13;//6ml透明管
unsigned int open_armstep2 = 9;//7ml开放试剂瓶
unsigned int open_armzmax = 1326;//16 为7ml试剂瓶  开放试剂插一次性试管，高度不一致
unsigned int wat_sypstep = 1000;
int scanoffset = 0;

//-----------------------------static----------------------------------------------

volatile bool flg_mainproexit = false;//程序终止标志
volatile bool stop_flag = false;//程序暂停标志
volatile stbeep_state beep_state;	//
volatile bool CG1_SCAN = false;//第一架玻片需扫描信号
volatile bool CG2_SCAN = false;//第二架玻片需扫描信号
volatile bool CG3_SCAN = false;//第三架玻片需扫描信号
volatile bool CG4_SCAN = false;//第四架玻片需扫描信号

volatile bool inDischarge = false; //防止吸液于排液储气罐充气动作冲突
volatile bool special_reagent_recieved = false;//试BOOL nt_createsocket( )剂扫描后通知流程继续运行
volatile bool need_check_reagent = false;//试剂扫描后是否需要测试剂量


int GPIO_fd = 0; // NULL;



stnet_reportlist* preport_head = NULL;//report链表头指针
sterrcode_list* perror_head = NULL;//错误链表头指针
sterrcode_list* perror_exist_head = NULL; //错误存在链表头指针

reagentoutside_list * reagentoutside_head = NULL;//移除过的试剂的试剂信息，可与试剂平台试剂信息重复

volatile ewkevent wkevent = STOP_ALL;		//	全局工作状态

volatile bool NEED_PRINTF = USE_PRINT_LOG;
//unsigned char image_data[500000]={0};


volatile unsigned short need_perfusion_last[6] = {0};//WATER WASH DEW ALCHOLE ER1 ER2
volatile unsigned short need_perfusion[6] = {0};//当有拔出装入动作时 ++
static volatile char reagent_flag = NO_REAGENT; //试剂使 用全局变量
volatile char last_cabin_reagent = NO_REAGENT; //最后一次使用的大容量试剂
static volatile bool flg_cabinremovreagt_inport = false; //确保在函数IsReagentInCabinRemoved中灌注是不进行递归调用
int res_shelf = 0;//是否需要整根清洗

volatile unsigned char startmixworkid = 0;	//允许混合或清洗混合瓶操作标志 最高位表示A架 次高位表示B架  最低位表示有空闲时间
volatile unsigned char CriticalWork = 0;	//滴加试剂架上试剂和预处理ER的后一个步骤为critical 最高位表示A架 次高位表示B架
 bool first_dispense = true;//滴加时是否为本次滴加的首次
volatile bool readfinished485 = false;//485数据读取结束标识
volatile bool DOOR_OPENDED = false;	//门打开标识
volatile bool OUT_WASTHIGH_H = false; //外部废液桶
volatile bool OUT_WASTHIGH_L = false; //外部废液桶

volatile bool door_open_action = false;
volatile bool nor_wast_barrel = true;
volatile bool dng_wast_barrel = true;


bool setcabinzero = 0;




volatile bool flg_cabinreved = false;//大容量是否接收完毕标识

extern volatile char isWashWork;//清洗阀控制量
extern volatile char isHighConcentration;//高浓度清洗标识
volatile unsigned char replace_reagent = 0XFF;
volatile bool NeedDischarge = true;//排废液标识

volatile bool TRY_REAGENT_UNLOCK1 = true; //主臂如有吸试剂架试剂操作 等操作结束置TRUE ，md_lockreagentshelf中判断流程中不需锁定则可结锁
volatile bool TRY_REAGENT_UNLOCK2 = true;
volatile bool TRY_REAGENT_UNLOCK3 = true;
volatile bool TRY_REAGENT_UNLOCK4 = true;
volatile unsigned char reagent_lock_num1 = 0; //记录 多少加玻片要锁定 当值为0是说明可以TRY_UNLOCK
volatile unsigned char reagent_lock_num2 = 0; 					//最低位为 A架玻片需要锁状态 当A架滴加完成时将相应位置0 以此类推 
volatile unsigned char reagent_lock_num3 = 0;
volatile unsigned char reagent_lock_num4 = 0;
volatile stshelfstaut reagent_check[4] = {{0}};
volatile char reagent_lock[4][3];// 4个试剂架，每架有3架玻片架使用
volatile  short cabin_value[12] = {0};//	6个short类型容量大小（1~6个容器） +　６个ｓｈｏｒｔ类型传感器状态（１～６个状态）
volatile  unsigned short bigcabin_value[18] = {0};//	6个short类型容量大小（1~6个容器） +　６个ｓｈｏｒｔ类型传感器状态（１～６个状态）+6个容量增量
volatile  unsigned short bigcabin_value_init[18] = {0};
volatile  char bigcabin_ledstate[6] = {0};

unsigned short bigcabin_reset[6] = {32950,32950,32950,32950,32950,32950};

volatile unsigned  char ReadBuffer485[21] = {0};//读取485串口缓存
volatile unsigned  char ReadBufferweight[21] = {0};//读取485串口缓存

volatile unsigned char readindex485 = 0;//读取485串口数据位置值
volatile unsigned char readindexweight = 0;//读取485串口数据位置值
volatile unsigned	char ReadBufferMix[20] = {0};
volatile bool MixACK = false, Mix_Res = false;
volatile bool WGACK = false, WG_Res = false;

unsigned volatile char Readpump[21] = {0};

unsigned char readindexpump = 0;//读取pump串口数据



int CorrectDate[9] = {0};//第0 数值不变
unsigned int ReportArrayData[9] = {0};  //向PC发送Z轴高度缓存
volatile char SHELF_LOCK_STATE[3] = {0};  //置0不触发发送，1解锁状态，2锁定状态
char reagent_clr[3] = {0};
reagent_t reagent_code[36]; // = {{0}}; //试剂架上的试剂
//reagent_t reagent_outside_p;	// 
char flg_cabinhavereagent[6];//标记为空数组 DEW WATER ALCHOLE WASH ER1 ER2
mixstation_clear_state_t mixstation_clear_state[6];

short last_tmper[30] = {0};
parameter_t par;    // 读取配置文件中保存的玻片位置信息数据 

// antibody[36];	//抗体坐标点由PC发来数据确定   用作清洗时标记
// ordArray_H2O2[8];		//H2O2坐标点由PC发来数据确定
// ordArray_DAB[4];		//DAB坐标点由PC发来数据确定
//FILE* scan_fp = NULL;

/*************锁和条件变量*************/
//pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
//pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t head_lock = PTHREAD_MUTEX_INITIALIZER;//操作数据结构锁
pthread_mutex_t error_lock = PTHREAD_MUTEX_INITIALIZER;//错误链表锁
pthread_mutex_t error_exist_lock = PTHREAD_MUTEX_INITIALIZER;//错误存在链表锁
pthread_mutex_t report_lock = PTHREAD_MUTEX_INITIALIZER;//大容量数据锁
pthread_mutex_t netsend_lock = PTHREAD_MUTEX_INITIALIZER;//发送网络数据锁
pthread_mutex_t pump_lock = PTHREAD_MUTEX_INITIALIZER;//调用压缩泵锁
pthread_mutex_t head_step_lock = PTHREAD_MUTEX_INITIALIZER;//步数数据锁
pthread_mutex_t minib_send_lock = PTHREAD_MUTEX_INITIALIZER;//minib板接收命令锁

/*************************/

/*********三架玻片运行数据状态 可改成结构体便于维护***********/
volatile unsigned char work_cnt = 0;//  用于不定时初始化机械臂
volatile unsigned int workstep_state_a = 0X01110000;	//记录操作步数 最高字节置1表示温度加热完成，次高字节置1表示滴加动作完成 最低字节表示步数 第5个1表示时间到达
volatile unsigned int workstep_state_b = 0X01110000;
volatile unsigned int workstep_state_c = 0X01110000;
char report_reagentinfo[3] = {0};
	
volatile bool StartDispenseA = false, StartDispenseB = false, StartDispenseC = false;

unsigned int head1_start_time = 0;
unsigned int head2_start_time = 0;
unsigned int head3_start_time = 0;
volatile operate_head_list* operate_pri = NULL;  //最先开始架次    3架中接收到整个流程规则从3架都没开始过该时指定从哪家开始运行 andry

extern int mulnum_glb[3];
extern int mulnum_glb2[3];

volatile bool HEAD1_STEP_SENDED = true;
volatile bool HEAD2_STEP_SENDED = true;
volatile bool HEAD3_STEP_SENDED = true;
volatile bool flg_opwork1ready = false;	//玻片架1正在工作标识	用于结束时确定发给哪个minib stop命令. andry:接收到最后一个规程置位该标志表示可以开始工作了
volatile bool flg_opwork2ready = false;
volatile bool flg_opwork3ready = false;

extern volatile bool flg_temperheating1; //A架正在加热标识
extern volatile bool flg_temperheating2;
extern volatile bool flg_temperheating3;
static bool DAB1Array_exchange = false;//DAB瓶交换使用
static bool DAB2Array_exchange = false;//DAB瓶交换使用
static bool DAB3Array_exchange = false;//DAB瓶交换使用
volatile bool isDAB_mixedA = false;//A架DAB是否混合标识
volatile bool isDAB_mixedB = false;
volatile bool isDAB_mixedC = false;
volatile bool isDAB_mixedA_next = false;//A架DAB是否混合标识
volatile bool isDAB_mixedB_next = false;
volatile bool isDAB_mixedC_next = false;

extern volatile bool flg_checkreagentindex[]; //PC发送测试试剂位置
extern volatile char flg_sancmixstatioin;//PC发送混合站扫描

char aspirate_notenough[11];//第11为为能滴加的片数

unsigned int dispense_mixreagent_timeA = 0;	//流程中到滴加混合试剂那一步还有多少时间
unsigned int dispense_mixreagent_timeB = 0;
unsigned int dispense_mixreagent_timeC = 0;
unsigned int dispense_mixreagent_timeA_next = 0;	//流程中到滴加混合试剂那一步还有多少时间
unsigned int dispense_mixreagent_timeB_next = 0;
unsigned int dispense_mixreagent_timeC_next = 0;
char which_mix_kindA = 0, which_mix_kindB = 0, which_mix_kindC = 0;  // DAB 当前需要混合的玻片架号标识 andry
unsigned char workstep_mix_a = 0;		//混合操作的步数
unsigned char workstep_mix_b = 0;
unsigned char workstep_mix_c = 0;

extern volatile stminibd_cmdlist* pcmd_head;//A架自控臂命令头指针
extern volatile stminibd_cmdlist* pcmd_head2;//B架自控臂命令头指针
extern volatile stminibd_cmdlist* pcmd_head3;//C架自控臂命令头指针
ewkevent wkeventA = FREE_WORK;	//A架玻片工作状态
ewkevent wkeventB = FREE_WORK;//B架玻片工作状态
ewkevent wkeventC = FREE_WORK;//C架玻片工作状态

unsigned int Array_mixed_DAB1[2] = {2504, 0}; //混合后DAB坐标		由于两个轮换使用需要从文件读取
unsigned int Array_mixed_DAB2[2] = {2640, 0};
unsigned int Array_mixed_DAB3[2] = {2768, 0};

unsigned int Array_mixed_RED1[2] = {2504, 0}; //混合后RED坐标	
unsigned int Array_mixed_RED2[2] = {2640, 0};
unsigned int Array_mixed_RED3[2] = {2768, 0};
				
volatile hydrate_t hydrateA;//A架玻片水合操作链表
volatile hydrate_t hydrateB;
volatile hydrate_t hydrateC;
volatile bool KEYENDA, KEYENDB, KEYENC;

extern volatile bool flg_shelfcanindex[3];
extern volatile unsigned char    mini_recieve_codeACK;//A架接收的minib命令码
extern volatile unsigned char    mini_recieve_code2;
extern volatile unsigned char    mini_recieve_code3;
extern volatile bool mini_finished; 	//每次自控臂命令结束标志 用于自控臂动作的顺序性
extern volatile bool mini_finished2;
extern volatile bool mini_finished3;
extern bool flg_mianarmstop_a; //主臂所在架次的操作结束标志 用于不同架次主臂动作的顺序性, andry: 停止后置TRUE，工作时置FALSE 标志
extern bool flg_mianarmstop_b;
extern bool flg_mianarmstop_c;
unsigned int liquid_valA = 0; //试剂的分配量 100 or 150
unsigned int liquid_valB = 0;
unsigned int liquid_valC = 0;
mix_t mix_DAB[3];//每架混合液信息  按玻片位置填入
mix_t mix_SECEND[3];   //  双染时需要混合的试剂？？？？ andry

volatile unsigned char last_reagentA = NO_REAGENT;//A架玻片最近一次滴加的试剂
volatile unsigned char last_reagentB = NO_REAGENT;
volatile unsigned char last_reagentC = NO_REAGENT;
volatile char lastt_kind[3][9];
extern volatile char shelf_stirmode[];
extern 	volatile unsigned int shelf_stirtime[][3];

volatile bool IsWashProbeStart = false;

unsigned int ordArray_OW[2] = {6, 46};

//清洗站坐标
#if (1 == MACH_S300)
unsigned int ordArray_wash[4][2]={
		{2325, 32},{2360, 32},{2395, 32},{2430, 32}
};
#elif(1 == MACH_S600)
unsigned int ordArray_wash[4][2]={
		{2325, 632},{2360, 632},{2395, 632},{2430, 632}
};
#endif

 int shelf_check[2][3] = {
		{2416,250,500},{2416,1110,500} };

//试剂架坐标
unsigned int ordArray_reagent[36][2] = {
		{2416,250},{2416,370},{2416,490},{2416,610},{2416,730},{2416,850},{2416,970},{2416,1090},{2416,1110},
		{2528,250},{2528,370},{2528,490},{2528,610},{2528,730},{2528,850},{2528,970},{2528,1090},{2528,1110},
		{2680,250},{2680,370},{2680,490},{2680,610},{2680,730},{2680,850},{2680,970},{2680,1090},{2680,1110},
		{2840,250},{2840,370},{2840,490},{2840,610},{2840,730},{2840,850},{2840,970},{2840,1090},{2840,1110}
};
//混合站坐标
#if (1 == MACH_S300)
unsigned int ordArray_mixed_DAB[6][2] = {
		{2512, 0},{2579, 0},{2646, 0},{2713, 0},{2800, 0},{2864, 0}
};
#elif(1 == MACH_S600)
unsigned int ordArray_mixed_DAB[6][2] = {
		{2512, 600},{2579, 600},{2646, 600},{2713, 600},{2800, 600},{2864, 600}
};
#endif

unsigned int ordArray_plate1[30][2] = {			//每个玻片初始位置
		{232,0},{232,239},{232,470},{232,702},{232,933},{232,1164},{232,1396},{232,1627},{232,1858},{232,2089},
		{1128,0},{1128,239},{1128,470},{1128,702},{1128,933},{1128,1164},{1128,1396},{1128,1627},{1128,1858},{1128,2089},
		{2008,0},{2008,239},{2008,470},{2008,702},{2008,933},{2008,1164},{2008,1396},{2008,1627},{2008,1858},{2008,2089}
		
}; 
 
unsigned int ordArray_plate2[30][2] = {			//每个玻片150ul
		{160,0},{160,239},{160,470},{160,702},{160,933},{160,1164},{160,1396},{160,1627},{160,1858},{160,2089},
		{1058,0},{1058,239},{1058,470},{1058,702},{1058,933},{1058,1164},{1058,1396},{1058,1627},{1058,1858},{1058,2089},
		{1936,0},{1936,239},{1936,470},{1936,702},{1936,933},{1936,1164},{1936,1396},{1936,1627},{1936,1858},{1936,2089}
};

unsigned int ordArray_plate3[30][2] = {			//每个玻片100ul位置
		{100,0},{100,239},{100,470},{100,702},{100,933},{100,1164},{100,1396},{100,1627},{100,1858},{100,2089},
		{998,0},{998,239},{998,470},{998,702},{998,933},{998,1164},{998,1396},{998,1627},{998,1858},{998,2089},
		{1864,0},{1864,239},{1864,470},{1864,702},{1864,933},{1864,1164},{1864,1396},{1864,1627},{1864,1858},{1864,2089}
			
};

unsigned int ordArray_plate4[30][2] = {			//每个玻片组织位置
		{100, 0},{100,239},{100,470},{100,702},{100,933},{100,1164},{100,1396},{100,1627},{100,1858},{100,2089},
		{998, 0},{998,239},{998,470},{998,702},{998,933},{998,1164},{998,1396},{998,1627},{998,1858},{998,2089},
		{1864, 0},{1864,239},{1864,470},{1864,702},{1864,933},{1864,1164},{1864,1396},{1864,1627},{1864,1858},{1864,2089}
			 
};


#define INITPOS 2600
unsigned int load_motor[5] = {16000, 29000, 35000, 43000, 95000};//复位 装载 150 拉伸一半 拉伸全部
unsigned int arm_motor[5] = {0, INITPOS, INITPOS + 300 , INITPOS + 650, INITPOS + 1000};
unsigned int load_motor2[5] = {16000, 29000, 27000, 43000, 100000};
unsigned int arm_motor2[5] = {0, INITPOS, INITPOS + 300 , INITPOS + 650, INITPOS + 1000};
unsigned int load_motor3[5] = {16000, 29000, 27000, 43000, 95000};
unsigned int arm_motor3[5] = {0, INITPOS, INITPOS + 300, INITPOS + 650, INITPOS + 1000};





extern struct timeval begin_time;//程序开始运行时间

extern  double GetVal30(int step );
	
int sb_waitingframeaswer(sr_cmdstruct_t * CommandElemt);
void CutoffSpeedChange(short speed);


/******************************************************************************
*
* Function Name  : mb_setreportval
* Description    : .向报告链表中插入值
* 					 
* Input		   : stnet_report
* Output		   : None
* Return		   :  None
*******************************************************************************/
void mb_setreportval(stnet_report report)
{
	stnet_reportlist * preport= preport_head;;

	if (preport_head->report.num == 0XFF)
		preport_head->report = report;
	else
	{
		while(preport->next != NULL)
			preport = preport->next;
		
		while(( preport->next = (stnet_reportlist*)malloc(sizeof(stnet_reportlist)) ) == NULL)
		{
			sleep(1);
			printf("malloc error stnet_reportlist\n");
		}
		preport->next->report = report;
		preport->next->next = NULL;
	}
}


/******************************************************************************
*
* Function Name  : mb_readreportval
* Description    : .向报告链表中读取值
* 					 
* Input		   : None
* Output		   : None
* Return		   :  stnet_report
*******************************************************************************/
stnet_report mb_readreportval(void)
{
	stnet_report report;
	stnet_reportlist* preport;
	
	if (preport_head->report.num == 0XFF)
		return preport_head->report;

	report = preport_head->report;
	
	if (preport_head->next == NULL)
	{	
		preport_head->report.num = 0XFF;
	}
	else
	{
		preport = preport_head;
		preport_head = preport_head->next;
		free(preport);
		preport = NULL;
	}

	return report;	
}

/******************************************************************************
*
* Function Name  : mb_geterrorcode
* Description    : .向错误链表中读取值
* 					 
* Input		   : None
* Output		   : None
* Return		   :  int
*******************************************************************************/
int mb_geterrorcode(void)
{
	int error_code;
	sterrcode_list* perror;
	
	if (perror_head->error_code == NO_ERROR)
		return NO_ERROR;

	error_code = perror_head->error_code;
	
	if (perror_head->next == NULL)
	{	
		perror_head->error_code = NO_ERROR;
	}
	else
	{
		perror = perror_head;
		perror_head = perror_head->next;
		free(perror);
		perror = NULL;
	}

	return error_code;	
}

/******************************************************************************
*
* Function Name  : mb_geterrorcode
* Description    : .向错误链表中插入值
* 					 
* Input		   : int
* Output		   : None
* Return		   :  None
*******************************************************************************/
void mb_seterrorcode(int error_code)
{
	sterrcode_list * perror= perror_head;
	char senderrcodebuf[30];
	uint8_t i=0;

	memcpy(senderrcodebuf, &error_code, 4);
	if (DOOR_OPEN == error_code || MIX_STATION_CLEARERROR == error_code ||
		(error_code != ASPIRATE_WRONG_NOT_ENOUGH && error_code >= WAST_LIUID_LOW && error_code <= POWER_ERROR) || 
		(error_code >= WATERPOUR_WRONG && error_code <= ER2POUR_WRONG) )
	{
		senderrcodebuf[4] = 0;
		nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
	}
	else if ( ( error_code >= TEMPER1_EM && error_code <= TEMPER10_EM && tem_over_load)
		|| MIXA_DILUENT_WRONG == error_code || MIXA_DAB_WRONG == error_code)
	{
		senderrcodebuf[4] = 'A';
		nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
	}
	else if ( ( error_code >= TEMPER11_EM && error_code <= TEMPER20_EM && tem_over_load)
		|| MIXB_DILUENT_WRONG == error_code || MIXB_DAB_WRONG == error_code)
	{
		senderrcodebuf[4] = 'B';
		nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
	}
	else  if ( ( error_code >= TEMPER21_EM && error_code <= TEMPER30_EM && tem_over_load)
		|| MIXC_DILUENT_WRONG == error_code || MIXC_DAB_WRONG == error_code)
	{
		senderrcodebuf[4] = 'C';
		nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
	}
	else if (error_code >= TEMPER1 && error_code <=  TEMPER30)
	{	
		senderrcodebuf[4] =  error_code - TEMPER1 + 1;
		#if(USE_LOG_INFO == 1)
		printf("senderrcobuf[0]=%d senderrcodebuf[4]=%d\n", *(int*)(&senderrcodebuf[0]), senderrcodebuf[4]);
		#endif
		nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
	}
	else if (error_code == ASPIRATE_WRONG )
	{
		if(aspirate_notenough[10] == 10)
		{
			if (!flg_mianarmstop_a)
			{
				senderrcodebuf[4] = 'A';
				nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
			}
			else if (!flg_mianarmstop_b)
			{
				senderrcodebuf[4] = 'B';
				nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
			}
			else if (!flg_mianarmstop_c)
			{
				senderrcodebuf[4] = 'C';
				nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 5);
			}
		}
		else
		{
			#if(USE_LOG_INFO == 1)
			printf("aspirate_wrongaspirate_wrong\n");
			for(i=0;i<aspirate_notenough[10];i++)
				printf(" %d ", aspirate_notenough[i]);
			#endif
			memcpy(&senderrcodebuf[4], aspirate_notenough, aspirate_notenough[10]);
			nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 4 + aspirate_notenough[10]);
		}
	}
	else if(error_code == ASPIRATE_WRONG_NOT_ENOUGH )
	{
		#if(USE_LOG_INFO == 1)
		printf("aspirate_notenoughaspirate_notenough\n");
		for(i=0;i<aspirate_notenough[10];i++)
			printf(" %d ", aspirate_notenough[i]);
		#endif
		memcpy(&senderrcodebuf[4], aspirate_notenough, aspirate_notenough[10]);
		nt_sendpacketdata(AddBatchErrorLog, senderrcodebuf, 4 + aspirate_notenough[10]);	
	}

	pthread_mutex_lock(&error_lock);
	if (perror_head->error_code == NO_ERROR)
		perror_head->error_code = error_code;
	else
	{
		while(perror->next != NULL)
			perror = perror->next;
		
		while((perror->next = (sterrcode_list*)malloc(sizeof(sterrcode_list))) == NULL)
		{
			sleep(1);
			printf("malloc error sterrcode_list\n");
		}
		perror->next->error_code = error_code;
		perror->next->next = NULL;
	}
	if (pthread_mutex_unlock(&error_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error error_lock");
}


/******************************************************************************
*
* Function Name  : mb_finderrorcodeinlist
* Description    : .在错误存在链表中查找值，error_code为需要寻找的值，ISDELETE为TRUE时 找到后删除
* 					 
* Input		   : int BOOL
* Output		   : None
* Return		   :  找到TRUE 没找到FALSE
*******************************************************************************/
bool mb_finderrorcodeinlist(int error_code, bool flgdelect)
{
	sterrcode_list * perror = perror_exist_head;
	sterrcode_list * last_perror = perror_exist_head;

	#if(USE_LOG_INFO == 1)
	printf("[mb_finderrorcodeinlist] error_code=%x\n", error_code);
	printf("errorcode_exist=");
	while(perror != NULL)
	{
		printf(" %d ", perror->error_code);
		perror = perror->next;
	}
	#else
	while (perror != NULL)
	{
		perror = perror->next;
	}
	
	#endif

	error_code = error_code & 0X00FFFFFF;
	perror = perror_exist_head;
	while(perror != NULL)
	{
		if (perror->error_code == error_code)
		{
			if (flgdelect)
			{
				if (perror == perror_exist_head)
				{
					if (perror->next == NULL)
					{
						perror->error_code = NO_ERROR;	
					}
					else
					{
						perror_exist_head = perror_exist_head->next;
						free(last_perror);
						last_perror = NULL;
					}
				}
				else
				{
					last_perror->next = perror->next;
					free(perror);
					perror = NULL;
				}
			}		
			return true;
		}
		last_perror = perror;
		perror = perror->next;
	}
	return false;
}


void mb_printfoperatelist(operate_head_list* operate_head)
{
	operate_t* operate_p = NULL;
	operate_head_list* operate_head_p = NULL;
	printf("data in list\n");
	operate_head_p = operate_head;
	int first_plate_num[200] = {0};
	int i = 0,shelf_num;
	
	char isfirst = 1;
		
	if (operate_head == operate_head1)
		shelf_num = 0;
	else if (operate_head == operate_head2)
		shelf_num = 1;
	else
		shelf_num = 2;

	while(operate_head_p != NULL)
	{
		operate_p = &(operate_head_p->operate);

		i = 0;
		while(operate_p != NULL)
		{
			if (isfirst)
			{
				first_plate_num[i] = operate_p->plate_num;	
			}
			else
			{
				if(first_plate_num[i] > operate_p->plate_num && (first_plate_num[i]) > (operate_p->plate_num + 20))
					mb_seterrorcode( ARM_COLLISION_AVOIDED);
				if(first_plate_num[i] < operate_p->plate_num && (first_plate_num[i] + 20) < operate_p->plate_num)
					mb_seterrorcode( ARM_COLLISION_AVOIDED);
			}
			i++;
			printf(" %d %d %d %d", operate_p->reagent, operate_p->time, operate_p->temp, operate_p->plate_num);		
			printf("--%s", operate_p->reagent_info.code);
			printf("--%s", operate_p->reagent_info.reagent_kind);	
			printf("--%d *", operate_p->reagent_info.special_num);
			
			operate_p = operate_p->next;
		}
		isfirst = 0;
		printf("\n\n");
		operate_head_p = operate_head_p->next_head;
	}

	printf("mix_dab=\n");
	for(i = 0; i < 10; i++)
	{	
		printf(" %d ", mix_DAB[shelf_num].ordArrayA[i]);	
		printf("reagentA code=%s kind=%s \n", mix_DAB[shelf_num].reagentA[i].code, mix_DAB[shelf_num].reagentA[i].reagent_kind);
		printf(" %d ", mix_DAB[shelf_num].ordArrayB[i]);
		printf("reagentB code=%s kind=%s \n", mix_DAB[shelf_num].reagentB[i].code, mix_DAB[shelf_num].reagentB[i].reagent_kind);		
	}

	printf("mix_second=\n");
	for(i = 0; i < 10; i++)
	{
		printf(" %d ", mix_SECEND[shelf_num].ordArrayA[i]);
		printf(" %d ", mix_SECEND[shelf_num].ordArrayB[i]);		
	}
	printf("\n");
}


void mb_setallpara(void)
{
	if (SELF_ICOVERT)
	{
		load_motor[2] = 25000;
		load_motor[3] = 43000;
		load_motor[4] = 100000;
		load_motor2[2] = 25000;
		load_motor2[3] = 43000; 	
		load_motor2[4] = 100000;
		load_motor3[2] = 25000;
		load_motor3[3] = 43000;
		load_motor3[4] = 100000;
		PULL_DISTANC_INIT = 5 + TULUN_MISTAKE;
		PULL_DISTANC_LT = 32 - TULUN_MISTAKE*2;//初始位置网左走   拉伸位置网右走
		PULL_DISTANC_HALF = 80 - TULUN_MISTAKE*2;
		PULL_DISTANC_FULL = 155 - TULUN_MISTAKE*2;	
	}
	else
	{
		#if NEW_STRECH
		load_motor[2] = 27000;
		load_motor[3] = 40000;
		load_motor[4] = 103000;
		load_motor2[2] = 27000;
		load_motor2[3] = 40000;
		load_motor2[4] = 103000;
		load_motor3[2] = 27000;
		load_motor3[3] = 40000;
		load_motor3[4] = 103000;
		#else
		load_motor[2] = 27000;
		load_motor[3] = 40000;
		load_motor[4] = 98000;
		load_motor2[2] = 27000;
		load_motor2[3] = 40000;
		load_motor2[4] = 98000;
		load_motor3[2] = 27000;
		load_motor3[3] = 40000;
		load_motor3[4] = 98000;
		#endif
		#ifdef LAIYUE
		load_motor[4] = 98000;
		load_motor2[4] = 98000;
		load_motor3[4] = 98000;
		#endif

		PULL_DISTANC_INIT = TULUN_MISTAKE;
		PULL_DISTANC_LT = 32 - TULUN_MISTAKE*2;
		PULL_DISTANC_HALF = 76 - TULUN_MISTAKE*2;
		PULL_DISTANC_FULL = 175 - TULUN_MISTAKE*2;
	}
		
	SCAN_DISTANC_MIXSTATION = 30;

	if (NEW_MIXSTATION)
	{
		#ifdef BIG_VERSION//53.9
			MIX_WASH_DISTANCE = 241;//0.22345
		#else
			MIX_WASH_DISTANCE = 292;
		#endif
	}
	else
	{
		MIX_WASH_DISTANCE = 185;
	}

	#ifdef BIG_VERSION//19.85
	{
		// OFFSET_STEP = 210;
		OFFSET_STEP = 202;//理论值202
		SHELF_OFFSET = 97;
		MOV_ZH = 0;
	}
	#else
	{
		MOV_ZH = 300;
		SHELF_OFFSET = 0;

		OFFSET_STEP = 0;
	}
	#if(MACH_S600 == 1)
		MIX_WASH_DISTANCE = 210; 
		OFFSET_STEP = 204;//理论值202
		SHELF_OFFSET = 0;
		MOV_ZH = 90;
		
	#endif

	#endif
	ZSPEED_SLOW = 4000;
	ZWASH_STEP1 = 650 - OFFSET_STEP; // 1号口的最深位置
	ZWASH_STEP2_1 = 543 - OFFSET_STEP; 	// 2号的状态1
	ZWASH_STEP2_2 = 450 - OFFSET_STEP; // 2号的状态2
	ZWASH_STEP2_ZX = 520 - OFFSET_STEP;
	ZWASH_STEP3 = 565 - OFFSET_STEP;
	ZWASH_STEP4 = 1000 + 200 - OFFSET_STEP;
	ZWASH_STEP4_SLOW = 600 - OFFSET_STEP;
	ZDAB_STEP = zDAB_step - OFFSET_STEP;
	DAB_ZMAX = DAB_zmax  - OFFSET_STEP;

	#ifdef BIG_VERSION
	LIQUID_ZMAX =liquid_zmax  - OFFSET_STEP + SHELF_OFFSET;
	OPEN_ARMZMAX =open_armzmax - OFFSET_STEP+ SHELF_OFFSET;
	#else
	LIQUID_ZMAX = liquid_zmax;
	OPEN_ARMZMAX = open_armzmax;

	#endif

	if (SELF_PUMP)
	{
		PORT_OFFSET= 1;
		PORT_OFFSET_POOL = 7;

		SYSPEED_TST = 10;
		SYSPEED = 40;
		SYSPEED_CLEAN = 40;
		SYSPEED_AIR = 8;
		SYSPEED_WASTE = 8;
			
		SYSPEED_LIQUID = 8;
		SYSPEED_LIQUIDEBER = 6;
		SYSPEED_DISPENSE = 40;
		SYSPEED_DISPENSE_SHELF = 20;
		SYSPEED_ALCHOLE = 44;

		START_DISPENSE = 50;
		SYSSTARTSPD = 400;
		SYSCUTOFF_SPEED = 1500;//2000;

		SYSPEED_MIXDAB = 35;
		SYDELAY = 800;
		SYFSTEP = 9056;//6804
		SY45STEP = 7252	;// 4/5
		SYHSTEP = 4532;
		SYMHSTEP = 6032;	//一半多点
		SYLIQTSTSTEP = 4648 ;
		SYHHSTEP = 2266	;	//一半的一半
		SY15STEP = 1813	;// 1/5
		SY110STEP = 906;// 1/10
		SYSTEP_AIR = 37;  //5ul   travel空气柱						、、SYSTEP_AIR_LAG必须要大于 SYSTEP_AIR5ul
		SYSTEP_AIR_LAG = 55; //10ul  吸试剂架试剂前吸的空气柱  越小空气柱越稳定

		SYSTEP_WASTE = 725;
		SYSTEP_LIQTSTAIR = 150;
		SYSTEP_LIQUID = 362;//吸一张玻片液PUMP走的步数 (分配量为100ul) //测得量为43  额定为240步
		SYSTEP_LIQUID_REMAIN = 75;
	}	
	else  //  end of if (SELF_PUMP)
	{
		PORT_OFFSET = 0;
			
		SYSPEED_TST = 16;
		SYSPEED = 6;	// 32K //5   6(2600脉冲/秒)   瑞泽注射泵测量时有气泡，刘玉龙改为12（1200脉冲/秒）
		SYSPEED_CLEAN = 8;
		SYSPEED_AIR = 20;
		SYSPEED_WASTE = 16;
		SYSPEED_ALCHOLE = 12;

		SYSPEED_LIQUID = 15; // 16;       2024-05-24  车间因一台设备挂液，第一次做片好的，第二次阴性，所以增加一档看
		SYSPEED_LIQUIDEBER = 19; // 20;
		SYSPEED_DISPENSE = 11;// 12;// 12//the dospense speed should be fast但是太快可能会将气泡滴加进去
		SYSPEED_DISPENSE_SHELF = 14; // 15;
		SYSPEED_DISPENSEFUL = 15;
		SYSSTARTSPD = 900;
		SYSCUTOFF_SPEED = 2000;
		SYSPEED_MIXDAB = 7;
		SYDELAY = 800;
		SYFSTEP = 6000;
		SY45STEP = 4800;	// 4/5
		SYHSTEP = 3000;
		SYMHSTEP = 4000;	//一半多点
		SYLIQTSTSTEP = 3100 ;
		SYHHSTEP = 1500	;	//一半的一半
		SY15STEP = 1200;	// 1/5
		SY110STEP = 600;// 1/10
		SYSTEP_AIR = 12;  //5ul   travel空气柱						、、SYSTEP_AIR_LAG必须要大于 SYSTEP_AIR5ul
		SYSTEP_AIR_LAG = 18 ; //5ul  吸试剂架试剂前吸的空气柱  越小空气柱越稳定
		SYSTEP_WASTE = 480;
		SYSTEP_LIQTSTAIR = 100;
		SYSTEP_LIQUID = 240;//吸一张玻片液PUMP走的步数 (分配量为100ul) //测得量为43  额定为240步
		SYSTEP_LIQUID_REMAIN = 50 ;
	}
}


/*************从文件读取配置信息**************/
void mb_readconfparatinfo(char IsDef)
{
	char read_buffer[1000]={0};
	char *pstring = malloc(1000);

	if(pstring == NULL){
		printf("mb_readconfparatinfo malloc 1000 bytes error.\n");
		lprintf(log_my, ERROR, "mb_readconfparatinfo malloc 1000 bytes error.\n");
		return;
	}

	unsigned int i = 0, j = 0, ocnt = 0;
	char *ptr = NULL;
	char *ppath = NULL;
	int X_coordinate = 0, cal_tmp = 0;

	if (IsDef)
		ppath = MeasureConfigDefPath;
	else
		ppath = MeasureConfigPath;

	conf_zdab_addstep = rf_readintfromeprofile("ZdbaAddstep", "ZDBAADDSTEPS", 0, (const char*)ppath);

	X_coordinate = rf_readintfromeprofile("ReadConfigInfo", "LIQUID_SPEED", -1, (const char *)ppath);

	if (X_coordinate <= 100)
		X_coordinate = 350;
	scanoffset =(X_coordinate - 350) ;

	printf("scanoffset=%d\n\n", scanoffset);

	new_temper = rf_readstringfromeprofile("ReadtemperInfo", "temp_Input_Bias[60]", read_buffer, 1000, NULL, (const char *)ppath);
	new_version = new_temper;

	mb_readrunparainfo();

	big_version = 0;

	self_icovert = rf_readintfromeprofile("ReadConfigInfo", "SELF_ICOVERT", -1, (const char *)ppath);
	self_pump = rf_readintfromeprofile("ReadConfigInfo", "SELF_PUMP", -1, (const char *)ppath);
	new_mixstation = rf_readintfromeprofile("ReadConfigInfo", "NEW_MIXSTATION", -1, (const char *)ppath);   //  1
	shelf_lock_new = rf_readintfromeprofile("ReadConfigInfo", "SHELF_LOCK_NEW", -1, (const char *)ppath);   //  1
	new_outwast_sen = rf_readintfromeprofile("ReadConfigInfo", "NEW_OUTWAST_SEN", -1, (const char *)ppath);  //  1  废液桶在位检测传感器检测电平
	#if NEW_BARREL
	new_inwast_sen = rf_readintfromeprofile("ReadConfigInfo", "NOR_WAST_BARREL", -1, (const char *)ppath);  // 1   废液桶液位检测传感器电平方式
	#endif

	#if (USE_LOG_INFO == 1)
	{
		printf("big_version=%d self_icovert=%d, self_pump=%d, new_mixstation=%d, shelf_lock_new=%d, new_outwast_sen=%d\n",
					big_version, self_icovert, self_pump, new_mixstation, shelf_lock_new, new_outwast_sen);
	}
	#endif

	pour_liquid_zmax = rf_readintfromeprofile("ReadConfigInfo", "POUR_LIQUID_ZMAX", -1, (const char *)ppath);

	if(pour_liquid_zmax >= 2000)
		honey_scaner = 1;
	else
	{
		honey_scaner = 0;
		if (pour_liquid_zmax >= 1000)
			new_scaner = 1;
		else
			new_scaner = 0;
	}

	DAB_zmax = rf_readintfromeprofile("ReadConfigInfo", "DAB_ZMAX", -1, (const char *)ppath);
	
	rf_readstringfromeprofile("ReadConfigInfo", "isREAGENT_IN_CABIN_AVAILABLE[6]", read_buffer, 1000, NULL, (const char *)ppath);

	printf("honey_scaner=%d new_scaner=%d\n", honey_scaner, new_scaner);

	for (i = 0; i < 6; i++)
	{
		strcpy(pstring, read_buffer);
		flg_cabinhavereagent[i] = atoi(strtok_r(pstring, "|", &ptr));	
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}

	printf("Read Config info befor call the strtok_r function.\n");

	rf_readstringfromeprofile("ReadCoordinateInfo", "ordArray_wash[3]", read_buffer, 100,NULL, (const char *)ppath);
	rf_read2array(par.ordArray_wash, read_buffer, 1);
	rf_readstringfromeprofile("ReadCoordinateInfo", "ordArray_reagent[4][3]", read_buffer, 100, NULL, (const char *)ppath);
	rf_read2array(par.ordArray_reagent, read_buffer, 4);

	rf_readstringfromeprofile("ReadCoordinateInfo", "ordArray_plate[9][3]", read_buffer, 200, NULL, (const char *)ppath);
	rf_read2array(par.ordArray_plate, read_buffer, 9);

	printf(" par.ordArray_plate[i*3+2][2]; =%d.\n", par.ordArray_plate[6+2][2]);

	rf_readstringfromeprofile("ReadCoordinateInfo", "shelf_check[2][3]", read_buffer, 100, NULL, (const char *)ppath);
	rf_read2array(par.shelf_check, read_buffer, 2);
	rf_readstringfromeprofile("ReadCoordinateInfo", "ordArray_OW[3]", read_buffer, 100, NULL, (const char *)ppath);
	// rf_read2array(par.ordArray_OW[0], read_buffer, 1);
	rf_read2array(par.ordArray_OW, read_buffer, 1);
	ordArray_OW[0] = par.ordArray_OW[0];
	ordArray_OW[1] = par.ordArray_OW[1];
	zwash_step4 = par.ordArray_wash[2];

	rf_readstringfromeprofile("ReadMiniboardInfo", "load_motor1", read_buffer, 100, NULL, (const char *)ppath);

	par.load_motor1[0] = load_motor[0];
	par.load_motor1[1] = load_motor[1];
	par.load_motor2[0] = load_motor2[0];
	par.load_motor2[1] = load_motor2[1];
	par.load_motor3[0] = load_motor3[0];
	par.load_motor3[1] = load_motor3[1];
	liquid_zmax = par.ordArray_reagent[0][2];

	printf("ReadConfig infomation file will set the global vaule.\n");

	mb_setallpara();

	printf("Global seted infomation parametere.\n");

	for(i = 0; i < 4;i++)
	{
		ordArray_wash[i][0] = par.ordArray_wash[0]  + i * WASH_DISTANCE;
		ordArray_wash[i][1] = par.ordArray_wash[1]; 
	}
	
	ordArray_mixed_DAB[0][0] = ordArray_wash[0][0] + MIX_WASH_DISTANCE;
	ordArray_mixed_DAB[0][1] = 0;//初始化位置

	for(i = 0; i < 6;i++)
	{
		ordArray_mixed_DAB[i][0] = ordArray_mixed_DAB[0][0] + i * MIX_DISTANCE;
		ordArray_mixed_DAB[i][1] = ordArray_mixed_DAB[0][1];
	}
	
	for(i = 0; i < 9;i++)
	{
		ordArray_reagent[i][0] = par.ordArray_reagent[0][0];
		ordArray_reagent[i][1] = par.ordArray_reagent[0][1] + i * reagent_distance;

		ordArray_reagent[i + 9][0] = par.ordArray_reagent[1][0];
		ordArray_reagent[i + 9][1] = par.ordArray_reagent[1][1] + i * reagent_distance;
		
		ordArray_reagent[i + 18][0] = par.ordArray_reagent[2][0];
		ordArray_reagent[i + 18][1] = par.ordArray_reagent[2][1] + i * reagent_distance;

		ordArray_reagent[i + 27][0] = par.ordArray_reagent[3][0]; 
		ordArray_reagent[i + 27][1] = par.ordArray_reagent[3][1] + i * reagent_distance; 
	} 
	
	for(i = 0; i < 3;i++)
	{
		dispense_stepA[i] = par.ordArray_plate[i*3][2];	
		dispense_stepB[i]= par.ordArray_plate[i*3+1][2]; 
		dispense_stepC[i] = par.ordArray_plate[i*3+2][2];   
	}

	printf("dispense_stepA[0]=%d dispense_stepA[1]=%d dispense_stepA[2]=%d\n",
	dispense_stepA[0], dispense_stepA[1], dispense_stepA[2]);
	
	printf("dispense_stepC[0]=%d dispense_stepC[1]=%d dispense_stepC[2]=%d\n",
	dispense_stepC[0], dispense_stepC[1], dispense_stepC[2]);

	if (shelf_check[0][2] > 600)
		shelf_check[0][2] = 500;
	if (shelf_check[1][2] > 600)
		shelf_check[1][2] = 500;
	memcpy(shelf_check, par.shelf_check, 24);

	printf("shelf_check[0][2]=%d, shelf_check[1][2]=%d\n", shelf_check[0][2], shelf_check[1][2]);
		
	int correc_dec = (shelf_check[0][2] - shelf_check[1][2]) % 8;
	int correc_base = (shelf_check[0][2] - shelf_check[1][2]) / 8;
	{	
		int correc_index1[8] = {3,1,5,2,4,0,6,7};// 余数为奇数分布
		int correc_index2[8] = {2,5,1,6,3,4,0,7};// 余数为偶数
		int correc_c[8] = {0};
		int *correc_p = NULL;
		int index = 0;
		if ((correc_dec % 2) == 1 || (correc_dec % 2) == -1)
			correc_p = correc_index1;
		else
			correc_p = correc_index2;
		
		for (index = 0; index < abs(correc_dec); index++)
		{
			if (correc_dec > 0)
				correc_c[ correc_p[index] ] = 1;
			else
				correc_c[ correc_p[index] ] = -1;
		}
	
		printf("correc_base=%d, correc_dec=%d\n", correc_base, correc_dec);

		CorrectDate[0] = correc_base;
		for (index = 0; index < 8; index++)
		{
			 CorrectDate[index + 1] = correc_base +correc_c[index] + CorrectDate[index];
			
			 printf(" %d ",  CorrectDate[index]);
		}
	}
	ordArray_plate1[0][0] = par.ordArray_plate[0][0] ;
	ordArray_plate1[10][0] = par.ordArray_plate[1][0] ;
	ordArray_plate1[20][0] = par.ordArray_plate[2][0] ;
	
 	ordArray_plate1[0][1] = par.ordArray_plate[0][1];
	ordArray_plate1[10][1] = par.ordArray_plate[1][1];
	ordArray_plate1[20][1] = par.ordArray_plate[2][1];

	ordArray_plate2[0][0] = par.ordArray_plate[3][0] ;
	ordArray_plate2[10][0] = par.ordArray_plate[4][0] ;
	ordArray_plate2[20][0] = par.ordArray_plate[5][0] ;
	
 	ordArray_plate2[0][1] = par.ordArray_plate[3][1];
	ordArray_plate2[10][1] = par.ordArray_plate[4][1];
	ordArray_plate2[20][1] = par.ordArray_plate[5][1];
	
	ordArray_plate3[0][0] = par.ordArray_plate[6][0] ;
	ordArray_plate3[10][0] = par.ordArray_plate[7][0] ;
	ordArray_plate3[20][0] = par.ordArray_plate[8][0] ;
	
 	ordArray_plate3[0][1] = par.ordArray_plate[6][1];
	ordArray_plate3[10][1] = par.ordArray_plate[7][1];
	ordArray_plate3[20][1] = par.ordArray_plate[8][1];
	
	ordArray_plate4[0][0] = par.ordArray_plate[0][0] - PULL_DISTANC_FULL;
	ordArray_plate4[10][0] = par.ordArray_plate[1][0] - PULL_DISTANC_FULL;
	ordArray_plate4[20][0] = par.ordArray_plate[2][0] - PULL_DISTANC_FULL;
	
 	ordArray_plate4[0][1] = par.ordArray_plate[0][1];
	ordArray_plate4[10][1] = par.ordArray_plate[1][1];
	ordArray_plate4[20][1] = par.ordArray_plate[2][1];

	// printf("!!!!!!!!!!!!!!!!1ordArray_plate2[10][0]=%d ordArray_plate3[10][0]=%d\n\n",&ordArray_plate2[10][0],
	// 			&ordArray_plate3[10][0]);
	int stp_patch = 0;
	for (i = 1; i < 10; i++) 
	{
		if (i%3 == 0)
			stp_patch = 1;
		else if (i%2 == 0)
			stp_patch = 1;
		else
			stp_patch = 0;
		
		ordArray_plate1[i][0] = par.ordArray_plate[0][0];
		ordArray_plate1[i][1] = ordArray_plate1[i-1][1] + slide_distance + stp_patch;
		
		ordArray_plate1[i + 10][0] = par.ordArray_plate[1][0] ;
		ordArray_plate1[i + 10][1] = ordArray_plate1[10 + i-1][1] + slide_distance + stp_patch;

		ordArray_plate1[i + 20][0] = par.ordArray_plate[2][0] ;
		ordArray_plate1[i + 20][1] = ordArray_plate1[20 + i-1][1] + slide_distance + stp_patch;
		
		ordArray_plate2[i][0] = par.ordArray_plate[3][0];
		ordArray_plate2[i][1] = ordArray_plate2[i-1][1] + slide_distance + stp_patch;
		
		ordArray_plate2[i + 10][0] = par.ordArray_plate[4][0] ;
		ordArray_plate2[i + 10][1] = ordArray_plate2[10 + i-1][1] + slide_distance + stp_patch;

		ordArray_plate2[i + 20][0] = par.ordArray_plate[5][0] ;
		ordArray_plate2[i + 20][1] = ordArray_plate2[20 + i-1][1] + slide_distance + stp_patch;

		ordArray_plate3[i][0] = par.ordArray_plate[6][0];
		ordArray_plate3[i][1] = ordArray_plate3[i-1][1] + slide_distance + stp_patch;
		
		ordArray_plate3[i + 10][0] = par.ordArray_plate[7][0] ;
		ordArray_plate3[i + 10][1] = ordArray_plate3[10 + i-1][1] + slide_distance + stp_patch;

		ordArray_plate3[i + 20][0] = par.ordArray_plate[8][0] ;
		ordArray_plate3[i + 20][1] = ordArray_plate3[20 + i-1][1] + slide_distance + stp_patch;
	
		ordArray_plate4[i][0] = par.ordArray_plate[0][0] - PULL_DISTANC_FULL;
		ordArray_plate4[i][1] =ordArray_plate1[i][1];
		
		ordArray_plate4[i + 10][0] = par.ordArray_plate[1][0] - PULL_DISTANC_FULL;
		ordArray_plate4[i + 10][1] = ordArray_plate1[i + 10][1];
	
		ordArray_plate4[i + 20][0] = par.ordArray_plate[2][0] - PULL_DISTANC_FULL;
		ordArray_plate4[i + 20][1] =ordArray_plate1[i + 20][1];
	}

	#if (USE_LOG_INFO == 1)
	{
		lprintf(log_my, INFO, "shelf_check\n");
		for(i = 0; i < 2;i++)
			lprintf(log_my, INFO, " *%d %d %d* ", shelf_check[i][0],shelf_check[i][1],shelf_check[i][2]);
		lprintf(log_my, INFO, "\nordArray_wash zwash_step4 = %d\n",zwash_step4);
		for(i = 0; i < 4;i++)
			lprintf(log_my, INFO, " *%d %d* ",ordArray_wash[i][0],ordArray_wash[i][1]);
		lprintf(log_my, INFO, "(sizeof(ordArray_reagent)) / 4 = %d\n", (sizeof(ordArray_reagent)) / 4);
		lprintf(log_my, INFO, "\nordArray_reagent liquid_zmax=%d\n",liquid_zmax);
		for (i = 0; i < 36; i++)
			lprintf(log_my, INFO, " *%d %d* ",ordArray_reagent[i][0],ordArray_reagent[i][1]);
		
		lprintf(log_my, INFO, "\nordArray_plate1 dispense_stepA=%d,dispense_stepB=%d,dispense_stepC=%d\n",
				dispense_stepA, dispense_stepB, dispense_stepC);
		lprintf(log_my, INFO, "ordArray_plate1\n");
		for (i = 0; i < 30; i++)
			lprintf(log_my, INFO, " *%d %d* ",ordArray_plate1[i][0],ordArray_plate1[i][1]);
		lprintf(log_my, INFO, "ordArray_plate2\n");
		for (i = 0; i < 30; i++)
			lprintf(log_my, INFO, " *%d %d* ",ordArray_plate2[i][0], ordArray_plate2[i][1]);
		
		lprintf(log_my, INFO, "ordArray_plate3\n");
		for (i = 0; i < 30; i++)	
			lprintf(log_my, INFO, " *%d %d* ",ordArray_plate3[i][0],ordArray_plate3[i][1]);
		
		lprintf(log_my, INFO, "ordArray_plate4\n");
		for (i = 0; i < 30; i++)	
			lprintf(log_my, INFO, " *%d %d* ",ordArray_plate4[i][0],ordArray_plate4[i][1]);
			
		lprintf(log_my, INFO, "load_motor[0]=%d load_motor[1]=%d load_motor2[0]=%d load_motor2[1]=%d load_motor3[0]=%d load_motor3[1]=%d\n" ,
			load_motor[0],load_motor[1],load_motor2[0],load_motor2[1],load_motor3[0],load_motor3[1]);
	}
	#endif

	rf_readstringfromeprofile("ReadtemperInfo", "temp_sensor_type[30]", read_buffer, 1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}

	rf_readstringfromeprofile("ReadtemperInfo","temp_SV_High[30]", read_buffer, 1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		par.temp_SV_High[i] = atoi(strtok_r(pstring, "|", &ptr));
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}

	rf_readstringfromeprofile("ReadtemperInfo", "temp_MV[30]", read_buffer,1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		par.temp_MV[i] = atoi(strtok_r(pstring, "|", &ptr));	
		strcpy(pstring, read_buffer);
		strcpy(read_buffer,&read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}
	
	if(new_temper)
	{
		rf_readstringfromeprofile("ReadtemperInfo", "temp_Input_Bias[60]", read_buffer, 1000, NULL, (const char *)ppath);
		for (i = 0; i < 30; i++)
		{		
			strcpy(pstring, read_buffer);
			temp_Dvalmy[i][72] = par.temp_Input_Bias[i] = atoi(strtok_r(pstring, "|", &ptr));	//参数中只存72°的修正值
			
			for(j = 36; j < 72; j++)    //  temp_Input_Bias[60]的前30个修正值是30片玻片72度的修正值，后30个是100读的修正值
			{
				if(temp_Dvalmy[i][72] < 0)
				{
					temp_Dvalmy[i][j] = (~temp_Dvalmy[i][72] + 1) *(j - 35) / 37;
					temp_Dvalmy[i][j] = ~temp_Dvalmy[i][j] + 1;
				}
				else
					temp_Dvalmy[i][j] = temp_Dvalmy[i][72] *(j - 35) / 37;
			}
			strcpy(pstring, read_buffer);
			strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
		}
		for (i = 0; i < 30; i++)
		{	
			strcpy(pstring, read_buffer);
			temp_Dvalmy[i][100] = par.temp_Input_Bias[ i + 30] = atoi(strtok_r(pstring, "|", &ptr));

			for(ocnt = 1; ocnt < 5; ocnt++)   // 100 ～ 105度的修正值跟100度一样
				temp_Dvalmy[i][100 + ocnt] = temp_Dvalmy[i][100];
			
			printf(" hh%d ", temp_Dvalmy[i][100]);

			cal_tmp = temp_Dvalmy[i][100] - temp_Dvalmy[i][72];   //  100度 - 72度 差值

			printf("73cal_tmp = %d\n", cal_tmp);

			for(j = 73; j < 100; j++)
			{		
				if(cal_tmp < 0)
				{
					temp_Dvalmy[i][j] =  (j - 72) * (~cal_tmp + 1) / 28;
					temp_Dvalmy[i][j] = ~temp_Dvalmy[i][j] + 1 + temp_Dvalmy[i][72];
					temp_setval[i][j + temp_Dvalmy[i][j]/10] = temp_Dvalmy[i][j];
				}
				else
				{
					temp_Dvalmy[i][j] = temp_Dvalmy[i][72] + (j - 72) * (cal_tmp) / 28;
					temp_setval[i][j - temp_Dvalmy[i][j]/10] = temp_Dvalmy[i][j];
				}		
			}
			
			for(ocnt = 1; ocnt < 5; ocnt++)
				temp_setval[i][100 + ocnt] = temp_Dvalmy[i][100];
			
			strcpy(pstring, read_buffer);
			strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
		}
		for(i = 35; i < 105; i++)
		{
			printf(" %d-", temp_Dvalmy[20][i]);
			printf("%d ", temp_setval[20][i]);
		}
	}
	else    // 只有保存一种温度值的修正值
	{
		rf_readstringfromeprofile("ReadtemperInfo", "temp_Input_Bias[30]", read_buffer, 1000, NULL, (const char *)ppath);
		for (i = 0; i < 30; i++)
		{		
			strcpy(pstring, read_buffer);
			par.temp_Input_Bias[i] = atoi(strtok_r(pstring, "|", &ptr));	
			strcpy(pstring, read_buffer);
			strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
		}
	}
	rf_readstringfromeprofile("ReadtemperInfo", "temp_Input_Filter[30]", read_buffer, 1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		par.temp_Input_Filter[i] = atoi(strtok_r(pstring, "|", &ptr));		
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}

	rf_readstringfromeprofile("ReadtemperInfo", "temp_HeatingPro_Band[30]", read_buffer, 1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		par.temp_HeatingPro_Band[i] = atoi(strtok_r(pstring, "|", &ptr));			
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}

	rf_readstringfromeprofile("ReadtemperInfo", "temp_HeatingInt_Time[30]", read_buffer, 1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		par.temp_HeatingInt_Time[i] = atoi(strtok_r(pstring, "|", &ptr));	
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}

	rf_readstringfromeprofile("ReadtemperInfo", "temp_HeatingDer_Time[30]", read_buffer, 1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		par.temp_HeatingDer_Time[i] = atoi(strtok_r(pstring, "|", &ptr));	
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}
	rf_readstringfromeprofile("ReadtemperInfo", "temp_Heatingctl_Time[30]", read_buffer, 1000, NULL, (const char *)ppath);
	for (i = 0; i < 30; i++)
	{
		strcpy(pstring, read_buffer);
		par.temp_Heatingctl_Time[i] = atoi(strtok_r(pstring, "|", &ptr));	
		strcpy(pstring, read_buffer);
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstring, "|", &ptr)) + 1]);
	}

	free(pstring);
	
	printf("\n ReadConfigInfo finished \n");
}


void mb_readrunparainfo(void)
{
	char a = 0;
	door_open_action = rf_readintfromeprofile("ReadConfigInfo", "DOOR_OPEN_ACTION", -1, (const char *)MeasureConfigPath);
	printf("DOOR_OPEN_ACTION = %d\n",door_open_action);
	/***感应方式
	nor_wast_barrel = rf_readintfromeprofile("ReadConfigInfo", "NOR_WAST_BARREL", -1, (const char *)MeasureConfigPath);
	printf("NOR_WAST_BARREL = %d\n",nor_wast_barrel);
	*/
	a = rf_readintfromeprofile("ReadConfigInfo", "DNG_WAST_BARREL", -1, (const char *)MeasureConfigPath);
	nor_wast_barrel = a %2;
	dng_wast_barrel = a / 2;
	printf("NOR_WAST_BARREL= %d DNG_WAST_BARREL = %d\n", nor_wast_barrel, dng_wast_barrel);
	tem_over_load = rf_readintfromeprofile("ReadConfigInfo", "TEM_OVER_LOAD", -1, (const char *)MeasureConfigPath);
	printf("TEM_OVER_LOAD = %d\n", tem_over_load);
	tem_limit = rf_readintfromeprofile("ReadConfigInfo", "TEM_LIMIT", -1, (const char *)MeasureConfigPath);
	
	tem_limit *= 10;
	printf("TEM_LIMIT = %d\n", tem_limit);
}


int mb_mainarmcmd2(const char * cmp_str)
{
	ejobid secend_cmd = cmp_str[0], last_secend_cmd = 0;
	sr_cmdstruct_t cmdframe;

	static  int X_coordinate = 0, Y_coordinate = 0, Z_coordinate = 0;
	static unsigned char station_num = 0;
	static unsigned int* ordArray_p = NULL;
	unsigned int (*postion_p)[2] = {0};
	static unsigned int* Z_ordArray_p = NULL;
	char str[20] = {0};
	static char save_ps = 0;
	unsigned int offset = 0;
	char data_tmp[10];
	unsigned int* dispense_step_p = dispense_stepA;
	
	const char ps_str[6][30] = {"ordArray_wash[3]", "ordArray_reagent[4][3]",
								"ordArray_plate[9][3]", "ordArray_OW[3]", "shelf_check[2][3]", "LIQUID_SPEED"};  

	printf("cmp_str[1] = %d ordArray_plate1[][]=%d\n", cmp_str[1],ordArray_plate1[0][0]);
	printf("dispense_step_p = %d %d\n", dispense_step_p[1],dispense_step_p[2]);

	switch (secend_cmd)
	{
		case SHELF:
		{
			if (cmp_str[1]%30 < 10)
				Z_ordArray_p  = &dispense_stepA[cmp_str[1]/30];
			else if (cmp_str[1]%30 < 20)
				Z_ordArray_p  = &dispense_stepB[cmp_str[1]/30];
			else if (cmp_str[1]%30 < 30)
				Z_ordArray_p  = &dispense_stepC[cmp_str[1]/30];

			cmdframe.srdevaddr = ARM_ADDR;	
		    sprintf(cmdframe.cmdbuf, "ZI");
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframe.cmdbuf, 
			// 		cmdframe.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n", cmdframe.srdevaddr );
			// else
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n", cmdframe.srdevaddr );
			// sb_waitingframeaswer(&cmdframe);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);

			if (cmp_str[1] < 30)		
				postion_p = ordArray_plate1;
			else if (cmp_str[1] < 60)		
				postion_p = ordArray_plate2;
			else if (cmp_str[1] < 90)		
				postion_p = ordArray_plate3;

			ordArray_p = &postion_p[(uint8_t)(cmp_str[1])][0];	//save the pointer for MIAN_SAVE cmd

			// printf("befor ordArray_p=%d\n",ordArray_p);

			sprintf(cmdframe.cmdbuf, "PA %d %d %d", postion_p[cmp_str[1]%30][0],
					postion_p[cmp_str[1]%30][1] + 2082, 0);
			X_coordinate = postion_p[cmp_str[1]%30][0];	//save the last coordinate 
			Y_coordinate = postion_p[cmp_str[1]%30][1] + 2082;
			Z_coordinate = *Z_ordArray_p;
			station_num = cmp_str[1] / 10;

			printf("original X=%d Y=%d Z=%d\n", X_coordinate, Y_coordinate, Z_coordinate);
			
			save_ps = 2;

			#ifdef USER_DEBUF_INFO
			printf("[SHELF] \r\n");
			#endif

			break;
		}
		case REAGENT_STAION:
		{
			cmdframe.srdevaddr = ARM_ADDR;	
			sprintf(cmdframe.cmdbuf, "ZI");
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframe.cmdbuf, 
			// 		cmdframe.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n", cmdframe.srdevaddr );
			// else
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n", cmdframe.srdevaddr );
			// sb_waitingframeaswer(&cmdframe);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);
			
			Z_ordArray_p = &liquid_zmax;
			ordArray_p = &ordArray_reagent[(uint8_t)(cmp_str[1])][0];
			sprintf(cmdframe.cmdbuf, "PA %d %d %d", ordArray_reagent[(uint8_t)(cmp_str[1])][0],
					ordArray_reagent[(uint8_t)(cmp_str[1])][1], 0);
			X_coordinate = ordArray_reagent[(uint8_t)(cmp_str[1])][0];	//save the last coordinate 
			Y_coordinate = ordArray_reagent[(uint8_t)(cmp_str[1])][1];
			Z_coordinate = *Z_ordArray_p;
			station_num = (uint8_t)(cmp_str[1] / 9);
			save_ps = 1;

			#ifdef USER_DEBUF_INFO
			printf("[REAGENT_STAION:] \r\n");
			#endif

			break;
		}
		case MIX_STATION:
		{
			break;
			Z_ordArray_p = &DAB_zmax;
			ordArray_p = &ordArray_mixed_DAB[(uint8_t)(cmp_str[1])][0];
			sprintf(cmdframe.cmdbuf, "PA %d %d %d", ordArray_mixed_DAB[(uint8_t)(cmp_str[1])][0],
					ordArray_mixed_DAB[(uint8_t)(cmp_str[1])][1], 0);
			X_coordinate = ordArray_mixed_DAB[(uint8_t)(cmp_str[1])][0];	//save the last coordinate 
			Y_coordinate = ordArray_mixed_DAB[(uint8_t)(cmp_str[1])][1];
			Z_coordinate = *Z_ordArray_p;
			station_num = (uint8_t)(cmp_str[1]);

			#ifdef USER_DEBUF_INFO
			printf("[MIX_STATION:] mix_x[%d], mix_y[%d], mix_z[%d]. \r\n", ordArray_mixed_DAB[(uint8_t)(cmp_str[1])][0],
					ordArray_mixed_DAB[(uint8_t)(cmp_str[1])][1]);
			#endif

			break;
		}
		case CLEAR_STATION:
		{
			cmdframe.srdevaddr = ARM_ADDR;	
			sprintf(cmdframe.cmdbuf, "ZI");
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframe.cmdbuf, 
			// cmdframe.srdevaddr , 1, 0, 1) > 0)
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);
			
			Z_ordArray_p = &zwash_step4;//take NO.4 clearstation as standard
			ordArray_p = &ordArray_wash[WASH4][0];
			sprintf(cmdframe.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0],
					ordArray_wash[WASH4][1], 0);
			X_coordinate = ordArray_wash[WASH4][0];	//save the last coordinate 
			Y_coordinate = ordArray_wash[WASH4][1];
			Z_coordinate = *Z_ordArray_p;
			station_num = 0;
			save_ps = 0;

			#ifdef USER_DEBUF_INFO
			printf("[CLEAR_STATION:] clear_x[%d], clear_y[%d], clear_z[%d]. \r\n", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1],
					ordArray_wash[WASH4][2]);
			#endif

			break;
		}		
		case MAIN_X:
		{
			if (cmp_str[1] == 0)
				X_coordinate -= MAINTIAN_MAIN_INCREMENT;
			else
				X_coordinate += MAINTIAN_MAIN_INCREMENT;
			sprintf(cmdframe.cmdbuf, "XA %d", X_coordinate);
			break;
		}
		case MAIN_Y:
		{
			if (cmp_str[1] == 0)
				Y_coordinate -= MAINTIAN_MAIN_INCREMENT;
			else
				Y_coordinate += MAINTIAN_MAIN_INCREMENT;
			
			sprintf(cmdframe.cmdbuf, "YA %d", Y_coordinate);
			break;
		}
		case MAIN_Z:
		{
			if (cmp_str[1] == 0)
				Z_coordinate -= MAINTIAN_MAIN_INCREMENT;
			else
				Z_coordinate += MAINTIAN_MAIN_INCREMENT;

			sprintf(cmdframe.cmdbuf, "ZA %d", Z_coordinate);

		#ifdef USER_DEBUF_INFO
		printf("[MAIN_Z:] \r\n");

		#endif
			break;
		}
		case RELEASE:
		{
			cmdframe.srdevaddr = ARM_ADDR;	
			sprintf(cmdframe.cmdbuf, "ZD");
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);

		#ifdef USER_DEBUF_INFO
		printf("[RELEASE] \r\n");

		#endif
			break;
		}
		case MAIN_INIT:
		{
			sprintf(cmdframe.cmdbuf, "PI");
			//	station_num = 0;

			#ifdef USER_DEBUF_INFO
			printf("[MAIN_INIT] \r\n");
			#endif

			break;
		}		
		case INIT_SET:
		{
			X_coordinate = cmp_str[1];
			Y_coordinate = cmp_str[2];
			cmdframe.srdevaddr = ARM_ADDR;	
			sprintf(cmdframe.cmdbuf, "OX %d", X_coordinate);
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);

			sprintf(cmdframe.cmdbuf, "OY %d", Y_coordinate);
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);

			sprintf(cmdframe.cmdbuf, "OZ %d", Z_OFFSET);
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);
			
			sprintf(cmdframe.cmdbuf, "OW");
			printf("cmdbuf=%s\n", cmdframe.cmdbuf);
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);

			save_ps = 3;
			station_num = 0;
			sprintf(str, "%d %d %d", X_coordinate, Y_coordinate, 0);
			rf_writeintforprofile("ReadCoordinateInfo", &ps_str[(uint8_t)save_ps][0], str, (const char *)MeasureConfigPath, station_num);
			#ifdef USER_DEBUF_INFO
			printf("[MAIN_INIT]%d %d %d\r\n", X_coordinate, Y_coordinate, 0 );
			#endif
			
			return 0;			
		}
		case MIAN_SAVE: 
		{
			printf("last_secend_cmd=%d, save_ps=%d\n", last_secend_cmd, save_ps);
		
			 if( last_secend_cmd == MAIN_INIT)
				break;

			 if (save_ps == 5)
			{
				sprintf(data_tmp, "%d",X_coordinate);	
				rf_writeintforprofile("mb_readconfparatinfo", "LIQUID_SPEED", data_tmp, (const char *)MeasureConfigPath, 0);			
				scanoffset = X_coordinate - 350;	
				break;
			}
	 
			if ((save_ps == 0 && ordArray_p == 0) || (save_ps == 3))	//直接点保存
				break;

			if (save_ps == 2)
				Y_coordinate -= 2082;
			
			printf("MIAN_SAVE station_num=%d\n",station_num);

			if (X_coordinate < 0) X_coordinate = 0;			
			if (Y_coordinate < 0) Y_coordinate = 0;			
			if (Z_coordinate < 0) Z_coordinate = 0;			
			
			sprintf(str, "%d %d %d", X_coordinate, Y_coordinate, Z_coordinate);
			rf_writeintforprofile("ReadCoordinateInfo", &ps_str[(uint8_t)save_ps][0], str, (const char *)MeasureConfigPath, station_num);	

			#ifdef USER_DEBUF_INFO
			printf("[MAIN_SAVE] \r\n");
			#endif		

			break;
		}
		case SCANER:
		{
			sc_getscode(0XFF, FALSE);

			#ifdef USER_DEBUF_INFO
				printf("[SCANER] \r\n");
			#endif

			break;
		}
		case SHELF_CHECK:
		{		
			printf("cmp_str[1]=%d\n", cmp_str[1]);
			cmdframe.srdevaddr = ARM_ADDR;	
			sprintf(cmdframe.cmdbuf, "ZI");
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);

			printf("$%d %d %d$", shelf_check[0][0], shelf_check[0][1], shelf_check[0][2]);
			printf("$%d %d %d$", shelf_check[1][0], shelf_check[1][1], shelf_check[1][2]);

			Z_ordArray_p = (uint32_t*)(&shelf_check[(uint8_t)(cmp_str[1])][2]);
			ordArray_p = (uint32_t*)(&shelf_check[(uint8_t)(cmp_str[1])][0]);
			sprintf(cmdframe.cmdbuf, "PA %d %d %d", shelf_check[(uint8_t)(cmp_str[1])][0],
					shelf_check[(uint8_t)(cmp_str[1])][1], shelf_check[(uint8_t)(cmp_str[1])][2]);
			X_coordinate = shelf_check[(uint8_t)(cmp_str[1])][0];	//save the last coordinate 
			Y_coordinate = shelf_check[(uint8_t)(cmp_str[1])][1];
			Z_coordinate = *Z_ordArray_p;
			station_num = (uint8_t)(cmp_str[1]);
			save_ps = 4;

			#ifdef USER_DEBUF_INFO
			printf("[SHELF_CHECK:] \r\n");
			#endif		

			break;
		}
		case SHELF_SCAN:
		{
			if (new_scaner)
				offset = SCAN_OFFSET;
				
			cmdframe.srdevaddr = ARM_ADDR;	
			sprintf(cmdframe.cmdbuf, "ZI");
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);

			X_coordinate = 350 + scanoffset ;
			save_ps = 5;
			sprintf(cmdframe.cmdbuf, "PA %d %d %d", X_coordinate, ordArray_plate1[0][1] + 2082, MOV_ZH);

			#ifdef USER_DEBUF_INFO
			printf("[SHELF_SCAN:] \r\n");
			#endif		

			break;
		}

		case Z_DOWN:
		{
			sprintf(cmdframe.cmdbuf, "ZA %d", Z_coordinate);

			#ifdef USER_DEBUF_INFO
			printf("[Z_DOWN:] \r\n");
			#endif	

			break;
		}
		
		default:break;
	}

	last_secend_cmd = secend_cmd;

	if (secend_cmd != MIAN_SAVE && secend_cmd != RELEASE && secend_cmd != SCANER )
	{
		cmdframe.srdevaddr = ARM_ADDR;	
		printf("cmdbuf=%s\n", cmdframe.cmdbuf);

		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &cmdframe, 1, 0, 1);
	}	
	return 0;
}


int mb_actionmainarm(const char * cmp_str)
{
	eselfact secend_cmd = cmp_str[0];
	static  short Y_coordinate; //, X_coordinate, Z_coordinate;
	static unsigned char station_num = 0, lastarmsel=0;
	static unsigned int* ordArray_p = NULL;
	char str[20] = {0};
	
	printf("cmp_str[0]=%d cmp_str[1]=%d\n", cmp_str[0], cmp_str[1]);

	//部件校准自控臂必须初始化完成
	if(cmp_str[0] <= SELFC)
	{
		if(cmp_str[0] == 0)
		{
			ordArray_p = &arm_motor[(uint8_t)(cmp_str[1])];
			Y_coordinate = arm_motor[(uint8_t)(cmp_str[1])];
		}
		else if(cmp_str[0] == 1)
		{
			ordArray_p = &arm_motor2[(uint8_t)(cmp_str[1])];
			Y_coordinate = arm_motor2[(uint8_t)(cmp_str[1])];
		}
		else if(cmp_str[0] == 2)
		{
			ordArray_p = &arm_motor2[(uint8_t)(cmp_str[1])];
			Y_coordinate = arm_motor2[(uint8_t)(cmp_str[1])];
		}

		station_num = cmp_str[1] - 1;//保存的是后3个位置
		lastarmsel = cmp_str[0];
		
		ArmCabinCmdList[(uint8_t)(cmp_str[0])].dispense_pos = 20;//固定值
		ArmCabinCmdList[(uint8_t)(cmp_str[0])].pos_datalo = Y_coordinate&0XFF;
		ArmCabinCmdList[(uint8_t)(cmp_str[0])].pos_datahi = Y_coordinate>>8;
		ArmCabinCmdList[(uint8_t)(cmp_str[0])].cmd = ArmCabinArm;	
	}
	else if(cmp_str[0] <= SELFCY)
	{
		if(cmp_str[1] == 1)
			Y_coordinate += 10;  
		else
			Y_coordinate -= 10;  
		
		ArmCabinCmdList[(uint8_t)(cmp_str[0] - 3)].dispense_pos = 20;//固定值
		ArmCabinCmdList[(uint8_t)(cmp_str[0] - 3)].pos_datalo = Y_coordinate&0XFF;
		ArmCabinCmdList[(uint8_t)(cmp_str[0] - 3)].pos_datahi = Y_coordinate>>8;
		ArmCabinCmdList[(uint8_t)(cmp_str[0] - 3)].cmd = ArmCabinArm;
	}
	else if(SELF_SAVE == secend_cmd)
	{
		*ordArray_p = Y_coordinate;
		sprintf(str, "%d", Y_coordinate);
		if(lastarmsel == SELFA)
			rf_writeintforprofile("ReadMiniboardInfo", "arm_motor1", str, (const char *)MeasureConfigPath, station_num);
		else if(lastarmsel == SELFB)
			rf_writeintforprofile("ReadMiniboardInfo", "arm_motor2", str, (const char *)MeasureConfigPath, station_num);
		else if(lastarmsel == SELFC)
			rf_writeintforprofile("ReadMiniboardInfo", "arm_motor3", str, (const char *)MeasureConfigPath, station_num);
	}
			
	return 0;
}


int mb_actionstrech(const char * cmp_str)
{
	estrechid secend_cmd = cmp_str[0];
	stminibd_sendpacket em_minicmd;
	// unsigned int value;;
	// char str[20] = {0};
	// void* lock = NULL;
	// stminibd_cmdlist* p_head = NULL;
	// static char save_ps = 0;
	// const char ps_str[3][30] = {"load_motor1","load_motor2","load_motor3"};  
	// sr_cmdstruct_t CommandElemt;

	memset(&em_minicmd,0,sizeof(em_minicmd));

	if (cmp_str[1] < 1)
	{	
		em_minicmd.cmd = RELOAD;
		em_minicmd.minicmd_buffer[3] = (unsigned char)RELOAD;
		em_minicmd.minicmd_num=2;		
	}
	else
	{	
		em_minicmd.cmd = STRETCH;
		em_minicmd.minicmd_buffer[3] = (unsigned char)STRETCH;	
		em_minicmd.minicmd_buffer[5] = cmp_str[1] - 1;	
		em_minicmd.minicmd_num=3;
	}

	em_minicmd.minicmd_buffer[4] = secend_cmd + 1;
	
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,em_minicmd);
	pthread_mutex_unlock(&mutex_mlock);
				
	return 0;
}


int mb_actionpump(const char * cmp_str)
{
	PUMP_T secend_cmd = cmp_str[0];
	sr_cmdstruct_t CommandElemt;
	char str[20] = {0};
	stminibd_sendpacket em_minicmd;
	 int value = 0;
	static int lastvalue = 0;
	static char last_portsel=0;
	static PUMP_T last_secend_cmd = 0;
	// int spd = 0;
	char sendbuf[5] = {0};
	memset(&em_minicmd,0,sizeof(em_minicmd));
	em_minicmd.cmd = PUMP_TEST;
	em_minicmd.minicmd_buffer[3] = (unsigned char)PUMP_TEST;
	em_minicmd.minicmd_num=11;

	printf("cmp_str[0]=%d cmp_str[2]=%d cmp_str[3]=%d cmp_str[4]=%d cmp_str[5]=%d",cmp_str[0], cmp_str[2],cmp_str[3],cmp_str[4],cmp_str[5]);
	
	memcpy(&value, &cmp_str[2], 4);
	if (SELF_PUMP)
	{
		value = value * SYFSTEP / 1000;
		if(secend_cmd == PUMPA)
			CutoffSpeedChange((short)SYSCUTOFF_SPEED);
	}
	else
	{
		value *= 6;  
	}
	#if(S600_MANUAL_PUMPVAL == 1)
		value *= 100;
		value /= S600_MANUPUMP_VAL;
	#endif

	if(secend_cmd == PUMPA)
	{
		CommandElemt.srdevaddr = PUMP_ADDR;

		if (SELF_PUMP)
			sprintf(str, "S%dO%dA%dR", SYSPEED,cmp_str[1], value);
		else
			sprintf(str, "S%dO%dA%dR", SYSPEED,cmp_str[1] + 1, value);

		strcpy(CommandElemt.cmdbuf, str);

		printf("cmdbuf=%s\n", CommandElemt.cmdbuf);

		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);		
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	else if(secend_cmd <= PUMPD)
	{
		printf("last_secend_cmd=%d secend_cmd=%d last_portsel=%d cmp_str[1]=%d ",
				last_secend_cmd, secend_cmd, last_portsel,cmp_str[1]);

		if (!(last_secend_cmd == secend_cmd && last_portsel == cmp_str[1]))
		{
			ArmCabinCmdList[cmp_str[0] -1].reagent_sel = cmp_str[1];
			ArmCabinCmdList[cmp_str[0] -1].cmd = ArmCabinPumpValve;
		}	
		else//一次肯定控制阀或者控制注射器，二选一
		{
			if (!(last_secend_cmd == secend_cmd && lastvalue == value))
			ArmCabinCmdList[cmp_str[0] -1].pos_datalo = value&0XFF;
			ArmCabinCmdList[cmp_str[0] -1].pos_datahi = value>>8;
			ArmCabinCmdList[cmp_str[0] -1].cmd = ArmCabinPumpSy;
		}
		sleep(1);
		while(!ArmCabinWork_finished[cmp_str[0] -1])
			usleep(100000);
		lastvalue = value;
		last_portsel = cmp_str[1];
		last_secend_cmd =  cmp_str[0];
	}

	else if(secend_cmd == PUMP_INITA) 
	{
		CommandElemt.srdevaddr = PUMP_ADDR;
		strcpy(CommandElemt.cmdbuf,"ZR");
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);		
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	else if(secend_cmd <= PUMP_INITD) 
	{
		ArmCabinCmdList[cmp_str[0] -PUMP_INITA - 1].reagent_sel = 20;
		ArmCabinCmdList[cmp_str[0] -PUMP_INITA - 1].cmd = ArmCabinPumpValve;
		sleep(1);
		while(!ArmCabinWork_finished[cmp_str[0] -PUMP_INITA - 1])
			usleep(100000);				
	}
	sendbuf[0] = 4;
	sendbuf[1] = 1;
	nt_sendpacketdata(UpdateSlaveState , sendbuf, 2);

	return 0;
}


int mb_actionother(const char * cmp_str)
{
	OTHER_T secend_cmd = cmp_str[0];
	int (*GPIO_work)(unsigned int);
	stminibd_sendpacket em_minicmd;
	// char tstchar[20] = {0};
	char str[20] = {0};

	printf("cmp_str[1]=%d\n", cmp_str[1]);
	
	if(secend_cmd == MAINTAIN_SETZERO)
	{
		sprintf(str, "%d", bigcabin_value_init[(uint8_t)(cmp_str[1])]);
		rf_writeintforprofile("mb_readconfparatinfo", "reagent_cabin_zero[6]", str, (const char *)MeasureConfigPath, cmp_str[1]);
		reagent_cabin_zero[(uint8_t)(cmp_str[1])] = bigcabin_value_init[(uint8_t)(cmp_str[1])];
	}
	else if(secend_cmd == MAINTAIN_SETFULL)
	{
		sprintf(str, "%d", bigcabin_value_init[((uint8_t)(cmp_str[1]))]);
		rf_writeintforprofile("mb_readconfparatinfo", "reagent_cabin_full[6]", str, (const char *)MeasureConfigPath, cmp_str[1]);
		reagent_cabin_full[(uint8_t)(cmp_str[1])] = (uint16_t)bigcabin_value_init[(uint8_t)(cmp_str[1])];
	}
	
	if (secend_cmd <= MAINTIAN_VP1 || secend_cmd == MAINTIAN_BEEP)
	{
		if (cmp_str[1] == 0)
			GPIO_work = GPIO_OutClear;
		else
			GPIO_work = GPIO_OutSet;		

		if (secend_cmd == MAINTIAN_V1) GPIO_work(V1);
		else if (secend_cmd == MAINTIAN_V2) GPIO_work(V2);
		else if (secend_cmd == MAINTIAN_V3) GPIO_work(V3);
		else if (secend_cmd == MAINTIAN_V4) GPIO_work(V4);
		else if (secend_cmd == MAINTIAN_V5) GPIO_work(V5);
		else if (secend_cmd == MAINTIAN_V6) GPIO_work(V6);
		else if (secend_cmd == MAINTIAN_V7) GPIO_work(V7);
		else if (secend_cmd == MAINTIAN_V8) GPIO_work(V8);
		else if (secend_cmd == MAINTIAN_V9) GPIO_work(V9);
		else if (secend_cmd == MAINTIAN_VP1) GPIO_work(VP1);
		else if (secend_cmd == MAINTIAN_BEEP) GPIO_work(BEEPER);		
	}
	else if (secend_cmd <= MAINTIAN_FANC)
	{
		memset(&em_minicmd,0,sizeof(em_minicmd));
		em_minicmd.cmd = MAINTAIN_FAN_WORK;
		em_minicmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
		if (cmp_str[1] == 1)
		em_minicmd.minicmd_buffer[4] = 0;// ON
		else
		em_minicmd.minicmd_buffer[4] = 3; //OFF
		
		em_minicmd.minicmd_num=2;
		if (secend_cmd == MAINTIAN_FANA)
		{
			em_minicmd.minicmd_buffer[4] += 1;
		}
		else if (secend_cmd == MAINTIAN_FANB)
		{
			em_minicmd.minicmd_buffer[4] += 2;
		}
		else if (secend_cmd == MAINTIAN_FANC)
		{
			em_minicmd.minicmd_buffer[4] += 3;
		}
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head, em_minicmd);	
		pthread_mutex_unlock(&mutex_mlock);
	}
	
	return 0;
}


int mb_actionotemper(const char * cmp_str)
{
	stTemper_cmdframeunion temcmd;
	unsigned int plate_num, j;
	int cal_tmp = 0;
	// unsigned int tem_cmd_counter=0;
	emTemp_chopcmd cmd_type;
	unsigned int interval=0;
	char str[20] = {0};
	static uint8_t save_ps = 0;
	const char ps_str[8][30] = {"temp_SV_High[30]","temp_MV[30]","temp_Input_Bias[30]","temp_Input_Filter[30]",
								"temp_HeatingPro_Band[30]","temp_HeatingInt_Time[30]","temp_HeatingDer_Time[30]",
								"temp_Heatingctl_Time[30]"};  
	short wtemvalue = 0,rtemvalue;
	emTermper_cmdtype cmdmy = 0;
	
	plate_num = cmp_str[0];

	cmd_type = cmp_str[1];
	if (CH1_SV_High == cmd_type)
		save_ps = 0;
	else if (CH1_MV_High == cmd_type)
		save_ps = 1;
	else if (CH1_Input_Bias == cmd_type)
		save_ps = 2;
	else if (CH1_Input_Filter == cmd_type)
		save_ps = 3;
	else if (CH1_HeatingPro_Band == cmd_type)
		save_ps = 4;
	else if (CH1_HeatingInt_Time == cmd_type)
		save_ps = 5;
	else if (CH1_HeatingDer_Time == cmd_type)
		save_ps = 6;
	else if (CH1_Heatingctl_Time == cmd_type)
		save_ps = 7;
				//自己温控器校准方式，默认35°温差为0，校准72和100°输入温差值，35到72，72到100做线性插补
	if (new_temper)
	{
		strcpy( (char*)(&ps_str[2][0]),"temp_Input_Bias[60]");
		wtemvalue = *((int*)&cmp_str[2]);
		
		switch(cmd_type)
		{
			case CH1_SV:
			{
				cmdmy = SET;
				break;
			}
			case CH1_Present_Value:
			{
				cmdmy = CUR1;
				break;
			}
			case CH1_HeatingPro_Band:
			{
				cmdmy = KP;
				break;
			}
			case CH1_HeatingInt_Time:
			{
				cmdmy = TI;
				break;
			}
			case CH1_HeatingDer_Time:
			{
				cmdmy = TD;
				break;
			}
			case CH1_Input_Bias:
			{
				cmdmy = BIAS;
				break;
			}
			case CH1_Heatingctl_Time:
			{
				cmdmy = Tsam;
				break;
			}
			 default:break;
		}

		printf("cmd_type = %d value = %d, last_tmper=%d\n",cmd_type,wtemvalue,last_tmper[plate_num]);

		if ((cmd_type != CH1_SV))
		{				
			sprintf(str, "%d",wtemvalue);
			
				if(cmd_type == CH1_Input_Bias)
				{
					if( last_tmper[plate_num] == 720)
					{
						temp_Dvalmy[plate_num][72] = (int)wtemvalue;
						for(j = 36;j < 72;j++)
						{
							if(temp_Dvalmy[plate_num][72] < 0)
							{
								temp_Dvalmy[plate_num][j] = (~temp_Dvalmy[plate_num][72] + 1) *(j - 35) / 37;
								temp_Dvalmy[plate_num][j] = ~temp_Dvalmy[plate_num][j] + 1;
							}
							else
								temp_Dvalmy[plate_num][j] = temp_Dvalmy[plate_num][72] *(j - 35) / 37;
							
							printf(" %d ",temp_Dvalmy[plate_num][j]);
						}
						cal_tmp = temp_Dvalmy[plate_num][100] - temp_Dvalmy[plate_num][72];
						
						printf("73cal_tmp = %d\n", cal_tmp);

						for(j = 73;j < 100;j++)
						{		
							if(cal_tmp < 0)
							{
								temp_Dvalmy[plate_num][j] =  (j - 72) * (~cal_tmp + 1) / 28;
								temp_Dvalmy[plate_num][j] = ~temp_Dvalmy[plate_num][j] + 1 + temp_Dvalmy[plate_num][72];
							}
							else
								temp_Dvalmy[plate_num][j] = temp_Dvalmy[plate_num][72] + (j - 72) * (cal_tmp) / 28;
								
							printf(" %d ",temp_Dvalmy[plate_num][j]);
						}
						printf("temp_Dvalmy[%d][72]=%d\n",plate_num,  temp_Dvalmy[plate_num][72]);		
						printf("temp_Dvalmy[plate_num][100]=%d\n",  temp_Dvalmy[plate_num][100]);

						tp_sendtemperframe(plate_num, SET, 1,720 + temp_Dvalmy[plate_num][72],&rtemvalue);
						rf_writeintforprofile("ReadtemperInfo", &ps_str[(uint8_t)save_ps][0], str, (const char *)MeasureConfigPath,plate_num);
					}
					else if( last_tmper[plate_num] == 1000)
					{
						temp_Dvalmy[plate_num][100] = wtemvalue;
						for(j = 36;j < 72;j++)
						{
							if(temp_Dvalmy[plate_num][72] < 0)
							{
								temp_Dvalmy[plate_num][j] = (~temp_Dvalmy[plate_num][72] + 1) *(j - 35) / 37;
								temp_Dvalmy[plate_num][j] = ~temp_Dvalmy[plate_num][j] + 1;
							}
							else
								temp_Dvalmy[plate_num][j] = temp_Dvalmy[plate_num][72] *(j - 35) / 37;
							printf(" %d ",temp_Dvalmy[plate_num][j]);
						}
						cal_tmp = temp_Dvalmy[plate_num][100] - temp_Dvalmy[plate_num][72];
						printf("100cal_tmp = %d\n", cal_tmp);
						for(j = 73;j < 100;j++)
						{
							if(cal_tmp < 0)
							{
								temp_Dvalmy[plate_num][j] = (j - 72) * (~cal_tmp + 1) / 28;
								temp_Dvalmy[plate_num][j] = ~temp_Dvalmy[plate_num][j] + 1 + temp_Dvalmy[plate_num][72];
							}
							else
								temp_Dvalmy[plate_num][j] = temp_Dvalmy[plate_num][72]+(j - 72) * (cal_tmp) / 28;
							printf(" %d ",temp_Dvalmy[plate_num][j]);
						}
						printf("temp_Dvalmy[%d][72]=%d\n",plate_num,  temp_Dvalmy[plate_num][72]);	
						printf("temp_Dvalmy[plate_num][100]=%d\n",  temp_Dvalmy[plate_num][100]);

						tp_sendtemperframe(plate_num, SET, 1,1000 + temp_Dvalmy[plate_num][100],&rtemvalue);
						rf_writeintforprofile("ReadtemperInfo", &ps_str[save_ps][0], str, (const char *)MeasureConfigPath,plate_num+30);
					}	
				}
				else
					rf_writeintforprofile("ReadtemperInfo", &ps_str[(uint8_t)save_ps][0], str, (const char *)MeasureConfigPath,plate_num);
		}
		else if (cmdmy == SET)
		{
			last_tmper[plate_num] = wtemvalue;
			wtemvalue += temp_Dvalmy[plate_num][wtemvalue/10];
			printf("temp_Dvalmy[plate_num][%d]=%d\n",wtemvalue,temp_Dvalmy[plate_num][wtemvalue/10]);
		}
		if(cmd_type != CH1_Input_Bias)
		{
			tp_sendtemperframe(plate_num, cmdmy, 1,wtemvalue,&rtemvalue);//娴嬭瘯鏈熼棿杩欓噷鍙戦??
		}
		return 0;
		temcmd.cmdmy = cmdmy;
		temcmd.plate_num = plate_num;
		temcmd.wdata = wtemvalue;
		pthread_mutex_lock(&mutex_miantianlock);
		tc_testpacketcmdtemp(temcmd);
		pthread_mutex_unlock(&mutex_miantianlock);		

		return 0;
	}
	temcmd.temvalue = *((unsigned int*)&cmp_str[2]);

	if (plate_num < 4)
		temcmd.devaddr = 1;
	else if (plate_num < 8)
		temcmd.devaddr = 2;
	else if (plate_num < 12)
		temcmd.devaddr = 3;
	else if (plate_num < 16)
		temcmd.devaddr = 4;
	else if (plate_num < 20)
		temcmd.devaddr = 5;
	else if (plate_num < 24)
		temcmd.devaddr = 6;
	else if (plate_num < 28)
		temcmd.devaddr = 7;
	else if (plate_num < 32)
		temcmd.devaddr = 8;

	if ((cmd_type == CH1_RUN_STOP) || (cmd_type == CH1_Multi_SV_No) || (cmd_type == CH1_SV))
		interval = 3;
	else if ((cmd_type == CH1_AUTO_Tuning) || (cmd_type == CH1__MV_Low) || (cmd_type == CH1_MV_High)
			|| (cmd_type == CH1_Ramp_Up) || (cmd_type == CH1_Ramp_Down) || (cmd_type == CH1_Ramp_Unit)
			|| (cmd_type == CH1_HeatingPro_Band)|| (cmd_type == CH1_HeatingInt_Time)|| (cmd_type == CH1_HeatingDer_Time))
		interval = 9;
	else if ((cmd_type == CH1_Present_Value))
		interval = 1;
	else if ((cmd_type == CH1_Input_Type) || (cmd_type == CH1_Input_Bias)|| (cmd_type == CH1_Input_Filter)
			|| (cmd_type == CH1_SV_Low) || (cmd_type == CH1_SV_High) || (cmd_type == CH1_Tuning_Type) || (cmd_type == CH1_Heatingctl_Time))
		interval = 7;

	switch (plate_num%4)
	{
		case 0:
			temcmd.chopcmd = cmd_type ;
			break;
		case 1:
			temcmd.chopcmd = cmd_type + interval;
			break;
		case 2:
			temcmd.chopcmd = cmd_type + interval * 2;
			break;
		case 3:
			temcmd.chopcmd = cmd_type + interval * 3;
			break;
		default:break;
	}

	printf("cmd_type = %d value = %d\n",cmd_type,temcmd.temvalue);
	if ((cmd_type != CH1_SV) && (cmd_type != CH1_Input_Type))
	{				
		sprintf(str, "%d", temcmd.temvalue);
		
		rf_writeintforprofile("ReadtemperInfo", &ps_str[(uint8_t)save_ps][0], str, (const char *)MeasureConfigPath,plate_num);
	}
	else if (temcmd.temvalue >= 200)
	{
		temcmd.temvalue +=  temp_goalval[temcmd.temvalue / 10];//加速温度差值  
	}

	if (cmd_type == CH1_Input_Bias)//
	{
		printf("temcmd.temvalue=%d\n\n\n",temcmd.temvalue);
		//	return 0;
	}

	if (cmd_type == CH1_SV_High)
	{
		temcmd.temvalue += 300;	//	CH1_SV_High 的值
	}

	pthread_mutex_lock(&mutex_miantianlock);
	tc_testpacketcmdtemp(temcmd);
	pthread_mutex_unlock(&mutex_miantianlock);
			
	if (cmd_type == CH1_Input_Type && plate_num == 29)//第30个传感器配置时，将后两个配置
	{
		temcmd.chopcmd = CH3_Input_Type;
		pthread_mutex_lock(&mutex_miantianlock);
		tc_testpacketcmdtemp(temcmd);
		pthread_mutex_unlock(&mutex_miantianlock);	
		temcmd.chopcmd = CH4_Input_Type;
		pthread_mutex_lock(&mutex_miantianlock);
		tc_testpacketcmdtemp(temcmd);
		pthread_mutex_unlock(&mutex_miantianlock);
	}
			
	return 0;
}


int mb_setdefaulconfpara(const char * cmp_str)
{
	sr_cmdstruct_t CommandElemt;
	stTemper_cmdframeunion temcmd;
	int i = 0;
	char str[20] = {0};

	mb_readconfparatinfo(1);
	
	if(*cmp_str == 1)//mainarm
	{
		CommandElemt.srdevaddr = ARM_ADDR;	
		sprintf(CommandElemt.cmdbuf, "OX %d", par.ordArray_OW[0]);
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		sprintf(CommandElemt.cmdbuf, "OY %d", par.ordArray_OW[1]);
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
		sprintf(CommandElemt.cmdbuf, "OZ %d", par.ordArray_OW[2]);
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
		sprintf(CommandElemt.cmdbuf, "OW");

		printf("cmdbuf=%s\n", CommandElemt.cmdbuf);

		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
		sprintf(str, "%d %d %d",par.ordArray_OW[0], par.ordArray_OW[1], par.ordArray_OW[2]);
		rf_writeintforprofile("ReadCoordinateInfo", "ordArray_OW[3]", str, (const char *)MeasureConfigPath,0);	
			
		sprintf(str, "%d %d %d",par.ordArray_wash[0], par.ordArray_wash[1], par.ordArray_wash[2]);
		rf_writeintforprofile("ReadCoordinateInfo", "ordArray_wash[3]", str, (const char *)MeasureConfigPath,0);
		
		//	{"ordArray_wash[3]","ordArray_reagent[4][3]",
			//					"ordArray_plate[9][3]","ordArray_OW[3]","shelf_check[2][3]","LIQUID_SPEED"}		

		for(i = 0; i < 4;i++)
		{
			sprintf(str, "%d %d %d",par.ordArray_reagent[i][0], par.ordArray_reagent[i][1], par.ordArray_reagent[i][2]);
			rf_writeintforprofile("ReadCoordinateInfo", "ordArray_reagent[4][3]", str, (const char *)MeasureConfigPath,i);
		}
	
		for(i = 0; i < 3;i++)
		{
			sprintf(str, "%d %d %d",par.shelf_check[i][0], par.shelf_check[i][1], par.shelf_check[i][2]);
			rf_writeintforprofile("ReadCoordinateInfo", "shelf_check[2][3]", str, (const char *)MeasureConfigPath,i);
		}
		
		for(i = 0; i < 9;i++)
		{
			sprintf(str, "%d %d %d",par.ordArray_plate[i][0], par.ordArray_plate[i][1], par.ordArray_plate[i][2]);
			rf_writeintforprofile("ReadCoordinateInfo", "ordArray_plate[9][3]", str, (const char *)MeasureConfigPath,i);
		}
	}
	else if(*cmp_str == 2)//Selfarm
	{
		
	}
	else if(*cmp_str == 3)//Temp
	{
		printf("adgfetgh*******************");

		temcmd.devaddr = 9;//设置全部默认
		temcmd.chopcmd = CONFIG_TEMPER;
		pthread_mutex_lock(&mutex_miantianlock);
		tc_testpacketcmdtemp(temcmd);
		pthread_mutex_unlock(&mutex_miantianlock);
		while(!TEM_init_setting_finished)		
			sleep(1);
		TEM_init_setting_finished = FALSE;	
	}
	return 0;
}


void mb_menualwork(const char * cmp_string)
{
	// emianid cmd = cmp_string[0];
	static char last_cmd = 0XFF,last_sec = 0XFF;
	int (*maintianfun[7])(const char * ) = {mb_mainarmcmd2, mb_actionmainarm, mb_actionstrech, mb_actionpump, \
			mb_actionother, mb_actionotemper, mb_setdefaulconfpara};
	sr_cmdstruct_t CommandElemt;

	// assert(*cmp_string > 5);
	
	printf("last_cmd = %d last_sec=%d cmp_string[0]=%d,", last_cmd, last_sec, cmp_string[0]);

	if (last_cmd == MAIN_ARM && last_sec == Z_DOWN  &&  cmp_string[0] == STRECH)//防止撞针
	{
		CommandElemt.srdevaddr = ARM_ADDR;	
		sprintf(CommandElemt.cmdbuf, "ZI");
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_mainarmcmd2]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	maintianfun[(uint8_t)cmp_string[0]](cmp_string + 1);
	last_cmd = cmp_string[0];
	last_sec =  cmp_string[1];
}



void mb_reagentchecking(unsigned char plate_num)
{
	sr_cmdstruct_t CommandElemt;
	int i=0,ZX_res = 0;
	CommandElemt.srdevaddr = ARM_ADDR;
	unsigned char TCP_Buffer[50] = {0};
	unsigned int dwPinState;
	bool washall = false;
		
	assert(plate_num < 36);
	pthread_mutex_lock(&mutex_mianarmlock);
	memset(ReportArrayData,0,sizeof(ReportArrayData));

	dwPinState = reagent_check[plate_num / 9].sen;
	reagent_check[plate_num / 9].NEED_CHECK = true;
	GPIO_PinState(&dwPinState);
	if ((dwPinState & reagent_check[plate_num / 9].sen) == 1)//已经拿掉
	{
		if (SHELF_LOCK_WAY_NEW)
			GPIO_OutClear(reagent_check[plate_num / 9].lock);//锁住进行扫描
		else
			GPIO_OutSet(reagent_check[plate_num / 9].lock);//锁住进行扫描
				
		reagent_check[plate_num / 9].NEED_CHECK = false;
		if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
		return;
	}	

	if (SHELF_LOCK_WAY_NEW)
		GPIO_OutSet(reagent_check[plate_num / 9].lock);  //锁住进行扫描
	else
		GPIO_OutClear(reagent_check[plate_num / 9].lock);  //锁住进行扫描
	
	mb_monitdoorstate();

	if (last_cabin_reagent != REAGENT_WATER || last_cabin_reagent != REAGENT_WASH)
		mb_procwashprobe(REAGENT_WATER);
	else
	{	
		tp_washchange(true);
		if (last_cabin_reagent == REAGENT_WATER)
			mb_probewash_c(WATERPORT);
		else if (last_cabin_reagent == REAGENT_WASH )
			mb_probewash_c(WASHPORT);
		tp_washchange(false);
	}

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	usleep(ASPIRATE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d",ordArray_reagent[plate_num][0], ordArray_reagent[plate_num][1], MOV_ZH);//到达玻片位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sprintf(CommandElemt.cmdbuf, "ZA %d", 350 - OFFSET_STEP + SHELF_OFFSET);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	
	usleep(300000);			
			
	flg_getedserailarr = true;
	sprintf(CommandElemt.cmdbuf, "ZX 0 0 %d", LIQUID_ZMAX);////	sprintf(CommandElemt.cmdbuf, "ZA 500");
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	//if ((ZX_res = sb_waitingframeaswer(&CommandElemt)) < 0)
	ZX_res = sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	if(ZX_res < 0){
		if (ZX_res == -3)
		{
			sprintf(CommandElemt.cmdbuf, "ZI");
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			return;
		}
		else if (ZX_res == -9 || ZX_res == -11)
		{
			flg_getedserailarr = true;
			washall = 1;
			sprintf(CommandElemt.cmdbuf, "ZX 0 0 %d", LIQUID_ZMAX);//测液体
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// if (sb_waitingframeaswer(&CommandElemt) < 0)
			if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
			{		
				ReportArrayData[plate_num % 9] = 0XFFFFFFFF;//没测到		
			}
			else{
				ReportArrayData[plate_num % 9] = serail_dataarr;
			}
		}
		else{
			ReportArrayData[plate_num % 9] = 0XFFFFFFFF; //没测到
		}
	}
	else{
		ReportArrayData[plate_num % 9] = serail_dataarr;
	}	
	printf("Arraydata=%d\n",serail_dataarr);
	CommandElemt.srdevaddr = ARM_ADDR;
	if (ReportArrayData[plate_num % 9] != 0XFFFFFFFF)
	{	
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 200, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 				CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	sprintf(CommandElemt.cmdbuf, "ZA 0");
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	tp_washchange(true);
	if (last_cabin_reagent == REAGENT_WATER)
	{
		if(washall)
			mb_probewash_a(WATERPORT);
		else
			mb_probewash_c(WATERPORT);
	}
	else if (last_cabin_reagent == REAGENT_WASH )
	{
		if(washall)
			mb_probewash_a(WASHPORT);
		else
			mb_probewash_c(WASHPORT);
	}
		
	tp_washchange(false);

	if (SHELF_LOCK_WAY_NEW)
		GPIO_OutClear(reagent_check[plate_num / 9].lock);//锁住进行扫描
	else
		GPIO_OutSet(reagent_check[plate_num / 9].lock);//锁住进行扫描
			
	reagent_check[plate_num / 9].NEED_CHECK = false;

	if(ReportArrayData[plate_num % 9] >= LIQUID_ZMAX || ReportArrayData[plate_num % 9] ==  0XFFFFFFFF)
		ReportArrayData[plate_num % 9] = 0XFFFFFFFF;
	else
	{
		for (i = 0; i < 9;i++)
		{
			ReportArrayData[i] += CorrectDate[i];
			printf("ReportArrayData[%d]=%x ", i, ReportArrayData[i]);
		}
	}

	TCP_Buffer[0] = plate_num;

	memcpy(&TCP_Buffer[1], &ReportArrayData[plate_num % 9], 4);
	nt_sendpacketdata(REAGENT_SEND, (char*)TCP_Buffer, 4 + 1);
	
	if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
		lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
	
	#if(USE_LOG_INFO == 1)
	lprintf(log_my, INFO, "out of mb_reagentchecking\n");
	printf("out of mb_reagentchecking\n");
	#endif
}



void mb_canceloperate(operate_t** operate_p, bool head)
{
		
	if ((*operate_p)->next != NULL)
	{
		mb_canceloperate(&((*operate_p)->next), false);
		
	}
	//退出SetOperateDisable后还需判断
	if ((*operate_p)->next == NULL)
	{
		if (!head)	//连表头不进行free
		{
			free(*operate_p);
			*operate_p = NULL;
		}
	}	
}

//将一架次的操作清空 operate_head为头， IsHead 是否为头标志
void mb_setoperateheaddisable(operate_head_list** operate_head, bool IsHead)
{
	operate_t* operate_p = NULL;	
	
	if ((*operate_head)->next_head != NULL)
	{
		mb_setoperateheaddisable(&((*operate_head)->next_head), false);
	}
	
	if ((*operate_head)->next_head == NULL)
	{
		operate_p = &((*operate_head)->operate);
		mb_canceloperate(&operate_p, true);

		if (!IsHead)	//连表头不进行free
		{
			free(*operate_head);
			*operate_head = NULL;
		}
		else
		{
			(*operate_head)->operate.reagent = STOP_OPERATE;
			memset( &(*operate_head)->operate.reagent_info, 0, sizeof(reagent_t));
		}
	}
}

//将水合坐标填入
void mb_sethydratepale(unsigned char shelfnum)
{
	operate_head_list* operate_head_p = NULL;
	unsigned char i = 0;
	dispense_t *hydrate_platetmp = NULL;
	
	if (shelfnum == 1)
	{
		operate_head_p = operate_head1;
		hydrate_platetmp = hydrateA.hydrate_plate;
	}
	else if (shelfnum == 2)
	{
		operate_head_p = operate_head2;
		hydrate_platetmp = hydrateB.hydrate_plate;
	}
	else if (shelfnum == 3)
	{
		operate_head_p = operate_head3;
		hydrate_platetmp = hydrateC.hydrate_plate;
	}
	
	memset(hydrate_platetmp, 0xFF, sizeof(hydrateA.hydrate_plate));
	
	while(operate_head_p != NULL)	//轮询一架玻片
	{
		hydrate_platetmp[i].plate_num = (operate_head_p->operate.plate_num) % 30;
		hydrate_platetmp[i].reagent = REAGENT_WASH;
		operate_head_p = operate_head_p->next_head;
		i++;
		if (operate_head_p == NULL)
			break;
	}	//一轮查询结束
	hydrate_platetmp[i].reagent = STOP_OPERATE;	

	#if(USE_PRINT_LOG  == 1)
	printf("hydrateA.flage = %d\n", hydrateA.flage);
	printf("hydrateA.start_time = %d\n", hydrateA.start_time);
	printf("hydrateA.hydrate_plate= \n");
	for (i = 0; i < 11;i++)
		printf(" %d-%d", hydrateA.hydrate_plate[i].plate_num, hydrateA.hydrate_plate[i].reagent);
	
	printf("hydrate_plate= \n");
	#endif
}


int mb_setmeasconfig()
{
	char sendbuf[10] = {0};
	sendbuf[0] = 3;
	sendbuf[1] =1;
	char buf[30] ={0};
	sprintf(buf, "cp %s %s", MeasureConfigPath, MeasureConfigDefPath);
	if(system(buf) < 0)
		sendbuf[1] =0;

	nt_sendpacketdata(UpdateSlaveState , sendbuf, 2);
					
	return 0;
}


int mb_getmeasconfig()
{
	char data_buf[1000000] = {0},sendbuf[10], data_hang[1000] = {0};
	FILE *F = NULL;
	int size = 0;
	
	printf("int GeGetDev\n");
	data_buf[0] = 2;
	data_buf[1] = 1;

	if((F = fopen(MeasureConfigPath, "r=")) == NULL)
	{
		printf("open failed\n");
		sendbuf[1] =0;	
		nt_sendpacketdata(UpdateSlaveState , sendbuf, 2);
	}
	fseek(F, 0, SEEK_END);
	size = ftell(F);
	rewind(F);
	while(fgets(data_hang, 1000000, F) != NULL)
		strcat(&data_buf[2], data_hang);

	fclose(F);
	#if(USE_LOG_INFO == 1)
	printf("GetDevGetDevGetDevGetDevGetDevGetDev\n");
	printf(" %s size=%d", data_buf,size);
	printf("readend\n\n");
	#endif

	nt_sendpacketdata(UpdateSlaveState , data_buf, 2 + size);
	return 0;
}

int mb_updatemeasconfig(char*data, int part_len)
{
	fd_set DeviceRead;
	fd_set DeviceWrite;
	int 	DeviceMax = 0, res, i;
	struct timeval timeout;
	int data_index = 0, netread_len,rec_time = 0, all_len = 0;
	char data_buf[1000000], sendbuf[10];
	char netbuf_read[10000];
	FILE *F = NULL;
	char str = 1;

	memcpy(&all_len,data,4);
	all_len -= 2;//减去命令和种类
	memcpy(data_buf, &data[6], part_len);
	data_index = part_len;
	sendbuf[0] = 1;
	printf("in upgrate part_len=%d all_len=%d\n", part_len, all_len);
	
	if (data_index >= all_len )
	{
		printf("upgrate finished data_index=%d\n", data_index);

		if((F = fopen(MeasureConfigPath, "w=")) == NULL)
		{
			printf("open failed\n");
			sendbuf[1] = 0;	
			nt_sendpacketdata(UpdateSlaveState , sendbuf, 2);
		}
		fwrite(data_buf, 1, data_index, F);

		fclose(F);

		for(str = 1; str < 4; str++)	
			mb_setdefaulconfpara(&str);
		
		sendbuf[1] =1;
		nt_sendpacketdata(UpdateSlaveState , sendbuf, 2);
		return 0;
	}
	
	while(!flg_mainproexit)
	{
						//读取网口命令
		timeout.tv_sec = 0;
		timeout.tv_usec = 10;
		FD_ZERO(&DeviceRead);		
		DeviceMax = local_fd + 1;
		FD_SET(local_fd, &DeviceRead);
	
		res = select(DeviceMax, &DeviceRead, &DeviceWrite, NULL, &timeout);
		
		if(res < 0) {
			perror("[ListenNet] select error");
			sendbuf[1] =0;
			nt_sendpacketdata(UpdateSlaveState , sendbuf, 2);
			return -1;
		}

		if (FD_ISSET(local_fd, &DeviceRead))
		{
			printf("one rute\n");

			if ((netread_len = read(local_fd, (char *)netbuf_read , 2000)) < 0)
			{
				printf("[ListenNet]read netdata error in upgrate %s\n",strerror(errno));
			}
			else 
			{
				if (data_index + netread_len > all_len)//防止连包
					memcpy(&data_buf[data_index], netbuf_read, (all_len - data_index));
				else
					memcpy(&data_buf[data_index], netbuf_read, netread_len);
		
				data_index += netread_len;
			}
		}
		else{
			rec_time++;
		}

		if (data_index >= all_len )
		{
			printf("upgrate finished data_index=%d\n", data_index);
			
			for (i = 0; i < all_len; i++)
				printf("%c", data_buf[i]);

			printf("******************");

			F = fopen(MeasureConfigPath, "w=");
			if (F == NULL)
			{
				sendbuf[1] = 0;
				perror("open file error\n");
			}
			else
			{
				fwrite(data_buf, 1, all_len, F);
				fclose(F);
				sendbuf[1] = 1;
			}

			nt_sendpacketdata(UpdateSlaveState, sendbuf, 2);	
		}	
	}
	return 0;
}



//part_len 为第一段数据长度
int mb_updatefirmware(char*data, int part_len)
{
	fd_set DeviceRead;
	fd_set DeviceWrite;
	int  DeviceMax = 0, res = 0;
	struct timeval timeout;
	int data_index = 0, netread_len, rec_time = 0, all_len = 0;
	char data_buf[1000000], sendbuf[10];
	char netbuf_read[10000];
	FILE *F = NULL;

	memcpy(&all_len, data, 4);
	all_len -= 2;//减去命令和种类
	memcpy(data_buf, &data[6], part_len);
	data_index = part_len;
	sendbuf[0] = 0;
	
	printf("in upgrate part_len=%d all_len=%d\n", part_len, all_len);
	
	if (data_index >= all_len )
	{
		printf("upgrate finished data_index=%d\n", data_index);
			
		if ((F = fopen("/mnt/nandflash/pooltest", "w=")) == NULL)
		{
			printf("open failed\n");

			sendbuf[1] =0;
			nt_sendpacketdata(UpdateSlaveState, sendbuf, 2);
		}
		fwrite(data_buf, 1, data_index, F);
		fclose(F);

		sendbuf[1] = 1;
		nt_sendpacketdata(UpdateSlaveState, sendbuf, 1);
		return 0;
	}
	
	while(!flg_mainproexit)
	{
		//读取网口命令
		timeout.tv_sec = 0;
		timeout.tv_usec = 10;
		FD_ZERO(&DeviceRead);		
		DeviceMax = local_fd + 1;
		FD_SET(local_fd, &DeviceRead);
	
		res = select(DeviceMax, &DeviceRead, &DeviceWrite, NULL, &timeout);
		
		if(res < 0) {
			perror("[ListenNet] select error");

			sendbuf[1] = 0;
			nt_sendpacketdata(UpdateSlaveState, sendbuf, 2);
			return -1;
		}

		if (FD_ISSET(local_fd, &DeviceRead))
		{
			printf("one rute\n");

			if ((netread_len = read(local_fd, (char *)netbuf_read , 2000)) < 0)
			{
				printf("[ListenNet]read netdata error in upgrate %s\n",strerror(errno));
			}
			else 
			{
				if (data_index + netread_len > all_len)//防止连包
					memcpy(&data_buf[data_index],netbuf_read,(all_len - data_index));
				else
					memcpy(&data_buf[data_index],netbuf_read,netread_len);
		
				data_index += netread_len;
			}
		}
		else{
			rec_time++;
		}
		if (data_index >= all_len )
		{
			printf("upgrate finished data_index=%d\n",data_index);

			F = fopen("/mnt/nandflash/pooltesttmp","w=");
			if (F == NULL)
			{
				sendbuf[1] =0;
				perror("open file error\n");
			}
			else
			{
				fwrite(data_buf,1,all_len,F);
				fclose(F);
				sendbuf[1] =1;
			}

			//	if (system("sh /mnt/nandflash/upgrate.sh") < 0)
			if (system("cp /mnt/nandflash/pooltesttmp /mnt/nandflash/pooltest") < 0)
				sendbuf[1] =0;
			if (system("chmod 777 /mnt/nandflash/pooltest") < 0)
				sendbuf[1] =0;	
			nt_sendpacketdata(UpdateSlaveState , sendbuf, 2);		
			//	return 0;
		}				
	}
	return 0;
}



/******************************************************************************
*
* Function Name  : mb_errorinfoproce
* Description    : .错误处理
* 					 
* Input		   :emERRORCODE *error_code 错误数组指针, int error_num 错误个数
* Output		   :None
* Return		   :  None
*******************************************************************************/
void mb_errorinfoproce(emERRORCODE *error_code, int error_num)
{
	int i = 0;

	for (i = 0; i < error_num; i++)
	{
		if (
			error_code[i] == CONNECT_ERROR_WATER || 
			error_code[i] == CONNECT_ERROR_MINIA  || error_code[i] == CONNECT_ERROR_MINIB  || 
			error_code[i] == CONNECT_ERROR_MINIC  || error_code[i] == CONNECT_ERROR_PUMPA  ||
			error_code[i] == CONNECT_ERROR_PUMPB  || error_code[i] == CONNECT_ERROR_PUMPC  ||
			error_code[i] == ARM_LOSE_MINIA       || error_code[i] == ARM_LOSE_MINIB       ||
			error_code[i] == ARM_LOSE_MINIC       || 
			error_code[i] == ARM_STEP_LOSS_X      || error_code[i] == ARM_STEP_LOSS_Y ||
			error_code[i] == ARM_STEP_LOSS_Z      || error_code[i] == ARM_DEVICE_NOT_INITIALIZED ||
			error_code[i] == CONNECT_ERROR_SCANNER || error_code[i] == CONNECT_ERROR_WEIGHT ||
			error_code[i] == CONNECT_ERROR_ARM ||
			//	 error_code[i] == WAST_LIUID_LOW ||
			//	error_code[i] == WAST_LIUID_HIGH ||
			error_code[i] == CONNECT_ERROR_TEMPER || error_code[i] == CONNECT_ERROR_TEMPRO ||
			error_code[i] == PUMP_PLUNGER_OVERLOAD || error_code[i] == PUMP_VALVE_OVERLOAD
		)
		{
			GPIO_OutSet(ER_LED);
			beep_state.error = 1;
			sleep(2);
			flg_mainproexit = TRUE;     //  必须断电重启 andry
			#if( USE_LOG_INFO  == 1)
			lprintf(log_my, ERROR, "errorinforoproce error to stop machin");
			#endif
			printf("\n******************Progress exited*************\n");
		}
		else if (error_code[i] == DOOR_OPEN || error_code[i] == WATERPOUR_WRONG || 
			error_code[i] == WASHPOUR_WRONG || error_code[i] == ALCOHOLPOUR_WRONG ||
			error_code[i] == ER1POUR_WRONG || error_code[i] == ER2POUR_WRONG
			//	|| error_code[i] == ASPIRATE_WRONG
			// || error_code[i] == REAGENT_REMOVED
		)		
		{
			stop_flag = TRUE;    // 不用断电重启，只要故障处理掉后能自动恢复   andry
			printf("\n******************Progress stoped*************\n");
		}
		else if (error_code[i] == SHELF_LOSE_MINIA)
		{
			wkeventA = STOP_WORK;
			hydrateA.flage = FALSE;	
			mb_mainworkstop(1);	
		}
		else if (error_code[i] == SHELF_LOSE_MINIB )
		{
			wkeventB = STOP_WORK;
			hydrateB.flage = FALSE;	
			mb_mainworkstop(2);	
		}
		else if(error_code[i] == SHELF_LOSE_MINIC  )
		{
			wkeventC = STOP_WORK;
			hydrateC.flage = FALSE;	
			mb_mainworkstop(3);	
		}		
	}
}


/******************************************************************************
*
* Function Name  : mb_reportshelfstatus
* Description    : .向PC发送玻片架状态
* 					 
* Input		   :unsigned char* mini_recieve_code mini返回的码, char shelf_num 玻片架次
* Output		   :None
* Return		   :  None
*******************************************************************************/
void mb_reportshelfstatus(void)
{
	uint8_t i = 0, slf_num = 0;
	static char last_send_state[3] = {0};
	
	for (i = 0; i < 3;i++)
	{
		slf_num = i + 1;

		if ((SHELF_LOCK_STATE[i] == 1) && (last_send_state[i] != 1))
		{
			nt_sendpacketdata(SHELF_UNLOCKED, (char*)&slf_num, 1);
			last_send_state[i] = 1;
			printf("report_shelf_stateSHELF_UNLOCKED=%d, last_send_state=%d\n", i, last_send_state[i]);
		}
		else if ((SHELF_LOCK_STATE[i] == 2) && (last_send_state[i] != 2))
		{
			nt_sendpacketdata(SHELF_LOCKED, (char*)&slf_num, 1);
			last_send_state[i] = 2;
			printf("report_shelf_statSHELF_LOCKEDe=%d, last_send_state=%d\n", i, last_send_state[i]);
		}
	}
}



void DischargeHighWork(char tim)
{
	pthread_mutex_lock(&pump_lock);
	#if(USE_LOG_INFO == 1)
	printf("The task is mb_dischwasteliquid_hi.\n");
	lprintf(log_my, INFO,"The task is mb_dischwasteliquid_hi.\n");
	#endif

 	inDischarge = TRUE;

	sleep(5);//确保吸液控制已关闭
	GPIO_OutClear(V1);
	GPIO_OutClear(V2);
	GPIO_OutClear(V3);
	GPIO_OutClear(V4);
	sleep(1);
	GPIO_OutClear(V9);
	GPIO_OutSet(V8);
	GPIO_OutClear(V7);
	GPIO_OutClear(V5);
	GPIO_OutSet(V6);
	GPIO_OutSet(VP1);

	sleep(tim);

	GPIO_OutClear(V8);
	GPIO_OutSet(V9);
	sleep(3);
	GPIO_OutClear(VP1);
	GPIO_OutClear(V8);
	GPIO_OutClear(V7);
	GPIO_OutClear(V5);
	GPIO_OutClear(V6);
	sleep(3);
	if (pthread_mutex_unlock(&pump_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
	inDischarge = FALSE;

}


/******************************************************************************
	
	排高浓度废液
*******************************************************************************/
bool mb_dischwasteliquid_hi(char tim)
{
	unsigned int dwPinState;
	static bool hidischargefaulting = 0;

	if (hidischargefaulting)
		return true;
	
	DischargeHighWork(tim);

	dwPinState = In_H;
	GPIO_PinState(&dwPinState);
	if (NEW_INWATST_SEN)
	{
		if ((dwPinState & In_H) == 0)
		{	
			DischargeHighWork(tim);		
			GPIO_PinState(&dwPinState);
			if ((dwPinState & In_H) == 0)
			{	
				printf("mb_dischwasteliquid_hi error\n");
				hidischargefaulting = true;
				mb_seterrorcode(WAST_LIUID_HIGH);
			}
		 }	
	}
	else
	{
		if ((dwPinState & In_H) != 0)
		{
			DischargeHighWork(tim); 		
			GPIO_PinState(&dwPinState);
			if ((dwPinState & In_H) != 0)
			{	
				printf("mb_dischwasteliquid_hi error\n");
			
				mb_seterrorcode(WAST_LIUID_HIGH);
			}
		 }	
	}

	return true;
}


void mb_phydischanrgelo(char tim)
{
	pthread_mutex_lock(&pump_lock);
	
	#if(USE_LOG_INFO == 1)
	printf("The task is mb_dischargwateliquid_lo.\n");
	lprintf(log_my, INFO,"The task is mb_dischargwateliquid_lo.\n");
	#endif

	inDischarge = true;
	sleep(3);  // sleep(5);//确保吸液控制已关闭
	
	GPIO_OutClear(V1);
	GPIO_OutClear(V2);
	GPIO_OutClear(V3);
	GPIO_OutClear(V4);
	sleep(1);
	GPIO_OutClear(V8);
	GPIO_OutSet(V9);
	GPIO_OutClear(V7);
	GPIO_OutSet(V5);
	GPIO_OutSet(V6);
	GPIO_OutSet(VP1);

	sleep(tim);
	GPIO_OutClear(V9);
	GPIO_OutSet(V8);
	sleep(3);
	GPIO_OutClear(VP1);
	GPIO_OutClear(V9);
	GPIO_OutClear(V7);
	GPIO_OutClear(V5);
	GPIO_OutClear(V6);
	sleep(3);//防止飞溅
	if (pthread_mutex_unlock(&pump_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
	inDischarge = false;
}

/******************************************************************************
排低浓度废液
GPIO_OutClear为关闭
*******************************************************************************/
bool mb_dischargwateliquid_lo(char tim)
{
	unsigned int dwPinState;
	static bool lodischargefaulting = false;

	if (lodischargefaulting)
		return true;
	
	mb_phydischanrgelo(tim);	

	dwPinState = In_L;
	GPIO_PinState(&dwPinState);

	if (NEW_INWATST_SEN)
	{
		if ((dwPinState & In_L) == 0)
		{	
			mb_phydischanrgelo(tim);	
		
			GPIO_PinState(&dwPinState);
			if ((dwPinState & In_L) == 0)
			{	
				printf("mb_dischwasteliquid_hi error\n");
				lodischargefaulting = true;
				mb_seterrorcode(WAST_LIUID_LOW);
			}else{
				lodischargefaulting = false;
			}
		 }	
	}
	else
	{
		if ((dwPinState & In_L) != 0)
		{
			mb_phydischanrgelo(tim);	
		
			GPIO_PinState(&dwPinState);
			if ((dwPinState & In_L) != 0)
			{	
				printf("mb_dischwasteliquid_hi error\n");

				mb_seterrorcode(WAST_LIUID_LOW);
			}
		 }	
	}
	return true;
}


/******************************************************************************
排玻片架的残留液
参数shelf 1，2，3对应三组玻片架 4大仪器废液瓶
GPIO_OutClear 关闭
*******************************************************************************/
bool mb_dischargeshelfwateliquid(unsigned char shelf)
{
	bool flg_unlock = false;
	
	if (!flg_lockdischargliquit)
	{
		pthread_mutex_lock(&mutex_dischargelock);
		flg_lockdischargliquit = true;
		flg_unlock = true;
	}

	printf("The task is mb_dischargeshelfwateliquid.\n");
	lprintf(log_my, INFO, "The task is mb_dischargeshelfwateliquid.\n");

	pthread_mutex_lock(&pump_lock);
	GPIO_OutClear(V1);
	GPIO_OutClear(V2);
	GPIO_OutClear(V3);
	GPIO_OutClear(V4);
	GPIO_OutClear(V8);
	GPIO_OutClear(V9);
	GPIO_OutSet(V7);      //  
	GPIO_OutClear(V5);
	GPIO_OutClear(V6);
	GPIO_OutSet(VP1);
	
	sleep(5);           // 抽真空

	if(shelf == 1)
	{
		GPIO_OutSet(V1);
		sleep(6);
		GPIO_OutClear(V1);
	}
	else if(shelf == 2)
	{
		GPIO_OutSet(V2);
		sleep(6);
		GPIO_OutClear(V2);
	}
	else if (shelf == 3)
	{
		GPIO_OutSet(V3);
		sleep(6);
		GPIO_OutClear(V3);
	}
	
	GPIO_OutClear(VP1);
	GPIO_OutClear(V7);

	if (pthread_mutex_unlock(&pump_lock) != 0)
		lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error pump_lock");
	if (flg_unlock)
	{
		if (pthread_mutex_unlock(&mutex_dischargelock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mutex_dischargelock");
		flg_lockdischargliquit = false;
	}

	return true;
}


/******************************************************************************
*
* Function Name  : md_gpiointion
* Description    : GPIO initialization, configuration GPIO 0~11,15~23 output;12~14, 24~31 input mode.
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int md_gpiointion(void)
{
	int	rc = 0;
	unsigned int dwPinState = 0xffffffff;
	
	dwPinState = 0x0ffffe0f;
	GPIO_fd = open("/dev/em9280_gpio", O_RDWR);
	
	printf("GPIO_fd open file = %d\n", GPIO_fd);

	rc = GPIO_OutEnable(dwPinState);	//
	if(rc < 0)
	{
		printf("GPIO_OutEnable::failed %d\n", rc);
		return -1;
	}

	if (SHELF_LOCK_WAY_NEW)
	{
		GPIO_OutClear(CGLOCK1);
		GPIO_OutClear(CGLOCK2);
		GPIO_OutClear(CGLOCK3);
		GPIO_OutClear(CGLOCK4);
	}
	else
	{
		GPIO_OutSet(CGLOCK1);
		GPIO_OutSet(CGLOCK2);
		GPIO_OutSet(CGLOCK3);
		GPIO_OutSet(CGLOCK4);
	}
	
	GPIO_OutClear(V1);
	GPIO_OutClear(V2);
	GPIO_OutClear(V3);
	GPIO_OutClear(V4);
	GPIO_OutClear(V5); 
	GPIO_OutClear(V6);
	GPIO_OutClear(V7);
	GPIO_OutClear(V8);
	GPIO_OutClear(V9);
	GPIO_OutClear(VP1);

	GPIO_OutSet(TEM_KEYA);
	GPIO_OutSet(TEM_KEYB);
	GPIO_OutSet(TEM_KEYC);
	
	if (!flg_mainproexit)
		GPIO_OutSet(ER_LED);// 错误退出时调用 md_gpiointion
		
	return 1;
}


/******************************************************************************
*
* Function Name  : GPIO_OutEnable
* Description    : GPIO output enable.
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int GPIO_OutEnable(unsigned int dwEnBits)
{
	int  rc = 0;
	struct double_pars	dpars;

	dpars.par1 = EM9280_GPIO_OUTPUT_ENABLE;		// 0
	dpars.par2 = dwEnBits;

	rc = write(GPIO_fd, &dpars, sizeof(struct double_pars));
	return rc;
}

/******************************************************************************
*
* Function Name  : GPIO_OutDisable
* Description    : GPIO output disable.
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int GPIO_OutDisable(unsigned int dwDisBits)
{
	int rc = 0;
	struct double_pars	dpars;

	dpars.par1 = EM9280_GPIO_OUTPUT_DISABLE;	// 1
	dpars.par2 = dwDisBits;
	rc = write(GPIO_fd, &dpars, sizeof(struct double_pars));
	return rc;
}

/******************************************************************************
*
* Function Name  : GPIO_OutSet
* Description    : Set GPIO output mode.
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int GPIO_OutSet(unsigned int dwSetBits)
{
	int  rc = 0;
	struct double_pars	dpars;

	//硬件上F300电路板io输出与S300相反
	dpars.par1 = EM9280_GPIO_OUTPUT_SET;	// 2
	dpars.par2 = dwSetBits;

	rc = write(GPIO_fd, &dpars, sizeof(struct double_pars));
	return rc;
}

/******************************************************************************
*
* Function Name  : GPIO_OutClear
* Description    : Clear GPIO output mode.
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int GPIO_OutClear(unsigned int dwClearBits)
{
	int  rc = 0;
	struct double_pars	dpars;

	dpars.par1 = EM9280_GPIO_OUTPUT_CLEAR;	// 2
	dpars.par2 = dwClearBits;
	rc = write(GPIO_fd, &dpars, sizeof(struct double_pars));
	return rc;
}

int GPIO_PinState( unsigned int* pPinState)
{
	int 				rc;
	struct double_pars	dpars;

	dpars.par1 = EM9280_GPIO_INPUT_STATE;	// 5
	dpars.par2 = *pPinState;

	rc = read(GPIO_fd, &dpars, sizeof(struct double_pars));
	if(!rc)
	{
		*pPinState = dpars.par2;
	}
	return rc;
}


int PWM_Start(int fd, int freq, int duty )
{
	int 					rc;
	struct stmb_pwmconf	conf;

	conf.cmd = EM9280_PWM_START;
	conf.freq = freq;
	conf.duty = duty;
	conf.polarity = POLARITY;

	rc = write(fd, &conf, sizeof(struct stmb_pwmconf));
	return rc;
}

int PWM_Stop(int fd )
{
	int 					rc;
	struct stmb_pwmconf	conf;

	memset( &conf, 0, sizeof(struct stmb_pwmconf));
	conf.cmd = EM9280_PWM_STOP;

	rc = write(fd, &conf, sizeof(struct stmb_pwmconf));
	return rc;

}


bool mb_getstatefromeshelf(char num)
{
	return flg_shelfcanindex[(uint8_t)num];
}

void mb_setstatetoshelf(char num, bool shelfstate)
{
	 flg_shelfcanindex[(uint8_t)num] = shelfstate;
}


//fr == 0 停止声音 fr == 1 通知声 fr == 2 报警声
int mb_beeperproce(void)
{	
	// unsigned int duty = 50;
	// unsigned char FREQH[] ={0xF2,0xF3,0xF5,0xF5,0xF6,0xF7,0xF8,//低音1234567?????
	// 					0xF9,0xF9,0xFA,0xFA,0xFB,0xFB,0xFC,0xFC,//1// ,2,3,4,5,6,7,i?????
	// 					0xFC,0xFD,0xFD,0xFD,0xFD,0xFE,//高音?234567?????
	// 					0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFF};//超高音?1234567
	// int fd = 0;
	
	if (wkevent == MAINTAIN_WORK)
		return 0;
	
	if (beep_state.door == 0 && beep_state.cabin == 0 && beep_state.error == 0)
	{
		GPIO_OutSet(BEEPER);
	}
	else if ((beep_state.door == 1 || beep_state.cabin == 1) && beep_state.error == 0)
	{
		while(1)
		{
			GPIO_OutSet(BEEPER);
			if ((beep_state.door == 1 || beep_state.cabin == 1) && beep_state.error == 0)
			{}
			else	break;
			
			GPIO_OutClear(BEEPER);
			usleep(300000);
			GPIO_OutSet(BEEPER);
			usleep(3000000);
		}
	}
	else if (beep_state.error == 1)
	{	
		while(beep_state.error == 1)
		{
			GPIO_OutSet(ER_LED);
			GPIO_OutClear(BEEPER);
			usleep(500000);
			GPIO_OutClear(ER_LED);
			GPIO_OutSet(BEEPER);
			usleep(500000);
		}
	}
	
	return 0;
	// fd = open("/dev/em9280_pwm4", O_RDWR);
	// if ( fd < 0)
	// {
	// 	printf("can not open /dev/em9280_pwm4 device file!\n");
	// 	return -1;
	// }
	// while(!flg_mainproexit)
	// {
	// 	//	for (i = 0;i < sizeof(FREQH);i++)
	// 	{		
	// 		PWM_Start( fd, FREQH[0] * 2, duty );
	// 		sleep(1);
	// 		PWM_Start( fd, FREQH[7] * 2, duty );
	// 		sleep(1);
	// 	}
	// }
	
	// return PWM_Stop( fd);
}


bool mb_checksensorproce(void)
{
	unsigned int dwPinState,i;
	static bool DOOR_OPEN_SENDED = false, CG1_SCANED[4] = {false};
	static struct timeval disc_now,last_disc;

	gettimeofday(&disc_now, NULL);
	
	if(disc_now.tv_sec > (last_disc.tv_sec + 60*20) )
	{
		printf("discharge time reach\n");
		//	mb_dischargwateliquid_lo(DischargeTime);	
		last_disc.tv_sec = disc_now.tv_sec;
	}

	dwPinState = DOOR;
	GPIO_PinState(&dwPinState);

	if ((dwPinState & DOOR) != 0)			//在洗针时pthread_mutex_lock(&mutex_dischargelock);，门打开暂停洗针
	{										//，ExcuteOperate中不能pthread_mutex_lock(&mutex_dischargelock)
		if (DOOR_OPEN_SENDED == false)		//而门检测与ExcuteOperate同以县城导致死锁不能正常运行所以门检测需其他线程中
		{
			DOOR_OPENDED = true;
			beep_state.door = 1;
				mb_seterrorcode(DOOR_OPEN);
			DOOR_OPEN_SENDED = true;
		}
	}
	else
	{	
		if (DOOR_OPEN_SENDED == true)
		{
			DOOR_OPEN_SENDED = false;
			DOOR_OPENDED = false;
			mb_seterrorcode(DOOR_OPEN | 0X01000000);
			beep_state.door = 0;
		}
	}

	dwPinState = POWER_SEN;
	GPIO_PinState(&dwPinState);	
	if ((dwPinState & POWER_ERROR) == 1)
	{
		printf("^^^^^^^^^^^^POWER_ERROR^^^^^^^^^^^^^");
		mb_seterrorcode(POWER_ERROR);
	}
	

	dwPinState = Out_H;
	GPIO_PinState(&dwPinState);	

	if(NEW_OUTWATST_SEN)
	{
		if ((dwPinState & Out_H) == 0)
		{
			if (OUT_WASTHIGH_H == false)
			{
				printf("^^^^^^^^^^^^Out_H^^^^^^^^^^^^^");
				mb_seterrorcode(WAST_LIUID_HIGH_FULL);
				OUT_WASTHIGH_H = true;
			}
		}
	}
	else
	{
		if ((dwPinState & Out_H) != 0)
			{
				if (OUT_WASTHIGH_H == false)
				{
					printf("^^^^^^^^^^^^Out_H^^^^^^^^^^^^^");
					mb_seterrorcode(WAST_LIUID_HIGH_FULL);
					OUT_WASTHIGH_H = true;
				}
			}
	}
	dwPinState = Out_L;
	GPIO_PinState(&dwPinState);	

	if(NEW_OUTWATST_SEN)
	{
		if ((dwPinState & Out_L) == 0)
		{
			if(OUT_WASTHIGH_L == false)
			{
				printf("^^^^^^^^^^^^Out_L^^^^^^^^^^^^^");
				mb_seterrorcode(WAST_LIUID_LOW_FULL);
				OUT_WASTHIGH_L = true;
			}
		}
	}
	else
	{
		if ((dwPinState & Out_L) != 0)
		{
			if(OUT_WASTHIGH_L == false)
			{
				printf("^^^^^^^^^^^^Out_L^^^^^^^^^^^^^");
				mb_seterrorcode(WAST_LIUID_LOW_FULL);
				OUT_WASTHIGH_L = true;
			}
		}
	}

	dwPinState = In_H;
	GPIO_PinState(&dwPinState);

	if (NEW_INWATST_SEN)
	{
		if ((dwPinState & In_H) == 0)
		{	
			printf("^^^^^^^^^^^^In_H^^^^^^^^^^^^^");
			mb_dischwasteliquid_hi(DischargeTime);		
			last_disc.tv_sec = disc_now.tv_sec;
		}
	}
	else
	{
		if ((dwPinState & In_H) != 0)
		{	
			printf("^^^^^^^^^^^^In_H^^^^^^^^^^^^^");
			mb_dischwasteliquid_hi(DischargeTime);	
			last_disc.tv_sec = disc_now.tv_sec;
		}
	}
	dwPinState = In_L;
	GPIO_PinState(&dwPinState);

	if (NEW_INWATST_SEN)
	{
		if ((dwPinState & In_L) == 0)
		{	
			printf("^^^^^^^^^^^^In_L^^^^^^^^^^^^^");
			mb_dischargwateliquid_lo(DischargeTime);	
			last_disc.tv_sec = disc_now.tv_sec;
		}
	}
	else
	{
		if ((dwPinState & In_L) != 0)
		{
			printf("^^^^^^^^^^^^In_L^^^^^^^^^^^^^");
			mb_dischargwateliquid_lo(DischargeTime);	
			last_disc.tv_sec = disc_now.tv_sec;
		}
	}

	for (i = 0; i < 4; i++)
	{
		dwPinState = reagent_check[i].sen;
		GPIO_PinState(&dwPinState);
		
		if ((dwPinState & (reagent_check[i].sen)) == 0)
		{
			if (!CG1_SCANED[i])
			{
				printf("reagent_check[%d].NEED_SCAN",i);
				if (wkevent != MAINTAIN_WORK)
					reagent_check[i].NEED_SCAN = true;
			
				CG1_SCANED[i] = true;
			}
		}
		else
		{
			reagent_check[i].NEED_SCAN = false;
			if (CG1_SCANED[i] == true)
			{
				CG1_SCANED[i] = false;		
				mb_cancelreagentwork(i + 1);	//使流程中试剂无效 并将有效试剂加入outside链表中
			}
		}
	}
	
	return true;
}


//第一次灌注 perfusion_num不要求 在灌注操作的下层会赋值。 检查大容量桶是否被拿出，拿出2次自动灌注 andry
int mb_checkcabinhavereagt(unsigned char perfusion_num)
{
	int i = 0,res = 0;

	if (flg_cabinremovreagt_inport)
		return 0;

	flg_cabinremovreagt_inport = TRUE;

	i = 0;
	while(i < 6)
	{	
		usleep(100000);
		for (i = 0; i < 6; i++)
		{
			if (flg_cabinhavereagent[i] )	//有效的瓶子被拿掉 一直等待
				if ( cabin_value[i + 6] == 0)
				{
					#if( USE_LOG_INFO == 1)
					printf("Check the reagenttab removed cabin was removed num = %d\n",i);
					#endif
					break;
				}
		}
	}

	for (i = 0; i < 6; i++)
	{
		if (need_perfusion_last[i] < need_perfusion[i])//如果差值大于1 表示拔了两次只需灌注一次
		{
			need_perfusion_last[i] = need_perfusion[i];
			if (i == perfusion_num)//当此操作中正是对应试剂的灌注操作在外层就不需要在继续灌注
				res = -1;
			
			tp_washchange(TRUE);
			if (i == 0)
			{
				if (!mb_waterwashreagentpour(WATERPORT))
					mb_seterrorcode(WATERPOUR_WRONG);
			}
			else if (i == 1)
			{
				if (!mb_waterwashreagentpour(WASHPORT))
					mb_seterrorcode(WASHPOUR_WRONG);
			}
			else if (i == 2)
			{
				if  (!mb_muiltreagentpour(ALCOHOLPORT))
					mb_seterrorcode(ALCOHOLPOUR_WRONG);
			}
			else if (i == 3)
				mb_muiltreagentpour(DEWAXPORT);
			else if (i == 4)
			{
				if (!mb_muiltreagentpour(ER1PORT))
					mb_seterrorcode(ER1POUR_WRONG);
			}
			else if (i == 5)
			{
				if (!mb_muiltreagentpour(ER2PORT))
					mb_seterrorcode(ER2POUR_WRONG);
			}
			tp_washchange(FALSE);
		}	
	}

	flg_cabinremovreagt_inport = FALSE;

	return res;
}

int mb_monitdoorstate(void)
{
	sr_cmdstruct_t CommandElemt;

	if (inDischarge)	{
		while(inDischarge) 
			sleep(1);
	}

	if (!door_open_action)
	{
		if (DOOR_OPENDED)
		{
			printf("DOOR_OPENDED***********************\n");
			if (initialize_finished)
			{
				CommandElemt.srdevaddr = ARM_ADDR;

				strcpy(CommandElemt.cmdbuf, "ZA 0");	
				// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
				// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
				// 	printf("[WarterPour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
				// else
				// 	printf("[WarterPour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
				// sb_waitingframeaswer(&CommandElemt);
				sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
			}
			while(DOOR_OPENDED)
			sleep(1);
		}
	}
	if (!dng_wast_barrel)
	{
		while(OUT_WASTHIGH_L) sleep(1);
		while(OUT_WASTHIGH_H) sleep(1);
	}

	if (wkevent != FREE_WORK)
	{
		if (wkevent == PAUSE)
		{
			/*
				while(wkevent != FREE_WORK)	//等待PC 继续工作信息
				{
					if (wkevent == STOP_ALL)	//等待过程中收到停止命令
							pthread_exit(NULL);
					sleep(1);
				}
			*/
		}
		else if (wkevent == STOP_ALL)
			pthread_exit(NULL);
		else if (wkeventA == STOP_WORK )
			return -1;
		else if (wkeventB == STOP_WORK )
			return -2;
		else if (wkeventC == STOP_WORK)
			return -3;
	}

	return 0;
}

/***********************水 缓冲液的灌注**************************/
bool mb_waterwashreagentpour(unsigned char reagent_port)
{
	bool res = true;
	sr_cmdstruct_t CommandElemt;
	unsigned char perfusion_num;
	int perfusion_res = 0;
	printf("in **[mb_waterwashreagentpour] reagent_port=%d**",reagent_port);
	sleep(1);

	Water_Wash_Pour_AGAIN:

	if (reagent_port == WATERPORT)
	{
		mb_procwashprobe(REAGENT_WATER);
		last_cabin_reagent = reagent_flag = REAGENT_WATER;
		perfusion_num = 0;
	}
	else
	{
		mb_procwashprobe(REAGENT_WASH);
		last_cabin_reagent = reagent_flag = REAGENT_WASH;
		perfusion_num = 1;
	}
	mb_monitdoorstate(); 
			
	perfusion_res = mb_checkcabinhavereagt(perfusion_num);
	if (perfusion_res < 0)	return true;//确保当有多个需要灌注时，灌注操作的完整性
	else if (perfusion_res == 2)
		goto Water_Wash_Pour_AGAIN;
	
	mb_monitdoorstate(); 
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	pthread_mutex_lock(&pump_lock);//防止排液
	if (IsWashProbeStart == false)//防止dowork_WashProbe 线程也锁上但是优先级这里高
	{
		if (pthread_mutex_unlock(&pump_lock) != 0);//防止排液
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
		sleep(1);
		pthread_mutex_lock(&pump_lock);
	}
	
	CommandElemt.srdevaddr = PUMP_ADDR;						//注射器命令最多50个字符								//注射器共拉6下，打掉5下半的液体
	sprintf(CommandElemt.cmdbuf, "S%dA0gI%dP%dM%dO%dD%dM%dG5I%dP%dM%dO%dD%dR",
									SYSPEED,reagent_port,SYFSTEP,SYDELAY,PROBEPORT,SYFSTEP,SYDELAY,reagent_port,SYFSTEP,SYDELAY,PROBEPORT,SYHSTEP);
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	if (pthread_mutex_unlock(&pump_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
	/*	
	perfusion_res = mb_checkcabinhavereagt(perfusion_num);
	if (perfusion_res < 0)	return TRUE;//确保当有多个需要灌注时，灌注操作的完整性
	else if (perfusion_res == 2)
		goto Water_Wash_Pour_AGAIN;
	*/
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH3][0], ordArray_wash[WASH1][1], ZWASH_STEP3);// 3号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED_CLEAN, SYHHSTEP);	//打掉剩余的一半液体的1/2
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	mb_monitdoorstate();
	/*
	perfusion_res = mb_checkcabinhavereagt(perfusion_num);
	if (perfusion_res < 0)	return TRUE;//确保当有多个需要灌注时，灌注操作的完整性
	else if (perfusion_res == 2)
	goto Water_Wash_Pour_AGAIN;
	*/
	usleep(DISPENSE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 4号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(1);
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dM%dI%dP%dM%dO%dD%dR", 
							SYSPEED_CLEAN, 0, SYDELAY, reagent_port, SYFSTEP, SYDELAY,
							PROBEPORT, SYFSTEP); //打光剩余的液体。注射器吸一下，打光
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	/*
	perfusion_res = mb_checkcabinhavereagt(perfusion_num);
	if (perfusion_res < 0)	return TRUE;//确保当有多个需要灌注时，灌注操作的完整性
	else if (perfusion_res == 2)
	goto Water_Wash_Pour_AGAIN;
	*/
	usleep(DISPENSE_T);
		
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;

	//sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 600, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 				CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(3);
	sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);//
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt); 
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH2][0], ordArray_wash[WASH2][1], ZWASH_STEP2_2);// 2号清洗槽状态2
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	/*
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_AIR,SYSTEP_AIR);	//缓慢抽吸空气
		if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
				CommandElemt.srdevaddr , 1, 0, 1) > 0)
				printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			else
				printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			sb_waitingframeaswer(&CommandElemt);
	usleep(ASPIRATE_T);
	*/
	/*
	mb_monitdoorstate();
	perfusion_res = mb_checkcabinhavereagt(perfusion_num);
	if (perfusion_res < 0)	return TRUE;//确保当有多个需要灌注时，灌注操作的完整性
	else if (perfusion_res == 2)
		goto Water_Wash_Pour_AGAIN;
	*/
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d", ZWASH_STEP2_1);// 2号清洗槽状态1
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dP%dO%dS%dP%dR", SYSPEED,reagent_port,SYMHSTEP,PROBEPORT,SYSPEED_WASTE,SYSTEP_WASTE);	//一半多液体及吸取残留液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		/*
		perfusion_res = mb_checkcabinhavereagt(perfusion_num);
		if (perfusion_res < 0)	return TRUE;//确保当有多个需要灌注时，灌注操作的完整性
		else if (perfusion_res == 2)
			goto Water_Wash_Pour_AGAIN;
		*/
	usleep(ASPIRATE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED,SYLIQTSTSTEP + SYSTEP_WASTE - wat_sypstep); //打掉注射器中液体的一半
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(3);		//有空气会有液体滴下来
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH2][0], ordArray_wash[WASH2][1], ZWASH_STEP2_2);// 2号清洗槽状态2
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED_TST,SYSTEP_WASTE);	//打掉少量液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	//	return 1;
	usleep(DISPENSE_T);
	mb_monitdoorstate(); 
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);// 2号清洗槽状态3
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_AIR,SYSTEP_LIQTSTAIR);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	mb_monitdoorstate(); 

	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  ZWASH_STEP2_2 - 50);//
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(300000);

	sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", POUR_LIQUID_DISTANCE_ZMAX, POUR_LIQUID_DISTANCE_ZMAX, ZWASH_STEP2_ZX);//测液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		
	// if (sb_waitingframeaswer(&CommandElemt) < 0)
	if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
	{	
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		
		// if (sb_waitingframeaswer(&CommandElemt) < 0)
		if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
		{	
			res = false;
		}			
	}
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d", ZWASH_STEP2_1);// 2号清洗槽状态1
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_WASTE,SYSTEP_WASTE);	//吸取残留液
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dR", SYSPEED,0); //打光注射器中所有液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(1);
	if (res)	
		mb_seterrorcode((WATERPOUR_WRONG + perfusion_num) | 0X01000000);
		
	return res;
}


/***********************酒精，脱蜡剂，修复1，修复2的灌注**************************/
bool mb_muiltreagentpour(unsigned char reagent_port)
{
	sr_cmdstruct_t CommandElemt;
	bool res = true;
	unsigned char perfusion_num = 0;
	int perfusion_res = 0;

	CommandElemt.srdevaddr = ARM_ADDR;

	sprintf(CommandElemt.cmdbuf, "SL %d %d", 5000, 0X1200);
	// 	//	sprintf(CmdElemt.cmdbuf, "SL %d %d", 5000,0X3000);
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 2) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

OtherPour_AGAIN:	
	sleep(1);

	printf("reagent_port=%d", reagent_port);
	
	if (reagent_port == ALCOHOLPORT)
	{
		mb_procwashprobe(REAGENT_ALCOHOL);
		last_cabin_reagent = reagent_flag = REAGENT_ALCOHOL;
		perfusion_num = 3;
	}
	else if (reagent_port == DEWAXPORT)
	{
		mb_procwashprobe(REAGENT_DEWAX);
		last_cabin_reagent = reagent_flag = REAGENT_DEWAX;
		perfusion_num = 2;
	}
	else if (reagent_port == ER1PORT)
	{
		mb_procwashprobe(REAGENT_ER1);
		last_cabin_reagent = reagent_flag = REAGENT_ER1;
		perfusion_num = 4;
	}
	else if (reagent_port == ER2PORT)
	{
		mb_procwashprobe(REAGENT_ER2);
		last_cabin_reagent = reagent_flag = REAGENT_ER2;
		perfusion_num = 5;
	}
	
	mb_monitdoorstate(); 
		
	perfusion_res = mb_checkcabinhavereagt(perfusion_num);
	if (perfusion_res < 0)	
		return true;	
	else if (perfusion_res == 2)
		goto OtherPour_AGAIN;

	mb_monitdoorstate(); 
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// //	strcpy(CommandElemt.cmdbuf, "PA 2445 60 200");	
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	
	pthread_mutex_lock(&pump_lock);//防止排液
	if (IsWashProbeStart == false)//防止dowork_WashProbe 线程也锁上但是优先级这里高
	{
		if (pthread_mutex_unlock(&pump_lock) != 0)//防止排液
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error pump_lock");
		sleep(1);
		pthread_mutex_lock(&pump_lock);
	}

	CommandElemt.srdevaddr = PUMP_ADDR;						//注射器命令最多50个字符								//注射器共拉6下，打掉5下半的液体
	sprintf(CommandElemt.cmdbuf, "S%dA0gI%dP%dM%dO%dD%dM%dG5I%dP%dM%dO%dD%dR", SYSPEED_CLEAN, reagent_port, SYFSTEP, \
				SYDELAY, PROBEPORT, SYFSTEP, SYDELAY, reagent_port, SYFSTEP, SYDELAY, PROBEPORT, SYHSTEP);
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	if (pthread_mutex_unlock(&pump_lock) != 0)//防止排液
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
	usleep(ASPIRATE_T); 	
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH3][0], ordArray_wash[WASH3][1], ZWASH_STEP3);// 3号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED_CLEAN, SYHHSTEP);	//打掉剩余的液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d",  200, ZSPEED_SLOW);//针缓慢上升
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 					CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(3);
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//针缓慢上升
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 					CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	mb_monitdoorstate(); 
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH2][0], ordArray_wash[WASH2][1], ZWASH_STEP2_2);// 2号清洗槽状态2
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	/*
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_AIR,SYSTEP_AIR);	//缓慢抽吸空气
	if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			CommandElemt.srdevaddr , 1, 0, 1) > 0)
		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	else
		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	sb_waitingframeaswer(&CommandElemt);
	usleep(ASPIRATE_T);
	*/
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d", ZWASH_STEP2_1);// 2号清洗槽状态1
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		/*	
		perfusion_res = mb_checkcabinhavereagt(perfusion_num);
		if (perfusion_res < 0)	return TRUE;	
		else if (perfusion_res == 2)
			goto OtherPour_AGAIN;
		*/
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dP%dO%dS%dP%dR", SYSPEED,reagent_port,SYMHSTEP,PROBEPORT,SYSPEED_WASTE,SYSTEP_WASTE);	//一半多液体及吸取残留液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED,SYLIQTSTSTEP + SYSTEP_WASTE);	//打掉注射器中液体的一半
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(3);		//有空气会有液体滴下来
	
	mb_monitdoorstate(); 
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH2][0], ordArray_wash[WASH2][1], ZWASH_STEP2_2);// 2号清洗槽状态2
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED_TST,SYSTEP_WASTE);	//打掉少量液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	//while(1) scanf("%d",&res);
	usleep(DISPENSE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);// 2号清洗槽状态3
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else	
	// 	printf("mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_AIR,SYSTEP_LIQTSTAIR);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  ZWASH_STEP2_2 - 50);//
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(300000);
				
	sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", POUR_LIQUID_DISTANCE_ZMAX, POUR_LIQUID_DISTANCE_ZMAX, ZWASH_STEP2_1 );//测液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );		
	// if (sb_waitingframeaswer(&CommandElemt) < 0)
	if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
	{	
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );	
		// if (sb_waitingframeaswer(&CommandElemt) < 0)
		if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
		{	
			res = false; 
		}			  
	}  
			
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;	
	sprintf(CommandElemt.cmdbuf, "ZA %d", ZWASH_STEP2_1);  // 2号清洗槽状态1
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_WASTE, SYSTEP_WASTE);	//吸取残留液
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);	
		
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dR", SYSPEED, 0); //打光注射器中所有液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt); 
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(1);
	
	sprintf(CommandElemt.cmdbuf, "SL %d %d", DEC_SEN, CHECK_SEN);
	// //	sprintf(CmdElemt.cmdbuf, "SL %d %d", 5000,0X3000);
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 2) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
					
	if (res && (reagent_port != DEWAXPORT))
		mb_seterrorcode((WATERPOUR_WRONG + perfusion_num) | 0X01000000);
	return res;
}


/*****************清洗探针********************/
void mb_probewash_a(unsigned char liquid_port)		//清洗并灌注大容量试剂 水和缓冲液 其他不能用A
{
	sr_cmdstruct_t CommandElemt;
	
	sleep(1);
	mb_monitdoorstate();
 
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T); 
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	pthread_mutex_lock(&pump_lock);//防止排液
	if (IsWashProbeStart == FALSE)//防止dowork_WashProbe 线程也锁上但是优先级这里高
	{
		if(pthread_mutex_unlock(&pump_lock) != 0)//防止排液
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
		sleep(1);
		pthread_mutex_lock(&pump_lock);
	}
		
	CommandElemt.srdevaddr = PUMP_ADDR;						//注射器命令最多50个字符								//排空注射器拉4下打3+4/5液体
	//	sprintf(CommandElemt.cmdbuf, "S%dA0gI%dP%dM%dO%dD%dM%dG2I%dP%dM%dO%dD%dR",
	//										SYSPEED,liquid_port,SYFSTEP,SYDELAY,PROBEPORT,SYFSTEP,SYDELAY,liquid_port,SYFSTEP,SYDELAY,PROBEPORT,SY45STEP);
	sprintf(CommandElemt.cmdbuf, "S%dA0gI%dP%dM%dO%dD%dM%dG2R",
										SYSPEED,liquid_port,SYFSTEP,SYDELAY,PROBEPORT,SYFSTEP,SYDELAY);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	if (pthread_mutex_unlock(&pump_lock) != 0)//防止排液
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
	usleep(DISPENSE_T);	
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH3][0], ordArray_wash[WASH3][1], ZWASH_STEP3);// 3号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,liquid_port,SYFSTEP,PROBEPORT,SY15STEP);	//打掉剩余液体再拉1下打掉1/5
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 4号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sleep(1);
	CommandElemt.srdevaddr = PUMP_ADDR;
	//sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,liquid_port,SYFSTEP,PROBEPORT,SY45STEP);	//打掉剩余液体再拉1下打掉
	sprintf(CommandElemt.cmdbuf, "S%dA%dR", SYSPEED,0);	//打掉剩余液体再拉1下打掉
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);	
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	
	usleep(DISPENSE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 800, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		// 	if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// 	else
		// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// 	sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);	
	
//	sleep(1);	 
}



void mb_probewash_b(unsigned char liquid_port)
{
	sr_cmdstruct_t CommandElemt;
	int ssspd = 0;

	if (liquid_port == ALCOHOLPORT)
		ssspd = SYSPEED_ALCHOLE;
	else
		ssspd = SYSPEED_CLEAN;
	sleep(1);
	mb_monitdoorstate();
	 
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气	
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	pthread_mutex_lock(&pump_lock);//防止排液
	if (IsWashProbeStart == FALSE)//防止dowork_WashProbe 线程也锁上但是优先级这里高
	{
		if (pthread_mutex_unlock(&pump_lock) != 0)//防止排液
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
		sleep(1);
		pthread_mutex_lock(&pump_lock);
	}
	
	CommandElemt.srdevaddr = PUMP_ADDR;						//注射器命令最多50个字符								//排空注射器拉4下打3+4/5液体
	//	sprintf(CommandElemt.cmdbuf, "S%dA0gI%dP%dM%dO%dD%dM%dG3I%dP%dM%dO%dD%dR",
	//										ssspd,liquid_port,SYFSTEP,SYDELAY,PROBEPORT,SYFSTEP,SYDELAY,liquid_port,SYFSTEP,SYDELAY,PROBEPORT,SY45STEP);
	sprintf(CommandElemt.cmdbuf, "S%dA0gI%dP%dM%dO%dD%dM%dG3R",
										ssspd,liquid_port,SYFSTEP,SYDELAY,PROBEPORT,SYFSTEP,SYDELAY);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	if (pthread_mutex_unlock(&pump_lock) != 0)//防止排液
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
	usleep(DISPENSE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH3][0], ordArray_wash[WASH3][1], ZWASH_STEP3);// 3号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]successful.\n", CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]failed.\n" , CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;	
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED_ALCHOLE, 0, liquid_port, SYHSTEP, PROBEPORT, SYHSTEP);	//打掉剩余液体再拉1下打掉1/5
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]successful.\n" , CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]failed.\n" , CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 200, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// 	if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_b]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
//	sleep(1);
	
}

void mb_probewash_c(unsigned char liquid_port)		
{
	sr_cmdstruct_t CommandElemt;

Wash_ProbeC:	
	mb_monitdoorstate();
	if (mb_checkcabinhavereagt(3) < 0)	return;//洗针以前就需要灌注
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气	
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,liquid_port,SYHSTEP,PROBEPORT,SYHHSTEP);	//排空注射器拉1/2下打1/4液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_probewash_c]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_probewash_c]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH3][0], ordArray_wash[WASH3][1], ZWASH_STEP3);// 3号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("mb_probewash_c]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dR", SYSPEED_CLEAN, 0);	//打掉剩余液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 200, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// 	if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_c]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	if (mb_checkcabinhavereagt(3) < 0)	goto Wash_ProbeC;//洗针以后需要灌注,灌注好后重新洗针
//	sleep(1);
}



void mb_probewash_d(unsigned char liquid_port)		
{
	sr_cmdstruct_t CommandElemt;

	//	sleep(1);
	mb_monitdoorstate();
	
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,liquid_port,SYFSTEP,PROBEPORT,SYHHSTEP);	//排空注射器拉1下打1/4液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 3号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dI%dP%dO%dD%dR", SYSPEED_CLEAN, 0, liquid_port, SYFSTEP, PROBEPORT, SYFSTEP,
				liquid_port, SYHHSTEP, PROBEPORT, SYHHSTEP);	//打掉剩余液体拉1打1拉1/4打光
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 800, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_d]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

//	sleep(1);		
}


void mb_dab1wash(void)    // unusing this function
{
	sr_cmdstruct_t CommandElemt;
	bool flg_dischargliquitunlock = false;//确保只进行一次解锁
	
	mb_monitdoorstate();
	if (!flg_lockdischargliquit)
	{
		pthread_mutex_lock(&mutex_dischargelock);
		flg_lockdischargliquit = true;
		flg_dischargliquitunlock = true;
	}
	tp_washchange(true);
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气	
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,WATERPORT,SYFSTEP,PROBEPORT,SYHSTEP);	//排空注射器拉1下打1/2液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	mb_monitdoorstate();	
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 4号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dI%dP%dO%dD%dR", SYSPEED,0,WATERPORT,SYFSTEP,PROBEPORT,SYFSTEP,WATERPORT,SY15STEP,PROBEPORT,SY15STEP);	//打掉剩余液体拉1打1拉1/5打光
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	mb_monitdoorstate();

	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 200, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// //sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab1wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	tp_washchange(false);

	if (flg_dischargliquitunlock)
	{
		if (pthread_mutex_unlock(&mutex_dischargelock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error DiscahgeLock");
		flg_lockdischargliquit = false;
	}

}


void mb_dab2wash(void)   //  unusing this function
{
	sr_cmdstruct_t CommandElemt;
	//bool DischargeLock_NEED_UNLOCK = false; //确保只进行一次解锁
	
	mb_monitdoorstate();
	
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dR", SYSPEED,0,WATERPORT,SYFSTEP);	//排空注射器拉1下液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);
	mb_monitdoorstate();	
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 4号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,WATERPORT,SYFSTEP,PROBEPORT,SYFSTEP);	//打掉剩余液体拉1打光
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	usleep(DISPENSE_T);
	mb_monitdoorstate();
	
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 200, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// //sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dab2wash]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
}

/*********
send channel num 
*********/
void mb_getreagentliquitcab(emb_comid comPort)
{		
	int res , i, timeout_counter=0, error_counter = 0;
	unsigned short crc=0;
	unsigned char cmdbuffer[10] ={0};
	//	unsigned char sensor_state = 0;
	static unsigned short last_cabin_state[6] = {1,1,1,1,1,1};//0表示被拔出 1表示装入状态//DEW WATER ALCHOLE WASH ER1 ER2
	//	stminibd_sendpacket cmd;
	// static  short last_cabin_val[6] = {0};
	static unsigned short last_cabin_value[12]={0};
	static bool fisrt_value = true;
	memset(cmdbuffer, 0, sizeof(cmdbuffer));
	static int selcnt = 0;
	// short sen_change = 0;
	char Isallzero = 0;

	if (selcnt > 5)
		selcnt = 0;
	else
		selcnt++;
			
	selcnt = 0;
	cmdbuffer[0] = 0xff;
	cmdbuffer[1] = 0x00;
	cmdbuffer[2] = selcnt;
	crc = crc16_ccitt((const char*)&cmdbuffer[1], 2);
	memcpy((char*)&cmdbuffer[3],&crc,2);
	cmdbuffer[5] = 0xfe;
	tcflush(serail_hand[comPort],TCOFLUSH);
	
	//		for (i = 0;i < 6;i++)
	//		printf(" %x",cmdbuffer[i]);
	res = write(serail_hand[comPort],cmdbuffer,6);

	if (res < 0)
		printf("write weight error\n");
			
	//	printf("need_perfusion= %d\n ",need_perfusion[0]);
	while(!readfinished485)
	{
		usleep(100);
		if (timeout_counter >= 60000)
		{
			printf("recieve weight timeout send cmd again\n");
			//		printf("need_perfusion= %d\n ",need_perfusion[0]);
			res = write(serail_hand[comPort],cmdbuffer,6);
			if (res < 0)
				printf("write weight error\n");
			
			timeout_counter = 0;
		}
		if (error_counter >= 300000)
		{
			if (!initialize_finished)//只在初始化时报错
				mb_seterrorcode(CONNECT_ERROR_WEIGHT);
	
			return;
		}
		timeout_counter++;
		error_counter++;
	}
		 
	if (ReadBufferweight[0] == 0)
	{
		//	printf("data484 from weight\n");
		//	printf("need_perfusion= %d\n ",need_perfusion[0]);
		if(ReadBufferweight[1] == WATER_ACK)
		{
			//		printf("ack from weight\n");
			//		printf("need_perfusion= %d\n ",need_perfusion[0]);
			pthread_mutex_lock(&mutex_cabinlock);
			memset((uint8_t*)ReadBufferweight,0,sizeof(ReadBufferweight));
			readindexweight = 0;
			readfinished485 = false;
			pthread_mutex_unlock(&mutex_cabinlock);
			error_counter = 0;
			while(!readfinished485)
			{
				usleep(100000);
				if (error_counter >= 100)
				{
					printf("communicating with weight error response\n");
					memset((uint8_t*)ReadBufferweight, 0, sizeof(ReadBufferweight));
					readindexweight = 0;
					readfinished485 = false;
					return;
				}
				error_counter++;
			}
			//		printf("response need_perfusion= %d\n ",need_perfusion[0]);
			//		pthread_mutex_lock(&report_lock);
					
			memcpy((char*)cabin_value, (uint8_t*)&ReadBufferweight[1], 18);			
			for ( i = 12;i < 18;i++)
			{
				cabin_value[i - 6] = ReadBufferweight[i + 1];
			}

			#if( S600_CABIN_IN_0 == 1)
			for(i = 0; i < 6; i++)
			{
				cabin_value[6+i] = cabin_value[6+i]?0:1;
			}
			#endif

			for(i = 0; i < 6;i++)
			{
				if(last_cabin_state[i] == 1 && cabin_value[6 + i] == 0)
					Isallzero = 1;
				else
				{
					Isallzero = 0;
					break;
				}
			}
			if(!Isallzero)
			{
				for (i = 0; i < 6;i++)
				{
					//		printf(" %d **",last_cabin_state[i]);
					//		printf(" %d ",need_perfusion[i]);
					if (last_cabin_state[i] == 0 && cabin_value[6 + i] == 1)	//当有0变成1时表示试剂桶装入
					{
						need_perfusion[i]++; //在其他操作中发现先前值与当前值不同 灌注后重复其操作									
					}
					last_cabin_state[i] = cabin_value[6 + i];//赋值 当前值
				}
			}
			//		pthread_mutex_unlock(&report_lock);
			if (fisrt_value)
			{
				memcpy((char *)last_cabin_value, (char *)cabin_value, 24);
				fisrt_value = false;
				flg_cabinreved = true;
			}
			else
			{
				if(!Isallzero)
				{
					if (memcmp((char *)last_cabin_value, (char *)cabin_value, 24) != 0)
						flg_cabinreved = true;
					else 
						flg_cabinreved = false;
				}
				memcpy((char *)last_cabin_value, (char *)cabin_value, 24);
			}
		}
			
			/*	printf("after memcpy last_cabin_state[]=");
			for (i = 0; i < 6;i++)
			{
				printf(" %d **",last_cabin_state[i]);
				}
			*/
		//	printf("sensor_state=%d\n", sensor_state);

		//解析结束清数据
		pthread_mutex_lock(&mutex_cabinlock);
		memset((uint8_t*)ReadBufferweight, 0, sizeof(ReadBufferweight));
		readindexweight = 0;
		readfinished485 = false;
		pthread_mutex_unlock(&mutex_cabinlock);
	}
}


/******************************************************************************
*
* Function Name  : mb_waterctrl
* Description    : 水箱控制
* 					 
* Input		   :emb_comid comPort 串口号, unsigned char cmd 命令码
* Output		   :
* Return		   :  int 错误-1 正确0
*******************************************************************************/
int mb_waterctrl(emb_comid comPort,unsigned char cmd)
{
	int res,i,timeout_counter=0,error_counter = 0;
	unsigned short crc=0;
	unsigned char cmdbuffer[10] ={0};

	return 0;
	memset(cmdbuffer,0,sizeof(cmdbuffer));

	cmdbuffer[0] = 0xff;
	cmdbuffer[1] = 0x01;
	cmdbuffer[2] = cmd;
	crc = crc16_ccitt((const char*)&cmdbuffer[1], 2);
	memcpy(&cmdbuffer[3],&crc,2);
	cmdbuffer[5] = 0xfe;
	tcflush(serail_hand[comPort],TCOFLUSH);

	for (i = 0;i < 6;i++)
		printf(" %x",cmdbuffer[i]);
	res = write(serail_hand[comPort],cmdbuffer,6);
	if (res < 0)
		printf("write water error\n");
	else
		printf("write water successed\n");

	while(!readfinished485)
	{
		usleep(100000);
		if (timeout_counter >= 20)
		{
			printf("recieve water timeout send cmd again\n");
			res = write(serail_hand[comPort],cmdbuffer,6);
			if (res < 0)
				printf("write water error\n");
			else
				printf("write water successed\n");
			timeout_counter = 0;
		}
		if (error_counter >= 100)
		{
			mb_seterrorcode(CONNECT_ERROR_WATER);
	
			return -1;
		}
		timeout_counter++;
		error_counter++;
	}

	if (ReadBuffer485[0] == 1)
	{
		printf("data484 from water\n");
		if(ReadBuffer485[1] == WATER_ACK)
		{
			printf("ack from water\n");
			memset((uint8_t*)ReadBuffer485, 0, sizeof(ReadBuffer485));
			readindex485 = 0;
			readfinished485 = FALSE;
			error_counter = 0;
			while(!readfinished485)
			{
				usleep(100000);	
				if (error_counter >= 5000)
				{
					printf("communicating with water error response\n");
					return -1;
				}
				error_counter++;
			}

			if(ReadBuffer485[1] == WATER_WORK_NORMAL) 
			{
				printf("WATER_WORK_NORMAL\n");
			}
			else
			{
				//mb_seterrorcode(WATER_PROVIDE_ERROR);
			}
		}
		//解析结束清数据
		memset((uint8_t*)ReadBuffer485, 0, sizeof(ReadBuffer485));
		readindex485 = 0;
		readfinished485 = FALSE;
	}

	return 0;
}




/*************
当num == 0XFF 表示 测液体直到测不到或管路出错为止
***************/
/******************************************************************************
*
* Function Name  : mb_dabaspirateclear
* Description    : 在清洗混合站时吸取液体
* 					 
* Input		   :const unsigned char num 循环次数, BOOL NEED_ZS 是否测液体,unsigned int* Mixing_Sation 坐标指针
* Output		   :
* Return		   :  BOOL 错误FLASE 正确TRUE
*******************************************************************************/
int mb_dabaspirateclear(const unsigned char num, bool NEED_ZS, unsigned int* Mixing_Sation)
{
	sr_cmdstruct_t cmdframebuf;
	unsigned char error_counter = 0, i = 0;
	bool is_clear = false;

	for (i = 0; i < num; i++)		//循环吸取混合瓶中DAB混合液直到检测不到为止或管路出错
	{
		mb_monitdoorstate();
		cmdframebuf.srdevaddr = ARM_ADDR;
		sprintf(cmdframebuf.cmdbuf, "PA %d %d %d", Mixing_Sation[0], Mixing_Sation[1], MOV_ZH);//混合站目标位置
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);

		if (NEED_ZS)
		{
			mb_monitdoorstate();
			cmdframebuf.srdevaddr = ARM_ADDR;
			sprintf(cmdframebuf.cmdbuf, "ZX %d %d %d", 0, 0, DAB_ZMAX - 10);//测液体
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
			// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
			// 		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
			// 	else
			// 		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
				
			// 	if (sb_waitingframeaswer(&cmdframebuf) < 0)	
			if(sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1) < 0)
			{
				mb_monitdoorstate();
				sprintf(cmdframebuf.cmdbuf, "ZX %d %d %d", 0, 0, DAB_ZMAX - 10);//再测液体
					// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
					// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
					// 		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
					// 	else
					// 		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
						
					// if (sb_waitingframeaswer(&cmdframebuf) < 0)	
					// 	is_clear = true;
				if(sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1) < 0)
				{
					is_clear = true;
				}
			}
		}
		mb_monitdoorstate();
		if (is_clear)
			break;	
		cmdframebuf.srdevaddr = ARM_ADDR;
		#if(1 == S600_SHWON_DOWN_DABCNG)
		if(wkevent == INIT_WORK)
		{
			sprintf(cmdframebuf.cmdbuf, "ZA %d", DAB_ZMAX + 5);
		}else{
			sprintf(cmdframebuf.cmdbuf, "ZA %d", DAB_ZMAX);
		}	
		#else 
			sprintf(cmdframebuf.cmdbuf, "ZA %d", DAB_ZMAX);//
		#endif 

		sprintf(cmdframebuf.cmdbuf, "ZA %d", DAB_ZMAX);//
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
		// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
		// 	else
		// 		printf("[mb_muiltreagentpour]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );	
		// 	sb_waitingframeaswer(&cmdframebuf);
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);
				
		cmdframebuf.srdevaddr = PUMP_ADDR;
		//	sprintf(cmdframebuf.cmdbuf, "S%dP%dR", SYSPEED_MIXDAB,SYHHSTEP);	//吸取1/4DAB
		
		sprintf(cmdframebuf.cmdbuf, "S%dP%dR", SYSPEED_MIXDAB,SY45STEP);	//吸取1/4DAB
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
			// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
			// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
			// 	else
			// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
			// 	sb_waitingframeaswer(&cmdframebuf);	
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);

		usleep(ASPIRATE_T);
		cmdframebuf.srdevaddr = ARM_ADDR;
		sprintf(cmdframebuf.cmdbuf, "ZS %d %d",  MOV_ZH - DAB_ZMAX, LIQUID_SPEED);//混合站目标位置
		// if (sb_armpumpsend(port_arm, (unsigned char	*)cmdframebuf.cmdbuf, 
		// 	cmdframebuf.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
		// else
		// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
		// sb_waitingframeaswer(&cmdframebuf);
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);

		mb_monitdoorstate();
			
		cmdframebuf.srdevaddr = ARM_ADDR;
		sprintf(cmdframebuf.cmdbuf, "ZA %d", MOV_ZH);
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
		// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
		// 	else
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
		// 	sb_waitingframeaswer(&cmdframebuf);
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);

		cmdframebuf.srdevaddr = PUMP_ADDR;
		sprintf(cmdframebuf.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
		// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
		// else
		// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
		// sb_waitingframeaswer(&cmdframebuf);
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);

		cmdframebuf.srdevaddr = ARM_ADDR;
		sprintf(cmdframebuf.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
		// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
		// 	else
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
		// 	sb_waitingframeaswer(&cmdframebuf);
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);

		cmdframebuf.srdevaddr = PUMP_ADDR;
		sprintf(cmdframebuf.cmdbuf, "S%dA%dR", SYSPEED,0);	//打光注射器中所有液体拉一下打光

			////	sprintf(cmdframebuf.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,WATERPORT,SYFSTEP,PROBEPORT,SYFSTEP);	//打光注射器中所有液体拉一下打光
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
		// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
		// 	else
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
		// 	sb_waitingframeaswer(&cmdframebuf);	
		sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);
		error_counter++;
		usleep(DISPENSE_T);

		if (error_counter > 8 && num == 0XFF)
		{
			cmdframebuf.srdevaddr = ARM_ADDR;
			sprintf(cmdframebuf.cmdbuf, "ZA %d", MOV_ZH);
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)cmdframebuf.cmdbuf, 
			// 		cmdframebuf.srdevaddr , 1, 0, 1) > 0)
			// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,cmdframebuf.srdevaddr );
			// 	else
			// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,cmdframebuf.srdevaddr );
			// sb_waitingframeaswer(&cmdframebuf);
			sm_serailsenddat(port_arm, &cmdframebuf, 1, 0, 1);
			
			return -1;
		}
	}

	return 0;
}


int mb_dabprobleclear(void)
{
	sr_cmdstruct_t CommandElemt;
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dR", SYSPEED,0,WATERPORT,SYFSTEP,PROBEPORT);	//打光注射器中所有液体拉一下
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);	
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	mb_monitdoorstate();
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 4号清洗槽
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// 	else
		// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// 	sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA%dI%dP%dO%dD%dR", SYSPEED,0,WATERPORT,SYFSTEP,PROBEPORT,SYFSTEP);	//打光注射器中所有液体拉一下打光
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);	
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	usleep(DISPENSE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 500, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);	
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	sprintf(CommandElemt.cmdbuf, "ZA %d",0);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);	
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	return 0 ;
}


int mb_findoperatedabkind(operate_head_list* operate_head, char kind)
{
	operate_t * operate_p = NULL;
	bool IsDAB = false, IsRED = false;
	
	operate_p = &operate_head->operate;
		
	while (operate_p != NULL)		//找出滴加混合试剂的时间
	{
		if (operate_p->reagent == REAGENT_DAB)
			IsDAB = true;
		if (operate_p->reagent == REAGENT_SECEND)
			IsRED = true;
		
		operate_p = operate_p->next;
	}
	if (!IsDAB && !IsRED)
		return -1;
	
	if (kind == 1 && IsDAB)
		return 1;
	else if (kind == 2 && IsRED)
		return 2;

	return 0;
}




int soak_work(char reagent_num, char percent)
{
	sr_cmdstruct_t CommandElemt;
	int i = 0;
	
	mb_shelfaspirate(reagent_num,1,1300,-1);
	
	nt_sendpacketdata(WORK_PECENT_PROBR_CLR, &percent, 1);
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_dispensingliquid]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_dispensingliquid]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	
	usleep(ASPIRATE_T);
	
	percent += 2;
	nt_sendpacketdata(WORK_PECENT_PROBR_CLR, &percent, 1);
	
	mb_monitdoorstate(); 
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 4号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dD%dM%dR", SYSPEED,PROBEPORT,SYSTEP_LIQUID*13 - SYSTEP_LIQUID*135/100,800);	//打掉剩余液体再拉1下打掉
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// 	sb_waitingframeaswer(&CommandElemt); 
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	for(i = 0;i<21;i++)
	{
		percent += 1;
		nt_sendpacketdata(WORK_PECENT_PROBR_CLR, &percent, 1);
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dI%dD%dM%dP%dR", SYSPEED,PROBEPORT,SYSTEP_LIQUID*135/100,800,SYSTEP_LIQUID*135/100);	//打掉剩余液体再拉1下打掉
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt); 
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	
	nt_sendpacketdata(WORK_PECENT_PROBR_CLR, &percent, 1);
	CommandElemt.srdevaddr = ARM_ADDR;
	// sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 		printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// 	else
	// 		printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	reagent_flag = reagent_num;

	return 0;
}



void wash4_aspirate(void)
{
	sr_cmdstruct_t CommandElemt;

	tp_washchange(TRUE);
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH4][0], ordArray_wash[WASH4][1], ZWASH_STEP4);// 4号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[wash4_aspirate]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[wash4_aspirate]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dA0I%dP%dR",SYSPEED,PROBEPORT,SYSTEP_LIQUID*15);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[wash4_aspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[wash4_aspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	
	usleep(ASPIRATE_T);
	
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 4号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[wash4_aspirate]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[wash4_aspirate]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dD%dR", SYSPEED,PROBEPORT,SYSTEP_LIQUID*15);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[wash4_aspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[wash4_aspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	tp_washchange(FALSE);
}



int mb_probectrl(void)
{
	char percent;
	unsigned int dwPinState;
	int reagent_index = reagent_clr[0] / 9;
	reagent_check[reagent_index].NEED_CHECK  = TRUE;
	dwPinState = reagent_check[reagent_index].sen;

	GPIO_PinState(&dwPinState);
	if ((dwPinState & reagent_check[reagent_index].sen) == 1)//已经拿掉
	{
		if (SHELF_LOCK_WAY_NEW)
		GPIO_OutClear(reagent_check[reagent_index].lock);//锁住进行扫描
		else
		GPIO_OutSet(reagent_check[reagent_index].lock);//锁住进行扫描
	
		reagent_check[reagent_index].NEED_CHECK  = FALSE;
		if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mainarm_lock");
		return -1;
	}

	if (SHELF_LOCK_WAY_NEW)
		GPIO_OutSet(reagent_check[reagent_index].lock);//锁住进行扫描
	else
		GPIO_OutClear(reagent_check[reagent_index].lock);//锁住进行扫描

	wkevent = FREE_WORK;

	mb_procwashprobe(REAGENT_WATER);
	wash4_aspirate();
	tp_washchange(TRUE);
	mb_probewash_c(WATERPORT);
	tp_washchange(FALSE);
	soak_work(reagent_clr[0],10);
	wash4_aspirate();

	tp_washchange(TRUE);
	mb_probewash_c(WATERPORT);
	tp_washchange(FALSE);
	soak_work(reagent_clr[1],40);
	wash4_aspirate();

	tp_washchange(TRUE);
	mb_probewash_c(WATERPORT);
	tp_washchange(FALSE);
	soak_work(reagent_clr[2],70);
	wash4_aspirate();
	tp_washchange(TRUE);
	mb_probewash_c(WATERPORT);
	tp_washchange(FALSE);
	percent = 100;
	nt_sendpacketdata(WORK_PECENT_PROBR_CLR, &percent, 1);
	if (SHELF_LOCK_WAY_NEW)
		GPIO_OutClear(reagent_check[reagent_index].lock);//锁住进行扫描
	else
		GPIO_OutSet(reagent_check[reagent_index].lock);//锁住进行扫描

	if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mainarm_lock");
	
	reagent_check[reagent_index].NEED_CHECK  = FALSE;

	#ifdef BIG_VERSION
	GetMixStationVal(6,reagent_index);//解锁
	#endif
	printf("mb_probectrl\n\n");

	return 0;
}



int mb_clearmixstation(unsigned int Mixing_Sation[])
{
	sr_cmdstruct_t CommandElemt;
	unsigned char need_clr_num = 0, i = 0;
	char TCP_Buffer[24] ={0};
	int res = 0;
	
	bool DischargeLock_NEED_UNLOCK = false;//确保只进行一次解锁
	mb_procwashprobe(REAGENT_WATER);

	if (!flg_lockdischargliquit)
	{
		pthread_mutex_lock(&mutex_dischargelock);
		flg_lockdischargliquit = true;
		DischargeLock_NEED_UNLOCK = true;
	}

	tp_washchange(true);
	mb_probewash_c(WATERPORT); 
	
	pthread_mutex_lock(&pump_lock);//防止排液
	if (IsWashProbeStart == false)//防止dowork_WashProbe 线程也锁上但是优先级这里高
	{
		if(pthread_mutex_unlock(&pump_lock) != 0)//防止排液
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
		sleep(1);
		pthread_mutex_lock(&pump_lock);
	}
	lprintf(log_my, INFO,"mb_clearmixstation\n");
	if (mb_dabaspirateclear(0XFF, true, Mixing_Sation) < 0)
	{
		if (DischargeLock_NEED_UNLOCK)
		{
			if (pthread_mutex_unlock(&mutex_dischargelock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_dischargelock");
			flg_lockdischargliquit = false;
		}
		
		if (pthread_mutex_unlock(&pump_lock) != 0)//防止排液
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
		tp_washchange(false);
		return -1;
	}
	
	last_cabin_reagent = reagent_flag = REAGENT_WATER;
	//	sleep(1);
		
	//	mb_dabprobleclear();
	
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", Mixing_Sation[0], Mixing_Sation[1], DAB_ZMAX);//混合站目标位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dP%dO%dD%dI%dP%dO%dD%dR", SYSPEED,WATERPORT,SYFSTEP,PROBEPORT,SYFSTEP,
												WATERPORT,SYHSTEP,PROBEPORT,SYHSTEP);	//注射器拉一下打光拉一半打光
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);	
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(DISPENSE_T);

	mb_dabprobleclear();

	mb_dabaspirateclear(2, false, Mixing_Sation);
	//	mb_dabaspirateclear(1, true, Mixing_Sation);
	res = mb_dabaspirateclear(0XFF, true, Mixing_Sation);

	if (DischargeLock_NEED_UNLOCK)
	{
		if (pthread_mutex_unlock(&mutex_dischargelock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_dischargelock");
		flg_lockdischargliquit = false;
	}
	
	if (pthread_mutex_unlock(&pump_lock) != 0)//防止排液
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
	tp_washchange(false);

	if (res < 0)
	{	
		return -1;	
	}

	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//混合站目标位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	for (i = 0; i < 6;i++)
	{
		if(Mixing_Sation[0] == ordArray_mixed_DAB[i][0])
			mixstation_clear_state[i].NEEDCLEAR = false;
		
		if (mixstation_clear_state[i].NEEDCLEAR == true)
			need_clr_num++;
	}

	if (need_clr_num == 0)
	{
		//全部清洗干净发送信息给PC
		TCP_Buffer[0] = 0;
		//		nt_sendpacketdata(MIXSTATION_NEED_CLEAR, TCP_Buffer, 1);
	}
	return 0;
}


int  ReagentReArrange()//(char* reagent_table,)
{
	return 0;
}


int DetectReagent(char shelf_num,unsigned char* reagent_array, int liquid_stp, int aspirate_val)
{
	sr_cmdstruct_t CommandElemt;
	int res_zx = 0;
	int liquid_zmaxstep=0;

	if (reagent_code[*reagent_array].special_num == 1 )//开放试剂 配置文件中读取 开放试剂瓶的截面积计算
	{
		liquid_zmaxstep = OPEN_ARMZMAX;
	}
	else if(reagent_code[*reagent_array].special_num == 3 || reagent_code[*reagent_array].special_num == 6 ||
		reagent_code[*reagent_array].special_num == FREE_REAGENT2 ||
				reagent_code[*reagent_array].special_num == AP1)//DAB浓缩液和清洗系统,7ml开放试剂
	{
		liquid_zmaxstep = OPEN_ARMZMAX;
	}
	else 
	{ 
		liquid_zmaxstep = LIQUID_ZMAX;
	}

	CommandElemt.srdevaddr = ARM_ADDR;
	mb_monitdoorstate(); 	
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_reagent[*reagent_array][0], ordArray_reagent[*reagent_array][1], MOV_ZH);//H2O2
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	mb_monitdoorstate();
	flg_getedserailarr = true;	
	ReportArrayData[0] = 0;
	if(reagent_code[*reagent_array].special_num == 3 || reagent_code[*reagent_array].special_num == 6 ||
		reagent_code[*reagent_array].special_num == FREE_REAGENT2 ||
		reagent_code[*reagent_array].special_num == 5)	
		sprintf(CommandElemt.cmdbuf, "ZX %d %d %d",liquid_stp , LIQUID_DISTANCE_ZMAX, OPEN_ARMZMAX);//测液体
	else
		sprintf(CommandElemt.cmdbuf, "ZX %d %d %d",liquid_stp , LIQUID_DISTANCE_ZMAX, LIQUID_ZMAX);//测液体
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// res_zx = sb_waitingframeaswer(&CommandElemt);
	res_zx = sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	if (res_zx == -3)
		return -3;
	else if (res_zx == -9 || res_zx == -11)
	{
		flg_getedserailarr = true;
		sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", liquid_stp, LIQUID_DISTANCE_ZMAX, liquid_zmaxstep);//测液体
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// if (sb_waitingframeaswer(&CommandElemt) < 0)
		if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
		{		
			ReportArrayData[0] = 0XFFFFFFFF;//没测到		
		}
		if((serail_dataarr+liquid_stp)  > liquid_zmaxstep)//机械臂测量不足没有报警
		{
			ReportArrayData[0] = 0XFFFFFFFF;//没测到		
		}
	}
	else if (serail_dataarr < (500 - OFFSET_STEP+SHELF_OFFSET))
	{
		ReportArrayData[0] = 1;//发送最大满瓶
		res_zx = -1;
	}
	
	if (ReportArrayData[0] == 0XFFFFFFFF || res_zx == -1)
	{
		sprintf(CommandElemt.cmdbuf, "ZA %d", liquid_zmaxstep);//测液体
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// 		sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	return res_zx;
}


int mb_islockedreagent(uint8_t reagent_num)
{
	if(reagent_lock[reagent_num][0] == 0 && reagent_lock[reagent_num][1] == 0 && reagent_lock[reagent_num][2] == 0)
		return 0;
	return -1;
}
	
	/******************
			mix_diluent:混合站稀释剂，	mulnum 稀释比例
				mulliquid 一片几步混合液
	*****************/
void mb_mixdilutedab(const char* mix__concentrate, const char* mix__diluent, int reagent_cnt, unsigned int liquid_val, const float mulnum,
			const char mulliquid, unsigned int* Mixing_Sation, const char shelf_num)
{
	sr_cmdstruct_t CommandElemt;
	float liquid_step_H2O2 = 0, liquid_step_DAB = 0, AS_num = 0, sysstep_H2O2 = 0, sysstep_DAB = 0;
	float liquid_mulnum = 0;
	int  mstp = 0, aspirate_val = 0, mixliqstp = 0;
	uint32_t i = 0, index = 0;
	unsigned char Array_diluent = 0, Array_Reagent = 0, ori_Array_diluent = 0;
	int res_zx = 0, mix_cnt = 0;
	char TCP_Buffer[24] = {0};
	unsigned char H2O2_cnt = 0;
	static bool NEED_CLEAR_SENDED = false;

	#if(USE_LOG_INFO == 1)
	lprintf(log_my, INFO,"mix__concentrate");
	printf("mix__concentrate");
	for(i = 0; i < reagent_cnt; i++)
	{
		printf(" %d ", mix__concentrate[i]);
		lprintf(log_my, INFO," %d ", mix__concentrate[i]);
	}
	printf("mix__diluent");
	lprintf(log_my, INFO,"mix__diluent");
	for(i = 0; i < reagent_cnt; i++)
	{
		printf(" %d ",mix__diluent[i]);
		lprintf(log_my, INFO," %d ",mix__diluent[i]);
	}

	lprintf(log_my, INFO, "liquid_val = %d, reagent_cnt = %d mulnum=%f\n", liquid_val, reagent_cnt, mulnum);
	printf("liquid_val = %d, reagent_cnt = %d mulnum=%f\n", liquid_val, reagent_cnt, mulnum);
	#endif

	for(i = 0; i < reagent_cnt; i++)
	{
		reagent_check[(mix__concentrate[i] / 9)].NEED_MIX = true;   // 1个试剂架9个样本
		reagent_check[(mix__diluent[i] / 9)].NEED_MIX = true;
	}
	sleep(1);
	for(i = 0; i < reagent_cnt; i++)
	{		
		if ((mix__concentrate[i] / 9) == 0 || (mix__diluent[i] / 9) == 0)
		{	
			if (SHELF_LOCK_WAY_NEW)
				GPIO_OutSet(CGLOCK1);
			else
				GPIO_OutClear(CGLOCK1);
		}
		
		if ((mix__concentrate[i] / 9) == 1|| (mix__diluent[i] / 9) == 1)
		{
			if (SHELF_LOCK_WAY_NEW)
				GPIO_OutSet(CGLOCK2);
			else
				GPIO_OutClear(CGLOCK2);
		}

		if ((mix__concentrate[i] / 9) == 2 || (mix__diluent[i] / 9) == 2)
		{
			if (SHELF_LOCK_WAY_NEW)
				GPIO_OutSet(CGLOCK3);
			else
				GPIO_OutClear(CGLOCK3);
		}

		if ((mix__concentrate[i] / 9) == 3|| (mix__diluent[i] / 9) == 3)
		{
			if (SHELF_LOCK_WAY_NEW)
				GPIO_OutSet(CGLOCK4);
			else
				GPIO_OutClear(CGLOCK4);
		}
	}

	#if(1 == USE_REAGENT_VAL_80)
	if (liquid_val == 80 )
	{
		liquid_mulnum = 0.8;
		mstp = 11;
	}else
	#endif
	if (liquid_val == 100 )
	{
		liquid_mulnum = 1;
		mstp = 13;
	}
	#if(1 == USE_REAGENT_VAL_80)
	if (liquid_val == 120 )
	{
		liquid_mulnum = 1.2;
		mstp = 18;
	}
	#endif
	else if (liquid_val == 150)
	{
		liquid_mulnum = 1.5;
		mstp = 20;
	}
	else if (liquid_val == 200)
		liquid_mulnum = 2;
	else perror("mb_mixdilutedab liquid_val");

	liquid_step_H2O2 = LIQUID_STEP * mulnum / (mulnum + 1) ;
	
	if (reagent_code[(uint8_t)mix__concentrate[0]].special_num == GREEN1 ||
			reagent_code[(uint8_t)mix__concentrate[0]].special_num == FR1)
		liquid_step_DAB = LIQUID_STEP / (mulnum + 1) ;
	else
		liquid_step_DAB = LIQUID_STEPDAB / (mulnum + 1);  //dab 为7ml小瓶子

	liquid_step_H2O2 = liquid_step_H2O2 * liquid_mulnum;
	liquid_step_DAB = liquid_step_DAB * liquid_mulnum;
	
	sysstep_H2O2 = SYSTEP_LIQUID * mulnum / (mulnum + 1) * MIX_SYSTEP_MOR * liquid_mulnum;	
	sysstep_DAB = SYSTEP_LIQUID / (mulnum + 1) * MIX_SYSTEP_MOR * liquid_mulnum;    //18

	for (i = 0; i < 6;i++)
	{	//发现当前混合瓶未清洗
		if (ordArray_mixed_DAB[i][0] == Mixing_Sation[0] && mixstation_clear_state[i].NEEDCLEAR)
		{
			if (mb_clearmixstation(&ordArray_mixed_DAB[i][0]) < 0)
				mb_seterrorcode(MIX_STATION_CLEARERROR);		
			mixstation_clear_state[i].NEEDCLEAR = false;
		}
	}
	
	index = 0;
	while(index < reagent_cnt)// 浓缩液和稀释液数量一一对应，位置可以不同一架
	{	
		AS_num = 1;  //  默认一个稀释剂瓶
		while((mix__diluent[index] == mix__diluent[index + 1]) && ((index + 1) < reagent_cnt))
		{     /* UPI （SN：） DAB的稀释剂，如果一架试剂架上的前后2个试剂瓶都是稀释剂则分2次2个瓶中各吸取所使用量的一半。  */
			AS_num++;   // 有2个稀释剂瓶
			index++;
		}
		printf("index = %d\n", index);
		ori_Array_diluent = Array_diluent = mix__diluent[index];
		AS_num *= mulliquid;
		
		printf("liquid_step_H2O2 = %f AS_num = %f Array_diluent=%d mulliquid=%d\n",
			liquid_step_H2O2, AS_num, Array_diluent, mulliquid);
		lprintf(log_my, INFO, "liquid_step_H2O2 = %f AS_num = %f Array_diluent = %d mulliquid = %d.\n",
			liquid_step_H2O2, AS_num, Array_diluent, mulliquid);
		index++;   //指向下个试剂

		//*********************************************吸取稀释液
		aspirate_val = (int)liquid_val * AS_num * mulnum / (1 + mulnum) / 2;
		
		printf("H2O2 aspirate_val = %d AS_num = %f mulnum = %f$$$$$$$\n", aspirate_val, AS_num, mulnum);
		lprintf(log_my, INFO, "H2O2 aspirate_val = %d AS_num = %f mulnum = %f$$$$$$$\n", aspirate_val, AS_num, mulnum);
		
		res_shelf = -1;// 探到底进行吸取 马上进行清洗
		mb_procwashprobe(REAGENT_WATER);
		res_shelf = 0;
		/*  到2个DAB稀释液瓶子里稀释液轮流吸取总量的各一半，如果只有一个稀释液的话就吸一瓶  */
		for (H2O2_cnt = 0; H2O2_cnt < 2; H2O2_cnt++)// 到两个瓶子吸取两次
		{
			CommandElemt.srdevaddr = PUMP_ADDR;
			sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED_AIR, PROBEPORT, SYSTEP_AIR_LAG);	//缓慢抽吸空气
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			usleep(ASPIRATE_T); 
			res_zx = DetectReagent(shelf_num, &Array_diluent, (int)(liquid_step_H2O2 * AS_num/2)  + LIQUID_MSTEP,  aspirate_val);

			if (ReportArrayData[0] != 0XFFFFFFFF)
				ReportArrayData[0] = serail_dataarr - (liquid_step_H2O2 * AS_num/2 + LIQUID_MSTEP) +CorrectDate[Array_diluent % 9];
			if (res_zx == -3)
				return;
			else if (res_zx < 0)
			{
				mb_seterrorcode(MIXA_DILUENT_WRONG + shelf_num);
			}

			memcpy(&TCP_Buffer[5], &aspirate_val, 4); 
			TCP_Buffer[9] = shelf_num;
			TCP_Buffer[0] = Array_diluent;
			#ifdef BIG_VERSION
			if(ReportArrayData[0] != 0XFFFFFFFF)
			ReportArrayData[0] =ReportArrayData[0] + OFFSET_STEP  - SHELF_OFFSET;
			#endif
			memcpy(&TCP_Buffer[1], &ReportArrayData[0], sizeof(ReportArrayData[0]));

			nt_sendpacketdata(REAGENT_SEND_INWORK, TCP_Buffer, sizeof(ReportArrayData[0]) + 1 + 4+1);

			CommandElemt.srdevaddr = PUMP_ADDR;
			printf("begin asp\n");
			sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID, (int)(sysstep_H2O2 * AS_num/2) ); //缓慢抽吸液体
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("mb_dabmixer]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			usleep(ASPIRATE_T);
			mb_monitdoorstate(); 
			CommandElemt.srdevaddr = ARM_ADDR;
			sprintf(CommandElemt.cmdbuf, "ZS -%d %d", (int)(liquid_step_H2O2 * AS_num/2)  + LIQUID_ZS_MOR+ LIQUID_MSTEP, LIQUID_SPEED);//缓慢上升
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 					CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			sleep(1);
			sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
					
			CommandElemt.srdevaddr = PUMP_ADDR;
			sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_AIR,SYSTEP_AIR);	//缓慢抽吸空气
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			usleep(ASPIRATE_T);
			mb_monitdoorstate();
			CommandElemt.srdevaddr = ARM_ADDR;
			sprintf(CommandElemt.cmdbuf, "PA %d %d %d", Mixing_Sation[0], Mixing_Sation[1], DAB_ZMAX);//混合站目标位置
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	
			CommandElemt.srdevaddr = PUMP_ADDR;
			sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED_DISPENSE,SYSTEP_AIR_LAG + (int)(sysstep_H2O2 * AS_num/2));//滴加试剂
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt); 
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			usleep(DISPENSE_T);
			CommandElemt.srdevaddr = ARM_ADDR;
			
		//	CommandElemt.srdevaddr = ARM_ADDR;
			sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 500, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
					
			sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			if (reagent_code[ori_Array_diluent - 1].special_num == reagent_code[ori_Array_diluent].special_num)
				Array_diluent = ori_Array_diluent -1;
			else if(reagent_code[ori_Array_diluent + 1].special_num == reagent_code[ori_Array_diluent].special_num)
				Array_diluent = ori_Array_diluent + 1;
			else
				Array_diluent = ori_Array_diluent;
		}	
		reagent_flag = Array_diluent;
		for (i = 0; i < 6;i++)
		{
			if (*Mixing_Sation == ordArray_mixed_DAB[i][0])
			{
				mixstation_clear_state[i].NEEDCLEAR = true;
		//		if (!NEED_CLEAR_SENDED)
				{
					TCP_Buffer[0] = 1;
			//		nt_sendpacketdata(MIXSTATION_NEED_CLEAR, TCP_Buffer, 1);
					NEED_CLEAR_SENDED = true;
				}
				if (shelf_num == 0 && !flg_opwork1ready)//防止在混合时PC端放弃不进行混合站清洗 
					mixstation_clear_state[i].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[i].AFTER_DISPENSE = false;
				
				if (shelf_num == 1 && !flg_opwork2ready)//防止在混合时PC端放弃不进行混合站清洗 
					mixstation_clear_state[i].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[i].AFTER_DISPENSE = false;

				if (shelf_num == 2 && !flg_opwork3ready)//防止在混合时PC端放弃不进行混合站清洗 
					mixstation_clear_state[i].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[i].AFTER_DISPENSE = false;
			
			}	
		}
	}
		//*********************************************吸取浓缩液		
	mb_procwashprobe(REAGENT_WATER);
	index = 0;
	while(index < reagent_cnt)// 浓缩液和稀释液数量一一对应，位置可以不同一架
	{	
		AS_num = 1;
		while((mix__concentrate[index] ==mix__concentrate[index + 1]) && ((index+1) < reagent_cnt))
		{
			AS_num++;
			index++;
		}
		printf("index = %d\n",index);
		Array_Reagent = mix__concentrate[index];

		AS_num *= mulliquid;
		
		printf("liquid_step_DAB = %f AS_num = %f Array_Reagent=%d mulliquid=%d\n",
			liquid_step_DAB, AS_num, Array_Reagent, mulliquid);
		lprintf(log_my, INFO,"liquid_step_DAB = %f AS_num = %f Array_Reagent=%d mulliquid=%d\n",
			liquid_step_DAB, AS_num, Array_Reagent, mulliquid);

		index++;//指向下个试剂
		aspirate_val = (int)liquid_val * AS_num /(1+ mulnum);

		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR_LAG);	//缓慢抽吸空气
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		usleep(ASPIRATE_T);
		
		res_zx = DetectReagent(shelf_num,&Array_Reagent, (int)(liquid_step_DAB * AS_num)  + LIQUID_MSTEP,  aspirate_val);		
		if (ReportArrayData[0] != 0XFFFFFFFF)	
			ReportArrayData[0] = serail_dataarr -((liquid_step_DAB * AS_num)  + LIQUID_MSTEP)  + CorrectDate[Array_Reagent % 9]; 
		if (res_zx == -3)
			return;
		else if (res_zx < 0)
			mb_seterrorcode(MIXA_DAB_WRONG + shelf_num);
	
		memcpy(&TCP_Buffer[5], &aspirate_val, 4);
		// printf("DAB aspirate_val=%d AS_num=%f mulnum=%df$$$$$$$\n",aspirate_val,AS_num,mulnum);
		lprintf(log_my, INFO, "DAB aspirate_val=%d AS_num=%f mulnum=%df$$$$$$$\n", aspirate_val, AS_num, mulnum);

		TCP_Buffer[0] = Array_Reagent;
		#ifdef BIG_VERSION
		if(ReportArrayData[0] != 0XFFFFFFFF)
			ReportArrayData[0] =ReportArrayData[0] + OFFSET_STEP  - SHELF_OFFSET;
		#endif
		memcpy(&TCP_Buffer[1],&ReportArrayData[0],sizeof(ReportArrayData[0]));
		TCP_Buffer[9] = shelf_num;
		
		nt_sendpacketdata(REAGENT_SEND_INWORK, TCP_Buffer, sizeof(ReportArrayData[0]) + 1 + 4+1);

		CommandElemt.srdevaddr = PUMP_ADDR;
		if (mulnum < 5)
			sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID ,(int)(sysstep_DAB * AS_num) ); //缓慢抽吸液体
		else
			sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID + 2,(int)(sysstep_DAB * AS_num) );	//缓慢抽吸液体
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		usleep(ASPIRATE_T + 2000000);
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d",(int)(liquid_step_DAB * AS_num) + LIQUID_ZS_MOR+ LIQUID_MSTEP, LIQUID_SPEED);//缓慢上升
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 					CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// 	else
		// 		printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// 	sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		sleep(1);
		sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
				
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_AIR,SYSTEP_AIR);	//缓慢抽吸空气
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		usleep(ASPIRATE_T);
		mb_monitdoorstate();
		CommandElemt.srdevaddr = ARM_ADDR;
		if(reagent_cnt >= 8)//8片
			sprintf(CommandElemt.cmdbuf, "PA %d %d %d", Mixing_Sation[0], Mixing_Sation[1], DAB_ZMAX - 400);//混合站目标位置
		else
			sprintf(CommandElemt.cmdbuf, "PA %d %d %d", Mixing_Sation[0], Mixing_Sation[1], DAB_ZMAX);//混合站目标位置
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dD%dR", SYSPEED_LIQUID ,SYSTEP_AIR_LAG +	(int)(sysstep_DAB * AS_num));//滴加试剂
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt); 
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		usleep(DISPENSE_T + 2000000);
		mb_monitdoorstate();
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 500, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
					
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	}		//  end of while(index < reagent_cnt)// 浓缩液和稀释液数量一一对应，位置可以不同一架	

	//*********************************************混合
	if(reagent_cnt >= 9)
		mixliqstp =  SYSTEP_LIQUID * 6 * liquid_val/100;
	else
		mixliqstp = SYSTEP_LIQUID * 5 * liquid_val/100;//大于5片使用

		//	mb_procwashprobe(REAGENT_WATER);
		//	mb_dab1wash();
	mb_monitdoorstate();
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", Mixing_Sation[0], Mixing_Sation[1], MOV_ZH);//混合站目标位置
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	//	sprintf(CommandElemt.cmdbuf, "S%dP%dI%dP%dR", SYSPEED,SYSTEP_AIR + 240,WATERPORT,SYHSTEP);	//缓慢抽吸空气拉部分水

	//	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED,PROBEPORT,SYSTEP_AIR + 240 + SYHHSTEP);	//缓慢抽吸空气
	if(reagent_cnt >= 5)
		sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED,PROBEPORT,SYSTEP_AIR + 240 );	//缓慢抽吸空气
	else
		sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED,PROBEPORT,SYSTEP_AIR + 240 );	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	mb_monitdoorstate(); 
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "ZA %d",  DAB_ZMAX);//Z轴下降
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	#if 0 //DE_TONG
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dO%dP%dM%dD%dM%dR", SYSPEED_MIXDAB, PROBEPORT,
		reagent_cnt * SYSTEP_LIQUID,500,reagent_cnt * SYSTEP_LIQUID,500); //抽吸6下
		//	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dD%dP%dD%dP%dD%dR", SYSPEED_MIXDAB, PROBEPORT,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP); //抽吸6下
		if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		else
			printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		sb_waitingframeaswer(&CommandElemt);
		/*		sprintf(CommandElemt.cmdbuf, "S%dP%dD%dP%dD%dP%dD%dR", SYSPEED_MIXDAB,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP);	//抽吸6下
		if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		else
			printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		sb_waitingframeaswer(&CommandElemt);
		*/	usleep(DISPENSE_T);
		mb_monitdoorstate();
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 500, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		//	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		else
			printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		sb_waitingframeaswer(&CommandElemt);
						
	#else
		//	reagent_cnt = 10;//test
	if ( reagent_cnt <= 5)//最多吸5片量
	{
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dO%dgP%dM%dD%dM%dG6R", SYSPEED_MIXDAB, PROBEPORT,
			reagent_cnt * mulliquid  * SYSTEP_LIQUID/2,500,reagent_cnt * mulliquid * SYSTEP_LIQUID/2,500); //抽吸6下
		// //	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dD%dP%dD%dP%dD%dR", SYSPEED_MIXDAB, PROBEPORT,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP,SYHHSTEP); //抽吸6下
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		usleep(DISPENSE_T);
		mb_monitdoorstate();
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 500, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	else
	{
		for(mix_cnt = 0; mix_cnt < 3;mix_cnt++)
		{	
			if(mix_cnt == 0)
			{
				sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR",	SYSPEED_MIXDAB, PROBEPORT,mixliqstp);	//抽吸6下
			}
			else
			{
				CommandElemt.srdevaddr = ARM_ADDR;
				sprintf(CommandElemt.cmdbuf, "ZA %d",  DAB_ZMAX);//Z轴下降
				// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
				// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
				// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
				// else
				// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
				// sb_waitingframeaswer(&CommandElemt);
				sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
				
				sprintf(CommandElemt.cmdbuf, "S%dO%dD%dM%dP%dR",	SYSPEED_MIXDAB, PROBEPORT,mixliqstp,500,mixliqstp);	//抽吸6下
			}
	
			CommandElemt.srdevaddr = PUMP_ADDR;
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
			
			CommandElemt.srdevaddr = ARM_ADDR;
			sprintf(CommandElemt.cmdbuf, "ZS -%d %d",  mstp * reagent_cnt * mulliquid*3/4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

			CommandElemt.srdevaddr = PUMP_ADDR;
			sprintf(CommandElemt.cmdbuf, "S%dO%dD%dM%dP%dR",
				SYSPEED_MIXDAB, PROBEPORT,mixliqstp,500,mixliqstp);	//抽吸6下
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		}	
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZA %d",  DAB_ZMAX);//Z轴下降
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dO%dD%dM%dgP%dM%dD%dM%dG3P%dR",
			SYSPEED_MIXDAB, PROBEPORT,mixliqstp,500,mixliqstp,500,mixliqstp,1500,mixliqstp);	//抽吸6下
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d",  mstp * reagent_cnt * mulliquid/2, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
					//scanf("%d", &i);
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dD%dM%dO%dgP%dM%dD%dM%dG3P%dR", 
				SYSPEED_MIXDAB,mixliqstp, 1000,PROBEPORT,mixliqstp,500,mixliqstp,500,mixliqstp);	//抽吸6下
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d", mstp * reagent_cnt*mulliquid / 4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt); 
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
				//scanf("%d", &i);
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dD%dM%dO%dgP%dM%dD%dM%dG3R", SYSPEED_MIXDAB,mixliqstp,1000, PROBEPORT,SYSTEP_LIQUID*2,500,SYSTEP_LIQUID*2,1500);	//抽吸6下
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr ); 
			// else
			// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
				
		usleep(DISPENSE_T);
		mb_monitdoorstate();
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 200, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// //	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", ZWASH_STEP4, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
			// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// else
			// 	printf("[mb_probewash_a]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}
	#endif

	usleep(DISPENSE_T); 		
	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH );//Z轴上升
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
	reagent_flag = Array_Reagent;

	reagent_check[Array_diluent / 9].NEED_MIX = false;
	reagent_check[Array_Reagent / 9].NEED_MIX = false;
	
	printf("mb_islockedreagent(0)=%d TRY_REAGENT_UNLOCK1=%d\n",mb_islockedreagent(0),TRY_REAGENT_UNLOCK1);
	for(i = 0; i < reagent_cnt;i++)
	{
		if ((mb_islockedreagent(0) == 0) && ((mix__concentrate[i] / 9) == 0 || (mix__diluent[i] / 9) == 0))
		{
			if (SHELF_LOCK_WAY_NEW)
				 GPIO_OutClear(CGLOCK1);
			else
				GPIO_OutSet(CGLOCK1);
		}
		else if ((mb_islockedreagent(1) == 0) && ((mix__concentrate[i] / 9) == 1|| (mix__diluent[i] / 9) == 1))
		{
			if (SHELF_LOCK_WAY_NEW)
				GPIO_OutClear(CGLOCK2);
			else
				GPIO_OutSet(CGLOCK2);
		}
		else if ((mb_islockedreagent(2) == 0) && ( (mix__concentrate[i] / 9) == 2|| (mix__diluent[i] / 9) == 2))
		{
			if (SHELF_LOCK_WAY_NEW)
				 GPIO_OutClear(CGLOCK3);
			else
				GPIO_OutSet(CGLOCK3);
		}
		else if ((mb_islockedreagent(3) == 0) && ((mix__concentrate[i] / 9) == 3|| (mix__diluent[i] / 9) == 3))
		{
			if (SHELF_LOCK_WAY_NEW)
				GPIO_OutClear(CGLOCK4);
			else
				 GPIO_OutSet(CGLOCK4);
		}
	}

	res_shelf = -1;
		mb_procwashprobe(Array_diluent);
	res_shelf = 0;

//	while(1) sleep(1);
}	


	/**********
	Dilute_num 为0 DAB, 1 RED
	*****/
int mb_searchcomatiblereagentval(char plate_num, unsigned char *ordArray_reagent, reagent_t *reagent, int reagent_val)
{
	int cmp_res=0;
	char TCP_Buffer[50] = {0};
	cmp_res = mb_searchcomatiblereagent(reagent, reagent_val);
	if (cmp_res < 0 )
		TCP_Buffer[23] = 2;	
	else
	{
		printf("cmp_res=%d\n", cmp_res);
		//减去对应的试剂量
		reagent_code[cmp_res].val -= reagent_val;
		memcpy(&TCP_Buffer[10], reagent_code[cmp_res].code, 9);
		*ordArray_reagent = cmp_res;
		if ((cmp_res & 0X80) > 0)//不同批号
			TCP_Buffer[23] = 1; 	
		cmp_res &= 0X7F;
	}
	TCP_Buffer[0] = plate_num;                  // 玻片位置
	memcpy(&TCP_Buffer[1], reagent->code, 9);   // 被替换试剂
	memcpy(&TCP_Buffer[19], &reagent_val, 4);   // 替换的试剂量
	nt_sendpacketdata(REPLACE_REAGENT, TCP_Buffer, 24);
						
	return cmp_res;
}

/****************
1.混合操作为当仪器处在一段空闲时间时去操作
2.离滴加混合时间不足10分钟时无空闲时也去?  僮?
3.当需要滴加关键步骤时暂停混合，等没有关键步骤时继续
step 为混合步骤全局量的地址
***************/
int mb_dabmixer(const operate_head_list* operate_head)
{
	unsigned int* Array_mixed_DAB;
	unsigned int* Array_mixed_SECOND;
	unsigned int liquid_val = 0;
	char which_mix_kind = 0;
	bool isDAB_mixed = false;
	bool isDAB_mixed_next = false;
	char mix_DAB_concentrate[10]={0},mix_DAB_diluent[10]={0};
	char mix_SECOND_concentrate[10]={0},mix_SECOND_diluent[10]={0};
	int i = 0, DAB_cnt = 0, SECOND_cnt = 0, mulnum=0,cmp_res=0,shelf_num;
	bool  isoperate_workingp = false;
	bool HAVE_DAB = false, HAVE_SECD = false;
	unsigned char* mix_stp = NULL;		 

	if (operate_head == NULL)
		return -1;

	if (operate_head == operate_head1)
	{
		which_mix_kind = which_mix_kindA;
		isDAB_mixed = isDAB_mixedA;
		isDAB_mixed_next = isDAB_mixedA_next;
		liquid_val = liquid_valA;
		mix_stp = &workstep_mix_a;
		shelf_num = 0;
		isoperate_workingp = flg_opwork1ready;
	}

	if (operate_head == operate_head2)
	{
		which_mix_kind = which_mix_kindB;
		isDAB_mixed = isDAB_mixedB;
		isDAB_mixed_next = isDAB_mixedB_next;
		liquid_val = liquid_valB;
		mix_stp = &workstep_mix_b;
		shelf_num = 1;
		isoperate_workingp = flg_opwork2ready;
	}
	
	if (operate_head == operate_head3)
	{
		which_mix_kind = which_mix_kindC;
			isDAB_mixed = isDAB_mixedC;
		isDAB_mixed_next = isDAB_mixedC_next;
		liquid_val = liquid_valC;
		mix_stp = &workstep_mix_c;
		shelf_num = 2;
		isoperate_workingp = flg_opwork3ready;
	}

	printf("in  mb_dabmixer\n");
	
	for(i = 0; i < 10; i++)
	{
		printf(" %d ", mix_DAB[shelf_num].ordArrayA[i]);
		
		printf("reagentA code=%s kind=%s \n", mix_DAB[shelf_num].reagentA[i].code, mix_DAB[shelf_num].reagentA[i].reagent_kind);
		printf(" %d ", mix_DAB[shelf_num].ordArrayB[i]);
		printf("reagentB code=%s kind=%s \n", mix_DAB[shelf_num].reagentB[i].code, mix_DAB[shelf_num].reagentB[i].reagent_kind);		
	}

	for (i = 0; i < 10;i++)
	{
		printf("i=%d\n",i);
		if(mix_DAB[shelf_num].ordArrayA[i] != 0 && isDAB_mixed == false)
		{
			HAVE_DAB = true;
			if ((mix_DAB[shelf_num].ordArrayA[i] & 0X80) > 0)
			{
				if ((cmp_res = mb_searchcomatiblereagentval(shelf_num*10+i,&mix_DAB[shelf_num].ordArrayA[i],&mix_DAB[shelf_num].reagentA[i],liquid_val  /(mulnum+1))) < 0 )
					continue;
			}	
			mix_DAB_concentrate[DAB_cnt] = mix_DAB[shelf_num].ordArrayA[i];
		}
		
		if(mix_DAB[shelf_num].ordArrayB[i] != 0 && isDAB_mixed == false)
		{
			if ((mix_DAB[shelf_num].ordArrayB[i] & 0X80) > 0)
			{
				if ((cmp_res = mb_searchcomatiblereagentval(shelf_num*10+i,&mix_DAB[shelf_num].ordArrayB[i],&mix_DAB[shelf_num].reagentB[i],liquid_val* mulnum /(mulnum+1))) < 0 )
					continue;
			}
			mix_DAB_diluent[DAB_cnt] = mix_DAB[shelf_num].ordArrayB[i];
			DAB_cnt++;
		}

		if(mix_SECEND[shelf_num].ordArrayA[i] != 0)
		{
			HAVE_SECD = true;
			if ((mix_SECEND[shelf_num].ordArrayA[i] & 0X80) > 0)
			{
				if ((cmp_res = mb_searchcomatiblereagentval(shelf_num*10+i,&mix_SECEND[shelf_num].ordArrayA[i],&mix_SECEND[shelf_num].reagentA[i],liquid_val  /(mulnum+1))) < 0 )
					continue;
			}
			mix_SECOND_concentrate[SECOND_cnt] = mix_SECEND[shelf_num].ordArrayA[i];
		}
		
		if(mix_SECEND[shelf_num].ordArrayB[i] != 0)
		{
			if ((mix_SECEND[shelf_num].ordArrayB[i] & 0X80) > 0)
			{
				if ((cmp_res = mb_searchcomatiblereagentval(shelf_num*10+i,&mix_SECEND[shelf_num].ordArrayB[i],&mix_SECEND[shelf_num].reagentB[i],liquid_val* mulnum /(mulnum+1))) < 0 )
					continue;
			}
			mix_SECOND_diluent[SECOND_cnt] = mix_SECEND[shelf_num].ordArrayB[i];
			SECOND_cnt++;
		}	
	}
	
	if (DAB_cnt == 0 && SECOND_cnt == 0) //流程中没有需混合的试剂
	{
		if (HAVE_DAB || HAVE_SECD)
		{
			if (shelf_num == 0)
				mb_seterrorcode(SHELF1_ABANDON);
			else if (shelf_num == 1)
				mb_seterrorcode(SHELF2_ABANDON);
			else if (shelf_num == 2)
				mb_seterrorcode(SHELF3_ABANDON);
			return -3;
		}
	}
		//*///////////////////////////////////
	printf("yyyyyyyyyy\n\n");
					
	if ((operate_head->operate.plate_num % 30) < 10)
	{	
		liquid_val = liquid_valA;
		if (DAB1Array_exchange  == false)
		{
			Array_mixed_DAB1[0] = ordArray_mixed_DAB[0][0];
			Array_mixed_DAB1[1] = ordArray_mixed_DAB[0][1];
			Array_mixed_RED1[0] = ordArray_mixed_DAB[1][0];
			Array_mixed_RED1[1] = ordArray_mixed_DAB[1][1];
			Array_mixed_DAB = (unsigned int *)ordArray_mixed_DAB;
			Array_mixed_SECOND = (unsigned int *)ordArray_mixed_DAB + 2;
		}
		else
		{
			Array_mixed_DAB1[0] = ordArray_mixed_DAB[1][0];
			Array_mixed_DAB1[1] = ordArray_mixed_DAB[1][1];
			Array_mixed_RED1[0] = ordArray_mixed_DAB[0][0];
			Array_mixed_RED1[1] = ordArray_mixed_DAB[0][1];
			Array_mixed_DAB = (unsigned int *)ordArray_mixed_DAB + 2;
			Array_mixed_SECOND = ordArray_mixed_DAB[0];		
		}
	}
	else if ((operate_head->operate.plate_num % 30) < 20)
	{
		liquid_val = liquid_valB;
		if (DAB2Array_exchange  == false)
		{
			Array_mixed_DAB2[0] = ordArray_mixed_DAB[2][0];
			Array_mixed_DAB2[1] = ordArray_mixed_DAB[2][1];
			Array_mixed_RED2[0] = ordArray_mixed_DAB[3][0];
			Array_mixed_RED2[1] = ordArray_mixed_DAB[3][1];
			Array_mixed_DAB = (unsigned int *)ordArray_mixed_DAB + 4;
			Array_mixed_SECOND = (unsigned int *)ordArray_mixed_DAB + 6;	
		}
		else
		{
			Array_mixed_DAB2[0] = ordArray_mixed_DAB[3][0];
			Array_mixed_DAB2[1] = ordArray_mixed_DAB[3][1];
			Array_mixed_RED2[0] = ordArray_mixed_DAB[2][0];
			Array_mixed_RED2[1] = ordArray_mixed_DAB[2][1];
			Array_mixed_DAB = (unsigned int *)ordArray_mixed_DAB + 6;
			Array_mixed_SECOND = (unsigned int *)ordArray_mixed_DAB + 4;
		}
	}
	else
	{
		liquid_val = liquid_valC;
		if (DAB3Array_exchange  == false)
		{
			Array_mixed_DAB3[0] = ordArray_mixed_DAB[4][0];
			Array_mixed_DAB3[1] = ordArray_mixed_DAB[4][1];
			Array_mixed_RED3[0] = ordArray_mixed_DAB[5][0];
			Array_mixed_RED3[1] = ordArray_mixed_DAB[5][1];
			Array_mixed_DAB = (unsigned int *)ordArray_mixed_DAB + 8;
			Array_mixed_SECOND = (unsigned int *)ordArray_mixed_DAB + 10;
		}
		else
		{
			Array_mixed_DAB3[0] = ordArray_mixed_DAB[5][0];
			Array_mixed_DAB3[1] = ordArray_mixed_DAB[5][1];
			Array_mixed_RED3[0] = ordArray_mixed_DAB[4][0];
			Array_mixed_RED3[1] = ordArray_mixed_DAB[4][1];
			Array_mixed_DAB = (unsigned int *)ordArray_mixed_DAB + 10;
			Array_mixed_SECOND = (unsigned int *)ordArray_mixed_DAB + 8;
		}
	}
		
	printf("xxxxxxxxxxx\n\n");

	pthread_mutex_lock(&mutex_mianarmlock);		
	if (which_mix_kind == 1)		
		mb_mixdilutedab( mix_DAB_concentrate, mix_DAB_diluent, DAB_cnt, liquid_val, mulnum_glb[shelf_num],mix_DAB[shelf_num].tep_num / DAB_cnt,  Array_mixed_DAB, shelf_num);
	else	
		mb_mixdilutedab( mix_SECOND_concentrate, mix_SECOND_diluent, SECOND_cnt, liquid_val, mulnum_glb2[shelf_num],mix_SECEND[shelf_num].tep_num / SECOND_cnt, Array_mixed_SECOND, shelf_num);
	
	pthread_mutex_unlock(&mutex_mianarmlock);

	last_cabin_reagent = REAGENT_WATER;//

	if (operate_head == operate_head1)
		startmixworkid &= 0X7F;

	if (operate_head == operate_head2)
		startmixworkid &= 0XBF;
	
	if (operate_head == operate_head3)
		startmixworkid &= 0XDF;
	if (isoperate_workingp)
	{
		if (which_mix_kind == 1)
			isDAB_mixed = true;
		else	
			isDAB_mixed_next = true;
	}
		
	//	tp_washchange(FALSE);
	if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mainarm_lock");

	return 0;
}



bool mb_isspecialreagent(unsigned char reagent_num)
{
	printf("[mb_isspecialreagent]reagent_num = %d\n", reagent_num);

	if (reagent_code[reagent_num].special_num == 1 || reagent_code[reagent_num].special_num == 9
		|| reagent_code[reagent_num].special_num == 10)
		return true;	
	
	return false;	
}


int mb_procwashprobe(unsigned char reagent_num)
{
	unsigned char liquid_port = 0,last_cabin_port = 0;
	bool DischargeLock_NEED_UNLOCK = false;//确保只进行一次解锁

	if (wkevent == INIT_WORK)//初始化时灌注不进行清洗
		return 0;
	
	if (reagent_num == REAGENT_WATER)
		liquid_port = WATERPORT;
	else if (reagent_num == REAGENT_WASH)
		liquid_port = WASHPORT;
	else if (reagent_num == REAGENT_ER1)
		liquid_port = ER1PORT;
	else if (reagent_num == REAGENT_ER2)
		liquid_port = ER2PORT;	
	else if (reagent_num == REAGENT_DEWAX)
		liquid_port = DEWAXPORT;	
	else if (reagent_num == REAGENT_ALCOHOL)
		liquid_port = ALCOHOLPORT;
	else
	{
		//试剂架上试剂 或DDAB
	}
	if (last_cabin_reagent == REAGENT_WATER)
		last_cabin_port = WATERPORT;
	else if (last_cabin_reagent == REAGENT_WASH)
		last_cabin_port = WASHPORT;
	else if (last_cabin_reagent == REAGENT_ER1)
		last_cabin_port = ER1PORT;
	else if (last_cabin_reagent == REAGENT_ER2)
		last_cabin_port = ER2PORT;	
	else if (last_cabin_reagent == REAGENT_DEWAX)
		last_cabin_port = DEWAXPORT;	
	else if (last_cabin_reagent == REAGENT_ALCOHOL)
		last_cabin_port = ALCOHOLPORT;
		
	 //大容量且与上次使用相同不清晰
	if ((reagent_num == reagent_flag) && (reagent_num >= REAGENT_CASE) && (reagent_num <= REAGENT_ER2))
	{
		/*	pthread_mutex_lock(&mutex_dischargelock);
			tp_washchange(TRUE);
			mb_probewash_c(liquid_port);
			tp_washchange(FALSE);
			pthread_mutex_unlock(&mutex_dischargelock);
		*/		
		printf("same reagent\n");
		return 0;
	}
	if (!flg_lockdischargliquit)
	{
		pthread_mutex_lock(&mutex_dischargelock);
		flg_lockdischargliquit = true;
		DischargeLock_NEED_UNLOCK = true;
	}

	if (res_shelf < 0)
	{
		tp_washchange(true);
		mb_probewash_a(last_cabin_port);
		tp_washchange(false);
		goto WashEnd;
	}

	printf("[mb_procwashprobe]reagent_num=%d reagent_flag=%d last_cabin_reagent=%d\n",
	reagent_num,reagent_flag,last_cabin_reagent);
	
	if ((reagent_num >= REAGENT_CASE) && (reagent_num < REAGENT_DAB))	//大于REAGENT_CASE为大容量试剂小于为架试剂试剂  REAGENT_CASE值在小仪器不同
	{	
		printf("大容量试剂清洗灌注\n");	
			
		tp_washchange(true);
		switch (last_cabin_reagent)	//前次使用的试剂
		{
			case REAGENT_WATER:
				if (liquid_port == DEWAXPORT)
					mb_probewash_b(ALCOHOLPORT);	
				break;
			case REAGENT_WASH:
				if (liquid_port == DEWAXPORT)
					mb_probewash_b(ALCOHOLPORT);	
				break;	
			case REAGENT_ALCOHOL:
				if ((liquid_port == ER1PORT) || (liquid_port == ER2PORT))
					mb_probewash_a(WATERPORT);	
				break;
			case REAGENT_ER1:
				if (liquid_port == ALCOHOLPORT)
					mb_probewash_a(WATERPORT);	
				else if (liquid_port == DEWAXPORT)
				{
					mb_probewash_a(WATERPORT);	
					mb_probewash_b(ALCOHOLPORT);	
				}
				break;	
			case REAGENT_ER2:
			if (liquid_port == ALCOHOLPORT)
				mb_probewash_a(WATERPORT);	
			else if (liquid_port == DEWAXPORT)
			{
				mb_probewash_a(WATERPORT);	
				mb_probewash_b(ALCOHOLPORT);	
			}
			break;	
			case REAGENT_DEWAX:
				if ((liquid_port == WATERPORT) || (liquid_port == WASHPORT))
					mb_probewash_b(ALCOHOLPORT);	
				else if ((liquid_port == ER1PORT) || (liquid_port == ER2PORT))
				{
					mb_probewash_b(ALCOHOLPORT);	
					mb_probewash_a(WATERPORT);	
				}
				break;	
			default: 
				break;
		}
		
		if ((reagent_num >= REAGENT_CASE) && (reagent_num <= REAGENT_WASH) )//前一步骤是大容量试剂进行灌注清洗
		{
			if(last_cabin_reagent != reagent_num)//WATER WASH 切换
				mb_probewash_a(liquid_port);
			else
				mb_probewash_c(last_cabin_port);//同种
		}
		else if ((reagent_num > REAGENT_WASH) && (reagent_num <= REAGENT_ER2) )
		{
			mb_probewash_b(liquid_port);			
		}
		else //前一步骤是试剂架试剂或者DAB
		{		//前一步骤是试剂架试剂或者DAB 则管路中的大容量必定是WATER 或 WASH 可直接对下次的大容量直接灌注
			if (last_cabin_reagent != reagent_num)//前次的大容量试剂不同进行灌注清洗
			{	
				if ((reagent_num >= REAGENT_CASE) && (reagent_num <= REAGENT_WASH) )//前一步骤是大容量试剂进行灌注清洗
					mb_probewash_a(liquid_port);
				else if ((reagent_num > REAGENT_WASH) && (reagent_num <= REAGENT_ER2) )
					mb_probewash_b(liquid_port);	
			}
			else	//
			{			
				mb_probewash_c(last_cabin_port);
			}
		}
		last_cabin_reagent = reagent_flag = reagent_num;
		tp_washchange(false);
	}
	else if (mb_isspecialreagent(reagent_num))
	{
		printf("抗体清洗\n");

		if (reagent_flag < REAGENT_WATER || reagent_flag >= REAGENT_DAB)//上次步骤为试剂架上试剂 或DAB
		{
			tp_washchange(true);
			mb_probewash_d(last_cabin_port);
			tp_washchange(false);
		}
		else if (reagent_flag != REAGENT_WATER && reagent_flag != REAGENT_WASH)//水和缓冲液不清洗
		{	
			if (reagent_flag == REAGENT_DEWAX)	//
			{
				tp_washchange(true);
				mb_probewash_b(ALCOHOLPORT);
				mb_probewash_a(WATERPORT);
				last_cabin_reagent = REAGENT_WATER;
				tp_washchange(false);
			}
			else if (reagent_flag == REAGENT_ALCOHOL || reagent_flag == REAGENT_ER1 
				|| reagent_flag == REAGENT_ER2)
			{
				tp_washchange(true);
				mb_probewash_a(WATERPORT);
				last_cabin_reagent = REAGENT_WATER;
				tp_washchange(false);
			}	
		}	
	}
	else	//其他试剂架试剂或者DAB
	{
		printf("其他试剂架上试剂 或DAB\n");
		
		if ( (reagent_flag < REAGENT_WATER || reagent_flag >= REAGENT_DAB) && reagent_num != reagent_flag)//上次步骤为试剂架上试剂 或DAB
		{
			tp_washchange(true);
			mb_probewash_c(last_cabin_port);
			tp_washchange(false);
		}
		else if (reagent_flag != REAGENT_WATER && reagent_flag != REAGENT_WASH)//水和缓冲液不清洗
		{	
			if (reagent_flag == REAGENT_DEWAX)	//
			{
				tp_washchange(true);
				mb_probewash_b(ALCOHOLPORT);
				mb_probewash_a(WATERPORT);
				last_cabin_reagent = REAGENT_WATER;
				tp_washchange(false);
			}
			else if (reagent_flag == REAGENT_ALCOHOL || reagent_flag == REAGENT_ER1 
				|| reagent_flag == REAGENT_ER2)
			{
				tp_washchange(true);
				mb_probewash_a(WATERPORT);
				last_cabin_reagent = REAGENT_WATER;
				tp_washchange(false);
			}	
		}
	}
	WashEnd:

	if (DischargeLock_NEED_UNLOCK)
	{
		if (pthread_mutex_unlock(&mutex_dischargelock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_dischargelock");
		flg_lockdischargliquit = false;
	}
	printf("\n");
	return 0;
}


/******************************************************************************
*
* Function Name  : mb_shelfaspirate
* Description    : 吸取试剂架试剂
* 					 
* Input		   :const unsigned char reagent_num 试剂位置, const unsigned char same_num 吸取的玻片数
				, unsigned char liquid_val每片玻片滴加量
* Output		   :
* Return		   :  BOOL 错误FLASE 正确TRUE
*******************************************************************************/
int mb_shelfaspirate(const unsigned char reagent_num, const unsigned char same_num, 
						const unsigned int liquid_val, const unsigned char shelf_num)
{
	sr_cmdstruct_t CommandElemt;
	int res = 0;
	unsigned int liquid_step = 0, sysstep = 0,liquid_zmaxstep = 0;
	int res_zx = 0,aspirate_val = 0;
	char TCP_Buffer[50] = {0};
	memset(ReportArrayData,0,sizeof(ReportArrayData));
	
	printf("in mb_shelfaspirate\n");
	lprintf(log_my, INFO,"in mb_shelfaspirate\n");
	if (reagent_code[reagent_num].special_num == 1 || reagent_code[reagent_num].special_num == EBER6)//开放试剂 配置文件中读取 开放试剂瓶的截面积计算
	{
		#if(1 == USE_REAGENT_VAL_80)
		if (liquid_val == 80)
		{
			liquid_step = open_armstep * 4/5;
			sysstep = SYSTEP_LIQUID * 4/5;
		}else 
		#endif
		if (liquid_val == 100)
		{
			liquid_step = open_armstep;
			sysstep = SYSTEP_LIQUID;
		}
		#if(1 == USE_REAGENT_VAL_80)
		if (liquid_val == 120)
		{
			liquid_step = open_armstep * 6/5;
			sysstep = SYSTEP_LIQUID * 6/5;
		}
		#endif
		else if (liquid_val == 150)
		{
			liquid_step =  open_armstep * 3 / 2;
			sysstep = SYSTEP_LIQUID * 3 / 2;
		}
		else if (liquid_val == 200)
		{
			liquid_step =  open_armstep * 2;
			sysstep = SYSTEP_LIQUID *  2;
		}

		liquid_zmaxstep = OPEN_ARMZMAX;
	}
	else if(reagent_code[reagent_num].special_num == 3 || reagent_code[reagent_num].special_num == 6 ||
		reagent_code[reagent_num].special_num == FREE_REAGENT2 || reagent_code[reagent_num].special_num == EBER7 )//DAB浓缩液和清洗系统,7ml开放试剂
	{
		#if(1 == USE_REAGENT_VAL_80)
		if (liquid_val == 80)
		{
			liquid_step = open_armstep2 * 4/5;
			sysstep = SYSTEP_LIQUID * 4/5;
		}else 
		#endif
		if (liquid_val == 100)
		{
			liquid_step = open_armstep2;
			sysstep = SYSTEP_LIQUID;
		}
		#if(1 == USE_REAGENT_VAL_80)
		if (liquid_val == 120)
		{
			liquid_step = open_armstep2 * 6/5;
			sysstep = SYSTEP_LIQUID * 6/5;
		}
		#endif
		else if (liquid_val == 150)
		{
			liquid_step =  open_armstep2 * 3 / 2;
			sysstep = SYSTEP_LIQUID * 3 / 2;
		}
		else if (liquid_val == 200)
		{
			liquid_step =  open_armstep2 * 2;
			sysstep = SYSTEP_LIQUID *  2;
		}
		
		liquid_zmaxstep = OPEN_ARMZMAX;
	}
	else 
	{ 
		#if(1 == USE_REAGENT_VAL_80)
		if (liquid_val == 80)
		{
			liquid_step = LIQUID_STEP * 4/5;
			sysstep = SYSTEP_LIQUID * 4/5;
		}else 
		#endif
		if (liquid_val == 100)
		{
			liquid_step = LIQUID_STEP;
		
			sysstep = SYSTEP_LIQUID;
		}
		#if(1 == USE_REAGENT_VAL_80)
		if (liquid_val == 120)
		{
			liquid_step = LIQUID_STEP * 6/5;
			sysstep = SYSTEP_LIQUID * 6/5;
		}
		#endif
		else if (liquid_val == 150)
		{
			liquid_step = LIQUID_STEP * 3 / 2;
			
			sysstep = SYSTEP_LIQUID * 3 / 2;
		}
		else if (liquid_val == 200)
		{
			liquid_step = LIQUID_STEP * 2;
			
			sysstep = SYSTEP_LIQUID * 2;
		}
		
		liquid_zmaxstep = LIQUID_ZMAX;
	}

	if (liquid_val > 200)
	{
		liquid_step =  LIQUID_STEP * liquid_val / 100;
		sysstep = SYSTEP_LIQUID *  liquid_val / 100;
	}
	
	liquid_step = liquid_step * SYSTEP_MOR;

	printf("liquid_val=%d liquid_step=%d sysstep=%d\n",liquid_val,liquid_step,sysstep);
	lprintf(log_my, INFO,"liquid_val=%d liquid_step=%d sysstep=%d\n",liquid_val,liquid_step,sysstep);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR_LAG);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_reagent[reagent_num][0], ordArray_reagent[reagent_num][1], MOV_ZH);//到达试剂位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	mb_monitdoorstate();
	sprintf(CommandElemt.cmdbuf, "ZA %d", 350 - OFFSET_STEP+SHELF_OFFSET);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(300000);			
	flg_getedserailarr = TRUE;
	sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", liquid_step * same_num + LIQUID_MSTEP, LIQUID_DISTANCE_ZMAX, liquid_zmaxstep);//测液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// res_zx = sb_waitingframeaswer(&CommandElemt);
	res_zx = sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	if (res_zx == -3)
	{
		res = -3;
	}	
	else if (res_zx == -9 || res_zx == -11)
	{
		flg_getedserailarr = TRUE;
		res_shelf = -1;
		sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", liquid_step * same_num + LIQUID_MSTEP, LIQUID_DISTANCE_ZMAX,1+ liquid_zmaxstep);//测液体
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// if (sb_waitingframeaswer(&CommandElemt) < 0)
		if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
		{
			res = res_zx;
			ReportArrayData[0] = 0XFFFFFFFF;//没测到		
		}
		if((serail_dataarr+liquid_step * same_num + LIQUID_MSTEP)  > liquid_zmaxstep)//机械臂测量不足没有报警
		{
			ReportArrayData[0] = 0XFFFFFFFF;//没测到		
		}
	}
	else
	{
		printf("Arraydata=%d\n",serail_dataarr);
		lprintf(log_my, INFO,"Arraydata=%d\n",serail_dataarr);
		if (serail_dataarr < (500 - OFFSET_STEP+SHELF_OFFSET))
		{
			ReportArrayData[0] = 1;//发送最大满瓶
			res = -9;
		}
		ReportArrayData[0] = serail_dataarr -(liquid_step * same_num + LIQUID_MSTEP) + CorrectDate[reagent_num % 9];
		printf("ReportArrayData[0]=%x ",ReportArrayData[0]);
	}

	if(res_zx == -9)
	{

	}	
	else if(res_zx == -11)
	{
		flg_getedserailarr = TRUE;
		sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", 0, LIQUID_DISTANCE_ZMAX, 1306);//测液体
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dabmixer]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// if (sb_waitingframeaswer(&CommandElemt) == -9)//已预留打底量测液体
		if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) == -9)
		{
			res = -9;
		}
		else
		{
			lprintf(log_my, INFO,"aspirate_notenough Arraydata=%d\n",serail_dataarr);
			printf("aspirate_notenough Arraydata=%d\n", serail_dataarr);
			aspirate_notenough[10] = (1306 - serail_dataarr - CorrectDate[reagent_num % 9]) / liquid_step ;
			if (aspirate_notenough[10] >= same_num)//按1306测量满足所需量时按只有一片不足进行报警
				aspirate_notenough[10] = same_num - 1;
		}			
	}

	TCP_Buffer[0] = reagent_num;
	aspirate_val = liquid_val * same_num;
	memcpy(&TCP_Buffer[1],&ReportArrayData[0],sizeof(ReportArrayData[0]));
	memcpy(&TCP_Buffer[5],&aspirate_val,4);
	TCP_Buffer[9] = shelf_num;
	nt_sendpacketdata(REAGENT_SEND_INWORK, TCP_Buffer, sizeof(ReportArrayData[0]) + 1 + 4 + 1);
	reagent_code[reagent_num].val -= aspirate_val;
	report_reagentinfo[shelf_num] = reagent_num;

	if (ReportArrayData[0] == 0XFFFFFFFF || res == -9)
	{
		CommandElemt.srdevaddr = ARM_ADDR;
		// sprintf(CommandElemt.cmdbuf, "ZA %d", liquid_zmaxstep);
		// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}

	CommandElemt.srdevaddr = PUMP_ADDR;
	if(reagent_code[reagent_num].special_num == EBER6 || reagent_code[reagent_num].special_num == EBER7
			|| reagent_code[reagent_num].special_num == EBER30)
		sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUIDEBER,(sysstep * SYSTEP_MOR)  * same_num);	//缓慢抽吸液体
	else
		sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID,(sysstep * SYSTEP_MOR)  * same_num);	//缓慢抽吸液体

	// //		sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID,(syss tep * SYSTEP_MOR) * same_num + SYSTEP_LIQUID_REMAIN);	//缓慢抽吸液体		
	// //	sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID,(sysstep ) * same_num + SYSTEP_LIQUID_REMAIN);	//缓慢抽吸液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	if(reagent_code[reagent_num].special_num == EBER6 || reagent_code[reagent_num].special_num == EBER7
			|| reagent_code[reagent_num].special_num == EBER30)
		usleep(ASPIRATE_T + 2000000);	//使注射器中液体平衡
	else
		usleep(ASPIRATE_T);	//使注射器中液体平衡

	CommandElemt.srdevaddr = ARM_ADDR;
	if (res >= 0)
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", liquid_step * same_num + LIQUID_ZS_MOR + LIQUID_MSTEP, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
	else
	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 500, LIQUID_SPEED); //缓慢上升 当>500步时z轴回到500步位置

	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	return res;
}

/******************************************************************************
*
* Function Name  : mb_aspiratedab
* Description    : 吸取试剂混合站
* 					 
* Input		   :unsigned int* Array_DAB 混合站坐标, unsigned char num 吸取的玻片数
				, unsigned char liquid_val每片玻片滴加量
* Output		   :
* Return		   :  BOOL 错误FLASE 正确TRUE
*******************************************************************************/
bool mb_aspiratedab(unsigned int* dabarray , unsigned char num, unsigned char liquid_val)
{
	sr_cmdstruct_t CommandElemt;
	bool res = true;
	unsigned int liquid_step = 0, sysstep = 0;
	
	#if(1 == USE_REAGENT_VAL_80)
	if (liquid_val == 80)
	{
		liquid_step = LIQUID_STEP_MIX_STATION * 4/5;
		sysstep = SYSTEP_LIQUID*4/5;
	}
	#endif
	if (liquid_val == 100)
	{
		liquid_step = LIQUID_STEP_MIX_STATION;
		sysstep = SYSTEP_LIQUID;
	}
	#if(1 == USE_REAGENT_VAL_80)
	if (liquid_val == 120)
	{
		liquid_step = LIQUID_STEP_MIX_STATION * 6/5;
		sysstep = SYSTEP_LIQUID * 6/5;
	}
	#endif
	else if(liquid_val == 150)
	{
		liquid_step = LIQUID_STEP_MIX_STATION * 3 / 2;
		sysstep = SYSTEP_LIQUID * 3 / 2;
	}
	else if(liquid_val == 200)
	{
		liquid_step = LIQUID_STEP_MIX_STATION * 2;
		sysstep = SYSTEP_LIQUID * 2;
	}
	liquid_step = liquid_step * SYSTEP_MOR;
		
	//	sysstep = sysstep * SYSTEP_DAB_DISPENS_PEC;
	//	liquid_step = liquid_step * SYSTEP_MOR;
	sysstep = (sysstep * SYSTEP_DABMOR);
	lprintf(log_my, INFO, "mb_aspiratedab\n");

	//mb_monitdoorstate();
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dP%dR", SYSPEED_AIR, PROBEPORT, SYSTEP_AIR_LAG);	//缓慢抽吸空气		
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);	
	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", dabarray[0], dabarray[1], MOV_ZH);//到达试剂位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	mb_monitdoorstate();
	//sprintf(CommandElemt.cmdbuf, "ZA %d", 350);
	#if (S600_WRONING_ASPIRATE_WRONG == 1)
		sprintf(CommandElemt.cmdbuf, "ZA %d", 320);
	#else
		sprintf(CommandElemt.cmdbuf, "ZA %d", 350);
	#endif
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(300000);
	
	//	sprintf(CommandElemt.cmdbuf, "ZA %d", DAB_ZMAX);//最底下吸排除气泡影响
	#if (S600_WRONING_ASPIRATE_WRONG == 1)   // 用卡尺量探針上到頂后量到混合杯的距离50mm，减去杯5mm，45/0.03mm = 1500 
		sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", 0, 0, 1500);//测液体     dab_zmax= 915 2024i年参展机吸取混合好的试剂时报错22，改为945后报错更明显
	#else
		sprintf(CommandElemt.cmdbuf, "ZX %d %d %d", 0, 0, DAB_ZMAX - 4);//测液体
	#endif
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// if (sb_waitingframeaswer(&CommandElemt) < 0)
	if(sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1) < 0)
	{
		res = false;
	}
	sprintf(CommandElemt.cmdbuf, "ZA %d", DAB_ZMAX);//最底下吸排除气泡影响
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	//sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID,(sysstep *108 /100) * num);	//缓慢抽吸液体
	sprintf(CommandElemt.cmdbuf, "S%dP%dR", SYSPEED_LIQUID, sysstep  * num);	//缓慢抽吸液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	usleep(ASPIRATE_T);	//使注射器中液体平衡
	CommandElemt.srdevaddr = ARM_ADDR;

	usleep(ASPIRATE_T);

	sprintf(CommandElemt.cmdbuf, "ZS -%d %d", DAB_ZMAX - MOV_ZH - 100, LIQUID_SPEED);//缓慢上升
	// //	sprintf(CommandElemt.cmdbuf, "ZA%d", 0);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_aspiratedab]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH);
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_clearmixstation]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
			
	return res;
}

/******************************************************************************
*
* Function Name  : CabinAspirate
* Description    : 吸取试剂大容量
* 					 
* Input		   :unsigned int reagent_num 试剂信息, unsigned char num 吸取的玻片数
				, unsigned char liquid_val每片玻片滴加量
* Output		   :
* Return		   :  None
*******************************************************************************/
void CabinAspirate(const unsigned int reagent_num ,unsigned char num, unsigned int liquid_val)
{
	sr_cmdstruct_t CommandElemt;
	int reagent_port = 0;
	unsigned int  sysstep = 0;
	int sysspd = SYSPEED;

	printf("in CabinAspirate\n");
	
	if (reagent_num == REAGENT_WATER)
		reagent_port = WATERPORT;
	else if (reagent_num == REAGENT_WASH)
		reagent_port = WASHPORT;
	else if (reagent_num == REAGENT_ER1)
		reagent_port = ER1PORT;
	else if (reagent_num == REAGENT_ER2)
		reagent_port = ER2PORT;	
	else if (reagent_num == REAGENT_DEWAX)
		reagent_port = DEWAXPORT;	
	else if (reagent_num == REAGENT_ALCOHOL)
	{
		reagent_port = ALCOHOLPORT;
		sysspd = SYSPEED_ALCHOLE;
	}
	
	sysstep = SYSTEP_LIQUID * liquid_val / 100;
	/************清洗步骤就是大容量灌注 如为试剂滴加中有WEATER替代清洗时亦灌注*************/
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED,reagent_port,(sysstep) * num);	//抽液体
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[CabinAspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[CabinAspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	usleep(ASPIRATE_T);	//使注射器中液体平衡
}

void DispenseSpeedChange(short speed)
{
	sr_cmdstruct_t CommandElemt;
	
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "v%dR", speed);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
}


void CutoffSpeedChange(short speed)
{
	sr_cmdstruct_t CommandElemt;
	
	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "c%dR", speed);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
}


void DispenseRemain(void)
{
	sr_cmdstruct_t CommandElemt;

	CommandElemt.srdevaddr = ARM_ADDR;
	sprintf(CommandElemt.cmdbuf, "PA %d %d %d", ordArray_wash[WASH1][0], ordArray_wash[WASH1][1], ZWASH_STEP1);// 1号清洗槽
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = PUMP_ADDR;
	sprintf(CommandElemt.cmdbuf, "S%dO%dA%dR", SYSPEED,PROBEPORT,0);	//缓慢抽吸空气
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_probewash_a]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

	CommandElemt.srdevaddr = ARM_ADDR;
	strcpy(CommandElemt.cmdbuf, "ZA 0");	
	// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
	// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);			
	sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);			
}


/************整架滴加**************/
/******************************************************************************
*
* Function Name  : CabinAspirate
* Description    : 吸取试剂大容量
* 					 
* Input		   :dispense_t* dispense 滴加试剂数据,unsigned int liquid_val每片玻片滴加量
* Output		   :
* Return		   :  None
*******************************************************************************/
void mb_shelfnumdispenseval(dispense_t* dispense, unsigned int liquid_val)
{
	unsigned char i = 0, same_num = 1,j = 0,cab_cnt = 0,slf_cnt = 0,add_cnt = 0;
	bool repeat = false, try_unlock1 = false, try_unlock2 = false, try_unlock3 = false, try_unlock4 = false;
	unsigned char reagent_num = NO_REAGENT;
	int error_code = 0, res = 0;
	char TCP_Buffer[50] = {0};
	uint8_t shelf_num = 0;
	sr_cmdstruct_t CommandElemt;
	operate_head_list* operate_head_p = NULL;
	unsigned char last_reagent = 0;

	lprintf(log_my, INFO,"in mb_shelfnumdispenseval\n");
	printf("in mb_shelfnumdispenseval\n");
	printf("dispense=");

	if (dispense[i].plate_num % 30 < 10)  
	{
		shelf_num = 0;
		operate_head_p = operate_head1;
		last_reagent = last_reagentA;
	}
	else if (dispense[i].plate_num % 30 < 20)
	{
		shelf_num = 1;
		operate_head_p = operate_head2;
		last_reagent = last_reagentB;
	}
	else if (dispense[i].plate_num % 30 < 30)
	{
		shelf_num = 2;
		operate_head_p = operate_head3;
		last_reagent = last_reagentC;
	}
		
	while(i < 11 && dispense[i].reagent != STOP_OPERATE)
	{
		#if(USE_LOG_INFO == 1)
		printf(" reagent=%d plate=%d", dispense[i].reagent, dispense[i].plate_num);
		lprintf(log_my, INFO," reagent=%d plate=%d", dispense[i].reagent, dispense[i].plate_num);
		#endif

		if ((dispense[i].reagent & 0X80) > 0 && (dispense[i].reagent & 0X7F) <= 35)//滴加时候替换只对试剂架上试剂有效
		{
			TCP_Buffer[0] = dispense[i].plate_num % 30;
			if ( (res = mb_searchcomatiblereagent(&dispense[i].reagent_info, liquid_val)) < 0)
			{
				j++;
				TCP_Buffer[23] = 2;		
			}
			else
			{
				reagent_lock[res / 9][shelf_num] = 1;
					
				if ((res / 9) == 0 )
				{
					if (SHELF_LOCK_WAY_NEW)
						GPIO_OutSet(CGLOCK1);
					else
						GPIO_OutClear(CGLOCK1);
				}
				else if ((res / 9) == 1 )
				{
					if (SHELF_LOCK_WAY_NEW)
						GPIO_OutSet(CGLOCK2);
					else
						GPIO_OutClear(CGLOCK2);
				}
				else if ((res / 9) == 2 )
				{
					if (SHELF_LOCK_WAY_NEW)
						GPIO_OutSet(CGLOCK3);
					else
						GPIO_OutClear(CGLOCK3);
				}
				else if ((res / 9) == 3 )
				{
					if (SHELF_LOCK_WAY_NEW)
						GPIO_OutSet(CGLOCK4);
					else
						GPIO_OutClear(CGLOCK4);
				}
				
				//减去对应的试剂量
				reagent_code[res].val -= liquid_val;
				memcpy(&TCP_Buffer[1], dispense[i].reagent_info.code, 9);
				memcpy(&TCP_Buffer[10], reagent_code[res].code, 9);
				memcpy(&TCP_Buffer[19], &liquid_val, 4);
				if ((res & 0X80) > 0)//不同批号
					TCP_Buffer[23] = 1;		
				res  &= 0X7F;
				dispense[i].reagent = res;
			}	
			nt_sendpacketdata(REPLACE_REAGENT, TCP_Buffer, 24);	
		}
		else    //混合站试剂在滴加时有试剂拿掉无影响
		{
			dispense[i].reagent &= 0X7F;
		}
		i++; 
	}
	printf("*(****************i = %d j =%d\n", i, j);
	if (i == j)    //全部替换成WASH 停止此架操作
	{
		#if(USE_LOG_INFO == 1)
		printf("compatible wash all\n");
		lprintf(log_my, INFO,"compatible wash all\n");
		#endif
		
		if (dispense[0].plate_num % 30 < 10)
			mb_seterrorcode(SHELF1_ABANDON);
		else if (dispense[0].plate_num % 30 < 20)
			mb_seterrorcode(SHELF2_ABANDON);
		else if (dispense[0].plate_num % 30 < 30)
			mb_seterrorcode(SHELF3_ABANDON);
		return;
	}else
	{

	}
	i = 0;
	while(i < 11 && dispense[i].reagent != STOP_OPERATE)
	{
		#if(USE_LOG_INFO == 1)
		printf(" reagent=%d plate=%d", dispense[i].reagent, dispense[i].plate_num);
		lprintf(log_my, INFO," reagent=%d plate=%d", dispense[i].reagent, dispense[i].plate_num);
		#endif

		if ((dispense[i].reagent & 0X7F) >= REAGENT_CASE)
		{
			//	memcpy(&dispense_cab[cab_cnt], &dispense[i], sizeof(dispense_t));
			cab_cnt++;
		}
		else
		{
			//		memcpy(&dispense_slf[slf_cnt], &dispense[i], sizeof(dispense_t));
			slf_cnt++;
			if ((dispense[i].reagent & 0X7F) / 9 == 0)
				try_unlock1 = true;
			else if ((dispense[i].reagent & 0X7F) / 9 == 1)
				try_unlock2 = true;
			else if ((dispense[i].reagent & 0X7F) / 9 == 2)
				try_unlock3 = true;
			else if ((dispense[i].reagent & 0X7F) / 9 == 3)
				try_unlock4 = true;
		}
		i++;
	}

	i = 0;
	while(i < 11 && dispense[i].reagent != STOP_OPERATE)
	{
		while((i + same_num) < 11 && dispense[i + same_num].reagent != STOP_OPERATE)
		{
			if (dispense[i].reagent == dispense[i + same_num].reagent)
				same_num++;
			else
				break;
		}
		
		reagent_num = dispense[i].reagent;
		mb_procwashprobe(reagent_num);
		error_code = reagent_num;
		reagent_flag = dispense[i].reagent;
		while(!flg_mainproexit)	//当same_num > 5时重复一吸液体
		{	
			printf("same_num = %d repeat = %d i = %d",same_num,repeat,i);
			printf("do aspirate\n");
			//	if (mb_monitdoorstate() < 0)
			{	//当PC发来停止某架操作 而刚好执行的是这架的操作则退出
				if ((shelf_num == 0 && !flg_opwork1ready && !hydrateA.flage) ||
					(shelf_num == 1 && !flg_opwork2ready && !hydrateB.flage) ||
					(shelf_num == 2 && !flg_opwork3ready && !hydrateC.flage))
					return; 
			}

			if (reagent_num < REAGENT_CASE)		//试剂架上试剂吸取 对于大容量试剂已在清洗步骤中灌注
			{
				if ((res_shelf = mb_shelfaspirate(reagent_num, same_num, liquid_val,shelf_num)) < 0)
				{
					if (res_shelf == -9)
					{
						for(add_cnt = 0; add_cnt < same_num ;add_cnt++)
						aspirate_notenough[add_cnt] = (dispense[i+add_cnt].plate_num %30) + 1;		
						aspirate_notenough[10] = same_num ;
						mb_seterrorcode(ASPIRATE_WRONG );
					}
					else	
					{
						// printf("dispense=%d dispenseA=%d dispenseB=%d dispenseC=%d dispense[0].plate_num=%d \n",
						// 				dispense,dispenseA,dispenseB,dispenseC,dispense[0].plate_num);

						lprintf(log_my, INFO,"same_num=%d aspirate_notenough[10]=%d i=%d\n",same_num, aspirate_notenough[10],i);
						printf("same_num=%d aspirate_notenough[10]=%d i=%d\n",same_num, aspirate_notenough[10],i);

						for(add_cnt = 0; add_cnt < same_num - aspirate_notenough[10];add_cnt++)
							aspirate_notenough[add_cnt] = (dispense[i+add_cnt].plate_num %30) + 1;
						
						aspirate_notenough[10] = same_num - aspirate_notenough[10];			
						mb_seterrorcode(ASPIRATE_WRONG_NOT_ENOUGH );

						// printf("dispense=%d dispenseA=%d dispenseB=%d dispenseC=%d dispense[0].plate_num=%d \n",
						// dispense,dispenseA,dispenseB,dispenseC,dispense[0].plate_num);
					}
				}	
			}
			else if (reagent_num == REAGENT_DAB)
			{
				if ((dispense[i].plate_num % 30) < 10)
				{
					if (!mb_aspiratedab(Array_mixed_DAB1,same_num, liquid_valA))	
						mb_seterrorcode(ASPIRATE_WRONG);	
				}
				else if ((dispense[i].plate_num % 30) < 20)
				{
					if (!mb_aspiratedab(Array_mixed_DAB2,same_num, liquid_valB))	
						mb_seterrorcode(ASPIRATE_WRONG);		
				}
				else if ((dispense[i].plate_num % 30) < 30)
				{
					if (!mb_aspiratedab(Array_mixed_DAB3,same_num, liquid_valC))	
						mb_seterrorcode(ASPIRATE_WRONG);
				}
				
			}
			else if (reagent_num == REAGENT_SECEND)//双染滴加RED
			{
				if ((dispense[i].plate_num % 30) < 10)
				{
					if (!mb_aspiratedab(Array_mixed_RED1,same_num, liquid_valA))
					{
						aspirate_notenough[10] = 10;//整架有影响
						mb_seterrorcode(ASPIRATE_WRONG );
					}
				}
				else if ((dispense[i].plate_num % 30) < 20)
				{
					if (!mb_aspiratedab(Array_mixed_RED2,same_num, liquid_valB))
					{
						aspirate_notenough[10] = 10;//整架有影响
						mb_seterrorcode(ASPIRATE_WRONG );
					}
				}
				else if ((dispense[i].plate_num % 30) < 30)
				{
					if (!mb_aspiratedab(Array_mixed_RED3,same_num, liquid_valC))
					{
						aspirate_notenough[10] = 10;//整架有影响
						mb_seterrorcode(ASPIRATE_WRONG );
					}
				}
			}
			else	//需要在破片拉伸位置滴加大容量试剂  针对大仪器
			{		
				if(dispense[i].plate_num < 30 || (dispense[i].plate_num >= 60 && dispense[i].plate_num < 90))
					liquid_val = 100;
				else
					liquid_val = 160;
					
				CabinAspirate(reagent_num,same_num, liquid_val);	
			}
			
			if (reagent_num >= REAGENT_CASE)//试剂架试剂可能发生试剂替换，在吸取试剂中赋值
				report_reagentinfo[shelf_num] = reagent_num;
			
			// printf("dispense=%d dispenseA=%d dispenseB=%d dispenseC=%d dispense[0].plate_num=%d \n",
			// 			dispense,dispenseA,dispenseB,dispenseC,dispense[0].plate_num);

			flg_normaldispenseliquid[shelf_num] = true;

			if (!door_open_action)
			{
				if (DOOR_OPENDED)
				{
					printf("DOOR_OPENDED***********************\n");
					if (initialize_finished)
					{
						CommandElemt.srdevaddr = ARM_ADDR;
						strcpy(CommandElemt.cmdbuf, "ZA 0");	
						// if (sb_armpumpsend(port_arm, (unsigned char	*)CommandElemt.cmdbuf, 
						// CommandElemt.srdevaddr , 1, 0, 1) > 0)
						// 	printf("[WarterPour]: Send message	to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
						// else
						// 	printf("[WarterPour]: Send message	to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
						// sb_waitingframeaswer(&CommandElemt);
						sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
					}
					while(DOOR_OPENDED)
						sleep(1);
				}
			}

			if (shelf_num == 0)
				StartDispenseA = true;
			else if (shelf_num == 1)
				StartDispenseB = true;
			else if (shelf_num == 2)
				StartDispenseC = true;

			CutoffSpeedChange(SYSCUTOFF_SPEED);
			first_dispense = true;

			if (dispense[0].reagent < 37 || dispense[0].reagent >= REAGENT_DAB || dispense[0].reagent == REAGENT_DEWAX)
				dispense_spd = SYSPEED_DISPENSE_SHELF;
			else
				dispense_spd = SYSPEED_DISPENSE;

			while(same_num > 0)		//滴加试剂相同的玻片架
			{	
				//		if (mb_monitdoorstate() < 0)
				{	//当PC发来停止某架操作 而刚好执行的是这架的操作则退出
					if ((shelf_num == 0 && !flg_opwork1ready && !hydrateA.flage) ||
					(shelf_num == 1 && !flg_opwork2ready && !hydrateB.flage) ||
					(shelf_num == 2 && !flg_opwork3ready && !hydrateC.flage))
					{
						DispenseRemain();		
						CutoffSpeedChange(SYSSTARTSPD);
						goto DOSHELFDISPENSE_END;
					}
				}
				// printf("dispense=%d dispenseA=%d dispenseB=%d dispenseC=%d dispense[0].plate_num=%d \n",
				// 		dispense, dispenseA, dispenseB, dispenseC, dispense[0].plate_num);
				if (mb_dispensingliquid(dispense[i].plate_num, (unsigned char )liquid_val,same_num,reagent_num) < 0)
				{
					DispenseRemain();
					CutoffSpeedChange(SYSSTARTSPD);
					goto DOSHELFDISPENSE_END;
				}
				first_dispense = false;
				i++;	
				same_num--;
			}	
			
			CutoffSpeedChange(SYSSTARTSPD);
			//滴加相同玻片试剂
			if (repeat)
			{
				same_num = 5;
				repeat = false;
				mb_procwashprobe(reagent_num);
				continue;
			}
			else
				break;
		
		}//当超过5片相同吸取试剂
		same_num = 1;
	}

	printf("operate_head_p->operate.reagent=%d, dispense[0].reagent=%d\n",operate_head_p->operate.reagent,dispense[0].reagent);
	
	if (dispense[0].plate_num  < 90 &&  
		 operate_head_p->operate.reagent != STOP_OPERATE)//  拉升全部 不西风, 水合和最后一步不吸风放置按钮导致通信异常
	{
		if(dispense[0].reagent == REAGENT_ER1 || dispense[0].reagent == REAGENT_ER2)
		{
			//	if (operate_head_p->operate.reagent != REAGENT_ER1 && operate_head_p->operate.reagent != REAGENT_ER2)
			//		{}
			//	else
			if(last_reagent != REAGENT_ER1 && last_reagent != REAGENT_ER2)
			{
				fan[shelf_num].time = 13;
				fan[shelf_num].NORMAL_FAN = true;
			}
		}
		#ifdef LAIYUE
		else if((dispense[0].reagent < REAGENT_CASE ) //&&  strcmp(reagent_code[dispense[0].reagent].reagent_kind, "*Hem") != 0
			|| (dispense[0].reagent == REAGENT_DAB) || (dispense[0].reagent == REAGENT_SECEND))
			//|| dispense[0].reagent == REAGENT_WATER)
		{
			//	fan[shelf_num].time = 6;
			//	fan[shelf_num].NORMAL_FAN = true;
		}
		#else
		else if(dispense[0].reagent < REAGENT_CASE || dispense[0].reagent == REAGENT_DAB || (dispense[0].reagent == REAGENT_SECEND)
				|| dispense[0].reagent == REAGENT_WATER)
			{
				fan[shelf_num].time = 6;
				fan[shelf_num].NORMAL_FAN = true;   // 澳泉客户的染色机是每个新试剂步骤都有10～15秒的抽风动作，以抽干玻片上的试剂，防止下次实验时产生交叉污染 andry
			}
		#endif

		else if((dispense[0].reagent == REAGENT_DEWAX) || (dispense[0].reagent == REAGENT_ALCOHOL))
		{
		#ifdef LAIYUE
			if(dispense[0].reagent == REAGENT_DEWAX && operate_head_p->operate.reagent == REAGENT_ALCOHOL)
				{}
			else
		#endif
			{
				fan[shelf_num].time = 1;
				fan[shelf_num].NORMAL_FAN = true;
			}
		}
		else
		{
			if(( (dispense[0].reagent == REAGENT_WATER) || (dispense[0].reagent == REAGENT_WASH)) &&  
				( (operate_head_p->operate.reagent < REAGENT_CASE ) || (operate_head_p->operate.reagent == REAGENT_DAB)
				|| (operate_head_p->operate.reagent == REAGENT_SECEND) ))
			{}
			else
			{	
				if(strcmp((const char*)&lastt_kind[shelf_num][0], "*Hem") == 0)
					fan[shelf_num].time = 13;
				else if(lastt_kind[shelf_num][0] == 0)
					fan[shelf_num].time = 13;
				else
					fan[shelf_num].time = 6;
				fan[shelf_num].NORMAL_FAN = true;
			}	
		}
	}

	if (res_shelf < 0 || (dispense[0].reagent == REAGENT_DAB) || (dispense[0].reagent == REAGENT_SECEND))// 探到底进行吸取 马上进行清洗
	{
		mb_procwashprobe(last_cabin_reagent);
		res_shelf = 0;
	}
	
	/*
	if(dispense[0].reagent == REAGENT_DEWAX || dispense[0].reagent == REAGENT_ALCOHOL)
		tp_fanworkproce(shelf_num + 1, 0);
	*/	
	DOSHELFDISPENSE_END:
	if (try_unlock1)
		reagent_lock[0][shelf_num] = 0;//滴加结束尝试解锁对应试剂架
	if (try_unlock2)
		reagent_lock[1][shelf_num] = 0;
	if (try_unlock3)
		reagent_lock[2][shelf_num] = 0;
	if (try_unlock4)
		reagent_lock[3][shelf_num] = 0;	
}



// void ZRaise(void)
// {
// 	sr_cmdstruct_t CommandElemt;
	
// 	CommandElemt.srdevaddr = ARM_ADDR;
// 	sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
// 	if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
// 		printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
// 	else
// 		printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
// 	sb_waitingframeaswer(&CommandElemt);
// }


/******************************************************************************
*
* Function Name  : mb_dispensingliquid
* Description    : 滴加一片玻片操作
* 					 
* Input		   :unsigned char plate_num 滴加玻片位置,unsigned int liquid_val每片玻片滴加量
* Output		   :
* Return		   :  None
*******************************************************************************/
int  mb_dispensingliquid(unsigned char plate_num, unsigned char liquid_val, unsigned char  same_num, char reagent_num)
{
	sr_cmdstruct_t CommandElemt;
	unsigned int* ordArray_plate = NULL;
	unsigned int* dispense_step_p = NULL;
	unsigned int dispense_step = 0, sysstep = 0;
	int sysspd = dispense_spd;
	
		/*	
		if (liquid_val == 100)
			sysstep = SYSTEP_LIQUID;
		else if (liquid_val == 150)
			sysstep = SYSTEP_LIQUID * 3 / 2;
		else if (liquid_val == 200)
			sysstep = SYSTEP_LIQUID * 2;
		*/
		//	sysstep = SYSTEP_LIQUID * liquid_val / 100 *SYSTEP_MOR;//多滴加百分之10
	sysstep = SYSTEP_LIQUID * liquid_val / 100;

	#if(1 == USE_LOG_INFO)

	#else
	if (liquid_val < 100)
	{
		flg_mainproexit = true;
		return -1;
	}
	#endif
			
	if (plate_num% 30 < 10)
		dispense_step_p = dispense_stepA;
	else if (plate_num% 30 < 20)
		dispense_step_p = dispense_stepB;
	else
		dispense_step_p = dispense_stepC;

	if (plate_num < 30) //初始位置
	{
		ordArray_plate = (unsigned int *)ordArray_plate1;
		dispense_step = dispense_step_p[0];
	}
	else if (plate_num < 60)//150ul位置
	{
		plate_num = plate_num - 30;
		ordArray_plate = (unsigned int *)ordArray_plate2;
		dispense_step = dispense_step_p[1];
	}
	else if (plate_num < 90)//100ul位置
	{
		plate_num = plate_num - 60;
		ordArray_plate = (unsigned int *)ordArray_plate3;
		dispense_step = dispense_step_p[2];
	}
	else if (plate_num < 120)//组织位置
	{
		plate_num = plate_num - 90;
		ordArray_plate = (unsigned int *)ordArray_plate4;
		dispense_step = dispense_step_p[0] + DISPENSE_INTERVAL_ALL;
	//	dispense_spd = SYSPEED_DISPENSEFUL;
	}

	if (first_dispense)
	{
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// 	else
		// 		printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// 	sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
	}

	if ((plate_num % 30) < 10)
	{
		if( !flg_opwork1ready&& !hydrateA.flage)
			return -1;	
	}
	else if ((plate_num % 30) < 20)
	{
		if( !flg_opwork2ready&& !hydrateB.flage)
			return -1;
	}
	else
	{
		if( !flg_opwork3ready&& !hydrateC.flage)//水合时玻片已经抬升
			return -1;
	}

	#if(USE_PRINT_LOG == 1)
		printf("ordArray_plate1=%d ordArray_plate2=%d ordArray_plate3=%d ordArray_plate4=%d par.ordArray_plate=%d\n",
				ordArray_plate1[0][0],ordArray_plate2[0][0], ordArray_plate3[0][0], ordArray_plate4[0][0], par.ordArray_plate[0][0]);
		printf("plate_num=%d ordArray_plate=%d dispense_step=%d\n", plate_num, ordArray_plate[0], dispense_step);
	#endif

	CommandElemt.srdevaddr = ARM_ADDR;

	//		sprintf(CommandElemt.cmdbuf, "PA %d %d %d",*( ordArray_plate + plate_num * 2 + 0), *( ordArray_plate + plate_num * 2 + 1), MOV_ZH);//到达玻片位置

	sprintf(CommandElemt.cmdbuf, "PA %d %d %d",*( ordArray_plate + plate_num * 2 + 0), *( ordArray_plate + plate_num * 2 + 1), dispense_step);//到达玻片位置
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);
	if(0 > sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1)){
		lprintf(log_my, ERROR, "mb_dispensingliquid send pa %d %d %d error\n", *( ordArray_plate + plate_num * 2 + 0), \
			*( ordArray_plate + plate_num * 2 + 1), dispense_step);
	}
		
	if ((plate_num % 30) < 10)
	{
		if( !flg_opwork1ready&& !hydrateA.flage)
			return -1;
			
	}
	else if ((plate_num % 30) < 20)
	{
		if( !flg_opwork2ready&& !hydrateB.flage)
			return -1;
	}
	else
	{
		if( !flg_opwork3ready&& !hydrateC.flage)//水合时玻片已经抬升
			return -1;
	}

	//	sleep(3);
	//	mb_monitdoorstate();

	CommandElemt.srdevaddr = PUMP_ADDR;
	if (reagent_num < 37)
	{
		if (first_dispense)
			sprintf(CommandElemt.cmdbuf, "S%dD%dR", sysspd,sysstep + SYSTEP_AIR / 2);//滴加试剂
		else	
			sprintf(CommandElemt.cmdbuf, "S%dD%dR", sysspd,sysstep * SYSTEP_DISPENSMOR );
	}
	else
	{
		if (first_dispense)
		sprintf(CommandElemt.cmdbuf, "S%dD%dR", sysspd,sysstep + SYSTEP_AIR );
		else
		sprintf(CommandElemt.cmdbuf, "S%dD%dR", sysspd,sysstep );
	}
	// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
	// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
	// else
	// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
	// sb_waitingframeaswer(&CommandElemt);	
	if(0 > sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1)){
		lprintf(log_my, ERROR, "mb_dispensingliquid send pump s:%d d:%d r error\n", sysspd,sysstep);
	}

	//usleep(DISPENSE_T);
	//usleep(100000);

	#if 0
		CommandElemt.srdevaddr = PUMP_ADDR;
		sprintf(CommandElemt.cmdbuf, "S%dI%dP%dR", SYSPEED_AIR,PROBEPORT,SYSTEP_AIR);	//缓慢抽吸空气
		if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		CommandElemt.srdevaddr , 1, 0, 1) > 0)
			printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		else
			printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		sb_waitingframeaswer(&CommandElemt);
		mb_monitdoorstate();	
	#endif

	if (same_num == 1)//最后一次上升
	{
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "ZA %d",  MOV_ZH);//Z轴上升
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 	CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_dispensingliquid]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		if(0 > sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1)){
			lprintf(log_my, ERROR, "mb_dispensingliquid za %d error\n", MOV_ZH);
		}
	}
	return 0;	
}



void do_work_hydration(void)
{	
	struct timeval  now;
	unsigned char i = 0;
	stminibd_sendpacket cmd;
	static bool isReload_setedA = true;
	static bool isReload_setedB = true;
	static bool isReload_setedC = true;
	char TCP_Buffer[4] = {0};
	
	gettimeofday(&now, NULL);
			
	if (hydrateA.flage && flg_mianarmstop_a)
	{
		if ( (now.tv_sec - hydrateA.start_time) > 30 * (60))//15分钟
		//	if ( (now.tv_sec - hydrateA.start_time) > 2*60 )//15分钟
		{
			bs_packetshelfstreach(0, &cmd); //此命令使玻片架按键失效(待验证)
		
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head, cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");
			while(mini_work_finished[0])//确保miniBoard线程已置FALSE
				usleep(1000);
			sleep(1);
			while(!mini_work_finished[0])
				usleep(200000);
			mini_work_finished[0] = false; //防止上次拉伸对这次 mini_finished的干扰
			mb_dischargeshelfwateliquid(1);
			sleep(3);//防止按键刚好按下
			if (hydrateA.flage == false)
			{
				memset(&cmd,0,sizeof(cmd));
				#ifdef glint
				cmd.cmd = WORK_FINISH;//RELOAD;
				cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
				#else
				cmd.cmd = RELOAD;//RELOAD;
				cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
				#endif
				cmd.minicmd_buffer[4] = 1;
				cmd.minicmd_num=2;
				pthread_mutex_lock(&mutex_mlock);
				set_minicmd(pcmd_head,cmd);
				if (pthread_mutex_unlock(&mutex_mlock) != 0)
					lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");
				return ;
			}
			//		hydrateA.start_time = now.tv_sec;
			printf("hydrateA FLAG =%d", hydrateA.flage);
	
			TCP_Buffer[0] = 1;
			nt_sendpacketdata(HYDRATE_WORK, TCP_Buffer, 1);
			if (hydrateA.flage)	//防止在发送doStrechShelf之前已经上台玻片架，
			{
				//  大仪器主臂不动作
				flg_mianarmstop_a = false;
				isReload_setedA = false;
			}
		}
		else if (flg_mianarmstop_a && (!isReload_setedA))	//由于主臂在动作时可能发送拉伸动作导致不能RELOAD
		{
			memset(&cmd,0,sizeof(cmd));
			#ifdef glint
			cmd.cmd = WORK_FINISH;//RELOAD;
			cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
			#else
			cmd.cmd = RELOAD;//RELOAD;
			cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
			#endif
			cmd.minicmd_buffer[4] = 1;
			cmd.minicmd_num=2;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			isReload_setedA = true;
		}
		
	}

	if (hydrateB.flage && flg_mianarmstop_b)
	{
		if ( (now.tv_sec - hydrateB.start_time) >  30 * (60))
		//if ( (now.tv_sec - hydrateB.start_time) >  60)
		{	
			bs_packetshelfstreach(10, &cmd); //此命令使玻片架按键失效
		
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			while(mini_work_finished[1])//确保miniBoard线程已置FALSE
				usleep(1000);
			sleep(1);
			while(!mini_work_finished[1])
				usleep(200000);
			mini_work_finished[1] = false; //防止上次拉伸对这次 mini_finished的干扰
			mb_dischargeshelfwateliquid(2);
			//		hydrateB.start_time = now.tv_sec;
			sleep(3);//防止按键刚好按下
			if (hydrateB.flage == false)
			{
				memset(&cmd,0,sizeof(cmd));
				#ifdef glint
				cmd.cmd = WORK_FINISH;//RELOAD;
				cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
				#else
				cmd.cmd = RELOAD;//RELOAD;
				cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
				#endif
				cmd.minicmd_buffer[4] = 2;
				cmd.minicmd_num=2;
				pthread_mutex_lock(&mutex_mlock);
				set_minicmd(pcmd_head,cmd);
				if (pthread_mutex_unlock(&mutex_mlock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
				return ;
			}
			
			printf("hydrateB=");

			for (i = 0; i < 11;i++)
			{
				dispenseB[i] = hydrateB.hydrate_plate[i];

				printf(" %d-%d ",dispenseB[i].reagent, dispenseB[i].plate_num);
			}

			TCP_Buffer[0] = 2;
			nt_sendpacketdata(HYDRATE_WORK, TCP_Buffer, 1);
			if (hydrateB.flage)	//防止在发送doStrechShelf之前已经上台玻片架，
			{
				//  大仪器主臂不动作
				flg_mianarmstop_b = false;
				isReload_setedB = false;
			}
		}
		else if (flg_mianarmstop_b && (!isReload_setedB))	//由于主臂在动作时可能发送拉伸动作导致不能RELOAD
		{
			memset(&cmd, 0, sizeof(cmd));
			#ifdef glint
			cmd.cmd = WORK_FINISH;//RELOAD;
			cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
			#else
			cmd.cmd = RELOAD;//RELOAD;
			cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
			#endif
			cmd.minicmd_buffer[4] = 2;
			cmd.minicmd_num=2;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			isReload_setedB = true;
		}	
	}

	if (hydrateC.flage && flg_mianarmstop_c)
	{
		if ( (now.tv_sec - hydrateC.start_time) >  30 * (60))
		//	if ( (now.tv_sec - hydrateC.start_time) > (60))
		{	
			bs_packetshelfstreach(20,&cmd); //此命令使玻片架按键失效
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");
			while(mini_work_finished[2])//确保miniBoard线程已置FALSE
				usleep(1000);
			sleep(1);
			while(!mini_work_finished[2])
				usleep(200000);
			mini_work_finished[2] = false; //防止上次拉伸对这次 mini_finished的干扰
			mb_dischargeshelfwateliquid(3);
			sleep(3);//防止按键刚好按下
			if (hydrateC.flage == false)
			{
				memset(&cmd,0,sizeof(cmd));
				#ifdef glint
				cmd.cmd = WORK_FINISH;//RELOAD;
				cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
				#else
				cmd.cmd = RELOAD;//RELOAD;
				cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
				#endif
				cmd.minicmd_buffer[4] = 3;
				cmd.minicmd_num=2;
				pthread_mutex_lock(&mutex_mlock);
				set_minicmd(pcmd_head,cmd);
				if (pthread_mutex_unlock(&mutex_mlock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
				return ;
			}
		//	hydrateC.start_time = now.tv_sec;
			printf("hydrateC=");

			for (i = 0; i < 11;i++)
			{
				dispenseC[i] = hydrateC.hydrate_plate[i];

				printf(" %d-%d ",dispenseC[i].reagent, dispenseC[i].plate_num);
			}
			
			TCP_Buffer[0] = 3;
			nt_sendpacketdata(HYDRATE_WORK, TCP_Buffer, 1);
			//	sleep(3);
			if (hydrateC.flage)	//防止在发送doStrechShelf之前已经上台玻片架，
			{
				//  大仪器主臂不动作
				flg_mianarmstop_c = false;
				isReload_setedC = false;
			}
		}
		else if (flg_mianarmstop_c && (!isReload_setedC))	//由于主臂在动作时可能发送拉伸动作导致不能RELOAD
		{
			memset(&cmd,0,sizeof(cmd));
			#ifdef glint
			cmd.cmd = WORK_FINISH;//RELOAD;
			cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
			#else
			cmd.cmd = RELOAD;//RELOAD;
			cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
			#endif
			cmd.minicmd_buffer[4] = 3;
			cmd.minicmd_num=2;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			isReload_setedC = true;
		}
	}
}



void mb_lockallregent(uint8_t shelf_num)
{
	reagent_lock[0][shelf_num] =reagent_lock[1][shelf_num]=reagent_lock[2][shelf_num]=reagent_lock[3][shelf_num]= 1;
	if (SHELF_LOCK_WAY_NEW)
	{
		GPIO_OutSet(CGLOCK1);
		GPIO_OutSet(CGLOCK2);
		GPIO_OutSet(CGLOCK3);
		GPIO_OutSet(CGLOCK4);
	}
	else
	{
		GPIO_OutClear(CGLOCK1);
		GPIO_OutClear(CGLOCK2);
		GPIO_OutClear(CGLOCK3);
		GPIO_OutClear(CGLOCK4);
	}
}

int md_lockreagentshelf(void) //这个函数中发送段错误 多为流程中没有DAB
{
	unsigned char i = 0;
	bool lock_contorl1 = false, lock_contorl2 = false, lock_contorl3 = false, lock_contorl4 = false; //为TURE时必定上锁
	struct timeval now;
	operate_head_list* operate_head = NULL, *operate_head_p = NULL;
	operate_t * operate_p = NULL;
	volatile unsigned char * reagent_lock_num_p = NULL;
	char TCP_Buffer[24] = {0};
	char Isunlock[4] = {0};

	pthread_mutex_lock(&head_lock);
	
	gettimeofday(&now, NULL);
		
	for (i = 0; i < 3;i++)
	{
		if (i == 0)
		{
			operate_head_p = operate_head = operate_head1;
			//	mainarm_finished = flg_mianarmstop_a;
		}
		else if (i == 1)
		{
			operate_head_p = operate_head = operate_head2;
			//	mainarm_finished = flg_mianarmstop_b;
		}
		else if (i == 2)
		{
			operate_head_p = operate_head = operate_head3;
			//	mainarm_finished = flg_mianarmstop_c;
		}
		
		//		printf("in md_lockreagentshelf\n");
		do
		{
			operate_p = &operate_head_p->operate;

			//根据操作规程 同一架流程中 不同玻片滴加试剂架上试剂时可能全是不同的瓶子(抗体)
			while (operate_p != NULL && operate_p->reagent != STOP_OPERATE)  //在一张玻片的流程中找小于N分钟的要用到的试剂
			{	
				if (operate_p->reagent >= REAGENT_DAB)
				{
					operate_p = operate_p->next;
					continue;
				}
				//小于N分钟锁定
				//	printf("operate_p->time=%d now.tv_sec - begin_time.tv_sec=%d\n",
				//	operate_p->time,now.tv_sec - begin_time.tv_sec);
				
				if ((operate_p->time <= (now.tv_sec - begin_time.tv_sec)) || 
					( (operate_p->time- (now.tv_sec - begin_time.tv_sec) ) <= 300 ) )
				{
					//break;
					//		printf("operate_p->reagent =%d\n",operate_p->reagent);
					//试剂信息无效  说明被拿掉 直接报警
					if ((operate_p->reagent & 0X80) > 0)
					{
						lprintf(log_my, ERROR, "REAGENT_REMOVED reagent has been removed&&&&&&&&&&&&&&&&&&&&&&\n");
		
						//发送被拿掉的试剂码
						TCP_Buffer[0] = 0;
						memcpy(&TCP_Buffer[1], operate_p->reagent_info.code, sizeof(operate_p->reagent_info.code));

						nt_sendpacketdata(REAGENT_REMOVED, TCP_Buffer, 21);
						if (pthread_mutex_unlock(&head_lock) != 0)
							lprintf(log_my, ERROR,"%s", "pthread_mutex_unlock error head_lock");
						return -1;
					}

					if (operate_p->reagent < 9)	
					{
						reagent_lock_num_p = &reagent_lock_num1;
						lock_contorl1 = true;
						reagent_lock[0][i] = 1;
					}
					else if (operate_p->reagent < 18)
					{
						reagent_lock_num_p = &reagent_lock_num2;
						lock_contorl2 = true;	
						reagent_lock[1][i] = 1;
					}
					else if (operate_p->reagent < 27 )
					{
						reagent_lock_num_p = &reagent_lock_num3;
						lock_contorl3 = true;
						reagent_lock[2][i] = 1;
					}
					else if (operate_p->reagent < 36 )
					{
						reagent_lock_num_p = &reagent_lock_num4;
						lock_contorl4 = true;
						reagent_lock[3][i] = 1;
					}	
					else if ( operate_p->reagent == REAGENT_ER1			
							|| operate_p->reagent == REAGENT_ER2)
					{
						//混合在混合操作函数中锁定	
						operate_p = operate_p->next;
						continue;
					}
					else			//其他试剂但是不用锁定
					{
						operate_p = operate_p->next;
						continue;
					}
					
					if (operate_head == operate_head1)
							*reagent_lock_num_p |= 0X01;
						else if (operate_head == operate_head2)
							*reagent_lock_num_p |= 0X02;
						else if (operate_head == operate_head3)
							*reagent_lock_num_p |= 0X04;				
				}
				else	//大于N分钟退出
					break;

				operate_p = operate_p->next;
			}
		
			operate_head_p = operate_head_p->next_head;
		}
		while(operate_head_p != NULL);
		
		if (operate_head->operate.reagent < REAGENT_CASE)  //只有当前操作为滴加试剂架上试剂时才 
		{												   //等待主臂滴加完成给信号解锁
			pthread_mutex_lock(&mutex_tryworklock);
			if (lock_contorl1)
				TRY_REAGENT_UNLOCK1 = false;
			if (lock_contorl2)
				TRY_REAGENT_UNLOCK2 = false;
			if (lock_contorl3)
				TRY_REAGENT_UNLOCK3 = false;
			if (lock_contorl4)
				TRY_REAGENT_UNLOCK4 = false;
			if (pthread_mutex_unlock(&mutex_tryworklock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mutex_tryworklock");
		}
		//	if ( operate_head->operate.reagent == STOP_OPERATE)
	}
		
	if (SHELF_LOCK_WAY_NEW)
	{
		if (lock_contorl1 == true)
			GPIO_OutSet(CGLOCK1);
		else if ((mb_islockedreagent(0) == 0) && reagent_check[0].NEED_SCAN == false
				&& reagent_check[0].NEED_MIX == false  && reagent_check[0].NEED_CHECK == false)
		{
			Isunlock[0] = 1;
			GPIO_OutClear(CGLOCK1);
		}
		
		if (lock_contorl2 == true)
			GPIO_OutSet(CGLOCK2);
		else if ((mb_islockedreagent(1) == 0) && reagent_check[1].NEED_SCAN == false
				&& reagent_check[1].NEED_MIX == false && reagent_check[1].NEED_CHECK == false)
		{
			Isunlock[1] = 1;
			GPIO_OutClear(CGLOCK2);
		}
		if (lock_contorl3 == true)
			GPIO_OutSet(CGLOCK3);
		else if ((mb_islockedreagent(2) == 0) && reagent_check[2].NEED_SCAN == false
				&& reagent_check[2].NEED_MIX == false && reagent_check[2].NEED_CHECK == false)
		{
			Isunlock[2] = 1;
			GPIO_OutClear(CGLOCK3);
		}
		
		if (lock_contorl4 == true)
			GPIO_OutSet(CGLOCK4);
		else if ((mb_islockedreagent(3) == 0) && reagent_check[3].NEED_SCAN == false
				&& reagent_check[3].NEED_MIX == false && reagent_check[3].NEED_CHECK == false)
		{
			Isunlock[3] = 1;
			GPIO_OutClear(CGLOCK4);
		}
	}
	else
	{
		if (lock_contorl1 == true)
			GPIO_OutClear(CGLOCK1);
		else if ((mb_islockedreagent(0) == 0) && reagent_check[0].NEED_SCAN == false
				&& reagent_check[0].NEED_MIX == false  && reagent_check[0].NEED_CHECK == false)
		{
			Isunlock[0] = 1;
			GPIO_OutSet(CGLOCK1);
		}
		
		if (lock_contorl2 == true)
			GPIO_OutClear(CGLOCK2);
		else if ((mb_islockedreagent(1) == 0) && reagent_check[1].NEED_SCAN == false
				&& reagent_check[1].NEED_MIX == false && reagent_check[1].NEED_CHECK == false)
		{
			Isunlock[1] = 1;
			GPIO_OutSet(CGLOCK2);
		}
		if (lock_contorl3 == true)
			GPIO_OutClear(CGLOCK3);
		else if ((mb_islockedreagent(2) == 0) && reagent_check[2].NEED_SCAN == false
				&& reagent_check[2].NEED_MIX == false && reagent_check[2].NEED_CHECK == false)
		{
			Isunlock[2] = 1;
			GPIO_OutSet(CGLOCK3);
		}
		
		if (lock_contorl4 == true)
			GPIO_OutClear(CGLOCK4);
		else if ((mb_islockedreagent(3) == 0) && reagent_check[3].NEED_SCAN == false
				&& reagent_check[3].NEED_MIX == false && reagent_check[3].NEED_CHECK == false)
		{
			Isunlock[3] = 1;
			GPIO_OutSet(CGLOCK4);
		}
	}

	if (pthread_mutex_unlock(&head_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");

	return 0;
}


  #if 0
static double Get30Milimeter(int steps)
{
	int maxStep = 1300;
	double oneStepHeight = 0.098;//0.098 mm per step
	return (maxStep - steps) * oneStepHeight;
}

static double Get30S1(double mm)
{
	return 63.05 * pow(mm, 2) / 56.0 + 493.25 * mm;
}

static double Get30S2(double mm)
{
	return 15.1 * pow(mm, 2) / 4.0 + 344.9 * mm;
}

static double Get30S3(double mm)
{
	return 26.3 * pow(mm, 2) / 80.0 + 551.2 * mm;
}


static double Get30Result(int steps)
{
	double mm = Get30Milimeter(steps);
	if (mm >= 0 && mm <= 28)
		return Get30S1(mm);
	else if (mm > 28 && mm <= 31)
	{
		double ret = 0;
		ret += Get30S1(28);
		ret += Get30S2(mm) - Get30S2(28);

		return ret;
	}
	else
	{
		if (mm > 71)
			mm = 71;
		double ret = 0;
		ret += Get30S1(28);
		ret += Get30S2(31) - Get30S2(28);
		ret += Get30S3(mm) - Get30S3(31);
		return ret;
	}
}

#endif

#if 1

// int _7mLMaxStepCount = 1311; //理论值
// int _7mLcolumnCount = 1261;

//  int _7mLMaxStepCount = 1315;//卡尺测量值
// int _7mLcolumnCount = 1265;

#define M_PI 3.14159265358979323846 
#define mis_val 20 //机械臂和试剂仓底座误差
#define leave_val  367.8; //理论367.8球冠体积//实际值400
  int _7mLcolumnCount = 1276 - mis_val;//到达球冠表面液面
 double oneStepHeight = 0.098;
 int _6mLcolumnCount = 1285 - mis_val;//到达球冠表面液面
#define _6mLleave_val  241.8;

 double GetVolume7(int steps)
 {
 	int _7mLMaxStepCount = _7mLcolumnCount + 50;//探针测量值
 	double h = oneStepHeight *  (_7mLMaxStepCount - steps - 50) ;//圆台高
 	double qgh = oneStepHeight *  (_7mLMaxStepCount - steps) ;//圆台高
	double R = 6.35 + 0.25 / 72.8 * h;
	double r = 6.35;

	 if (steps >= _7mLMaxStepCount) return 0;

	if (steps >= _7mLcolumnCount) 

		return M_PI * qgh * qgh * (6.35 - qgh / 3);//球冠体积
	
	double c = M_PI * h *( R * R + R * r + r * r) / 3;//圆台体积

	 return c + leave_val; //
 }

#if 0
 static double GetVolume6(int step)
{
 	int mainMaxStepCount = _6mLcolumnCount + 50;//探针测量值
// 	int error_step = _6mLcolumnCount  - 35 / oneStepHeight;//929
	int error_step = 907;
 	double h = oneStepHeight *  (mainMaxStepCount - step - 50) ;//圆台高
 	double qgh = oneStepHeight *  (mainMaxStepCount - step) ;//圆台高
	// double R = 5 + 0.001745 * h;
	//double r = 5;
	double error_val = 0;
	double error_R = 4.5 + 0.014285 * h;
	double error_r = 4.5;
	
	
	 if (step >= mainMaxStepCount) return 0;

	if (step >= _6mLcolumnCount) 

		return M_PI * qgh * qgh * (5 - qgh / 3);//球冠体积

	if (step > error_step)
	{
		 error_val =  M_PI * h *( error_R * error_R + error_R * error_r + error_r * error_r) / 3;
		return error_val + _6mLleave_val;
	}

	error_val = M_PI * oneStepHeight*(_6mLcolumnCount - error_step) *( error_R * error_R + error_R * error_r + error_r * error_r) / 3;
	double good_val =  M_PI * oneStepHeight*(error_step - step) *25;
	return  error_val + good_val + _6mLleave_val;
 }	
 #endif

	//double c = M_PI * h *( R * R + R * r + r * r) / 3;//圆台体积
//	double c = M_PI * h *25;//按圆柱算70mm高度少130ul
	// return c + _6mLleave_val; //




//#define M_VAL(S) ( (1307 - S) * 0.098 ) //高度mm S为步数 1311
//#define M_VAL(S) ( (1292 - S) * 0.098 ) //高度mm S为步数 理论值1292
#define M_VAL(S) ( (1288 - S) * 0.098 )// 2ML打底后取最低高度值为1288


#define XJ(h) ( (0.00785*h)*2 ) //底边长度变化h 为高度
#define SJ(x) ( (x + 26.68) * (x + 20.66))//截面积变化x为底边长度

//以上为试剂瓶不考虑槽

#define C_X1J(h) ((0.565 - 0.02123*h)*2 ) //底边长度变化h 为高度 (上底)
#define C_X2J(h) ((0.37486 - 0.0213*h)*2 ) //底边长度变化h 为高度(下底)
#define C_SJ(x1,x2,h) ( (x1 + 7.32 + x2 + 2.37) * h / 2)//截面积变化x为底边长度,h高度
//以上为槽

#define TV(s,sd,h) ( s + sd +  sqrt(s * sd)) * h / 3 

/*
double TV(double s,double sd,double h) 
{

	//double val = pow(s + sd,0.5); 

return ( ( s + sd +  sqrt(s + sd)) * h / 3 );//台体体积s为变化的截面积sd为底部截面积
}
*/
/*
  static double TV(double s, double sd, double h)
        {
            return (s + sd + sqrt(s * sd)) * h / 3; //台体体积s为变化的截面积sd为底部截面积
        }

*/
 static double GetVolume30other(int steps, int subtracts, double originals, bool batching)
{
	if (steps >= 1292) return 0;
	double h = (1292 - steps) * oneStepHeight;
	double res = 0;

	double x = 0.00785 * h * 2; //底边长度变化h 为高度
	double y = (x + 26.68) * (x + 20.66); //截面积变化x为底边长度
	double s_sj = y;//试剂瓶截面积变
	double a = (0.565 - 0.02123 * h) * 2; //底边长度变化h 为高度 (上底)
	double b = (0.37486 - 0.0213 * h) * 2; //底边长度变化h 为高度(下底)
	double c_sj = (a + 7.32 + b + 2.37) * h / 2;//槽截面积变化 //截面积变化x为底边长度,h高度

	if (h <= 30.79)//有槽
	{
		double hc = TV(c_sj, 14.4625, h);
		if (hc > 276) hc = 276;
		res = TV(s_sj, 551.2088, h) - hc;
	}
	else //槽以上
	{
		res = TV(s_sj, 551.2088, h) - 276;//槽体积276
	}

	//		 LogHelper.LogDebug(string.Format("试剂吸液测量命令【2】:步骤{0}；", steps), string.Format("试剂计算后值：{0}", res));
	printf("试剂吸液测量命令 steps=%d  试剂计算后值 res=%lf\n", steps, res);

	if (res > 30000)
		res = 30000;

	return res;
}



 double GetVal30(int step)
{
	double h = M_VAL(step),hc,res = 0;
	if (h < 0) return 0;
	double s_sj = SJ(XJ(h));//试剂瓶截面积变
	double c_sj = C_SJ(C_X1J(h),C_X2J(h),h);//槽截面积变化
	
	printf("XJ(h)=%lf\n",XJ(h));

	printf("C_X1J(h)=%lf\n",C_X1J(h));
	printf("C_X2J(h)=%lf\n",C_X2J(h));

	printf("s_sj  = %lf  c_sj =%lf h =%lf\n",s_sj ,c_sj ,h);
	printf("TV(s_sj, 551.2088 ,h)=%lf TV(c_sj, 14.4625 ,h)=%f",TV(s_sj, 551.2088 ,h),TV(c_sj, 14.4625 ,h));

	if (h <= 30.79)//有槽
	{	
		hc = TV(c_sj, 14.4625 ,h);
		if(hc > 276) hc = 276;
		
		res = ( TV(s_sj, 551.2088 ,h) - hc);
	}
	else //槽以上
	{
		res = TV(s_sj, 551.2088 ,h) - 276 ;//槽体积276
	}
	
	return res;
}
#endif

/******************************************************************************
*
* Function Name  : mb_checkshelfreagent
* Description    : 测试剂架试剂量
* 					 
* Input		   :unsigned int reagent_num 试剂架次,unsigned int ordArray_plate[][2]试剂坐标
				,unsigned char *check_reagent 需要测试剂的标识数据
* Output		   :
* Return		   :  None
*******************************************************************************/
void mb_checkshelfreagent(unsigned int reagent_num,unsigned int ordArray_plate[][2],unsigned char *check_reagent)
{
	sr_cmdstruct_t CommandElemt;
	int i=0, ZX_res = 0;
	CommandElemt.srdevaddr = ARM_ADDR;
	char TCP_Buffer[50] = {0};

	sleep(1);

	printf("in mb_checkshelfreagent\n");
	memset(ReportArrayData,0,sizeof(ReportArrayData));
	
	for(i = 0;i < 9; i++)
	{	
		mb_monitdoorstate();
	
		usleep(ASPIRATE_T);
		CommandElemt.srdevaddr = ARM_ADDR;
		sprintf(CommandElemt.cmdbuf, "PA %d %d %d",ordArray_plate[i][0], ordArray_plate[i][1], MOV_ZH);//到达玻片位置
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// 	else
		// 		printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// 	sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		
		sprintf(CommandElemt.cmdbuf, "ZA %d", MOV_ZH + 100);
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// 	else
		// 		printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// 	sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		//usleep(300000);			
		
		flg_getedserailarr = TRUE;
		sprintf(CommandElemt.cmdbuf, "ZX 0 0 %d", 1320);
		//sprintf(CommandElemt.cmdbuf, "ZX 0 0 %d", LIQUID_ZMAX);////	sprintf(CommandElemt.cmdbuf, "ZA 500");
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// if ((ZX_res = sb_waitingframeaswer(&CommandElemt)) < 0)
		ZX_res = sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		if(ZX_res < 0)
		{
			if (ZX_res == -3)
			{
				sprintf(CommandElemt.cmdbuf, "ZI");
				// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
				// 			CommandElemt.srdevaddr , 1, 0, 1) > 0)
				// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
				// else
				// 	printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
				// sb_waitingframeaswer(&CommandElemt);
				sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
				return;
			}
			else
				ReportArrayData[i] = 0XFFFFFFFF;//没测到
		}
		else
			ReportArrayData[i] = serail_dataarr;
			
		printf("Arraydata=%d\n",serail_dataarr);

		CommandElemt.srdevaddr = ARM_ADDR;
		if (ReportArrayData[i] != 0XFFFFFFFF)
		{
			sprintf(CommandElemt.cmdbuf, "ZS -%d %d", 200, LIQUID_SPEED);//缓慢上升 当>500步时z轴回到500步位置
			// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
			// 				CommandElemt.srdevaddr , 1, 0, 1) > 0)
			// 		printf("[mb_shelfaspirate]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			// 	else
			// 		printf("[mb_shelfaspirate]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			// 	sb_waitingframeaswer(&CommandElemt);
			sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);
		}
		sprintf(CommandElemt.cmdbuf, "ZA %d",MOV_ZH);
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 		printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// 	else
		// 		printf("[mb_checkshelfreagent]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &CommandElemt, 1, 0, 1);

		TCP_Buffer[0] = i + (reagent_num-1)*9;
		memcpy(&TCP_Buffer[1],&ReportArrayData[i],4);
		//	nt_sendpacketdata(REAGENT_SEND, TCP_Buffer, sizeof(ReportArrayData) + 1);	
	}

	return ;
	/*
	ReportArrayData[0] = 1288;
	ReportArrayData[1] = 1285;
	ReportArrayData[2] = 1286;
	ReportArrayData[3] = 1288;
	ReportArrayData[4] = 1282;
	ReportArrayData[5] = 1282;
	ReportArrayData[6] = 1288;
	ReportArrayData[7] = 1284;
	ReportArrayData[8] = 1289;
	
	*/
	//	for (i = 0; i < 9;i++)
	{
	//	ReportArrayData[i] += i;
		printf("ReportArrayData[%d]=%d ", i, ReportArrayData[i]);
		printf(" reagent_val=%lf ", GetVal30(ReportArrayData[i]));//+ i 每隔加误差1
		GetVolume30other(ReportArrayData[i], 0, 0, 0);
		//	printf(" reagent_val=%lf ",GetVolume6(ReportArrayData[i]));
		//	printf(" reagent_val=%lf ",GetVolume7(ReportArrayData[i]));

		//	printf(" reagent_val=%lf ",GetVolume6(ReportArrayData[i]));
	}
	
//	nt_sendpacketdata(REAGENT_SEND, TCP_Buffer, sizeof(ReportArrayData) + 1);
	printf("out of mb_checkshelfreagent\n");
}

void mb_shelfscanning(unsigned int shelf_num)  //,unsigned char* mini_recieve_code)
{
	sr_cmdstruct_t sendcmdbuf;
	int i=0;
	unsigned int ordArray_plate[10][2];
	stminibd_sendpacket cmd;
	unsigned int X_Array = 0;

	if(flg_intosleep)
		return;

	printf("in mb_shelfscanning %d\n",shelf_num);

	sleep(2);

	if (shelf_num == 1)
	{
		//	X_Array = 400;tecan
		X_Array = 350 + scanoffset;  //430;
		memcpy(ordArray_plate, ordArray_plate1, (sizeof(ordArray_plate1)) / 3);
	}
	else if (shelf_num == 2)
	{
		memcpy(ordArray_plate, &ordArray_plate1[10], (sizeof(ordArray_plate1)) / 3);	
		//	X_Array = 1165;   //tecan
		#ifdef BIG_VERSION
		X_Array = 1165 + scanoffset;
		#else
		X_Array = 1115 + scanoffset;
		#endif
	}
	else if (shelf_num == 3)
	{
		//	X_Array = 1930;  //tecan
		#ifdef BIG_VERSION
		X_Array = 1980 + scanoffset;
		#else
		X_Array = 1880 + scanoffset;  // 1945;
		#endif
		memcpy(ordArray_plate, &ordArray_plate1[20], (sizeof(ordArray_plate1)) / 3);
	}else{
		return;
	}
		
	//	bs_packetshelfstreach( (shelf_num - 1) * 10,&cmd); //lock
	bs_packetshelfstreach( (shelf_num + 2) * 10, &cmd); //150ul 凸轮误差先到150ul位置扫描结束再回去
	pthread_mutex_lock(&mutex_mlock);	//1.29号补的
	set_minicmd(pcmd_head, cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");

	sleep(1);
	nt_sendpacketdata(SHELF_SCAN__START, (char*)&shelf_num, 1);

	
	pthread_mutex_lock(&mutex_mianarmlock);
	sendcmdbuf.srdevaddr = ARM_ADDR;

	for(i = 0;i < 10; i++)
	{	
		mb_monitdoorstate();
		
		sprintf(sendcmdbuf.cmdbuf, "PA %d %d %d", X_Array, ordArray_plate[i][1], MOV_ZH);//到达玻片位置
		// //	sprintf(CommandElemt.cmdbuf, "PA %d %d %d",ordArray_plate[i][0] + distance_scan, ordArray_plate[i][1], MOV_ZH);//到达玻片位置
		// if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
		// 		CommandElemt.srdevaddr , 1, 0, 1) > 0)
		// 	printf("[mb_shelfscanning]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
		// else
		// 	printf("[mb_shelfscanning]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
		// sb_waitingframeaswer(&CommandElemt);
		sm_serailsenddat(port_arm, &sendcmdbuf, 1, 0, 1);

		//	usleep(500000);
		sc_getscode(i + (shelf_num-1) * 10,FALSE);
		usleep(100000);      // 扫描时间久给其他线程分配CPU资源
	}
	
	/*
	memcpy(TCP_Buffer,&eenetevent,4); 
	memcpy(&TCP_Buffer[4],scan_OCR_data,scan_OCR_len);
	while(flg_netdisconnect) sleep(1);
	pthread_mutex_lock(&netsend_lock);
	if (send(local_fd, TCP_Buffer, sizeof(eenetevent) + scan_OCR_len, 0) < 0)
		perror("write net\n");
	pthread_mutex_unlock(&netsend_lock);
	*/

	//	usleep(100000);
	
	nt_sendpacketdata(SHELF_SCAN_END, (char *)&shelf_num, 1);

	bs_packetshelfstreach( (shelf_num - 1) * 10, &cmd); //150ul 凸轮误差先到150ul位置扫描结束再回去
	pthread_mutex_lock(&mutex_mlock);    //1.29号补的
	set_minicmd(pcmd_head, cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");

	sleep(2);

	while(!mini_work_finished[shelf_num - 1])
		usleep(10000);

	memset(&cmd,0,sizeof(cmd));
	cmd.cmd = RELOAD;
	cmd.minicmd_buffer[3] = (unsigned char)RELOAD;
	cmd.minicmd_buffer[4] = shelf_num;
	cmd.minicmd_num=2;
	
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	printf("RELOAD finished\n");

	if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mainarm_lock");
	printf("scanner_test finished*********************\n");

	lprintf(log_my, INFO, "In scanning shelf number[%d] compunation.\n", shelf_num);
}



/****************将流程中的试剂信息最高位置1***由于锁定时候判断***并将有效试剂加入outside***********/
void mb_cancelreagentwork(unsigned char reagent_num)
{
	operate_head_list* 	operate_head = NULL;
	operate_t* operate_p = NULL;
	unsigned char i = 0,j=0, regt_cnt = 0,start_reagent=0;
//	unsigned int eenetevent=SET_REAGENT_DISABLE;
	char TCP_Buffer[20] = {0};
	reagentoutside_list * reagentoutside_head_tmp = reagentoutside_head, *reagentoutside_head_lst = NULL;

	start_reagent = (reagent_num -1) * 9;
	
	for (i = 0; i < 9;i++)
	{
		if (strlen(reagent_code[start_reagent + i].code) != 0)//有试剂
		{
			reagentoutside_head_tmp = reagentoutside_head;
			do
			{	//already exist
				if (strcmp(reagent_code[(reagent_num -1) * 9 + i].code, reagentoutside_head_tmp->reagent_info.code) == 0)
					break;

				reagentoutside_head_lst = reagentoutside_head_tmp;
				reagentoutside_head_tmp = reagentoutside_head_tmp->next;
			}
			while(reagentoutside_head_tmp != NULL);

			if (reagentoutside_head_tmp == NULL)//not exist
			{
				while((reagentoutside_head_tmp = reagentoutside_head_lst->next 
					= (reagentoutside_list*)malloc(sizeof(reagentoutside_list))) == NULL)
				{
					sleep(1);
					printf("malloc error reagentoutside_list\n");
				}
				memcpy(&reagentoutside_head_tmp->reagent_info, &reagent_code[start_reagent + i], sizeof(reagent_t));
				reagentoutside_head_tmp->next = NULL;
			}
		}
	}
		
	reagentoutside_head_tmp = reagentoutside_head;
	printf("outside code \n");
	do
	{
		printf(" %s ", reagentoutside_head_tmp->reagent_info.code);
		reagentoutside_head_tmp = reagentoutside_head_tmp->next;
	}
	while(reagentoutside_head_tmp != NULL);
		
	printf("In mb_cancelreagentwork***************\n");
	pthread_mutex_lock(&head_lock);
	for (i = 0; i < 3;i++)
	{
		if (i == 0)
			operate_head = operate_head1;
		else if (i == 1)
			operate_head = operate_head2;
		else if (i == 2)
			operate_head = operate_head3;

		if (operate_head->operate.reagent == STOP_OPERATE)
			continue; 
		for (regt_cnt = 0; regt_cnt < 9; regt_cnt++)
		{		
			for(j = 0; j < 10; j++)
			{
				if (mix_DAB[i].ordArrayA[j] != 0)
				{
				  if (strcmp(mix_DAB[i].reagentA[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  	mix_DAB[i].ordArrayA[j] |= 0X80; 
				}
				if (mix_DAB[i].ordArrayB[j] != 0)
				{
					if (strcmp(mix_DAB[i].reagentB[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  		mix_DAB[i].ordArrayB[j] |= 0X80;
				}
				 
				if (mix_SECEND[i].ordArrayA[j] != 0)
				{
				  if (strcmp(mix_SECEND[i].reagentA[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  	mix_SECEND[i].ordArrayA[j] |= 0X80;
				  
				}
				if ((mix_SECEND[i].ordArrayB[j] != 0))
				{
				  if (strcmp(mix_SECEND[i].reagentB[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  	mix_SECEND[i].ordArrayB[j] |= 0X80;
				}
			}
		}
		while(operate_head != NULL)
		{
			operate_p = &(operate_head->operate);
			while(operate_p != NULL && operate_p->reagent != STOP_OPERATE)
			{
				//		if (operate_p->reagent < REAGENT_CASE)
				{
					for (regt_cnt = 0; regt_cnt < 9; regt_cnt++)
					{
						if (strcmp(operate_p->reagent_info.code, reagent_code[start_reagent + regt_cnt].code) == 0
							&& reagent_code[start_reagent + regt_cnt].code[0]!=0  && operate_p->reagent < REAGENT_CASE)
						{
							printf(" reagechaned %d ",operate_p->reagent);		
							 operate_p->reagent |= 0X80;
							 break;
						}
					}
				}	
				operate_p = operate_p->next;
			}		
			operate_head = operate_head->next_head;
		}
	}

	//	mb_printfoperatelist(operate_head);
	if (pthread_mutex_unlock(&head_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");

	TCP_Buffer[0] = reagent_num;	
	nt_sendpacketdata(SET_REAGENT_DISABLE, TCP_Buffer, 1);
	memset(&reagent_code[start_reagent], 0,(sizeof(reagent_t)) * 9);

	for (i = 0; i < 36;i++)
		printf(" %s ", reagent_code[i].code);

	printf("Out of mb_cancelreagentwork***************\n");
}


int CheckRemove(char reagent_index, reagent_t* p_r, reagent_t* s_r)
{
	char TCP_Buffer[24] ={0};
	
	if (reagent_code[ (uint8_t)reagent_index].reagent_kind[0] == 0)
	{
		printf("reagent shelf removed reagent num=%d\n", reagent_index);
		
		TCP_Buffer[0] = 0;
	//	memcpy(&TCP_Buffer[1], (const void *)0, 0);
		nt_sendpacketdata(REAGENT_REMOVED, TCP_Buffer, 21);
		return -1;
	}
	else
		memcpy(p_r, s_r, sizeof(reagent_t));
	
	return 0;
}

/*************试剂扫描后调用 用于修正流程中试剂信息***************/
void ChangeReagentInOperate(unsigned char reagent_num)
{
	operate_head_list* 	operate_head = operate_head1;
	unsigned char i = 0,j = 0,regt_cnt=0,start_reagent=0;
	operate_t* operate_p = NULL;

return;

	start_reagent =(reagent_num -1) * 9;
	printf("In ChangeReagentInOperate*******************\n");
	lprintf(log_my, INFO,"In ChangeReagentInOperate*******************\n");
	pthread_mutex_lock(&head_lock);
	//mb_printfoperatelist(operate_head3);
	for (i = 0; i < 3; i++)
	{
		printf("iiiiiiiiiiiiii =%d\n",i);
		if (i == 0)
			operate_head = operate_head1;
		else if (i == 1)
			operate_head = operate_head2;
		else if (i == 2)
			operate_head = operate_head3;

		if (operate_head->operate.reagent == STOP_OPERATE)
			continue;
		for (regt_cnt = 0; regt_cnt < 9; regt_cnt++)
		{
			for(j = 0; j < 10; j++)
			{
				if (mix_DAB[i].ordArrayA[j] != 0)
				{
				  if (strcmp(mix_DAB[i].reagentA[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  	mix_DAB[i].ordArrayA[j] = start_reagent + regt_cnt;
				   if (strcmp(mix_DAB[i].reagentB[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  	mix_DAB[i].ordArrayB[j] = start_reagent + regt_cnt;
				}
				if (mix_SECEND[i].ordArrayA[j] != 0)
				{
				  if (strcmp(mix_SECEND[i].reagentA[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  	mix_SECEND[i].ordArrayA[j] = start_reagent + regt_cnt;
				   if (strcmp(mix_SECEND[i].reagentB[j].code, reagent_code[start_reagent + regt_cnt].code) == 0) 
				  	mix_SECEND[i].ordArrayB[j] = start_reagent + regt_cnt;
				}
				
			}
		}
		while(operate_head != NULL)
		{
			
			operate_p = &(operate_head->operate);
		//	printf("operate_p->reagent=%d operate_head =%d operate_head->next_head=%d\n\n",operate_p->reagent,operate_head,operate_head->next_head);
			while(operate_p != NULL && operate_p->reagent != STOP_OPERATE)
			{
			//	printf("operate_p->reagent=%d ",operate_p->reagent);
				if ((operate_p->reagent & 0X7F) < REAGENT_CASE)
				{
					 
					// for (regt_cnt = 0; regt_cnt < 9; regt_cnt++)
					 for(j = 0; j < 36;j++)	
					{		
						if ( (strcmp(operate_p->reagent_info.code, reagent_code[j].code ) == 0))
						{
								operate_p->reagent = j;
							
							break;
						}				
					}
				}
				operate_p = operate_p->next;
			//	printf("next operate_p->reagent=%d ",operate_p->reagent);
			}
			
			operate_head = operate_head->next_head;
			// printf("operate_head=%d NULL=%d hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee", (int)operate_head, NULL);
		}
	}
	mb_printfoperatelist(operate_head3);
	if (pthread_mutex_unlock(&head_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");
	
	printf("Out of ChangeReagentInOperate*******************\n");
}

void mb_reagentstationscanner(unsigned int reagent_num)
{
	sr_cmdstruct_t CommandElemt;
	int i=0;

	unsigned int ordArray_plate[9][2] = {{0}};
	reagentoutside_list * reagentoutside_p = reagentoutside_head;
	unsigned char check_reagent[9] = {0}; //0 无效 1 需要测量
	unsigned int dwPinState, reagent_index = reagent_num - 1;

	if(flg_intosleep)
		return;

	printf("in mb_reagentstationscanner %d\n", reagent_num);

		/*
			sleep(5);
			reagent_check[reagent_index].NEED_SCAN = FALSE;
			GPIO_OutSet(reagent_check[reagent_index].lock);
			printf("scan reagent end  %d\n",reagent_num);
			return;
			*/
	
	if (reagent_num == 1)
		memcpy(ordArray_plate,ordArray_reagent,(sizeof(ordArray_reagent)) / 4);
	else if (reagent_num == 2)
		memcpy(ordArray_plate,&ordArray_reagent[9],(sizeof(ordArray_reagent)) / 4);
	else if (reagent_num == 3)
		memcpy(ordArray_plate,&ordArray_reagent[18],(sizeof(ordArray_reagent)) / 4);
	else if (reagent_num == 4)
		memcpy(ordArray_plate,&ordArray_reagent[27],(sizeof(ordArray_reagent)) / 4);
	else
		return;

	nt_sendpacketdata(REAGENT_SCAN_START, (char*)&reagent_num, 1);

	//	mb_checkshelfreagent(reagent_num,ordArray_plate, check_reagent);
	//	return;
	pthread_mutex_lock(&mutex_mianarmlock);

	if (SHELF_LOCK_WAY_NEW)
		GPIO_OutSet(reagent_check[reagent_index].lock);//锁住进行扫描
	else
		GPIO_OutClear(reagent_check[reagent_index].lock);//锁住进行扫描

	dwPinState = reagent_check[reagent_index].sen;
	GPIO_PinState(&dwPinState);
	if ((dwPinState & reagent_check[reagent_index].sen) == 1)//已经拿掉
	{
		if (SHELF_LOCK_WAY_NEW)
			GPIO_OutClear(reagent_check[reagent_index].lock);//锁住进行扫描
		else
			GPIO_OutSet(reagent_check[reagent_index].lock);//锁住进行扫描
	
		reagent_check[reagent_index].NEED_SCAN = FALSE;
		if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mainarm_lock");
		return;
	}

	CommandElemt.srdevaddr = ARM_ADDR;
	for(i = (reagent_num - 1) * 9;i < (reagent_num * 9); i++)
	{
		if(flg_intosleep)
			break;
	
		mb_monitdoorstate();
		//高的40 低的70
		if (new_scaner)
		sprintf(CommandElemt.cmdbuf, "PA %d %d %d",ordArray_reagent[i][0] - 10, ordArray_reagent[i][1], MOV_ZH);//到达玻片位置
		else
		sprintf(CommandElemt.cmdbuf, "PA %d %d %d",ordArray_reagent[i][0] + 22, ordArray_reagent[i][1], MOV_ZH);//到达玻片位置
		//	sprintf(CommandElemt.cmdbuf, "PA %d %d %d",ordArray_reagent[i][0] + 70, ordArray_reagent[i][1], MOV_ZH);//到达玻片位置
		if (sb_armpumpsend(port_arm, (unsigned char 	*)CommandElemt.cmdbuf, 
				CommandElemt.srdevaddr , 1, 0, 1) > 0)
	 			printf("[mb_reagentstationscanner]: Send message  to tecan device [%x]successful.\n" ,CommandElemt.srdevaddr );
			else
				printf("[mb_reagentstationscanner]: Send message  to tecan device [%x]failed.\n" ,CommandElemt.srdevaddr );
			sb_waitingframeaswer(&CommandElemt);
	
	
		if (!sc_getscode(i,TRUE)) //表示此坐标没放试剂//试剂序号30以后
		{
		//	ordArray_plate[i][0] = 0;
		//	ordArray_plate[i][1] = 0;
		}
		//	usleep(200000);
	}

	usleep(200000);
	nt_sendpacketdata(REAGENT_SCAN_END, (char*)&reagent_num, 1);

	printf("begin to do check work*********\n");
	//while (!special_reagent_recieved && !flg_netdisconnect)
	//	sleep(1);
	memset(check_reagent, 1, sizeof(check_reagent));

	for (i = (reagent_num - 1) * 9;i < reagent_num * 9;i++)
	{
	//	printf("i =%d %s --",i, &reagent_code[i].code);
		reagentoutside_p = reagentoutside_head;
		do
		{
			if (strcmp(reagent_code[i].code, reagentoutside_p->reagent_info.code) == 0)
			{
				check_reagent[i % 9] = 0;
				break;
			}
			reagentoutside_p = reagentoutside_p->next;
		}
		while(reagentoutside_p != NULL);	
	}
	
	ChangeReagentInOperate(reagent_num);

	if (!flg_netdisconnect)	//网络正常进行扫描
	{
	//	while(1)
	//		mb_checkshelfreagent(reagent_num,ordArray_plate, check_reagent);
	}
	sleep(3);//等待接收灌注信息
	if (SHELF_LOCK_WAY_NEW)
		GPIO_OutClear(reagent_check[reagent_index].lock);//锁住进行扫描
	else
		GPIO_OutSet(reagent_check[reagent_index].lock);//锁住进行扫描


	//	sleep(1);
	if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mainarm_lock");
	
	reagent_check[reagent_index].NEED_SCAN = FALSE;

	#ifdef BIG_VERSION
	GetMixStationVal(6,reagent_index);//解锁
	#endif
	
	/*	
		printf("reagent_code=");
		
		char TCP_Buffer[4] ={0},str[20] = {0};		//测试兼容试剂选择
	eenetevent = REAGENT_REPLACED;
		
	scanf("%d",&i);
	printf("i == %d\n",i);
	if (i == 0)
	{
		memcpy(TCP_Buffer,&eenetevent,4);
		TCP_Buffer[2] = 1;

		if (reagent_num == 1)
			TCP_Buffer[3] = 0;
		else if (reagent_num == 2)
			TCP_Buffer[3] = 9;
		else if (reagent_num == 3) 
			TCP_Buffer[3] = 18;
		else if (reagent_num == 4)
			TCP_Buffer[3] = 27;

			while(flg_netdisconnect) sleep(1);
			pthread_mutex_lock(&netsend_lock);
			if (send(local_fd, TCP_Buffer, sizeof(TCP_Buffer), 0) < 0)
				perror("write net\n");
			pthread_mutex_unlock(&netsend_lock);

		while (replace_reagent == 0xFF)	//等待接收PC端发送的代替试剂号
		{
				sleep(1);
				printf("wait for replace_reagent****\n");
		}
		printf("**replace_reagent = %d**\n", replace_reagent);
		replace_reagent = 0xFF;
	}	
	*/	
		//	sleep(20);//等待PC发送抗体 双氧水 DAB信息
	printf("OUT OF mb_reagentstationscanner %d\n", reagent_num);
}


void mb_mixstationscanner(void)
{
	sr_cmdstruct_t sendbuf;

	sendbuf.srdevaddr = ARM_ADDR;
	mb_monitdoorstate();
	sprintf(sendbuf.cmdbuf, "PA %d %d %d", ordArray_mixed_DAB[0][0] + SCAN_DISTANC_MIXSTATION, ordArray_mixed_DAB[0][1], 0);//到达玻片位置
	if (sb_armpumpsend(port_arm, (unsigned char *)sendbuf.cmdbuf, sendbuf.srdevaddr, 1, 0, 1) > 0)
		printf("[mb_mixstationscanner]: Send message  to tecan device [%x]successful.\n", sendbuf.srdevaddr );
	else
		printf("[mb_mixstationscanner]: Send message  to tecan device [%x]failed.\n", sendbuf.srdevaddr );
	sb_waitingframeaswer(&sendbuf);
	sc_getscode( 0XEE,FALSE ); //混合站序号100
}

/**********
针对小仪器，检测大容量试剂是否兼容
当为试剂架上试剂时传入参数值为-1
reagentA为当前滴加的试剂
return 1
return -1 不匹配
*********/
int mb_checkbakreagent(emREAGENT curreagent, emREAGENT nextreagent)
{
	if (curreagent == -1 && nextreagent != REAGENT_WASH && nextreagent != REAGENT_WATER)
		return -1;

	if (nextreagent == STOP_OPERATE || nextreagent == NO_REAGENT  || nextreagent > REAGENT_DAB)
		return -1;

	if (curreagent == NO_REAGENT || curreagent > REAGENT_DAB)
		return 1;
	
	switch (curreagent)
	{
	case REAGENT_DEWAX:
		if (nextreagent != REAGENT_ALCOHOL)
			return -1;
		break;
	case REAGENT_ALCOHOL:
		if (nextreagent == REAGENT_ER1 || nextreagent == REAGENT_ER2)
			return -1;
		break;
	case REAGENT_ER1:
		if (nextreagent == REAGENT_DEWAX || nextreagent == REAGENT_ALCOHOL || nextreagent == -1)
			return -1;
		break;
	case REAGENT_ER2:
		if (nextreagent == REAGENT_DEWAX || nextreagent == REAGENT_ALCOHOL || nextreagent == -1)
			return -1;
	break;
	case REAGENT_WASH:
		if (nextreagent == REAGENT_ER1 || nextreagent == REAGENT_ER2)// 脱蜡分开操作
			return -1;
	break;
	case REAGENT_WATER:
		if (nextreagent == REAGENT_DEWAX)
			return -1;
	break;
	case NO_REAGENT:
		return 1;
	break;
	default: return -2;
	}
	return 1;
}

/*   检查试剂瓶内剩余试剂量是否大于需要滴加的量(val)    */
int IsCompatibleReagentAvaliable(char i, int val )
{
	//printf("val1=%d val2=%d\n", reagent_code[i].val, val);
	if (reagent_code[(uint8_t)i].val >= val)  //DAB大于100ul可用于替换
		return (int)i;

	return -1;
}

/**********
liquid_num 片数量
reagent 试剂类型
return 位置信息
return i || 0X80; 不同批号
return -1 当匹配到WASH
val 需要的试剂两
  //========   mb_searchcomatiblereagent :  根据上位机发送来的试剂参数的编码来查找试剂架上处于哪个位置（实际架位置从面对设备操作正面从里往外1,2.。。）
*********/
int mb_searchcomatiblereagent(reagent_t * reagent, int val)
{
	uint32_t i = 0, reg_cnt = 0;
	
	lprintf(log_my, INFO,"in mb_searchcomatiblereagent, reagent->code=%s reagent_kind=%s\n",reagent->code, reagent->reagent_kind);

	printf("in mb_searchcomatiblereagent, reagent->code=%s reagent_kind=%s\n",reagent->code,reagent->reagent_kind);
	for (i = 0; i < 36;i++)
	{
		printf(" %s %s*", reagent_code[i].code, reagent_code[i].reagent_kind);
		lprintf(log_my, INFO, " %s %s*", reagent_code[i].code, reagent_code[i].reagent_kind);
	}
	for (i = 0; i < 36;i++)//找同一试剂系统
	{	
		if (strcmp(reagent->code, reagent_code[i].code) == 0 && reagent_code[i].code[0] != 0)
			break;
	}
	reg_cnt = i;
	if (i != 36)//同一试剂系统上找
	{	printf("search in the same reagent system\n");
		for (i = reg_cnt / 9 * 9; i < (reg_cnt / 9 + 1) * 9; i++)
		{
			if (strcmp(reagent->reagent_kind, reagent_code[i].reagent_kind) == 0 )
			{
				if (IsCompatibleReagentAvaliable(i, val) >= 0)
					return i;
			}
		}
	}
	
	printf("search as the same lot\n");
	for (i = 0; i < 36;i++)//同批号
	{	
		if ((strcmp(reagent->reagent_kind, reagent_code[i].reagent_kind) == 0)
			&& (strcmp(reagent_code[i].lot_num, reagent->lot_num) == 0) )
		{
			if (IsCompatibleReagentAvaliable(i, val) >= 0)
				return i;
		}
			
	}

	printf("search as the different lot\n");
	for (i = 0; i < 36;i++)//不同批号
	{	
		if ((strcmp(reagent->reagent_kind, reagent_code[i].reagent_kind) == 0))
		{
			if (IsCompatibleReagentAvaliable(i, val) >= 0)
				return i;
		}
	}
	printf("not find\n");
	return -1;
}

/******************可能同时滴加大容量和试剂架试剂，试剂架试剂可能同时滴加10种不同抗体*******************/
int mb_aspiratedispensework(dispense_t* dispense)
{
	unsigned char plate_num_cmd = 0;//用来确定发送至哪个minibcmd
	unsigned char i = 0;
	
	printf("in mb_aspiratedispensework\n");
	printf("dispense=");
	
	lprintf(log_my, INFO,"in mb_aspiratedispensework\n");
	
	while(i < 11 && dispense[i].reagent != STOP_OPERATE)
	{
		i++;
	}
		
	plate_num_cmd = dispense[0].plate_num;

	//*************试剂架试剂 被拿掉**************// 混合试剂被拿掉再前面混合时已经等待所以要滴加时已经混合好
	
	if (((plate_num_cmd % 30) < 10) && (flg_mianarmstop_a))	//主臂操作完成
	{
		printf("copy dispense to dispenseA\n");
		memcpy((char*)dispenseA, (const char*)dispense, sizeof(dispense_t)*11);
		flg_mianarmstop_a = FALSE;
	}
	else if (((plate_num_cmd % 30) < 20) && (flg_mianarmstop_b))	//主臂操作完成
	{
		printf("copy dispense to dispenseB\n");
		memcpy((char*)dispenseB, (const char*)dispense, sizeof(dispense_t)*11);
		flg_mianarmstop_b = FALSE;
	}
	else if (((plate_num_cmd % 30) < 30) && (flg_mianarmstop_c))	//主臂操作完成
	{
		printf("copy dispense to dispenseC\n");
		memcpy((char*)dispenseC, (const char*)dispense, sizeof(dispense_t)*11);
		flg_mianarmstop_c = FALSE;
	}
		
	printf("mb_aspiratedispensework finished \n");
	printf("\n");

	return 0;
}


void mb_mainworkstop(unsigned char shelf_num)
{
	stminibd_sendpacket cmd;
	struct timeval now;
	
	assert(shelf_num <= 3 && shelf_num > 0);

	pthread_mutex_lock(&mutex_mneedworked);
	
	printf("stop_mini_work %d finished*********************\n",shelf_num);

	lprintf(log_my, INFO,"stop_mini_work %d finished*********************\n",shelf_num);

	memset (&mix_DAB[shelf_num-1], 0, sizeof(mix_t));
	memset (&mix_SECEND[shelf_num-1], 0, sizeof(mix_t));

	shelf_stirmode[shelf_num - 1] = 0;
	//	bzero(&shelf_stirtime[shelf_num - 1][0], 12);
	memset((uint8_t*)&shelf_stirtime[shelf_num - 1][0], 0, 12);

	reagent_lock[0][shelf_num - 1] = 0;
	reagent_lock[1][shelf_num-1] = 0;
	reagent_lock[2][shelf_num-1] = 0;
	reagent_lock[3][shelf_num-1] = 0;
				
	gettimeofday(&now, NULL);

	if (shelf_num == 1)
	{
		flg_opwork1ready = false;
		if (operate_pri == operate_head1)
			operate_pri = NULL;
		pthread_mutex_lock(&head_lock);
		mb_setoperateheaddisable((operate_head_list**)&operate_head1, true);
		if (pthread_mutex_unlock(&head_lock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");
		tc_packettempcmd(STOP_CONTROL1);	

		bs_packetshelfstreach(0, &cmd);	
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error m_lock");
		//	pthread_mutex_lock(&mutex_mneedworked);
		while((mini_work_finished[0]))//确保miniBoard线程已置FALSE
			usleep(1000);
			sleep(1);
		while(!(mini_work_finished[0]))//动作完成
			usleep(200000);
		//	pthread_mutex_unlock(&mutex_mneedworked);
	
		memset(&mini_cmd,0,sizeof(mini_cmd));
		mini_cmd.cmd = MAINTAIN_FAN_WORK;
		mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
		mini_cmd.minicmd_buffer[4] = 4;
		mini_cmd.minicmd_num=2;
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,mini_cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");

		//		mb_dischargeshelfwateliquid(1);	
		which_mix_kindA=0;
		isDAB_mixedA = false;
		isDAB_mixedA_next = false;
		workstep_mix_a = 0;
		mixstation_clear_state[0].AFTER_DISPENSE = true;	
		mixstation_clear_state[1].AFTER_DISPENSE = true;	
		startmixworkid &= 0X7F; 
		if (DAB1Array_exchange)
			DAB1Array_exchange = false;
		else
			DAB1Array_exchange = true;
		
		HEAD1_STEP_SENDED = true;
		wkeventA = STOP_WORK;
		workstep_state_a = 0X01110000;
		last_reagentA = NO_REAGENT;
		CriticalWork &= 0X7F;
		flg_mianarmstop_a = true;
		StartDispenseA = false;
		//bzero(&lastt_kind[0][0], 9);
		memset((uint8_t*)&lastt_kind[0][0], 0, 9);
	}
	else if (shelf_num == 2)
	{
		if (operate_pri == operate_head2)
				operate_pri = NULL;
		flg_opwork2ready = false;
		
		pthread_mutex_lock(&head_lock);
		mb_setoperateheaddisable((operate_head_list**)&operate_head2, true);
		if (pthread_mutex_unlock(&head_lock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_Lock");
		tc_packettempcmd(STOP_CONTROL2);

		#ifndef BIG_VERSION
		bs_packetshelfstreach(10,&cmd);
			
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
		//	pthread_mutex_lock(&mutex_mneedworked);
		while((mini_work_finished[1]))//确保miniBoard线程已置FALSE
			usleep(1000);
			sleep(1);
		while(!(mini_work_finished[1]))//动作完成
			usleep(200000);
		//	pthread_mutex_unlock(&mutex_mneedworked);

		memset(&mini_cmd,0,sizeof(mini_cmd));
		mini_cmd.cmd = MAINTAIN_FAN_WORK;
		mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
		mini_cmd.minicmd_buffer[4] = 5;
		mini_cmd.minicmd_num=2;
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,mini_cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
		//	mb_dischargeshelfwateliquid(2);
		#endif

		which_mix_kindB=0;
		isDAB_mixedB = false;
		isDAB_mixedB_next = false;
		workstep_mix_b = 0;
		mixstation_clear_state[2].AFTER_DISPENSE = true;	
		mixstation_clear_state[3].AFTER_DISPENSE = true;
		startmixworkid &= 0XBF; 
		if (DAB2Array_exchange)
			DAB2Array_exchange = false;
		else
			DAB2Array_exchange = true;
		
		HEAD2_STEP_SENDED = true;
		wkeventB = STOP_WORK;
		workstep_state_b = 0X01110000;
		last_reagentB = NO_REAGENT;
		CriticalWork &= 0XBF;
		flg_mianarmstop_b = true;
		StartDispenseB = false;
		//	bzero(&lastt_kind[1][0], 9);
		memset((char*)&lastt_kind[1][0], 0, 9);
	}
	else if (shelf_num == 3)
	{
		if (operate_pri == operate_head3)
				operate_pri = NULL;
		flg_opwork3ready = false;
		
		pthread_mutex_lock(&head_lock);
		mb_setoperateheaddisable((operate_head_list**)&operate_head3, true);
		if (pthread_mutex_unlock(&head_lock) != 0)
			lprintf(log_my, ERROR,"%s", "pthread_mutex_unlock error head_Lock");
		
		tc_packettempcmd(STOP_CONTROL3);

		bs_packetshelfstreach(20, &cmd);
			
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s", "pthread_mutex_unlock error mlock");
		//	pthread_mutex_lock(&mutex_mneedworked);
		while((mini_work_finished[2]))//确保miniBoard线程已置FALSE
			usleep(1000);
			sleep(1);
		while(!(mini_work_finished[2]))//动作完成
			usleep(200000);
		//	pthread_mutex_unlock(&mutex_mneedworked);
			
		if (pthread_mutex_unlock(&head_lock) !=0 )
			lprintf(log_my, ERROR,"%s", "pthread_mutex_unlock error head_lock");

		memset(&mini_cmd,0,sizeof(mini_cmd));
		mini_cmd.cmd = MAINTAIN_FAN_WORK;
		mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
		mini_cmd.minicmd_buffer[4] = 6;
		mini_cmd.minicmd_num=2;
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,mini_cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
		//			mb_dischargeshelfwateliquid(3);

		which_mix_kindC=0;
		isDAB_mixedC = false;
		isDAB_mixedC_next = false;
		workstep_mix_c = 0;
		mixstation_clear_state[4].AFTER_DISPENSE = true;	
		mixstation_clear_state[5].AFTER_DISPENSE = true;
		startmixworkid &= 0XDF; 
		if (DAB3Array_exchange)
			DAB3Array_exchange = false;
		else
			DAB3Array_exchange = true;
		
		HEAD3_STEP_SENDED = true;
		wkeventC = STOP_WORK;
		workstep_state_c = 0X01110000;
		last_reagentC = NO_REAGENT;
		CriticalWork &= 0XDF;
		flg_mianarmstop_c = true;
		StartDispenseC = false;
		//bzero(&lastt_kind[2][0], 9);
		memset((uint8_t*)&lastt_kind[2][0], 0, 9);
	}

	memset(&cmd,0,sizeof(cmd));
	#ifdef glint
	cmd.cmd = WORK_FINISH;//RELOAD;
	cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
	#else
	cmd.cmd = RELOAD;//RELOAD;
	cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
	#endif
	cmd.minicmd_buffer[4] = shelf_num;
	cmd.minicmd_num=2;
	{
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	}

	pthread_mutex_unlock(&mutex_mneedworked);
}


 //  计算搅拌时间
void mb_mixprogressintime(unsigned int work_time, operate_head_list* next_head)
{
	int pre_time = 20*60;
	unsigned char reagentA = 0, reagentB = 0, reagentC = 0;

	reagentA = operate_head1->operate.reagent;	
	reagentB = operate_head2->operate.reagent;	
	reagentC = operate_head3->operate.reagent;

#if DE_TONG
//	printf("headsel reagentB=%d, reagentC=%d\n",reagentB,reagentC);
	if ((reagentA == REAGENT_DAB && isDAB_mixedA) || (reagentA == REAGENT_SECEND && isDAB_mixedA_next))
	{
		return;
	}
	if ((reagentB == REAGENT_DAB && isDAB_mixedB) || (reagentB == REAGENT_SECEND && isDAB_mixedB_next))
	{
		return;
	}
	if ((reagentC == REAGENT_DAB && isDAB_mixedC) || (reagentC == REAGENT_SECEND && isDAB_mixedC_next))
	{
		return;
	}
#endif

	//滴加混合试剂前20分钟必须进行混合
	if (dispense_mixreagent_timeA != 0 && (dispense_mixreagent_timeA - work_time) <= pre_time 
		&& !isDAB_mixedA && flg_opwork1ready)
	{
		startmixworkid = startmixworkid | 0X80;
		which_mix_kindA = 1;
	}
	else if (dispense_mixreagent_timeB != 0 && (dispense_mixreagent_timeB - work_time) <= pre_time 
		&& !isDAB_mixedB && flg_opwork2ready)
	{
		startmixworkid = startmixworkid | 0X40;
		which_mix_kindB = 1;
	}
	else if (dispense_mixreagent_timeC != 0 && (dispense_mixreagent_timeC - work_time) <= pre_time 
		&& !isDAB_mixedC && flg_opwork3ready)
	{
		startmixworkid = startmixworkid | 0X20;
		which_mix_kindC = 1;
	}
	#ifdef LAIYUE
		pre_time = 5*60;
	#else 
		pre_time = 5;
	#endif

	if (dispense_mixreagent_timeA_next != 0 && (dispense_mixreagent_timeA_next - work_time) <= pre_time 
				&& !isDAB_mixedA_next && flg_opwork1ready )
	{
		startmixworkid = startmixworkid | 0X80;
		which_mix_kindA = 2;
	}
	else if (dispense_mixreagent_timeB_next != 0 && (dispense_mixreagent_timeB_next - work_time) <= pre_time 
			&& !isDAB_mixedB_next && flg_opwork2ready)
	{
		startmixworkid = startmixworkid | 0X40;
		which_mix_kindB = 2;
	}
	else if (dispense_mixreagent_timeC_next != 0 && (dispense_mixreagent_timeC_next - work_time) <= pre_time  
			&& !isDAB_mixedC_next && flg_opwork3ready)
	{
		startmixworkid = startmixworkid | 0X20;
		which_mix_kindC = 2;
	}
}


char mb_getminnumber(unsigned int a, unsigned int b, unsigned int c)
{
	uint8_t numtem = 0;
	unsigned int data[3] = {268435455, 268435455, 268435455};
	
	if (flg_opwork1ready)
		data[0] = a;
	if (flg_opwork2ready)
		data[1] = b;
	if (flg_opwork3ready)
		data[2] = c; 
	
	numtem = data[0] < data[1] ? 0 : 1;
	numtem = data[numtem] < data[2] ? numtem : 2;

	if (NEED_PRINTF)
		printf("mininumber data[0] =%d data[1] =%d data[2] =%d mb_getminnumber numtem=%d\n", data[0],data[1],data[2],numtem);
	return numtem;
}


int mb_opheadheaterpriorit(operate_head_list* operate_head_table[])
{
	int i = 0;
	bool in_temper_control = false;

	if (operate_pri == operate_head1)   // 整个实验从 A 架开始
		in_temper_control = flg_temperheating1;
	else if (operate_pri == operate_head2)
		in_temper_control = flg_temperheating2;
	if (operate_pri == operate_head3)
		in_temper_control = flg_temperheating3;
	
	for (i = 0; i < 3; i++)
	{
		if (operate_pri == operate_head_table[i] && operate_pri != NULL && !in_temper_control)
		{
			operate_head_table[0] = operate_head_table[1] = operate_head_table[2] = NULL;
			memset(operate_head_table, 0, sizeof(operate_head_table));
			operate_head_table[i] = (operate_head_list*)operate_pri;
			return 0;
		}
	}
	
	if(operate_head_table[0] != NULL || operate_head_table[1] != NULL || operate_head_table[2] != NULL)
		return 0;

	return -1;
}


int mb_opprocepriorit(operate_head_list* operate_head_table[])
{
	uint8_t reagentA = 0, reagentB = 0, reagentC = 0;
	char reagent_tstA = 0, reagent_tstB = 0, reagent_tstC = 0;
	struct timeval now;
	uint32_t tolerate_time = 0, now_time = 0;
	char cmpcnt = 0;

	reagentA = operate_head1->operate.reagent;	
	reagentB = operate_head2->operate.reagent;	
	reagentC = operate_head3->operate.reagent;
	gettimeofday(&now, NULL);
	tolerate_time = now.tv_sec - begin_time.tv_sec +  10;
	now_time = now.tv_sec - begin_time.tv_sec;
	#if (USE_LOG_INFO == 1)
	{		
		printf("tolerate_time %d operate_head1->operate.time = %d operate_head2->operate.time = %d operate_head3->operate.time = %d\n",\
			tolerate_time, operate_head1->operate.time, operate_head2->operate.time, operate_head3->operate.time);
	}
	#endif	
	if (reagentA == STOP_OPERATE && wkeventA == BUSY_WORK)   // 还没执行停止操作
	{
		operate_head_table[0] = operate_head1;	
	}
	if (reagentB ==  STOP_OPERATE && wkeventB == BUSY_WORK)
	{
		operate_head_table[1] = operate_head2;
	}
	if (reagentC ==  STOP_OPERATE && wkeventC == BUSY_WORK)
	{
		operate_head_table[2] = operate_head3;
	}

	// #if(USE_LOG_INFO == 1)
	// lprintf(log_my, INFO, "STOP_OPERATE reagentA[%d] reagentB[%d] reagentC[%d], wkeventA[%d], wkeventB[%d], wkeventC[%d].\n", 
	// 	reagentA, reagentB, reagentC, wkeventA, wkeventB, wkeventC);
	// printf("STOP_OPERATE reagentA[%d] reagentB[%d] reagentC[%d], wkeventA[%d], wkeventB[%d], wkeventC[%d].\n", 
	// 	reagentA, reagentB, reagentC, wkeventA, wkeventB, wkeventC);
	// #endif

	if (mb_opheadheaterpriorit(operate_head_table) >= 0)    //  
		return 0;
	
	#if DE_TONG
	if (reagentA >= REAGENT_SECEND)
	{
		operate_head_table[0] = operate_head1;
		return 1;
	}
	if (reagentB >= REAGENT_SECEND)
	{
		operate_head_table[1] = operate_head2;
		return 2;
	}
	if (reagentC >= REAGENT_SECEND)
	{
		operate_head_table[2] = operate_head3;
		return 3;
	}
	#endif

	printf("mb_opprocepriorit first\n");

	//小仪器关键试剂在这里判断
	if ((last_reagentA == REAGENT_DEWAX || last_reagentA == REAGENT_ALCOHOL )       // 上次是脱腊或酒精大容量液且本架，且上位机发送了一架最后STOP_OPREATE命令标志flg_opwork1ready 
			&& (operate_head1->operate.time <= tolerate_time) && flg_opwork1ready)  // operate_head1->operate.time : 一架流程的总设定时间
	{     //  tolerate_time = nowtime - begingtime (现在时间减去 开机时的时间。（开机启动时间）)
		operate_head_table[0] = operate_head1;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentA[%d], operatetime[%d], toletime[%d].\n", last_reagentA, \
				operate_head1->operate.time, tolerate_time);
		#endif
	}
	if ((last_reagentB == REAGENT_DEWAX || last_reagentB == REAGENT_ALCOHOL )
			&& (operate_head2->operate.time <= tolerate_time) && flg_opwork2ready)
	{
		operate_head_table[1] = operate_head2;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentB[%d], operatetime[%d], toletime[%d].\n", last_reagentB, \
				operate_head2->operate.time, tolerate_time);
		#endif
	}
	if ((last_reagentC == REAGENT_DEWAX  || last_reagentC == REAGENT_ALCOHOL )
			&& (operate_head3->operate.time <= tolerate_time) && flg_opwork3ready)
	{
		operate_head_table[2] = operate_head3;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentC[%d], operatetime[%d], toletime[%d].\n", last_reagentC, \
				operate_head3->operate.time, tolerate_time);
		#endif
	}

	// #if(USE_LOG_INFO == 1)
	// lprintf(log_my, INFO, "REAGENT_DEWAX_ALCOHOL last_reagentA[%d] last_reagentB[%d] last_reagentC[%d], operate_head_table[0][%d], operate_head_table[2][%d], operate_head_table[2][%d].\n", 
	// 	last_reagentA, last_reagentB, last_reagentC, operate_head_table[0], operate_head_table[1], operate_head_table[2]);
	// printf( "REAGENT_DEWAX_ALCOHOL last_reagentA[%d] last_reagentB[%d] last_reagentC[%d], operate_head_table[0][%d], operate_head_table[2][%d], operate_head_table[2][%d].\n", 
	// 	last_reagentA, last_reagentB, last_reagentC, operate_head_table[0], operate_head_table[1], operate_head_table[2]);
	// #endif

	if (mb_opheadheaterpriorit(operate_head_table) >= 0)
		return 0;

	printf("mb_opprocepriorit first agian\n");

	if (( last_reagentA < REAGENT_CASE  || last_reagentA >= REAGENT_DAB)
			&& (operate_head1->operate.time <= tolerate_time) && flg_opwork1ready)
	{
		operate_head_table[0] = operate_head1;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentA 2 [%d], operatetime[%d], toletime[%d].\n", last_reagentA, \
				operate_head1->operate.time, tolerate_time);
		#endif
	}
	if (( last_reagentB < REAGENT_CASE  || last_reagentB >= REAGENT_DAB)
			&& (operate_head2->operate.time <= tolerate_time) && flg_opwork2ready)
	{
		operate_head_table[1] = operate_head2;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentB 2 [%d], operatetime[%d], toletime[%d].\n", last_reagentB, \
				operate_head2->operate.time, tolerate_time);
		#endif
	}
	if ((last_reagentC < REAGENT_CASE  || last_reagentC >= REAGENT_DAB)
			&& (operate_head3->operate.time <= tolerate_time) && flg_opwork3ready)
	{
		operate_head_table[2] = operate_head3;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentC 2 [%d], operatetime[%d], toletime[%d].\n", last_reagentC, \
				operate_head3->operate.time, tolerate_time);
		#endif
	}
	
	// #if(USE_LOG_INFO == 1)
	// lprintf(log_my, INFO, "last_reagentA[%d] last_reagentB[%d] last_reagentC[%d], operate_head_table[0][%d], operate_head_table[2][%d], operate_head_table[2][%d].\n", 
	// 	last_reagentA, last_reagentB, last_reagentC, *(operate_head_table[0]), *(operate_head_table[1]), *(operate_head_table[2]));
	// printf("last_reagentA[%d] last_reagentB[%d] last_reagentC[%d], operate_head_table[0][%d], operate_head_table[2][%d], operate_head_table[2][%d].\n", 
	// 	last_reagentA, last_reagentB, last_reagentC, *(operate_head_table[0]), *(operate_head_table[1]), *(operate_head_table[2]));
	// #endif

	if (mb_opheadheaterpriorit(operate_head_table) >= 0)
		return 0;

	printf("reagent_flag = %d reagentA = %d reagentB = %d, reagentC = %d\n", reagent_flag, reagentA, reagentB, reagentC);
	printf("mb_opprocepriorit secend\n");

	if (reagentA == NO_REAGENT && (operate_head1->operate.time <= tolerate_time)&& flg_opwork1ready) 
	{
		 operate_head_table[0] = operate_head1;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentA 3 NO_REAGENT, operatetime[%d], toletime[%d].\n", \
				operate_head1->operate.time, tolerate_time);
		#endif
	}
	if (reagentB == NO_REAGENT && (operate_head2->operate.time <= tolerate_time) && flg_opwork2ready)
	{
		operate_head_table[1] = operate_head2;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentB 3 NO_REAGENT, operatetime[%d], toletime[%d].\n", \
				operate_head2->operate.time, tolerate_time);
		#endif
	}
	if (reagentC == NO_REAGENT && (operate_head3->operate.time <= tolerate_time) && flg_opwork3ready)
	{
		operate_head_table[2] = operate_head3;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentC 3 NO_REAGENT, operatetime[%d], toletime[%d].\n", \
				operate_head3->operate.time, tolerate_time);
		#endif
	}

	// #if(USE_LOG_INFO == 1)
	// lprintf(log_my, INFO, "NO_REAGENT reagentA[%d] reagentB[%d] reagentC[0][%d], operate_head_table[0][%d], operate_head_table[2][%d], operate_head_table[2][%d].\n", 
	// 	reagentA, reagentB, reagentC, operate_head_table[0], operate_head_table[1], operate_head_table[2]);
	// printf("NO_REAGENT reagentA[%d] reagentB[%d] reagentC[0][%d], operate_head_table[0][%d], operate_head_table[2][%d], operate_head_table[2][%d].\n", 
	// 	reagentA, reagentB, reagentC, operate_head_table[0], operate_head_table[1], operate_head_table[2]);
	// #endif

	if (mb_opheadheaterpriorit(operate_head_table) >= 0)
		return 0;
	
	printf("mb_opprocepriorit third\n");

		//下次为大容量试剂和试剂架试剂首先滴加试剂架试剂
	if((reagentA < 37 ||reagentA >= REAGENT_DAB ) && (operate_head1->operate.time <= tolerate_time) && flg_opwork1ready)
	{
		operate_head_table[0] = operate_head1;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentA 4 [%d], operatetime[%d], toletime[%d].\n", last_reagentA, \
				operate_head1->operate.time, tolerate_time);
		#endif
	}
	if((reagentB < 37 ||reagentB >= REAGENT_DAB ) && (operate_head2->operate.time <= tolerate_time) && flg_opwork2ready)
	{
		operate_head_table[1] = operate_head2;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentB 4 [%d], operatetime[%d], toletime[%d].\n", last_reagentB, \
				operate_head2->operate.time, tolerate_time);
		#endif
	}
	if((reagentC < 37 ||reagentC >= REAGENT_DAB ) && (operate_head3->operate.time <= tolerate_time) && flg_opwork3ready)
	{
		operate_head_table[2] = operate_head3;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReag 4 [%d], operatetime[%d], toletime[%d].\n", last_reagentC, \
				operate_head3->operate.time, tolerate_time);
		#endif
	}
		
	#if(USE_LOG_INFO == 1)
	lprintf(log_my, INFO, "reagentA[%d] reagentB[%d] reagentC[0][%d], operate_head1->time[%d], operate_head2->time[%d], operate_head3->time[%d].\n", \
		reagentA, reagentB, reagentC, operate_head1->operate.time, operate_head2->operate.time, operate_head3->operate.time);
	#endif
	
	if (mb_opheadheaterpriorit(operate_head_table) >= 0)
		return 0;

	printf("mb_opprocepriorit forth\n");
	//在允许等待的时间内找同种试剂
	if (reagent_flag == reagentA && (operate_head1->operate.time <= tolerate_time) && \
			flg_opwork1ready && temper_control1[0].state != START_FAN)
	{
		operate_head_table[0] = operate_head1;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentA 5 [%d], operatetime[%d], toletime[%d], temperSTATE=[%d].\n", last_reagentA, \
				operate_head1->operate.time, tolerate_time, temper_control1[0].state);
		#endif
	}
	if (reagent_flag == reagentB && (operate_head2->operate.time <= tolerate_time) && \
			flg_opwork2ready && temper_control2[0].state != START_FAN)
	{
		operate_head_table[1] = operate_head2;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentB 5 [%d], operatetime[%d], toletime[%d], temperSTATE=[%d].\n", last_reagentB, \
				operate_head2->operate.time, tolerate_time, temper_control2[0].state);
		#endif
	}
	if (reagent_flag == reagentC && (operate_head3->operate.time <= tolerate_time) && \
			flg_opwork3ready && temper_control3[0].state != START_FAN)
	{
		operate_head_table[2] = operate_head3;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentC 5 [%d], operatetime[%d], toletime[%d], temperSTATE=[%d].\n", last_reagentC, \
				operate_head1->operate.time, tolerate_time, temper_control3[0].state);
		#endif
	}
		
	// #if(USE_LOG_INFO == 1)
	// lprintf(log_my, INFO, "reagent_flag[%d] tolerate_time[%d] operate_head_table[0][%d], operate_head_table[1][%d],operate_head_table[2][%d],.\n", 
	// 	reagent_flag, tolerate_time, operate_head_table[0], operate_head_table[1], operate_head_table[2]);
	// #endif

	if (mb_opheadheaterpriorit(operate_head_table) >= 0)
		return 0;

	printf("mb_opprocepriorit fifth\n");
	reagent_tstA = reagentA < 37 ? -1 : reagentA;
	reagent_tstB = reagentB < 37 ? -1 : reagentB;
	reagent_tstC = reagentC < 37 ? -1 : reagentC;
	//下次为大容量试剂找与上次使用过的大容量试剂相同
	if(last_cabin_reagent == reagent_tstA  && (operate_head1->operate.time <= tolerate_time) &&\
			 flg_opwork1ready && temper_control1[0].state != START_FAN)
	{
		operate_head_table[0] = operate_head1;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentA 6 [%d], operatetime[%d], toletime[%d], temperSTATE=[%d].\n", reagentA, \
				operate_head1->operate.time, tolerate_time, temper_control1[0].state);
		#endif
	}
	if(last_cabin_reagent == reagent_tstB  && (operate_head2->operate.time <= tolerate_time) && \
			flg_opwork2ready && temper_control2[0].state != START_FAN)
	{
		operate_head_table[1] = operate_head2;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentB 6 [%d], operatetime[%d], toletime[%d], temperSTATE=[%d].\n", reagentB, \
				operate_head2->operate.time, tolerate_time, temper_control2[0].state);
		#endif
	}
	if(last_cabin_reagent == reagent_tstC  && (operate_head3->operate.time <= tolerate_time) && \
			flg_opwork3ready && temper_control3[0].state != START_FAN)
	{
		operate_head_table[2] = operate_head3;
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, INFO, "OpProcePriorit lastReagentC 6 [%d], operatetime[%d], toletime[%d], temperSTATE=[%d].\n", reagentC, \
				operate_head3->operate.time, tolerate_time, temper_control3[0].state);
		#endif
	}

	// #if(USE_LOG_INFO == 1)
	// lprintf(log_my, INFO, "last_cabin_reagent[%d] tolerate_time[%d] operate_head_table[0][%d], operate_head_table[1][%d],operate_head_table[2][%d],.\n", 
	// 	last_cabin_reagent, tolerate_time, operate_head_table[0], operate_head_table[1], operate_head_table[2]);
	// #endif

	if (mb_opheadheaterpriorit(operate_head_table) >= 0)
		return 0;
	
	printf("mb_opprocepriorit sexth\n");
	//在允许等待的时间内找配伍的试剂
	
	if ((mb_checkbakreagent(reagent_flag, reagent_tstA) > 0) && (operate_head1->operate.time <= tolerate_time) && \
				flg_opwork1ready && temper_control1[0].state != START_FAN)
		operate_head_table[0] = operate_head1;
	
	if ((mb_checkbakreagent(reagent_flag, reagent_tstB) > 0) && (operate_head2->operate.time <= tolerate_time) && \
				flg_opwork2ready && temper_control2[0].state != START_FAN)
		operate_head_table[1] = operate_head2;
	
	if ((mb_checkbakreagent(reagent_flag, reagent_tstC) > 0) && (operate_head3->operate.time <= tolerate_time) && \
				flg_opwork3ready && temper_control3[0].state != START_FAN)
		operate_head_table[2] = operate_head3;
	// #if(USE_LOG_INFO == 1)
	// lprintf(log_my, INFO, "checkbakreagent reagent_flag[%d] operate_head_table[0][%d], operate_head_table[1][%d],operate_head_table[2][%d],.\n", 
	// 	reagent_flag, operate_head_table[0], operate_head_table[1], operate_head_table[2]);
	// #endif

	if (mb_opheadheaterpriorit(operate_head_table) >= 0)
		return 0;
	/*
	//按优先级
	if (NEED_PRINTF)
		printf("mb_opprocepriorit seventh\n");
	if (operate_pri == operate_head1)
		operate_head_table[0] = operate_head1;
	if (operate_pri == operate_head2)
		operate_head_table[1] = operate_head2;
	if (operate_pri == operate_head3)
		operate_head_table[2] = operate_head3;
	if (operate_head_table[0] != NULL || operate_head_table[1] != NULL ||
		operate_head_table[2] != NULL)//有配伍试剂
		return 0;
	*/
	//按时间顺序
	printf("mb_opprocepriorit eighth\n");

	cmpcnt = mb_getminnumber(operate_head1->operate.time, operate_head2->operate.time, operate_head3->operate.time);
	if (cmpcnt == 0)
		operate_head_table[0] = operate_head1;
	else if(cmpcnt == 1)
		operate_head_table[1] = operate_head2;
	else if (cmpcnt == 2)
		operate_head_table[2] = operate_head3;

	#if(USE_LOG_INFO == 1)
	lprintf(log_my, INFO, "cal minnumber time1[%d], time2[%d], time3[%d], result[%d]. \n", \
			operate_head1->operate.time, operate_head2->operate.time, operate_head3->operate.time);
	#endif
	return 0;
}


/**************执行操作*****************/
void mb_procework(void)
{
	operate_head_list* operate_head_p = NULL;
	operate_head_list* last_head_p = NULL;			//当前操作玻片的上个玻片用于free
	operate_head_list* free_head_p = NULL;
	static	operate_head_list* operate_head = NULL;
	operate_t* free_operate_p = NULL;
	unsigned char isall_reagent_same = 0;
	dispense_t dispense[11]; 
	dispense_t dispense_temp;
	int i = 0, j = 0, g = 0;
	struct timeval now;
	temper_control_t* temper_control = NULL;
	int is_alltemp_zero = 0;
	static bool is_VP1_0N = false;
	static unsigned int Discharge_counter = 0;
	static bool IsStartLock1 = true, IsStartLock2 = true, IsStartLock3 = true;//流程的第一次锁定
	bool start_Discharge_counter = false;
	operate_head_list*  operate_head_table[3] = {NULL};
	char tcpdata = 0;

	if (wkevent == MAINTAIN_WORK)
		return;

	memset(dispense, 0, sizeof(dispense_t) * 11);

	do_work_hydration();
	if (operate_head1->operate.reagent == STOP_OPERATE && operate_head2->operate.reagent == STOP_OPERATE
			&& operate_head3->operate.reagent == STOP_OPERATE)
	{
		//	GPIO_OutSet(VP1);	//空闲关闭	//不能在这里关闭，还有水合操作
		is_VP1_0N = false;		
	//	printf("Operate1reagent1[%d],Operate1reagent2[%d],Operate1reagent3[%d] .\n", operate_head1->operate.reagent, operate_head1->operate.reagent, operate_head3->operate.reagent);
	}

	gettimeofday(&now, NULL);
	pthread_mutex_lock(&head_lock);	  //轮询三组玻片架
	
	#if(CHANGE_SHELF_ORDER == 1)
	uint8_t shelf_num_workfinish[3] = {0};

		if(operate_head == NULL)
		{
			if(operate_pri != NULL)
				operate_head = operate_pri;
			else
				operate_head = operate_head1;

			memset(shelf_num_workfinish, 0, sizeof(shelf_num_workfinish));
			printf("shelf 3's operate_head=[%d].\n", (int)operate_pri);

		}else if (shelf_num_workfinish[0] == 1 && operate_head == operate_head1)		
		{	
			lprintf(log_my, INFO, "**************in operate_head1**************\n");
			operate_head = operate_head2;         //  Net to run step is from Shelf A to Shelf B ??????????? andry
			temper_control = (temper_control_t*)temper_control2;
		}
		else if (shelf_num_workfinish[1] == 1 && operate_head == operate_head2)
		{
			lprintf(log_my, INFO, "**************in operate_head2**************\n");
			operate_head = operate_head3;
			temper_control = (temper_control_t*)temper_control3;
		}
		else if (shelf_num_workfinish[2] == 1 && operate_head == operate_head3)
		{
			lprintf(log_my, INFO, "**************in operate_head3**************\n");
			operate_head = operate_head1;
			temper_control = (temper_control_t*)temper_control1;
		}
	#else
		if (operate_head == operate_head1)		
		{	
			lprintf(log_my, INFO, "**************in operate_head1**************\n");
			operate_head = operate_head2;         //  Net to run step is from Shelf A to Shelf B ??????????? andry
			temper_control = (temper_control_t*)temper_control2;
		}
		else if (operate_head == operate_head2)
		{
			lprintf(log_my, INFO, "**************in operate_head2**************\n");
			operate_head = operate_head3;
			temper_control = (temper_control_t*)temper_control3;
		}
		else if (operate_head == operate_head3 || operate_head == NULL)
		{
			lprintf(log_my, INFO, "**************in operate_head3**************\n");
			operate_head = operate_head1;
			temper_control = (temper_control_t*)temper_control1;
		}
	#endif

	if (pthread_mutex_unlock(&head_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_Lock");

	md_lockreagentshelf();  //强制停止检测试剂架锁定
		
	if (operate_head == operate_head1 && !flg_opwork1ready)  // last runing opeate_head3 and flg_opwork1ready is false to return this function
		return;
	if (operate_head == operate_head2 && !flg_opwork2ready)  // flg_opwork2ready == false 时，证明上位机重新发送规程来了，正在做规程时不处理，直接退出。
		return;
	if (operate_head == operate_head3 && !flg_opwork3ready)
		return;
	
	pthread_mutex_lock(&head_lock);

	ct_patchtime(operate_head);         //确保每次都执行

	printf("procework++.\n");

	if (pthread_mutex_unlock(&head_lock) != 0)
		lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_Lock");

	memset(operate_head_table, 0, sizeof(operate_head_table));
	pthread_mutex_lock(&head_lock);
	
	mb_opprocepriorit(operate_head_table);    //operate_head_table中为NULL不运行，有就运行

	if (pthread_mutex_unlock(&head_lock) != 0)
		lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head-Lock");
	for(i = 0; i < 3; i++)
	{
		if ((operate_head == operate_head_table[i]) && (operate_head != NULL))  // 此架玻片为下次操作的目标架次
			break;
	}
	if (i == 3)
		return;

	if (operate_head == operate_head1)
	{
		temper_control = (temper_control_t*)temper_control1;
		#if (USE_LOG_INFO == 1)
		{
			printf("work operate_head = operate_head1\n");
			lprintf(log_my, INFO, "work operate_head = operate_head1\n");
		}
		#endif
	}
	else if (operate_head == operate_head2)
	{
		temper_control = (temper_control_t*)temper_control2;
		#if (USE_LOG_INFO == 1)
		{
			printf("work operate_head = operate_head2\n");
			lprintf(log_my, INFO, "work operate_head = operate_head2\n");
		}
		#endif
	}
	else if (operate_head == operate_head3)
	{
		temper_control = temper_control3;
		#if (USE_LOG_INFO == 1)
		{
			printf("work operate_head = operate_head3\n");
			lprintf(log_my, INFO, "work operate_head = operate_head3\n");
		}
		#endif
	}

	if (md_lockreagentshelf() < 0)	//当主臂还没将操作完成必须锁住 
	{
		return;
	}
		
	mb_mixprogressintime(now.tv_sec - begin_time.tv_sec, operate_head);  //  计算搅拌时间

	if (operate_head == operate_head1 && !flg_opwork1ready)
		return;
	if (operate_head == operate_head2 && !flg_opwork2ready)
		return;
	if (operate_head == operate_head3 && !flg_opwork3ready)
		return;
	
	/*******************当3架都在加热时不允许其他动作continue**********************/
	if ((operate_head == operate_head1) && (flg_temperheating1))	//当一架玻片还在加热过程中读其他架
		return;
	if ((operate_head == operate_head2) && (flg_temperheating2))
		return;
	if ((operate_head == operate_head3) && (flg_temperheating3))
		return;
	
	if (start_Discharge_counter)
	{
		Discharge_counter++;
		if (Discharge_counter > 360000)
			Discharge_counter = 36000;
	}
	
	#if (USE_PRINT_LOG == 1)
	{
		{count = 0;
		lprintf(log_my, INFO, "in_temper_control=%d %d %d\n", flg_temperheating1,flg_temperheating2,flg_temperheating3);
		printf("operate_head = %d flg_temperheating1=%din_temper_control2=%din_temper_control3=%d\n",operate_head,flg_temperheating1,flg_temperheating2,flg_temperheating3);
		printf("operate.time =%d (now.tv_sec - begin_time.tv_sec) = %d\n", operate_head->operate.time, (now.tv_sec - begin_time.tv_sec));
		lprintf(log_my, INFO,"operate.time =%d all = %d\n", operate_head->operate.time, (now.tv_sec - begin_time.tv_sec));
		}
		count++;
		printf("flg_mianarmstop_a = %d flg_mianarmstop_b=%d flg_mianarmstop_c=%d\n", flg_mianarmstop_a, flg_mianarmstop_b,flg_mianarmstop_c);
	}
	#endif

	if (!flg_mianarmstop_a || !flg_mianarmstop_b || !flg_mianarmstop_c) //小仪器   3架玻片架有一架在工作则执行下面的语句 andry
		return;

	if ((operate_head->operate.reagent == STOP_OPERATE))//判断是否是玻片的最后一个步骤且全部完成
	{
		if (flg_opwork1ready && (operate_head == operate_head1) && !flg_temperheating1)
		{
			hydrateA.start_time = now.tv_sec;
			hydrateA.flage = TRUE;  //水合标志
			if (Discharge_counter > 600)
			{
				NeedDischarge = TRUE;
				start_Discharge_counter = TRUE;
			}
			while (!HEAD1_STEP_SENDED) usleep(100000);
				Discharge_counter = 0;
			IsStartLock1 = TRUE;
			
			pthread_mutex_lock(&head_step_lock);
			workstep_state_a = 0X01110000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			mb_mainworkstop(1);	//通知自控臂1动作结束

			#if(CHANGE_SHELF_ORDER == 1)
			 shelf_num_workfinish[0] = 1;
			#endif
		}
		if (flg_opwork2ready && (operate_head == operate_head2) && !flg_temperheating2)
		{	
			hydrateB.start_time = now.tv_sec;
			hydrateB.flage= TRUE; 
			if (Discharge_counter > 600)
			{
				NeedDischarge = TRUE;
				start_Discharge_counter = TRUE;
			}
			while (!HEAD2_STEP_SENDED) usleep(100000);
				Discharge_counter = 0;
			IsStartLock2 = TRUE;
			
			pthread_mutex_lock(&head_step_lock);
			workstep_state_b = 0X01110000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			mb_mainworkstop(2);
		
			#if(CHANGE_SHELF_ORDER == 1)
			 shelf_num_workfinish[1] = 1;
			#endif
		}
		if (flg_opwork3ready && (operate_head == operate_head3) && !flg_temperheating3)
		{	
			hydrateC.start_time = now.tv_sec;
			hydrateC.flage= TRUE; 
			if (Discharge_counter > 600)
			{
				NeedDischarge = TRUE;
				start_Discharge_counter = TRUE;
			}
			Discharge_counter = 0;
			while (!HEAD3_STEP_SENDED) usleep(100000);
			IsStartLock3 = TRUE;
			
			pthread_mutex_lock(&head_step_lock);
			workstep_state_c = 0X01110000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			mb_mainworkstop(3);

			#if(CHANGE_SHELF_ORDER == 1)
			 shelf_num_workfinish[2] = 1;
			#endif
		}
		
		return;
	}
	else if ((operate_head->operate.reagent == STOP_OPERATE) && (operate_head->next_head != NULL) 
				&& (operate_head->operate.time <= now.tv_sec - begin_time.tv_sec))//free余下表头
	{
		pthread_mutex_lock(&head_lock);
		while(operate_head->next_head != NULL)
		{
			last_head_p = operate_head;
			while(!flg_mainproexit)
			{
				free_head_p = last_head_p->next_head;
				if (free_head_p->next_head == NULL)
				{
					free(free_head_p);
					free_head_p = NULL;
					last_head_p->next_head = NULL;
					break;
				}

				last_head_p = last_head_p->next_head;
			}
		}
		memset(&(operate_head->operate), 0, sizeof(operate_head->operate));
		operate_head->operate_work_time = 0;
		operate_head->operate.reagent = STOP_OPERATE;
		if (pthread_mutex_unlock(&head_lock) != 0)
			lprintf(log_my, ERROR,"%s", "pthread_mutex_unlock error head_Lock");
		return;
	}

		/****************流程中有步骤*****************/
		/*	if (operate_head == operate_head1)
		{
			md_lockreagentshelf();
				if (IsStartLock1)
					IsStartLock1 = FALSE;
		}
		*/
	operate_head_p = operate_head;
	gettimeofday(&now, NULL);  //patchtime后必须在获取一次
	if (operate_head_p->operate.time <= (now.tv_sec - begin_time.tv_sec))
	{
		//	if (!ct_patchtime(operate_head))//	还没到运行时间
		//		return;	
			
		if (operate_head == operate_head1)
		{
			if ((operate_head->operate.reagent == REAGENT_DAB) && (isDAB_mixedA == FALSE))
				return;
			if ((operate_head->operate.reagent == REAGENT_SECEND) && (isDAB_mixedA_next == FALSE))
				return;
			while (!HEAD1_STEP_SENDED) usleep(100000);
			pthread_mutex_lock(&head_step_lock);
			workstep_state_a++;
			workstep_state_a &= 0X0000FFFF; 
			HEAD1_STEP_SENDED = FALSE;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			//	head1_start_time = now.tv_sec - begin_time.tv_sec;
			lprintf(log_my, INFO, "remove{} operater-head1 HEAD1_STEP_SENDED workstep_state_a[%d].\n", workstep_state_a);
		}
		if (operate_head == operate_head2)
		{
			if ((operate_head->operate.reagent == REAGENT_DAB) && (isDAB_mixedB == FALSE))
				return;
			if ((operate_head->operate.reagent == REAGENT_SECEND) && (isDAB_mixedB_next == FALSE))
				return;
			while (!HEAD2_STEP_SENDED) usleep(100000);
			
			pthread_mutex_lock(&head_step_lock);
			workstep_state_b++;
			workstep_state_b &= 0X0000FFFF; 	
			HEAD2_STEP_SENDED = FALSE;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			//	head2_start_time = now.tv_sec - begin_time.tv_sec;
			lprintf(log_my, INFO, "remove{} operater-head2 HEAD2_STEP_SENDED workstep_state_b[%d].\n", workstep_state_b);
		}
		if (operate_head == operate_head3)
		{
			if ((operate_head->operate.reagent == REAGENT_DAB) && (isDAB_mixedC == FALSE))
				return;
			if ((operate_head->operate.reagent == REAGENT_SECEND) && (isDAB_mixedC_next == FALSE))
				return;
			while (!HEAD3_STEP_SENDED) usleep(100000);
			pthread_mutex_lock(&head_step_lock);
			workstep_state_c++;
			workstep_state_c &= 0X0000FFFF; 
			HEAD3_STEP_SENDED = FALSE;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			//	head3_start_time = now.tv_sec - begin_time.tv_sec;

			lprintf(log_my, INFO, "remove{} operater-head3 HEAD3_STEP_SENDED workstep_state_c[%d].\n", workstep_state_c);
		}
		i = 0;
		memset(dispense, 0, sizeof(dispense));
		memset(temper_control, 0, sizeof(temper_control_t) * 10);

		printf("begin tooooo set\n");
		pthread_mutex_lock(&head_lock);
		while(!flg_mainproexit )	//轮询一架玻片
		{
			if (operate_head_p->operate.reagent == STOP_OPERATE)
			{
				operate_head_p = operate_head_p->next_head;
				if (operate_head_p == NULL)
					break;
				continue;
			}
			//将试剂信息和坐标信息记入dispense，
			//**********************************
		
			dispense[i].reagent = operate_head_p->operate.reagent;
			dispense[i].plate_num = operate_head_p->operate.plate_num;
			memcpy(&dispense[i].reagent_info, &operate_head_p->operate.reagent_info, sizeof(reagent_t));
			//将温度信息和坐标信息记入temper_control，
			temper_control[i].plate_num = operate_head_p->operate.plate_num;
			temper_control[i].time = operate_head_p->operate.next->time - operate_head_p->operate.time; //加热的时间
			temper_control[i].temp = operate_head_p->operate.temp;
			if (temper_control[i].temp == 0)
				temper_control[i].temp = TMP_ZERO;     //无需加热需要计算时间
			operate_head_p->operate_work_time = temper_control[i].time;
				
			free_operate_p = operate_head_p->operate.next;
			operate_head_p->operate = *(operate_head_p->operate.next);
			free(free_operate_p);
			free_operate_p = NULL;

			operate_head_p = operate_head_p->next_head;
			if (operate_head_p == NULL)
				break;
			i++;
		}	//一轮查询结束
		
		if (pthread_mutex_unlock(&head_lock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");
		//		reagent_flag = dispense[i].reagent;//test 步骤优化测试时使用

		if (operate_head == operate_head1 && workstep_state_a == 1)
		{
			tcpdata = 1;
			nt_sendpacketdata(WORK_START, &tcpdata,1);
		}
		else if (operate_head == operate_head2 && workstep_state_b == 1)
		{
			tcpdata = 2;
			nt_sendpacketdata(WORK_START, &tcpdata,1);
		}
		else if (operate_head == operate_head3 && workstep_state_c == 1)
		{
			tcpdata = 3;
			nt_sendpacketdata(WORK_START, &tcpdata,1);
		}
		#if(USE_LOG_INFO == 1)
		printf("一架查询结束 out\n");
		lprintf(log_my, INFO,"一架查询结束 out\n");
		printf("operate_head = %d flg_temperheating1=%din_temper_control2=%din_temper_control3=%d\n", (int)operate_head,\
					flg_temperheating1, flg_temperheating2, flg_temperheating3);
		printf("\n");
		#endif
		dispense[i + 1].reagent = STOP_OPERATE;	
		//排序
		printf("dispense[i + 1].reagent=%d i = %d\n", dispense[i + 1].reagent, i);
		isall_reagent_same = dispense[0].reagent;
		j=0;
		while(j < 11 && dispense[j].reagent != STOP_OPERATE)
		{
			//	lprintf(log_my, INFO," reagent=%d plate=%d", dispense[j].reagent,dispense[j].plate_num);
			printf(" reagent=%d plate=%d", dispense[j].reagent, dispense[j].plate_num);
			if (isall_reagent_same != dispense[j].reagent)
				isall_reagent_same = 0XFF;
			j++;
		}
		printf("\n");
		if (isall_reagent_same == 0XFF)
		{
			j = 0;
			while(j < 11 && dispense[j].reagent != STOP_OPERATE)
			{
				printf(" %d %d *", dispense[j].reagent, dispense[j].plate_num);
				if (isall_reagent_same != dispense[j].reagent)
					isall_reagent_same = 0XFF;
				j++;
			}
			printf("\n");
			printf("after\n");
			while(i > 0)
			{
				j = i - 1;
				while(j >= 0)
				{
					if (dispense[j].reagent == dispense[i].reagent)
					{
						if (i != j+1)//  相同试剂之间数据移动一位
						{
							dispense_temp = dispense[j];
							
							g = 0;
							do
							{
								dispense[j+g] = dispense[j+g+1];
								g++;
							}while( (j+g+1) <= (i - 1));

							dispense[i - 1] = dispense_temp;
							g = 0;
							while(g < 11 && dispense[g].reagent != STOP_OPERATE)
							{
								printf(" %d %d *", dispense[g].reagent, dispense[g].plate_num);
								g++;
							}
							printf("\n");
						}
						i--;
					}
					j--;
				}
				i--;
			}

			i = 0;
			while(i < 11 && dispense[i].reagent != STOP_OPERATE)
			{
				printf(" %d %d*", dispense[i].reagent,dispense[i].plate_num);
				i++;
			}
			printf("\n");
			printf("end\n");
		}
		/*
		{
			while(i > 0)
			{
				j = i - 1;
				while(j >= 0)
				{
					if (dispense[j].reagent == dispense[i].reagent)
					{
						dispense_temp = dispense[i - 1];
						dispense[i - 1] = dispense[j];
						dispense[j] = dispense_temp;
					}	
					j--;
				}
				i--;
			}
		}
		*/
		//试剂架试剂 DAB 修复液为关键试剂
		if (operate_head == operate_head1)
		{	
			if (last_reagentA < REAGENT_CASE || last_reagentA >= REAGENT_DAB
					|| last_reagentA == REAGENT_ER1 || last_reagentA == REAGENT_ER2 )
			{
				pthread_mutex_lock(&mutex_critialworklock);
				CriticalWork |= 0X80;
				if (pthread_mutex_unlock(&mutex_critialworklock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_critialworklock");
			}
			//mb_dischargeshelfwateliquid(1);
		}
		else if (operate_head == operate_head2)
		{		
			if (last_reagentB < REAGENT_CASE || last_reagentB >= REAGENT_DAB
				|| last_reagentB == REAGENT_ER1 || last_reagentB == REAGENT_ER2 )
			{
				pthread_mutex_lock(&mutex_critialworklock);
				CriticalWork |= 0X40;
				if (pthread_mutex_unlock(&mutex_critialworklock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_critialworklock");
			}
			//mb_dischargeshelfwateliquid(2);
		}
		else if (operate_head == operate_head3)
		{	
			if (last_reagentC < REAGENT_CASE || last_reagentC >= REAGENT_DAB
				|| last_reagentC == REAGENT_ER1 || last_reagentC == REAGENT_ER2)
			{
				pthread_mutex_lock(&mutex_critialworklock);
				CriticalWork |= 0X20;
				if (pthread_mutex_unlock(&mutex_critialworklock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_critialworklock");
			}
			//mb_dischargeshelfwateliquid(3);
		}
		printf("CriticalWork=%x last_reagentA=%d last_reagentB=%d last_reagentC=%d\n",\
				CriticalWork, last_reagentA, last_reagentB, last_reagentC);
	
		/************先滴加试剂再加热**************/
		//发送滴加试剂命令
			//	sleep(2);
		//	printf("\ntemper_control=%x\n",temper_control);
		printf("*************************do mb_aspiratedispensework******************\n");
		if (mb_aspiratedispensework(dispense) < 0)
			return;

		//temper_control 指针无故变动 改回来
		if (operate_head == operate_head1)
			temper_control = (temper_control_t*)temper_control1;
		else if (operate_head == operate_head2)
			temper_control = (temper_control_t*)temper_control2;
		else if (operate_head == operate_head3)
			temper_control = (temper_control_t*)temper_control3;
		
		//	sleep(5);
			//发送加热命令									
		for (i = 0; i < 10; i++)
		{
			//	printf("\ntemper_control=%x\n",temper_control);
			if (temper_control[i].temp > is_alltemp_zero)
				is_alltemp_zero = temper_control[i].temp;

			if (temper_control[i].temp >= 20 && temper_control[i].temp != TMP_ZERO)
			{
				if(new_temper)
					temper_control[i].temp = temper_control[i].temp*10 + temp_Dvalmy[temper_control[i].plate_num % 30][temper_control[i].temp] ;//加速温度差值	
				else
					temper_control[i].temp +=  temp_goalval[temper_control[i].temp] / 10;   //加速温度差值  
			}
		}
		
			//	if (is_alltemp_zero > 35)
		#ifdef EMCTST
		if(FALSE)
		#else
		if (TRUE)//将持续的时间计入加热为0的时间
		#endif
		{
			printf("*************************do TempContorl******************\n");
			if (temper_control == temper_control1 && wkeventA != STOP_WORK)
			{
				printf("in flg_temperheating1\n");
				lprintf(log_my, INFO, "in flg_temperheating1\n");
				flg_temperheating1 = TRUE;
				tc_packettempcmd(START_CONTROL1);		
				printf("temp.palte=");
				lprintf(log_my, INFO, "temp.palte=");
				for (i = 0; i < 10; i++)
				{
					printf(" %d ", temper_control[i].plate_num);
					lprintf(log_my, INFO, " %d ", temper_control[i].plate_num);
				}
			}
			else if (temper_control == temper_control2 && wkeventB != STOP_WORK)
			{
				printf("in flg_temperheating2\n");
				lprintf(log_my, INFO, "in flg_temperheating2\n");
				flg_temperheating2 = TRUE;
				tc_packettempcmd(START_CONTROL2);
				printf("temp.palte=");
				lprintf(log_my, INFO, "temp.palte=");
				for (i = 0; i < 10; i++)
				{
					printf(" %d ", temper_control[i].plate_num);
					lprintf(log_my, INFO, " %d ", temper_control[i].plate_num);
				}
			}
			else if (temper_control == temper_control3 && wkeventC != STOP_WORK)
			{
				printf("in flg_temperheating3\n");
				lprintf(log_my, INFO,"in in_temper_contro3\n");
				flg_temperheating3 = TRUE;
				tc_packettempcmd(START_CONTROL3);
				printf("temp.palte=");
				lprintf(log_my, INFO,"temp.palte=");
				for (i=0;i<10;i++)
				{
					printf(" %d ",temper_control[i].plate_num);
					lprintf(log_my, INFO," %d ",temper_control[i].plate_num);
				}
			}
		}
		else	//不需加热最高字节置1
		{
			if (temper_control == temper_control1)
			{
				pthread_mutex_lock(&head_step_lock);
				workstep_state_a = workstep_state_a | 0X01010000;
				if (pthread_mutex_unlock(&head_step_lock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			}
			else if (temper_control == temper_control2)
			{
				pthread_mutex_lock(&head_step_lock);
				workstep_state_b = workstep_state_b | 0X01010000;
				if (pthread_mutex_unlock(&head_step_lock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			}
			else if (temper_control == temper_control3)
			{
				pthread_mutex_lock(&head_step_lock);
				workstep_state_c = workstep_state_c | 0X01010000;
				if (pthread_mutex_unlock(&head_step_lock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			}
		}
	}		
}



char cal_XOR(const char* frame, char num)
{
	char buff = frame[0];
	uint8_t i = 0;

	for (i = 1; i < num; i++)
	{
		buff = (char)( buff ^ frame[i]);

	}
	return buff;
}


/******************************************************************************
*
* Function Name  : Tp_thread_recvserail
* Description	 : The string according to the RSP9000 OME communication protocol format function.
*					   
* Input 		 : None
* Output	 : None
* Return		 : None
*******************************************************************************/
void mb_thread_recvserail(void* arg)	//	不允许有等待操作
{
	int 	rel = 0, i = 0, len = 0;
	fd_set 	DeviceRead;   //设备读准备就绪;
	fd_set 	DeviceError; // 设备出现错误
	int    	DeviceMax = 0;
	unsigned short crc = 0; 
	bool VRC_flage = false;
	char check_c = 0;
	struct timeval timeout;
	
	memset(serail_recvbuf, 0x00, sizeof(serail_recvbuf));
	memset(WriteBuffer, 0x00, sizeof(WriteBuffer));
	
	// init serials
	#if(USE_LOG_INFO == 1)
	printf("==========[Tp_thread_recvserail]: Begin to listening all serial data.=======\n ");	
	#endif
	tcflush(serail_hand[COM2], TCIFLUSH);
	tcflush(serail_hand[port_arm], TCIFLUSH);
	tcflush(serail_hand[port_arm], TCIFLUSH);
	tcflush(serail_hand[port_pump], TCIFLUSH);
	tcflush(serail_hand[cabin_port], TCIFLUSH);

	while(!flg_mainproexit)
	{
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;
		
		FD_ZERO(&DeviceRead);
		FD_ZERO(&DeviceError);
		DeviceMax = serail_hand[COM2] > DeviceMax ? serail_hand[COM2] : DeviceMax;
		DeviceMax = serail_hand[port_arm] > DeviceMax ? serail_hand[port_arm] : DeviceMax;
		//DeviceMax = serail_hand[COM5] > DeviceMax ? serail_hand[COM5] : DeviceMax;
		DeviceMax = serail_hand[cabin_port] > DeviceMax ? serail_hand[cabin_port] : DeviceMax;
		DeviceMax = serail_hand[port_pump] > DeviceMax ? serail_hand[port_pump] : DeviceMax;
		FD_SET(serail_hand[COM2], &DeviceRead);
		//FD_SET(serail_hand[COM5], &DeviceRead);
		FD_SET(serail_hand[port_arm], &DeviceRead);
		FD_SET(serail_hand[port_arm], &DeviceRead);
		FD_SET(serail_hand[cabin_port], &DeviceRead);
		FD_SET(serail_hand[port_pump], &DeviceRead);
		//	FD_SET(serail_hand[COM5], &DeviceRead);

		rel = select(DeviceMax+1, &DeviceRead, NULL, NULL, &timeout);
		if(rel < 0) {
			perror("select error");
			return;
		}
		else
		{
			if(FD_ISSET(serail_hand[port_pump],&DeviceRead))
			{
				if ((len = read(serail_hand[port_pump], (char*)Readpump, sizeof(Readpump))) > 0)
				{
					for (i = 0; i < len; i++)
					{
						if(flg_startrecvpump)
							pump_readbuf[readindexpump] = Readpump[i];
						
						if (VRC_flage)//最后一个校验位
						{
							VRC_flage = false;
							check_c = cal_XOR((const char*)pump_readbuf, (char)readindexpump);
							if(check_c == pump_readbuf[readindexpump])
							{
								flg_recvpump = true;
							
								flg_startrecvpump = false;
							}			
							readindexpump = 0;
						}		
							
						if (Readpump[i] == 0X03 && flg_startrecvpump)
						{
							VRC_flage = true;		
						}

						if(++readindexpump > 20)
						{
							readindexpump = 0;
							memset((uint8_t*)pump_readbuf, 0, sizeof(pump_readbuf));
							VRC_flage = false;
							flg_startrecvpump = false;
							break;
						}
						if (Readpump[i] == 0X02)
						{
							flg_startrecvpump = true;
							readindexpump = 1;
							pump_readbuf[0] = Readpump[i];
							VRC_flage = false;
						}
					}
					if(i < len)
					{
						memset((uint8_t*)Readpump, 0, sizeof(Readpump));
					}		
				}
				memset((uint8_t*)Readpump, 0x00, sizeof(Readpump));	
			} 
			 
			if(FD_ISSET(serail_hand[COM2], &DeviceRead) && !big_version)
			{
				if ((len = read(serail_hand[COM2], serail_recvbuf, sizeof(serail_recvbuf))) > 0)
				{
					#if( USE_LOG_INFO == 1) //  #if (USE_PRINT_LOG == 1)
					{
						printf("COM2 Request read len = %d.: \n", len);
						lprintf(log_my, INFO,"COM2 read len = %d.: \n", len);
						for (i = 0; i < len; i++)
						{
							printf(" %02x ", serail_recvbuf[i]);
							lprintf(log_my, INFO," %02x ", serail_recvbuf[i]);
						}
						printf("\n\n");
					}
					#endif
					sm_parseminibdframe(COM2, len);
					
				}	
				memset(serail_recvbuf, 0x00, sizeof(serail_recvbuf));
				
			} 
		
			if(FD_ISSET(serail_hand[port_arm], &DeviceRead))
			{		
				if ((len = read(serail_hand[port_arm], serail_recvbuf, sizeof(serail_recvbuf))) > 0)
				{
					tcflush(serail_hand[port_arm],TCIFLUSH);

					#if( USE_LOG_INFO == 1) // 	#if (USE_PRINT_LOG == 1)
					{
						printf("port_arm Request read len = %d.: \n", len);
						lprintf(log_my, INFO, "port_arm = %d.: \n", len);
						for (i = 0; i < len; i++)
						{
							lprintf(log_my, INFO, " %02x ", serail_recvbuf[i]);
							printf(" %02x ", serail_recvbuf[i]);
						}
						printf("\n");
					} 
					#endif
				}
				sb_parseframe(RSP9000_FRAME, len);
				memset(serail_recvbuf, 0x00, sizeof(serail_recvbuf));	
			} 

			if(FD_ISSET(serail_hand[cabin_port], &DeviceRead) && !readfinished485)
			{				//防止两线程同时写操作	
				if ((len = read(serail_hand[cabin_port], serail_recvbuf, sizeof(serail_recvbuf))) > 0)
				{
					for (i = 0; i < len;i++)
					{
						if (serail_recvbuf[i] != 0xfe)	
						{
							if (readindex485 > 22)
							{
								memset((uint8_t*)ReadBuffer485, 0, sizeof(ReadBuffer485));
								readindex485 = 0;
							}
							ReadBuffer485[readindex485] = serail_recvbuf[i];
							readindex485++;				
						}
						else
						{
							if (readindex485 != 22 && readindex485 != 5)
							{
								ReadBuffer485[readindex485] = serail_recvbuf[i];
								readindex485++;
							}
							else
							{
								readindex485--;	
								crc = crc16_ccitt((const char*)&ReadBuffer485[1], readindex485 - 2);
								if (((crc & 0x00ff) != ReadBuffer485[readindex485]) || ((crc >> 8) != ReadBuffer485[readindex485 - 1]))
								{
									printf("cabin_port crc error\n");
									memset((uint8_t*)ReadBuffer485, 0, sizeof(ReadBuffer485));
									readindex485 = 0;
									tcflush(serail_hand[cabin_port],TCIFLUSH);	
								}
								else
								{
									pthread_mutex_lock(&mutex_cabinlock);
									
									memcpy((uint8_t*)ReadBufferweight, (const uint8_t*)&ReadBuffer485[1], readindex485);
									readindexweight = readindex485;
									readindex485 = 0;
									readfinished485 = true;
									pthread_mutex_unlock(&mutex_cabinlock);
									tcflush(serail_hand[cabin_port],TCIFLUSH);	
								}
							}
						}
					}     //  end of 	for (i = 0; i < len;i++)				
					memset(serail_recvbuf, 0x00, sizeof(serail_recvbuf));	
				}
			}
		}   //  end of if(rel < 0) {  } else
	}
	//	close(serail_hand[1]);
}



void MainArmTest(unsigned int Ary[],int x,int y,unsigned int z)
{
	sr_cmdstruct_t sendbuf;
 
	sendbuf.srdevaddr = ARM_ADDR;
	sprintf(sendbuf.cmdbuf, "PA %d %d %d", Ary[0] + x,Ary[1] + y,z);	//缓慢抽吸空气
	sm_serailsenddat(port_arm, &sendbuf, 1, 0, 1);
}






