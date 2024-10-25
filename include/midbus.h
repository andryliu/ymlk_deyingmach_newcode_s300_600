#ifndef _MIDBUS_H
#define _MIDBUS_H

#include <stdio.h>
#include <stdlib.h>
#include "log.h"
#include "port.h"
#include "arm_ctl_cabin.h"      /*    //   #define  LAIYUE 1  澳泉医疗公司跟我们跟莱悦公司的玻片拉伸搅拌是不一样的，我们需要搅拌，澳泉是不需要搅拌的  // */
#include "sttype.h"



//#define __DEBUG__
#define NORMAL_VER   1   //非配置小程序
#define NEED_INIT    1  //简易初始化


//#define BIG_VERSION 1	//大仪器标志
#define DEC_SEN 5000
#define CHECK_SEN 0x2500
//#define CHECK_SEN 0x3000

//#define EMCTST 1
#define LOCAL_PORT 1000


#define NEW_BARREL 1  //  是否都去文件
#define NEW_STRECH 0
#define DE_TONG 1    // 染色液2现配现用
//#define glint      // 结束后闪烁必须更新协处理器



// extern int new_version;	
extern int big_version,self_icovert,self_pump,new_mixstation,shelf_lock_new,new_outwast_sen;
#define SELF_ICOVERT self_icovert
#define SELF_PUMP self_pump
#define NEW_MIXSTATION new_mixstation
#define SHELF_LOCK_WAY_NEW shelf_lock_new
#define NEW_OUTWATST_SEN new_outwast_sen     //  废液桶在位检测传感器检测电平
#define NEW_INWATST_SEN new_inwast_sen      //  废液桶液位检测传感器检测电平

// #define new_version new_version
// #define new_scaner new_scaner
// #define NEW_TEMPER new_temper


//------ GPIO Defime----------------------
#define	GPIO0		(1 <<  0)
#define	GPIO1		(1 <<  1)
#define	GPIO2		(1 <<  2)
#define	GPIO3		(1 <<  3)
#define	GPIO4		(1 <<  4)
#define	GPIO5		(1 <<  5)
#define	GPIO6		(1 <<  6)
#define	GPIO7		(1 <<  7)
#define	GPIO8		(1 <<  8)
#define	GPIO9		(1 <<  9)
#define	GPIO10		(1 << 10)	// 电磁开关
#define	GPIO11		(1 << 11)
#define	GPIO12		(1 << 12)
#define	GPIO13		(1 << 13)
#define	GPIO14		(1 << 14)
#define	GPIO15		(1 << 15)
#define	GPIO16		(1 << 16)
#define	GPIO17		(1 << 17)
#define	GPIO18		(1 << 18)
#define	GPIO19		(1 << 19)
#define	GPIO20		(1 << 20)
#define	GPIO21		(1 << 21)
#define	GPIO22		(1 << 22)
#define	GPIO23		(1 << 23)
#define	GPIO24		(1 << 24)
#define	GPIO25		(1 << 25)
#define	GPIO26		(1 << 26)
#define	GPIO27		(1 << 27)
#define	GPIO28		(1 << 28)
#define	GPIO29		(1 << 29)
#define	GPIO30		(1 << 30)
#define	GPIO31		(1 << 31)
#define	GPIOX_FLAG	(1 << 31)
#define EM9280_DEV_MAJOR			251
#define EM9280_MAGIC								EM9280_DEV_MAJOR
#define EM9280_GPIO_IOCTL_OUT_ENABLE				_IOW(EM9280_MAGIC,  0x60, unsigned int)
#define EM9280_GPIO_IOCTL_OUT_DISABLE			_IOW(EM9280_MAGIC,  0x61, unsigned int)
#define EM9280_GPIO_IOCTL_OUT_SET					_IOW(EM9280_MAGIC,  0x62, unsigned int)
#define EM9280_GPIO_IOCTL_OUT_CLEAR				_IOW(EM9280_MAGIC,  0283, unsigned int)
#define EM9280_GPIO_IOCTL_OPEN_DRAIN				_IOW(EM9280_MAGIC,  0284, unsigned int)
#define EM9280_GPIO_IOCTL_PIN_STATE				_IOR(EM9280_MAGIC,  0285, unsigned int)

#define EM9280_PWM_START				10
#define EM9280_PWM_STOP					11
#define EM9280_COUNT_START				12
#define EM9280_COUNT_STOP				13

#define EM9280_PWM1_MINOR			8					//em9280_pwm1(->GPIO6 of EM9280)
#define EM9280_PWM2_MINOR			9					//em9280_pwm2(->GPIO7 of EM9280)
#define EM9280_PWM3_MINOR			10					//em9280_pwm3(->GPIO20 of EM9280)
#define EM9280_PWM4_MINOR			11					//em9280_pwm4(->GPIO21 of EM9280)

#define	PWM_POLARITY_NORMAL				(0 << 0)
#define PWM_POLARITY_INVERTED			(1 << 0)
#define POLARITY			PWM_POLARITY_INVERTED;

#define  ORG_POSIN_X  800

#ifdef BIG_VERSION

	#define	V1 GPIO2
	#define	V2 GPIO18
	#define	V3 GPIO3
	#define	V4 GPIO19
	#define	V8 GPIO20
	#define	V9 GPIO21
	#define	V10 GPIO6

	#define	V5 GPIO23
	#define	V6 GPIO24
	#define	V7 GPIO25
	#define VP1 GPIO26	//压缩机
	#define	BEEPER GPIO27

	#define RS485RW GPIO4

	#define	CG1 GPIO7	
	#define	CG2 GPIO6	
	#define	CG3 GPIO5	
	#define	CG4 GPIO4	

	#define	CGLOCK1 GPIO0		
	#define	CGLOCK2 GPIO16	
	#define	CGLOCK3 GPIO1
	#define	CGLOCK4 GPIO17



	#define In_W GPIO12
	#define In_H GPIO28
	#define In_L GPIO13
	#define Out_H GPIO29
	#define Out_L  GPIO14


	#define TEM_KEYC GPIO7
	#define TEM_KEYB GPIO8
	#define TEM_KEYA  GPIO9

	#define POWER_SEN GPIO5
	#define DOOR GPIO15
	#define ER_LED GPIO30
	#define SLEEP_LED GPIO31
#else

	#define	V1 GPIO24
	#define	V2 GPIO23
	#define	V3 GPIO22
	#define	V4 GPIO20
	#define	V5 GPIO27
	#define	V6 GPIO26
	#define	V7 GPIO25
	#define	V8 GPIO16
	#define	V9 GPIO17
	#define RS485RW GPIO4
	#define VP1 GPIO18	//压缩机


	#define	CG1 GPIO7	
	#define	CG2 GPIO6	
	#define	CG3 GPIO5	
	#define	CG4 GPIO4	

	#define	CGLOCK1 GPIO12		
	#define	CGLOCK2 GPIO15	
	#define	CGLOCK3 GPIO14
	#define	CGLOCK4 GPIO13

	#define In_H GPIO31
	#define In_L GPIO30
	#define Out_H GPIO29
	#define Out_L  GPIO28


	#define TEM_KEYC GPIO9
	#define TEM_KEYB GPIO3
	#define TEM_KEYA GPIO2

	#define POWER_SEN GPIO8
	#define DOOR GPIO0
	#define	BEEPER GPIO21
	#define ER_LED GPIO11

#endif





#ifdef BIG_VERSION
	#define TEM_PORT COM9

#else
	#define TEM_PORT COM5
#endif


#define	EM9280_GPIO_OUTPUT_ENABLE		0
#define	EM9280_GPIO_OUTPUT_DISABLE		1
#define	EM9280_GPIO_OUTPUT_SET			2
#define	EM9280_GPIO_OUTPUT_CLEAR		3
#define	EM9280_GPIO_INPUT_STATE			5


#define DischargeTime 10

extern volatile int  new_scaner, new_temper;
#define TULUN_MISTAKE 8 //  凸轮旋转误差

extern int MIX_WASH_DISTANCE;
extern volatile  unsigned short bigcabin_value[];

extern int OFFSET_STEP,MOV_ZH,ZSPEED_SLOW,ZWASH_STEP1,ZWASH_STEP2_1,ZWASH_STEP2_2,ZWASH_STEP2_ZX;
extern int ZWASH_STEP3,ZWASH_STEP4,ZWASH_STEP4_SLOW,ZDAB_STEP,LIQUID_ZMAX,DAB_ZMAX,POUR_LIQUID_ZMAX;
extern int OPEN_ARMZMAX,SCAN_DISTANC_MIXSTATION;
extern int PULL_DISTANC_INIT  ,PULL_DISTANC_LT,PULL_DISTANC_HALF,PULL_DISTANC_FULL;

#define ZWASH_STEP2_WASTE 20
#define DISPENSE_INTERVAL_LT 10 //加载点与150ulZ轴下降高度差
#define DISPENSE_INTERVAL_HALF 15 //加载点与拉伸一半Z轴下降高度差
#define DISPENSE_INTERVAL_ALL 0 //加载点与拉伸全部Z轴下降高度差

int PORT_OFFSET,PORT_OFFSET_POOL,SYSPEED_TST, SYSPEED, SYSPEED_CLEAN, SYSPEED_AIR, SYSPEED_WASTE, SYSPEED_ALCHOLE, SYSPEED_LIQUID, SYSPEED_LIQUIDEBER;
int SYSPEED_DISPENSE,SYSPEED_DISPENSEFUL,SYSPEED_DISPENSE_SHELF, START_DISPENSE,SYSCUTOFF_SPEED,SYSSTARTSPD,SYSPEED_MIXDAB, SYDELAY, SYFSTEP, SY45STEP, SYHSTEP, SYMHSTEP, SYLIQTSTSTEP;
int SYHHSTEP, SY15STEP, SY110STEP, SYSTEP_AIR, SYSTEP_AIR_LAG, SYSTEP_WASTE, SYSTEP_LIQTSTAIR ;
int SYSTEP_LIQUID, SYSTEP_LIQUID_REMAIN;

#define SYSTEP_MULDAB 20	//稀释DAB的倍数

#define SYSTEP_MULGREEN 1	//稀释DAB的倍数
// #define SYSTEP_MOR 103 / 100	//吸试剂多吸的倍数
#define SYSTEP_DABMOR 103/100
#define SYSTEP_DISPENSMOR 102/100

//#define MIX_SYSTEP_MOR 105 / 100

#define MIX_SYSTEP_MOR 120 / 100	//吸试剂多吸的倍数
#define SYSTEP_DAB_DISPENS_PEC (9 / 10)

#define ER2PORT 3 -PORT_OFFSET		//小仪器使用
#define ER1PORT 4 -PORT_OFFSET	//小仪器使用
#define DEWAXPORT 5	-PORT_OFFSET//小仪器使用
#define ALCOHOLPORT 6 -PORT_OFFSET	//小仪器使用
#define WATERPORT 8	-PORT_OFFSET
#define WASHPORT 7 - PORT_OFFSET
#define PROBEPORT 2 - PORT_OFFSET
#define POOLPORT 1 + PORT_OFFSET_POOL

#define TECAN_OEM_BUFFER_SIZE 2048
#define LIQUID_STEP_MIX_STATION 1//混合站 吸一张玻片液下降的高度 (分配量为100ul)
#define LIQUID_STEP liquid_step_glb//普通试剂瓶 吸一张玻片液下降的高度 (分配量为100ul) 0.05mm/perstep
//#define LIQUID_STEP open_armstep//TEST
#define LIQUID_STEPDAB open_armstep2//DAB RED试剂瓶 吸一张玻片液下降的高度 (分配量为100ul) 7ml
#define LIQUID_SPEED 2500	//针下降速度
#define LIQUID_MSTEP 5	//针下降以后再下降的步数确保吸液体
#define LIQUID_ZS_MOR 100
#define LIQUID_ER_STEP 50
#define LIQUID_MSTEP_DAB 3
#define LIQUID_DISTANCE_ZMAX 0	

#define POUR_LIQUID_DISTANCE_ZMAX 0

#define DISTANCE_SCAN 65

#define SLIDE_DISTANCE 231
#define REAGENT_DISTANCE 231
#define WASH_DISTANCE 36

#define MIX_DISTANCE 67
#define MIX_WASH_Y 28

#define Z_OFFSET 0

#define min(a,b) ((a < b) ? a :b)



//全局变量
/*
int HS0;            		//Com1 主机械臂控制板
int HS1;				//Com2 注射器控制板
int HS2;				//Com3 加热板温控保护控制板
int HS3;				//Com4 扫描头
int HS4;				//Com5 玻璃片装载控制板
int HS5;          		//Com6 温控器模块
*/

extern volatile unsigned short lightdata;
extern unsigned short lightcleandata;
extern int new_versio;

extern volatile bool IsWashProbeStart;

unsigned char WriteBuffer[100]; //Com Device Write Buffer
unsigned char DataBuffer[100];  // 经过协议转换后的数据
unsigned char COM9Buffer[100];
extern char flg_cabinhavereagent[6];
extern int remote_fd,local_fd;
extern volatile bool flg_netdisconnect;

extern volatile unsigned int workstep_state_a;
extern volatile unsigned int workstep_state_b;
extern volatile unsigned int workstep_state_c;
extern unsigned int Array_mixed_DAB1[];
extern unsigned int Array_mixed_DAB2[];
extern unsigned int Array_mixed_DAB3[];
extern unsigned int ordArray_mixed_DAB[][2];
extern volatile char last_cabin_reagent ;
extern pthread_mutex_t head_lock;
extern pthread_mutex_t report_lock;
extern pthread_mutex_t netsend_lock;
extern pthread_mutex_t mutex_mianarmlock;
extern pthread_mutex_t pump_lock;
extern pthread_mutex_t head_step_lock;
extern pthread_mutex_t mutex;
extern pthread_cond_t cond ;
extern pthread_mutex_t armlock_ack;
extern pthread_mutex_t armlock_answer;
extern volatile bool flg_mainproexit;
extern volatile bool stop_flag;
extern volatile stbeep_state beep_state;
extern volatile bool flg_maintianrunning;
extern volatile bool OUT_WASTHIGH_H; //外部废液桶
extern volatile bool OUT_WASTHIGH_L;

extern volatile hydrate_t hydrateA;
extern volatile hydrate_t hydrateB;
extern volatile hydrate_t hydrateC;
extern ewkevent wkeventA;	//A架玻片工作状态
extern ewkevent wkeventB;
extern ewkevent wkeventC;
extern volatile bool flg_opwork1ready;
extern volatile bool flg_opwork2ready;
extern volatile bool flg_opwork3ready;
extern unsigned int Array_mixed_DAB1[];
extern unsigned int Array_mixed_DAB2[];
extern unsigned int Array_mixed_DAB3[];
extern unsigned int Array_mixed_RED1[];
extern unsigned int Array_mixed_RED2[];
extern unsigned int Array_mixed_RED3[];
extern mixstation_clear_state_t mixstation_clear_state[];
extern volatile stshelfstaut reagent_check[];
extern volatile char reagent_lock[][3];

extern volatile  short cabin_value[];
extern volatile unsigned short need_perfusion[];
extern parameter_t par;
extern volatile int tem_limit;


extern volatile unsigned char work_cnt;
extern volatile bool StartDispenseA , StartDispenseB , StartDispenseC ;
extern volatile	bool NEED_PRINTF ;

extern mix_t mix_DAB[];//每架混合液信息  按玻片位置填入
extern mix_t mix_SECEND[];
extern volatile char SHELF_LOCK_STATE[];

extern volatile bool MixACK , Mix_Res;
extern int serail_hand[];//串口句柄数据
extern int conf_zdab_addstep;

//------ Defime----------------------
#define FRAME_IS(type, frame) ((frame->control & type) == type)



/* 
*Config the GPIO input/output function. 
* * PARAMS: 
* dwEnBits - config the gpios frome 0 to 30 as input or output, one gpio statu is the Hex variables one bit 
*  
* RETURNS: 
*  The function excutes wether or not
*/


int md_gpiointion(void);
int GPIO_OutEnable(unsigned int dwEnBits);
int GPIO_OutDisable(unsigned int dwDisBits);
int GPIO_OpenDrainEnable(unsigned int dwODBits);
int GPIO_OutSet(unsigned int dwSetBits);
int GPIO_OutClear(unsigned int dwClearBits);
int GPIO_PinState(unsigned int* pPinState);

// int sb_armpumpsend(emb_comid comPort, unsigned char *SndCmdStr, unsigned char srdevaddr, unsigned char seq, unsigned char isReaptSnd, unsigned char ConnectMode);
bool mb_waterwashreagentpour(unsigned char reagent_port);
bool mb_muiltreagentpour(unsigned char reagent_port);
void mb_thread_recvserail(void* arg);
void mb_thread_recvscanner(void *arg);
int mb_setdefaulconfpara(const char * cmp_str);
void mb_mixprogressintime(unsigned int work_time, operate_head_list* next_head);
void mb_updateconfilefrornet(void);
int mb_shelfaspirate(const unsigned char reagent_num, const unsigned char same_num, const unsigned int liquid_val,\
	const unsigned char shelf_num);

int mb_dispensingliquid(unsigned char plate_num,unsigned char liquid_val,unsigned char  same_num,char reagent_num);
int mb_searchcomatiblereagent(reagent_t * reagent, int val);
int mb_searchcomatiblereagentval(char shelf_num,unsigned char *ordArray_reagent,reagent_t *reagent,int reagent_val);

int mb_aspiratedispensework(dispense_t* dispense);
void mb_procework(void);
void mb_printfoperatelist(operate_head_list* operate_head);
int mb_findoperatedabkind(operate_head_list* operate_head,char kind);
int mb_probectrl(void);
int mb_clearmixstation(unsigned int Mixing_Sation[]);
int mb_dabmixer(const operate_head_list* operate_head);
int mb_procwashprobe(unsigned char reagent_num);
void mb_readconfparatinfo(char IsDef);

void mb_errorinfoproce(emERRORCODE *error_code, int error_num);
void mb_setoperateheaddisable(operate_head_list** operate_head, bool IsHead);
void mb_reportshelfstatus(void);
bool mb_finderrorcodeinlist(int error_code, bool ISDELETE);
void mb_setreportval(stnet_report report);
stnet_report mb_readreportval(void);
int mb_geterrorcode(void);
void mb_seterrorcode(int  error_code);
void mb_checkshelfreagent(unsigned int reagent_num,unsigned int ordArray_plate[][2],unsigned char *check_reagent);
void mb_cancelreagentwork(unsigned char reagent_num);
void mb_shelfscanning(unsigned int shelf_num);// ,unsigned char* mini_recieve_code);
bool mb_checksensorproce(void);
char mb_getminnumber(unsigned int a,unsigned int b, unsigned int c);
void mb_lockallregent(uint8_t shelf_num);

void mb_mainworkstop(unsigned char shelf_num);
int mb_beeperproce(void);
void mb_getreagentliquitcab(emb_comid comPort);
int mb_waterctrl(emb_comid comPort, unsigned char cmd);

bool mb_dischargwateliquid_lo(char tim);
bool mb_dischwasteliquid_hi(char tim);
//将水合坐标填入
void mb_sethydratepale(unsigned char shelfnum);

void mb_menualwork(const char * cmp_string);
int mb_updatefirmware(char*data, int part_len);
int mb_updatemeasconfig(char*data, int part_len);
int mb_getmeasconfig();
int mb_setmeasconfig();
void mb_reagentchecking(unsigned char plate_num);
void mb_mixstationscanner(void);
int mb_checkcabinhavereagt(unsigned char perfusion_num);
void mb_shelfnumdispenseval(dispense_t* dispense, unsigned int liquid_val);
void mb_reagentstationscanner(unsigned int reagent_num);
void mb_readrunparainfo(void);
bool mb_dischargeshelfwateliquid(unsigned char shelf);
int mb_islockedreagent(uint8_t reagent_num);
void mb_probewash_c(unsigned char liquid_port);
void mb_probewash_b(unsigned char liquid_port);
void mb_probewash_a(unsigned char liquid_port);		//清洗并灌注大容量试剂 水和缓冲液 其他不能用A
void mb_probewash_d(unsigned char liquid_port);
int mb_monitdoorstate(void);
void mb_setstatetoshelf(char num, bool shelfstate);
bool mb_getstatefromeshelf(char num);


#endif
