/* -------------------------------------------------------------------------
 * tpool.h - htun thread pool defs
 * -------------------------------------------------------------------------
 */

#ifndef __TP_THREAD_H__
#define __TP_THREAD_H__

#include  <stdio.h>
#include  <pthread.h>
#include "port.h"
#include "midbus.h"
#include "serailminibd.h"



#define MAX_FILE_SIZE 1024*50
#define PUMPA_ADDR 0X34		//PUMP SWITCH + 0X31
#define PUMPB_ADDR	0X33
#define PUMPC_ADDR	0X32

/***********水箱控制宏定义*************/
#define WATER_ADDR 1
#define WATER_WORK_START 1
#define WATER_WORK_STATE 2
#define WATER_WORK_STOP 3
#define WATER_V_WORK 4
#define WATER_V_STOP 5
#define WATER_PUMP_WORK 6
#define WATER_PUMP_STOP 7
#define WATER_ACK 9
#define WATER_WORK_NORMAL 0

#define WATER_AVALIABLE 0X0A
#define WATER_ERROR 0X0B
#define TMP_ZERO 0XFFFFFFFF
/*
#define SENDED_TEMPERCMD 0XFF
#define REACH_TEMP 0XEE
#define START_TIME 0XDD
#define START_FAN 0XCC
#define WAIT_STRECH 0XBB
#define IN_85 0XAA
#define IN_85_TIME 0X99
#define SEND_TEMP 0X88
*/


	
pthread_t thread_id[16];
int thread_num;

extern volatile int env_temp;
extern volatile bool initialize_finished;
extern pthread_mutex_t mutex_dischargelock;
extern pthread_mutex_t mutex_mlock;
extern pthread_mutex_t mutex_mneedworked;
extern pthread_mutex_t mutex_mixlock;
extern pthread_mutex_t mutex_cabinlock;

extern pthread_mutex_t mutex_mlock2;
extern pthread_mutex_t mutex_mlock3;
extern pthread_mutex_t mutex_tempotherlock;
extern pthread_mutex_t mutex_washworklock;
extern pthread_mutex_t mutex_tryworklock;//TRY_REAGENT_UNLOCK全局量锁
extern pthread_mutex_t mutex_critialworklock;//CriticalWork全局量锁
extern pthread_mutex_t mutex_miantianlock;//温控器调试命令锁
extern pthread_mutex_t mutex_modbuslock;//modbus协议锁
extern pthread_mutex_t	mutex_mianarmlock;//主臂动作锁
extern int thread_id_miniboard;
extern int thread_id_temper;
extern volatile char shelf_statue[];
extern volatile bool flg_fanworked[];
extern volatile bool mini_work_finished[];
extern volatile unsigned char  mini_recieve_code_all[];

extern volatile stfanstruct fan[3];
extern volatile bool flg_normaldispenseliquid[3];
extern volatile bool needinitialize;
extern volatile bool flg_dischargeremainliquid[];

extern volatile char isWashWork;

extern volatile bool TEM_init_setting_finished;
extern volatile bool flg_intosleep;
extern volatile bool flg_shelfarmsleep[];

extern volatile stminibd_cmdlist* pcmd_head;
extern volatile stminibd_cmdlist* pcmd_head2;
extern volatile stminibd_cmdlist* pcmd_head3;

extern  temper_control_t temper_control1[];
extern  temper_control_t temper_control2[];
extern  temper_control_t temper_control3[];
extern volatile dispense_t dispenseA[]; //读取A架流程时取出滴加A架玻片操作信息
extern volatile dispense_t dispenseB[]; 
extern volatile dispense_t dispenseC[]; 

int suspend( pthread_t thread ) ; 
int resume( pthread_t thread );  

/*
 * returns a newly created thread pool   初始化连接池
 */
extern stthreadpl *tp_threadini(int num_worker_threads,
    int max_queue_size, int do_not_block_when_full);

/*
 * returns -1 if work queue is busy
 * otherwise places it on queue for processing, returning 0
 *
 * the routine is a func. ptr to the routine which will handle the
 * work, arg is the arguments to that same routine
    routine 为函数指针  用于处理具体业务的函数实现  arg为参数
 
 */
int tp_addthreadwk(stthreadpl *pool, void (*routine)(void *), void *arg);

/*
 * cleanup and close,
 * if finish is set the any working threads will be allowd to finish
 */
extern int tp_threaddestry(stthreadpl *pool, int finish);

/* private */
int tp_getrpara(void);


/*extern void tpool_thread(stthreadpl *pool); */
void tp_thread_probwashclear(void *arg);

int tp_devinit(bool SHORT);
void tp_thread_mainarm(void *arg);
void tp_cabinlat(void *arg);
void tp_cabina(void *arg);
void tp_cabinb(void *arg);
void tp_cabinc(void *arg);
void tp_thread_can(void *arg);


// void tp_thread_miniarm(void *arg);
// void tp_mbadctrtwo(void *arg);
// void tp_mbadctrtre(void *arg);
void tp_thread_beep(void *arg);
void tp_thread_fanctrl(void *arg);

/* the worker thread */
void *tpool_thread(void *tpool);
void tp_thread_tempernew(void *arg);
void tp_thread_temper(void *arg);
void tp_thread_otherwork(void *arg);
void tp_thread_monitsersor(void *arg);

bool tp_temperinit(void);
void mb_readconfparatinfo(char IsDef);
void tp_washchange(bool add);

void tp_mainarmbusses(bool mainarm_finished, dispense_t* dispenseIN, bool mini_work_finished,const unsigned int liquid_val, \
        unsigned char* last_reagent, uint8_t shelfindex);




#endif /* _TPOOL_H_ */

