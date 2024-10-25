

/* -------------------------------------------------------------------------
 * tpool.c - htun thread pool functions
 * -------------------------------------------------------------------------
 */
#include  <stdio.h>
#include  <stdlib.h>
#include  <string.h> /* strerror() */
#include  <pthread.h>
#include  <unistd.h>  
#include  <signal.h> 
#include  <assert.h>
#include  <errno.h>
#include  <sys/time.h>
#include <termios.h>
#include "serailbussfile.h"
#include "tpthread.h"
#include "netbuss.h"
#include "log.h"
#include "common.h"
#include "midbus.h"
#include "readinifile.h"
#include "buss.h"
#include "serailminibd.h"
#include "temper_control.h"
#include "temperctlmy.h"
#include "sttype.h"
#include "temper_control.h"
#include "mb.h"
#include "mbport.h"
#include "mbrtu.h"
#include "arm_ctl_cabin.h"
#include "can.h"
#include "midbus.h"
#include "mbcrc.h"
#include "cantool.h"
#include "xmodem.h"
#include "termperbusses.h"
#include "scaner.h"




#define BUF_SIZE 			50
#define	InitMaxStrNum		130	 
#define SIGRESUME SIGUSR2  
#define SIGSUSPEND SIGUSR1  


int thread_id_miniboard = -1;
int thread_id_temper = -1;

static unsigned char pumpNum = 0;
static unsigned char pumpConnectMode = 2;
static unsigned char armNum = 0;
//LinkList cmdstack;

extern volatile unsigned char sr_armansercode;
extern volatile unsigned char flg_armack;
extern int  new_versio;

extern pthread_cond_t cond;
extern pthread_mutex_t mutex;
extern pthread_mutex_t error_lock;



stTemper_cmdframeunion tem_cmd_test;  //用于调试
pthread_mutex_t mutex_mlock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_mneedworked = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_armcabinlock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_canlock = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t mutex_mlock2 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_mlock3 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_temlock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_miantianlock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_modbuslock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_tempotherlock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_washworklock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_dischargelock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_mianarmlock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_critialworklock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_tryworklock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_mixlock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_cabinlock = PTHREAD_MUTEX_INITIALIZER;


extern struct timeval begin_time;
extern char reagent_clr[];

extern unsigned int load_motor[];
extern unsigned int arm_motor[];
extern unsigned int load_motor2[];
extern unsigned int arm_motor2[];
extern unsigned int load_motor3[];
extern unsigned int arm_motor3[];
extern unsigned int ordArray_wash[][2];
extern unsigned int ordArray_mixed_DAB[][2];
extern unsigned int ordArray_reagent[][2];
extern unsigned int ordArray_plate1[][2];
extern unsigned int ordArray_plate2[][2];
extern unsigned int ordArray_plate3[][2];
extern unsigned int temp_sensor_type[];	//22DPt100L
//extern unsigned int temp_SV_low[];		//350 35°
extern unsigned int temp_SV_High[];		//1000 100°
extern unsigned int temp_MV[];		//700 70%
extern unsigned int temp_Input_Bias[];	
extern unsigned int temp_Input_Filter[];
extern unsigned int temp_HeatingPro_Band[];	//10
extern unsigned int temp_HeatingInt_Time[];	//30
extern unsigned int temp_HeatingDer_Time[];		//5
extern unsigned int temp_Heatingctl_Time[];	// 20

extern stTemper_cmdframeunion* tem_cmd_p[];

/******************************************/
 extern  operate_head_list* operate_head1;
 extern  operate_head_list* operate_head2;
 extern  operate_head_list* operate_head3;
extern volatile int honey_scaner;



volatile dispense_t dispenseA[11] ; 
volatile dispense_t dispenseB[11] ; 
volatile dispense_t dispenseC[11] ; 
// ArmCabin_t DoArmCabin[3] = {{0}};


volatile char shelf_statue[3] = {0};

volatile bool flg_shelfcanindex[3] = {false};
volatile bool flg_fanworked[3] = {false};
volatile bool mini_work_finished[3] = {true};
volatile bool flg_checkreagentindex[36] = {false};
volatile stfanstruct fan[3];
volatile char shelf_stirmode[3] = {0};         // 盖片电机抽拉位置
volatile unsigned int shelf_stirtime[3][3] = {{0}};    // 间隔时间做盖片抽拉


volatile bool flg_normaldispenseliquid[3] = {false};
volatile bool flg_dischargeremainliquid[3] = {false};

volatile bool flg_sancmixstatioin = false;

volatile bool flg_shelfarmsleep[4] = {true,true,true,true};//0，机械臂，1，2，3，三个玻片架
volatile bool flg_intosleep = false;
volatile bool flg_sleepfordevinit = false;
volatile bool flg_washproblewater = false;


volatile unsigned char   mini_recieve_codeACK = 0;	//minib接收ACK
volatile unsigned char   mini_recieve_code_all[3] = {0};	//minib接收answer

volatile bool In_bs_minishelfwork = false;
volatile bool mini_finished=true;
volatile bool mini_finished2=true;
volatile bool mini_finished3=true;
volatile bool flg_mianarmstop_a = true;	//完成时为TRUE //需要动作时置FALSE
volatile bool flg_mianarmstop_b = true;
volatile bool flg_mianarmstop_c = true;
extern unsigned int liquid_valA; //试剂的分配量 (100 ，150)
extern unsigned int liquid_valB;
extern unsigned int liquid_valC;
extern volatile unsigned char last_reagentA;
extern volatile unsigned char last_reagentB;
extern volatile unsigned char last_reagentC;
extern unsigned char workstep_mix_a;
extern unsigned char workstep_mix_b;
extern unsigned char workstep_mix_c;
extern volatile bool isDAB_mixedA;
extern volatile bool isDAB_mixedB;
extern volatile bool isDAB_mixedC;
extern volatile unsigned char startmixworkid;
extern volatile unsigned char CriticalWork;
extern bool CG1_SCAN;
extern bool CG2_SCAN;
extern bool CG3_SCAN;
extern bool CG4_SCAN;
volatile extern ewkevent wkevent;


extern volatile bool inDischarge;
extern volatile bool TRY_REAGENT_UNLOCK1;
extern volatile bool TRY_REAGENT_UNLOCK2;
extern volatile bool TRY_REAGENT_UNLOCK3;
extern volatile bool TRY_REAGENT_UNLOCK4;
extern volatile unsigned char reagent_lock_num1; //记录 多少加玻片要锁定
extern volatile unsigned char reagent_lock_num2;
extern volatile unsigned char reagent_lock_num3;
extern volatile unsigned char reagent_lock_num4;
volatile bool NeedDischarge;
volatile bool TEM_init_setting_finished = false;
volatile bool TEM_init_work_finished = false;

extern bool IN_CMP_WORK;
volatile char SelfCheckEvent = 0; //自检中出现一个问题加1 解决一个问题减1  0 为正常	
volatile char isWashWork = 0;			//清洗吸液请求  没请求一次加1 停止请求一次减1  (大于0即需要吸液体)
volatile char isHighConcentration = 0;			//排高浓度请求  没请求一次加1 停止请求一次减1  (大于0即需要排高浓度)
//extern unsigned char ReadBuffer485[];
extern unsigned char readindex485;
extern sterrcode_list* perror_exist_head;
/* Thread specied var, the status*/  
static __thread int g_bSuspend;  


const cmdStr_t srailcmdstringarray[InitMaxStrNum] = {
	{"OM", 0x38}, {"OX", 0x38}, {"OY", 0x38}, {"OZ", 0x38}, {"FX", 0x38}, {"FY", 0x38}, {"FZ", 0x38}, 
	{"SA", 0x38}, {"SP", 0x38}, {"SS", 0x38}, {"SZ", 0x38}, {"ST", 0x38}, {"SD", 0x38}, {"PA", 0x38},
	{"XI", 0x38}, {"YI", 0x38}, {"ZI", 0x38}, {"XA", 0x38}, {"YA", 0x38}, {"ZA", 0x38}, {"XS", 0x38}, 
	{"YS", 0x38}, {"ZS", 0x38}, {"SF", 0x38}, {"SR", 0x38}, {"SW", 0x38}, {"SC", 0x38}, {"E%", 0x38},
	{"PF", 0x38}, {"ZD", 0x38}, {"ZM", 0x38}, {"ZT", 0x38}, {"ZX", 0x38}, {"ZZ", 0x38}, {"PR", 0x38},
	{"PW", 0x38}, {"PC", 0x38}, {"RV", 0x38}, {"PI", 0x38}, {"N",  PUMP_ADDR}, {"K",  PUMP_ADDR}, {"k",  PUMP_ADDR}, 
	{"Z",  PUMP_ADDR}, {"Y",  PUMP_ADDR}, {"W",  PUMP_ADDR}, {"w",  PUMP_ADDR}, {"z",  PUMP_ADDR}, {"I",  PUMP_ADDR}, {"O",  PUMP_ADDR}, 
	{"E",  PUMP_ADDR}, {"B",  PUMP_ADDR}, {"A",  PUMP_ADDR}, {"P",  PUMP_ADDR}, {"D",  PUMP_ADDR}, {"L",  PUMP_ADDR}, {"v",  PUMP_ADDR}, 
	{"V",  PUMP_ADDR}, {"S",  PUMP_ADDR}, {"c",  PUMP_ADDR}, {"Q",  PUMP_ADDR}, {">",  PUMP_ADDR}, {"R",  PUMP_ADDR}, {"X",  PUMP_ADDR}, 
	{"G", PUMP_ADDR}, {"ZR", PUMP_ADDR}, {"DLCW", 0x30}, {"DHCW", 0x30}, {"SMSB", 0x30}, {"IGS", 0x30}, {"THM", 0x30},
	{"PERFUSIONDEWAX", 0x30},{"PERFUSIONALCOHOL", 0x30},{"PERFUSIONER1", 0x30},{"PERFUSIONER2", 0x30},{"PERFUSIONWATER", 0x30},
	{"PERFUSIONWASH", 0x30},{"FAN", 0x30},{"ASPIRATELIQUID", 0x30},{"MOVCLEANSTATION", 0x30},{"WASHALCOHOL", 0x30},{"FF", 0xFF}

};


int mb_monitdoorstate(void);



/******************************************************************************
*
* Function Name  : OpenComm
* Description    : .
*                      
* Input          : None
* Output         : None
* Return         :  error return -1, success return 0
*******************************************************************************/
void suspend_handler(int signum)  
{  
    g_bSuspend = 1;  
      
    sigset_t nset;  
    pthread_sigmask(0, NULL, &nset);  
    /* make sure that the resume is not blocked*/  
    sigdelset(&nset, SIGRESUME);  
    while(g_bSuspend) 
		sigsuspend(&nset);  
}  
  
/******************************************************************************
*
* Function Name  : OpenComm
* Description    : .
* 					 
* Input		   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
void resume_handler(int signum)  
{  
    g_bSuspend = 0;  
}  

/******************************************************************************
*
* Function Name  : OpenComm
* Description    : .
* 					 
* Input		   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
int suspend( pthread_t thread)  
{  
	//printf("=====================suspend==================>thread_id = %d.\n\n", (int)thread);
    return pthread_kill( thread, SIGSUSPEND);  
}  
  
/******************************************************************************
*
* Function Name  : OpenComm
* Description    : GPIO initialization, configuration GPIO 0~19 output, 20~31 input mode.
* 				   
* Input		 : None
* Output			 : None
* Return			 :	error return -1, success return 0
*******************************************************************************/
int resume( pthread_t thread)  
{  
	//printf("=====================resume==================>thread_id = %d.\n\n", (int)thread);
    return pthread_kill (thread, SIGRESUME);  
}  


/******************************************************************************
*
* Function Name  : OpenComm
* Description	 : The thread pool initialization.
*					 
* Input 	   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
stthreadpl *tp_threadini(int num_worker_threads,             /*线程池线程个数    最大任务数      是否阻塞任务满的时候 */
        int max_queue_size, int do_not_block_when_full)
{
    int i, rtn;
    stthreadpl *pool; 

	struct sigaction suspendsa;  
    struct sigaction resumesa;  

	pthread_attr_t thread_attr;
	pthread_attr_init(&thread_attr);
	//	pthread_attr_setstacksize(&thread_attr, 7*1024*1024); /// 线程栈空间2MB

	memset(&suspendsa, 0x00, sizeof(suspendsa));
	memset(&resumesa, 0x00, sizeof(resumesa));
	
    suspendsa.sa_handler =  suspend_handler;  
    sigaddset(&suspendsa.sa_mask, SIGRESUME);  
    sigaction( SIGSUSPEND, &suspendsa, NULL);  
      
    resumesa.sa_handler = resume_handler;  
    sigaddset(&resumesa.sa_mask, SIGSUSPEND);  
    sigaction( SIGRESUME, &resumesa, NULL);  

	//	lprintf(log_my, INFO, "init pool  begin ...\n");
    /* make the thread pool structure */
    if((pool = (struct tpool *)malloc(sizeof(struct tpool))) == NULL)
    {
        lprintf(log_my, FATAL, "Unable to malloc() thread pool!\n");
        return NULL;
    }

    /* set the desired thread pool values */
    pool->num_threads = num_worker_threads;                      /*工作线程个数*/
    pool->max_queue_size = max_queue_size;                       /*任务链表最大长度*/
    pool->do_not_block_when_full = do_not_block_when_full;       /*任务链表满时是否等待*/

    /* create an array to hold a ptr to the worker threads   生成线程池缓存 */
    if((pool->threads = (pthread_t *)malloc(sizeof(pthread_t)
                    *num_worker_threads)) == NULL)
    {
        lprintf(log_my, FATAL,"Unable to malloc() thread info array\n");
        return NULL;
    }

    /* initialize the work queue  初始化任务链表 */
    pool->cur_queue_size = 0;
    pool->queue_head = NULL;
    pool->queue_tail = NULL;
    pool->queue_closed = 0;
    pool->shutdown = 0;
	
    /* create the mutexs and cond vars  初始化互斥变量 条件变量 用于线程之间的同步 */
    if((rtn = pthread_mutex_init(&(pool->queue_lock),NULL)) != 0) {
        lprintf(log_my,FATAL,"pthread_mutex_init %s",strerror(rtn));
        return NULL;
    }
    if((rtn = pthread_cond_init(&(pool->queue_not_empty),NULL)) != 0) {
        lprintf(log_my,FATAL,"pthread_cond_init %s",strerror(rtn));
        return NULL;
    }
    if((rtn = pthread_cond_init(&(pool->queue_not_full),NULL)) != 0) {
        lprintf(log_my,FATAL,"pthread_cond_init %s",strerror(rtn));
        return NULL;
    }
    if((rtn = pthread_cond_init(&(pool->queue_empty),NULL)) != 0) {
        lprintf(log_my,FATAL,"pthread_cond_init %s",strerror(rtn));
        return NULL;
    }

    /* 
     * from "man 3c pthread_attr_init"
     * Define the scheduling contention scope for the created thread.  The only
     * value     supported    in    the    LinuxThreads    implementation    is
     * !PTHREAD_SCOPE_SYSTEM!, meaning that the threads contend  for  CPU  time
     * with all processes running on the machine.
     *
     * so no need to explicitly set the SCOPE
     */

    /* create the individual worker threads */
    for(i = 0; i != num_worker_threads; i++)
    {
        if( (rtn = pthread_create(&(pool->threads[i]), NULL, tpool_thread,(void*)pool)) != 0)
		//  if( (rtn = pthread_create(&(pool->threads[i]), &thread_attr, tpool_thread,(void*)pool)) != 0)
        {
            lprintf(log_my,FATAL,"pthread_create %s\n",strerror(rtn));
            return NULL;
        }

		thread_id[i] = pool->threads[i];		
    }
	
	thread_num = num_worker_threads;

	lprintf(log_my, INFO, "init pool end!\n");
    return pool;
}

/******************************************************************************
*
* Function Name  : tp_addthreadwk
* Description	 : The thread pool initialization.
*					 
* Input 	   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
int tp_addthreadwk(stthreadpl *pool, void (*routine)(void *), void *arg)
{
    int rtn;
    sttpwk *workp;

    if((rtn = pthread_mutex_lock(&pool->queue_lock)) != 0)
    {
        lprintf(log_my,FATAL,"pthread mutex lock failure\n");
        return -1;
    }

    /* now we have exclusive access to the work queue ! */

    if((pool->cur_queue_size == pool->max_queue_size) &&
            (pool->do_not_block_when_full))
    {
        if((rtn = pthread_mutex_unlock(&pool->queue_lock)) != 0)
        {
            lprintf(log_my,FATAL,"pthread mutex lock failure\n");
            return -1;
        }
        return -1;
    }

    /* wait for the queue to have an open space for new work, while
     * waiting the queue_lock will be released */
    while((pool->cur_queue_size == pool->max_queue_size) &&
            (!(pool->shutdown || pool->queue_closed)))
    {
        if((rtn = pthread_cond_wait(&(pool->queue_not_full),
                        &(pool->queue_lock)) ) != 0)
        {
            lprintf(log_my,FATAL,"pthread cond wait failure\n");
            return -1;
        }
    }

    if(pool->shutdown || pool->queue_closed)
    {
        if((rtn = pthread_mutex_unlock(&pool->queue_lock)) != 0)
        {
            lprintf(log_my,FATAL,"pthread mutex lock failure\n");
            return -1;
        }
        return -1;
    }

    /* allocate the work structure */
    if((workp = (sttpwk *)malloc(sizeof(sttpwk)))
            == NULL)
    {
        lprintf(log_my,FATAL,"unable to create work struct\n");
        return -1;
    }

    /* set the function/routine which will handle the work,
     * (note: it must be reenterant) */
    workp->handler_routine = routine;
    workp->arg = arg;
    workp->next = NULL;

    if(pool->cur_queue_size == 0)
    {
        pool->queue_tail = pool->queue_head = workp;
        if((rtn = pthread_cond_broadcast(&(pool->queue_not_empty))) != 0)
        {
            lprintf(log_my,FATAL,"pthread broadcast error\n");
            return -1;
        }
    }
    else
    {
        pool->queue_tail->next = workp;
        pool->queue_tail = workp;
    }

    pool->cur_queue_size++;

    /* relinquish control of the queue */
    if((rtn = pthread_mutex_unlock(&pool->queue_lock)) != 0)
    {
        lprintf(log_my,FATAL,"pthread mutex lock failure\n");
        return -1;
    }

    return 0;
}

/******************************************************************************
*
* Function Name  : tp_addthreadwk
* Description	 : The thread pool initialization.
*					 
* Input 	   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
int tp_threaddestry(stthreadpl *pool, int finish)
{
    int i, rtn;
    sttpwk *cur;   /*当前工作线程句柄*/

	lprintf(log_my, INFO, "destroy pool begin!\n");

    /* relinquish control of the queue */
    if((rtn = pthread_mutex_lock(&(pool->queue_lock))) != 0)
    {
        lprintf(log_my,FATAL,"pthread mutex lock failure\n");
        return -1;
    }
	
	lprintf(log_my, INFO, "destroy pool begin 1!\n");
    /* is a shutdown already going on ? */
    if(pool->queue_closed || pool->shutdown)
    {
        if((rtn = pthread_mutex_unlock(&(pool->queue_lock))) != 0)
        {
            lprintf(log_my,FATAL,"pthread mutex lock failure\n");
            return -1;
        }
        return 0;
    }

	lprintf(log_my, INFO, "destroy pool begin 2!\n");
    /* close the queue to any new work   对于新任务来说所有交易禁止 */
    pool->queue_closed = 1;

    /* if the finish flag is set, drain the queue */
    if(finish)
    {
        while(pool->cur_queue_size != 0)
        {
            /* wait for the queue to become empty,
             * while waiting queue lock will be released */
            if((rtn = pthread_cond_wait(&(pool->queue_empty),
                            &(pool->queue_lock))) != 0)
            {
                lprintf(log_my,FATAL,"pthread_cond_wait %d\n",rtn);
                return -1;
            }
        }
    }

	lprintf(log_my, INFO, "destroy pool begin 3!\n");

	/* set the shutdown flag */
    pool->shutdown = 1;

    if((rtn = pthread_mutex_unlock(&(pool->queue_lock))) != 0)
    {
        lprintf(log_my,FATAL,"pthread mutex unlock failure\n");
        return -1;
    }
	  /*  return 0; */
  
	  lprintf(log_my, INFO, "destroy pool begin 4!\n");

    /* wake up all workers to rechedk the shutdown flag */
    if((rtn = pthread_cond_broadcast(&(pool->queue_not_empty)))
            != 0)
    {
        lprintf(log_my,FATAL,"pthread_cond_boradcast %d\n",rtn);
        return -1;
    }
    if((rtn = pthread_cond_broadcast(&(pool->queue_not_full)))
            != 0)
    {
        lprintf(log_my,FATAL,"pthread_cond_boradcast %d\n",rtn);
        return -1;
    }

    /* wait for workers to exit */
    for(i = 0; i < pool->num_threads; i++)
    {
        if((rtn = pthread_join(pool->threads[i],NULL)) != 0)
        {
            lprintf(log_my,FATAL,"pthread_join %d\n",rtn);
            return -1;
        }
    }

    /* clean up memory */
    free(pool->threads);
    while(pool->queue_head != NULL)
    {
        cur = pool->queue_head->next;
        pool->queue_head = pool->queue_head->next;
        free(cur);
    }
    free(pool);

	lprintf(log_my, INFO, "destroy pool end!\n");

    return 0;
}


/******************************************************************************
*
* Function Name  : tp_addthreadwk
* Description	 : The thread pool function.
*					 
* Input 	   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
void *tpool_thread(void *tpool)
{
    sttpwk *my_work;
    stthreadpl *pool = (struct tpool *)tpool;

   /* int *soket;  */                /*socket 句柄*/
    for(;;) /* go forever */
    {
        pthread_mutex_lock(&(pool->queue_lock));   

        /* sleep until there is work,
         * while asleep the queue_lock is relinquished */
        while((pool->cur_queue_size == 0) && (!pool->shutdown))  /* 任务列表为0  并且 线程池没有关闭 */
        {
            pthread_cond_wait(&(pool->queue_not_empty),          /* 等待直到，任务到来为止 */
                    &(pool->queue_lock));
        }

        /* are we shutting down ?  线程池是否已经关闭，如果线程池关闭则线程自己主动关闭 */
        if(pool->shutdown)
        {
            pthread_mutex_unlock(&(pool->queue_lock));
            pthread_exit(NULL);      /*线程退出状态为空，主线程不捕获各副线程状态*/
        }

        /* process the work */
        my_work = pool->queue_head;  
        pool->cur_queue_size--;
        
        if(pool->cur_queue_size == 0)    /*将任务链表头部去掉，改任务正在处理中*/
            pool->queue_head = pool->queue_tail = NULL;
        else
            pool->queue_head = my_work->next;

        /* broadcast that the queue is not full */ 
        if((!pool->do_not_block_when_full) &&
                (pool->cur_queue_size == (pool->max_queue_size - 1)))
        {
            pthread_cond_broadcast(&(pool->queue_not_full));
        }

        if(pool->cur_queue_size == 0) /*任务链表为空*/
        {
            pthread_cond_signal(&(pool->queue_empty));
        }

        pthread_mutex_unlock(&(pool->queue_lock));

        /* perform the work */
        (*(my_work->handler_routine))(my_work->arg);        /*启动线程业务处理逻辑*/

		free(my_work);
    }
    return(NULL);
}


/******************************************************************************
*
* Function Name  : tp_getrpara
* Description	 : The thread pool function.
*					 
* Input 	   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
int tp_getrpara(void)
{
	// Get the pump connect mode
	pumpNum = rf_readintfromeprofile((const char *) "pump numbers", (const char*) "PUMP_NUMBERS", -1, (const char *)g_szConfigPath);
	assert(pumpNum != -1);

	armNum = rf_readintfromeprofile((const char *) "arm numbers", (const char*) "ARM_NUMBER", -1, (const char *)g_szConfigPath);
	assert(armNum != -1);

	printf("[tp_getrpara]: pumpConnectMode = %d, pumpNum = %d, armNum = %d.\n\n", pumpConnectMode, pumpNum, armNum);

	return 1;
}


void tp_setrtpoint(void)
{
	
}

/*****************************************
所有清洗站洗液控制   当主臂或者自控臂有吸液请求时 isWashWork++; 停止吸液请求时isWashWork--
当isWashWork == 0 时 所有请求都为停止吸液 吸液线程即可停止吸液
******************************************/
void tp_washchange(bool add)
{
	pthread_mutex_lock(&mutex_washworklock);
	if (add)
		isWashWork++;
	else
	{
		sleep(1);//确保液体吸收
		isWashWork--;
	}
	if (pthread_mutex_unlock(&mutex_washworklock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_washworklock");
	sleep(1);//确保清洗线程启动清洗
}

/******************************************************************************
	清洗针
*******************************************************************************/
void tp_washprob(void)
{
	GPIO_OutSet(V8);
	GPIO_OutSet(V9);
	GPIO_OutSet(V5);
	GPIO_OutSet(V6);

	GPIO_OutClear(V4);
	sleep(1);
}

/*************
GPIO_OutClear 关闭
**************/
void tp_thread_probwashclear(void *arg)
{
	char last_isWashWork = isWashWork;

	while(!flg_mainproexit)
	{
		if (wkevent == MAINTAIN_WORK)
		{
			sleep(1);
			continue;
		}

		if (inDischarge)		{
			IsWashProbeStart = FALSE;//
		}
		
		if (isWashWork > 0)
		{
			pthread_mutex_lock(&pump_lock);
			GPIO_OutClear(V1);
			GPIO_OutClear(V2);
			GPIO_OutClear(V3);
			GPIO_OutSet(V4);
			GPIO_OutSet(V7);
			GPIO_OutClear(V6);
			GPIO_OutClear(V8);
			GPIO_OutClear(V9);
			if (isHighConcentration > 0)
				GPIO_OutClear(V5);
			else				
				GPIO_OutSet(V5);
			sleep(1);
			GPIO_OutSet(VP1);
			if (pthread_mutex_unlock(&pump_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
			IsWashProbeStart = true;
			
			if (last_isWashWork < isWashWork)	//说明有新的清洗事件加入需要外壁冲水
			{
				sleep(1);//吸液充分打开后再打水
				printf("doing tp_washprob Out");

				usleep(500000);
			}
		}
		else if (isWashWork == 0)//停止吸液
		{
			sleep(2);
			pthread_mutex_lock(&pump_lock);
			 GPIO_OutClear(VP1);
			GPIO_OutClear(V4);
			GPIO_OutClear(V5);
			GPIO_OutClear(V6);	
			GPIO_OutClear(V7);
			if (pthread_mutex_unlock(&pump_lock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error pump_lock");
			IsWashProbeStart = false;
		}
		else
			printf("tp_thread_probwashclear errorrrrrrrrrrrrrrrrr isWashWork < 0");
		
		last_isWashWork = isWashWork;
		
		usleep(10000);
	}
}



int Is_Error_Occur(void)
{
	sterrcode_list * perror = perror_exist_head;

	printf("errorcode_exist=");
	while(perror != NULL)
	{
		printf(" %d ", perror->error_code);
		if (perror->error_code >= WATERPOUR_WRONG && perror->error_code <= ER2POUR_WRONG)
		{
			return -1;
		}
		perror = perror->next;
	}
	return 0;
};



int GetMixStationVal(char cmd,char num)
{
	int res = 0, i = 0;
	unsigned short crc = 0;
	char cmdbuffer[10] ={0};
	static char seq = 0;
	int eor_cnt=0, timout_cnt=0;

	pthread_mutex_lock(&mutex_mixlock);
	usleep(500000);//防止两个线程发两个命令连续
	if (seq > 7)
		seq = 0;
	
	cmdbuffer[0] = 'M';
	cmdbuffer[1] = 0;
	cmdbuffer[2] = seq;
	cmdbuffer[3] = cmd;
	cmdbuffer[4] = num;
	
	crc = usMBCRC16((uint8_t*)cmdbuffer, 5);
	memcpy(&cmdbuffer[5],&crc,2);
	tcflush(serail_hand[mix_port],TCOFLUSH);
	if (0)
	{
		printf("the data send to MIX\n");
		for (i = 0;i < 5 + 2;i++)
			printf(" %x",cmdbuffer[i]);
	}
	res = write(serail_hand[mix_port],cmdbuffer,5 + 2);
	if (res < 0)
		printf("write mix error\n");
	else
		printf("write mix successed\n");
	{
		tcflush(serail_hand[mix_port],TCIFLUSH);

		Mix_Res = MixACK = false;
		while(!MixACK && !Mix_Res)
		{
			if(timout_cnt++ > 9)
			{
				timout_cnt = 0;
				if (eor_cnt++ > 9)
				{
					mb_seterrorcode(CONNECT_ERROR_MIX);
					
					pthread_mutex_unlock(&mutex_mixlock);
					return -3;
				}
				cmdbuffer[2] |= 0X80; 
				res = write(serail_hand[mix_port],cmdbuffer,5 + 2);
				if (res < 0)
					printf("write mix error\n");
				else
					printf("write mix successed\n");
				printf("recieve mix timeout send again\n");
			}																		
			
			usleep(200000);
		}

		timout_cnt = eor_cnt = 0;

		while(!Mix_Res)
		{
			if(timout_cnt++ > 100)
			{
				pthread_mutex_unlock(&mutex_mixlock);
				return -3;
			}
			usleep(100000);
		}	
	}
	pthread_mutex_unlock(&mutex_mixlock);	

	return 0;
}

/******************************************************************************
*
* Function Name  : tp_devinit
* Description	 : The thread pool function.
*					 
* Input 	   : None
* Output		   : None
* Return		   :  error return -1, success return 0
*******************************************************************************/
int tp_devinit(bool littleinit)
{
	int cnt = 0, command_field_cnt = 0, mixstation_state = 0;
	unsigned char isReaptSnd = 0;
	unsigned char pumpNo = 0;
	unsigned int sr_cmdcount = 0;
	unsigned int command_field_num = 0;
	static unsigned char seqNum = 1;
	uint8_t i = 0;
	char pstring[BUF_SIZE] = {0};
	char command_field_str[16][BUF_SIZE];
	char str[64] = {0};
	char string[64] = {0};
	char last_perfusion_port = 0; //主臂
	char *ptr = NULL;
	sr_cmdstruct_t	sr_cmdarray[50];
	sr_cmdstruct_t sr_cmdframe;
	int cmdret = 0;
	stminibd_sendpacket cmd;

	i = 0;
	while(i < 6)
	{	
		if(littleinit)
		{
			break;
		}
		usleep(100000);  //	usleep(1000000);
		for (i = 0; i < 6; i++)
		{
			if (flg_cabinhavereagent[i] )	
			{	
				if ( cabin_value[i + 6] == 0)
				{
					#if(USE_LOG_INFO == 1)
					printf("Device inition the cabin low level was removed num = %d\n", i);
					#endif
					break;
				}
				if (cabin_value[i] == 1)
				{
					#if(USE_LOG_INFO == 1)
					printf("Device inition the cabin high level cabin was not enough num = %d\n", i);
					#endif
					break;
				}
			}	
		}
	}

	sleep(1);//确保传感器检测过
	while(OUT_WASTHIGH_L || OUT_WASTHIGH_H) sleep(1);
		
	sleep(1);
	#if(USE_LOG_INFO == 1)
	printf("[================tp_devinit]: Begin to initing device.==================\n" );
	#endif

	memset(&sr_cmdarray, '\0', sizeof(sr_cmdstruct_t));
	memset(str, '\0', sizeof(str));

	sr_cmdframe.srdevaddr = ARM_ADDR;
	
	//灵敏度在酒精灌注后又有设置
	sprintf(sr_cmdframe.cmdbuf, "SL %d %d", DEC_SEN, 0X2500);
	sm_serailsenddat(port_arm, &sr_cmdframe, 1, 0, 2);

	sprintf(sr_cmdframe.cmdbuf, "OM %d %d %d", 2878, 2109, 1681);
	sm_serailsenddat(port_arm, &sr_cmdframe, 1, 0, 2);

	// sprintf(sr_cmdframe.cmdbuf, "FX %d %d %d %d", 2000, 4, 30000, 2000);//细分0.9° 330HZ力最大 涤速度最大8否则失步
	// sm_serailsenddat(port_arm, &sr_cmdframe, 1, 0, 2);

	//初始化文件命令后面不能有空格
	command_field_num = rf_readintfromeprofile((const char *) "field information", (const char *)"COMMAND_FIELD_NUMBERS", -1,\
					 (const char *) g_szConfigPath);
	assert(command_field_num != 0);

	printf("0.-------->>>command_field_num = %d.\n", command_field_num);
	for(i = 0; i < command_field_num; i++)
	{	
		sprintf(str, "COMMAND_FIELD_%d", i+1);
		if (!rf_readstringfromeprofile((const char *)"field information", (const char *)str, command_field_str[i], \
				BUF_SIZE, NULL, (const char *)g_szConfigPath)) {
			continue;
		}
	}
	//	printf("command_field_num = %d\n", command_field_num);
	//字段解析
	for (command_field_cnt = 0; command_field_cnt < command_field_num; command_field_cnt++) 
	{
		sr_cmdcount = rf_readintfromeprofile((const char *) command_field_str[command_field_cnt], (const char*) 
			"COMMAND_NUMBERS", -1, (const char *)g_szConfigPath);
		//	assert(sr_cmdcount != 0);

		memset(string, '\0', sizeof(string));
		rf_readstringfromeprofile((const char *) command_field_str[(uint8_t)command_field_cnt], (const char *) 
				"COMMAND_STRING", string, 64, NULL, (const char *) g_szConfigPath);
		assert(string != NULL);

		for (i = 0; i < sr_cmdcount; i++)
		{
			memset(&(sr_cmdarray[i].cmdbuf), '\0', sizeof(sr_cmdarray[i].cmdbuf));
			memset(str, '\0', sizeof(str));
			
			sprintf(str, "%s%d", string, i + 1);
			if (!rf_readstringfromeprofile((const char *)command_field_str[(uint8_t)command_field_cnt], 
				(const char *)str, sr_cmdarray[i].cmdbuf, BUF_SIZE, NULL, (const char *)g_szConfigPath)) {
				continue;
			}

			strcpy(pstring, sr_cmdarray[i].cmdbuf);
			char *token = strtok_r(pstring, " ", &ptr);
			//	printf("6.-------->>>token = %s.\n", token);
			for (cnt = 0; cnt < InitMaxStrNum; cnt++)
			{
				if (strcmp(srailcmdstringarray[cnt].str, token) == 0)
				{	
					sr_cmdarray[i].srdevaddr = srailcmdstringarray[cnt].device_addr;
					//printf("3.-------->>>str = %s, CommandElemt.srdevaddr = %02x, device_addr = %02x.\n",srailcmdstringarray[cnt].str, CommandElemt.srdevaddr, srailcmdstringarray[cnt].device_addr);
					break;
				}
			}
			
			memset(str, '\0', sizeof(str));
		}

		cnt = 0;
		if (wkevent == MAINTAIN_WORK)
		{
			printf("in MAINTAIN_WORK\n");
			return 0;
		}
		wkevent = INIT_WORK;
		flg_shelfarmsleep[0] = false;

		printf("commandNumbers=%d\n", sr_cmdcount);
		while(cnt < sr_cmdcount)
		{
			if (flg_mainproexit)
				return -1;
			if (wkevent == MAINTAIN_WORK)
			{
				printf("in MAINTAIN_WORK\n");
				return 0;
			}
			mb_monitdoorstate();		
			if(littleinit )
			{
				if((sr_cmdarray[cnt].srdevaddr == ARM_ADDR) || (sr_cmdarray[cnt].srdevaddr == PUMP_ADDR))
				{
					
				}
				else
				{
					cnt++;
					continue;
				}
			}
			
			switch (sr_cmdarray[cnt].srdevaddr) 
			{

				case PUMP_ADDR:	// 佂对多泵
					tp_washchange(true);            //吸液 注射泵初始化可能有液体
					strcpy(sr_cmdarray[cnt].cmdbuf, "ZR");
					sr_cmdarray[cnt].srdevaddr += pumpNo;
					sm_serailsenddat(port_arm, sr_cmdarray + cnt, seqNum, isReaptSnd, pumpConnectMode);
					tp_washchange(false);
							
					sr_cmdarray[cnt].srdevaddr = ARM_ADDR;
					sprintf(sr_cmdarray[cnt].cmdbuf, "ZA %d", MOV_ZH);
					sm_serailsenddat(port_arm, sr_cmdarray + cnt, 1, 0, 1);
						
					break;
				case 0x38: 
					if(0 > sm_serailsenddat(port_arm, sr_cmdarray + cnt, seqNum, 0, pumpConnectMode))
					{
						return -1;
					}

					break;
				case 0x30:
					if (strcmp("MOVCLEANSTATION", sr_cmdarray[cnt].cmdbuf) == 0) {
						printf("***************in MOVCLEANSTATION**************\n");
					}
								
					if (strcmp("DLCW", sr_cmdarray[cnt].cmdbuf) == 0) {
							//排低浓度废液
						printf("***************in DLCW**************\n");

						#if NEED_INIT	
							if(!mb_dischargwateliquid_lo(DischargeTime - 3))  // if(!mb_dischargwateliquid_lo(DischargeTime))
						#endif
							{
								printf("mb_dischargwateliquid_lo error\n");
								printf("\n");
							}			
					} 

					if (strcmp("DHCW", sr_cmdarray[cnt].cmdbuf) == 0) {
						// 排高浓度废液
						printf("***************in DHCW**************\n");
						#if NEED_INIT				
						if(!mb_dischwasteliquid_hi(DischargeTime - 3))   // if(!mb_dischwasteliquid_hi(DischargeTime))
						#endif
						{
							printf("mb_dischwasteliquid_hi error\n");
							printf("\n");
						}
					} 	
							
					if (strcmp("IGS", sr_cmdarray[cnt].cmdbuf) == 0) {
						memset(&cmd, 0, sizeof(cmd));		//
						cmd.cmd = INIT_MiniCmd;   //   
						cmd.minicmd_buffer[3] = (unsigned char)INIT_MiniCmd;
						cmd.minicmd_buffer[4] = 150;     //SPEED //char 类型最大255
						cmd.minicmd_num = 2;
						pthread_mutex_lock(&mutex_mlock);
						set_minicmd(pcmd_head, cmd);
						if (pthread_mutex_unlock(&mutex_mlock) != 0)
							lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");

						printf("***************in IGS**************\n");
					}
						
					if (strcmp("THM", sr_cmdarray[cnt].cmdbuf) == 0) {
						// 加热模块检测
						printf("***************in THM**************\n");
						//在温控线程中初始化			
					} 
					
					if (strcmp("SMSB", sr_cmdarray[cnt].cmdbuf) == 0)
					{
						// 扫描混合站
						printf("***************in SMSB**************\n");

						mb_monitdoorstate();	
						sr_cmdframe.srdevaddr = ARM_ADDR;
						if(honey_scaner)
							sprintf(sr_cmdframe.cmdbuf, "PA %d %d %d", ordArray_mixed_DAB[0][0] + SCAN_DISTANC_MIXSTATION - 40, ordArray_mixed_DAB[0][1], MOV_ZH);// 1号清洗槽标准的邋錎AB扫描位置
						else		
							sprintf(sr_cmdframe.cmdbuf, "PA %d %d %d", ordArray_mixed_DAB[0][0] + SCAN_DISTANC_MIXSTATION, ordArray_mixed_DAB[0][1], MOV_ZH);// 1号清洗槽标准的邋錎AB扫描位置
						sm_serailsenddat(port_arm, &sr_cmdframe, 1, 0, 1);
						#if(USE_LOG_INFO == 1)
						printf("SMSB PA %d %d %d", ordArray_mixed_DAB[0][0] + SCAN_DISTANC_MIXSTATION, ordArray_mixed_DAB[0][1], MOV_ZH);
						#endif
						sleep(1);
						
						mixstation_state = 0;
						if (!sc_getscode(0XED, false))
						{
							if (!sc_getscode(0XEE, false))	
								mixstation_state = -1;
						}

						if(mixstation_state == 0)
						{	
							for (i = 0; i < 6; i++)
							{
								mb_monitdoorstate();
								sr_cmdframe.srdevaddr = ARM_ADDR;
								#if(1 == S600_SHWON_DOWN_DABCNG)  //  +20 X太往右走，撞到第2个混合杯右边孔壁， -5： X往左偏了。
								ordArray_mixed_DAB[i][0] += 4; //7; // 6; +=6： 从仪器正前方看探针还要往右2mm到混合杯孔中心
								sprintf(sr_cmdframe.cmdbuf, "PA %d %d %d", ordArray_mixed_DAB[i][0], ordArray_mixed_DAB[i][1], MOV_ZH);
								#else
								sprintf(sr_cmdframe.cmdbuf, "PA %d %d %d", ordArray_mixed_DAB[i][0], ordArray_mixed_DAB[i][1], MOV_ZH);//混合站目标位置
								#endif
								//sprintf(sr_cmdframe.cmdbuf, "PA %d %d %d", ordArray_mixed_DAB[i][0], ordArray_mixed_DAB[i][1], MOV_ZH);  //混合站目标位置
								sm_serailsenddat(port_arm, &sr_cmdframe, 1, 0, 1);
								mb_monitdoorstate();
								
								#if(S600_DABCLEAR_DEPTH == 1)
								sprintf(sr_cmdframe.cmdbuf, "ZX 0, 0, %d", ZDAB_STEP + conf_zdab_addstep);
								#else
								sprintf(sr_cmdframe.cmdbuf, "ZX %d %d %d", 0, 0, ZDAB_STEP);  //测液体
								#endif
								cmdret = sm_serailsenddat(port_arm, &sr_cmdframe, 1, 0, 1);

								#if(USE_LOG_INFO == 1)
								lprintf(log_my, INFO, "TP_devinit[%d] ZX result=[%d]. dabStep=[%d] addStep=[%d].\n", i, cmdret, ZDAB_STEP, conf_zdab_addstep);
								printf("TP_devinit [%d] ZX result=[%d]. dabStep=[%d] addStep=[%d].\n", i, cmdret, ZDAB_STEP, conf_zdab_addstep);
								#endif

								if(cmdret != -9){
									#if(USE_LOG_INFO == 1)
									lprintf(log_my, INFO, "TP_devinit[%d] [%d] ready clear mixstation.\n", i, cmdret);
									printf("TP_devinit [%d] [%d] ready clear mixstation.\n", i, cmdret);
									#endif

									if (mb_clearmixstation( &ordArray_mixed_DAB[i][0]) < 0)
									{
										#if(USE_LOG_INFO == 1)
										lprintf(log_my, INFO, "mb_clearmixstation mixdab[i][0]=[%d].\n", cmdret, ordArray_mixed_DAB[i][0]);
										#endif
										mb_seterrorcode(MIX_STATION_CLEARERROR);
										break;
									}					
								}
							}
						}
						else if (mixstation_state == -1)
							mb_seterrorcode(MIX_STATION_MOVED);
						else if (mixstation_state == -2)
							mb_seterrorcode(MIX_STATION_NOTCLEAR);					 						
					}
					if (strcmp("FAN", sr_cmdarray[cnt].cmdbuf) == 0){
						printf("***************in FAN**************\n");
						/*	GPIO_OutSet(V10);
							sleep(20);
							GPIO_OutClear(V10);
						*/	
					}

					if (strcmp("PERFUSIONDEWAX", sr_cmdarray[cnt].cmdbuf) == 0){
						printf("***************in PERFUSIONDEWAX**************\n");
						
						last_perfusion_port = DEWAXPORT;
						//	if (flg_cabinhavereagent[0] == 0) //检测不到取消灌注
						{	//当被设置成无效是不进行灌注
							cnt++;
							continue;
						}
					}

					if (strcmp("PERFUSIONALCOHOL", sr_cmdarray[cnt].cmdbuf) == 0){

						printf("***************in PERFUSIONALCOHOL**************\n");
							last_perfusion_port = ALCOHOLPORT;
						if (flg_cabinhavereagent[1] == 0){	
							cnt++;
							continue;
						}
					}
					if (strcmp("PERFUSIONWATER", sr_cmdarray[cnt].cmdbuf) == 0){
		
						printf("***************in PERFUSIONWATER**************\n");
							last_perfusion_port = WATERPORT;
							if (flg_cabinhavereagent[2] == 0){	
							cnt++;
							continue;
						}
					}
					if (strcmp("PERFUSIONWASH", sr_cmdarray[cnt].cmdbuf) == 0){
						
						printf("***************in PERFUSIONWASH**************\n");
							last_perfusion_port = WASHPORT;		
							if (flg_cabinhavereagent[3] == 0){	
							cnt++;
							continue;
						}
					}
					if (strcmp("PERFUSIONER1", sr_cmdarray[cnt].cmdbuf) == 0){

						printf("***************in PERFUSIONER1**************\n");
							last_perfusion_port = ER1PORT;	
							if (flg_cabinhavereagent[4] == 0){	
							cnt++;
							continue;
						}
					}
					if (strcmp("PERFUSIONER2", sr_cmdarray[cnt].cmdbuf) == 0){

						printf("***************in PERFUSIONER2**************\n");
							last_perfusion_port = ER2PORT;
							if (flg_cabinhavereagent[5] == 0){	
							cnt++;
							continue;
						}
					}
					
					if (strcmp("ASPIRATELIQUID", sr_cmdarray[cnt].cmdbuf) == 0){
							printf("***************in ASPIRATELIQUID**************\n");
						//		mb_dischargeshelfwateliquid(1);
						//		mb_dischargeshelfwateliquid(2);
						//		mb_dischargeshelfwateliquid(3);
						//		mb_dischargeshelfwateliquid(4);	
					}

					sr_cmdarray[cnt].cmdbuf[9] = '\0';
					if (strcmp("PERFUSION", sr_cmdarray[cnt].cmdbuf) == 0)
					{
						if (cnt >= 26)
						{	
							tp_washchange(true);
							//	last_perfusion_port = WATERPORT;
							if (last_perfusion_port == WATERPORT || last_perfusion_port == WASHPORT)
							{
								if (!mb_waterwashreagentpour(last_perfusion_port))
								{
									if (last_perfusion_port == WATERPORT)	
										mb_seterrorcode(WATERPOUR_WRONG);
									if (last_perfusion_port == WASHPORT)//大仪器只需要灌注WASH
										mb_seterrorcode(WASHPOUR_WRONG);
								}
							}
							else
							{			
								if (!mb_muiltreagentpour(last_perfusion_port))
								{
									if (last_perfusion_port == ALCOHOLPORT)
										mb_seterrorcode(ALCOHOLPOUR_WRONG);
									//		else if (last_perfusion_port == DEWAXPORT)//脱蜡剂检测不到
									//			mb_seterrorcode(DEWAXPOUR_WRONG);	
									else if (last_perfusion_port == ER1PORT)
										mb_seterrorcode(ER1POUR_WRONG);
									else if (last_perfusion_port == ER2PORT)
										mb_seterrorcode(ER2POUR_WRONG);
								}
							}
							tp_washchange(false);
						}
						else
						{
				
						}
					}										
					break;

				default:
				break;
			}     //  end for switch (CommandElemt[cnt].srdevaddr) 
				
			cnt++;   //   for while(cnt < commandNumbers)	
		}  // end for while(cnt < commandNumbers)
	}   //  end for 	for (command_field_cnt = 0; command_field_cnt < command_field_num; command_field_cnt++) 

	if(littleinit)
	{
		flg_shelfarmsleep[0] = true;
		ArmCabinCmdList[0].cmd = ArmCabinCmdList[1].cmd = ArmCabinCmdList[2].cmd = ArmCabinKey;
		wkevent = FREE_WORK;
		printf("OUTOF short init\n");
		return 0;
	}
	last_cabin_reagent = REAGENT_WATER;
	
	while(!TEM_init_work_finished)  usleep(200000);  // sleep(1);

	while(!mini_work_finished[2] || !mini_work_finished[1] || !mini_work_finished[0])
		usleep(10000);

	sleep(20);//防止温度初始化关风机程序错误
	memset(&cmd,0,sizeof(cmd));
	cmd.cmd = RELOAD;
	cmd.minicmd_buffer[3] = (unsigned char)RELOAD;
	cmd.minicmd_buffer[4] = 1;
	cmd.minicmd_num=2;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");

	cmd.minicmd_buffer[4] = 2;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	
	cmd.minicmd_buffer[4] = 3;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_mlock");
	
	initialize_finished = true;
	nt_sendpacketdata(INIT_END, (char*)netbuf_write, 0);
	wkevent = FREE_WORK;
	if (new_version)
	{
		flg_shelfarmsleep[0] = flg_shelfarmsleep[1] = flg_shelfarmsleep[2] = flg_shelfarmsleep[4] = TRUE;
		//发送可休眠状态
		//	tp_sendtemperframe(30, 3, 1,1,&rdata);
	}

	printf("[tp_devinit]: Init device end.\n" );
	return 0;
}


int tp_checkmachstoped(bool mainarm_finished)
{
	stminibd_sendpacket cmd;
	
	memset(&cmd,0,sizeof(cmd));
	#ifdef glint
	cmd.cmd = WORK_FINISH;//RELOAD;
	cmd.minicmd_buffer[3] = (unsigned char)WORK_FINISH;//RELOAD;
	#else
	cmd.cmd = RELOAD;//RELOAD;
	cmd.minicmd_buffer[3] = (unsigned char)RELOAD;//RELOAD;
	#endif
	cmd.minicmd_num=2;
	if (mainarm_finished == flg_mianarmstop_a && !flg_opwork1ready && !hydrateA.flage && wkeventA != STANDBY_WORK)
	{
		#if(USE_LOG_INFO == 1)
		printf("tp_checkmachstoped_a.\n");
		lprintf(log_my, INFO, "tp_checkmachstoped_a.\n");
		#endif

		pthread_mutex_lock(&mutex_mneedworked);
		cmd.minicmd_buffer[4] = 1;
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
		pthread_mutex_unlock(&mutex_mneedworked);
		return -1;
	}
	if (mainarm_finished == flg_mianarmstop_b && !flg_opwork2ready && !hydrateB.flage && wkeventB != STANDBY_WORK)
	{
		#if(USE_LOG_INFO == 1)
		printf("tp_checkmachstoped_b.\n");
		lprintf(log_my, INFO, "tp_checkmachstoped_b.\n");
		#endif
		pthread_mutex_lock(&mutex_mneedworked);
		cmd.minicmd_buffer[4] = 2;
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
		pthread_mutex_unlock(&mutex_mneedworked);
		return -1;
	}
	if (mainarm_finished == flg_mianarmstop_c && !flg_opwork3ready && !hydrateC.flage && wkeventC != STANDBY_WORK)
	{
		#if(USE_LOG_INFO == 1)
		printf("tp_checkmachstoped_c.\n");
		lprintf(log_my, INFO, "tp_checkmachstoped_c.\n");
		#endif
		pthread_mutex_lock(&mutex_mneedworked);
		cmd.minicmd_buffer[4] = 3;
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
		pthread_mutex_unlock(&mutex_mneedworked);
		return -1;
	}	

	return 0;
}


void tp_fanworkproce(char shelf_num, int time_sec)
{
	stminibd_sendpacket cmd;

	cmd.cmd = MAINTAIN_FAN_WORK;
	cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
	cmd.minicmd_buffer[4] = shelf_num;
	cmd.minicmd_num=2;

	cmd.minicmd_buffer[4] += 0;//ON
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s\n", "pthread_mutex_unlock error tp_fanworkproce");

	while(In_bs_minishelfwork)//动作完成
	{
		usleep(200000);
	}
					
	sleep(time_sec);		
	cmd.minicmd_buffer[4] += 3;//OFF
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
	lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error tp_fanworkproce");
}


extern reagent_t reagent_code[];
extern volatile char lastt_kind[][9];

void tp_mainarmbusses(bool mainarm_finished, dispense_t* dispenseIN, bool mini_work_finished, const unsigned int liquid_val, \
					unsigned char* last_reagent, uint8_t shelfindex)
{
	stminibd_sendpacket cmd;
	unsigned char reagent_lock_num = 0, CriticalWork_num = 0;
	struct timeval now;
	gettimeofday(&now, NULL);
	operate_head_list* operate_head = NULL;
	uint8_t shelf_num = 0;
	uint32_t* head_step_p = NULL;
	uint32_t i = 0;
	temper_control_t* temper_control = NULL;
	dispense_t dispense_change[2][10];
	dispense_t dispense[11];   //  对应试剂架上的10种试剂
	uint8_t index1 = 0;
	char *lst_k = NULL;
	
	unsigned int now_time = 0;
	
	if (mainarm_finished )
		return;	

	memcpy(dispense, dispenseIN, sizeof(dispense_t)*11);

	if (last_reagent == &last_reagentA)
	{ 
		head_step_p = (uint32_t*)&workstep_state_a;
		temper_control = temper_control1;
		operate_head = operate_head1;
		lst_k = (char*)&lastt_kind[0][0];
	}
	if (last_reagent == &last_reagentB)
	{ 
		head_step_p = (uint32_t*)&workstep_state_b;
		temper_control = temper_control2;
		operate_head = operate_head2;
		lst_k = (char*)(&lastt_kind[1][0]);
	}
	if (last_reagent == &last_reagentC)
	{ 
		head_step_p = (uint32_t*)&workstep_state_c;
		temper_control = temper_control3;
		operate_head = operate_head3;
		lst_k = (char*)(&lastt_kind[2][0]);
	}
		
	if (dispense[0].plate_num % 30 < 10)
	{
		reagent_lock_num = 0xFE;
		CriticalWork_num = 0x7F;
		shelf_num = 0;
	}
	else if (dispense[0].plate_num % 30 < 20)
	{
		reagent_lock_num = 0xFD;
		CriticalWork_num = 0xBF;
		shelf_num = 1;	
	}
	else if (dispense[0].plate_num % 30 < 30)
	{
		reagent_lock_num = 0xFB;
		CriticalWork_num = 0xDF;
		shelf_num = 2;
	}

	printf("dispenseB[0].reagent=%d dispenseC[0].reagent=%d operate_head2->operate.reagent=%d operate_head3->operate.reagent=%d\n",
		dispenseB[0].reagent, dispenseC[0].reagent, operate_head2->operate.reagent, operate_head3->operate.reagent);

	while(dispense[i].reagent != STOP_OPERATE)
		i++;

	pthread_mutex_lock(&mutex_mianarmlock);

	//清洗步骤
		//	mb_probewash_c(WATERPORT); //test
		
	//同一架不同修复液滴加顺序优化  =====================================================================
	#if(1 == USE_LOG_INFO)
	lprintf(log_my, INFO, "last_cabin_reagent[%c]=%d, dispense[0].reagent=%d\n", (shelfindex==1?'A':(shelfindex==2?'B':(shelfindex==3?'C':'N'))), last_cabin_reagent, dispense[0].reagent);
	#endif
	if (dispense[0].reagent >= REAGENT_ER1 && last_cabin_reagent != dispense[0].reagent )
	{
		i=0;
		while(i < 11 && dispense[i].reagent != STOP_OPERATE)
		{
			#if(1 == USE_LOG_INFO)
			lprintf(log_my, INFO, "dispense[%d].reagent=%d plate=%d\n", dispense[i].reagent, i, dispense[i].plate_num);
			#endif
			i++;
		}

		if (last_cabin_reagent == dispense[--i].reagent )
		{	
			while( (dispense[index1].reagent != last_cabin_reagent) )
			{	
				index1++;
			}
			//		index1--;
			i++;
			memcpy(	&dispense_change[0][0], &dispense[0], sizeof(dispense_t) * (index1 ));
			memcpy(	&dispense_change[1][0], &dispense[index1], sizeof(dispense_t) * (i - index1));
			memcpy( &dispense[0], &dispense_change[1][0], sizeof(dispense_t) * (i - index1));
			memcpy( &dispense[i - index1 ], &dispense_change[0][0], sizeof(dispense_t) * (index1 ));
		}
	}
			//=============================================================================================
	i=0;
	while(i < 11 && dispense[i].reagent != STOP_OPERATE)
	{
		printf("dispense[i].reagent=%d plate=%d\n", dispense[i].reagent, dispense[i].plate_num);
		i++;
	}
	#if(USE_LOG_INFO == 1)
	printf("000000dispense[0].plate_num=%d\n", dispense[0].plate_num);
	//	if (  dispense[0].reagent != NO_REAGENT)
	//		mb_procwashprobe(dispense[0].reagent);
	printf("111111111dispense[0].plate_num=%d\n", dispense[0].plate_num);
	printf("dispense[0].reagent = %d *head_step_p=%x\n",dispense[0].reagent, *head_step_p);

	lprintf(log_my, INFO, "000000dispense[0].plate_num=%d\n", dispense[0].plate_num);
	lprintf(log_my, INFO, "111111111dispense[0].plate_num=%d\n", dispense[0].plate_num);
	lprintf(log_my, INFO, "dispense[0].reagent = %d *head_step_p=%x\n",dispense[0].reagent, *head_step_p);
	#endif

	if (dispense[0].reagent == REAGENT_DEWAX && (( (*head_step_p) & 0X0F000000) == 0) )//  脱蜡液先加热
	{	
		while(1)
		{
			for (i = 0; i < 10; i++)
			{
				if (temper_control[i].state == REACH_TEMP || temper_control[i].state == START_TIME)
					break;
			}
			if (i < 10)
				break;

			if (tp_checkmachstoped(mainarm_finished) < 0)
			{
				mainarm_finished = true;
				if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error tp_threadmarm");
				return;
			}
		}					
	}

	if (tp_checkmachstoped(mainarm_finished) < 0)
	{
		mainarm_finished = true;
		if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error tp_threadmarm");
		return;
	}

	if (dispense[0].reagent == NO_REAGENT)//烘烤
	{
		if((dispense[0].plate_num % 30) < 10 )	//将滴加结束信息填入滴加步骤中通知patchtime函数滴加结束开始计时
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_a = workstep_state_a | 0X00100000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
			StartDispenseA = true;
		}
		else if((dispense[0].plate_num % 30) < 20 )
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_b = workstep_state_b | 0X00100000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
			StartDispenseB = true;
		}
		else if((dispense[0].plate_num % 30) < 30 )
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_c = workstep_state_c | 0X00100000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
			StartDispenseC = true;
		}

		*last_reagent = NO_REAGENT;
		mainarm_finished = true;	
		if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
		
		if (tp_checkmachstoped(mainarm_finished) < 0)
		{
			mainarm_finished = true;
		//	pthread_mutex_unlock(&tp_threadmarm);
			return;
		}
		return;
	}

	mb_procwashprobe(dispense[0].reagent);   //大容量的先转换

	/****************拉伸位置**********************/

	#ifdef LAIYUE

	printf("dispense[0].reagent=%d *last_reagent=%d \n", dispense[0].reagent, *last_reagent);

	if(  ( dispense[0].reagent < REAGENT_CASE )// && strcmp(reagent_code[dispense[0].reagent].reagent_kind, "*Hem") != 0)
			|| ( dispense[0].reagent >= REAGENT_DAB) || (dispense[0].reagent != *last_reagent &&
			(dispense[0].reagent == REAGENT_ALCOHOL)))// || dispense[0].reagent == REAGENT_ER1 || dispense[0].reagent == REAGENT_ER2 //拉开彻底排液
		/*		(dispense[0].reagent >= REAGENT_CASE && dispense[0].reagent != *last_reagent && (*last_reagent != REAGENT_ALCOHOL //
			&& *last_reagent != NO_REAGENT  && *last_reagent != REAGENT_ER1  && *last_reagent != REAGENT_ER2 && (strcmp(lst_k, "*Hem") != 0)
			|| (dispense[0].reagent == REAGENT_ER1|| dispense[0].reagent == REAGENT_ER2)) ) )
			*/
	{	
		if (bs_packetshelfstreach(90 + shelf_num * 10, &cmd) >= 0)
		{
			if (tp_checkmachstoped(mainarm_finished) < 0)
			{
				mainarm_finished = true;
				if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
					lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
				return;
			}
			pthread_mutex_lock(&mutex_mneedworked);
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head, cmd);	
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");

			while((mini_work_finished))  // 确保miniBoard线程已置FALSE
			{
				usleep(1000);
			}
			sleep(3);
			while(!(mini_work_finished) || mini_recieve_code_all[shelf_num] != 0x5E)//动作完成
			{
				usleep(200000);
			}
			pthread_mutex_unlock(&mutex_mneedworked);		
		}
	}
	#endif

	printf("222222dispense[0].plate_num=%d\n", dispense[0].plate_num);
		
	if (bs_packetshelfstreach(dispense[0].plate_num, &cmd) >= 0)
	{
		if (tp_checkmachstoped(mainarm_finished) < 0)
		{
			mainarm_finished = true;
			if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
			return;
		}
		pthread_mutex_lock(&mutex_mneedworked);
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);	
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");

		while((mini_work_finished))//确保miniBoard线程已置FALSE
		{
			usleep(1000);
		}
		sleep(1);
		while(!(mini_work_finished) || mini_recieve_code_all[shelf_num] != 0x5E)//动作完成
		{
			usleep(200000);
		}
		pthread_mutex_unlock(&mutex_mneedworked);
	}

	if (tp_checkmachstoped(mainarm_finished) < 0)
	{
		mainarm_finished = true;
		if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
		return;
	}

	mb_shelfnumdispenseval(dispense, liquid_val);
	if (tp_checkmachstoped(mainarm_finished) < 0)
	{
		mainarm_finished = true;
		if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
		return;
	}

	if (dispense[0].plate_num >= 90)// || dispense[0].reagent == REAGENT_ER1 || dispense[0].reagent == REAGENT_ER2)//拉伸全部的马上推回来防止干片
	{
		bs_packetshelfstreach(dispense[0].plate_num % 30 ,&cmd);//初始位置l位置
		pthread_mutex_lock(&mutex_mneedworked);
		pthread_mutex_lock(&mutex_mlock);
		set_minicmd(pcmd_head,cmd);
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");
		
		while((mini_work_finished))//确保miniBoard线程已置FALSE
		{
			usleep(1000);
		//	printf("there");
		}
		sleep(1);
		while(!(mini_work_finished))//动作完成
		{
			usleep(200000);
			//printf("there2");
		}
		pthread_mutex_unlock(&mutex_mneedworked);
	}

	if (!big_version || dispense[0].reagent < REAGENT_CASE || dispense[0].reagent >= REAGENT_DAB
		|| ( dispense[0].reagent != dispense[--i].reagent))//不同修复液
	{
		#if(USE_LOG_INFO == 1)
		printf(" shelf che!!!!!,dispense[0].reagent=%d\n",dispense[0].reagent);
		lprintf(log_my, INFO," shelf che!!!!!,reagent=%d\n",dispense[0].reagent);
		#endif
		if((dispense[0].plate_num % 30) < 10 )	//将滴加结束信息填入滴加步骤中通知patchtime函数滴加结束开始计时
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_a = workstep_state_a | 0X00100000;
			if(pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
			if (hydrateA.flage)
				hydrateA.start_time = now.tv_sec;
		}
		else if((dispense[0].plate_num % 30) < 20 )
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_b = workstep_state_b | 0X00100000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
			if (hydrateB.flage)
				hydrateB.start_time = now.tv_sec;
		}
		else if((dispense[0].plate_num % 30) < 30 )
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_c = workstep_state_c | 0X00100000;
			if (pthread_mutex_unlock(&head_step_lock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
			if (hydrateC.flage)
				hydrateC.start_time = now.tv_sec;
		}
	}
				
	if (*last_reagent < REAGENT_CASE || *last_reagent >= REAGENT_DAB
			|| *last_reagent == REAGENT_ER1 || *last_reagent == REAGENT_ER2)  //说明这次滴加的是关键试剂将对应标志清零
		CriticalWork &= CriticalWork_num;

	if ( dispense[0].reagent == REAGENT_DAB	&& *last_reagent == REAGENT_DAB)
	{
		if (dispense[0].plate_num % 30 < 10)
		{
			if (mb_findoperatedabkind(operate_head1,1) != 1)
			{
				if (Array_mixed_DAB1[0] == ordArray_mixed_DAB[0][0])
					mixstation_clear_state[0].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[1].AFTER_DISPENSE = true;
			}
		}
		else if (dispense[0].plate_num % 30 < 20)
		{
			if (mb_findoperatedabkind(operate_head2,1) != 1)
			{
				if (Array_mixed_DAB2[0] == ordArray_mixed_DAB[2][0])
					mixstation_clear_state[2].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[3].AFTER_DISPENSE = true;
			}
		}
		else if (dispense[0].plate_num % 30 < 30)
		{
			if (mb_findoperatedabkind(operate_head3,1) != 1)
			{
				if (Array_mixed_DAB3[0] == ordArray_mixed_DAB[4][0])
					mixstation_clear_state[4].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[5].AFTER_DISPENSE = true;
			}
		}
	}
	if ( dispense[0].reagent == REAGENT_SECEND	&& *last_reagent == REAGENT_SECEND)
	{
		if (dispense[0].plate_num % 30 < 10)
		{
			if (mb_findoperatedabkind(operate_head1,2) != 2)
			{
				if (Array_mixed_RED1[0] == ordArray_mixed_DAB[0][0])
					mixstation_clear_state[0].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[1].AFTER_DISPENSE = true;
			}
		}
		else if (dispense[0].plate_num % 30 < 20)
		{
			if (mb_findoperatedabkind(operate_head2,2) != 2)
			{
				if (Array_mixed_RED2[0] == ordArray_mixed_DAB[2][0])
					mixstation_clear_state[2].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[3].AFTER_DISPENSE = true;
			}
		}
		else if (dispense[0].plate_num % 30 < 30)
		{
			if (mb_findoperatedabkind(operate_head3,2) != 2)
			{
				if (Array_mixed_RED3[0] == ordArray_mixed_DAB[4][0])
					mixstation_clear_state[4].AFTER_DISPENSE = true;
				else
					mixstation_clear_state[5].AFTER_DISPENSE = true;
			}
		}
	}		
		
	if (dispense[0].reagent >= REAGENT_CASE && dispense[0].reagent < REAGENT_DAB && !big_version)//大容量
			*last_reagent = last_cabin_reagent;
	else
	{
		*last_reagent = dispense[0].reagent;
			if( dispense[0].reagent < REAGENT_CASE )
			strcpy(lst_k, reagent_code[dispense[0].reagent].reagent_kind);
	}
	printf("*last_reagent = %d\n", *last_reagent);
	
	gettimeofday(&now, NULL);
	now_time = now.tv_sec - begin_time.tv_sec;
		
	#ifdef LAIYUE
	if((*last_reagent < REAGENT_CASE || *last_reagent >= REAGENT_DAB))
	{
		if((now_time < operate_head->operate.time)&& (operate_head->operate.time - now_time >= 2*60 ))//小于2分钟不进行搅拌
			shelf_stirtime[shelf_num][0] =  15 + now_time;
		
		printf("operate_head->operate.time =%d now_tim=%d\n",operate_head->operate.time,now_time);
		if((operate_head->operate.time - now_time <= 5*60 ))
		{
			
		}
		else if((operate_head->operate.time - now_time <= 8*60 ))
		{

		}
		else if((operate_head->operate.time - now_time <= 10*60 ))//搅拌2次
		{
			shelf_stirtime[shelf_num][1] =  (operate_head->operate.time - now_time - 2*60 ) / 2 + now_time;
			shelf_stirtime[shelf_num][2] = operate_head->operate.time - 2*60;
			printf("shelf_stirtime[%d][0]=%d \n", shelf_num,shelf_stirtime[shelf_num][0]);
		}
		else if((operate_head->operate.time - now_time <= 20*60 ))//搅拌2次
		{
			shelf_stirtime[shelf_num][1] =  (operate_head->operate.time - now_time - 4*60 ) / 2 + now_time;
			shelf_stirtime[shelf_num][2] = operate_head->operate.time - 4*60;
			printf("shelf_stirtime[%d][0]=%d \n", shelf_num,shelf_stirtime[shelf_num][0]);
		}
		else if((operate_head->operate.time - now_time <= 30*60 ))//搅拌2次
		{
			shelf_stirtime[shelf_num][1] =  (operate_head->operate.time - now_time - 8*60 ) / 2 + now_time;
			shelf_stirtime[shelf_num][2] = operate_head->operate.time - 8*60;
			printf("shelf_stirtime[%d][0]=%d \n", shelf_num,shelf_stirtime[shelf_num][0]);
		}		
	}
	#endif
	//	if(strcmp(lst_k, "*Hem")==0 && *last_reagent == REAGENT_WASH)
	//			shelf_stirmode[shelf_num] = 1;
	
	if (dispense[0].reagent < REAGENT_CASE)
	{
		//结束全部占用
		reagent_lock[0][shelf_num] = 0;
		reagent_lock[1][shelf_num] = 0;
		reagent_lock[2][shelf_num] = 0;
		reagent_lock[3][shelf_num] = 0;
		
		if(dispense[0].reagent < 9 )
		{
			reagent_lock_num1 &= reagent_lock_num;
			if( reagent_lock_num1 == 0)
				TRY_REAGENT_UNLOCK1 = true;	
		}
		else if(dispense[0].reagent < 18)
		{
			reagent_lock_num2 &= reagent_lock_num;
			if( reagent_lock_num2 == 0)
				TRY_REAGENT_UNLOCK2 = true;
		}
		else if(dispense[0].reagent < 27)
		{
			reagent_lock_num3 &= reagent_lock_num;
			if( reagent_lock_num3 == 0)
				TRY_REAGENT_UNLOCK3 = true;
		}
		else if(dispense[0].reagent < 36)
		{
			reagent_lock_num4 &= reagent_lock_num;
			if( reagent_lock_num4 == 0)
				TRY_REAGENT_UNLOCK4 = true;
		}
		/***防止终止操作后，滴加操作运行后又锁住玻片架重新进行解锁***/	
	}

	if (operate_head->operate.reagent == STOP_OPERATE)//等吹风结束
	{
		while (fan[shelf_num].NORMAL_FAN != false)
			sleep(1);
	}

	mainarm_finished = true;	
	
	if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error tp_threadmarm");	
}



void tp_thread_can(void *arg)
{
    uint8_t *precvframe = (uint8_t *)&recvframe;
    const int can_frame_len = sizeof(S_CanFrame); 
	int ret = 0,i;
	S_CanFrame sendframeACK;
	
	//sendframeACK.can_id = (send_frame_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
	sendframeACK.can_id = 0;
	sendframeACK.can_dlc = 8;
	sendframeACK.data[1] = 1;
	sendframeACK.data[2] = 0;
	sendframeACK.data[3] = 0X40;
	while (1)
	{
		memset(precvframe, 0x00, can_frame_len);
		ret = can_recvframe(recv_socket_fd, precvframe, can_frame_len, 10);
		if (ret > 0)
		{
			#if(USE_PRINT_LOG == 1)
			can_printframe(recvframe.can_id & CAN_EFF_MASK, recvframe.data, recvframe.can_dlc, 
				((recvframe.can_id & CAN_EFF_FLAG) ? TRUE : false),
				true, 
				false);
			#endif
			CanReState[ recvframe.data[0]] = recvframe.data[3];

			printf("Canrecvframe");
			for(i = 0; i < 8;i++)
				printf(" %x ",recvframe.data[i]);

			if( recvframe.data[3] == ARM_CABIN_EOR || recvframe.data[3] == ARM_CABIN_RESP || recvframe.data[3] == ARM_CABIN_STATE)
			{
				sendframeACK.data[0] = recvframe.data[0];

				printf("the ACK send to shelf\n");
				for (i = 0;i < 8;i++)
					printf(" %x",sendframeACK.data[i]);

				ret = can_sendframe((const int)send_socket_fd, (const uint8_t *)&sendframeACK, sizeof(sendframeACK));
				#if(USE_PRINT_LOG == 1)
				can_printframe(sendframeACK.can_id & CAN_EFF_MASK, sendframeACK.data, sendframeACK.can_dlc, 
						((sendframeACK.can_id & CAN_EFF_FLAG) ? true : false),
						ret > 0 ? true : false, 
						true);
				#endif
				if(recvframe.data[3] == ARM_CABIN_EOR )
				{
					mb_seterrorcode(SHELFA_ERROR_ARMLOS + recvframe.data[4] - 1 + recvframe.data[0]*50);
				}
				
				if( recvframe.data[3] == ARM_CABIN_STATE)
				{
					if(recvframe.data[4] == 0)//卸载成功
					{
						unload[recvframe.data[0]] = true;
							//ArmCabinCmdList[recvframe.data[0]].cmd = ArmCabinKey;
						if (recvframe.data[0]  == 0)
						{
							if (wkeventA != FREE_WORK && wkeventA != MAINTAIN_WORK && wkeventA != INIT_WORK)
								flg_dischargeremainliquid[0] = true;
						
							wkeventA = FREE_WORK;	//水合结束回到空闲工作状态
							hydrateA.flage = false;		
						//	flg_opwork1ready  = false;
						}
						else if (recvframe.data[0] == 1)
						{
							if (wkeventB != FREE_WORK && wkeventB != MAINTAIN_WORK && wkeventB != INIT_WORK)
								flg_dischargeremainliquid[1] = true;
							wkeventB = FREE_WORK;	//水合结束回到空闲工作状态
							hydrateB.flage = false;	
							//	flg_opwork2ready  = false;
						}
						else if (recvframe.data[0] == 2)
						{
							if (wkeventC != FREE_WORK && wkeventC != MAINTAIN_WORK && wkeventC != INIT_WORK)
								flg_dischargeremainliquid[2] = true;
							wkeventC = FREE_WORK;	//水合结束回到空闲工作状态
							hydrateC.flage = false;	
							//	flg_opwork3ready  = false;
						}			

						SHELF_LOCK_STATE[recvframe.data[0]] = 1;		
						flg_shelfarmsleep[recvframe.data[0] + 1] = true;
					}
					else if(recvframe.data[4] == 1)//装载成功
					{
						// flg_shelfcanindex[recvframe.data[0]] = true;  这条赋值语句跟下面的SetSShelfState()是一样的，所以取消该语句.  andry
						mb_setstatetoshelf(recvframe.data[0], true);   // 装载成功，第几架玻片架需要扫描 andry
						SHELF_LOCK_STATE[recvframe.data[0]] = 2;    // 锁定第几架玻片架  andry
						flg_shelfarmsleep[recvframe.data[0] + 1] = false;   // 取消第几个玻片架的休眠  andry
					}
					else if(recvframe.data[4] == 2)//滴加完成
					{
						printf("recvframe.data[0] =%d flg_opwork3ready=%d\n", recvframe.data[0], flg_opwork3ready);
						pthread_mutex_lock(&head_step_lock);
						if(recvframe.data[0] == 0)
						{
							if(!flg_opwork1ready)//水合的滴加结束
							{
								//	ArmCabinCmdList[0].cmd = ArmCabinKey;
								unload[recvframe.data[0]] = true;
							}
							else
							{
								workstep_state_a = workstep_state_a | 0X00100000;    // 时间到 开始滴加 A 架 andry
								StartDispenseA = true;
							}
						}
						else if(recvframe.data[0] == 1)
						{
							if(!flg_opwork2ready)//水合的滴加结束
							{
								unload[recvframe.data[0]] = true;
								//	ArmCabinCmdList[1].cmd = ArmCabinKey;
							}
							else
							{
								workstep_state_b = workstep_state_b | 0X00100000;
								StartDispenseB = true;
							}
						}
						else if(recvframe.data[0] == 2)
						{
						
							if(!flg_opwork3ready)//水合的滴加结束
							{
								unload[recvframe.data[0]] = true;
							//	ArmCabinCmdList[2].cmd = ArmCabinKey;
							}
							else
							{
								workstep_state_c = workstep_state_c | 0X00100000;
								StartDispenseC = true;
							}
						}
						if (pthread_mutex_unlock(&head_step_lock) != 0)
							lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");	
					}
				}
				else if( recvframe.data[3] == WASH_STATE)//取消 自然流到废液收集瓶，定时排液
				{
					if(recvframe.data[4] == 0)//无需清洗
					{

					}
					else if(recvframe.data[4] == 1)//需清洗
					{

					}
				}
			}
		}
	}
}


void tp_cabina(void *arg)	
{
	char cmdbuffer[10] ={0};

	ArmCabin_t cmd;
	unsigned char shelf = 0, i = 0;
	
	printf("tp_cabina!!!!!!!\n");
	cmdbuffer[0] =1;
	cmdbuffer[1] =0;

	for(i = 0; i < 10;i++)
	{
		cmd.dispense_pos = i;
		if(i<5)
		{
			cmd.pos_datalo =*( (char*) &load_motor[i]);
			cmd.pos_datahi = *( (char*) &load_motor[i] + 1);
		}
		else
		{
			cmd.pos_datalo =*( (char*) &arm_motor[i-5]);
			cmd.pos_datahi = *( (char*) &arm_motor[i-5] + 1);
		}
		
		SendCmdArmCtlCabin(ArmCabinPar, shelf, &cmd.init_kind);
	}

	while(1)
	{
		if(unload[shelf])
		{
			unload[shelf] = FALSE;
			ArmCabinCmdList[shelf].cmd = ArmCabinKey;
		}
	
		if(ArmCabinCmdList[shelf].cmd != ArmCabinNo)
		{	
			if(ArmCabinCmdList[shelf].cmd == ArmCabinKey)//接受到answer发送ack给下位机响应时间后发送命令
				usleep(200000);
			
			ArmCabinWork_finished[shelf] = FALSE;
			SendCmdArmCtlCabin(ArmCabinCmdList[shelf].cmd, shelf, &ArmCabinCmdList[shelf].init_kind);
			ArmCabinCmdList[shelf].cmd = ArmCabinNo;
			ArmCabinWork_finished[shelf] = TRUE;		
		}
		
		usleep(100000);
	}
		//	tp_threadmarm(&flg_mianarmstop_a,dispenseA,&mini_work_finished[0],liquid_valA,&last_reagentA);
}


void tp_cabinb(void *arg)
{
	char cmdbuffer[10] ={0};

	ArmCabin_t cmd;
	unsigned char shelf = 1, i = 0;

	printf("tp_cabinb!!!!!!!\n");
	cmdbuffer[0] =1;
	cmdbuffer[1] =0;

	for(i = 0; i < 10;i++)
	{
		cmd.dispense_pos = i;
		if(i<5)
		{
			cmd.pos_datalo =*( (char*) &load_motor2[i]);
			cmd.pos_datahi = *( (char*) &load_motor2[i] + 1);
		}
		else
		{
			cmd.pos_datalo =*( (char*) &arm_motor2[i-5]);
			cmd.pos_datahi = *( (char*) &arm_motor2[i-5] + 1);
		}
		
		SendCmdArmCtlCabin(ArmCabinPar, shelf, &cmd.init_kind);
	}

	while(1)
	{		
		if(unload[shelf])
		{
			unload[shelf] = FALSE;
			ArmCabinCmdList[shelf].cmd = ArmCabinKey;
		}
		if(ArmCabinCmdList[shelf].cmd != ArmCabinNo)
		{	
			if(ArmCabinCmdList[shelf].cmd == ArmCabinKey)//接受到answer发送ack给下位机响应时间后发送命令
				usleep(200000);
			
			ArmCabinWork_finished[shelf] = FALSE;
			SendCmdArmCtlCabin(ArmCabinCmdList[shelf].cmd, shelf, &ArmCabinCmdList[shelf].init_kind);
			ArmCabinCmdList[shelf].cmd = ArmCabinNo;
			ArmCabinWork_finished[shelf] = TRUE;
			
		}
		
		usleep(100000);
	}
//	tp_threadmarm(&flg_mianarmstop_a,dispenseA,&mini_work_finished[0],liquid_valA,&last_reagentA);
}

void tp_cabinc(void *arg)
{
	char cmdbuffer[10] ={0};

	ArmCabin_t cmd;
	unsigned char shelf = 2, i = 0;
	
	printf("tp_cabinc!!!!!!!\n");
	cmdbuffer[0] =1;
	cmdbuffer[1] =0;

	for(i = 0; i < 10;i++)
	{
		cmd.dispense_pos = i;
		if(i<5)
		{
			cmd.pos_datalo =*( (char*) &load_motor3[i]);
			cmd.pos_datahi = *( (char*) &load_motor3[i] + 1);
		}
		else
		{
			cmd.pos_datalo =*( (char*) &arm_motor3[i-5]);
			cmd.pos_datahi = *( (char*) &arm_motor3[i-5] + 1);
		}
		
		SendCmdArmCtlCabin(ArmCabinPar, shelf, &cmd.init_kind);
	}

	while(1)
	{
		if(unload[shelf])
		{
			unload[shelf] = FALSE;
			ArmCabinCmdList[shelf].cmd = ArmCabinKey;
		}
		if(ArmCabinCmdList[shelf].cmd != ArmCabinNo)
		{	
			if(ArmCabinCmdList[shelf].cmd == ArmCabinKey)//接受到answer发送ack给下位机响应时间后发送命令
				usleep(200000);
			
			ArmCabinWork_finished[shelf] = FALSE;
			SendCmdArmCtlCabin(ArmCabinCmdList[shelf].cmd, shelf, &ArmCabinCmdList[shelf].init_kind);
			ArmCabinCmdList[shelf].cmd = ArmCabinNo;
			ArmCabinWork_finished[shelf] = TRUE;		
		}
		
		usleep(100000);
	}
	//	tp_threadmarm(&flg_mianarmstop_a,dispenseA,&mini_work_finished[0],liquid_valA,&last_reagentA);
}



void tp_cabinlat(void *arg)
{

}


//cmd = 0,电源箱温度
//cmd = 1,休眠控制字
int xiumianctl(char cmd )
{
//	short set_temp = 500,rdata;
	
	/*
	pthread_mutex_lock(&mutex_modbuslock);
		
		tp_sendtemperframe(30, cmd, r_w,set_temp,&rdata);
	pthread_mutex_unlock(&mutex_modbuslock);
*/

return 0;
}




extern int iSerialFd;
void tp_thread_tempernew(void *arg)
{
	int i = 0,j=0,tem_count = 0,cmp_cnt = 0,is_alltemp_zero = 0;
	emTemper_cmd cmd = NOTEMPERCMD;
	stTemper_cmdframeunion temcmd;
	short temvalue = 0;
	char error_seted_num[30] = {0};
	char error_cnt[30] = {0};
    unsigned char tem_cnt = 0;
	struct timeval now;
	bool NEED_CTL[3] = {false};

	#if( USE_LOG_INFO == 1)
	printf("=====================dowork_temper_control_controlmy==================>thread_id = %d.\n\n", thread_id_temper = (int)pthread_self());
	#endif

	if( eMBInit( MB_RTU, 0x01, TEM_PORT, 9600, MB_PAR_NONE ) != MB_ENOERR ); //    printf("%s: can't initialize modbus stack!\n", stderr);

	if ( eMBEnable(  ) == MB_ENOERR )
	{
		while(wkevent != INIT_WORK && wkevent != MAINTAIN_WORK) sleep(1);  //wait for init start
		
		#if NEED_INIT
			if (!tb_temperinitnew())
				return;
		#endif

		printf("begin temoer\n");

		TEM_init_work_finished = true;

		while(!flg_mainproexit)
		{
			usleep(10000);
			pthread_mutex_lock(&mutex_temlock);
			cmd = tc_readframtemp();
			if (pthread_mutex_unlock(&mutex_temlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error temlock");

			pthread_mutex_lock(&mutex_miantianlock);
			temcmd = tc_testreadtcmdtemp();
			if (pthread_mutex_unlock(&mutex_miantianlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mutex_miantianlock");

			if (temcmd.devaddr == 9 && temcmd.chopcmd == CONFIG_TEMPER)
			{}
			else if (temcmd.chopcmd != NOTEMPERCMD)
			{}
			
			if (cmd != NOTEMPERCMD)
			{
				if(cmd == CONFIG_TEMPER)	//由于配置时间过长，只在仪器装机阶段使用，调试阶段直接发命令
				{
					
				}
				if (cmd == START_CONTROL1)
				{			
					NEED_CTL[0] = true;	

					is_alltemp_zero = 0;
					for (i = 0; i < 10; i++)
					{
						printf(" %d ", temper_control1[i].temp);

						if (temper_control1[i].temp > is_alltemp_zero)
							is_alltemp_zero = temper_control1[i].temp;

						temper_control1[i].plate_num %= 10; 
					}
					if (is_alltemp_zero == 0) 
					{
						//	TemperControl_p[0] = NULL;
						//	flg_temperheating1 = FALSE;		
					}
					else
					{
						gettimeofday(&now, NULL);
						for(i = 0; i < 10; i++)
						{
							if (temper_control1[i].temp == TMP_ZERO)
							{
								if (last_reagentA == NO_REAGENT || last_reagentA == REAGENT_DEWAX)		
									workstep_state_a = workstep_state_a | 0X01000000;
								temper_control1[i].state = REACH_TEMP;
							//	temper_control1[i].time += now.tv_sec - begin_time.tv_sec;
							}
						}
					}	
				}
				if (cmd == START_CONTROL2)
				{
					NEED_CTL[1] = true; 
					
					is_alltemp_zero = 0;
					printf("START_CONTROL2\n");
					for (i = 0; i < 10; i++)
					{
						printf(" %d ", temper_control2[i].temp);
						if (temper_control2[i].temp > is_alltemp_zero)
							is_alltemp_zero = temper_control2[i].temp;

						temper_control2[i].plate_num %= 10; 
					}
					if (is_alltemp_zero == 0) 
					{
						//	TemperControl_p[1] = NULL;
						//	flg_temperheating2 = FALSE;		
					}
					else
					{
						gettimeofday(&now, NULL);
						for(i = 0; i < 10; i++)
						{
							if (temper_control2[i].temp == TMP_ZERO)
							{
							if (last_reagentB == NO_REAGENT || last_reagentB == REAGENT_DEWAX)		
									workstep_state_b = workstep_state_b | 0X01000000;
								temper_control2[i].state = REACH_TEMP;
								//	temper_control2[i].time += now.tv_sec - begin_time.tv_sec;			
							}	
						}
					}
				}
				if (cmd == START_CONTROL3)
				{
					NEED_CTL[2] = true; 
					
					is_alltemp_zero = 0;
					printf("temper_control3[i].temp=");
					for (i = 0; i < 10; i++)
					{
						printf(" %d ", temper_control3[i].temp);
						if (temper_control3[i].temp > is_alltemp_zero)
							is_alltemp_zero = temper_control3[i].temp;

						temper_control3[i].plate_num %= 10; 
					}
					if (is_alltemp_zero == 0) 
					{
						//	TemperControl_p[2] = NULL;
						//	flg_temperheating3 = false;		
					}
					else
					{
						gettimeofday(&now, NULL);
						for(i = 0; i < 10; i++)
						{
							if (temper_control3[i].temp == TMP_ZERO)
							{
							if (last_reagentC == NO_REAGENT || last_reagentC == REAGENT_DEWAX)		
									workstep_state_c = workstep_state_c | 0X01000000;
								temper_control3[i].state = REACH_TEMP;
								//		temper_control3[i].time += now.tv_sec - begin_time.tv_sec;
							}	
						}

					}
				}
				if (cmd == STOP_CONTROL1)
				{					
					for(i = 0; i < 10;i++)
					{
						tp_sendtemperframe(i, SET, 1, 0,&temvalue);
					}
					flg_temperheating1 = false;
					TemperControl_p[0] = NULL;		
					NEED_851 = false;
				}
				if (cmd == STOP_CONTROL2)
				{						
					for(i = 10; i < 20;i++)
					{
						tp_sendtemperframe(i, SET, 1, 0,&temvalue);
					}
					flg_temperheating2 = false;
					TemperControl_p[1] = NULL;		
					NEED_852 = false;
				}
				if (cmd == STOP_CONTROL3)
				{						
					for(i = 20; i < 30;i++)
					{
						tp_sendtemperframe(i, SET, 1, 0,&temvalue);
					}
					flg_temperheating3 = false;
					TemperControl_p[2] = NULL;	
					NEED_853 = false;
				}									
			}  // end of  	if (cmd != NOTEMPERCMD)
			else
			{
				//		printf("no temcmd\n");
			}

			for(i = 0; i < 3;i++)
			{
				if(NEED_CTL[i])
				{
					for(j=0; j<10; j++)
						tp_tempctrnew(i*10, j, NEED_CTL[i]);
				}
			}
			
			// 0.1S
			if (!flg_intosleep && tem_count > 10)	//温度值不用返回pc可适当加快更新速度
			{
				//实时更新temp_value值
				tp_sendtemperframe(tem_cnt, CUR1, 0, 0, (int16_t*)&temp_value[tem_cnt]);

				cmp_cnt++;
			
				//	if ( ( temp_value[i] <= 0 || (temp_value[i] > tem_limit && temp_value[i] <10000)  && error_seted_num[i] == 0)
				if ( ( temp_value[tem_cnt] <= 0 || (temp_value[tem_cnt] > tem_limit ) ) && error_seted_num[tem_cnt] == 0)
				{
					if(  error_cnt[tem_cnt]++ > 3)//连续3次报错
					{
						error_cnt[tem_cnt] = 0;
			
						temp_value[tem_cnt] = 0;//PC程序需要
						mb_seterrorcode(tem_cnt + TEMPER1);
						error_seted_num[tem_cnt] = 1;
						tp_sendtemperframe(tem_cnt, RUN_STOP, 1, 1, (int16_t*)&temp_value[tem_cnt]);
		
						printf("temp error!!!!!!!!!!\n");
							//	sleep(10);//冷却时间
						tp_sendtemperframe(tem_cnt, CUR2, 0, 0, (int16_t*)&temp_value[tem_cnt]);
						printf("temp error!!!!!!!!!!CUR2=%d\n",temp_value[tem_cnt]);
						sleep(3);//转换通道后后第一个值不是实际值序等待采样后的第二个值
						tp_sendtemperframe(tem_cnt, CUR2, 0, 0, (int16_t*)&temp_value[tem_cnt]);
						printf("temp error!!!!!!!!!!CUR22=%d\n",temp_value[tem_cnt]);
						if ( temp_value[tem_cnt] < 0 || (temp_value[tem_cnt] > tem_limit))//取比较传感器值还是错误值说明加热不受控制关闭电源
						{
							tp_sendtemperframe(tem_cnt, CUR2, 0, 0, (int16_t*)&temp_value[tem_cnt]);
							printf("temp error!!!!!!!!!!CUR23=%d\n",temp_value[tem_cnt]);
							sleep(3);//转换通道后后第一个值不是实际值序等待采样后的第二个值
							tp_sendtemperframe(tem_cnt, CUR2, 0, 0, (int16_t*)&temp_value[tem_cnt]);
							printf("temp error!!!!!!!!!!CUR24=%d\n",temp_value[tem_cnt]);
							if ( (temp_value[tem_cnt] < 0 || (temp_value[tem_cnt] > tem_limit)) && temp_value[tem_cnt] < 2000)
							{
								if (tem_cnt < 10 && tem_over_load)
								{
								//	GPIO_OutClear(TEM_KEYA);
								//	tem_over_loadA = 1;
								}
								else if (tem_cnt < 20 && tem_over_load)
								{
								//	GPIO_OutClear(TEM_KEYB);
								//	tem_over_loadB = 1;
								}
								else if (tem_cnt < 30 && tem_over_load)
								{
								//		GPIO_OutClear(TEM_KEYC);
								//		tem_over_loadC = 1;
								}
								temp_value[tem_cnt] = 0;//PC程序需要
								mb_seterrorcode(tem_cnt + TEMPER1_EM);
							}
						}
							
					}
				}
				else
				{
					error_cnt[tem_cnt] = 0;
				}			

				if (++tem_cnt >= 30)
				{
					tem_cnt = 0;
					printf("get tmpervalue\n");	
					for (i = 0; i < 30; i++)
					{
						printf(" %d ", temp_value[i]);
					}

					get_temvale = true;
				}
				tem_count=0;
		
			}
			tem_count++;
			//		printf("NOTEMCMD = %d ptemcmd_head .temcmd = %dptemcmd_head->next = %d cmd.temcmd = %d/n", NOTEMCMD,ptemcmd_head->temcmd,ptemcmd_head->next, cmd.temcmd);
			//			printf("\n");
		}
	}
	return;
}


void tp_thread_temper(void *arg)
{
	bool res;
	int i = 0, tem_count = 0, cmp_cnt = 0, is_alltemp_zero = 0;
	emTemper_cmd cmd;
	stTemper_cmdframeunion temcmd;
	int temvalue = 0;
	unsigned char recieve_tem;
	stTemper_cmdframeunion te_cmd;
	int work_ret = 0;
	char error_seted_num[30] = {0};
	char error_cnt[30] = {0};
	unsigned char tem_cnt = 0;
	struct timeval now;

	#if( USE_LOG_INFO == 1)
	printf("=====================dowork_temper_control_control==================>thread_id = %d.\n\n",\
		 thread_id_temper = (int)pthread_self());
	#endif
	if( eMBInit( MB_RTU, 0x01, COM5, 9600, MB_PAR_NONE ) != MB_ENOERR )
	{
		#if(USE_LOG_INFO == 1)
		lprintf(log_my, ERROR, "modebus inition error.\n");
		#endif
	}

	if ( eMBEnable( ) == MB_ENOERR )
	{			
		while(wkevent != INIT_WORK && wkevent != MAINTAIN_WORK) sleep(1);//wait for init start
		
		#if NEED_INIT
		if (!tb_temperinitnew())
			return;
		#endif
		printf("begin temoer\n");

		TEM_init_work_finished = true;
		while(!flg_mainproexit)
		{
			usleep(10000);
			pthread_mutex_lock(&mutex_temlock);
			cmd = tc_readframtemp();
			if (pthread_mutex_unlock(&mutex_temlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error temlock");

			pthread_mutex_lock(&mutex_miantianlock);
			temcmd = tc_testreadtcmdtemp();
			if (pthread_mutex_unlock(&mutex_miantianlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_miantianlock");

			if (temcmd.devaddr == 9 && temcmd.chopcmd == CONFIG_TEMPER)
			{
				temcmd.devaddr = 0;
				if (tc_sendcmdchanl(22, CH1_Input_Type, &temvalue))
					printf("tc_sendcmdchanl success\n");
				pthread_mutex_lock(&mutex_modbuslock);
				if (tc_temsendcmd(8,CH3_Input_Type, 22)) 
				{
					if (!tc_recivtempframe(&recieve_tem,&temcmd))
					{
						printf("recieve error_code = %x", recieve_tem);	
						res = false;
					}
				}
				else
				{
					printf("tc_temsendcmd failed\n");
					res = false; 
				}
				if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_modbuslock");
				
				for(i = 0; i < 30; i++)
				{
					te_cmd.chopcmd = CH1_MV_High;
					te_cmd.temvalue = par.temp_MV[i];
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&temvalue);
					te_cmd.chopcmd = CH1_Input_Bias;
					te_cmd.temvalue = par.temp_Input_Bias[i];
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&temvalue);
					te_cmd.chopcmd = CH1_Input_Filter;
					te_cmd.temvalue = par.temp_Input_Bias[i];
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&temvalue);
					te_cmd.chopcmd = CH1_HeatingPro_Band;
					te_cmd.temvalue = par.temp_Input_Bias[i];
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&temvalue);
					te_cmd.chopcmd = CH1_HeatingInt_Time;
					te_cmd.temvalue = par.temp_Input_Bias[i];
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&temvalue);
					te_cmd.chopcmd = CH1_HeatingDer_Time;
					te_cmd.temvalue = par.temp_Input_Bias[i];
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&temvalue);
					te_cmd.chopcmd = CH1_Heatingctl_Time;
					te_cmd.temvalue = par.temp_Input_Bias[i];
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&temvalue);		
				}
			
				TEM_init_setting_finished = true;	
			}			
			else if (temcmd.chopcmd != NOTEMPERCMD)
			{
				printf("temcmd.devaddr%d,temcmd.temcmd%d, temcmd.temvalue%d\n", temcmd.devaddr, temcmd.chopcmd, temcmd.temvalue);
				pthread_mutex_lock(&mutex_modbuslock);
				if (tc_temsendcmd(temcmd.devaddr,temcmd.chopcmd, temcmd.temvalue)) 
				{		
					if (!tc_recivtempframe(&recieve_tem,&temcmd))
					{
						printf("recieve error_code = %x", recieve_tem);	
						res = false;
					}
				}
				else
				{
					printf("tc_temsendcmd failed\n");
					res = false;
				}
				if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_modbuslock");
				usleep(100000);
			}
			
			if (cmd != NOTEMPERCMD)
			{
				if(cmd == CONFIG_TEMPER)	//由于配置时间过长，只在仪器装机阶段使用，调试阶段直接发命令
				{
					
				}
				if (cmd == START_CONTROL1)
				{
					TemperControl_p[0] = tb_tempctrola;	

					is_alltemp_zero = 0;
					for (i = 0; i < 10; i++)
					{
						#if(USE_LOG_INFO == 1)
						lprintf(log_my, INFO, " %d ", temper_control1[i].temp);
						printf(" %d ", temper_control1[i].temp);
						#endif

						if (temper_control1[i].temp > is_alltemp_zero)
							is_alltemp_zero = temper_control1[i].temp;

						temper_control1[i].plate_num %= 10; 
					}
					if (is_alltemp_zero == 0) 
					{
						//	TemperControl_p[0] = NULL;
						//	flg_temperheating1 = FALSE;		
					}
					else
					{
						gettimeofday(&now, NULL);
						for(i = 0; i < 10; i++)
						{
							if (temper_control1[i].temp == TMP_ZERO)
							{
								if (last_reagentA == NO_REAGENT || last_reagentA == REAGENT_DEWAX)		
									workstep_state_a = workstep_state_a | 0X01000000;
								temper_control1[i].state = REACH_TEMP;
								//	temper_control1[i].time += now.tv_sec - begin_time.tv_sec;
							}
							else
								temper_control1[i].temp *= 10;
						}
					}
				}
				if (cmd == START_CONTROL2)
				{
					TemperControl_p[1] = tb_tempctrolb;	
					
					is_alltemp_zero = 0;
					for (i = 0; i < 10; i++)
					{
						#if(USE_LOG_INFO == 1)
						lprintf(log_my, INFO, " %d ", temper_control2[i].temp);
						printf(" %d@%d", temper_control2[i].temp,temper_control2[i].time);
						#endif

						if (temper_control2[i].temp > is_alltemp_zero)
							is_alltemp_zero = temper_control2[i].temp;

						temper_control2[i].plate_num %= 10; 
					}
					if (is_alltemp_zero == 0) 
					{
						//	TemperControl_p[1] = NULL;
						//	flg_temperheating2 = FALSE;		
					}
					else
					{
						gettimeofday(&now, NULL);
						for(i = 0; i < 10; i++)
						{
							if (temper_control2[i].temp == TMP_ZERO)
							{
								if (last_reagentB == NO_REAGENT || last_reagentB == REAGENT_DEWAX)		
										workstep_state_b = workstep_state_b | 0X01000000;
									temper_control2[i].state = REACH_TEMP;
									//	temper_control2[i].time += now.tv_sec - begin_time.tv_sec;		
							}
							else
								temper_control2[i].temp *= 10;
						}
					}
				}
				if (cmd == START_CONTROL3)
				{
					TemperControl_p[2] = tb_tempctrolc;
					
					is_alltemp_zero = 0;
					printf("temper_control3[i].temp=");
					
					for (i = 0; i < 10; i++)
					{
						#if(USE_LOG_INFO == 1)
						lprintf(log_my, INFO, " %d ", temper_control3[i].temp);
						printf(" %d@%d", temper_control3[i].temp,temper_control3[i].time);
						#endif

						if (temper_control3[i].temp > is_alltemp_zero)
							is_alltemp_zero = temper_control3[i].temp;

						temper_control3[i].plate_num %= 10; 
					}
					if (is_alltemp_zero == 0) 
					{
						//	TemperControl_p[2] = NULL;
						//	flg_temperheating3 = FALSE;		
					}
					else
					{
						gettimeofday(&now, NULL);
						for(i = 0; i < 10; i++)
						{
							if (temper_control3[i].temp == TMP_ZERO)
							{
								if (last_reagentC == NO_REAGENT || last_reagentC == REAGENT_DEWAX)		
										workstep_state_c = workstep_state_c | 0X01000000;
									temper_control3[i].state = REACH_TEMP;
								//		temper_control3[i].time += now.tv_sec - begin_time.tv_sec;
							}
							else
								temper_control3[i].temp *= 10;
							
						}

					}
				}
				if (cmd == STOP_CONTROL1)
				{					
					tc_sendcmdshelf(0, CH1_SV, &temvalue, 1);
					flg_temperheating1 = false;
					TemperControl_p[0] = NULL;		
					NEED_851 = false;
				}
				if (cmd == STOP_CONTROL2)
				{						
					tc_sendcmdshelf(0, CH1_SV, &temvalue, 2);
					flg_temperheating2 = false;
					TemperControl_p[1] = NULL;		
					NEED_852 = false;
				}
				if (cmd == STOP_CONTROL3)
				{						
					tc_sendcmdshelf(0, CH1_SV, &temvalue, 3);
					flg_temperheating3 = false;
					TemperControl_p[2] = NULL;	
					NEED_853 = false;
				}									
			}
			else
			{
				//		printf("no temcmd\n");
			}
			//	continue;
			for (i = 0;i < 3; i++)	//轮询
			{
				if (TemperControl_p[i] != NULL)
					if ( (work_ret = TemperControl_p[i]() ) > 0)
					{
						if (i < 1)
							GPIO_OutClear(TEM_KEYA);
						else if (i < 2)
							GPIO_OutClear(TEM_KEYB);
						else if (i < 3)
							GPIO_OutClear(TEM_KEYC);
						
						mb_seterrorcode( i * 10 + (work_ret - 1) + TEMPER1_EM);
					}
			}

			// 0.1S
			if (tem_count > 10)	//温度值不用返回pc可适当加快更新速度
			{
				te_cmd.chopcmd = CH1_Present_Value;
				te_cmd.temvalue = 1;
				sm_minisendonecmd(&te_cmd, tem_cnt, (uint8_t*)&temp_value[tem_cnt]);
				cmp_cnt++;

				if ( ( temp_value[tem_cnt] <= 0 || (temp_value[tem_cnt] > tem_limit ))	&& error_seted_num[tem_cnt] == 0)
				{
					if(  error_cnt[tem_cnt]++ > 3)
					{
						error_cnt[tem_cnt] = 0;
						temp_value[tem_cnt] = 0;//PC程序需要
						mb_seterrorcode(tem_cnt + TEMPER1);
						error_seted_num[tem_cnt] = 1;
						te_cmd.chopcmd = CH1_RUN_STOP;
						//	tc_gettemperaddr(&te_cmd,i + 1);
					
						te_cmd.temvalue = 1;
						sm_minisendonecmd(&te_cmd, tem_cnt, &recieve_tem);

						//	sleep(10);//冷却时间
						tc_getcmtempval(tem_cnt + 1, (int32_t*)&temp_value[tem_cnt]);
						if ( temp_value[tem_cnt] < 0 || (temp_value[tem_cnt] > tem_limit))//取比较传感器值还是错误值说明加热不受控制关闭电源
						{
							tc_getcmtempval(tem_cnt + 1, (int32_t*)&temp_value[tem_cnt]);
							if ( (temp_value[tem_cnt] < 0 || (temp_value[tem_cnt] > tem_limit)) && temp_value[tem_cnt] < 2000)
							{
								if (tem_cnt < 10 && !tem_over_load)
								{
									GPIO_OutClear(TEM_KEYA);
									tem_over_loadA = 1;
								}
								else if (tem_cnt < 20 && !tem_over_load)
								{
									GPIO_OutClear(TEM_KEYB);
									tem_over_loadB = 1;
								}
								else if (tem_cnt < 30 && !tem_over_load)
								{
									GPIO_OutClear(TEM_KEYC);
									tem_over_loadC = 1;
								}
								temp_value[tem_cnt] = 0;//PC程序需要
								mb_seterrorcode(tem_cnt + TEMPER1_EM);
							}
						}	
					}
				}			
					
				if (++tem_cnt >= 30)
				{
					tem_cnt = 0;

					printf("get tmpervalue\n");	
					for (i = 0; i < 30; i++)
					{
						printf(" %d ", temp_value[i]);
					}
					for(i = 0; i < 10; i++)
					{
						if( temper_control1[i].state != 0 && temper_control1[i].state < REACH_TEMP)
							break;
					}
					if(i < 10)
					{
						lprintf(log_my, INFO, " tmpervalue1\n");
						for (i = 0; i < 10; i++)
							lprintf(log_my, INFO," %d ",temp_value[i]);

					}
					
					for(i = 0; i < 10; i++)
					{
						if( temper_control2[i].state != 0 && temper_control2[i].state < REACH_TEMP)
							break;
					}
					if(i < 10)
					{
						lprintf(log_my, INFO, " tmpervalue2\n");
						for (i = 0; i < 10; i++)
							lprintf(log_my, INFO," %d ",temp_value[i+10]);

					}
					for(i = 0; i < 10; i++)
					{
						if( temper_control3[i].state != 0 && temper_control3[i].state < REACH_TEMP)
							break;
					}
					if(i < 10)
					{
						lprintf(log_my, INFO, " tmpervalue3\n");
						for (i = 0; i < 10; i++)
							lprintf(log_my, INFO," %d ",temp_value[i+20]);
					}	
					get_temvale = TRUE;
				}
				tem_count=0;

			}
			tem_count++;
		}	
	}
	return;
}



/// @brief andry
/// @param arg 
void tp_thread_monitsersor(void *arg)
{
	printf("tp_Thread_Monitsersor@@@@@@@@@@@@@\n");

	while(!flg_mainproexit)
	{
		usleep(300000);
		mb_checksensorproce();	
	}
}


void tp_thread_otherwork(void *arg)
{
	uint8_t i = 0;
	char xiumian_cnt=0;
	short rdata=0;
	stminibd_sendpacket cmd;
	bool shelfcmdsend = false, sleepcmdsend = false, wakeupcmdsend = false;
	char TCP_Buffer;
	
	while(!flg_mainproexit)
	{
		//	mb_checksensorproce();			
		for(i = 0; i < 3; i++)
		{
			if (flg_normaldispenseliquid[i])
			{
				mb_dischargeshelfwateliquid(i + 1);
				flg_normaldispenseliquid[i] = false;
			}	
			if(flg_dischargeremainliquid[i])
			{
				flg_dischargeremainliquid[i] = false;
				mb_dischargeshelfwateliquid(i + 1);
				sleep(2);
				mb_dischargeshelfwateliquid(i + 1);
				sleep(2);
				mb_dischargeshelfwateliquid(i + 1);
				if ((wkeventA == FREE_WORK || wkeventA == INIT_WORK)&& (wkeventB == FREE_WORK || wkeventB == INIT_WORK)&&
					(wkeventC == FREE_WORK || wkeventC == INIT_WORK) )
					flg_washproblewater = true;
			}
		}
		pthread_mutex_lock(&mutex_tempotherlock);
		mb_getreagentliquitcab(cabin_port);
		if (pthread_mutex_unlock(&mutex_tempotherlock) != 0)
			lprintf(log_my, ERROR,"%s", "pthread_mutex_unlock error otherlock");

		if (!flg_maintianrunning)
		{	
			pthread_mutex_lock(&mutex_tempotherlock);
			//		mb_waterctrl(COM8,WATER_WORK_STATE);
			if (pthread_mutex_unlock(&mutex_tempotherlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error otherlock");
		}

		if (initialize_finished)
		{	
			pthread_mutex_lock(&mutex_tempotherlock);
			//	mb_waterctrl(COM8,WATER_WORK_START);
			if (pthread_mutex_unlock(&mutex_tempotherlock) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error otherlock");
		}
		
		usleep(300000);
		if(new_version) 
		{
			if  (flg_shelfarmsleep[0] == false || flg_shelfarmsleep[1] == false || flg_shelfarmsleep[2] == false || \
					flg_shelfarmsleep[3] == false)
			{
				if(!wakeupcmdsend)
				{
					tp_sendtemperframe(30, 3, 1, 0, &rdata);
					printf("sendnoxiumian!!!!!!!!!!!!! \n");
					wakeupcmdsend = true;
					sleepcmdsend = false;
				}
			}
			if  (flg_shelfarmsleep[0] == true && flg_shelfarmsleep[1] == true &&
				flg_shelfarmsleep[2] == true && flg_shelfarmsleep[3] == true && (xiumian_cnt++ >10))
			{
				if(!sleepcmdsend)
				{
					tp_sendtemperframe(30, 3, 1, 1, &rdata);
					printf("sendxiumian!!!!!!!!!!!!! \n");
					sleepcmdsend = true;
					wakeupcmdsend = false;
				}
			
				xiumian_cnt = 0;
				{
					//读取休眠状态
					tp_sendtemperframe(30, 1, 0, 0, &rdata);
					if(rdata == 1)//已经休眠
					{
						flg_intosleep = true;
						printf("flg_intosleep\n");

						if(shelfcmdsend == false)
						{
							GPIO_OutClear(TEM_KEYA);
							GPIO_OutClear(TEM_KEYB);
							GPIO_OutClear(TEM_KEYC);

							memset(&cmd,0,sizeof(cmd));
							cmd.cmd = 4;
							cmd.minicmd_buffer[3] = 4;//下面整型赋值要覆盖所以从3开始
							cmd.minicmd_num=2;
							cmd.minicmd_buffer[4] =0;
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head,cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");

							shelfcmdsend = true;
							TCP_Buffer = 0;
							nt_sendpacketdata(InstrumentSleep, &TCP_Buffer, 1);
						}
					}
					else
					{
						printf("OUT_XIUMIAN\n");

						if(shelfcmdsend == true)
						{
							GPIO_OutSet(TEM_KEYA);
							GPIO_OutSet(TEM_KEYB);
							GPIO_OutSet(TEM_KEYC);
							memset(&cmd, 0, sizeof(cmd));
							cmd.cmd = 4;
							cmd.minicmd_buffer[3] = 4;//下面整型赋值要覆盖所以从3开始
							cmd.minicmd_num=2;
							cmd.minicmd_buffer[4] = 1;
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head,cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
							
							//	flg_shelfarmsleep[0] = FALSE;
							sleepcmdsend = false;
							wakeupcmdsend = false;
							shelfcmdsend = false;
						}
					
						if(flg_intosleep)
							flg_sleepfordevinit = true;
					}
				}
			}	
		}
	}
}


void tp_thread_beep(void *arg)
{
	while(1)
	{
		mb_beeperproce();
		sleep(3);
	}
}

void tp_thread_fanctrl(void *arg)
{
	unsigned char i = 0;
	
	while(1)
	{
		for(i = 0; i < 3; i++)
		{
			if (fan[i].NORMAL_FAN == true)    
			{
				tp_fanworkproce(i + 1, fan[i].time);
				fan[i].NORMAL_FAN = false;
			}
		}
		usleep(10000);
	}
}





void tp_thread_mainarm(void *arg)
{
	stminibd_sendpacket cmd;
	sr_cmdstruct_t srcmdframe;
	char netbuf;
	uint16_t i = 0;
	
	#if(USE_LOG_INFO == 1)
	printf("===================== tp_thread_mainarm ==================>\n\n");
	#endif

	while(!flg_mainproexit)
	{
		usleep(100);

		if (initialize_finished == TRUE )
		{
			if (wkevent != MAINTAIN_WORK)       //部件校准不使能
			{
				pthread_mutex_lock(&mutex_mianarmlock);
				mb_checkcabinhavereagt(3/* ER2PORT */);     // 初始化结束后不检测ER2PORT大容量桶，其他试剂桶拔插2次就自动灌注 andry
				if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
					lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error tp_threadmarm");
			}
			if (work_cnt > 10)   // 这个在上位机发送试剂参数解析时 ++， 当整个机器规程大于 10 个时清零
			{
				work_cnt = 0;
			}

			if(!flg_mianarmstop_a && ( ((dispenseA[0].reagent) & 0x7f) < REAGENT_CASE)){}
			else if(!flg_mianarmstop_b && ( ((dispenseB[0].reagent) & 0x7f) < REAGENT_CASE)){}
			else if(!flg_mianarmstop_c && ( ((dispenseC[0].reagent) & 0x7f) < REAGENT_CASE)){}
			else  // if( flg_mianarmstop_a == true || (dispendA[0].reagent & 0x7f) >= REAGENT_CASE) 
			{
				for (i = 0; i < 3; i++)
				{
					if (flg_shelfcanindex[i])   // A, B, C 架玻片架反馈回来说应经锁住了，需要扫描  andry
					{	
						if (wkevent == MAINTAIN_WORK)
						{
							memset(&cmd, 0, sizeof(cmd));
							cmd.cmd = RELOAD;
							cmd.minicmd_buffer[3] = (unsigned char)RELOAD;
							cmd.minicmd_buffer[4] = i + 1;
							cmd.minicmd_num = 2;
							printf("RELOAD SETED\n\n\n");

							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head, cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");
							flg_shelfcanindex[i] = FALSE;
						}
						else
						{		
							mb_shelfscanning(i + 1);    //, &mini_recieve_code);   // 扫描玻片架
							flg_shelfcanindex[i] = FALSE;		
						}
					}
				}
			}
			for (i = 0; i < 4; i++)
			{
				if (reagent_check[i].NEED_SCAN)
				{
					if (wkevent == MAINTAIN_WORK)
					{
						reagent_check[i].NEED_SCAN = FALSE;
						GPIO_OutSet(reagent_check[i].lock);
					}
					else
					{
						mb_reagentstationscanner(i + 1);   //添加扫过试剂链表 在次扫描时如 此试剂已经扫过将不再检测改试剂的量
					}
				}
			}
			for (i = 0; i < 36; i++)
			{
				if(flg_checkreagentindex[i])
				{
					mb_reagentchecking(i);
					flg_checkreagentindex[i] = FALSE;
				}
			}

			if (flg_sancmixstatioin)
			{		
				mb_mixstationscanner();
				flg_sancmixstatioin = FALSE;
			}
		}

		//需尽快混合操作
		#if DE_TONG
		if (//( (dispenseA[0].reagent == REAGENT_DAB && !flg_mianarmstop_a ) || operate_head1->operate.reagent == REAGENT_SECEND) 
				( (dispenseB[0].reagent == REAGENT_DAB && !flg_mianarmstop_b ) || operate_head2->operate.reagent == REAGENT_SECEND) ||\
				( (dispenseC[0].reagent == REAGENT_SECEND && !flg_mianarmstop_c ) || operate_head3->operate.reagent == REAGENT_SECEND)
		)
		{

		}
		else			
		#endif
		{
			if ((startmixworkid & 0x80) > 0 )// && !isDAB_mixedA )
			{
				if (mb_dabmixer(operate_head1) == -3)
				{
					mb_mainworkstop(1);
				}
			}
		}
		#if DE_TONG
		if (( (dispenseA[0].reagent == REAGENT_DAB && !flg_mianarmstop_a ) || operate_head1->operate.reagent == REAGENT_SECEND) 
			// ( (dispenseB[0].reagent == REAGENT_DAB && !flg_mianarmstop_b ) || operate_head2->operate.reagent == REAGENT_SECEND) 
			||  ( (dispenseC[0].reagent == REAGENT_SECEND && !flg_mianarmstop_c ) || operate_head3->operate.reagent == REAGENT_SECEND)
		)
		{

		}
		else
		#endif
		{
			if ((startmixworkid & 0X40) > 0)//  && !isDAB_mixedB )
			//if ((startmixworkid & 0X40) > 0  && !isDAB_mixedB && (CriticalWork == 0))
			{
				if (mb_dabmixer(operate_head2) == -3)
				{
					mb_mainworkstop(2);
				}
			}
		}
		#if DE_TONG
		if (( (dispenseA[0].reagent == REAGENT_DAB && !flg_mianarmstop_a ) || operate_head1->operate.reagent == REAGENT_SECEND) ||
				( (dispenseB[0].reagent == REAGENT_DAB && !flg_mianarmstop_b ) || operate_head2->operate.reagent == REAGENT_SECEND) 
			//  ( (dispenseC[0].reagent == REAGENT_DAB && !flg_mianarmstop_c ) || operate_head3->operate.reagent == REAGENT_SECEND)
		)
		{

		}
		else
		#endif

		if ((startmixworkid & 0X20) > 0)   //  && !isDAB_mixedC )
		{
			if (mb_dabmixer(operate_head3) == -3)
			{
				mb_mainworkstop(3);
			}
		}

		//	MainArm_Dispense:
		if ((CriticalWork & 0X80) > 0)
			tp_mainarmbusses((bool)flg_mianarmstop_a, (dispense_t*)dispenseA, (bool)mini_work_finished[0], (const uint32_t)liquid_valA, (uint8_t*)&last_reagentA, 1);
		if ((CriticalWork & 0X40) > 0)
			tp_mainarmbusses((bool)flg_mianarmstop_b, (dispense_t*)dispenseB, (bool)mini_work_finished[1], (const uint32_t)liquid_valB, (uint8_t*)&last_reagentB, 2);
		if ((CriticalWork & 0X20) > 0)
			tp_mainarmbusses((bool)flg_mianarmstop_c, (dispense_t*)dispenseC, (bool)mini_work_finished[2], (const uint32_t)liquid_valC, (uint8_t*)&last_reagentC, 3);

		tp_mainarmbusses((bool)flg_mianarmstop_a, (dispense_t*)dispenseA, (bool)mini_work_finished[0], (const uint32_t)liquid_valA, (uint8_t*)&last_reagentA, 1);
		tp_mainarmbusses((bool)flg_mianarmstop_b, (dispense_t*)dispenseB, (bool)mini_work_finished[1], (const uint32_t)liquid_valB, (uint8_t*)&last_reagentB, 2);
		tp_mainarmbusses((bool)flg_mianarmstop_c, (dispense_t*)dispenseC, (bool)mini_work_finished[2], (const uint32_t)liquid_valC, (uint8_t*)&last_reagentC, 3);

		if (!flg_opwork1ready && !flg_opwork2ready && !flg_opwork3ready)
		{
			for (i = 0; i < 6; i++)				
			{
				if (mixstation_clear_state[i].AFTER_DISPENSE && mixstation_clear_state[i].NEEDCLEAR)
				{
					pthread_mutex_lock(&mutex_mianarmlock);
						
					if (mb_clearmixstation(&ordArray_mixed_DAB[i][0]) < 0)
						mb_seterrorcode(MIX_STATION_CLEARERROR);		
						
					mixstation_clear_state[i].NEEDCLEAR = FALSE;

					uint8_t j = 0;
					for(j = i;j < 6; j++)						
					{
						if (mixstation_clear_state[j].AFTER_DISPENSE && mixstation_clear_state[j].NEEDCLEAR)
						break;
					}
					if (j == 6 || i == 5)
					{
						#if(DIFF_DISCHARGE_L_H == 1)
						if(dispenseA[0].reagent == REAGENT_DAB || dispenseB[0].reagent == REAGENT_DAB || dispenseC[0].reagent == REAGENT_DAB){
							mb_dischwasteliquid_hi(DischargeTime);
						}else{
							mb_dischargwateliquid_lo(DischargeTime);
						}
						#else
						mb_dischargwateliquid_lo(DischargeTime);	
						mb_dischwasteliquid_hi(DischargeTime);	
						#endif

						srcmdframe.srdevaddr = ARM_ADDR;
						sprintf(srcmdframe.cmdbuf, "PI");
						if (sb_armpumpsend(port_arm, (unsigned char*)srcmdframe.cmdbuf, 
							srcmdframe.srdevaddr , 1, 0, 2) > 0)
							printf("[pi]: Send message	to tecan device [%x]successful.\n" , srcmdframe.srdevaddr );
						else
							printf("[pi]: Send message	to tecan device [%x]failed.\n" , srcmdframe.srdevaddr );
						sb_waitingframeaswer(&srcmdframe);
					}
					if (pthread_mutex_unlock(&mutex_mianarmlock) != 0)
						lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error tp_threadmarm");
				}
			}
		}

		if (wkevent == PROBE_CLR_WORK)
		{		
			mb_probectrl();
			wkevent = FREE_WORK;
		}
		if (flg_washproblewater)
		{
			mb_procwashprobe(REAGENT_WATER);		
			flg_washproblewater = FALSE;
			if(new_version) 
			{
				flg_shelfarmsleep[0] = TRUE;
				//发送可休眠状态
				//	tp_sendtemperframe(30, 3, 1,1,&rdata);
			}
		}

		if(new_version) 
		{
			if(flg_sleepfordevinit)		
			{
				flg_sleepfordevinit = FALSE;		
				flg_intosleep = FALSE;
				flg_shelfarmsleep[0] = FALSE;   //苏醒就不能休眠
				netbuf = 1;
				nt_sendpacketdata(InstrumentSleep, &netbuf, 1);
				tp_devinit(1);
			}
		}		
	}
}



void mb_thread_recvscanner(void *arg)
{
	if (honey_scaner)
		sc_listenscanhoney();

	if (new_scaner)
	{
		fd_set 	DeviceRead;
		struct timeval timeout;
		int rel,len,i;
		char ACKBUF[6] = {4,0XD0,4,0,0XFF,0X28};
		const char Buf1[6] = {4,0xE4,4,0,0xFF,0x14};//4 //4 E4 4 0 FF 14
		int fd	= serail_hand[port_scanner];

		printf("[Tp_thread_recvscanner]: Begin to listening Scaner.\n ");	
		tcflush(serail_hand[port_scanner], TCIFLUSH);

		while(!flg_mainproexit)
		{
			timeout.tv_sec = 0;
			timeout.tv_usec = 1000;

			FD_ZERO(&DeviceRead);
			FD_SET(serail_hand[port_scanner], &DeviceRead);

			rel = select(serail_hand[port_scanner]+1, &DeviceRead, NULL, NULL, &timeout);
			if(rel < 0) {
				perror("select error");
				return;
			}
			else
			{
				if (FD_ISSET(serail_hand[port_scanner], &DeviceRead))
				{	
					if ((len = read(serail_hand[port_scanner], ScanBuffer, sizeof(ScanBuffer))) > 0)
					{	
						printf("port_scanner Request read len = %d.: \n", len);
						for (i = 0; i < len; i++)
							printf(" %x ", ScanBuffer[i]);
						printf("\n\n");		
					}
					if (ScanBuffer[1] == 0XD0)
						scanner_acknum = 1;
					else if (ScanBuffer[1] == 0XD1)
						scanner_acknum = 2;	
					else if (IsGetCode)
					{
						printf("OCR recieved IsGetImage=%d\n", IsGetImage);
						memcpy((uint8_t*)scan_OCR_data, ScanBuffer, len);
						scan_OCR_len = len;
						flg_scanerworking = 1;	 
					}
					
					if (IsGetImage && ScanBuffer[1] == 0XD0)
					{
						IsGetImage = false;
						InGetImage = true;
						/*
						usleep(10000);
							if (write(fd, Buf1, sizeof(Buf1)) < 0)
							printf("send scaner activate trigger failed fd= %d\n", fd);
						else
							printf("send scaner activate trigger success fd= %d\n", fd);
						printf("BUF=\n");
							for (i = 0; i < sizeof(Buf1); i++)
							{
								printf(" %x ",Buf1[i]);
							}
						printf("\n");
						*/
					}
					
					if (ScanBuffer[1] == 0XB1)
					{
						if (write(fd, ACKBUF, sizeof(ACKBUF)) < 0)
						printf("send scaner ACK failed fd= %d\n", fd);
							
						printf("send scaner activate trigger success fd= %d\n", fd);
						printf("BUF=\n");
						for (i = 0; i < sizeof(Buf1); i++)
						{
							printf(" %x ",ACKBUF[i]);
						}
						printf("\n");

						if (InGetImage)
						{
							memcpy((uint8_t*)&scan_data[scan_data_len],&ScanBuffer[4],ScanBuffer[0] - 4);
							scan_data_len += ScanBuffer[0] - 4;	
							{
								flg_scannerrecivimage = 1;	
								//S		InGetImage = FALSE;
							}

							if ( (ScanBuffer[3] & 0x2) == 0)
								flg_scannerlastimage = true;
						}	
					}		
					tcflush(serail_hand[port_scanner],TCIFLUSH);
					memset(ScanBuffer, 0x00, sizeof(ScanBuffer));	
				}
			}
		}       //   end of while(!flg_mainproexit)
		return;
	}
	else
	{
		fd_set 	DeviceRead;
		struct timeval timeout;
		int rel = 0,len =0;
		unsigned int image_len = 1028;
		char scan_ack[20] = {0}; 
		
		printf("[Tp_thread_recvscanner]: Begin to listening Scaner.\n ");	

		tcflush(serail_hand[port_scanner],TCIFLUSH);

		while(!flg_mainproexit)
		{	
			timeout.tv_sec = 0;
			timeout.tv_usec = 100000;

			FD_ZERO(&DeviceRead);
			FD_SET(serail_hand[port_scanner], &DeviceRead);
		
			rel = select(serail_hand[port_scanner]+1, &DeviceRead, NULL, NULL, &timeout);
			if(rel < 0) {
				perror("select error");
				return;
			}
			else
			{
				if (FD_ISSET(serail_hand[port_scanner], &DeviceRead))
				{	
					if ((len = read(serail_hand[port_scanner], ScanBuffer, sizeof(ScanBuffer))) > 0)
					{
						/*
						printf("port_scanner Request read len = %d.: \n", len);
						for (i = 0; i < len; i++)
							printf(" %c ", ScanBuffer[i]);
						printf("\n\n");
						*/
					};
					
					memcpy(scan_ack, ScanBuffer, 6);
					if (ScanBuffer[0] == 0x16)	//进行图片数据传输
					{
						tcflush(serail_hand[port_scanner],TCIFLUSH);
						memset(ScanBuffer, 0x00, sizeof(ScanBuffer));			
						
						xmodemReceive(serail_hand[port_scanner],ScanBuffer,image_len);//获取图片数据
						
						//	printf("scan_data_len=%d\n",scan_data_len);
						//	for(i=0;i<scan_data_len;i++)
						//		printf("%c",scan_data[i]);
						IMGSNP_ACK = false;
						flg_scanerworking = 1;	 
					}
					else if(strcmp(scan_ack, "IMGSNP") == 0)
					{
						//	for(i=0;i<len;i++)
						//			printf("%c",ScanBuffer[i]);
						IMGSNP_ACK = true;
					}
					else if (len > 20)
					{
						//	for(i=0;i<len;i++)
						//			printf("%c",ScanBuffer[i]);
					}
					else	//OCR码
					{
						tcflush(serail_hand[port_scanner],TCIFLUSH);
						printf("OCR recieved\n");
						memcpy((uint8_t*)scan_OCR_data, ScanBuffer, len);
						scan_OCR_len = len;
						flg_scanerworking = 1;	 
					}
	
					tcflush(serail_hand[port_scanner],TCIFLUSH);
					memset(ScanBuffer, 0x00, sizeof(ScanBuffer));		
				}
			}
		}
		
		return;
	}
}



