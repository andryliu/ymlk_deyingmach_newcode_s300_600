
#include  <stdio.h>
#include  <stdlib.h>
#include  <string.h> /* strerror() */
#include  <pthread.h>
#include  <unistd.h>
#include <termios.h>
#include <signal.h>
#include "netbuss.h"
#include "log.h"
#include "tpthread.h"
#include "common.h"
#include "netbuss.h"
#include "midbus.h"
#include "temper_control.h"
#include "arm_ctl_cabin.h"
#include "temperctlmy.h"
#include "sys/time.h"
#include "serailbussfile.h"
#include "serailminibd.h"




#define TEST 0


//log_t *log_g;        /*进程全局日志文件句柄，线程内的日志文件句柄可在线程内自行装载，如果交易分日志可根据交易码自行*/
//log_t *log_my[5]; 

extern struct timeval begin_time;
extern volatile unsigned char minicmd_buffer[];
extern volatile unsigned char minicmd_num;
extern volatile unsigned char mini_recieve_code;
extern volatile unsigned char mini_recieve_code2;
extern volatile unsigned char mini_recieve_code3;

extern volatile stminibd_cmdlist* pcmd_head;
extern volatile stminibd_cmdlist* pcmd_head2;
extern volatile stminibd_cmdlist* pcmd_head3;
extern reagentoutside_list * reagentoutside_head;
extern volatile temper_cmd_list* ptemcmd_head;
extern volatile tem_cmd_list* ptem_cmd_head;
extern stnet_reportlist* preport_head;
extern sterrcode_list* perror_head;
extern sterrcode_list* perror_exist_head;
extern pthread_mutex_t mutex_mlock;
extern pthread_mutex_t error_lock;
extern pthread_mutex_t pump_lock;

extern pthread_mutex_t error_exist_lock;
extern unsigned char scan_data[] ;
extern volatile bool inDischarge;
extern pthread_mutex_t mutex_temlock;
extern pthread_mutex_t mutex_miantianlock;
extern pthread_mutex_t minib_send_lock;
extern volatile bool initialize_finished;
extern ewkevent wkevent;

extern volatile unsigned char replace_reagent;
extern volatile int new_scaner, new_temper, honey_scaner, new_version;

static void init_signals(void);
static void init_global_val(void);
static int dev_init(void);
static stthreadpl * init_pool_work(void);



int main(int argc, char *argv[])
{ 
	stthreadpl *pool;

	#if(0 == S600_CONFIG_DIR)

	log_open(&log_my[4], "/usr/libfile/log_1", 0);
	#else

	log_open(&log_my[4], S600log_path, 0);
	#endif

	lprintf(log_my, INFO, "%s\n", "***system start***");
	dev_init();

	printf("[]Run outo dev_init().\n");

	init_signals();

	printf("[]Run outo the inti_signals().\n");

	// nt_createsocket(); //while(1) sleep(1);

	printf("[]run outo the nt_createsocket(). \n");

	init_global_val();

	printf("[]run outo the init_global_val().\n");

	pool = init_pool_work();

	printf("[]run outo the init_pool_work().\n");

	tp_getrpara( );
	#if(USE_LOG_INFO == 1)
	printf("***started S600****\n");	
	lprintf(log_my, INFO, "[****S600 %s ****]\n", "==== changed ====");
	#endif
	
	if (new_scaner)
	{
		sc_scanerinition();		
	}
	
	RESTART_DIVIC:
	wkevent = FREE_WORK;
	initialize_finished = FALSE;
	if (tp_devinit(0) < 0)
		while(!flg_mainproexit) sleep(1);    //等待错误信息发给PC
	
	ct_readstarttime();	

	printf("the device initioned .\n");
	char tim_cnt = 0;
	while(!flg_mainproexit)
	{		
		mb_procework();

		if (wkevent == STANDBY_WORK)
		{
			goto RESTART_DIVIC;
		}
		if (tim_cnt > 10)
		{
			NEED_PRINTF = true;
			tim_cnt++;
			if (tim_cnt > 13)
				tim_cnt = 0;
		}
		else
		{
			tim_cnt++;
			NEED_PRINTF = false;
		}
		usleep(100000);
	}
  
    sleep(5);
	sb_serialclose(5 );
	
	printf("[main]: tp_threaddestry, leave out.\n");

    tp_threaddestry(pool, 1); 
	md_gpiointion();          //关闭所有控制器件
    exit(NULL);
}


static void sig_work(int signum)
{
	// int signa = 0;
	if (signum == SIGBUS)
		write(STDERR_FILENO, "SIGBUS\n", 7);
	else if (signum == SIGSEGV)
		write(STDERR_FILENO, "SIGSEGV\n", 8);
	else if (signum == SIGSYS)
		write(STDERR_FILENO, "SIGSYS\n", 8);

	_exit(EXIT_FAILURE);
}

static void init_signals(void)
{
	sigset_t set;
	struct sigaction act;

	sigfillset(&set);
	sigprocmask(SIG_SETMASK, &set, NULL);
	memset(&act, 0, sizeof(act));
	sigfillset(&act.sa_mask);
	act.sa_handler = SIG_IGN;
	sigaction(SIGHUP, &act, NULL); 
	sigaction(SIGQUIT, &act, NULL);
	sigaction(SIGPIPE, &act, NULL);
	act.sa_handler = sig_work;
	sigaction(SIGTERM, &act, NULL);
	sigaction(SIGBUS, &act, NULL);
	sigaction(SIGFPE, &act, NULL);
	sigaction(SIGILL, &act, NULL);
	sigaction(SIGSEGV, &act, NULL);
	sigaction(SIGSYS, &act, NULL);
	sigemptyset(&set);
	sigprocmask(SIG_SETMASK, &set, NULL);
}


static void init_global_val(void)
{
	uint8_t i = 0;
	
	operate_head1 = (operate_head_list*)malloc(sizeof(operate_head_list));
	operate_head1->next_head = NULL;
	operate_head1->operate.next = NULL;
	operate_head1->operate.reagent = STOP_OPERATE;
	memset(&operate_head1->operate.reagent_info, 0, sizeof(reagent_t));
	operate_head2 = (operate_head_list*)malloc(sizeof(operate_head_list));
	operate_head2->next_head = NULL;
	operate_head2->operate.next = NULL;
	operate_head2->operate.reagent = STOP_OPERATE;
	memset(&operate_head2->operate.reagent_info, 0, sizeof(reagent_t));
	operate_head3 = (operate_head_list*)malloc(sizeof(operate_head_list));
	operate_head3->next_head = NULL;
	operate_head3->operate.next = NULL;
	operate_head3->operate.reagent = STOP_OPERATE;
	memset(&operate_head3->operate.reagent_info, 0, sizeof(reagent_t));
	
	preport_head = (stnet_reportlist*)malloc(sizeof(stnet_reportlist));
	preport_head->next = NULL;
	preport_head->report.num = 0XFF;

	perror_head = (sterrcode_list*)malloc(sizeof(sterrcode_list));
	perror_head->next = NULL;
	perror_head->error_code = NO_ERROR;

	perror_exist_head = (sterrcode_list*)malloc(sizeof(sterrcode_list));
	perror_exist_head->next = NULL;
	perror_exist_head->error_code = NO_ERROR;
	
	pcmd_head = (stminibd_cmdlist*)malloc(sizeof(stminibd_cmdlist));
	pcmd_head->cmd.cmd= NOCMD;
	memset(pcmd_head->cmd.minicmd_buffer, 0, sizeof(pcmd_head->cmd.minicmd_buffer));
	pcmd_head->cmd.minicmd_num = 0;
	pcmd_head->next = NULL;
	
	pcmd_head2 = (stminibd_cmdlist*)malloc(sizeof(stminibd_cmdlist));
	pcmd_head2->cmd.cmd= NOCMD;
	memset(pcmd_head2->cmd.minicmd_buffer, 0, sizeof(pcmd_head2->cmd.minicmd_buffer));
	pcmd_head2->cmd.minicmd_num = 0;
	pcmd_head2->next = NULL;
	
	pcmd_head3 = (stminibd_cmdlist*)malloc(sizeof(stminibd_cmdlist));
	pcmd_head3->cmd.cmd= NOCMD;
	memset(pcmd_head3->cmd.minicmd_buffer, 0, sizeof(pcmd_head3->cmd.minicmd_buffer));
	pcmd_head3->cmd.minicmd_num = 0;
	pcmd_head3->next = NULL;

	ptemcmd_head = (temper_cmd_list*)malloc(sizeof(temper_cmd_list));
	ptemcmd_head->temcmd = NOTEMPERCMD;
	ptemcmd_head->next = NULL;

	ptem_cmd_head = (tem_cmd_list*)malloc(sizeof(tem_cmd_list));
	ptem_cmd_head->cmdframeunion.chopcmd = NOTEMPERCMD;
	ptem_cmd_head->next = NULL;

	reagentoutside_head = (reagentoutside_list*)malloc(sizeof(reagentoutside_list));
	reagentoutside_head->next = NULL;
	memset(&reagentoutside_head->reagent_info, 0, sizeof(reagentoutside_head->reagent_info));
	
	hydrateA.flage = false;
	hydrateB.flage = false;
	hydrateC.flage = false;

	wkeventA = wkeventB = wkeventC = INIT_WORK;

	#if(CAL_TIME_NEWCAL != 1)
	gettimeofday(&begin_time, NULL);
	#endif
	pthread_mutex_init(&head_lock, NULL);
	
	pthread_mutex_init(&error_lock, NULL);
	pthread_mutex_init(&error_exist_lock, NULL);
	pthread_mutex_init(&report_lock, NULL);
	pthread_mutex_init(&netsend_lock, NULL);
	pthread_mutex_init(&pump_lock, NULL);	
	pthread_mutex_init(&head_step_lock, NULL);
	pthread_mutex_init(&armlock_ack, NULL);
	pthread_mutex_init(&armlock_answer, NULL);
	pthread_mutex_init(&minib_send_lock, NULL);

	pthread_mutex_init(&mutex_mlock, NULL);
	pthread_mutex_init(&mutex_mneedworked, NULL);
	pthread_mutex_init(&mutex_mlock2, NULL);
	pthread_mutex_init(&mutex_mlock3, NULL);
	pthread_mutex_init(&mutex_temlock, NULL);
	pthread_mutex_init(&mutex_miantianlock, NULL);
	pthread_mutex_init(&mutex_modbuslock, NULL);
	pthread_mutex_init(&mutex_tempotherlock, NULL);
	pthread_mutex_init(&mutex_washworklock, NULL);
	pthread_mutex_init(&mutex_dischargelock, NULL);
	pthread_mutex_init(&mutex_mianarmlock, NULL);
	pthread_mutex_init(&mutex_critialworklock, NULL);
	pthread_mutex_init(&mutex_tryworklock, NULL);
	pthread_mutex_init(&mutex_mixlock, NULL);
	pthread_mutex_init(&mutex_cabinlock, NULL);
	
	reagent_check[0].lock = CGLOCK1;
	reagent_check[0].NEED_SCAN = false;
	reagent_check[0].sen = CG1;
	reagent_check[1].lock = CGLOCK2;
	reagent_check[1].NEED_SCAN = false;
	reagent_check[1].sen = CG2;
	reagent_check[2].lock = CGLOCK3;
	reagent_check[2].NEED_SCAN = false;
	reagent_check[2].sen = CG3;
	reagent_check[3].lock = CGLOCK4;
	reagent_check[3].NEED_SCAN = false;
	reagent_check[3].sen = CG4;

	for(i =0;i<3;i++)
	{
		reagent_lock[0][i] = 0;
		reagent_lock[1][i] = 0;
		reagent_lock[2][i] = 0;
		reagent_lock[3][i] = 0;
	}
	Array_mixed_DAB1[0] = ordArray_mixed_DAB[0][0]; 
	Array_mixed_DAB1[1] = ordArray_mixed_DAB[0][1]; 
	Array_mixed_DAB2[0] = ordArray_mixed_DAB[2][0]; 
	Array_mixed_DAB2[1] = ordArray_mixed_DAB[2][1]; 
	Array_mixed_DAB3[0] = ordArray_mixed_DAB[4][0]; 
	Array_mixed_DAB3[1] = ordArray_mixed_DAB[4][1]; 

	memset ((uint8_t*)cabin_value, 0XFFFF, 12 * sizeof(short));
	memset ((uint8_t*)need_perfusion, 0, 6 * sizeof(short));
	memset ((uint8_t*)&beep_state, 0, sizeof(beep_state));
	memset (mixstation_clear_state, 0, sizeof(48));
	memset (mix_DAB, 0, sizeof(mix_t) *3);
	memset (mix_SECEND, 0, sizeof(mix_t)*3);

	for(i = 0; i < 10;i++)
		temper_control1[i].state = temper_control2[i].state = temper_control3[i].state = END_TIME;

	memset((uint8_t*)fan, 0, sizeof(fan));
}



static int dev_init(void)
{ 
	//unsigned int i = 0, dwPinState = 0x0ffffc0f;	
	int sendbuf[sizeof(parameter_t) / 4 - 30];

	mb_readconfparatinfo(0);	
	
	port_arm = COM3;
	port_pump = COM8;
	port_scanner = COM4;
	cabin_port = COM9;
	mix_port = COM7;//空串口接受线程兼容
	sb_serailcreat(COM2, 9600, COM_BYTESIZE8, MYPARITY_NONE);
	sb_serailcreat(cabin_port, 9600, COM_BYTESIZE8, MYPARITY_NONE);
	sb_serailcreat(port_arm, 9600, COM_BYTESIZE8, MYPARITY_NONE);
	sb_serailcreat(port_pump, 9600, COM_BYTESIZE8, MYPARITY_NONE);
	sb_serailcreat(port_scanner, 115200, COM_BYTESIZE8, MYPARITY_NONE);
	md_gpiointion( );
	
	#if NORMAL_VER	
	nt_createsocket(); //while(1) sleep(1);
	if(new_version)
	{
		nt_sendpacketdata(PARAMETER, (char*)&par, sizeof(par));
	}
	else
	{
		memcpy(sendbuf, &par, sizeof(parameter_t) - 30 * 6 * 4);
		memcpy(&sendbuf[150], &par.temp_Input_Filter[0], 600);
		nt_sendpacketdata(PARAMETER, (char*)sendbuf, sizeof(sendbuf));
	}		
	#endif
	
	return 0;
}


static stthreadpl *init_pool_work(void)
{
	stthreadpl *pool;  /*线程池句柄*/

	pool = tp_threadini(12, 12, 1);

	tp_addthreadwk(pool, mb_thread_recvserail, NULL);
	tp_addthreadwk(pool, mb_thread_recvscanner, NULL);

	if (new_temper)
		tp_addthreadwk(pool, tp_thread_tempernew, NULL);
	else
		tp_addthreadwk(pool, tp_thread_temper, NULL);

	tp_addthreadwk(pool, sm_thread_miniarm, NULL);
	tp_addthreadwk(pool, tp_thread_probwashclear, NULL);	//洗针为紧急事件需单独线程
	tp_addthreadwk(pool, tp_thread_fanctrl, NULL);
	tp_addthreadwk(pool, tp_thread_monitsersor, NULL);

	usleep(100000);//延时等待接收线程完成
	tp_addthreadwk(pool, tp_thread_otherwork,  NULL);
	usleep(300000);	//等待minib配置命令完成
	tp_addthreadwk(pool, tp_thread_mainarm,  NULL);

	#if NORMAL_VER
	tp_addthreadwk(pool, nt_thread_listensocket,  NULL);
	tp_addthreadwk(pool, nt_thread_reportnet,  NULL);
	#endif
	tp_addthreadwk(pool, tp_thread_beep,  NULL);

	return pool;
}



