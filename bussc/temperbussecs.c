


#include "log.h"
#include "unistd.h"
#include "string.h"
#include <errno.h>
#include <sys/time.h>
#include "termperbusses.h"
#include "temper_control.h"
#include "serailminibd.h"
#include "midbus.h"






extern volatile bool IN_CMP_WORK;
extern volatile ewkevent wkevent;
extern volatile bool flg_opwork1ready;	//玻片架1正在工作标识	用于结束时确定发给哪个minib stop命令. andry:接收到最后一个规程置位该标志表示可以开始工作了
extern volatile bool flg_opwork2ready;
extern volatile bool flg_opwork3ready;
extern volatile unsigned char last_reagentA;//A架玻片最近一次滴加的试剂
extern volatile unsigned char last_reagentB;
extern volatile unsigned char last_reagentC;
extern volatile bool StartDispenseA;
extern volatile bool StartDispenseB;
extern volatile bool StartDispenseC;
extern volatile unsigned int workstep_state_a;	//记录操作步数 最高字节置1表示温度加热完成，次高字节置1表示滴加动作完成 最低字节表示步数 第5个1表示时间到达
extern volatile unsigned int workstep_state_b;
extern volatile unsigned int workstep_state_c;
extern struct timeval begin_time;//程序开始运行时间
extern operate_head_list* operate_head1; //A架操作头指针
extern operate_head_list* operate_head2;//B架操作头指针
extern operate_head_list* operate_head3; 
extern pthread_mutex_t head_step_lock;



pthread_mutex_t mutex_mlock;
extern pthread_mutex_t mutex_modbuslock;
volatile int env_temp = 25;
volatile int MaxTemp = 1500;
volatile unsigned int temp_value[30] = {0};
volatile bool get_temvale = false;  //在温度维护界面向PC返回温度值标志
volatile int tem_limit = 1500;
temper_control_t temper_control1[10];	//温度控制全局变量
temper_control_t temper_control2[10];	//温度控制全局变量
temper_control_t temper_control3[10];	//温度控制全局变量
htemper_ctrfunc TemperControl_p[3] = {NULL};
volatile stminibd_cmdlist* pcmd_head = NULL;
volatile stminibd_cmdlist* pcmd_head2 = NULL;
volatile stminibd_cmdlist* pcmd_head3 = NULL;
volatile temper_cmd_list* ptemcmd_head = NULL;
volatile tem_cmd_list* ptem_cmd_head = NULL;
volatile bool tem_over_load;
volatile bool tem_over_loadA;
volatile bool tem_over_loadB;
volatile bool tem_over_loadC;
volatile bool flg_temperheating1 = false;  // 玻片正在加热中  andry  
volatile bool flg_temperheating2 = false;
volatile bool flg_temperheating3 = false;
volatile bool NEED_851 = false, NEED_852 = false, NEED_853 = false;




void EnvTemp(void)
{
	stTemper_cmdframeunion te_cmd;
	unsigned char recieve_tem = 0;
	int res = 0;
	
	te_cmd.devaddr = 8;
	te_cmd.chopcmd = CH1_Present_Value;
	te_cmd.temvalue = 1;
	pthread_mutex_lock(&mutex_modbuslock);
	if (tc_temsendcmd(8, CH4_Present_Value, 1)) 
	{
		if (!tc_recivtempframe((uint8_t*)&env_temp, &te_cmd ))
		{
			printf("recieve error_code = %x", recieve_tem); 
			res = FALSE;
		}
	}
	else
	{
		printf("tc_temsendcmd failed\n");
		res = FALSE;
	}
	if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_modbuslock");

	printf("env_temp=%d\n",env_temp);
}


bool tp_temperinit(void)
{
	int temvalue[30] = {0}, cmp_temvalue[30] = {0};
	unsigned int i, er_cnt=0, j = 0;
	int res_cmp = 0;
	bool res = true;
	stminibd_sendpacket mini_cmd;
	stTemper_cmdframeunion te_cmd;
    int recieve_tem = 0;
	char er_counter[30] = {0};

	IN_CMP_WORK = false;

	printf("in [tp_temperinit]\n");

	sleep(5);  //等待温控器稳定

	if (tc_sendcmdchanl(1, CH1_Present_Value, (int*)&temp_value[0]))	//读取温度值
	{
		printf("all temper value\n");
	}
	else
	{
		printf("get all temper value failed\n");
	}
	get_temvale = true;
	if ( tc_sendcmdchanl(0, CH1_RUN_STOP, &temvalue[0]) )
		printf("tc_sendcmdchanl success\n");
	else
		return false;

	printf("tp_temperinit() first step end\n");

	if (tc_sendcmdchanl(800 + temp_goalval[80], CH1_SV, &temvalue[0]))	//		发送加热50度命令
		printf("tc_sendcmdchanl success\n");
	else
		return false;


	printf("in sleep(30 * 6)\n");

	j = 0;
	while(1)
	{
		sleep(30);
		if (wkevent == MAINTAIN_WORK)
		{
			temvalue[0] = 0;
			if ( tc_sendcmdchanl(0, CH1_SV, (int *)&temvalue[0]) )
			printf("tc_sendcmdchanl success\n");
			return true;
		}
		if (tc_sendcmdchanl(1, CH1_Present_Value, (int *)&temp_value[0]))	//读取温度值
		{
			printf("all temper value\n");
			for(i =0; i < 30;i++)
			{
				if (temp_value[i] > tem_limit)//比较传感器固定
					er_counter[i]++;	
			}			
			get_temvale = true;
		}
		else
		{
			printf("get all temper value failed\n");
		}
			
		if (j++ >= 5)
			break;
	}

	for(i =0; i < 30;i++)
	{
		if (er_counter[i] >= 5)//比较传感器固定
		{
			er_counter[i] = 0;
			mb_seterrorcode(i + TEMPER1);	
		}
	}	


	if (tc_sendcmdchanl(1, CH1_Present_Value, &temvalue[0]))	//读取温度值
		printf("tc_sendcmdchanl success\n");
	else
		return false;
	
	for (i = 0 ;i < TEM_NUM; i++)
		printf(" %d ", temvalue[i]);	

	printf("\n");

	for(i = 0; i < TEM_NUM; i++)		//计算是否超过正负5度
	{
		if ((temvalue[i] > (800 + temp_goalval[80] + 50)) || ((temvalue[i] < 800 + temp_goalval[80] - 50)))
		{	
			printf("might error num=%d, val=%d\n", i, temvalue[i]);
			for (er_cnt = 0; er_cnt < 5; er_cnt++)
			{
				te_cmd.chopcmd = CH1_Present_Value;
				te_cmd.temvalue = 1;
				sm_minisendonecmd(&te_cmd, i, (uint8_t*)&recieve_tem);
				printf("recieve_tem=%d\n", recieve_tem);
				if ((recieve_tem > (800 + temp_goalval[80] - 50)) && ((recieve_tem < 800 + temp_goalval[80] + 50)))
					break;
				sleep(1);
			}
					
			if (er_cnt >= 5)
			{
				printf("some heater error num=%d\n", i);
				//	//向发送PC加热块信息
				temp_value[i] = 0;//PC程序需要
				te_cmd.chopcmd = CH1_RUN_STOP;			
				te_cmd.temvalue = 1;
				sm_minisendonecmd(&te_cmd, i, (uint8_t*)&recieve_tem);
				mb_seterrorcode(i + TEMPER1);	
			}
			//	res  = FALSE;//初始化允许有错误加热块
		}
	}

	if ((res_cmp = tc_getfulltempval(cmp_temvalue) ) == -2)
		return true;
	else if (res_cmp < 0)
		return false;
		
	printf("CmpTem\n");
	for (i = 0 ; i < TEM_NUM; i++)
		printf(" %d ", cmp_temvalue[i]);

	for (i = 0 ; i < TEM_NUM; i++)	//		与测试传感器比较100 为模拟开关误差偏差值在5°范围内
	{
		if (cmp_temvalue[i] > (tem_limit + 500) && cmp_temvalue[i] < 5000)//比较传感器固定
		{
			tc_getcmtempval(i + 1, &cmp_temvalue[i]);
			if (cmp_temvalue[i] > (tem_limit + 500) && cmp_temvalue[i] < 5000)//比较传感器固定
			{
				tc_getcmtempval(i + 1, &cmp_temvalue[i]);
				if (cmp_temvalue[i] > (tem_limit + 500) && cmp_temvalue[i] < 5000)//比较传感器固定
				{
					te_cmd.chopcmd = CH1_Present_Value;
					sm_minisendonecmd(&te_cmd, i, (uint8_t*)&recieve_tem);
					if (recieve_tem > tem_limit && recieve_tem < 2000)
						mb_seterrorcode(i + TEMPER1_EM);	
				}
			} 	
		}
	}

	if (tc_sendcmdchanl(0, CH1_SV, &temvalue[0]))	//测试结束停止加热
		printf("tc_sendcmdchanl success\n");
	else
		return false;

	
	//发送降温命令给minb
	memset(&mini_cmd,0,sizeof(mini_cmd));
	mini_cmd.cmd = MAINTAIN_FAN_WORK;
	mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;	

	mini_cmd.minicmd_num=2;

	mini_cmd.minicmd_buffer[4] = 1;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head, mini_cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");

	mini_cmd.minicmd_buffer[4] = 2;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");

	mini_cmd.minicmd_buffer[4] = 3;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
				
	for (i = 0;i < 36;i++)
	{
		sleep(5);
		tc_sendcmdchanl(1, CH1_Present_Value, (int32_t*)&temp_value[0]);
		get_temvale = true;
		if (wkevent == MAINTAIN_WORK)
		{
			mini_cmd.minicmd_buffer[4] = 4;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			mini_cmd.minicmd_buffer[4] = 5;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			mini_cmd.minicmd_buffer[4] = 6;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			return true;
		}
	}
	mini_cmd.minicmd_buffer[4] = 4;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	mini_cmd.minicmd_buffer[4] = 5;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	mini_cmd.minicmd_buffer[4] = 6;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	if (wkevent == MAINTAIN_WORK)
		return true;

	return res;
}


//sel = 0, 10 , 20对应A,B,C
int tp_tempctrnew(char sel, unsigned char i, bool isctrl)
{	
	static unsigned int counter=0;
	unsigned int j = 0, is_alltemp_zero = 0;
	struct timeval now;
	stminibd_sendpacket mini_cmd;
	int res = 0;
	static int  internal_time[10];
	static int fan_time = 0;
	short rtemvalue;
	bool tem_over_load = false;
	temper_control_t * temper_control = NULL;
	bool isoperate_working = false;
	unsigned char *last_reagent = NULL;
	unsigned int *head_step = NULL;
	bool StartDispense = false;
	operate_head_list* operate_head = NULL;
	bool NEED_85 = false;
	bool in_temper_control = false;
	static char Isreboot[30] = {0};

	if(sel == 0)
	{
		tem_over_load = (bool)tem_over_loadA;
		temper_control = temper_control1;
		isoperate_working = (bool)flg_opwork1ready;
		last_reagent = (uint8_t*)&last_reagentA;
		head_step = (uint32_t*)&workstep_state_a;
		StartDispense = (bool)StartDispenseA;
		operate_head = operate_head1;
		NEED_85 = (bool)NEED_851;
		in_temper_control = (bool)flg_temperheating1;
	}
	else if(sel == 10)
	{
		tem_over_load = (bool)tem_over_loadB;
		temper_control = temper_control2;
		isoperate_working = (bool)flg_opwork2ready;
		last_reagent = (uint8_t*)&last_reagentB;
		head_step = (uint32_t*)&workstep_state_b;
		StartDispense = (bool)StartDispenseB;
		operate_head = operate_head2;
		NEED_85 = (bool)NEED_852;
		in_temper_control = (bool)flg_temperheating2;
	}
	else if(sel == 20)
	{
		tem_over_load = (bool)tem_over_loadC;
		temper_control = temper_control3;
		isoperate_working = (bool)flg_opwork3ready;
		last_reagent = (uint8_t*)&last_reagentC;
		head_step = (uint32_t*)&workstep_state_c;
		StartDispense = (bool)StartDispenseC;
		operate_head = operate_head3;
		NEED_85 = (bool)NEED_853;
		in_temper_control = (bool)flg_temperheating3;
	}
	gettimeofday(&now, NULL);
	
	if ((temper_control[i].temp > 0) && (temper_control[i].state == 0) ) //需加热 且 还没法送加热命令
	{
		if (temper_control[i].temp > 900)
		{
			NEED_85 = true;
			temper_control[i].state = IN_85;
		}
		else
		{
			temper_control[i].state = SEND_TEMP;
		}
	}		
	else if(temper_control[i].state == IN_85)
	{
		temper_control[i].state = IN_85_TIME;
	}
	else if(temper_control[i].state == IN_85_TIME)
	{
		temper_control[i].state = SEND_TEMP;
	}
	else if ((temper_control[i].temp > 0) && (temper_control[i].state == SEND_TEMP)) //需加热 且 还没到指定温度
	{
		internal_time[i] = now.tv_sec;
	
		tp_sendtemperframe((temper_control[i].plate_num) + sel, SET, 1, temper_control[i].temp, &rtemvalue);
				
			//向发送PC加热信息
		//	mb_seterrorcode(ATEMPERL);
		Isreboot[(temper_control[i].plate_num) + sel] = 0;
		temper_control[i].state= SENDED_TEMPERCMD;//已发送加热命令标识	
	}
	else if ((temper_control[i].temp > 0) && (temper_control[i].state == SENDED_TEMPERCMD)) //需加热 且 还没到指定温度
	{
		if (now.tv_sec > (internal_time[i] + 480) && Isreboot[(temper_control[i].plate_num) + sel] == 0)//8min重启
		{
			Isreboot[(temper_control[i].plate_num) + sel] = 1;
			tp_sendtemperframe((temper_control[i].plate_num) + sel, SET, 1, 0,&rtemvalue);
			tp_sendtemperframe((temper_control[i].plate_num) + sel, SET, 1, temper_control[i].temp, &rtemvalue);		
		}
		if (now.tv_sec > (internal_time[i] + 600))
		{
			mb_seterrorcode(temper_control1[i].plate_num + TEMPER1+sel);
			temper_control[i].state = REACH_TEMP;
		}
		if (!(isoperate_working) || tem_over_load)
			temper_control[i].state = REACH_TEMP;	
		if (counter > 0x0000efff )
		{
			counter = 0;
			printf("temper_control1[%d].temp =%d temp_value[temper_control1[%d].plate_num] = %d\n",i,temper_control[i].temp, i,temp_value[temper_control[i].plate_num]);
		}
		counter++;	
		if (temper_control[i].temp <= (temp_value[temper_control[i].plate_num + sel] + 20))//到达指定温度 10为1度
		{
			temper_control[i].state = REACH_TEMP;	//到达指定温度后temp标识
			
			if (*last_reagent == NO_REAGENT || *last_reagent == REAGENT_DEWAX)		
					*head_step = (*head_step) | 0X01000000;
				
			//	if (tc_gettemtempvalprintf(i + 1) > MaxTemp)
			//		res = i + 1;
			#if(USE_PRINT_LOG == 1)
			printf("\nwhen REACH_TEMP temper_control1[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n",i,temper_control[i].time,(now.tv_sec - begin_time.tv_sec));
			#endif
		}
	//	while(!flg_mainproexit);
	}
	else if((temper_control[i].temp > 0) && temper_control[i].state == REACH_TEMP)	//等待加热时间到达 并且滴加结束
	{
		
		if (StartDispense)
		{
			temper_control[i].time += now.tv_sec - begin_time.tv_sec;//相对于刚开机时的绝对时间
		//	temper_control1[i].state = WAIT_STRECH;
			temper_control[i].state = START_TIME;
		}
		//	printf("goto WAIT_STRECH\n");
	}
	
	else if ((temper_control[i].temp > 0) && temper_control[i].state == WAIT_STRECH && ((*head_step) & 0X00FF0000) > 0)
	{
		gettimeofday(&now, NULL);

		printf("temper_control1[i].time=%d\n",temper_control[i].time);
		{
			temper_control[i].state = START_TIME;
		}
	}
	else if ((temper_control[i].temp > 0) && temper_control[i].state == START_TIME)
	{					
		if (temper_control[i].time <= (now.tv_sec - begin_time.tv_sec) || !(isoperate_working)) //加热时间到达
		{
			#if(USE_PRINT_LOG == 1)
			printf("\nwhen REACH_TTIME temper_control1[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n", 
				i, temper_control[i].time,(now.tv_sec - begin_time.tv_sec));		
			#endif

			tp_sendtemperframe((temper_control[i].plate_num)  + sel, SET, 1,0,&rtemvalue);
			
			temper_control[i].temp = 0;//不用加热标识
			temper_control[i].state = END_TIME;
			printf("[temper_control1]all temp\n");
			for (j = 0; j < 10; j++)
			{
				printf(" %d ", temper_control[j].temp);
				if (temper_control[j].temp > is_alltemp_zero)
					is_alltemp_zero = temper_control[j].temp;
			}
			if (is_alltemp_zero == 0) 
			{
				StartDispense = false;
			#if DE_TONG								
				if (isoperate_working && (
					//(last_reagentA == REAGENT_DEWAX && operate_head1->operate.reagent != REAGENT_DEWAX) ||
					//(last_reagentA == REAGENT_ALCOHOL && operate_head1->operate.reagent != REAGENT_ALCOHOL) ||
					(NEED_85 &&  *last_reagent == REAGENT_ER1 && operate_head->operate.reagent != REAGENT_ER1) ||
					(NEED_85 &&  *last_reagent == REAGENT_ER2 && operate_head->operate.reagent != REAGENT_ER2)//制备结束或热修复结束需要打开风机
				))
			#else
				printf("*NEED_85=%d *last_reagent=%d, operate_head->operate.reagent=%d", NEED_85,*last_reagent,operate_head->operate.reagent);

					//防止强制停止降温操作
				// if (*isoperate_working &&  // 怎么对一个bool取地址？？？？？？ andry
                if(isoperate_working && 
						(//(last_reagentA == REAGENT_DEWAX && operate_head1->operate.reagent != REAGENT_DEWAX) ||
						//(last_reagentA == REAGENT_ALCOHOL && operate_head1->operate.reagent != REAGENT_ALCOHOL) ||
				//		 ( last_reagentA == REAGENT_ER1 && operate_head1->operate.reagent == REAGENT_ER1 && operate_head1->operate.next->reagent != REAGENT_ER1) ||
				//		 ( last_reagentA == REAGENT_ER2 && operate_head1->operate.reagent == REAGENT_ER2 && operate_head1->operate.next->reagent != REAGENT_ER2)//制备结束或热修复结束需要打开风机
							(NEED_85 &&  *last_reagent == REAGENT_ER1 && operate_head->operate.reagent != REAGENT_ER1) ||
							(NEED_85 &&  *last_reagent == REAGENT_ER2 && operate_head->operate.reagent != REAGENT_ER2) //制备结束或热修复结束需要打开风机
						//	(NEED_851 && (operate_head1->operate.next->reagent == REAGENT_ER1 || operate_head1->operate.next->reagent == REAGENT_ER2))
						)
					)
			#endif
				{	
					NEED_85 = false;

					fan_time = now.tv_sec;
					#ifdef BIG_VERSION
						ArmCabinCmdList[sel / 10].cmd = ArmCabinFANON ;
					#else
						memset(&mini_cmd,0,sizeof(mini_cmd));
						mini_cmd.cmd = MAINTAIN_FAN_WORK;
						mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
						mini_cmd.minicmd_buffer[4] = 1 + sel / 10; //ON
						mini_cmd.minicmd_num=2;
						pthread_mutex_lock(&mutex_mlock);
						set_minicmd(pcmd_head,mini_cmd	);	
						pthread_mutex_unlock(&mutex_mlock);
					#endif
					
						temper_control[0].state = START_FAN;		
				}	
				else
				{		
					pthread_mutex_lock(&head_step_lock);
					*head_step = (*head_step) | 0X01010000;
					if (pthread_mutex_unlock(&head_step_lock) != 0)
						lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");

					isctrl = false;
					in_temper_control = false;
				}		
			}
		}
	}
	else if (temper_control[0].state == START_FAN)
	{
		if (  (now.tv_sec - fan_time) > 5*60 ||  !(isoperate_working))
		{
			#ifdef BIG_VERSION
				ArmCabinCmdList[sel / 10].cmd = ArmCabinFANOFF ;
			#else
			memset(&mini_cmd,0,sizeof(mini_cmd));
			mini_cmd.cmd = MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[4] = 4+ sel / 10; //OFF	
			mini_cmd.minicmd_num=2;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);	
			pthread_mutex_unlock(&mutex_mlock);
			#endif
		
			pthread_mutex_lock(&head_step_lock);
			*head_step = (*head_step) | 0X01010000;
			if (pthread_mutex_unlock(&head_step_lock) !=0 )
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			isctrl = false;
			in_temper_control = false;
			temper_control[0].state = END_TIME;		
		}
	}

	return res;
}


//超过最大值返回 位置1 ~10
//无错误返回0
int tb_tempctrola(void)
{	
	static unsigned char i = 0;
	static unsigned int counter=0;
	unsigned int j = 0, is_alltemp_zero = 0;
	struct timeval now;
	unsigned char recieve_code;
	unsigned char dev_addr;
	emTemp_chopcmd cmd;
	stminibd_sendpacket mini_cmd;
	unsigned int value;
	stTemper_cmdframeunion te_cmd;
	int res = 0;
	static int internal_time[10];
	static int fan_time = 0;

	gettimeofday(&now, NULL);
	
	if ((temper_control1[i].temp > 0) && (temper_control1[i].state == 0) ) //需加热 且 还没法送加热命令
	{		
		te_cmd.devaddr = dev_addr = temper_control1[i].plate_num / 4 + 1;
		te_cmd.chopcmd = cmd = CH1_SV + (temper_control1[i].plate_num % 4) * 3;
		te_cmd.temvalue = value = temper_control1[i].temp;
		printf("dev_addr = %d cmd = %d value = %d\n", dev_addr,cmd,value);

		if (temper_control1[i].temp > 1000)
		{
			NEED_851 = TRUE;

			temper_control1[i].state = IN_85;
		}
		else
		{
			temper_control1[i].state = SEND_TEMP;
		}		
	}
	else if(temper_control1[i].state == IN_85)
	{	
		temper_control1[i].state = IN_85_TIME;
	}
	else if(temper_control1[i].state == IN_85_TIME)
	{
		temper_control1[i].state = SEND_TEMP;
	}
	else if ((temper_control1[i].temp > 0) && (temper_control1[i].state == SEND_TEMP)) //需加热 且 还没到指定温度
	{
		internal_time[i] = now.tv_sec;
		te_cmd.devaddr = dev_addr = temper_control1[i].plate_num / 4 + 1;
		te_cmd.chopcmd = cmd = CH1_SV + (temper_control1[i].plate_num % 4) * 3;
		te_cmd.temvalue = value = temper_control1[i].temp;
		printf("dev_addr = %d cmd = %d value = %d\n", dev_addr,cmd,value);
	
		pthread_mutex_lock(&mutex_modbuslock);	//发送
		if (tc_temsendcmd(dev_addr, cmd, value) )
		{
			printf("\n[TemperControl1]SendStartCmdTemper sucessi = %d\n\n", i);
			if (!tc_recivtempframe(&recieve_code,&te_cmd))
				printf("[TemperControl1]recieve error_code = %x", recieve_code);	
		}
		else
			printf("tc_temsendcmd failed\n");
		if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_modbuslock");
			
			//向发送PC加热信息
			//	mb_seterrorcode(ATEMPERL);
			
		temper_control1[i].state= SENDED_TEMPERCMD;//已发送加热命令标识	
	}
	else if ((temper_control1[i].temp > 0) && (temper_control1[i].state == SENDED_TEMPERCMD)) //需加热 且 还没到指定温度
	{
		if (now.tv_sec > (internal_time[i] + 600))
		{
			mb_seterrorcode((temper_control1[i].plate_num % 30) + TEMPER1);
			temper_control1[i].state = REACH_TEMP;
		}
			
		if (!flg_opwork1ready || tem_over_loadA)
			temper_control1[i].state = REACH_TEMP;	
		if (counter > 0x0000efff )
		{
			counter = 0;
			printf("temper_control1[%d].temp =%d temp_value[temper_control1[%d].plate_num] = %d\n",
				i,	temper_control1[i].temp, i,	temp_value[temper_control1[i].plate_num]);
		}
		counter++;	
		if (temper_control1[i].temp <= (temp_value[temper_control1[i].plate_num] + 20))//到达指定温度 10为1度
		{
			temper_control1[i].state = REACH_TEMP;	//到达指定温度后temp标识
	
			if (last_reagentA == NO_REAGENT || last_reagentA == REAGENT_DEWAX)		
					workstep_state_a = workstep_state_a | 0X01000000;
			
			//	if (tc_gettemtempvalprintf(i + 1) > MaxTemp)
			//		res = i + 1;
			#if(USE_PRINT_LOG == 1)
			printf("\nwhen REACH_TEMP temper_control1[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n",i,temper_control1[i].time,(now.tv_sec - begin_time.tv_sec));
			#endif
		}
		//	while(!flg_mainproexit);
	}
	else if((temper_control1[i].temp > 0) && temper_control1[i].state == REACH_TEMP)	//等待加热时间到达 并且滴加结束
	{	
		if (StartDispenseA)
		{
			temper_control1[i].time += now.tv_sec - begin_time.tv_sec;//相对于刚开机时的绝对时间
		//	temper_control1[i].state = WAIT_STRECH;
			temper_control1[i].state = START_TIME;
		}
		//	printf("goto WAIT_STRECH\n");
	}
	
	else if ((temper_control1[i].temp > 0) && temper_control1[i].state == WAIT_STRECH && (workstep_state_a & 0X00FF0000) > 0)
	{
		gettimeofday(&now, NULL);

		printf("temper_control1[i].time=%d\n",temper_control1[i].time);
	
		temper_control1[i].state = START_TIME;
		
	}
	else if ((temper_control1[i].temp > 0) && temper_control1[i].state == START_TIME)
	{					
		if (temper_control1[i].time <= (now.tv_sec - begin_time.tv_sec) || !flg_opwork1ready)	//加热时间到达
		{
			//printf("\nwhen REACH_TTIME temper_control1[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n",i,temper_control1[i].time,(now.tv_sec - begin_time.tv_sec));
			te_cmd.devaddr = dev_addr = temper_control1[i].plate_num / 4 + 1;
			te_cmd.chopcmd = cmd = CH1_SV + (temper_control1[i].plate_num % 4) * 3;
			te_cmd.temvalue = value = 0;
			pthread_mutex_lock(&mutex_modbuslock);	//发送
			if (tc_temsendcmd(dev_addr, cmd, value) )
			{
				printf("[TemperControl1]SendStopCmdTemper sucess i= %d dev_addr=%d cmd=%d\n", i, dev_addr, cmd);
				if (!tc_recivtempframe(&recieve_code,&te_cmd))
					printf("[TemperControl1]recieve error_code = %x", recieve_code);	
			}
			else
				printf("tc_temsendcmd failed\n");
			if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
				lprintf(log_my, ERROR, "%s","pthread_mutex_unlock error mutex_modbuslock");
				
			temper_control1[i].temp = 0;//不用加热标识
			temper_control1[i].state = END_TIME;
			printf("[temper_control1]all temp\n");
			for (j = 0; j < 10; j++)
			{
				printf(" %d ", temper_control1[j].temp);
				if (temper_control1[j].temp > is_alltemp_zero)
					is_alltemp_zero = temper_control1[j].temp;
			}
			if (is_alltemp_zero == 0) 
			{
				StartDispenseA = FALSE;

					//防止强制停止降温操作
				if (	flg_opwork1ready &&
						(//(last_reagentA == REAGENT_DEWAX && operate_head1->operate.reagent != REAGENT_DEWAX) ||
						//(last_reagentA == REAGENT_ALCOHOL && operate_head1->operate.reagent != REAGENT_ALCOHOL) ||
					//		 ( last_reagentA == REAGENT_ER1 && operate_head1->operate.reagent == REAGENT_ER1 && operate_head1->operate.next->reagent != REAGENT_ER1) ||
					//		 ( last_reagentA == REAGENT_ER2 && operate_head1->operate.reagent == REAGENT_ER2 && operate_head1->operate.next->reagent != REAGENT_ER2)//制备结束或热修复结束需要打开风机
							(NEED_851 &&  last_reagentA == REAGENT_ER1 && operate_head1->operate.reagent != REAGENT_ER1) ||
							(NEED_851 &&  last_reagentA == REAGENT_ER2 && operate_head1->operate.reagent != REAGENT_ER2) //制备结束或热修复结束需要打开风机
						//	(NEED_851 && (operate_head1->operate.next->reagent == REAGENT_ER1 || operate_head1->operate.next->reagent == REAGENT_ER2))
						)
					)
				{
				
					NEED_851 = false;

					fan_time = now.tv_sec;
					memset(&mini_cmd, 0, sizeof(mini_cmd));
					mini_cmd.cmd = MAINTAIN_FAN_WORK;
					mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
					mini_cmd.minicmd_buffer[4] = 1; //ON
					mini_cmd.minicmd_num=2;
					pthread_mutex_lock(&mutex_mlock);
					set_minicmd(pcmd_head,mini_cmd	);	
					pthread_mutex_unlock(&mutex_mlock);
					temper_control1[0].state = START_FAN;		
				}	
				else
				{		
					pthread_mutex_lock(&head_step_lock);
					workstep_state_a = workstep_state_a | 0X01010000;
					if (pthread_mutex_unlock(&head_step_lock) != 0)
						lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
					TemperControl_p[0] = NULL;
					flg_temperheating1 = false;
				}	
			}
		}
	}
	else if (temper_control1[0].state == START_FAN)
	{
		if (  (now.tv_sec - fan_time) > 5*60 ||  !flg_opwork1ready)
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_a = workstep_state_a | 0X01010000;
			if (pthread_mutex_unlock(&head_step_lock) !=0 )
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			TemperControl_p[0] = NULL;
			flg_temperheating1 = false;
			temper_control1[0].state = END_TIME;

			memset(&mini_cmd,0,sizeof(mini_cmd));
			mini_cmd.cmd = MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[4] = 4; //OFF	
			mini_cmd.minicmd_num=2;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);	
			pthread_mutex_unlock(&mutex_mlock);			
		}
	}
	//tempctl1_end:
	i++;
	if (i > 9)
		i = 0;

	return res;
}
	
	
int tb_tempctrolb(void)
{
	static unsigned char i = 0;
	static unsigned int counter=0;
	unsigned int j = 0, is_alltemp_zero = 0;
	struct timeval now;
	unsigned char recieve_code;
	unsigned char dev_addr;
	emTemp_chopcmd cmd;
	stminibd_sendpacket mini_cmd;
	unsigned int value;
	stTemper_cmdframeunion te_cmd;
	int res = 0;
	static int  internal_time[10];
	static int fan_time = 0;

	gettimeofday(&now, NULL);

	if ((temper_control2[i].temp > 0) && (temper_control2[i].state == 0) ) //需加热 且 还没法送加热命令
	{
		if (temper_control2[i].plate_num < 2)
		{
			cmd = CH1_SV + (temper_control2[i].plate_num + 2) * 3;
			dev_addr =  3;
		}
		else if (temper_control2[i].plate_num  < 6)
		{
			cmd = CH1_SV + (temper_control2[i].plate_num - 2) * 3;
			dev_addr =  4;	
		}
		else
		{
			cmd = CH1_SV + (temper_control2[i].plate_num - 6) * 3;
			dev_addr =  5;	
		}
		
		value = temper_control2[i].temp;
		printf("dev_addr = %d cmd = %d value = %d\n", dev_addr,cmd,value);
		te_cmd.devaddr = dev_addr;
		te_cmd.chopcmd = cmd;
		te_cmd.temvalue = value;
		if (temper_control2[i].temp > 1000)
		{
			NEED_852 = true;

			temper_control2[i].state = IN_85;
		}
		else
		{
			temper_control2[i].state = SEND_TEMP;
		}
					
	}
	else if(temper_control2[i].state == IN_85)
	{	
		temper_control2[i].state = IN_85_TIME;
	}
	else if(temper_control2[i].state == IN_85_TIME)
	{
	
		temper_control2[i].state = SEND_TEMP;
	}
	else if ((temper_control2[i].temp > 0) && (temper_control2[i].state == SEND_TEMP)) //需加热 且 还没到指定温度
	{
		if (temper_control2[i].plate_num < 2)
		{
			cmd = CH1_SV + (temper_control2[i].plate_num + 2) * 3;
			dev_addr =  3;
		}
		else if (temper_control2[i].plate_num  < 6)
		{
			cmd = CH1_SV + (temper_control2[i].plate_num - 2) * 3;
			dev_addr =  4;	
		}
		else
		{
			cmd = CH1_SV + (temper_control2[i].plate_num - 6) * 3;
			dev_addr =  5;	
		}
		internal_time[i] = now.tv_sec;
		te_cmd.devaddr = dev_addr;
		te_cmd.chopcmd = cmd;
		te_cmd.temvalue = value = temper_control2[i].temp;
		printf("dev_addr = %d cmd = %d value = %d\n", dev_addr,cmd,value);
		
		pthread_mutex_lock(&mutex_modbuslock); //发送
		if (tc_temsendcmd(dev_addr, cmd, value) )
		{
			printf("\n[tb_tempctrolb]SendStartCmdTemper sucessi = %d\n\n", i);
			if (!tc_recivtempframe(&recieve_code,&te_cmd))
				printf("[tb_tempctrolb]recieve error_code = %x", recieve_code);	
		}
		else
			printf("tc_temsendcmd failed\n");
		if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_modbuslock");
			
			//向发送PC加热信息
		//	mb_seterrorcode(ATEMPERL);
			
		temper_control2[i].state= SENDED_TEMPERCMD;//已发送加热命令标识
		
	}
	else if ((temper_control2[i].temp > 0) && (temper_control2[i].state == SENDED_TEMPERCMD)) //需加热 且 还没到指定温度
	{
		if (now.tv_sec > (internal_time[i] + 600))
		{
			mb_seterrorcode((temper_control1[i].plate_num % 30) + TEMPER1);
			temper_control2[i].state = REACH_TEMP;
		}
		if (!flg_opwork2ready || tem_over_loadB)
			temper_control2[i].state = REACH_TEMP;	
		if (counter > 0x0000efff )
		{
			counter = 0;
			printf("temper_control2[%d].temp =%d temp_value[temper_control2[%d].plate_num] = %d\n",i,temper_control2[i].temp, i,temp_value[temper_control2[i].plate_num + 10]);
		}
		counter++;	
			
		if (temper_control2[i].temp <= (temp_value[10 + temper_control2[i].plate_num] + 20))//到达指定温度 10为1度
		{
			//		if (tc_gettemtempvalprintf(i + 1 + 10) > MaxTemp)
			//			res = i + 1;
					
			temper_control2[i].state = REACH_TEMP;	//到达指定温度后temp标识

			if (last_reagentB == NO_REAGENT || last_reagentB == REAGENT_DEWAX)		
					workstep_state_b = workstep_state_b | 0X01000000;
			#if(USE_PRINT_LOG == 1)
			printf("\nwhen REACH_TEMP temper_control2[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n",i,temper_control2[i].time,(now.tv_sec - begin_time.tv_sec));
			#endif
		}
	//	while(!flg_mainproexit);
	}else if((temper_control2[i].temp > 0) && temper_control2[i].state == REACH_TEMP)	//等待加热时间到达
	{	
		printf("temper_control2[%d].time=%d\n",i,temper_control2[i].time);
		if (StartDispenseB)
		{
			temper_control2[i].time += now.tv_sec - begin_time.tv_sec;//相对于刚开机时的绝对时间
			//	temper_control2[i].state = WAIT_STRECH;	
			temper_control2[i].state = START_TIME;
		}
	}
	else if ((temper_control2[i].temp > 0) && temper_control2[i].state == WAIT_STRECH && (workstep_state_b & 0X00FF0000) > 0)
	{
		gettimeofday(&now, NULL);

		printf("temper_control2[i].time=%d\n",temper_control2[i].time);

		temper_control2[i].state = START_TIME;
	
	}
	else if ((temper_control2[i].temp > 0) && temper_control2[i].state == START_TIME)
	{
		if (temper_control2[i].time <= (now.tv_sec - begin_time.tv_sec) || !flg_opwork2ready)	//加热时间到达
		{	
			#if(USE_PRINT_LOG == 1)
			printf("\nwhen REACH_TTIME temper_control2[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n",i,temper_control2[i].time,(now.tv_sec - begin_time.tv_sec));
			#endif
			if (temper_control2[i].plate_num < 2)
			{
				cmd = CH1_SV + (temper_control2[i].plate_num + 2) * 3;
				dev_addr =  3;
			}
			else if (temper_control2[i].plate_num  < 6)
			{
				cmd = CH1_SV + (temper_control2[i].plate_num - 2) * 3;
				dev_addr =  4;	
			}
			else
			{
				cmd = CH1_SV + (temper_control2[i].plate_num - 6) * 3;
				dev_addr =  5;	
			}
			
			value = 0;
			te_cmd.devaddr = dev_addr;
			te_cmd.chopcmd = cmd;
			te_cmd.temvalue = value;
			
			pthread_mutex_lock(&mutex_modbuslock);	//发送
			if (tc_temsendcmd(dev_addr, cmd, value) )
			{
				printf("[tb_tempctrolb]SendStopCmdTemper sucess i= %d\n", i);
				if (!tc_recivtempframe(&recieve_code,&te_cmd))
					printf("[tb_tempctrolb]recieve error_code = %x", recieve_code);	
			}
			else
				printf("tc_temsendcmd failed\n");
			if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
							lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mutex_modbuslock");

			temper_control2[i].temp = 0;//不用加热标识
			temper_control2[i].state = END_TIME;
			printf("[temper_control2]all temp\n");
			for (j = 0; j < 10; j++)
			{
				printf(" %d ", temper_control2[j].temp);
				if (temper_control2[j].temp > is_alltemp_zero)
					is_alltemp_zero = temper_control2[j].temp;
			}
			if (is_alltemp_zero == 0) 
			{
				StartDispenseB = false;
		
				if (	flg_opwork2ready &&
						(//(last_reagentB == REAGENT_DEWAX && operate_head2->operate.reagent != REAGENT_DEWAX) ||
					//	 ( last_reagentB == REAGENT_ER1 && operate_head2->operate.reagent == REAGENT_ER1 && operate_head2->operate.next->reagent != REAGENT_ER1) ||
					//	 ( last_reagentB == REAGENT_ER2 && operate_head2->operate.reagent == REAGENT_ER2 && operate_head2->operate.next->reagent != REAGENT_ER2)//制备结束或热修复结束需要打开风机

							(NEED_852 &&  last_reagentB == REAGENT_ER1 && operate_head2->operate.reagent != REAGENT_ER1) ||
							(NEED_852 &&  last_reagentB == REAGENT_ER2 && operate_head2->operate.reagent != REAGENT_ER2) //制备结束或热修复结束需要打开风机
						//	(NEED_852 && (operate_head2->operate.next->reagent == REAGENT_ER1 || operate_head2->operate.next->reagent == REAGENT_ER2))
						)
				)
				{		
					NEED_852 = false;
					
					printf("begin fan&&&&&&&&&&&&&&&&&&&&&&\n");
						//发送降温命令给minb
						fan_time = now.tv_sec;
					memset(&mini_cmd, 0, sizeof(mini_cmd));
					mini_cmd.cmd = MAINTAIN_FAN_WORK;
					mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
					mini_cmd.minicmd_buffer[4] = 2; //ON
					mini_cmd.minicmd_num=2;
					pthread_mutex_lock(&mutex_mlock);
					set_minicmd(pcmd_head,mini_cmd	);	
					pthread_mutex_unlock(&mutex_mlock);
					temper_control2[0].state = START_FAN;
				}	
				else
				{		
					//		AddTemperTab(temper_control3);
					pthread_mutex_lock(&head_step_lock);
					workstep_state_b = workstep_state_b | 0X01010000;
					if(pthread_mutex_unlock(&head_step_lock) != 0)
						lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
					TemperControl_p[1] = NULL;
					flg_temperheating2 = false;
				}
			}
		}
	}
	else if (temper_control2[0].state == START_FAN)
	{
		if (  (now.tv_sec - fan_time) > 5*60 ||  !flg_opwork2ready)
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_b = workstep_state_b | 0X01010000;
			if (pthread_mutex_unlock(&head_step_lock) !=0 )
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			TemperControl_p[1] = NULL;
			flg_temperheating2 = false;
			temper_control2[0].state = END_TIME;

			memset(&mini_cmd,0,sizeof(mini_cmd));
			mini_cmd.cmd = MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[4] = 5; //OFF	
			mini_cmd.minicmd_num=2;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);	
			pthread_mutex_unlock(&mutex_mlock);		
		}
	}

	i++;
	if (i > 9)
		i = 0;

	return res;
}


int tb_tempctrolc(void)
{
	static unsigned char i = 0;
//	static bool is_fancmd_sended = false;
	static unsigned int counter=0;
	unsigned int j = 0, is_alltemp_zero = 0;
	struct timeval now;
	unsigned char recieve_code;
	unsigned char dev_addr;
	emTemp_chopcmd cmd;
	stminibd_sendpacket mini_cmd;
	unsigned int value;
	stTemper_cmdframeunion te_cmd;
	int res = 0;
	static int fan_time = 0;
	static  int internal_time[10];

	gettimeofday(&now, NULL);
	
	if ((temper_control3[i].temp > 0) && (temper_control3[i].state == 0) ) //需加热 且 还没法送加热命令
	{
		te_cmd.devaddr = dev_addr = temper_control3[i].plate_num / 4 + 6;
		te_cmd.chopcmd = cmd = CH1_SV + (temper_control3[i].plate_num % 4) * 3;
		te_cmd.temvalue = value = temper_control3[i].temp;
		printf("dev_addr = %d cmd = %d value = %d\n", dev_addr,cmd,value);
		if (temper_control3[i].temp > 1000)
		{
			NEED_853 = true;
			temper_control3[i].state = IN_85;
		}
		else
		{
			temper_control3[i].state = SEND_TEMP;
		}	
	}

	else if(temper_control3[i].state == IN_85)
	{
	
		temper_control3[i].state = IN_85_TIME;
	}
	else if(temper_control3[i].state == IN_85_TIME)
	{
		
		temper_control3[i].state = SEND_TEMP;

	}
	else if ((temper_control3[i].temp > 0) && (temper_control3[i].state == SEND_TEMP)) //需加热 且 还没到指定温度
	{
		te_cmd.devaddr = dev_addr = temper_control3[i].plate_num / 4 + 6;
		te_cmd.chopcmd = cmd = CH1_SV + (temper_control3[i].plate_num % 4) * 3;
		te_cmd.temvalue = value = temper_control3[i].temp;
		
		internal_time[i] = now.tv_sec;

		printf("dev_addr = %d cmd = %d value = %d\n", dev_addr,cmd,value);
		
		pthread_mutex_lock(&mutex_modbuslock); //发送
		if (tc_temsendcmd(dev_addr, cmd, value) )
		{
			printf("\n[tb_tempctrolc]SendStartCmdTemper sucessi = %d\n\n", i);

			if (!tc_recivtempframe(&recieve_code,&te_cmd))
				printf("[tb_tempctrolc]recieve error_code = %x", recieve_code);	
		}
		else
			printf("tc_temsendcmd failed\n");

		if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_modbuslock");
			
			//向发送PC加热信息
		//	mb_seterrorcode(ATEMPERL);
			
		temper_control3[i].state= SENDED_TEMPERCMD;//已发送加热命令标识	
	}

	else if ((temper_control3[i].temp > 0) && (temper_control3[i].state == SENDED_TEMPERCMD)) //需加热 且 还没到指定温度
	{
		if (now.tv_sec > (internal_time[i] + 600))
		{
			mb_seterrorcode((temper_control1[i].plate_num % 30) + TEMPER1);
			temper_control3[i].state = REACH_TEMP;
		}
		if (!flg_opwork3ready || tem_over_loadC)
			temper_control3[i].state = REACH_TEMP;	
		if (counter > 100 )
		{
			counter = 0;
			printf("temper_control3[%d].temp =%d temp_value[20 + temper_control3[%d].plate_num] = %d\n", i,temper_control3[i].temp, i,temp_value[20 + temper_control3[i].plate_num + 20]);
		}
		counter++;	
		if (temper_control3[i].temp <= (temp_value[20 + temper_control3[i].plate_num] + 20 ))//到达指定温度 10为1度
		{
			temper_control3[i].state = REACH_TEMP;	//到达指定温度后temp标识
			
		//	if (tc_gettemtempvalprintf(i + 1 + 20) > MaxTemp)
		//		res = i + 1;
			if (last_reagentC == NO_REAGENT || last_reagentC == REAGENT_DEWAX)		
					workstep_state_c = workstep_state_c | 0X01000000;
			//printf("\nwhen REACH_TEMP temper_control3[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n",i,temper_control3[i].time,(now.tv_sec - begin_time.tv_sec));
		}
	//	while(!flg_mainproexit);
	}
	else if((temper_control3[i].temp > 0) && temper_control3[i].state == REACH_TEMP )	//等待加热时间到达 并且滴加结束
	{
	
		if (StartDispenseC)
		{
			temper_control3[i].time += now.tv_sec - begin_time.tv_sec;//相对于刚开机时的绝对时间
			//temper_control3[i].state = WAIT_STRECH;
			temper_control3[i].state = START_TIME;
			printf("temper_control3[i].time=%d/n",temper_control3[i].time);
		}
	}
	else if ((temper_control3[i].temp > 0) && temper_control3[i].state == WAIT_STRECH && (workstep_state_c & 0X00FF0000) > 0)
	{
		gettimeofday(&now, NULL);

		printf("temper_control3[i].time=%d\n",temper_control3[i].time);
		{
			#if 0
			if(	flg_opwork3ready &&
							(//(last_reagentA == REAGENT_DEWAX && operate_head1->operate.reagent != REAGENT_DEWAX) ||
							//(last_reagentA == REAGENT_ALCOHOL && operate_head1->operate.reagent != REAGENT_ALCOHOL) ||
							 ( last_reagentC == REAGENT_ER1) ||
							 ( last_reagentC == REAGENT_ER2)//制备结束或热修复结束需要打开风机
							// ( last_reagentA == REAGENT_ER1 && operate_head1->operate.reagent != REAGENT_ER1) ||
							// ( last_reagentA == REAGENT_ER2 && operate_head1->operate.reagent != REAGENT_ER2)//制备结束或热修复结束需要打开风机

							)
						)
				{
					
					if ((temper_control3[i].time - 10*60) <= (now.tv_sec - begin_time.tv_sec) && !flg_opwork3ready)	//加热时间到达
					{					
						bs_packetshelfstreach(50,&mini_cmd);
						pthread_mutex_lock(&mutex_mlock);
						set_minicmd(pcmd_head,mini_cmd);	
						if (pthread_mutex_unlock(&mutex_mlock) != 0)
							lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
						
						temper_control3[i].state = START_TIME;
					}
				}
				else
			#endif
				temper_control3[i].state = START_TIME;
		}
	}
	
	else if ((temper_control3[i].temp > 0) && temper_control3[i].state == START_TIME )
	{

		if (temper_control3[i].time <= (now.tv_sec - begin_time.tv_sec)  || !flg_opwork3ready)	//加热时间到达
		{
			#if(USE_PRINT_LOG ==  1)
			printf("\nwhen REACH_TTIME temper_control3[%d].time = %d(now.tv_sec - begin_time.tv_sec)=%d\n\n", i, \
					temper_control3[i].time, (now.tv_sec - begin_time.tv_sec));
			#endif
			dev_addr = temper_control3[i].plate_num / 4 + 6;
			cmd = CH1_SV + (temper_control3[i].plate_num % 4) * 3;
			value = 0;
			te_cmd.devaddr = dev_addr;
			te_cmd.chopcmd = cmd;
			te_cmd.temvalue = value;
			pthread_mutex_lock(&mutex_modbuslock);	//发送
			if (tc_temsendcmd(dev_addr, cmd, value) )
			{
				printf("[tb_tempctrolc]SendStopCmdTemper sucess i= %d\n", i);
				if (!tc_recivtempframe(&recieve_code,&te_cmd))
					printf("[tb_tempctrolc]recieve error_code = %x", recieve_code);	
			}
			else
				printf("tc_temsendcmd failed\n");
			if (pthread_mutex_unlock(&mutex_modbuslock) != 0)
							lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_modbuslock");
			
			temper_control3[i].temp = 0;//不用加热标识
			temper_control3[i].state = END_TIME;
			printf("[temper_control3]all temp\n");
			for (j = 0; j < 10; j++)
			{
				printf(" %d ", temper_control3[j].temp);
				if (temper_control3[j].temp > is_alltemp_zero)
					is_alltemp_zero = temper_control3[j].temp;
			}
			if (is_alltemp_zero == 0) 
			{
				StartDispenseC = false;
			
				if (flg_opwork3ready &&
							(//(last_reagentC == REAGENT_DEWAX && operate_head3->operate.reagent != REAGENT_DEWAX) ||
							
						//	( last_reagentC == REAGENT_ER1 && operate_head3->operate.reagent == REAGENT_ER1 && operate_head3->operate.next->reagent != REAGENT_ER1) ||
						//	( last_reagentC == REAGENT_ER2 && operate_head3->operate.reagent == REAGENT_ER2 && operate_head3->operate.next->reagent != REAGENT_ER2)//制备结束或热修复结束需要打开风机
							(NEED_853 &&  last_reagentC == REAGENT_ER1 && operate_head3->operate.reagent != REAGENT_ER1) ||
							 (NEED_853 &&  last_reagentC == REAGENT_ER2 && operate_head3->operate.reagent != REAGENT_ER2) //制备结束或热修复结束需要打开风机
							//	(NEED_853 && (operate_head3->operate.next->reagent == REAGENT_ER1 || operate_head3->operate.next->reagent == REAGENT_ER2))
							)
					)
                {
                    NEED_853 = false;
                        /*
                        bs_packetshelfstreach(20,&mini_cmd);
                        pthread_mutex_lock(&mutex_mlock);
                        set_minicmd(pcmd_head,mini_cmd);	
                        if (pthread_mutex_unlock(&mutex_mlock) != 0)
                            lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");

                    while(mini_work_finished[2])//确保miniBoard线程已置FALSE
                    usleep(1000);
                    sleep(1);
                    while(!mini_work_finished[2])//动作完成
                        usleep(200000);
                    */
                
                    //发送降温命令给minb
                    fan_time = now.tv_sec;
                    memset(&mini_cmd,0,sizeof(mini_cmd));
                    mini_cmd.cmd = MAINTAIN_FAN_WORK;
                    mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
                    mini_cmd.minicmd_buffer[4] = 3; //ON
                    mini_cmd.minicmd_num=2;
                    pthread_mutex_lock(&mutex_mlock);
                    set_minicmd(pcmd_head,mini_cmd	);	
                    pthread_mutex_unlock(&mutex_mlock);
                    
                    temper_control3[0].state = START_FAN;
                }
                else
                {
                    pthread_mutex_lock(&head_step_lock);
                    workstep_state_c = workstep_state_c | 0X01010000;
                    if (pthread_mutex_unlock(&head_step_lock) !=0)
                        lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
                    TemperControl_p[2] = NULL;
                    flg_temperheating3 = false;		
                }
			}
		}
	}
	else if (temper_control3[0].state == START_FAN)
	{
		if (  (now.tv_sec - fan_time) > 5*60 ||  !flg_opwork3ready)
		{
			pthread_mutex_lock(&head_step_lock);
			workstep_state_c = workstep_state_c | 0X01010000;
			if (pthread_mutex_unlock(&head_step_lock) !=0 )
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
			TemperControl_p[2] = NULL;
			flg_temperheating3 = false;
			temper_control3[0].state = END_TIME;

			memset(&mini_cmd,0,sizeof(mini_cmd));
			mini_cmd.cmd = MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;
			mini_cmd.minicmd_buffer[4] = 6; //OFF	
			mini_cmd.minicmd_num=2;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);	
			pthread_mutex_unlock(&mutex_mlock);			
		}
	}
	i++;
	if (i > 9)
		i = 0;

	return res;
}






void tempertst(void)
{
	int i = 0, j = 0;
	int tempernum = 30;
	short wtemvalue[30] = {0};
	short rtemvalue[30] = {0};

	while(1)
	{
		//100搴?0鍒嗛挓	
		for(i = 0; i < tempernum; i++)
		{
			wtemvalue[i] = 800;
			//wtemvalue[i] =i+1;
			
			tp_sendtemperframe(i, SET, 1, wtemvalue[i], (int16_t*)&rtemvalue[i]);
		}

		for(i=0; i<tempernum; i++)
			tp_sendtemperframe(i, SET, 0, 0, (int16_t*)&temp_value[i]);
		printf("rtemvalue=");
		for(i = 0; i < tempernum; i++)
			printf(" %d ", temp_value[i]);
		//while(1) sleep(1);
		j = 0;
		while(1)
		{
			sleep(5);
			for(i=0; i<tempernum; i++)
				tp_sendtemperframe(i, CUR1, 0, 0, (int16_t*)&temp_value[i]);
			get_temvale = 1;
			printf("jjjjjjjjjj=%d\n",j);
			printf("temp_value=");
			for(i = 0; i < tempernum; i++)
			printf(" %d ",temp_value[i]);
			//	if (j++ >= 4*60)
			//	break;
		}	
		wtemvalue[0] = 0;
		for(i = 0; i < tempernum; i++)
			tp_sendtemperframe(i, SET, 1, wtemvalue[0], (int16_t*)&rtemvalue[i]);
		sleep(3*60);
							
			//72搴?鍒嗛挓

		for(i = 0; i < tempernum; i++)
		{
			wtemvalue[i] = 720;
			tp_sendtemperframe(i, SET, 1, wtemvalue[i], (int16_t*)&rtemvalue[i]);
		}
		j = 0;
		while(1)
		{
			sleep(1);
			for(i = 0; i < tempernum; i++)
				tp_sendtemperframe(i, CUR1, 0, 0, (int16_t*)&temp_value[i]);
			get_temvale = 1;
			if (j++ >= 1*60)
				break;
		}		
		wtemvalue[0] = 0;
		for(i = 0; i < tempernum; i++)
			tp_sendtemperframe(i, SET, 1, wtemvalue[0], (int16_t*)&rtemvalue[i]);
						
		sleep(2*60);
								
		//60搴?0鍒嗛挓

		for(i = 0; i < tempernum; i++)
		{
			wtemvalue[i] = 600;
			tp_sendtemperframe(i, SET, 1, wtemvalue[i], (short*)&rtemvalue[i]);
		}
		j = 0;
		while(1)
		{
			sleep(1);
			for(i = 0; i < tempernum; i++)
				tp_sendtemperframe(i, CUR1, 0, 0, (short*)&temp_value[i]);
			get_temvale = 1;
			if (j++ >= 4*60)
				break;
		}	
		wtemvalue[0] = 0;
		for(i = 0; i < tempernum; i++)
			tp_sendtemperframe(i, SET, 1, wtemvalue[0], (short*)&rtemvalue[i]);	
		sleep(3*60);

		//35搴?鍒嗛挓
		pthread_mutex_lock(&mutex_modbuslock);
		for(i = 0; i < tempernum; i++)
		{
			wtemvalue[i] = 350;
			tp_sendtemperframe(i, SET, 1, wtemvalue[i], (short*)&rtemvalue[i]);
		}
		pthread_mutex_unlock(&mutex_modbuslock);
		j = 0;
		while(1)
		{
			sleep(1);
			pthread_mutex_lock(&mutex_modbuslock);
			for(i = 0; i < tempernum; i++)
				tp_sendtemperframe(i, CUR1, 0, 0, (short*)&temp_value[i]);
			pthread_mutex_unlock(&mutex_modbuslock);
			get_temvale = 1;
			if (j++ >= 1*60)
				break;
		}
		wtemvalue[0] = 0;
		pthread_mutex_lock(&mutex_modbuslock);
		for(i = 0; i < tempernum; i++)
			tp_sendtemperframe(i, SET, 1, wtemvalue[0], (short*)&rtemvalue[i]);
		pthread_mutex_unlock(&mutex_modbuslock);

		sleep(60);
	}
}


int tb_temperinitnew(void)
{
	int i = 0,j=0,er_cnt=0;
	int tempernum = TEMPER_NUM;
	short wtemvalue[TEMPER_NUM] = {0};
	short rtemvalue[TEMPER_NUM] = {0};
	
	char er_counter[TEMPER_NUM] = {0};
	stminibd_sendpacket mini_cmd;

	printf("in [tb_temperinitnew]\n");

	for(i = 0; i < tempernum; i++)
		tp_sendtemperframe(i, CUR1, 0, 0, (short*)&temp_value[i]);
	
	for(i = 0; i < tempernum; i++)
		printf("temp_value=%d\n", temp_value[i]);

	//寮?鍚懡浠わ紝闃叉娓╁害涓嶅彈鎺у叧闂鑷村垵濮嬪寲涓嶅姞鐑?

	//鍔犵儹80搴?鍒嗛挓娓╁害瓒呰繃闄愬畾鍊?娆′互涓婃姤閿?

	if (wkevent == MAINTAIN_WORK)
		return TRUE;

	for(i = 0; i < tempernum; i++)
	{
		wtemvalue[i] = 800 + temp_Dvalmy[i][80];     // 开机设置30片加热器升温温度 80度 + 修正温度值
		tp_sendtemperframe(i, SET, 1, wtemvalue[i], (short*)&rtemvalue[i]);
	}

	printf("in sleep(30 * 6)\n");
	j = 0;
	while(1)
	{
		sleep(30);
		if (wkevent == MAINTAIN_WORK)
		{
			wtemvalue[0] = 0;
			for(i = 0; i < tempernum; i++)
				tp_sendtemperframe(i, SET, TEMPER_WR, wtemvalue[0], &rtemvalue[i]);   // 写设置温度
			
			return TRUE;
		}
		for(i = 0; i < tempernum; i++)
			tp_sendtemperframe(i, CUR1, TEMPER_RD, 0, (short *)&temp_value[i]);     //  读当前温度
		
		printf("all temper value\n");
		for(i = 0; i < tempernum; i++)
			printf(" %d ", temp_value[i]);
		for(i = 0; i < tempernum; i++)
		{	
			if (temp_value[i] > tem_limit)//比较传感器固定
				er_counter[i]++;	
		}		
		get_temvale = TRUE;
		
		if (j++ >= 5)
			break;
	}

	for(i = 0; i < tempernum; i++)
	{
		if (er_counter[i] >= 5)//比较传感器固定
		{
			er_counter[i] = 0;
			//鍏抽棴姝ゅ姞鐑揩
			mb_seterrorcode(i + TEMPER1); 
		}
	}

	for(i = 0; i < tempernum; i++)
	{
		tp_sendtemperframe(i, CUR1, TEMPER_RD, wtemvalue[0], (short*)&temp_value[i]);

		printf("temp_value=%d\n", temp_value[i]);
	}


	for(i = 0; i < tempernum; i++)		//计算是否超过正负5度
	{
		if ((temp_value[i] > (800 + temp_Dvalmy[i][80] + 50)) || ((temp_value[i] < 800 + temp_Dvalmy[i][80] - 50)))
		{
			#if(USE_LOG_INFO == 1)
			printf("might error num=%d, val=%d\n", i, temp_value[i]);
			lprintf(log_my, ERROR, "temper [%d] temp=[%d], is error.\n", i, temp_value[i]);
			#endif

			for (er_cnt = 0; er_cnt < 5; er_cnt++)
			{
				tp_sendtemperframe(i, CUR1, TEMPER_RD, wtemvalue[0], &rtemvalue[0]);
				if ((rtemvalue[0] > (800 + temp_Dvalmy[i][80] - 50)) && ((rtemvalue[0] < 800 + temp_Dvalmy[i][80] + 50)))
					break;
				sleep(1);
			}
					
			if (er_cnt >= 5)
			{
		  		printf("some heater error num=%d\n", i);
				//鍏抽棴姝ゅ姞鐑揩	
				mb_seterrorcode(i + TEMPER1); 
			}
			//	res  = FALSE;//初始化允许有错误加热块
		}
	}

	//璇绘瘮杈冧紶鎰熷櫒鍊艰秴杩?
	for(i = 0; i < tempernum; i++)
	{
		tp_sendtemperframe(i, CUR2, TEMPER_RD, wtemvalue[0], &rtemvalue[i]);//第一次读采样后更新数据
		tp_sendtemperframe(i, CUR2, TEMPER_RD, wtemvalue[0], &rtemvalue[i]);//第二次读取数据
	}
	
	if (rtemvalue[i] > tem_limit)
	{
		for (er_cnt = 0; er_cnt < 5; er_cnt++)
		{
			tp_sendtemperframe(i, CUR2, 0,wtemvalue[0],&rtemvalue[i]);
			tp_sendtemperframe(i, CUR2, 0,wtemvalue[0],&rtemvalue[i]);
			if (rtemvalue[i] < tem_limit)
				break;
			sleep(1);
		}
	
		if (er_cnt >= 5)
		{
			tp_sendtemperframe(i, CUR1, 0,wtemvalue[0],&rtemvalue[i]);
			if (rtemvalue[i] > tem_limit)
			{
				printf("some heater TEMPER_EM error num=%d\n",i);
				//鍏抽棴姝ゅ姞鐑揩
				mb_seterrorcode(i + TEMPER1_EM); 
			}
		}
	}

	wtemvalue[0] = 0;
	for(i = 0; i < tempernum; i++)
		tp_sendtemperframe(i, SET, 1,wtemvalue[0],&rtemvalue[i]);

	//发送降温命令给minb
	memset(&mini_cmd,0,sizeof(mini_cmd));
	mini_cmd.cmd = MAINTAIN_FAN_WORK;
	mini_cmd.minicmd_buffer[3] = (unsigned char)MAINTAIN_FAN_WORK;	
	
	mini_cmd.minicmd_num=2;
	
	mini_cmd.minicmd_buffer[4] = 1;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	
	mini_cmd.minicmd_buffer[4] = 2;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR, "%s","pthread_mutex_unlock error mlock");
	
	mini_cmd.minicmd_buffer[4] = 3;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);	
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR, "%s","pthread_mutex_unlock error mlock");
			
	for (j = 0; j < 36; j++)
	{
		sleep(5);
		for(i = 0; i < tempernum; i++)
			tp_sendtemperframe(i, CUR1, 0, wtemvalue[0], (int16_t*)&temp_value[i]);
		get_temvale = TRUE;
		if (wkevent == MAINTAIN_WORK)
		{
			mini_cmd.minicmd_buffer[4] = 4;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			mini_cmd.minicmd_buffer[4] = 5;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			mini_cmd.minicmd_buffer[4] = 6;
			pthread_mutex_lock(&mutex_mlock);
			set_minicmd(pcmd_head,mini_cmd);
			if (pthread_mutex_unlock(&mutex_mlock) != 0)
				lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
			return TRUE;
		}
	}
	mini_cmd.minicmd_buffer[4] = 4;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	mini_cmd.minicmd_buffer[4] = 5;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	mini_cmd.minicmd_buffer[4] = 6;
	pthread_mutex_lock(&mutex_mlock);
	set_minicmd(pcmd_head,mini_cmd);
	if (pthread_mutex_unlock(&mutex_mlock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
	if (wkevent == MAINTAIN_WORK)
		return TRUE;

	return TRUE;
}




