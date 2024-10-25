


#include "../caculateProceTime/caculteprocetime.h"
#include "sys/time.h"
#include "string.h"
#include "pthread.h"
#include "netbuss.h"
#include "log.h"


extern  operate_head_list* operate_head1;//A架操作头指针
extern  operate_head_list* operate_head2;//B架操作头指针
extern	operate_head_list* operate_head3;//C架操作头指针
extern  pthread_mutex_t head_lock;//操作数据结构锁
extern ewkevent wkeventA;	//A架玻片工作状态
extern  ewkevent wkeventB;//B架玻片工作状态
extern  ewkevent wkeventC;//C架玻片工作状态
extern volatile unsigned char last_reagentA;//A架玻片最近一次滴加的试剂
extern volatile unsigned char last_reagentB;
extern volatile unsigned char last_reagentC;
extern volatile bool isDAB_mixedA_next; //A架DAB是否混合标识
extern volatile bool isDAB_mixedB_next;
extern volatile bool isDAB_mixedC_next;
extern volatile unsigned int shelf_stirtime[3][3];
extern volatile char shelf_stirmode[3];
extern volatile unsigned int workstep_state_a;	//记录操作步数 最高字节置1表示温度加热完成，次高字节置1表示滴加动作完成 最低字节表示步数 第5个1表示时间到达
extern volatile unsigned int workstep_state_b;
extern volatile unsigned int workstep_state_c; 
extern unsigned int head1_start_time;
extern unsigned int head2_start_time;
extern unsigned int head3_start_time; 
extern  temper_control_t temper_control1[10];	//温度控制全局变量
extern temper_control_t temper_control2[10];	//温度控制全局变量
extern temper_control_t temper_control3[10];	//温度控制全局变量
extern volatile bool isDAB_mixedA;//A架DAB是否混合标识
extern volatile bool isDAB_mixedB;
extern volatile bool isDAB_mixedC;

time_cal_t work_timeA,work_timeB,work_timeC;
struct timeval begin_time;//程序开始运行时间



int ct_calateimeonehshelftime(operate_head_list* operate_head_p, unsigned char last_reagent,
	time_cal_t* work_time)
{
	char reagent_shelf_cnt = 0,stp_cnt = 0;
	bool have_dewax = false, have_ER = false;
	operate_t* operate_p;
	struct timeval now;

	if (operate_head_p->operate.reagent == STOP_OPERATE)//刚好运行到最后一步
		return 0;

	operate_p = &(operate_head_p->operate) ;
	while(operate_p->next->reagent != STOP_OPERATE)//轮询一片玻片
	{
		//试剂切换时间
		if (operate_p->reagent != operate_p->next->reagent)
		{
			if (operate_p->next->reagent == REAGENT_WATER || 
				operate_p->next->reagent == REAGENT_WASH)
				work_time->exchange_reagent_time += 35;
			else if (operate_p->next->reagent > REAGENT_WASH && 
				operate_p->next->reagent <= REAGENT_ER2)
				work_time->exchange_reagent_time += 30;
			else
				work_time->exchange_reagent_time += 15;
		}

		//试剂架试剂统计
		if(operate_p->reagent < 37 || operate_p->reagent >= REAGENT_DAB)
			reagent_shelf_cnt++;

		if (operate_p->reagent == REAGENT_DEWAX)
			have_dewax = true;
		if (operate_p->reagent == REAGENT_ER1 || operate_p->reagent == REAGENT_ER2)
			have_ER = true;			
		
		//加热温度时间
		if (operate_p->temp >= 90)
			work_time->tem_time += 60 * 7;			
		else if (operate_p->temp >= 70)
			work_time->tem_time += 60;
		else if (operate_p->temp >= 30)
			work_time->tem_time += 30;
	
		stp_cnt++;
		operate_p = operate_p->next;
	}
	//	printf("first_cac&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
		//风机时间
	work_time->fan_time = 5 * 60 * (have_ER);
		//全部孵育时间
	gettimeofday(&now, NULL);
	while(operate_head_p != NULL)	//轮询一架玻片
	{			
		operate_p = &(operate_head_p->operate);
		while(operate_p->next->reagent != STOP_OPERATE)//轮询一片玻片
		{
			if (operate_p->time > work_time->max_time)//修复时间可以不同取最大
				work_time->max_time = operate_p->time;
			operate_p = operate_p->next;
		}
		work_time->slide_cnt++;
		operate_head_p = operate_head_p->next_head;
	}
	
	work_time->max_time -= now.tv_sec - begin_time.tv_sec;
		
	//	if (work_time->slide_cnt >5)//试剂架试剂清洗
	//		work_time->reagent_shelf_time = reagent_shelf_cnt * 15;
				
		work_time->mov_time = work_time->slide_cnt * stp_cnt * 3;
	//	work_time->discharge_time= stp_cnt * 7 + stp_cnt * 5;//排液和风机时间

	printf("work_time->exchange_reagent_time=%d ,work_time->reagent_shelf_time=%d,work_time->tem_time=%d.work_time->max_time=%d , work_time->fan_time=%d,work_time->mov_time=%d,work_time->discharge_time=%d\n",
	work_time->exchange_reagent_time,work_time->reagent_shelf_time,work_time->tem_time,
	work_time->max_time, work_time->fan_time,work_time->mov_time, work_time->discharge_time);

	return work_time->exchange_reagent_time + work_time->reagent_shelf_time + work_time->tem_time + 
		work_time->max_time + work_time->fan_time + work_time->mov_time + work_time->discharge_time;	
}

//指定时间中查出步数
int ct_getdesttimestp(operate_head_list* comp_head_p,int des_time)
{
	operate_t* operate_p;
	int stp_cnt = 0;
	struct timeval now;
	
	gettimeofday(&now, NULL);
	//printf("des_time=%d,",des_time);	
	operate_p = &(comp_head_p->operate);
	while(operate_p->reagent != STOP_OPERATE)//轮询一片玻片
	{
		//		printf("operate_p->time=%d\n",operate_p->time);
		if ((operate_p->time - (now.tv_sec - begin_time.tv_sec)) >= des_time)//修复时间可以不同取最大
		{
			printf("stp_cnt=%d\n",stp_cnt);
			return stp_cnt;
		}
		stp_cnt++;	
		operate_p = operate_p->next;
	}

	printf("stp_cnt=%d\n",stp_cnt);

	return stp_cnt;
}


int ct_patchstptime(const time_cal_t des_time, const time_cal_t comp_time1,
	const time_cal_t comp_time2, operate_head_list* comp_head_p1, operate_head_list* comp_head_p2)
{
	int all_time_add1 = 0,all_time_add2 = 0;

	printf("des_time.all_time=%d,comp_time1.all_time=%d,comp_time2.all_time=%d, comp_time1.slide_cnt=%d, comp_time2.slide_cnt=%d\n",
		des_time.max_time,comp_time1.max_time,comp_time2.max_time, comp_time1.slide_cnt, comp_time2.slide_cnt);
	if (des_time.max_time <= comp_time1.max_time)
		all_time_add1 = ct_getdesttimestp(comp_head_p1,des_time.max_time) *
							(comp_time1.slide_cnt *3 + 7);
	else if (comp_time1.max_time != 0)
		all_time_add1 = comp_time1.mov_time +  comp_time1.discharge_time;

	if (des_time.max_time <= comp_time2.max_time)
		all_time_add2 =  ct_getdesttimestp(comp_head_p2,des_time.max_time) *( comp_time2.slide_cnt *3 + 7);
	else if (comp_time2.max_time != 0)
		all_time_add2 = comp_time2.mov_time + comp_time2.discharge_time;

	printf("all_time_add1=%d,all_time_add2=%d^^^^^^\n",all_time_add1,all_time_add2);	
	return all_time_add1 + all_time_add2;
}


/*****************运行步骤前检测当前时间是否超过此步骤的运行时间 超过则补起返回TRUE*********************
******************超时原因:1当仪器执行加热操作时时间超出预设值；2当仪器执行滴加操作时间过长*******/
bool ct_patchtime(operate_head_list* operate_head)
{
	unsigned int  maxtime = 0;
	int patch_time = 0;
	unsigned char i = 0;
	operate_head_list* operate_head_p = NULL;
	operate_t* operate_p = NULL;
	struct timeval now;
	bool res = false;
	unsigned int *head_step = NULL;
	unsigned int head_start_time = 0;
	char have_mix_reangent1st = 0, have_mix_reangent2sd = 0;
	char temper_state = 0;
	
	if(operate_head == NULL){
		return false;
	}
	gettimeofday(&now, NULL);
	
	//执行搅拌动作
	for(i = 0; i < 3; i++)
	{
		#if (USE_LOG_INFO == 1)
		{
			lprintf(log_my, INFO, "shelf_stirtime[%d][0]=%d shelf_stirtime[%d][1]=%d shelf_stirtime[%d][2]=%d\n", i, shelf_stirtime[i][0],\
					 i, shelf_stirtime[i][1], i, shelf_stirtime[i][2]);
			printf("shelf_stirtime[%d][0]=%d shelf_stirtime[%d][1]=%d shelf_stirtime[%d][2]=%d\n", i, shelf_stirtime[i][0],\
					 i, shelf_stirtime[i][1], i, shelf_stirtime[i][2]);
		}
		#endif
		printf("\nin ct_patchtime work[%d]\n", i);

		if(shelf_stirtime[i][0] > 0 && shelf_stirtime[i][0] <= now.tv_sec - begin_time.tv_sec)
		{
			shelf_stirmode[i] = 3;   //  盖片搅拌3次
			shelf_stirtime[i][0] = 0;
		}

		if(shelf_stirtime[i][1] > 0 && shelf_stirtime[i][1] <= now.tv_sec - begin_time.tv_sec)
		{
			shelf_stirmode[i] = 3;
			shelf_stirtime[i][1] = 0;
		}
		if(shelf_stirtime[i][2] > 0 && shelf_stirtime[i][2] <= now.tv_sec - begin_time.tv_sec)
		{
			shelf_stirmode[i] = 3;
			shelf_stirtime[i][2] = 0;
		}
	}
	printf("opearte_head=iiuuu1");
	  printf("[ct_patchtime]operate_head->operate.time = %d\n", operate_head->operate.time);

	if (operate_head->operate.next == NULL && operate_head->operate_work_time == 0)  //动作结束
	{
		printf("operate_head->operate.next == NULL && operate_head->operate_work_time == 0.\n");
		return res;
	}
	printf("OPERATEHEAD head_step = (unsigned int*)&workstep_state_a=[%d].\n",  (unsigned int*)&workstep_state_a);
	if (operate_head == operate_head1)
	{
		printf("opearte_head=head1");
		head_step = (unsigned int*)&workstep_state_a;
		head_start_time = head1_start_time;
		temper_state = temper_control1[0].state;
	}
	else if (operate_head == operate_head2)
	{
		printf("opearte_head=head2");
		head_step = (unsigned int*)&workstep_state_b;
		head_start_time = head2_start_time;
		temper_state = temper_control2[0].state;
	}
	else if (operate_head == operate_head3)
	{
		printf("opearte_head=head3");
		head_step = (unsigned int*)&workstep_state_c;
		head_start_time = head3_start_time;
		temper_state = temper_control3[0].state;
	}
	else{
		printf("cal time operate_head is NULL.\n");
		return false;
	} 
	printf("cal time next\n");

	operate_head_p = operate_head;
	while (operate_head_p != NULL)
	{
		maxtime = maxtime < operate_head_p->operate.time ? operate_head_p->operate.time : maxtime;
		operate_head_p = operate_head_p->next_head;
	}

	gettimeofday(&now, NULL);
	#if (USE_LOG_INFO == 1)
	printf("now.tv_sec - begin_time.tv_sec=%d maxtime = %d operate_p->time=%d\n",
				(int)(now.tv_sec - begin_time.tv_sec), maxtime, operate_head->operate.time);
	#endif
							//max补齐	
	operate_head_p = operate_head;
	
	while (operate_head_p != NULL)
	{	
		patch_time = maxtime - operate_head_p->operate.time;	
		operate_p = &(operate_head_p->operate);
		while (operate_p != NULL)
		{
			operate_p->time += patch_time;       //将执行操作时间补齐	
			operate_p = operate_p->next;
		}
		operate_head_p = operate_head_p->next_head;
	}

	#if (USE_PRINT_LOG == 1)
	printf("*head_step=%x operate_head->operate_work_time=%d temper_state=%d operate_p->time=%d\n",
				*head_step,operate_head->operate_work_time, temper_state, operate_head->operate.time);
	#endif
	if ( (temper_state < REACH_TEMP))//滴加未结束时间全部推移 //温度加热是否完成以在ExcuteOperate 中执行
	{
		operate_head_p = operate_head;
		while (operate_head_p != NULL)
		{		
			if (operate_head_p->operate.time > (now.tv_sec - begin_time.tv_sec))
			{
				patch_time = operate_head_p->operate_work_time - 
				(operate_head_p->operate.time - (now.tv_sec - begin_time.tv_sec) );
			}
			else
			{
				patch_time = operate_head_p->operate_work_time +
				( (now.tv_sec - begin_time.tv_sec) - operate_head_p->operate.time );
			}
			operate_p = &(operate_head_p->operate);
			//	printf("operate_head_p->operate.time=%d, patch_time=%d operate_p->time=%d\n", 
			//	operate_head_p->operate.time,patch_time,operate_head->operate.time);
			while (operate_p != NULL)
			{
				operate_p->time += patch_time;
				operate_p = operate_p->next;	
			}
			operate_head_p = operate_head_p->next_head;
		}
		res = true;
	}
	/*
	else if (operate_head->operate.time <= (now.tv_sec - begin_time.tv_sec))
	{
		pthread_mutex_lock(&head_step_lock);
		*head_step = (*head_step) | 0X00010000;
		if (pthread_mutex_unlock(&head_step_lock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_step_lock");
		operate_head->operate_work_time = 0;
	}
	*/
	#if (USE_LOG_INFO == 1)
		printf("operate_p->time=%d ",operate_head->operate.time);
	#endif
	if ((now.tv_sec - begin_time.tv_sec) >= maxtime)								//now.tv_sec - begin_time.tv_sec补齐
	{
		operate_head_p = operate_head;
		while (operate_head_p != NULL)
		{		
			patch_time = now.tv_sec - begin_time.tv_sec - operate_head_p->operate.time;
			operate_p = &(operate_head_p->operate);
			//	printf("operate_head_p->operate.time=%d, patch_time=%d\n", operate_head_p->operate.time,patch_time);
			while (operate_p != NULL)
			{
				operate_p->time += patch_time;
				operate_p = operate_p->next;
			}
			operate_head_p = operate_head_p->next_head;
		}
		res = true;
	}
	#if (USE_LOG_INFO == 1)
		printf("222operate_p->time=%d ",operate_head->operate.time);
	#endif

	operate_p = &(operate_head->operate);
	while (operate_p != NULL)		//找出滴加混合试剂的时间
	{
		if ( (operate_p->reagent & 0x7F) == REAGENT_DAB && !have_mix_reangent1st)
		{
			if (operate_head == operate_head1 && !isDAB_mixedA)
				dispense_mixreagent_timeA = operate_p->time;
			if (operate_head == operate_head2 && !isDAB_mixedB)
				dispense_mixreagent_timeB = operate_p->time;
			if (operate_head == operate_head3 && !isDAB_mixedC)
				dispense_mixreagent_timeC = operate_p->time;
			have_mix_reangent1st = 1;
		//	break;
		}
		//	printf("have_mix_reangent2sd=%d\n", have_mix_reangent2sd);
		if ( (operate_p->reagent & 0x7F) == REAGENT_SECEND && !have_mix_reangent2sd)
		{
			if (operate_head == operate_head1 && !isDAB_mixedA_next)
				dispense_mixreagent_timeA_next = operate_p->time;
			if (operate_head == operate_head2 && !isDAB_mixedB_next)
				dispense_mixreagent_timeB_next = operate_p->time;
			if (operate_head == operate_head3 && !isDAB_mixedC_next)
				dispense_mixreagent_timeC_next = operate_p->time;
			have_mix_reangent2sd = 1;
		//	break;
		}
		
		operate_p = operate_p->next;
	}
	if(have_mix_reangent1st == 0)//没有混合试剂
	{
		if (operate_head == operate_head1 )
				dispense_mixreagent_timeA = 0;
		if (operate_head == operate_head2 )
				dispense_mixreagent_timeB = 0;
		if (operate_head == operate_head3 )
				dispense_mixreagent_timeC = 0;
	}
	if(have_mix_reangent2sd == 0)//没有混合试剂
	{
		if (operate_head == operate_head1 )
				dispense_mixreagent_timeA_next = 0;
		if (operate_head == operate_head2 )
				dispense_mixreagent_timeB_next = 0;
		if (operate_head == operate_head3 )
				dispense_mixreagent_timeC_next = 0;
	}
	
//	mb_printfoperatelist(operate_head);
	printf("\nPatchTime work finished\n");
	return res;
}


//只在开始运行计算其中一架运行其他几架重新计算
int ct_reportcaculateprocetime(void)
{
	char sendbuf[50] = {0};
	int all_timeA_tmp = 0, all_timeB_tmp = 0, all_timeC_tmp = 0;

	memset(&work_timeA, 0, sizeof(time_cal_t));
	memset(&work_timeB, 0, sizeof(time_cal_t));
	memset(&work_timeC, 0, sizeof(time_cal_t));
	
	pthread_mutex_lock(&head_lock);
	ct_patchtime(operate_head1);//
	ct_patchtime(operate_head2);//
	ct_patchtime(operate_head3);//
	if (wkeventA == BUSY_WORK)
		work_timeA.all_time = all_timeA_tmp = ct_calateimeonehshelftime(operate_head1, last_reagentA, &work_timeA);
	if (wkeventB == BUSY_WORK)
		work_timeB.all_time = all_timeB_tmp = ct_calateimeonehshelftime(operate_head2, last_reagentB, &work_timeB);
	if (wkeventC == BUSY_WORK)
		work_timeC.all_time = all_timeC_tmp = ct_calateimeonehshelftime(operate_head3, last_reagentC, &work_timeC);
			//运行时全部架子的时间根据超过5min的步数进行移液时间的统计
	/*
	if (wkeventA == BUSY_WORK)
		work_timeA.all_time =all_timeA_tmp + ct_patchstptime(work_timeA,work_timeB,work_timeC,operate_head2,operate_head3);
		
	if (wkeventB == BUSY_WORK)
		work_timeB.all_time =all_timeB_tmp +  ct_patchstptime(work_timeB,work_timeA,work_timeC,operate_head1,operate_head3);

	if (wkeventC == BUSY_WORK)
		work_timeC.all_time =all_timeC_tmp +  ct_patchstptime(work_timeC,work_timeA,work_timeB,operate_head1,operate_head2);
	*/
	if (pthread_mutex_unlock(&head_lock) != 0)
		lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");

	#if(USE_LOG_INFO == 1)
	printf("all_timeA=%d,all_timeB=%d,all_timeC=%d***********************\n",
		work_timeA.all_time,work_timeB.all_time,work_timeC.all_time);
	#endif
	
	memcpy(((unsigned char *)&sendbuf[0]), (void*)&(work_timeA.all_time), 4);
	memcpy(((unsigned char *)&sendbuf[4]), (void*)&(work_timeB.all_time), 4);
	memcpy(((unsigned char *)&sendbuf[8]), (void*)&(work_timeC.all_time), 4);
		
	nt_sendpacketdata(ESTIMATE_TIME, sendbuf, 12);

	return 0;
}




/******************************************************************************
*
* Function Name  : ct_addtabtime
* Description    : .给操作加上时间戳在pc发送operate结束后调用
* 					 
* Input		   : operate_head_list* 
* Output		   : None
* Return		   :  None
*******************************************************************************/
void ct_addtabtime(operate_head_list* new_head, int time)	//需要pc端先把时间叠加,上位机已经吧时间加在operatetime 无需再加
{
	struct timeval now;
	int add_time = 0;
	operate_t* operate_p = NULL;
	operate_head_list* operate_head_p = new_head;

	if (!new_head)
		return;
	
	gettimeofday(&now, NULL);	
	add_time = now.tv_sec - begin_time.tv_sec;

	printf("in ct_addtabtime time=%d add_time=%d\n", time, add_time);

	while(operate_head_p != NULL)
	{
		operate_p = &(operate_head_p->operate);
		while(operate_p != NULL)
		{
			operate_p->time += add_time;
			operate_p = operate_p->next;
		}
		operate_head_p = operate_head_p->next_head;
	}
}


void ct_readstarttime(void)
{
	#if(CAL_TIME_NEWCAL != 1)
	gettimeofday(&begin_time, NULL);
	#endif
}

