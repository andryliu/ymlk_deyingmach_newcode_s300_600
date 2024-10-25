

#include "serailbussfile.h"
#include "pthread.h"
#include "string.h"
#include "serailminibd.h"
#include <unistd.h>
#include <termios.h>
#include "midbus.h"
#include "log.h"
#include "buss.h"
#include "mbcrc.h"


extern int thread_id_miniboard ;
unsigned int load_motor[5];

extern volatile bool flg_mainproexit;


extern pthread_mutex_t mutex_mlock;
extern pthread_mutex_t mutex_modbuslock;
extern pthread_mutex_t minib_send_lock;
extern volatile stminibd_cmdlist* pcmd_head;
extern volatile stminibd_cmdlist* pcmd_head2;
extern volatile stminibd_cmdlist* pcmd_head3;

extern volatile char SHELF_LOCK_STATE[3];
extern ewkevent wkeventA;  //A架玻片工作状态
extern ewkevent wkeventB;    //B架玻片工作状态
extern ewkevent wkeventC;
extern volatile hydrate_t hydrateA;//A架玻片水合操作链表
extern volatile hydrate_t hydrateB;
extern volatile hydrate_t hydrateC;


extern volatile bool flg_shelfcanindex[3];
extern volatile bool flg_opwork1ready;	//玻片架1正在工作标识	用于结束时确定发给哪个minib stop命令. andry:接收到最后一个规程置位该标志表示可以开始工作了
extern volatile bool flg_opwork2ready;
extern volatile bool flg_opwork3ready; 
extern volatile bool flg_dischargeremainliquid[3];


extern volatile bool flg_fanworked[3];
extern volatile bool flg_shelfarmsleep[4];
static unsigned char config_cmd_frame[100];




//plate_num 0~29
bool sm_minisendonecmd(stTemper_cmdframeunion* te_cmd, unsigned char plate_num, unsigned char* recieve_tem)
{
	int res = 0;
	//	te_cmd.temcmd = CH1_Present_Value;
	tc_gettemperaddr(te_cmd,plate_num + 1);
						
	//			te_cmd.temvalue = 0;
	//printf("te_cmd->devaddr=%x, te_cmd->temcmd=%x, te_cmd->temvalue=%x",te_cmd->devaddr, te_cmd->temcmd,te_cmd->temvalue);
	pthread_mutex_lock(&mutex_modbuslock);
	if (tc_temsendcmd(te_cmd->devaddr, te_cmd->chopcmd, te_cmd->temvalue)) 
	{
		//	printf("tc_temsendcmd sucess\n");
		if (!tc_recivtempframe(recieve_tem, te_cmd))
		{
			printf("[tc_sendcmdchanl]recieve error_code = %x", (char)*recieve_tem);	
			res = false;
		}
	}
	else
	{
		printf("tc_temsendcmd failed\n");
		res = false;
	}
	pthread_mutex_unlock(&mutex_modbuslock);

	return res;
}



						/***读取队列中的命令***/
stminibd_sendpacket  sm_minibdrecvdata(stminibd_cmdlist** pcmd_head)
{
	stminibd_sendpacket cmd;
	stminibd_cmdlist* pcmd;
	
	memset(&cmd,0,sizeof(cmd));
	if ((*pcmd_head)->cmd.cmd == NOCMD)
		return cmd;

	cmd = (*pcmd_head)->cmd;
	
	if ((*pcmd_head)->next == NULL)
	{	
		(*pcmd_head)->cmd.cmd = NOCMD;
	}
	else
	{
		pcmd = *pcmd_head;
		*pcmd_head = (*pcmd_head)->next;
		pcmd->next = NULL;
		free(pcmd);
		pcmd = NULL;
	}

	return cmd;
}


/******************************************************************************
*
* Function Name  : sm_sendminibdframe 
* Description    : .向自控臂板发送命令
* 					 
* Input		   :emb_comid comPort 串口号,char * frame 数据指针, unsigned int send_num 数据长度
* Output		   :None
* Return		   :  None
*******************************************************************************/
void sm_sendminibdframe (emb_comid comPort, char * frame, uint8_t send_num)
{
	int res;
	unsigned short crc=0;
	unsigned char cmdbuffer[102] ={0};
	
	memcpy(cmdbuffer, frame, send_num);
	crc = usMBCRC16((uint8_t*)frame, send_num);
	//	printf("crc=%xsend_num=%d",crc,send_num);
	
	memcpy(&cmdbuffer[send_num],&crc,2);
		
	//	cmdbuffer[send_num] = (unsigned char)0x33;
	//	cmdbuffer[send_num + 1] = (unsigned char)0x44;
	tcflush(serail_hand[comPort], TCOFLUSH);
	#if (USE_PRINT_LOG == 1)
	{
		printf("the data send to minib send_num=%d comPort=%d\n",send_num,comPort);
		lprintf(log_my, INFO,"the data send to minib\n");
		for (i = 0;i < send_num + 2;i++)
		{
			printf(" %x",cmdbuffer[i]);
			if(send_num < 20)//配置信息不计入太长
				lprintf(log_my, INFO," %x",cmdbuffer[i]);
		}
	}
	#endif
	res = write(serail_hand[comPort],cmdbuffer,send_num + 2);

	if (res < 0)
		printf("write em_minicmd error\n");
	else
		printf("write em_minicmd successed\n");
}



/*************
reg_addr modbus寄存器地址 
send_num发送字节个数
data数据指针
************************/
void sm_sendpacket(emb_comid comPort, unsigned int reg_addr, unsigned int send_num, char * data)
{
	// struct timeval minib_now;
	// struct timespec minib_timeout;
	// int res_minib;
	unsigned char cmd_frame[100];
	
	cmd_frame[0] = 1;
	cmd_frame[1] = 0x10;		//功能码
	cmd_frame[2] = reg_addr / 256;		//寄存器地址HI
	cmd_frame[3] = reg_addr % 256;		//寄存器地址LO
	cmd_frame[4] = send_num / 2 / 256;		//num of data HI
	cmd_frame[5] = send_num / 2 % 256;		//num of data LO
	cmd_frame[6] = send_num;		//byte of counter

		
	memcpy(&cmd_frame[7], data, send_num);
	pthread_mutex_lock(&minib_send_lock);

	sm_sendminibdframe (comPort, (char*)cmd_frame, 7 + send_num);
	pthread_mutex_unlock(&minib_send_lock);
	
}
 
void set_minicmd(volatile stminibd_cmdlist* pcmd, stminibd_sendpacket cmd)//pcmd为表头
{
	if (pcmd->cmd.cmd == NOCMD)
	{
		pcmd->cmd = cmd;
	}
	else
	{
		while(pcmd->next != NULL)
			pcmd = pcmd->next;
		
		while((pcmd->next = (stminibd_cmdlist *)malloc(sizeof(stminibd_cmdlist))) == NULL)
		{
			sleep(1);
			printf("malloc error stminibd_cmdlist\n");
		}
		pcmd->next->cmd = cmd;
		pcmd->next->next = NULL;
	}	
}



void sm_thread_miniarm(void *arg)
{
	unsigned int i = 0, j = 0,z = 0, buffer_counter;
	stminibd_sendpacket cmd;
	
	#if(USE_LOG_INFO == 1)
	printf("=====================sm_thread_miniarm==================>thread_id = %d.\n", thread_id_miniboard = (int)pthread_self());
	printf("load_motor=");
	for (i =0 ;i < 5;i++)
		printf(" %d ", load_motor[i]);
	#endif

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd = MINI_CONFIG;
	config_cmd_frame[3] = cmd.minicmd_buffer[3] = (unsigned char)MINI_CONFIG;   //下面整型赋值要覆盖所以从3开始
	buffer_counter = 4;
	i = 0;
	j = 0;
	z = 0;
	
	while(buffer_counter < 64)
	{	
		if (buffer_counter < 24)
        {
			*((int *)&config_cmd_frame[buffer_counter]) = load_motor[i];
			i++;
        }
		else if (buffer_counter < 44)
        {
			*((int *)&config_cmd_frame[buffer_counter]) = load_motor2[j];
			j++;
        }
		else if (buffer_counter < 64)
        {
			*((int *)&config_cmd_frame[buffer_counter]) = load_motor3[z];
			z++;
        }
		buffer_counter += 4;
	}
	cmd.minicmd_num = buffer_counter;

	sm_sendpacket(COM2, 0, cmd.minicmd_num, (char*)&config_cmd_frame[3]);
	if (!sm_miniwaitinganswer(COM2, (uint8_t*)&mini_recieve_code_all[0], cmd))  //只当接受到ACK时退出
	{	
		mb_seterrorcode(CONNECT_ERROR_MINIA);	
	}	

	sleep(1);
		
	memset(&cmd, 0, sizeof(cmd));
	while(!flg_mainproexit)
	{
		pthread_mutex_lock(&mutex_mlock);
		cmd = sm_minibdrecvdata((stminibd_cmdlist**)&pcmd_head); 
		if (pthread_mutex_unlock(&mutex_mlock) != 0)
			lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
		//		if (ewkeventA == FREE_WORK)  //防止以解锁后还有水合操作  这一判断在mini板完成
		//			cmd.cmd = NOCMD;
		if (NOCMD != cmd.cmd)
		{				
            printf("in send em_minicmd\n");
            printf("\n");	 
            
            mini_recieve_codeACK = 0;
            if (cmd.cmd == INIT_MiniCmd || cmd.cmd == MINI_CONFIG)
            {
                mini_work_finished[0] = mini_work_finished[1] = mini_work_finished[2] = false;
            }
            else if (cmd.cmd != RELOAD)
                mini_work_finished[cmd.minicmd_buffer[4] - 1] = false;
            
            mini_finished = false;
            if(cmd.cmd != INIT_MiniCmd && cmd.cmd != MINI_CONFIG)
                mini_recieve_code_all[cmd.minicmd_buffer[4] - 1] = 0;
            sm_sendpacket(COM2, 0, cmd.minicmd_num, (char*)&cmd.minicmd_buffer[3]);
            if (!sm_miniwaitinganswer(COM2, (uint8_t*)&mini_recieve_code_all[cmd.minicmd_buffer[4] - 1], cmd))//只当接受到ACK时退出
            {    
                mb_seterrorcode(CONNECT_ERROR_MINIA);      
            }
            
            memset(&cmd, 0, sizeof(cmd));
		}
		else
		{
		#ifdef LAIYUE
			for(i = 0; i < 3; i++)
			{
				if(shelf_stirmode[i] > 0)
				{
					#if(0)   //  #if(USE_REAGENT_VAL_80 == 1)   增加 80ul 120ul 先不调整玻片架拉伸位置  
					uint32_t liquitvaltmp = 0;
					if( 0 == i){
						if(liquid_valA == 80){
							liquitvaltmp = liquid_valA + 20;
						}else if(liquid_valA == 120)
						{
							liquitvaltmp = liquid_valA + 30;
						}else{
							liquitvaltmp = liquid_valA;
						}
					}else if( 1 == i){
						if(liquid_valB == 80){
							liquitvaltmp = liquid_valB + 20;
						}else if(liquid_valB == 120)
						{
							liquitvaltmp = liquid_valB + 30;
						}else{
							liquitvaltmp = liquid_valB;
						}
					}else if( 2 == i){
						if(liquid_valC == 80){
							liquitvaltmp = liquid_valC + 20;
						}else if(liquid_valC == 120)
						{
							liquitvaltmp = liquid_valC + 30;
						}else{
							liquitvaltmp = liquid_valC;
						}
					}
					bs_minishelfwork(i, liquitvaltmp, shelf_stirmode[i]);
					shelf_stirmode[i] = 0;
					#else
					if(i == 0)
						bs_minishelfwork(i, liquid_valA, shelf_stirmode[i]);
					else if(i == 1)
						bs_minishelfwork(i, liquid_valB, shelf_stirmode[i]);
					else
						bs_minishelfwork(i, liquid_valC, shelf_stirmode[i]);	
					shelf_stirmode[i] = 0;
					#endif
				}
			}
		#endif
		}
		usleep(200000);
	}

	return;
}





bool sm_miniwaitinganswer(emb_comid comPort, unsigned char* mini_recieve_code, stminibd_sendpacket cmd)
{
	unsigned int counter = 0, error_counter = 0;

	#if(USE_LOG_INFO == 1)
	printf("[sm_miniwaitinganswer]\n");
	printf("\n");
	#endif

	while ((mini_recieve_codeACK != MINI_ACK) && (((*mini_recieve_code) & 0X0F) != 0X0F))
	{
		if ((((*mini_recieve_code) & 0X0F)  ==  0X0E) && (cmd.cmd != MINI_CONFIG) && (cmd.cmd != INIT_MiniCmd))
		{
			printf("*mini_recieve_code=%d cmd.cmd=%d\n ", *mini_recieve_code, cmd.cmd);
			if (mini_work_finished[cmd.minicmd_buffer[4] - 1])//收到当前玻片的回复信息
				break;
		}
		usleep(100000);
		counter++;
		error_counter++;
		if (counter >= 20)
		{
			#if( USE_LOG_INFO == 1) // #if(USE_LOG_INFO == 1)
			printf("COM%d recieve ack from minib timeout send cmd again with repeat code\n", comPort+1);
			lprintf(log_my, INFO,"COM%d recieve ack from minib timeout \n", comPort+1);
			#endif
			cmd.minicmd_buffer[3] = cmd.minicmd_buffer[3] | 0X80;
		
			if(cmd.cmd == MINI_CONFIG)
				sm_sendpacket(comPort, 0, cmd.minicmd_num, (char*)&config_cmd_frame[3]);
			else	
				sm_sendpacket(comPort, 0, cmd.minicmd_num, (char*)&cmd.minicmd_buffer[3]); 
			counter = 0;
		}	
		if (error_counter >= 100)
		{
			printf("communicating with minib error ack\n");
			
			mb_seterrorcode(CONNECT_ERROR_MINIA);	
			return false;
		}
	}
	#if( USE_LOG_INFO == 1) // #if(USE_LOG_INFO == 1)
	printf("mini_recieve_code=%x\n", *mini_recieve_code);
	lprintf(log_my, INFO,"mini_recieve_code=%x\n", *mini_recieve_code);
	#endif
	if (((*mini_recieve_code) & 0X0F)  ==  0X0E)	//当ACK没收到 而收到了response退出
	{
	    //	if (*mini_recieve_code == MINI_04)//	表示滴加结束 ,可能有瓶子拔出时的灌注操作
		{
			mini_finished = true;//操作结束通知都流程操作当前玻片架可执行
		}
		return 1;
	}

	printf("minib ack recieved \n");
    //	*mini_recieve_code = 0;
	return true; 
}



/******************************************************************************
*
* Function Name  : sm_parseminibdframe
* Description    : .解析自控臂板数据
* 					 
* Input		   :emb_comid comPort 串口号, int ReadLength 数据长度
* Output		   :volatile unsigned char* mini_recieve_code 填入命令码
* Return		   :  None
*******************************************************************************/
void sm_parseminibdframe(emb_comid comPort, int ReadLength)
{
	unsigned short crc=0; 
	char data[5] ={0};
	stminibd_sendpacket cmd;
	 unsigned char mini_recieve_code;

	//sleep(2);
	if (ReadLength < 4)
		return;
	#if(USE_LOG_INFO == 1)
	printf("In [sm_parseminibdframe]\n");
	#endif
	
	crc = usMBCRC16(serail_recvbuf, ReadLength - 2);
	//	printf("crc=%x,(unsigned char)crc=%x (unsigned char)crc >> 8=%x\n",crc,(unsigned char)crc,(unsigned char)(crc >> 8));
	if (((unsigned char)crc != serail_recvbuf[ReadLength-2]) || (((unsigned char)(crc >> 8)) != serail_recvbuf[ReadLength-1]))
	{
		printf("COM%d crc error\n", comPort);	
	}	
	else
	{
		//	pthread_mutex_lock(&minib_recievecode_lock);	//另一线程*mini_recieve_code清零标志操作几乎不占用时间 允许加锁
		if (serail_recvbuf[1] == MINI_ACK)
			 mini_recieve_codeACK = serail_recvbuf[1];
		else	//when response, send ack to minib 	serail_recvbuf[1] is shelf_num
		{		
			data[0] = MINI_ACK;
			data[1] = serail_recvbuf[2];	
			//		if (*mini_recieve_code != 0x9e)	//9e为按键检测返回，不需回信息否则 那端停止按键检测
			sm_sendpacket(comPort, 0, 2, data); // sm_sendpacket(comPort, 0, 2, &data); 
			#if(USE_LOG_INFO == 1)
			printf("[sm_parseminibdframe] ack recbuf[2]=[%d]\n", serail_recvbuf[2]);
			#endif
			if(serail_recvbuf[2] == 0XFF)
				serail_recvbuf[2] = serail_recvbuf[3] % 3;
			mini_recieve_code = mini_recieve_code_all[serail_recvbuf[2] - 1] = serail_recvbuf[3];
			mini_work_finished[serail_recvbuf[2] - 1] = true;
					
			if ((mini_recieve_code) <= SHELF_UNLOCKED_C && (mini_recieve_code) >= SHELF_UNLOCKED_A)	//当为解锁信息时
			{	
				if (serail_recvbuf[2] == 1)
				{
					if (wkeventA != FREE_WORK && wkeventA != MAINTAIN_WORK && wkeventA != INIT_WORK)
						flg_dischargeremainliquid[0] = true;
					
					wkeventA = FREE_WORK;	//水合结束回到空闲工作状态
					hydrateA.flage = false;
				}
				else if (serail_recvbuf[2] == 2)
				{
					if (wkeventB != FREE_WORK && wkeventB != MAINTAIN_WORK && wkeventB != INIT_WORK)
						flg_dischargeremainliquid[1] = true;
					wkeventB = FREE_WORK;	//水合结束回到空闲工作状态
					hydrateB.flage = false;	
				}
				else if (serail_recvbuf[2] == 3)
				{
					if (wkeventC != FREE_WORK && wkeventC != MAINTAIN_WORK && wkeventC != INIT_WORK)
						flg_dischargeremainliquid[2] = true;
					wkeventC = FREE_WORK;	//水合结束回到空闲工作状态
					hydrateC.flage = false;	
				}
			}
			#if(USE_LOG_INFO == 1)
			printf("###*mini_recieve_code=%x\n", mini_recieve_code);  
			#endif
			if (mini_recieve_code >= SHELF_LOCKED_A && mini_recieve_code <= SHELF_LOCKED_C)
			{
				mb_setstatetoshelf(serail_recvbuf[2] - 1, true);   //  flg_shelfcanindex[serail_recvbuf[2] -1] = TRUE, 该架需要扫描   andry
				SHELF_LOCK_STATE[serail_recvbuf[2] - 1] = 2;   //  对该架玻片架锁定  andry
				flg_shelfarmsleep[serail_recvbuf[2]] = false;   //  如果有休眠功能则退出休眠  andry
			}
			if (mini_recieve_code >= SHELF_UNLOCKED_A && mini_recieve_code <= SHELF_UNLOCKED_C)
			{
				SHELF_LOCK_STATE[serail_recvbuf[2] - 1] = 1;		
				flg_shelfarmsleep[serail_recvbuf[2]] = true;
				memset(&cmd,0,sizeof(cmd));
				cmd.cmd = RELOAD;
				cmd.minicmd_buffer[3] = (unsigned char)RELOAD;
				cmd.minicmd_buffer[4] = serail_recvbuf[2];
				cmd.minicmd_num=2;
				pthread_mutex_lock(&mutex_mlock);
				set_minicmd(pcmd_head,cmd);
				if (pthread_mutex_unlock(&mutex_mlock) != 0)
					lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error m_lock");
			}
			
			if (mini_recieve_code == 0x2e)
			{
				if((serail_recvbuf[2] == 1 && flg_opwork1ready)
						|| (serail_recvbuf[2] == 2 && flg_opwork2ready)
						|| (serail_recvbuf[2] == 3 && flg_opwork3ready))  //防止降温时强制停止
					flg_fanworked[serail_recvbuf[2] - 1] = true;
			}
				
			if (mini_recieve_code >= SHELF_LOSE_A && mini_recieve_code <= SHELF_LOSE_C)	
			{
				mb_seterrorcode(SHELF_LOSE_MINIA + mini_recieve_code - SHELF_LOSE_A);			
			}
			
			if (mini_recieve_code >= SHELF_UNLOAD_A && mini_recieve_code <= SHELF_UNLOAD_C)		
				mb_seterrorcode(SHELF_UNLOAD_MINIA + mini_recieve_code - SHELF_UNLOAD_A);			
		}
	}
}




int sm_serailsenddat(emb_comid comm, sr_cmdstruct_t *sendbuf, unsigned char sq, unsigned char flgresend, unsigned char commod)
{
	if(comm < 0 || sendbuf == NULL)
	{
		return -243;
	}
	/*
	if(IN_XIUMIAN || exit_flag) return -245; */
	if(sb_armpumpsend(comm, (unsigned char	*)sendbuf->cmdbuf, sendbuf->srdevaddr , sq, flgresend, commod) <= 0){
		lprintf(log_my, ERROR, "sf_commandwt comm[%d], cmd=[%s], devaddr=[%x] sendcmd fault.\n", comm, \
				(char*)(sendbuf->cmdbuf), (sendbuf->srdevaddr));
		return -244;
	}
	int rt = sb_waitingframeaswer(sendbuf);
	if(0 < rt)
	{
		lprintf(log_my, ERROR, "sf_commandwt comm[%d], cmd=[%s], devaddr=[%x] report return soult=[%d] fault.\n", \
				comm, sendbuf->cmdbuf, sendbuf->srdevaddr, rt);
	}	
	return rt;
}



