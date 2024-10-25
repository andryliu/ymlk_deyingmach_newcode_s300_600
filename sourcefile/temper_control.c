/*
 * temper_control.c
 *
 *  Created on: 2013-9-6
 *      Author: Administrator
 */


/*
 * temper_control.c
 *
 *  Created on: 2013-9-5
 *      Author: Administrator
 */
#include <termios.h>
#include <pthread.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include "mb.h"
#include "mbrtu.h"
#include "portserial.h"
#include "midbus.h"
#include "temper_control.h"
#include "log.h"
#include "string.h"
#include <unistd.h>

#define MAX_TEMPCMD_NUM 100



static  stTemper_framedata temper_cmd[MAX_TEMPCMD_NUM] = {
	/****************************control operating group**************************/
		{0X06,0X00,0X32},{0X06,0X00,0X33},{0X06,0X00,0X34},//CH1 control operating
		{0X06,0X04,0X1A},{0X06,0X04,0X1B},{0X06,0X04,0X1C},//CH2 control operating
		{0X06,0X08,0X02},{0X06,0X08,0X03},{0X06,0X08,0X04},//CH3 control operating
		{0X06,0X0B,0XEA},{0X06,0X0B,0XEB},{0X06,0X0B,0XEC},//CH4 control operating

		{0X06,0X00,0X64},{0X06,0X00,0X65},{0X06,0X00,0X67},{0X06,0X00,0X69},//12
		{0X06,0X00,0X71},{0X06,0X00,0X72},{0X06,0X00,0X73},{0X06,0X00,0X74},
		{0X06,0X00,0X75},									//CH1 control operating
		{0X06,0X04,0X4C},{0X06,0X04,0X4D},{0X06,0X04,0X4F},{0X06,0X04,0X51},
		{0X06,0X04,0X59},{0X06,0X04,0X5A},{0X06,0X04,0X5B},{0X06,0X04,0X5C},
		{0X06,0X04,0X5D},									//CH2 control operating
		{0X06,0X08,0X34},{0X06,0X08,0X35},{0X06,0X08,0X37},{0X06,0X08,0X39},
		{0X06,0X08,0X41},{0X06,0X08,0X42},{0X06,0X08,0X43},{0X06,0X08,0X44},
		{0X06,0X08,0X45},									//CH3 control operating
		{0X06,0X0C,0X1C},{0X06,0X0C,0X1D},{0X06,0X0C,0X1F},{0X06,0X0C,0X21},
		{0X06,0X0C,0X29},{0X06,0X0C,0X2A},{0X06,0X0C,0X2B},{0X06,0X0C,0X2C},
		{0X06,0X0C,0X2D},									//CH4 control operating

	/****************************read input register**************************///48
		{0X04,0X03,0XE8},{0X04,0X03,0XEE},{0X04,0X03,0XF4},{0X04,0X03,0XFA},
		//CH1_Value			CH2_Value		CH3_Value		CH4_Value
	/***************************Initial Setting*******************************///52
		{0X06,0X00,0X96},{0X06,0X00,0X98},{0X06,0X00,0X99},{0X06,0X00,0X9A},
		{0X06,0X00,0X9B},{0X06,0X00,0X9E},{0X06,0X00,0X9F},//CH1 Initial Setting
		{0X06,0X04,0X7E},{0X06,0X04,0X80},{0X06,0X04,0X81},{0X06,0X04,0X82},
		{0X06,0X04,0X83},{0X06,0X04,0X86},{0X06,0X04,0X87},//CH2 Initial Setting
		{0X06,0X08,0X66},{0X06,0X08,0X68},{0X06,0X08,0X69},{0X06,0X08,0X6A},
		{0X06,0X08,0X6B},{0X06,0X08,0X6E},{0X06,0X08,0X6F},//CH3 Initial Setting
		{0X06,0X0C,0X4E},{0X06,0X0C,0X50},{0X06,0X0C,0X51},{0X06,0X0C,0X52},
		{0X06,0X0C,0X53},{0X06,0X0C,0X56},{0X06,0X0C,0X57},//CH4 Initial Setting

};

//

/*

//设置温度 = 所需温度 + 差值，所以送给温控器温度为 350 + temp_goalval[35]
unsigned int temp_goalval[111]={	70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,75,75,75,75,75,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								20,20,20,20,20,20,20,20,20,20,//90°以上差值减小
								20,20,20,20,20,20,20,20,20,20,
								20
								};



*/



//设置温度 = 所需温度 + 差值，所以送给温控器温度为 350 + temp_goalval[35]
unsigned int temp_goalval[111]={	70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,75,75,75,75,75,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								60,60,60,60,60,60,60,60,60,60,//90°以上差值小2度
								60,60,60,60,60,60,60,60,60,60,
								60
								};

//每一片的修正值
//成反比，加热温度减去修正值
int temp_Dvalmy[30][105] = {{0}, {0}};
// int temp_Dvalmy[111]={0};
int temp_setval[30][105] = {{0}, {0}};



/*



//设置温度 = 所需温度 + 差值，所以送给温控器温度为 350 + temp_goalval[35]
unsigned int temp_goalval[111]={	70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,70,70,70,70,70,
								70,70,70,70,70,75,75,75,75,75,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								80,80,80,80,80,80,80,80,80,80,
								90,90,90,90,90,90,90,90,90,90,
								90
								};
								

*/




static	stTemper_frame cmdfull;
extern volatile temper_cmd_list* ptemcmd_head;
extern volatile tem_cmd_list* ptem_cmd_head;
extern pthread_mutex_t mutex_modbuslock;
extern pthread_mutex_t error_lock;
extern volatile bool datacmp_recieved;
extern volatile bool dataMixstation_recieved;

extern ewkevent wkevent;
volatile bool IN_CMP_WORK = false;
extern log_t *log_g;

void tc_packettempcmd(emTemper_cmd cmd)
{
	temper_cmd_list* pcmd = (temper_cmd_list*)ptemcmd_head;

	if (pcmd->temcmd == NOTEMPERCMD)
	{
		pcmd->temcmd = cmd;	
	}
	else
	{
		while(pcmd->next != NULL)
			pcmd = pcmd->next;
		
		while((pcmd->next = (temper_cmd_list *)malloc(sizeof(temper_cmd_list))) == NULL)
		{
			sleep(1);
			printf("malloc error temper_cmd_list\n");
		}
		pcmd->next->temcmd = cmd;	
		pcmd->next->next = NULL;
	}		
}

/***读取队列中的命令***/
emTemper_cmd  tc_readframtemp(void)
{
	emTemper_cmd cmd = NOTEMPERCMD;
	temper_cmd_list* pcmd;
//	printf("ptemcmd_head=%d\n",ptemcmd_head);
//	printf("\n");
	if (ptemcmd_head->temcmd == NOTEMPERCMD)
	{
		cmd = NOTEMPERCMD;
		return cmd;
	}
	
	cmd = ptemcmd_head->temcmd;
	
	if (ptemcmd_head->next == NULL)
	{	
		ptemcmd_head->temcmd = NOTEMPERCMD;
	}
	else
	{	
		pcmd = (temper_cmd_list*)ptemcmd_head;
		ptemcmd_head = ptemcmd_head->next;
		pcmd->next = NULL;
		free(pcmd);
		pcmd = NULL;
	}
	return cmd;
}


//用于调试命令
void tc_testpacketcmdtemp(stTemper_cmdframeunion cmd)
{
	tem_cmd_list* pcmd = (tem_cmd_list*)ptem_cmd_head;

	if (pcmd->cmdframeunion.chopcmd == NOTEMPERCMD)
	{
		memcpy((char*)&(pcmd->cmdframeunion), (char*)&cmd, sizeof(stTemper_cmdframeunion));
	}
	else
	{
		while(pcmd->next != NULL)
			pcmd = pcmd->next;
		
		while((pcmd->next = (tem_cmd_list *)malloc(sizeof(tem_cmd_list))) == NULL)
		{
			sleep(1);
			printf("malloc error tem_cmd_list\n");
		}
		memcpy((char*)&(pcmd->next->cmdframeunion), (char*)&cmd, sizeof(stTemper_cmdframeunion));
		pcmd->next->next = NULL;
	}	
}


/***读取队列中的命令***/
stTemper_cmdframeunion  tc_testreadtcmdtemp(void)
{
	stTemper_cmdframeunion cmd;
	tem_cmd_list* pcmd;
	//	printf("ptemcmd_head=%d\n",ptemcmd_head);
	//	printf("\n");
	if (ptem_cmd_head->cmdframeunion.chopcmd == NOTEMPERCMD)
	{
		cmd.chopcmd = NOTEMPERCMD;
		return cmd;
	}
	
	cmd = ptem_cmd_head->cmdframeunion;
	
	if (ptem_cmd_head->next == NULL)
	{	
		ptem_cmd_head->cmdframeunion.chopcmd = NOTEMPERCMD;
	}
	else
	{	
		pcmd = (tem_cmd_list*)ptem_cmd_head;
		ptem_cmd_head = ptem_cmd_head->next;
		pcmd->next = NULL;
		free(pcmd);
		pcmd = NULL;
	}

	return cmd;
}


bool tc_recivtempframe(unsigned char *pdata, stTemper_cmdframeunion *cmd)
{
	unsigned char * rtu;
	unsigned short  rtu_long;
	int i = 0, error_counter = 0;
	bool res;
	int data_len;

	usleep(30000);//待机响应时间
	
	while(1)
	{
		eMBPoll(  );
		
		if (get_recieve_flag()) //等待接收响应
		{
			get_frame(&rtu, &rtu_long);
				
			if (*(rtu + 1) == 0x83)					//功能码为错误码
			{
				*pdata = *(rtu + 2);
				res =  false;
			}
			else									//没有出错
			{	
				data_len = *(rtu + 2);
				
				*pdata = *(rtu + 4);	//大小端格式相反
				*++pdata = *(rtu + 3);

				res =  true;
			}
			break;
		}

		i++;
		//	if (i > 5)
		if (i > 300)
		{
			i = 0;
			error_counter++;
			if (error_counter >= 20)
			{
				error_counter = 0;
				if (!IN_CMP_WORK)
					mb_seterrorcode(CONNECT_ERROR_TEMPER);
				else		
					lprintf(log_my, FATAL, "CONNECT_ERROR_TEMPER in cmpwork!\n");
				return false;
			}
			tcflush( serail_hand[TEM_PORT],TCIFLUSH);
			ClearBuf();
			printf("tc_recivtempframe timeout dev=%d send cmd again error_counter%d\n",cmd->devaddr,error_counter);
			tc_temsendcmd(cmd->devaddr, cmd->chopcmd, cmd->temvalue);
			usleep(500000);
			
		}
		usleep(100);//没eMBPoll 一次的时间
	}
	reset_recieve_flag();
	
	return res;					
}


/********************函数功能:发送命令给温控模块*************************
******************	参数dev_addr:温控小控制模块地址(1~8)******************
******************	cmd: 控制命令value:命令值***************************/

bool tc_temsendcmd(unsigned char dev_addr, emTemp_chopcmd cmd, unsigned int value)
{
	unsigned int index;
	
	index = (unsigned int)cmd;
	cmdfull.cmd = temper_cmd[index];
	cmdfull.valuelo = (unsigned char) (value & 0x00ff);
	cmdfull.valuehi = (unsigned char) ((value>>8) & 0x00ff);
	

	if (MB_ENOERR == eMBRTUSend(dev_addr, (unsigned char *)(&cmdfull.cmd), 5))
	{
		return true;
	}
	return false;
}


void tc_gettemperaddr(stTemper_cmdframeunion * cmd, unsigned char plate_num)
{
	emTemp_chopcmd cmd_type = cmd->chopcmd;
	unsigned int interval = 0;

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

	if (plate_num < 5)
		cmd->devaddr = 1;
	else if (plate_num < 9)
		cmd->devaddr = 2;
	else if (plate_num < 13)
		cmd->devaddr = 3;
	else if (plate_num < 17)
		cmd->devaddr = 4;
	else if (plate_num < 21)
		cmd->devaddr = 5;
	else if (plate_num < 25)
		cmd->devaddr = 6;
	else if (plate_num < 29)
		cmd->devaddr = 7;
	else if (plate_num < 33)
		cmd->devaddr = 8;
	else
		perror("*******cmd.devaddr ********\n");

	switch (plate_num%4)
	{
		case 1:
			cmd->chopcmd = cmd_type ;
			break;
		case 2:
			cmd->chopcmd = cmd_type + interval;
			//			printf("cmd[i].temcmd=%d CH1_SV_Low=%d",cmd[i].temcmd,CH1_SV_Low);
			break;
		case 3:
			cmd->chopcmd = cmd_type + interval * 2;
			break;
		case 0:
			cmd->chopcmd = cmd_type + interval * 3;
			break;
		default:break;
	}
}


//当控制的cmd_value相同时可用,命令只需用CH1
bool tc_sendcmdchanl(const unsigned int cmd_value, emTemp_chopcmd cmd_type, int* temvalue)
{
	stTemper_cmdframeunion cmd[TEM_NUM + 1];
	bool res = true;
	unsigned int i;
	
	for(i = 1; i < TEM_NUM + 1; i++)
	{
		cmd[i].chopcmd = cmd_type;
		tc_gettemperaddr(&cmd[i], i);
	
		cmd[i].temvalue = cmd_value;
		//	printf("cmd[%d].devaddr=%d cmd[%d].temcmd=%d\n",i,cmd[i].devaddr,i,cmd[i].temcmd);
		pthread_mutex_lock(&mutex_modbuslock);
		if (tc_temsendcmd(cmd[i].devaddr, cmd[i].chopcmd, cmd_value)) 
		{
			//	printf("tc_temsendcmd sucess\n");
			if (!tc_recivtempframe((uint8_t*)(&temvalue[i - 1]), &cmd[i]))
			{
				printf("[tc_sendcmdchanl]recieve error_code = %x", (char)temvalue[i - 1]);	
				res = false;
			}
		}
		else
		{
			printf("tc_temsendcmd failed\n");
			res = false;
		}
		pthread_mutex_unlock(&mutex_modbuslock);
		usleep(100);
	}
	return res;
}


//当整架控制的cmd_value相同时可用,命令只需用CH1
bool tc_sendcmdshelf(const unsigned int cmd_value, emTemp_chopcmd cmd_type, int* temvalue, char shelf_num)
{
	stTemper_cmdframeunion cmd[TEM_NUM + 1];
	// unsigned int interval=0;
	bool res = true;
	unsigned int i;
	char endi = 0;
	if (shelf_num == 1)
	{
		i = 1;
		endi = 11;
	}
	else if (shelf_num == 2)
	{
		i = 11;
		endi = 21;
	}
	else if (shelf_num == 3)
	{
		i = 21;
		endi = 31;
	}
	
	for(; i < endi; i++)
	{
		cmd[i].chopcmd = cmd_type;
		tc_gettemperaddr(&cmd[i],i);
	
		cmd[i].temvalue = cmd_value;
	//	printf("cmd[%d].devaddr=%d cmd[%d].temcmd=%d\n",i,cmd[i].devaddr,i,cmd[i].temcmd);
		pthread_mutex_lock(&mutex_modbuslock);
		if (tc_temsendcmd(cmd[i].devaddr,cmd[i].chopcmd, cmd_value)) 
		{
		//	printf("tc_temsendcmd sucess\n");
			if (!tc_recivtempframe((uint8_t*)(&temvalue[i - 1]), &cmd[i]))
			{
				printf("[tc_sendcmdchanl]recieve error_code = %x", (char)temvalue[i - 1]);	
				res = false;
			}
		}
		else
		{
			printf("tc_temsendcmd failed\n");
			res = false;
		}
		pthread_mutex_unlock(&mutex_modbuslock);
		usleep(100);
	}
	return res;
}



int tc_getcmtempval(unsigned char plate_num, int* temvalue)
{
	int res = 0, timeout_counter = 0, error_counter = 0, cmp_again_time = 0;
	unsigned char cmdbuffer[10] = {0};
	stTemper_cmdframeunion cmd;
	int temvaltmp[50] = {0};
	int j, pi = 0, pj = 0, temval_e = 0;

	CmpAgain:
	{
		cmdbuffer[1] = plate_num - 1;
	
		if (MB_ENOERR == eMBRTUSend(33, &cmdbuffer[1], 1))
		{
			//	printf("send cmd success\n");	
		}
		
		while(!datacmp_recieved)
		{
			eMBPoll( );
			if (timeout_counter >= 50)
			{
				printf("recieve CmpTem timeout send cmd again\n");
				eMBRTUSend(33, &cmdbuffer[1], 1);
				timeout_counter = 0;
				error_counter++;
			}
			if (error_counter >= 20)
			{
				mb_seterrorcode(CONNECT_ERROR_TEMPRO);
		
				return -1;
			}
			timeout_counter++;
			
			usleep(10);
		}
		datacmp_recieved = false;

		sleep(3);
		for (j = 0; j<5; j++)
		{
			reset_recieve_flag();
			usleep(800000);
			//	usleep(800);
			cmd.devaddr = 8;
			cmd.chopcmd = CH3_Present_Value ;
		
			pthread_mutex_lock(&mutex_modbuslock);
			if (tc_temsendcmd(cmd.devaddr, cmd.chopcmd, 1)) 
			{
				//	printf("tc_temsendcmd sucess\n");
				if (!tc_recivtempframe((uint8_t*)&temvaltmp[j], &cmd))
				{
					printf("[tc_getfulltempval]recieve error_code = %x", temvaltmp[j]);	
					//res = -1;
					if (cmp_again_time++ > 5)
					{
						//	mb_seterrorcode(CONNECT_ERROR_TEMPER);
						//	return -1;
						*temvalue = 0;
						return 0;
					}
					goto CmpAgain;
				}
			}
			else
			{
				printf("tc_temsendcmd failed\n");
				res = -1;
			}
			pthread_mutex_unlock(&mutex_modbuslock);	
		}
	}
	/*
	printf("temvaltmp=");
	for (j = 0; j < 5;j++)
	printf(" %x ",temvaltmp[j]);
	*/
	pi = 4;
	while(pi > 0)
	{
		pj = pi - 1;
		while(pj >= 0)
		{
			//	printf(" temvaltmp[pi]=%x", temvaltmp[pi]);
			if (temvaltmp[pj] > temvaltmp[pi])
			{
				temval_e = temvaltmp[pj];
				temvaltmp[pj] = temvaltmp[pi];
				temvaltmp[pi] = temval_e;
			}
			pj--;
		}
		pi--;
	}
		
	printf("temvaltmp=");
	for (j = 0; j < 5; j++)
		printf(" %d ", temvaltmp[j]);

	if ( temvaltmp[2] > 1500)
		*temvalue = temvaltmp[0];
	else
		*temvalue = temvaltmp[2];	
	
	return res;
}


int tc_gettemtempvalprintf(char sel)
{
	int temvalue = 0;
	if (tc_getcmtempval(sel, &temvalue) < 0)
		return -1;
		
	printf("sel = %d CMP %d ", sel, temvalue);

	return temvalue;
}


int tc_getfulltempval( int* temvalue)
{
	int i = 1;
	// unsigned char cmdbuffer[10] ={0};
	IN_CMP_WORK = true;

	for(i = 1; i < TEM_NUM + 1; i++)
	{	
		if (tc_getcmtempval(i,&temvalue[i-1]) < 0)
			return -1;

		printf("CMP %d ",temvalue[i-1]);
		if (wkevent == MAINTAIN_WORK)
			return -2;
	}

	IN_CMP_WORK = false;
	return 0;
}


