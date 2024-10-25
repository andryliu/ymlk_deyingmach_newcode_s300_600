
#include "temperctlmy.h"
#include <stdlib.h>
#include  <unistd.h>
#include <arpa/inet.h>
#include "mb.h"
#include "mbrtu.h"
#include "portserial.h"
#include "midbus.h"
#include "tpthread.h"
#include "log.h"
#include <termios.h>


extern pthread_mutex_t mutex_modbuslock ;

extern ewkevent wkevent;
extern log_t *log_g;


//chanel 0~29
int tp_sendtemperframe(char chanel, emTermper_cmdtype cmd, char r_w, short wdata, short *rdata)
{
	stTemper_cmdframe sttemper_frame;

	sttemper_frame.devaddr = 0X0A + chanel / 10;
	sttemper_frame.chanel = chanel % 10;
	sttemper_frame.temcmd = cmd;
	sttemper_frame.r_w = r_w;
	sttemper_frame.temvalue = wdata;
	static char isaddrd_ero = 0;

	if (isaddrd_ero && sttemper_frame.devaddr ==0X0D)
		return -1;
	pthread_mutex_lock(&mutex_modbuslock);

	if (MB_ENOERR == eMBRTUSend(sttemper_frame.devaddr, (unsigned char *)(&sttemper_frame.chanel), 5))
	{
		//	printf("send cmd success\n");
	}
	if (tp_revicetemperframe((uint8_t*)rdata, &sttemper_frame) < 0)
	{
		isaddrd_ero = 1;
		pthread_mutex_unlock(&mutex_modbuslock);
		return -1;
	}
	
	pthread_mutex_unlock(&mutex_modbuslock);
	return 0;
}

int tp_revicetemperframe(unsigned char *pdata,stTemper_cmdframe *temcmd)
{
	unsigned char * rtu = NULL;
	unsigned short  rtu_long = 0;  
	int i = 0, error_counter = 0;
	int  res = 0;
	int data_len = 0;

//	usleep(21);
	usleep(30000);//������Ӧʱ��
	
	while(1)
	{
	if(flg_intosleep && temcmd->devaddr != 0X0D)
		break;
		eMBPoll(  );
//		printf("i=%d error_counter=%d temcmd->devaddr=%d",i,error_counter,temcmd->devaddr);
		if (get_recieve_flag()) //�ȴ�������Ӧ
		{
	//		printf("read success\n");
			get_frame(&rtu, &rtu_long);
	/*
			printf("explain_ucRTUBuf=");
			for (i = rtu_long; i > 0; i-- )
			printf(" %x ",*(rtu + rtu_long - i));
			printf("\n");
			*/
									
			{	
				data_len = *(rtu + 2);
				
				*pdata = *(rtu + 4);	//��С�˸�ʽ�෴
				*++pdata = *(rtu + 5);

				res =  0;
			}

			break;
		}

		i++;
	//	if (i > 5)
		if (i > 50)
		{
			i = 0;
			error_counter++;
			if (error_counter >= 20)
			//if(0)
			{
				error_counter = 0;
				
					
					mb_seterrorcode(CONNECT_ERROR_TEMPERA + temcmd->devaddr -0X0A);
			
				return -1;
			}
			tcflush( serail_hand[TEM_PORT],TCIFLUSH);
			printf("tc_recivtempframe timeout send cmd again addr=%x,error_counter%d\n",temcmd->devaddr,error_counter);
			if (MB_ENOERR == eMBRTUSend(temcmd->devaddr, (unsigned char *)(&temcmd->chanel), 5))
		{
			printf("send cmd success\n");
			
		}
			usleep(500000);
			
		}
	usleep(100);//ûeMBPoll һ�ε�ʱ��
	}
	reset_recieve_flag();
	
	return 0;
					
}




