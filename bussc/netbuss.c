

#include "netbuss.h"
#include "log.h"
#include "config.h"
#include <netinet/in.h>
#include <netdb.h>
#include <termios.h>
#include <sys/ioctl.h>
#include  <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <assert.h>
#include <fcntl.h>
#include <linux/if.h>
#include <linux/mii.h>
#include <linux/sockios.h>
#include <pthread.h>  
#include "serailminibd.h"
#include "midbus.h"
#include "buss.h"
#include "sys/socket.h"
#include "readinifile.h"
#include "string.h"
#include "../caculateProceTime/caculteprocetime.h"
#include "temper_control.h"
#include "serailbussfile.h"
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

int mulnum_glb[3]={0};

int mulnum_glb2[3]={0};

int remote_fd, local_fd;//网络远端和本机端句柄
volatile bool flg_netdisconnect = FALSE;	//当失去连接时通知等待接收信息的操作退出等待
unsigned char netbuf_read[NETRDBUF_LEN];
unsigned char netbuf_write[NETWRBUF_LEN];
volatile bool initialize_finished=FALSE;
volatile bool needinitialize=TRUE;
volatile bool flg_maintianrunning = FALSE;//正在调试标识
operate_head_list* operate_head1 = NULL;//A架操作头指针
operate_head_list* operate_head2 = NULL;//B架操作头指针
operate_head_list* operate_head3 = NULL;//C架操作头指针


extern sterrcode_list* perror_exist_head;
extern volatile ewkevent wkevent;
extern ewkevent wkeventA, wkeventB, wkeventC;
extern volatile bool flg_opwork1ready;
extern pthread_mutex_t mutex_mlock;
//extern volatile stminibd_cmdlist* pcmd_head, pcmd_head1, pcmd_head2;
extern volatile operate_head_list* operate_pri;
extern volatile bool special_reagent_recieved;
extern volatile bool flg_checkreagentindex[36];
extern volatile unsigned char replace_reagent;
extern pthread_mutex_t mutex_tryworklock;
extern volatile bool flg_sancmixstatioin;
extern char reagent_clr[3];
extern volatile  short cabin_value[12];

extern volatile unsigned int temp_value[30];

extern volatile bool flg_cabinreved; //大容量是否接收完毕标识
extern volatile  unsigned short bigcabin_value[18]; //	6个short类型容量大小（1~6个容器） +　６个ｓｈｏｒｔ类型传感器状态（１～６个状态）+6个容量增量
extern volatile  unsigned short bigcabin_value_init[18];
extern sterrcode_list* perror_head; //错误链表头指针
extern char report_reagentinfo[3];
extern volatile bool HEAD1_STEP_SENDED;
extern volatile bool HEAD2_STEP_SENDED;
extern volatile bool HEAD3_STEP_SENDED;
extern volatile bool get_temvale;


bool nt_createsocket( void)
{
    int  res;   
    struct hostent *host;  
    struct sockaddr_in serv_addr;  

	if ((host = gethostbyname("192.168.12.100")) == NULL)  
    {  
    	fprintf(stderr, "gethostbyname error!\n");  
       	return 0;
    }  
    if ((local_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)  
    {  
        fprintf(stderr, "socket error!\n");  
       	return 0;  
    }  
	memset(&serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;  
    serv_addr.sin_port = htons(10000);  
    serv_addr.sin_addr = *((struct in_addr *)host->h_addr);  
    // bzero(&(serv_addr.sin_zero), 8);
	// memset(serv_addr.sin_zero, 0, sizeof(serv_addr.sin_zero));  
	// fcntl(local_fd, F_SETFL, O_NONBLOCK);//设置非阻塞方式
	
	while(1)
	{
	    if ((res = connect(local_fd, (struct sockaddr *)&serv_addr, sizeof(struct sockaddr)) ) != 0)  
	    {  
			#if(USE_LOG_INFO == 1)
	        printf( "[*] machin[%s] ip[%s] port[%d] connect net error try again! errorcode=%d\n", *(host->h_aliases),  
					inet_ntoa(serv_addr.sin_addr), serv_addr.sin_port, res); 
			lprintf(log_my, ERROR, "\n machin[%s] ip[%s] port[%d] connect net error try again! errorcode=%d\n", host->h_aliases, 
					inet_ntoa(serv_addr.sin_addr), serv_addr.sin_port, res);
			#endif
	    }  
		else
		{
			#if(USE_LOG_INFO == 1)
			 printf( "Connect net sucess rescode=%d\n", res); 
			 lprintf(log_my, INFO, "Connect net sucess rescode=%d\n", res);
			 #endif

			break;
		}
		sleep(1);
	}
 
	flg_netdisconnect = false;
	memcpy(netbuf_write, SOFT_VER, 10);
	netbuf_write[10] = needinitialize;

	nt_sendpacketdata(INIT_START, (char*)netbuf_write, 11);
	needinitialize = false;

	if(initialize_finished)
		nt_sendpacketdata(INIT_END, (char*)netbuf_write, 0);
	#if(USE_LOG_INFO == 1)
	printf("***********nt_createsocket finished*************\n");
	#endif
	return true;
}	



//read reagent's information from pc
void nt_chargespecialreagent(char *netbuf_read)
{
	int i;
	#if (1 == USE_PRINT_LOG)	
	printf("nt_chargespecialreagent\n\n");
	lprintf(log_my, INFO, "nt_chargespecialreagent\n\n");
	#endif
	for (i = 0; i < 9;i++)
	{
		memset(reagent_code[(netbuf_read[0] - 1) * 9 + i].reagent_kind, 0, 9);
		memcpy(reagent_code[(netbuf_read[0] - 1) * 9 + i].reagent_kind, &netbuf_read[1 + i * 23], 9);  
		reagent_code[(netbuf_read[0] - 1) * 9 + i].special_num = netbuf_read[1 + i * 23 + 9];
		memset(reagent_code[(netbuf_read[0] - 1) * 9 + i].lot_num, 0, 9);
		memcpy(reagent_code[(netbuf_read[0] - 1) * 9 + i].lot_num, &netbuf_read[1 + i * 23 + 10], 9);   
		memset(&reagent_code[(netbuf_read[0] - 1) * 9 + i].val, 0, 4);
		memcpy(&reagent_code[(netbuf_read[0] - 1) * 9 + i].val, &netbuf_read[1 + i * 23 + 19], 4);
		
		printf(" %s-%d-%s val=%d\n", reagent_code[(netbuf_read[0] - 1) * 9 + i].reagent_kind,	
				reagent_code[(netbuf_read[0] - 1) * 9 + i].special_num,
				reagent_code[(netbuf_read[0] - 1) * 9 + i].lot_num,
				reagent_code[(netbuf_read[0] - 1) * 9 + i].val);		
		lprintf(log_my, INFO," %s-%d-%s val=%d\n", reagent_code[(netbuf_read[0] - 1) * 9 + i].reagent_kind,
				reagent_code[(netbuf_read[0] - 1) * 9 + i].special_num,
				reagent_code[(netbuf_read[0] - 1) * 9 + i].lot_num,
				reagent_code[(netbuf_read[0] - 1) * 9 + i].val);

		if (reagent_code[(netbuf_read[0] - 1) * 9 + i].special_num >= FR1 &&
				reagent_code[(netbuf_read[0] - 1) * 9 + i].special_num <= FR2)
			reagent_code[(netbuf_read[0] - 1) * 9 + i].special_num -= 2;
	}
}



/*************网线是否拔出***************/
int nt_isnetmiireg(const char *if_name, int phy_id, int reg_num )
{
	struct ifreq ifr;   // 获取网卡信息  include <net/if.h>
	struct mii_ioctl_data *mii;
	int value;

	bzero(&ifr, sizeof(ifr));
	strncpy(ifr.ifr_name, if_name, IFNAMSIZ-1);
	ifr.ifr_name[IFNAMSIZ-1] = 0;

	if (ioctl(local_fd, SIOCGMIIPHY, &ifr) < 0)
	{
		perror("ioctl");
		return -1;
	}

	mii = (struct mii_ioctl_data *)&ifr.ifr_data;
	mii->reg_num = reg_num;//0x01
	if (ioctl(local_fd, SIOCGMIIREG, &ifr) < 0)
	{
		perror("ioctl");
		return -1;
	}

	value = ((mii->val_out&0x04)>>2);
	return value;
}




/******************************************************************************     shelf_num : 0(A), 1(B), 2(C)
* Function Name  : Nt_recreagentpara
* Description    : .把网络数据转化成操作数据填入数据结构中
* 					 
* Input		   :operate_head_list*	operate_head 链表头, char *netbuf_read,int netread_len
* Output		   :None
* Return		   :  int 成功0 错误-1
*******************************************************************************/
int nt_recreagentpara(operate_head_list* operate_head, const char *netbuf_read, int netread_len, uint8_t shelf_num)	//一个tcp包为一片玻片的操作玻片最后操作需为停止操作
{
	int  res = 0;
	uint16_t i =  0;
	operate_t * operate_p = NULL;
	operate_head_list* operate_head_p = NULL;
	operate_head_list* last_head = NULL;     //  最后的规程指针
	unsigned char last_head_flage = 0;	//operate头链表尾是否就是其链表头
	char have_mix_reagent1st = 0, have_mix_reagent2scd = 0;
	char TCP_Buffer[24] = {0};

	#if (1 == USE_PRINT_LOG)	
	printf("in Nt_recreagentpara\n\n");
	lprintf(log_my, INFO, "in Nt_recreagentpara\n\n");
	#endif

	operate_head_p = operate_head; 
	
	while(operate_head_p->next_head!= NULL)			//operate头链表尾
	{
		last_head = operate_head_p;
		operate_head_p = operate_head_p->next_head;
	}
	
	/*   当所有流程读发送完成后再发送一帧 WORKA，WORKB，WORKC + 1 字节滴加试剂量(100或150ul) + 1字节DAB比例 + 1字节第二染色比例  andry  */
	if (netread_len < 10)	//一个结束包小于10说明pc发来操作data结束标志 这个值为滴加一次
	{  
		#if (1 == USE_PRINT_LOG)	
		printf("[ReadOperate]netdata recieve finished\n");
		#endif

		if (netbuf_read[1] != -1)
			mulnum_glb[shelf_num] = netbuf_read[1];   // DAB 比例  andry
		mulnum_glb2[shelf_num] = netbuf_read[2];
		
		if (operate_head == operate_head1)
			liquid_valA = netbuf_read[0];     //  滴加的试剂量  andry
		else if (operate_head == operate_head2)
			liquid_valB = netbuf_read[0];
		
		else if (operate_head == operate_head3)
			liquid_valC = netbuf_read[0];
		
		ct_addtabtime(operate_head, *(int*)&netbuf_read[3]);	//加延时启动时间
		
		mb_printfoperatelist(operate_head);
		operate_p = &(operate_head->operate);
		while (operate_p != NULL)		//找出滴加混合试剂的时间
		{
			if (operate_p->reagent == REAGENT_DAB)
			{
				if (operate_head == operate_head1)
					dispense_mixreagent_timeA = operate_p->time;
				if (operate_head == operate_head2)
					dispense_mixreagent_timeB = operate_p->time;
				if (operate_head == operate_head3)
					dispense_mixreagent_timeC = operate_p->time;

				have_mix_reagent1st = 1;
			//	break;
			}
			
			if (operate_p->reagent == REAGENT_SECEND)
			{
				if (operate_head == operate_head1)
					dispense_mixreagent_timeA_next = operate_p->time;
				if (operate_head == operate_head2)
					dispense_mixreagent_timeB_next = operate_p->time;
				if (operate_head == operate_head3)
					dispense_mixreagent_timeC_next = operate_p->time;
				have_mix_reagent2scd = 1;
			//	break;
			}	
			operate_p = operate_p->next;
		}
		if(have_mix_reagent1st == 0)//没有混合试剂
		{
			if (operate_head == operate_head1 )
					dispense_mixreagent_timeA = 0;
			if (operate_head == operate_head2 )
					dispense_mixreagent_timeB = 0;
			if (operate_head == operate_head3 )
					dispense_mixreagent_timeC = 0;
		}
		if(have_mix_reagent2scd == 0)//没有混合试剂
		{
			if (operate_head == operate_head1 )
					dispense_mixreagent_timeA_next = 0;
			if (operate_head == operate_head2 )
					dispense_mixreagent_timeB_next = 0;
			if (operate_head == operate_head3 )
					dispense_mixreagent_timeC_next = 0;
		}	
		return 0;
	}   //  end of if (netread_len < 10)	
	else
	{
		printf("recieve operate data\n");
		if (operate_head->operate.reagent == STOP_OPERATE)	//operate头链表头空
		{	
			printf("in head\n");
			last_head_flage = 1;
			operate_p = &(operate_head->operate);	
		}
		else
		{
			operate_head_p = operate_head;
			#if (1 == USE_PRINT_LOG)	
			printf("operate_head_p->next_head = %doperate_head = %d\n", operate_head_p->next_head, operate_head->next_head);
			printf("not in head.\n");
			#endif

			while (operate_head_p->next_head != NULL)	//寻址到operate头链表尾
				operate_head_p = operate_head_p->next_head;

			#if (1 == USE_PRINT_LOG)	
			printf("head end find\n");
			#endif

			while((operate_head_p->next_head = (operate_head_list*)malloc(sizeof(operate_head_list))) == NULL)
			{
				sleep(1);
				#if(1 == USE_LOG_INFO)
				lprintf(log_my, ERROR, "malloc error operate_head_list\n");
				#endif
			}
			operate_head_p = operate_head_p->next_head;
			operate_head_p->next_head = NULL;
				
			operate_p = &(operate_head_p->operate); //====  0x3c(60)
		}
		i = 0;
		while(!flg_mainproexit)
		{
			if(netbuf_read[i] < REAGENT_CASE)  // 从第一个玻片规程开始，命令后的数据段netbuf[0]为试剂类型代码。 37
			{
				if (reagent_code[(uint8_t)(netbuf_read[i])].special_num == H2O2)// 稀释液   扫描到的第上位机发送来的NetReadBuffer[i]位置的试剂瓶上的种类标识是稀释液 andry
				{
					mix_DAB[shelf_num].ordArrayB[netbuf_read[i + 6] % 10] = netbuf_read[i];	 // 一个规程包含：试剂 1字节， 时间 4字节， 温度 1字节， 玻片位置 1字节，所以 i+6（4+1+1）
					memcpy(&mix_DAB[shelf_num].reagentB[(uint8_t)netbuf_read[i + 6] % 10],   //  拷贝上位机发送的NetReadBuffer[i+6]%10位置的试剂到mix_DAB[对应玻片架].reagentB数组中 andry
							(const uint8_t*)&reagent_code[ (uint8_t)netbuf_read[i]], sizeof(reagent_t));
					i += 7;   //  通讯协议格式里规定流程格式：试剂（1字节）+ 时间（4字节）+ 温度（1字节）+ 玻片位置（1字节），怎么跟上面memcpy的sizeof(reagent_t)不想等？？？andry
				}
				
				if(reagent_code[(uint8_t)netbuf_read[i]].special_num == RED2 ||    //  双染时需要用到的混合试剂 ????
					reagent_code[(uint8_t)netbuf_read[i]].special_num == FR2 ||
					reagent_code[(uint8_t)netbuf_read[i]].special_num == GREEN2 ||
					reagent_code[(uint8_t)netbuf_read[i]].special_num == AP2)
				{
					mix_SECEND[shelf_num].ordArrayB[netbuf_read[i + 6] % 10] = netbuf_read[i];	
					memcpy(&mix_SECEND[shelf_num].reagentB[(uint8_t)netbuf_read[i + 6] % 10], &reagent_code[(uint8_t) netbuf_read[i]], sizeof(reagent_t));
					i += 7;
				}
			}

			memset(&(operate_p->reagent_info), 0, sizeof(reagent_t));				
			operate_p->reagent = netbuf_read[i];
				
			if (netbuf_read[i] < REAGENT_CASE)	
			{
				if (reagent_code[(uint8_t)netbuf_read[i]].special_num == DAB)	
				{
					mix_DAB[shelf_num].ordArrayA[(uint8_t)netbuf_read[i + 6] % 10] = netbuf_read[i];
					memcpy(&mix_DAB[shelf_num].reagentA[(uint8_t)netbuf_read[i + 6] % 10], &reagent_code[(uint8_t)netbuf_read[i]], sizeof(reagent_t));
					operate_p->reagent = REAGENT_DAB;
					mix_DAB[shelf_num].tep_num++;
				}
				if (reagent_code[(uint8_t)netbuf_read[i]].special_num == RED1 ||
						reagent_code[(uint8_t)netbuf_read[i]].special_num == FR1 ||
						reagent_code[(uint8_t)netbuf_read[i]].special_num == GREEN1 ||
						reagent_code[(uint8_t)netbuf_read[i]].special_num == AP1)
				{
					mix_SECEND[shelf_num].ordArrayA[(uint8_t)netbuf_read[i + 6] % 10] = netbuf_read[i];
					memcpy(&mix_SECEND[shelf_num].reagentA[(uint8_t)netbuf_read[i + 6] % 10], &reagent_code[ (uint8_t)netbuf_read[i]], sizeof(reagent_t));
					operate_p->reagent = REAGENT_SECEND;
					mix_SECEND[shelf_num].tep_num++;
				}
				
				if (strlen(reagent_code[(uint8_t) netbuf_read[i]].reagent_kind) != 0)//接收时试剂被拿掉
					memcpy((uint8_t*)&(operate_p->reagent_info), (const uint8_t*)&reagent_code[(uint8_t) netbuf_read[i]], sizeof(reagent_t));
				else
				{
					printf("reagent shelf removed reagent num=%d\n", netbuf_read[i]);

					lprintf(log_my, INFO,"reagent shelf removed reagent num=%d\n", netbuf_read[i]);

					TCP_Buffer[0] = 0;
					memcpy(&TCP_Buffer[1],operate_p->reagent_info.code, sizeof(operate_p->reagent_info.code));
					nt_sendpacketdata(REAGENT_REMOVED, TCP_Buffer, 21);
					res = -1;
				}
			}

			memcpy(&operate_p->time, &netbuf_read[i + 1], 4);  // 每一个流程里的时间
			//	operate_p->time = ntohl(operate_p->time);
			operate_p->time = operate_p->time;      // 废话？？？andry
			operate_p->temp = netbuf_read[i + 5];     //  每个玻片流程中的温度
			operate_p->plate_num = netbuf_read[i + 6];  // 波片所在3架中的位置索引号
			
			if ((i += 7) < netread_len && (netread_len - i) >= 6)   // 一个TCP包中有多个流程  andry
			{
				while((operate_p->next = (operate_t*)malloc(sizeof(operate_t))) == NULL)
				{
					sleep(1);
					printf("malloc error operate_t\n");
				}
				operate_p = operate_p->next;
			}
			else       //  
			{
				operate_p->next = NULL;			
				break;    //  跳出  while(!flg_mainproexit),  flg_mainproexit 要是被置位的话在main函数里就跳出了while而跑到 exit(NULL)退出进程，只能重启了
			}
		}	 //  end of while(!flg_mainproexit)
	}  // end of  if (netread_len >= 10)	

	usleep(1000);

	return res;	
}




int nt_recvchangereagentinfo( char *netbuf_read, int netread_len)
{
	operate_head_list* operate_head_p = NULL;
	operate_t * operate_p = NULL;
	uint32_t reagent_index = 2;
	uint8_t shelf_num = (uint8_t)netbuf_read[0];
	unsigned char plate_num = netbuf_read[1];     //玻片序号0~9

	#if(USE_PRINT_LOG == 1)
	printf("in nt_recvchangereagentinfo netread_len=%d\n", netread_len);
	lprintf(log_my, INFO,"in nt_recvchangereagentinfo netread_len=%d\n", netread_len);
	{
		for (i = 0; i < netread_len; i++)
		{
			printf(" %d ", netbuf_read[i]); 
			lprintf(log_my, INFO," %d ", netbuf_read[i]);
		}
		printf("\n");
	}
	#endif

	if (netbuf_read[0] == 0)
		operate_head_p = operate_head1;
	else if (netbuf_read[0] == 1)
		operate_head_p = operate_head2;
	else if (netbuf_read[0] == 2)
		operate_head_p = operate_head3;
	else
		perror("nt_recvchangereagentinfo\n");
	mb_printfoperatelist(operate_head_p);

	while(operate_head_p!= NULL)//找玻片位子
	{
		if ((operate_head_p->operate.plate_num) % 10 == netbuf_read[1])
		{
			break;
		}
		operate_head_p = operate_head_p->next_head;
	}

	if (operate_head_p == NULL)
		perror("nt_recvchangereagentinfo position not found\n");

	operate_p = &(operate_head_p->operate);

	reagent_index = 2;
	while(operate_p != NULL)
	{
		if (operate_p->reagent == REAGENT_DAB)
		{
			printf("dabfind reagent_index=%d\n", reagent_index);
			memcpy(&mix_DAB[shelf_num].reagentA[plate_num],
			&reagent_code[(uint8_t) netbuf_read[reagent_index]] ,sizeof(reagent_t));	
			mix_DAB[shelf_num].ordArrayA[plate_num] = netbuf_read[reagent_index];		
			memcpy(&(operate_p->reagent_info), &reagent_code[ (uint8_t)netbuf_read[reagent_index]],sizeof(reagent_t));
			reagent_index++;	
		}
		else if (operate_p->reagent == REAGENT_SECEND)
		{	
			printf("secendfind reagent_index=%d\n", reagent_index);
			memcpy(&mix_SECEND[shelf_num].reagentA[plate_num],
						&reagent_code[(uint8_t) netbuf_read[reagent_index]],sizeof(reagent_t));  
			mix_SECEND[shelf_num].ordArrayA[plate_num] = netbuf_read[reagent_index];	   
			memcpy(&(operate_p->reagent_info), &reagent_code[ (uint8_t)netbuf_read[reagent_index]],sizeof(reagent_t));
			reagent_index++;
		}	
		else if ((operate_p->reagent & 0X7F) < REAGENT_CASE  )
		{
			#if(USE_LOG_INFO == 1)
			printf("operate_p->reagent=%d\n", operate_p->reagent);		
			printf("reagentfind reagent_index=%d\n", reagent_index);
			#endif

			memcpy(&(operate_p->reagent_info), &reagent_code[ (uint8_t)netbuf_read[reagent_index]],sizeof(reagent_t));
			operate_p->reagent = netbuf_read[reagent_index];
			reagent_index++;
		}

		if(reagent_index > netread_len)
			perror("reagent_index over\n");
			  
		operate_p = operate_p->next;
	}

	/*********************稀释液******************************/
	reagent_index = 2;

	while(reagent_index < netread_len)
	{
		if (reagent_code[(uint8_t)netbuf_read[reagent_index]].special_num == H2O2	)// 稀释液
		{
			mix_DAB[shelf_num].ordArrayB[plate_num] = netbuf_read[reagent_index]; 
			memcpy(&mix_DAB[shelf_num].reagentB[plate_num],
			&reagent_code[(uint8_t) netbuf_read[reagent_index]],sizeof(reagent_t));
		}
		else if (reagent_code[(uint8_t)netbuf_read[reagent_index]].special_num == RED2	||
			reagent_code[(uint8_t)netbuf_read[reagent_index]].special_num == FR2 ||
			reagent_code[(uint8_t)netbuf_read[reagent_index]].special_num == GREEN2 ||
			reagent_code[(uint8_t)netbuf_read[reagent_index]].special_num == AP2)// 稀释液
		{
			mix_SECEND[shelf_num].ordArrayB[plate_num] = netbuf_read[reagent_index]; 
			memcpy(&mix_SECEND[shelf_num].reagentB[plate_num],
				(const char *)&reagent_code[ (uint8_t)netbuf_read[reagent_index]], sizeof(reagent_t));
		}
			reagent_index++;
	}
	if(reagent_index >= netread_len)
		lprintf(log_my, INFO,"H2O2 over");

	mb_printfoperatelist(operate_head_p);

	return 0;
		
}


int nt_packetparse(void)
{
	int netread_len = 0, connect_counter = 0, rtu_len = 0;
	Eenetevent eenetevent = NO_EVENT;
	fd_set DeviceRead;
	fd_set DeviceWrite;
	int DeviceMax = 0, res = 0, read_buffer_index = 0, check_cmd = 0;
	char tcpdata = 0;
    unsigned char rtu[10000] = {0}, buffer_tmp[10000] = {0};
	char* ip = "192.168.12.201";
	char iptmp[50] = {0};
	struct timeval timeout,beattime,now;
	stminibd_sendpacket mini_cmd;
	bool ISmini_cmdSETEDA = false, ISmini_cmdSETEDB = false, ISmini_cmdSETEDC = false;

	tcflush(local_fd, TCIFLUSH);
	tcflush(local_fd, TCOFLUSH);

	#if(USE_LOG_INFO == 1)					
	printf("*********************in listen net***************************\n\n");
	#endif
	gettimeofday(&beattime, NULL);
	while(!flg_mainproexit)
	{
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;
		FD_ZERO(&DeviceRead);		
		DeviceMax = local_fd + 1;
		FD_SET(local_fd, &DeviceRead);
	
		res = select(DeviceMax,&DeviceRead, &DeviceWrite, NULL, &timeout);
		
		if(res < 0) {
			perror("[ListenNet] select error");
			return -1;
		}

		if (FD_ISSET(local_fd, &DeviceRead))
		{
			if ((netread_len = read(local_fd, (char *)netbuf_read + read_buffer_index, 2000)) < 0)
			{
				printf("[ListenNet]read netdata error %s\n", strerror(errno));			
			}
			else if (netread_len == 0)//服务器主动断开
			{
				gettimeofday(&now, NULL);
				//	printf("now.tv_sec=%d  beattime.tv_sec=%d",now.tv_sec ,beattime.tv_sec);
				if (now.tv_sec - beattime.tv_sec >= 5)
				{
					#if(USE_LOG_INFO == 1)
					printf("[ListenNet] lose connect with net begining to try again server disconnect %s\n", strerror(errno));
				    #endif
				   	connect_counter = 0;
					flg_netdisconnect = true;
					sleep(1);
					close(local_fd);
						
					nt_createsocket();
					gettimeofday(&beattime, NULL);
				}	
			}
			else
			{		
				read_buffer_index += netread_len;
				gettimeofday(&beattime, NULL);
			}
		}
		else
		{
			gettimeofday(&now, NULL);
				//		printf("now.tv_sec - beattime.tv_sec = %d\n",now.tv_sec - beattime.tv_sec);
			if (now.tv_sec - beattime.tv_sec >= 20)
			{
				if (nt_isnetmiireg("eth0", 0X10, 0X01) <= 0) //超时后检测网线
				{
					connect_counter = 0;
					flg_netdisconnect = true;
					sleep(1);
					close(local_fd);
						
					#if(USE_LOG_INFO == 1)
					printf("[ListenNet] lose connect with net begining to try again server disconnect time > 20m, fault:%s\n", strerror(errno));
				    #endif
					nt_createsocket();
					gettimeofday(&beattime, NULL);
				}	
			}	
		}

		if (netbuf_read[10] == UpdateSlave)//更新单独
		{
			if (netbuf_read[11] == 0)
				mb_updatefirmware((char*)&netbuf_read[6], netread_len - 12);
			else if (netbuf_read[11] == 1)
				mb_updatemeasconfig((char*)&netbuf_read[6], netread_len - 12);//需要再发送命令给机械臂和温控器
			else if (netbuf_read[11] == 2)
				mb_getmeasconfig();
			else if (netbuf_read[11] == 3)
				mb_setmeasconfig();
			
			read_buffer_index = 0;
			memset(netbuf_read,0,sizeof(netbuf_read));
		}
		
		else if (netbuf_read[10] == UpdateIP)
		{
			strcpy(&iptmp[1], ip);
			iptmp[0] = 0X22;
			sprintf(&iptmp[14], "%d", netbuf_read[11]);
			iptmp[15] = 0X22; 
			rf_writeintforprofile("LOCAL_MACHINE", "IPAddress", iptmp, (const char *)UserinfoPath, 0);				
			memset(netbuf_read, 0, sizeof(netbuf_read));
		}
		
		else
		{   /*  上位机发送到的   */
			memcpy(&rtu_len, &netbuf_read[6], 4);
		
			if (rtu_len + 10  <= read_buffer_index)
			{	
				memcpy(rtu , &netbuf_read[10] , rtu_len);
				rtu[rtu_len] = 0;//方便直接读取字符串
				
				memcpy(buffer_tmp, (char *)netbuf_read + rtu_len + 10, read_buffer_index - rtu_len - 10);	
				memcpy(netbuf_read, buffer_tmp, read_buffer_index - rtu_len - 10);

				read_buffer_index =  read_buffer_index - rtu_len - 10;

				if (netbuf_read[11] != HEART_BEAT)
				{
	
				}
				if (rtu_len == 0)	//更新最近一次BEAT时间
					gettimeofday(&beattime, NULL);
				
				eenetevent = rtu[0];

				switch(eenetevent)
				{
					case NO_EVENT:
						break;
					case INIT:	
						//	wkevent= INIT_WORK; 
						break;
					case MAINTIAN:
						flg_maintianrunning = true;
						wkevent = MAINTAIN_WORK;
						wkeventA = MAINTAIN_WORK;
						wkeventB = MAINTAIN_WORK;
						wkeventC = MAINTAIN_WORK;
						printf("MAINTIAN WORK CMD = %d scend = %d\n", rtu[1], rtu[2]);
						
						if (rtu[1] != 0XFF)
							mb_menualwork((const char*)&rtu[1]);
						//	bs_writeconfiginfo();//退出后接受配置文件
						//	mb_readconfparatinfo();
						break;
					case WORKA:
						//读取流程
						if (flg_opwork1ready)
							break;
						mb_lockallregent(0);
						if (ISmini_cmdSETEDA == false)
						{
							ISmini_cmdSETEDA = true;

							bs_packetshelfstreach(0, &mini_cmd);   
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head, mini_cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mLOCK");
						}
						pthread_mutex_lock(&head_lock);
						if (nt_recreagentpara(operate_head1, (const char*)&rtu[1], rtu_len - 1, 0) < 0)
						{
							printf("in mb_canceloperate WORKA\n" );
							tcpdata = -1;//接收失败
							nt_sendpacketdata(WORK_START, &tcpdata, 1);
							mb_setoperateheaddisable((operate_head_list**)&operate_head1, true);
						}
						else
							tcpdata = 1;//开始运行
							
						if (pthread_mutex_unlock(&head_lock) != 0)
							lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_lock");
						if (rtu_len < 10)
						{
							if (!flg_opwork1ready && !flg_opwork2ready && !flg_opwork3ready)
							{
								operate_pri = operate_head1;
							}	
							
							work_cnt++;
							ISmini_cmdSETEDA = false;
							//	tcpdata = 1;
							//	nt_sendpacketdata(WORK_START,&tcpdata,1);
							mb_sethydratepale(1);
							wkeventA = BUSY_WORK;
							ct_reportcaculateprocetime();
							flg_opwork1ready = true;
							
							reagent_lock[0][0] =reagent_lock[1][0]=reagent_lock[2][0]=reagent_lock[3][0]= 0;
							printf("Nt_recreagentpara finished A\n");
						}
						break;
					case WORKB:
						//读取流程
						printf("flg_opwork2ready=%d\n", flg_opwork2ready);
						
						if (flg_opwork2ready)
							break;
						mb_lockallregent(1);
						if (ISmini_cmdSETEDB == false)
						{
							ISmini_cmdSETEDB = true;

							bs_packetshelfstreach(10, &mini_cmd);    // 发送拉伸玻片架命令
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head, mini_cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mLOCK");
						}
							
						pthread_mutex_lock(&head_lock);
						if (nt_recreagentpara(operate_head2, (const char*)&rtu[1], rtu_len - 1, 1) < 0)
						{
							printf("in mb_canceloperate WORKB\n" );
							tcpdata = -2;//接收失败
							nt_sendpacketdata(WORK_START, &tcpdata, 1);
							mb_setoperateheaddisable((operate_head_list**)&operate_head2, true);
						}
						else
							tcpdata = 2;//开始运行
							
						if (pthread_mutex_unlock(&head_lock) != 0)
							lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");
						if (rtu_len < 10)
						{
							if (!flg_opwork1ready && !flg_opwork2ready && !flg_opwork3ready)
							{
								operate_pri = operate_head2;
							}
							work_cnt++;
							ISmini_cmdSETEDB = false;
							//	tcpdata = 2;
							//	nt_sendpacketdata(WORK_START,&tcpdata,1);
							
							mb_sethydratepale(2);
							wkeventB = BUSY_WORK;
							ct_reportcaculateprocetime();
							flg_opwork2ready = true;
							reagent_lock[0][1] =reagent_lock[1][1]=reagent_lock[2][1]=reagent_lock[3][1]= 0;
							printf("Nt_recreagentpara finished B\n");
						}
						break;
					case WORKC:
						//读取流程	
						if (flg_opwork3ready)
							break;
						mb_lockallregent(2);
						if (ISmini_cmdSETEDC == false)
						{
							ISmini_cmdSETEDC = true;

							bs_packetshelfstreach(20, &mini_cmd);
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head,mini_cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error mlock");
						}
						pthread_mutex_lock(&head_lock);
						if (nt_recreagentpara(operate_head3, (const char*)&rtu[1], rtu_len - 1, 2) < 0)
						{
							printf("in mb_canceloperate WORKC\n" );
							tcpdata = -3;//接收失败
							nt_sendpacketdata(WORK_START, &tcpdata, 1);
							mb_setoperateheaddisable((operate_head_list**)&operate_head3, true);
						}
						else
							tcpdata = 3;//开始运行

						if (pthread_mutex_unlock(&head_lock) != 0)
							lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error head_lock");
						if (rtu_len < 10)
						{
							if (!flg_opwork1ready && !flg_opwork2ready && !flg_opwork3ready)
							{
								operate_pri = (volatile operate_head_list *)operate_head3;
							}
							work_cnt++;
							ISmini_cmdSETEDC = false;
							//		tcpdata = 3;
							//		nt_sendpacketdata(WORK_START,&tcpdata,1);
							
							mb_sethydratepale(3);			
							wkeventC = BUSY_WORK;
							ct_reportcaculateprocetime();
							flg_opwork3ready = true;
							reagent_lock[0][2] =reagent_lock[1][2]=reagent_lock[2][2]=reagent_lock[3][2]= 0;
							printf("Nt_recreagentpara finished C\n");
						}
						//mb_setoperateheaddisable(&operate_head3, TRUE);
						//	mb_printfoperatelist(operate_head3);					
						break;
					case ChangeReagentA:
					//	nt_recvchangereagentinfo(operate_head1, &rtu[1],  rtu_len - 1, 0);
						break;
					case ChangeReagentB:
					//	nt_recvchangereagentinfo(operate_head2, &rtu[1],  rtu_len - 1, 1);
						break;
					case ChangeReagentC:
					//	nt_recvchangereagentinfo(operate_head3, &rtu[1],  rtu_len - 1, 2);
						break;
					case ChangeReagent2:
						pthread_mutex_lock(&head_lock);
						nt_recvchangereagentinfo((char*)&rtu[1], rtu_len - 1);
						pthread_mutex_unlock(&head_lock);
						break;
					case REAGNET_INFO: 
						nt_chargespecialreagent((char*)&rtu[1]);
						special_reagent_recieved = true;
						break;
					case CHECK_REAGENT:	//格式 CHECK_REAGENT + 1字节 坐标位置
						printf("****************CHECK_REAGENT**********************\n");
						flg_checkreagentindex[rtu[1]] = true;
						break;
					case REPLACE_REAGENT:
						replace_reagent = rtu[1];
						break;
					case WORK_NORMAL:
						wkevent = FREE_WORK; 	
						break;
					case PAUSE_NOW:
						wkevent = PAUSE; 
						break;
					case STOP_NOW:		
						pthread_mutex_lock(&mutex_tryworklock);
						if (pthread_mutex_unlock(&mutex_tryworklock) != 0)
							lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mutex_tryworklock");
						if (rtu[1] == 5)
						{		
							// flg_mainproexit = true;  andry modefy 
						}
						else
						{	
							if (rtu[1] == 1)
							{				
								//	mb_setoperateheaddisable((operate_t**)&operate_head1,TRUE);
								wkeventA = STOP_WORK;
								hydrateA.flage = false;	
								mb_mainworkstop(1);	
							}
							if (rtu[1] == 2)
							{
								//	mb_setoperateheaddisable((operate_t**)&operate_head2,TRUE);
								wkeventB = STOP_WORK;
								hydrateB.flage = false;	
								mb_mainworkstop(2);	
							}
							if (rtu[1] == 3)
							{
								//	mb_setoperateheaddisable((operate_t**)&operate_head3,TRUE);
								wkeventC = STOP_WORK;
								hydrateC.flage = false;	
								mb_mainworkstop(3);	
							}
							if (rtu[1] == 4)
							{
								//	mb_setoperateheaddisable((operate_t**)&operate_head1,TRUE);
								//		mb_setoperateheaddisable((operate_t**)&operate_head2,TRUE);
								//		mb_setoperateheaddisable((operate_t**)&operate_head3,TRUE);
								wkevent = STOP_ALL; 
								hydrateA.flage = false;	
								hydrateB.flage = false;	
								hydrateC.flage = false;	
								mb_mainworkstop(1);	
								mb_mainworkstop(2);	
								mb_mainworkstop(3);	
							}	
						}
						//GPIO_OutSet(VP1);
						ct_reportcaculateprocetime();
						break;
					case STANDBY:
						//	GPIO_OutSet(VP1);
						wkevent = STANDBY_WORK;		
						break;
					case CHECK_PROBLEM:
						memcpy(&check_cmd, &rtu[1], 4);
						printf("check_cmd = %d", check_cmd );
						if (check_cmd == MIX_STATION_MOVED)
							flg_sancmixstatioin = true;
						
						else if (check_cmd == WAST_LIUID_LOW_FULL)
							OUT_WASTHIGH_L = false;
						else if (check_cmd == WAST_LIUID_HIGH_FULL)
							OUT_WASTHIGH_H = false;
						break;
						
					case PROBE_CLR:
						printf("PROBE_CLR");
						wkevent = PROBE_CLR_WORK;
						reagent_clr[0] = rtu[1];
						reagent_clr[1] = rtu[2];
						reagent_clr[2] = rtu[3];
						break;
					case DELAY_START:
						if (rtu[1] == 1	)
						{
							reagent_lock[0][0] = reagent_lock[1][0] = reagent_lock[2][0] = reagent_lock[3][0] = 0;
							wkeventA = STANDBY_WORK;
							
							bs_packetshelfstreach(0, &mini_cmd);
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head, mini_cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
						}
						if (rtu[1] == 2	)
						{
							reagent_lock[0][1] = reagent_lock[1][1] = reagent_lock[2][1] = reagent_lock[3][1] = 0;
							wkeventB = STANDBY_WORK;
							bs_packetshelfstreach(10,&mini_cmd);
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head,mini_cmd);
							if (pthread_mutex_unlock(&mutex_mlock) != 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
						}
						if (rtu[1] == 3	)
						{
							reagent_lock[0][2] = reagent_lock[1][2] = reagent_lock[2][2] = reagent_lock[3][2] = 0;
							wkeventC = STANDBY_WORK;
							bs_packetshelfstreach(20,&mini_cmd);
							pthread_mutex_lock(&mutex_mlock);
							set_minicmd(pcmd_head,mini_cmd);
							if (pthread_mutex_unlock(&mutex_mlock)!= 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error mlock");
						}
						break;
						
					case RUNINGCONFIG:
						bs_writeconfiginfo((char*)&rtu[1]);
						mb_readrunparainfo();
						break;
				
					default:
						break;
				}
			}
		}
		usleep(10000);
	}

	return 0;
}



void nb_reconnectnet(int sign)
{
	if (sign == SIGPIPE)
	{
		//	printf("lose connect with net begining to try again\n");//信号处理函数中为不安全函数
		write(STDERR_FILENO,"lose connect with net begining to try again\n", 80);
		flg_netdisconnect = true;//BOOL 类型 为原子操作不会被中断 相当于 sig_atomic_t 
		//	close(local_fd);		
		//		nt_createsocket();
		signal(SIGPIPE, nb_reconnectnet);//重新注册
	}

	if (SIGALRM == sign)
	{
		//	printf("SIGALRM\n");//信号处理函数中为不安全函数
	}
}



void nt_thread_listensocket(void *arg)
{
	nt_packetparse();
}



/******************************************************************************
*
* Function Name  : nt_sendpacketdata
* Description    : .向网络端口发送数据
* 					 
* Input		   :char cmd 命令头, char * data 数据指针, int data_len 数据长度
* Output		   :None
* Return		   :  int 错误-1 正确 返回长度
*******************************************************************************/
int nt_sendpacketdata(char cmd, char *pdata, uint32_t data_len)
{	
		uint32_t send_framelen = 0, fd_num = 0, sendedlen = 0;
		int res = 0;
		struct timeval timeout;
		fd_set fd_write;
		fd_set fd_read;
		uint8_t net_sendbuf[5000] = {0};
		uint8_t pbkbufdata[5000] = {0};
		stnet_packet *pbufdata = (stnet_packet *)net_sendbuf;

		if (flg_netdisconnect || pdata == NULL) {
			return -2;
		}
				
		strncpy(pbufdata->head, NET_PACKET_HEAD, NET_LEN_HEAD);
		pbufdata->datalen = data_len + 1;
		pbufdata->cmdmain = cmd;
		
		if(data_len > 0)
		{
			memcpy(net_sendbuf + NET_LEN_HEADFIED, pdata, pbufdata->datalen);
		}
		send_framelen = NET_LEN_HEADFIED + data_len;
 // printf("net senddata headlen[%d] len [%d], datalen=[%d], cmd=[%d].\n", NET_LEN_HEADFIED, send_framelen, pbufdata->datalen, pbufdata->cmdmain);
		pthread_mutex_lock(&netsend_lock);
		while(!flg_mainproexit)
		{
			if (flg_netdisconnect) 
			{
				return -4;
			}
			
			timeout.tv_sec = 0;
			timeout.tv_usec = 10000;
			FD_ZERO(&fd_write);		
			fd_num = local_fd + 1;
			FD_SET(local_fd, &fd_write);
		
			res = select(fd_num, &fd_read, &fd_write, NULL, &timeout);	
			if(res < 0) {
				perror("[nt_sendpacketdata] select error");
				if(pthread_mutex_unlock(&netsend_lock) != 0)
					lprintf(log_my, ERROR, "NET %s", "pthread_mutex_unlock error select netsend_lock");
				return -5;
			}

			if (FD_ISSET(local_fd, &fd_write))
			{
				SEND_AGAIN:
				//	if (cmd == HEART_BEAT) printf("HEART_BEATHEART_BEATHEART_BEATHEART_BEATHEART_BEATHEART_BEAT\n");
				if ((res = write(local_fd, net_sendbuf, send_framelen)) < 0) //if (send(local_fd, netbuf_write, data_len, MSG_NOSIGNAL) < 0)
				{
					if(pthread_mutex_unlock(&netsend_lock) != 0)
						lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error netsend_lock");
					return -6;			
				}
				else
				{
					// #if(USE_LOG_INFO == 1)
					// for(j = 0; j < res; j++){
					// 	printf("Net send data[%d] 0x%2x.  ", j, net_sendbuf[j]);
					// }
					// printf("\n");
					// #endif
					sendedlen += res;
				}

				if(res < send_framelen)  // 缓存不够 发送余下的  不应该是  if(sendedlen < send_framelen) 吗？？？？
				{
					send_framelen -= res;
					memcpy(pbkbufdata, net_sendbuf + res, send_framelen);
					memcpy(net_sendbuf, pbkbufdata, send_framelen);
					continue;
				}
				break;
			}
			usleep(1000);
		}
		if(pthread_mutex_unlock(&netsend_lock) != 0)
			lprintf(log_my, ERROR, "NET %s", "pthread_mutex_unlock error netsendoverload_lock");
		// #endif
	return sendedlen;
}


/******************************************************************************
*
* Function Name  : set_exist_error
* Description    : .向错误存在链表中插入值
* 					 
* Input		   : int
* Output		   : None
* Return		   :  None
*******************************************************************************/
void set_exist_error(int error_code)
{
	sterrcode_list * perror = perror_exist_head;

	//pthread_mutex_lock(&error_exist_lock);
	if (perror_exist_head->error_code == NO_ERROR)
	{
		perror_exist_head->error_code = error_code;
	}else
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
	//pthread_mutex_unlock(&error_exist_lock);
}



/***********返回数据给PC端************/
void nt_reportpacketinfo(void)
{
	unsigned int write_len = 0;
	emERRORCODE error_code = NO_ERROR;
	emERRORCODE error_table[100] = {0};
	static unsigned char temp_value_report[30] = {0};
	int i = 0;
	char net_sendbuf[512] = {0};
	static unsigned int head1step = 0, head2step = 0, head3step = 0;
	static unsigned short last_bigcabin_value[18] = {0};

	if (perror_head->error_code != NO_ERROR || perror_exist_head->error_code != NO_ERROR)
	{	
		memset(net_sendbuf, 0, sizeof(net_sendbuf));
		write_len = 0;
		i = 0;
		while((error_code = mb_geterrorcode()) != NO_ERROR)
		{	
			if ((error_code & 0X00FFFFFF) == NO_ERROR)  //有可能是最高位至1的NO_ERROR
				continue;
			if ((error_code & 0XFF000000) > 0)  //error has been solved
			{
				if (!mb_finderrorcodeinlist(error_code, true))	//delete the error as it was solved
				{
					printf("error has[%d] been solved but it does't occured before\n", error_code);
					continue;
				}
			}
			else if (mb_finderrorcodeinlist(error_code, false))	 //error occured but the error has happened before
			{
				//  1107 取消
				/*
				if (error_code != DOOR_OPEN && error_code != WAST_LIUID_LOW_FULL
					&& error_code != WAST_LIUID_HIGH_FULL)//除了门错误需要发送
				continue;
				*/
			}
			else	//new error set exist
			{
				printf("set new error in exist\n");
				set_exist_error(error_code);
			}
			
			/*********send new error or solved error***********/		
			error_table[i] = error_code;
			memcpy(&netbuf_write[write_len], &error_code, 4);
			write_len += 4;
			i++;
		}
		if (error_table[0] != 0)	//code needs to be sent
		{
			printf("in report error\n");
			nt_sendpacketdata(ERROR_WARNING, (char*)netbuf_write, write_len);
			sleep(1);
			//调用错误处理函数 处理仪器动作
			mb_errorinfoproce(error_table, i + 1);
		}
	}

	if (head1step != workstep_state_a || head2step != workstep_state_b || head3step != workstep_state_c)
	{
		memset(netbuf_write, 0, sizeof(netbuf_write));
		memcpy(((unsigned char *)&netbuf_write[0]), (void*)&workstep_state_a, sizeof(workstep_state_a));  // 4字节A架步骤
		memcpy(((unsigned char *)&netbuf_write[4]), (void*)&workstep_state_b, sizeof(workstep_state_b));  // 4字节B架步骤
		memcpy(((unsigned char *)&netbuf_write[8]), (void*)&workstep_state_c, sizeof(workstep_state_c));  // 4字节C架步骤
		netbuf_write[12] = report_reagentinfo[0];    // 1字节A试剂信息
		netbuf_write[13] = report_reagentinfo[1];    // 1字节B试剂信息
		netbuf_write[14] = report_reagentinfo[2];    // 1字节C试剂信息
		
		nt_sendpacketdata(WORK_STEP, (char*)netbuf_write, 15);  // workstep_state_a=0x01110001表示地一个规程动作全部完成 andry

		head1step = workstep_state_a;
		head2step = workstep_state_b;
		head3step = workstep_state_c;
		pthread_mutex_lock(&head_step_lock);
		if ((workstep_state_a & 0XFFFF0000) == 0X01110000)  //  温度到，滴加完成，时间到
			HEAD1_STEP_SENDED = true;
		if ((workstep_state_b & 0XFFFF0000) == 0X01110000)
			HEAD2_STEP_SENDED = true;
		if ((workstep_state_c & 0XFFFF0000) == 0X01110000)
			HEAD3_STEP_SENDED = true;
		if (pthread_mutex_unlock(&head_step_lock) != 0)
			lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error head_step_lock");
	}

	memset(net_sendbuf, 0, sizeof(net_sendbuf));    //heartbeat	
	net_sendbuf[0] = 25;
	nt_sendpacketdata(HEART_BEAT, net_sendbuf, 1);

	mb_reportshelfstatus();

	if (get_temvale == true)	
	{		
		memset(net_sendbuf, 0, sizeof(net_sendbuf));
		write_len = 6;

		for (i = 0; i < 30;i++)
		{
			if (temp_value[i] > 2550)
			{
				temp_value_report[i] = 0;
			}
			else
			{
				if (new_temper)
				{
					temp_value_report[i] = 1 + (temp_value[i] - temp_setval[i][temp_value[i]/10])/10;//加1°小数点进位
				}
				else	
					temp_value_report[i] = (temp_value[i] - temp_goalval[temp_value[i] / 10]) / 10;
			}	
		}
		
		memcpy((unsigned char *)net_sendbuf, temp_value_report, sizeof(temp_value_report));
		
		nt_sendpacketdata(WORK_TEMP, net_sendbuf, 30);
		get_temvale = false;	
	}

	if (flg_cabinreved)
	{	
		int i = 0;
		#if (USE_LOG_INFO == 1)
		for(i = 0; i < 6; i++)
			printf("[NET] report cabin status[%d] = [%d]--", i, cabin_value[i]);
		#endif
		memset(net_sendbuf, 0, sizeof(net_sendbuf));
		flg_cabinreved = false;
		//	pthread_mutex_lock(&report_lock);
		memcpy((uint8_t*)bigcabin_value, (uint16_t*)cabin_value, sizeof(cabin_value));
		memcpy((unsigned char *)net_sendbuf, (uint8_t*)bigcabin_value, sizeof(bigcabin_value));	
		nt_sendpacketdata(CASE_STATE, net_sendbuf, 36);
		memcpy(last_bigcabin_value, (uint8_t*)bigcabin_value, sizeof(bigcabin_value));	
	}
}


void nt_thread_reportnet(void *arg)
{
	sterrcode_list * perror = perror_exist_head;
	unsigned int dwPinState;
	
	#if(USE_LOG_INFO == 1)
	printf("***************begin to nt_thread_reportnet**************************\n");
	#endif
	
	while(!flg_mainproexit)
	{
		nt_reportpacketinfo();
		usleep(500000);
		continue;
		if (perror_exist_head->error_code != NO_ERROR)	//处理可以不断电解决的报警信息
		{
			perror = perror_exist_head;
			do
			{
				switch(perror->error_code & 0X0000FFFF)
				{
					case DOOR_OPEN: 
					{
						dwPinState = DOOR;
						GPIO_PinState(&dwPinState);
						if ((dwPinState & DOOR) != 0)
						{	
							mb_seterrorcode(DOOR_OPEN | 0X01000000);
						}
						break;
					}
					case MIX_STATION_MOVED:
						break;
					case MIX_STATION_NOTCLEAR:
					break;
					case MIX_STATION_CLEARERROR:
					break;
					case WATERPOUR_WRONG:
					break;
					case WASHPOUR_WRONG:
					break;
					case DEWAXPOUR_WRONG:
					break;
					case ALCOHOLPOUR_WRONG:
					break;
					case ER1POUR_WRONG:
					break;
					case ER2POUR_WRONG:
					break;
					case ASPIRATE_WRONG:
					break;
				
					default:break;
				}
			}while((perror = perror->next) != NULL);
		}
	}
}





