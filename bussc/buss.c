

#include "readinifile.h"
#include "serailminibd.h"
#include "buss.h"
#include "string.h"
#include "tpthread.h"
#include <unistd.h>







//pos=100,拉伸盖片100ul到150ul位置
//pos=150,拉伸盖片150ul到初始位置
void bs_minishelfwork(char shelf, char pos, char cnt)
{
	uint8_t shelf_index = shelf;
	stminibd_sendpacket cmd;
	char i = 0;

	while(!mini_work_finished[shelf_index])//抽风命令返回后
	{
		usleep(200000);
	}

	In_bs_minishelfwork = TRUE;
	for(i = 0; i < cnt; i++)
	{	
		bs_packetshelfstreach(shelf_index*10, &cmd);//初始位置
        mini_recieve_codeACK = 0;
        mini_work_finished[shelf_index] = FALSE;
        mini_finished = FALSE;
        mini_recieve_code_all[shelf_index] = 0;
        sm_sendpacket(COM2, 0, cmd.minicmd_num, (char*)&cmd.minicmd_buffer[3]);
        if (!sm_miniwaitinganswer(COM2, (uint8_t*)&mini_recieve_code_all[shelf_index], cmd))//只当接受到ACK时退出
        {	
            mb_seterrorcode(CONNECT_ERROR_MINIA);	
        }
        sleep(1);
        while(!mini_work_finished[shelf_index])  //动作完成
        {
            usleep(200000);
        }
        usleep(500000);
        if(pos == 100)
    	   bs_packetshelfstreach(shelf_index*10 + 60, &cmd);  //100ul
        else
     	   bs_packetshelfstreach(shelf_index*10 + 30, &cmd);  //150ul

        mini_recieve_codeACK = 0;
        mini_work_finished[shelf_index] = FALSE;
        mini_finished = FALSE;
        mini_recieve_code_all[shelf_index] = 0;

        sm_sendpacket(COM2, 0, cmd.minicmd_num, (char*)&cmd.minicmd_buffer[3]);
        if (!sm_miniwaitinganswer(COM2, (uint8_t*)&mini_recieve_code_all[shelf_index], cmd))//只当接受到ACK时退出
        {	
            mb_seterrorcode(CONNECT_ERROR_MINIA);	
        }
        sleep(1);
        while(!mini_work_finished[shelf_index])//动作完成
        {
            usleep(200000);
        }
        usleep(500000);
    }
	In_bs_minishelfwork = FALSE;;
}



//shelf_num 0,1,2     函数功能：  根据参数plate_num%30 整数为A，B，C哪个玻片架， 余数为该架玻片架中的第几片玻片素要做拉伸的动作。
/*  玻片 plate_num：拉伸玻片架档位阈值  */
int bs_packetshelfstreach(uint8_t plate_num, stminibd_sendpacket *cmd)
{
	char shelf_num = 0;  // 玻片架号
	
	printf("3333333dispense[0].plate_num=%d\n", plate_num);

	//	memset(cmd,0,sizeof(stminibd_sendpacket));
		// 哪一架玻片
	if (plate_num % 30 < 10)
		shelf_num = cmd->minicmd_buffer[4] = 1; 
	else if (plate_num % 30 < 20)
		shelf_num = cmd->minicmd_buffer[4] = 2; 
	else
		shelf_num = cmd->minicmd_buffer[4] = 3; 

	cmd->cmd = STRETCH;
	//	cmd->minicmd_buffer[3] = (unsigned char)STRETCH | 0x80;
	cmd->minicmd_buffer[3] = (unsigned char)STRETCH;

	if (plate_num < 30)
	{
		// 发送拉伸玻片架命令到 STM32 
		printf("strech 0\n");
		//sleep(1);
		cmd->minicmd_buffer[5] = 0; // 初始位置	   不能退出
		//	if (shelf_statue[shelf_num - 1] == 3)
		//			return -1;
		
		shelf_statue[shelf_num - 1] = 3;
	}
	else if (plate_num < 60)
	{	
		if (shelf_statue[shelf_num - 1] == 6)
			return -1;
		
		shelf_statue[shelf_num - 1] = 6;
		//  发送拉伸玻片架命令     发送拉伸玻片架命令第一档位
		printf("strech 150\n");    
		//sleep(1);
		cmd->minicmd_buffer[5] = 1; 		
	}
	else if (plate_num < 90)
	{
		if (shelf_statue[shelf_num - 1] == 9)
			return -1;
		
		shelf_statue[shelf_num - 1] = 9;

		printf("strech 100\n");    //  发送拉伸玻片架命令第二档位
		//sleep(1);
		cmd->minicmd_buffer[5] = 2; 
	}
	else if (plate_num < 120)
	{
		if (shelf_statue[shelf_num - 1] == 12)
			return -1;
		
		shelf_statue[shelf_num - 1] = 12;
		
		printf("strech all \n");    // 发生拉伸玻片架第三档位
		//sleep(1);
		cmd->minicmd_buffer[5] = 3; 
	}   //  别家有120ul和80ul拉伸滴加动作，于是要求增加 120ul， 80ul 2 种滴加不同试剂。
	#if(USE_REAGENT_VAL_80 == 1)
	else if (plate_num < 150)
	{
		if (shelf_statue[shelf_num - 1] == 15)  // 12+3
			return -1;
		
		shelf_statue[shelf_num - 1] = 15;
		
		printf("strech all \n");    // 发生拉伸玻片架第三档位
		//sleep(1);
		cmd->minicmd_buffer[5] = 4;    //  80ul 
	}   //  别家有120ul和80ul拉伸滴加动作，于是要求增加 120ul， 80ul 2 种滴加不同试剂。
	else if (plate_num < 180)
	{
		if (shelf_statue[shelf_num - 1] == 18)
			return -1;
		
		shelf_statue[shelf_num - 1] = 18;
		
		printf("strech all \n");    // 发生拉伸玻片架第三档位
		//sleep(1);
		cmd->minicmd_buffer[5] = 5; 
	}   //  别家有120ul和80ul拉伸滴加动作，于是要求增加 120ul， 80ul 2 种滴加不同试剂。)
	#endif
	else{
		printf("strech error \n");
		#if(1 == USE_LOG_INFO)
		lprintf(log_my, ERROR, "strech shelf number error.\n");
		#endif
	}
	cmd->minicmd_num = 3;	

	return 0;
}



void bs_writeconfiginfo(char* data)
{
	printf("data= %d %d %d %d %d", data[0], data[1], data[2], data[3], data[4]);
	
	char data_tmp0[3], data_tmp2[3], data_tmp3[3], data_tmp4[5];

	data[2] = data[2] * 2 + data[1];
		
	sprintf(data_tmp0, "%d", data[0]);
	//	sprintf(data_tmp1, "%d",data[1]);
	sprintf(data_tmp2, "%d", data[2]);
	sprintf(data_tmp3, "%d", data[3]);
	sprintf(data_tmp4, "%d", data[4]);
	rf_writeintforprofile("mb_readconfparatinfo", "DOOR_OPEN_ACTION", data_tmp0, (const char *)MeasureConfigPath, 0);
	
	//rf_writeintforprofile("mb_readconfparatinfo", "NOR_WAST_BARREL",data_tmp1, (const char *)MeasureConfigPath, 0);
	rf_writeintforprofile("mb_readconfparatinfo", "DNG_WAST_BARREL", data_tmp2, (const char *)MeasureConfigPath, 0);
	rf_writeintforprofile("mb_readconfparatinfo", "TEM_OVER_LOAD", data_tmp3, (const char *)MeasureConfigPath, 0);
	rf_writeintforprofile("mb_readconfparatinfo", "TEM_LIMIT", data_tmp4, (const char *)MeasureConfigPath, 0);
}


void bs_stanbywork(void)
{
	char commend_buffer[30][50] = {{0}};
	unsigned int commandNumbers = 0;
	unsigned int i = 0,cnt = 0;
	char string[64];
	char str[64];
	char last_perfusion_port = 0; //主臂
	stminibd_cmdlist* pcmd_head_p = NULL;


	printf("in ReadStanbyInfo\n");
	commandNumbers = rf_readintfromeprofile("stanby device", "COMMAND_NUMBERS", -1, (const char *)g_szConfigPath);
	memset(string, '\0', sizeof(string));
	rf_readstringfromeprofile("stanby device", (const char *) "COMMAND_STRING", string, 64, NULL, (const char *) g_szConfigPath);
		assert(string != NULL);

	printf("commandNumbers=%d\n",commandNumbers);
	printf("\n");

	for (i = 0; i < commandNumbers; i++)
	{
		memset(commend_buffer, '\0', sizeof(commend_buffer));
		memset(str, '\0', sizeof(str));
		sprintf(str, "%s%d", string, i+1);
		if (!rf_readstringfromeprofile("stanby device", 
			(const char *)str, &commend_buffer[i][0], 50, NULL, (const char *)g_szConfigPath)) {
			continue;
		}
	
		printf("str=%s\n",str);
		printf("&commend_buffer[i][0]=%s\n",&commend_buffer[i][0]);		
		memset(str, '\0', sizeof(str));	
	}
	printf("ReadStanbyInfo finished\n");
	printf("\n");
	
	while( cnt < commandNumbers )
	{
		if (cnt < 6)	//区分自控臂的灌注操作
			pcmd_head_p = (stminibd_cmdlist*)pcmd_head;
		else if (cnt >= 6 && cnt < 12)
			pcmd_head_p = (stminibd_cmdlist*)pcmd_head2;
		else if (cnt >= 12 && cnt < 18)
			pcmd_head_p = (stminibd_cmdlist*)pcmd_head3;
		
		if (strcmp("PERFUSIONDEWAX", commend_buffer[cnt]) == 0){

			printf("***************in PERFUSIONDEWAX**************\n");
				last_perfusion_port = DEWAXPORT;
		}

		if (strcmp("PERFUSIONALCOHOL", commend_buffer[cnt]) == 0){

			printf("***************in PERFUSIONALCOHOL**************\n");
				last_perfusion_port = ALCOHOLPORT;
		}
		if (strcmp("PERFUSIONER1", commend_buffer[cnt]) == 0){

			printf("***************in PERFUSIONER1**************\n");
				last_perfusion_port = ER1PORT;	
		}
		if (strcmp("PERFUSIONER2", commend_buffer[cnt]) == 0){

			printf("***************in PERFUSIONER2**************\n");
				last_perfusion_port = ER2PORT;
		}
		if (strcmp("PERFUSIONWASH", commend_buffer[cnt]) == 0){
			printf("***************in PERFUSIONWASH**************\n");
				last_perfusion_port = WASHPORT;		
		}
		if (strcmp("PERFUSIONWATER", commend_buffer[cnt]) == 0){
		//	break;
			printf("***************in PERFUSIONWATER**************\n");
				last_perfusion_port = WATERPORT;
		}
		if (strcmp("ASPIRATELIQUID", commend_buffer[cnt]) == 0){
				printf("***************in ASPIRATELIQUID**************\n");
		//		mb_dischargeshelfwateliquid(1);
		//		mb_dischargeshelfwateliquid(2);
		//		mb_dischargeshelfwateliquid(3);
		//		mb_dischargeshelfwateliquid(4);
		}

		commend_buffer[cnt][9] = '\0';
		if (strcmp("PERFUSION", commend_buffer[cnt]) == 0)
		{
			if (cnt >= 18)
			{	
				tp_washchange(TRUE);
				last_perfusion_port = WATERPORT;
				if (last_perfusion_port == WATERPORT || last_perfusion_port == WASHPORT)
				{
					if (!mb_waterwashreagentpour(last_perfusion_port))
					{
						if (last_perfusion_port == WATERPORT)
							mb_seterrorcode(WATERPOUR_WRONG);
						else if (last_perfusion_port == WASHPORT)
							mb_seterrorcode(WASHPOUR_WRONG);
					}
				}
	
				else
				{
					if (!mb_muiltreagentpour(last_perfusion_port))
					{
						if (last_perfusion_port == ALCOHOLPORT)
							mb_seterrorcode(ALCOHOLPOUR_WRONG);
						else if (last_perfusion_port == DEWAXPORT)
							mb_seterrorcode(DEWAXPOUR_WRONG);	
						else if (last_perfusion_port == ER1PORT)
							mb_seterrorcode(ER1POUR_WRONG);
						else if (last_perfusion_port == ER2PORT)
							mb_seterrorcode(ER2POUR_WRONG);
					}
				}
				tp_washchange(FALSE);
			}
			else
			{
			
			}
		}	
		cnt++;
	}
					
	//	mb_dischargwateliquid_lo();
	//	mb_dischwasteliquid_hi();

	printf("out of bs_stanbywork********************\n");
}





