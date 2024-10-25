
#include "serailbussfile.h"
#include "sys/termios.h"
#include "sys/select.h"
#include "scaner.h"
#include "string.h"
#include "unistd.h"
#include "netbuss.h"
#include "log.h"
#include "errno.h"
#include "midbus.h"




volatile char scanner_acknum = 0;
char Isinitscanban = 0;
volatile bool IsGetCode = false; 
volatile volatile bool IMGSNP_ACK = false;
volatile bool IsGetImage = false, InGetImage = false;//IsGetImage要扫描标识，为了区别识别图像命令ACK
volatile unsigned char image_cmd_len = 0;
volatile unsigned int scan_data_len = 0;//扫描器返回数据长度
volatile unsigned char scan_OCR_data[50] = {0};//扫描器OCR和条形码数据
volatile unsigned int scan_OCR_len = 0;//扫描器OCR和条形码数据长度
volatile unsigned char flg_scanerworking = 1;	//防止还没接收完全又发送命令
volatile unsigned char flg_scannerrecivimage = false;
volatile unsigned char flg_scannerlastimage = false;
volatile unsigned char scan_data[500000] = {0};  //扫描器返回的数据
unsigned char ScanBuffer[50000];

extern int serail_hand[10];
extern volatile int  new_scaner, new_temper, honey_scaner;
extern reagentoutside_list * reagentoutside_head;
extern reagent_t reagent_code[36];
extern volatile bool flg_mainproexit;


char sc_senddata(uint8_t *buf, uint16_t size)
{
	char ret = 0;
	char parambuf[40] = {0x26, 0xC6, 0x04, 0x00, 0xFF, 0xF0, 0x31, 0x40, 0xF0, 0x30, 0x01, 0x88, 0x14, 0x80, 0x00, 0xF0,
		0x2D, 0x01, 0xF4, 0xF0, 0x3B, 0x00, 0x82, 0xF4, 0xF0, 0x3C, 0x01, 0x28, 0xF4, 0xF0, 0x3D, 0x01, 0x00, 0xF4,
		0xF0, 0x3E, 0x01, 0xE6, 0xEF, 0x40};
    
    stscanner_ssipaket *psendbuf = (stscanner_ssipaket *)parambuf;
    
    //if("srail is opened")
    /*
    0x26,    // send data lenght (One frame data lenght is no included the checksum)
    0xC6,  //  Opcode fixed.  0xC6: Send Paramet data
    0x04,  // MesageSource fixed ,  0x00: decoder,  0x04: host
    0x00,  //  Status fixed,   0x00:
    0xFF,  //  BeepCode fixed,  0xff: desable beep
    0xF0, 0x31, 0x40; // set the JPEG Quality vaule is 0x40(64), default is 65
    0xF0, 0x30, 0x01,  // select the Image file format is 0x01: JPEG, 0x03: BMP,  0x05: TIFF
    0x88, 0x14      //  Set the decode ssesion timeout is 0x14(20) sec.
    0x80, 0x00,    // Set the power mode is 0x00:continuous on , 0x01: Low power mode
    0xF0, 0x2D, 0x01,  // Image Cropping(图像裁剪)  0x01: enable Image Cropping, 0x00: Disable Image Cropping, Use Full 742 X 480 Pixels
    0xF4, 0xF0, 0x3B, 0x00, 0x82,  //  Crop to Pixel Addresses (Top)  0x00 0x82 (Top = 130)
    0xF4, 0xF0, 0x3C, 0x01, 0x28,  //  Crop to Pixel Addresses (Left) 0x01 0x28 (Left = 296)   0x128
    0xF4, 0xF0, 0x3D, 0x01, 0x00,  //  Crop to Pixel Addresses (Bottom) 0x01 0x00 (Bottom = 256)  0x100
    0xF4, 0xF0, 0x3E, 0x01, 0xE6,  //  Crop to Pixel Addresses (Right)  0x01 0xe6 (Right = 486)  0x1e6
    0xEF, 0x40,   // checksum

    */
    return ret;
 }



 
bool sc_getcsanbanack(const int fd, const char *Buf1, int len)
{
	char ReBuf[50] = {0};
	short check_sum = 0, sum = 0;
	int ack_counter=0, try_connect_again = 0, i;
	char inited = 0;
	
	#if(USE_LOG_INFO == 1)
	printf("Buf= len=%d\n", len );
	for(i = 0; i< len; i++)
		printf(" %x ", Buf1[i]);
	#endif

	memcpy(ReBuf, Buf1, len);
	ReBuf[3] = 1; // set resend

	for (i = 0; i < len - 2; i++)
		sum +=  ReBuf[i];

	check_sum = ~sum + 1;

	ReBuf[len - 2] = (check_sum & 0XFF00) >> 8;	
	ReBuf[len - 1] = check_sum & 0XFF;

	while(scanner_acknum != 1)
	{
		usleep(100000);
		if (ack_counter++ > 50)
		{
			if(!inited)
			{
				ack_counter = 0;
				try_connect_again = 0;
				inited = 1;
				if(!Isinitscanban)
					sc_scanerinition();
			}
			else
			{//	flg_scanerworking = 1;//确保下次扫描
				printf("CONNECT_ERROR_SCANNER\n");
				mb_seterrorcode(CONNECT_ERROR_SCANNER);
				return false;
			}	
		}
		if (try_connect_again++ > 10)
		{		
			try_connect_again = 0;
			#if(USE_LOG_INFO == 1)
			printf("send try_connectd scaner reactivate trigger success\n");	
				printf("ReBuf= len=%d\n",len );
			for(i = 0; i< len;i++)
				printf(" %x ",ReBuf[i]);
			printf("\n");
			#endif

			tcflush( fd, TCOFLUSH);
			write(fd, ReBuf, len);
		}
		else if (scanner_acknum == 2)
		{
			sleep(1);
			#if(USE_LOG_INFO == 1)
			printf("send scaner reactivate trigger success\n");	
			printf("ReBuf= len=%d\n",len );
			for(i = 0; i< len;i++)
				printf(" %x ",ReBuf[i]);
			printf("\n");
			#endif
			tcflush( fd,TCOFLUSH);
			write(fd, ReBuf, len);
		}
	}
	scanner_acknum = 0;

	return true;
}




bool sc_scanergetban(const unsigned char plate_num, bool IsReagent)
{
	char net_reportbuf[100000] = {0};
	int fd	= serail_hand[port_scanner];
	int counter = 0;
	char eenetevent = NO_EVENT;
	const char	  Buf0 = 0;
	const char	  Buf1[6] = {4,0xE4,4,0,0xFF,0x14};//4 //4 E4 4 0 FF 14 START
	// const char	  Buf3[6]={4,0xE5,4,0,0xFF,0x13};//STOP
	char	Buf4[7] = {5,0xF7,4,0,1,0XFE,0XFF};
	bool res = true;
	reagentoutside_list * reagentoutside_head_tmp = reagentoutside_head;
	int image_cnt = 0, code_cnt = 0;

	printf("sc_scanergetban\n");

	flg_scanerworking = 0;
	IsGetImage = false;
	InGetImage = false;
	flg_scannerlastimage = false;
	IsGetCode = false;

	if (!IsReagent)
	{
		scan_data_len = 0;
		scanner_acknum = 0;
		 IsGetImage = 1; 

		#if newxin
		
		#else
		if (write(fd, Buf1, sizeof(Buf1)) < 0)
			printf("send scaner activate trigger failed fd= %d\n", fd);
		else
			printf("send scaner activate trigger success fd= %d\n", fd);

		if (!sc_getcsanbanack(fd, Buf1, sizeof(Buf1)))
			return false;	
		#endif

		 //获取图像指令
		//	while(1)
		{
			 tcflush( fd, TCOFLUSH);
			if (write(fd, Buf4, sizeof(Buf4)) < 0)
				printf("send scaner activate trigger failed fd= %d\n", fd);
			else
				printf("send scaner activate trigger success fd= %d\n", fd);
		}		   
		if (!sc_getcsanbanack(fd, Buf4, sizeof(Buf4)))
			return false;			   
		 //扫描指令
		if (write(fd, Buf1, sizeof(Buf1)) < 0)
			printf("send scaner activate trigger failed fd= %d\n", fd);
		else
			printf("send scaner activate trigger success fd= %d\n", fd);
	
		if (!sc_getcsanbanack(fd, Buf1, sizeof(Buf1)))
			return false;

		while(!flg_scannerlastimage)
		{
			if (flg_scannerrecivimage)
			{		
				flg_scannerrecivimage = false;
				//	printf("flg_scannerrecivimage((((((((((((((((((((((\n");
			}
			usleep(10000);
			if (image_cnt++ > 1000)
			{
				if (write(fd, Buf1, sizeof(Buf1)) < 0)
					printf("send scaner activate trigger failed fd= %d\n", fd);
				else
					printf("send scaner activate trigger success fd= %d\n", fd);
			
				if (!sc_getcsanbanack(fd, Buf1, sizeof(Buf1)))
					return false;
			}
			if (image_cnt++ > 1500)
			{
				printf("CONNECT_ERROR_SCANNER\n");
					mb_seterrorcode(CONNECT_ERROR_SCANNER);
					return false;
				break;
			}
		}
		printf("image_cnt =%d scan_data_len=%d\n", image_cnt, scan_data_len);
		flg_scannerlastimage = false;
		scan_data_len -= 10;
		net_reportbuf[0] = plate_num;
		if (plate_num == 0xED)
			net_reportbuf[0] = 0xEE;
		memcpy(&net_reportbuf[1], (const uint8_t*)&scan_data[10], scan_data_len);//10字节头去掉

		if ((counter = nt_sendpacketdata(SCANNER_IMAGE_SEND, net_reportbuf, scan_data_len + 1)) > 0)
		{
			//		printf("send scanner data to tcp success len = %d*******************\n", counter);
		}
		
		memset((uint8_t*)scan_data, 0, sizeof(scan_data));
		scan_data_len = 0;	
	}
	//扫描指令
	IsGetCode = true;

	write(fd, &Buf0, 1);   //激活扫描头
	usleep(20000);
	if (write(fd, Buf1, sizeof(Buf1)) < 0)
		printf("send scaner activate trigger failed fd= %d\n", fd);
	else
		printf("send scaner activate trigger success fd= %d\n", fd);

	if (!sc_getcsanbanack(fd, Buf1, sizeof(Buf1)))
		return false;

	while(!flg_scanerworking)
	{
		usleep(200000);
		if (code_cnt++ > 10)
		{
			res = false;
			break;
		}
	}

	printf("scan_OCR_data = %s\n", scan_OCR_data);

	memcpy(&net_reportbuf[1], (const uint8_t*)scan_OCR_data, scan_OCR_len);

	if (IsReagent)
		eenetevent = REAGENTCODE_SEND;
	else
		eenetevent = SCANNER_SEND;
		
		//		eenetevent = eenetevent | 0x05000000;
	eenetevent = eenetevent | ((unsigned int )plate_num << 24);
	net_reportbuf[0] = plate_num;
	memcpy(&net_reportbuf[1], (uint8_t*)scan_OCR_data, scan_OCR_len);
		//	if (scan_OCR_len != 0)	//scanner get OCR					//OCR码与二维码数据类似
	{
		if (IsReagent)
		{
			strcpy(reagent_code[plate_num].code, (const char*)scan_OCR_data);
			while(strcmp(reagentoutside_head_tmp->reagent_info.code, (const char*)scan_OCR_data) != 0)//寻找在已扫描试剂中该试剂是否存在
			{
				reagentoutside_head_tmp = reagentoutside_head_tmp->next;
				if (reagentoutside_head_tmp == NULL)
					break;
			}
			
			if ( reagentoutside_head_tmp != NULL)//存在 FALSE 将不去检测此试剂	
			{
				//填入试剂信息
				memcpy(&reagent_code[plate_num], &reagentoutside_head_tmp->reagent_info, sizeof(reagent_t));
				res = false;
			}
		}

		nt_sendpacketdata(eenetevent, net_reportbuf, scan_OCR_len + 1);
			
		memset((uint8_t*)scan_OCR_data, 0, sizeof(scan_OCR_data));
		scan_OCR_len = 0;				
	}
	
	return res;
}



/**************扫描器*****************
plate_num == 0XEE 为扫描混合站
*****************************************/
/******************************************************************************
*
* Function Name  : sc_getscode
* Description    : 扫描头扫描
* 					 
* Input		   :unsigned char plate_num 玻片位置,BOOL IsReagent 是否为扫描试剂
* Output		   :
* Return		   :  BOOL 错误FLASE 正确TRUE
*******************************************************************************/
bool sc_getscode(const unsigned char plate_num, bool IsReagent)
{
  char TCP_Buffer[50000] = {0};
  int write_len = 0;
  int fd = serail_hand[port_scanner];
  int res_scan = 0, counter=0;
  char eenetevent = NO_EVENT;
	const char Buf1[3]={0x16, 0x54, 0x0D};
	const char Buf3[3]={0x16, 0x55, 0x0D};
	//	const char	  Buf4[11]={22,77,13,'S','C','N','A','I','M','0','.'};
	//	const char	  Buf5[14]={0x16, 0x4D, 0x0D, 0x52, 0x45, 0x56, 0x5F, 0x43, 0x43, 0x3F, 0x2E, 0x16, 0x54, 0x0D};
	bool res = true;
	char try_again = 0;
	reagentoutside_list * reagentoutside_head_tmp = reagentoutside_head; //, *reagentoutside_head_lst = NULL;
	//	char * image_snap_cmd = "IMGSNP2P50E255D150W1L.";
	char * image_snap_cmd = "IMGSNP1L.";
	//	char * image_cmd = "IMGSHP178L463R163T318B14E8F1D3P0~.";
	//	char * image_cmd = "IMGSHP1D14E8F180L460R150T380B3P0~.";//MY
	char * image_cmd = "IMGSHP1D14E8F135L505R150T310B3P0~.";//TST
	//	char * image_cmd = "IMGSHP1D5E8F0IR0IF0T450B3P0~.";
	//	char * image_cmd = "IMGSHP3P1D8F0~.";
	char Buf2[50] = {0};
	/*
	const char	Buf2[42] = {0x16,0x4D,0x0D,0x49,0x4D,0x47,0x53,0x48,0x50,0x38,0x46,0x31,
	0x44,0x33,0x50,0x31,0x32,0x35,0x4C,0x35,0x52,0x35,0x42,0x35,0x4D,0x32,0x49,0x52,0x33,0x30,0x4B,0x30,0x55,0x31,0x35,0x45,0x30,0x49,
	0x46,0x30,0x7E,0x2E};
	*/ 
	static char lat_scan_OCR_data[20] = {0};
	char retry_cnt = 0;
	char get_image_cnt = 0;

	if (honey_scaner)
		return sc_getscanerhoney(plate_num, IsReagent);
	
	if (new_scaner)
		return sc_scanergetban(plate_num, IsReagent);

	while (flg_scanerworking == 0) 
		usleep(1000);

SCAN_AGAIN:
	flg_scanerworking = 0;
	//	sleep(1);
	tcflush( fd,TCOFLUSH);
	//扫描指令
	if (write(fd, Buf1, sizeof(Buf1)) < 0)
	printf("send scaner activate trigger failed fd= %d\n", fd);
	else
	printf("send scaner activate trigger success fd= %d\n", fd);

		//	 usleep(100000);
	while(!flg_scanerworking)
	{
		counter++;
			
		usleep(1000);
		if (counter > 1000) 	//超时说明OCR 码没收到 或连接错误 (连接错误由接收图像处理)
		{
			printf("recived data from scaner time out \n");
			printf("\n");
			counter = 0;
			try_again++;
			if (IsReagent)	//扫描试剂只扫描条形码	没扫到直接return
			{
				printf("send scaner reactivate trigger success\n");
				flg_scanerworking = 1;//确保下次扫描
				tcflush( fd,TCOFLUSH);
				write(fd, Buf3, sizeof(Buf3));
				return false;		
				}
			else 
			{
				tcflush( fd,TCOFLUSH);
				write(fd, Buf3, sizeof(Buf3));
				res = false;
				break;
			}
		}
	}
	
	res_scan = 0;
	printf("cmp %s %s\n",lat_scan_OCR_data,scan_OCR_data);
	if (!IsReagent)
	{
		if ( (strcmp((const char*)scan_OCR_data, lat_scan_OCR_data) == 0) && scan_OCR_data[0] != 0 && retry_cnt < 2)
		{
			retry_cnt++;
			sleep(1);
			goto	SCAN_AGAIN;
		}
			
		if ( (strcmp((const char*)scan_OCR_data, (const char*)lat_scan_OCR_data) == 0))
		{
			memset((uint8_t*)scan_OCR_data, 0, sizeof(scan_OCR_data));
			scan_OCR_len = 0;
		}
	
	}
	if (IsReagent)
	eenetevent = REAGENTCODE_SEND;
	else
	eenetevent = SCANNER_SEND;
		//		eenetevent = eenetevent | 0x05000000;
	eenetevent = eenetevent | ((unsigned int )plate_num << 24);
	TCP_Buffer[0] = plate_num;
	memcpy(&TCP_Buffer[1], (uint8_t*)scan_OCR_data, scan_OCR_len);
	strcpy(lat_scan_OCR_data, (const char*)scan_OCR_data);
		
	//	if (scan_OCR_len != 0)	//scanner get OCR					//OCR码与二维码数据类似
	{
		if (IsReagent)
		{
			strcpy(reagent_code[plate_num].code, (const char*)scan_OCR_data);
			while(strcmp(reagentoutside_head_tmp->reagent_info.code,(char*)scan_OCR_data) != 0)//寻找在已扫描试剂中该试剂是否存在
			{
				reagentoutside_head_tmp = reagentoutside_head_tmp->next;
				if (reagentoutside_head_tmp == NULL)
					break;
			}
			
			if ( reagentoutside_head_tmp != NULL)//存在 FALSE 将不去检测此试剂	
			{
				//填入试剂信息
				memcpy(&reagent_code[plate_num], &reagentoutside_head_tmp->reagent_info, sizeof(reagent_t));
				res = false;
			}
			//没扫到OCR码或试剂条形码也要将位置信息发送出去
		//		while ((res_scan = nt_sendpacketdata(eenetevent, TCP_Buffer, scan_OCR_len + 1)) < 0) sleep(1);

			nt_sendpacketdata(eenetevent, TCP_Buffer, scan_OCR_len + 1);
				
			memset((uint8_t*)scan_OCR_data, 0, sizeof(scan_OCR_data));
			scan_OCR_len = 0;
		}	
	}
		
	if (!IsReagent)
	{
		counter = 0;

GET_IMAGE:
		 //获取图像指令 
		Buf2[0]=0x16;
		Buf2[1]=0x4D;
		Buf2[2]=0x0D;
		memcpy(&Buf2[3],image_snap_cmd,strlen(image_snap_cmd));
		write_len = write(fd, Buf2, (strlen(image_snap_cmd) + 3));
		counter = 0;
		while(!IMGSNP_ACK)
		{
			counter++;
			usleep(10000);
			if (counter > 500)
			{
				flg_scanerworking = 1;//确保下次扫描
				printf("CONNECT_ERROR_SCANNER\n");
				mb_seterrorcode(CONNECT_ERROR_SCANNER);
				return false;
			}
		} 
		Buf2[0]=0x16;
		Buf2[1]=0x4D;
		Buf2[2]=0x0D;
		memset(&Buf2[3], 0, strlen(image_snap_cmd));
		memcpy(&Buf2[3],image_cmd,strlen(image_cmd));
		
		write_len = write(fd, Buf2, (strlen(image_cmd) + 3));

		tcflush( fd,TCOFLUSH);
		write(fd, Buf3, sizeof(Buf3));
		
		flg_scanerworking = 0;
		counter = 0;
		while (flg_scanerworking == 0)
		{
			counter++;
			usleep(10000);
			if (counter > 500)
			{
				flg_scanerworking = 1;//确保下次扫描
				printf("CONNECT_ERROR_SCANNER\n");
				mb_seterrorcode(CONNECT_ERROR_SCANNER);
				return false;
			}
		}
		printf("scan_OCR_data = %s\n", scan_OCR_data);
		memcpy(&TCP_Buffer[1], (uint8_t*)scan_OCR_data, scan_OCR_len);
		if (scan_OCR_data[0] == 0 && plate_num == 0XED)//第一次混合站没扫到
			{}
		else
			nt_sendpacketdata(eenetevent, TCP_Buffer, scan_OCR_len + 1);
				
		memset((uint8_t*)scan_OCR_data, 0,sizeof(scan_OCR_data));
		scan_OCR_len = 0;

		if (scan_data_len == 0 && get_image_cnt == 0)
		{
			get_image_cnt++;
			goto GET_IMAGE;
		}
		TCP_Buffer[0] = plate_num;
		if (plate_num == 0XED)
			TCP_Buffer[0] = 0XEE;
		memcpy(&TCP_Buffer[1], (uint8_t*)scan_data, scan_data_len);

		if ((counter = nt_sendpacketdata(SCANNER_IMAGE_SEND, TCP_Buffer, scan_data_len + 1)) > 0)
		{
			//		printf("send scanner data to tcp success len = %d*******************\n", counter);
		}
	
		memset((uint8_t*)scan_data, 0, sizeof(scan_data));
		scan_data_len = 0;
	}
	write(fd, Buf3, sizeof(Buf3));
		//	printf("send scaner stop cmd success fd= %d\n", fd);
	
	return res;
}




void sc_listenscanhoney(void)
{
	fd_set	DeviceRead;
	struct timeval timeout;
	int rel, len, i=0;
	char scan_ack[20] = {0}; 
	
	#if(USE_LOG_INFO == 1)
	printf("[Tp_thread_recvscanner]: Begin to listening Scaner.\n ");	
	#endif

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
			return ;
		}
		else
		{
			if (FD_ISSET(serail_hand[port_scanner], &DeviceRead))
			{	
				if ((len = read(serail_hand[port_scanner], ScanBuffer, sizeof(ScanBuffer))) > 0)
				{
					#if(USE_LOG_INFO == 1)
					printf("port_scanner Request read len = %d.: \n", len);
					for (i = 0; i < len; i++)
						printf(" %x ", ScanBuffer[i]);
					printf("\n\n");	
					#endif
				}
	
				//	memcpy(scan_ack, ScanBuffer, 6);
				//	if (ScanBuffer[0] == 0x16)	//进行图片数据传输
				if(IsGetImage)
				{
					printf("ingetscan\n");
					
					IsGetImage = 0;	
					//	 xmodemReceive(serail_hand[port_scanner],ScanBuffer,image_len);//获取图片数据					
					i=0;
					while(ScanBuffer[++i] != 0XFF);

					printf("i=%d image_cmd_len=%d\n", i, image_cmd_len);
					scan_data_len = len - (i + 1) - (image_cmd_len);	
					printf("scan_data_len=%d\n", scan_data_len);
					memcpy((uint8_t*)&scan_data[0], &ScanBuffer[i], scan_data_len);
					for(i=0;i<scan_data_len;i++)
						printf("%x", scan_data[i]);
					IMGSNP_ACK = false;
					flg_scanerworking = 1;	 
				}
				else if(strcmp((const char*)scan_ack, "IMGSNP") == 0)
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
				else	//OCR�?
				{
					tcflush(serail_hand[port_scanner],TCIFLUSH);
					printf("OCR recieved，scan_OCR_len=%d\n", scan_OCR_len);
					memcpy((uint8_t*)scan_OCR_data, ScanBuffer, len - 2);//多两个字符
					scan_OCR_len = len - 2;
					printf("OCR recieved，scan_OCR_len=%d\n", scan_OCR_len);
					flg_scanerworking = 1;	 
				}
				tcflush(serail_hand[port_scanner], TCIFLUSH);
				memset(ScanBuffer, 0x00, sizeof(ScanBuffer));
			}
		}
	}
}



#define newxin 1
bool sc_scanerinition(void)
{
	int fd = serail_hand[port_scanner];
	char Buf0 = 0;
	// const char	  Buf1[6]={4,0xE4,4,0,0xFF,0x14};//4 //4 E4 4 0 FF 14 START
	// char	Buf2[10]={8,0XC6,4,0,0XFF,0XF0,0X31,64,0XFC,0XCE};// 扫描图像质量
	// char	Buf3[10]={8,0XC6,4,0,0XFF,0XF0,0X2D,0X01,0XFD,0X11};// 扫描图像大小 ENABLE
	// char	Buf4[10]={8,0XC6,4,0,0XFF,0XF0,0X30,0X01,0XFD,0XE};// 扫描图像格式JPG
	//	char	Buf4[10]={8,0XC6,4,0,0XFF,0XF0,0X30,0X03,0XFD,0XE};// 扫描图像格式BMP
	
	// char	Buf5[9]={7,0XC6,4,0,0XFF,0X88,0X14,0XFD,0X94};// 扫描超时时间
	//	char	Buf6[13]={11,0XC6,4,0,0XFF,0XF4,0XF0,0X3B,'1','0','1',0XFB,0XAB};// 扫描图像TOP	
	//	char	Buf7[13]={11,0XC6,4,0,0XFF,0XF4,0XF0,0X3C,'3','0','0',0XFB,0XAA};// 扫描图像LEFT
	//	char	Buf8[13]={11,0XC6,4,0,0XFF,0XF4,0XF0,0X3D,'3','7','9',0XFB,0XA9};// 扫描图像BOTTOM
	//	char	Buf9[13]={11,0XC6,4,0,0XFF,0XF4,0XF0,0X3E,'4','5','0',0XFB,0XA8};// 扫描图像RIGHT
	//	char	Buf10[10]={8,0XC6,4,0,0XFF,0XF0,0X2f,0X00,0XFD,0X10};// 扫描图像灰度
	//	char	Buf11[10]={8,0XC6,4,0,0XFF,0XF0,0X86,0X3A,0XFC,0X59};// 扫描图像白平衡
	//	char	Buf12[10]={8,0XC6,4,0,0XFF,0XF0,0X68,0X01,0XFC,0XD7};// 扫描图像自动曝光DISABLE
	//	char	Buf13[10]={8,0XC6,4,0,0XFF,0XF1,0X38,0X10,0XFC,0XE6};// 扫描图像 GAIN 
	//	char	Buf14[11]={9,0XC6,4,0,0XFF,0XF4,0XF1,0X37,0X9,0XFC,0XE6};// 扫描图像 曝光时间
	//	char	Buf15[10]={8,0XC6,4,0,0XFF,0XF0,0X2c,0X00,0XFD,0X10};// 扫描图像 瞄准点
	//	char	Buf16[10]={8,0XC6,4,0,0XFF,0XF0,0X9D,0X03,0XFD,0X10};// 扫描图像 补光亮度
	//	char	Buf17[10]={8,0XC6,4,0,0XFF,0XF0,0X69,0X01,0XFD,0X10};// 扫描图像 补光disable
	//	char 	Buf18[10]={8,0XC6,4,0,0XFF,0XF1,0X9D,10,0XFD,0XC};// LED亮度
	// char	Buf19[30]={0X1C,0XC6,4,0,0XFF,0XF0,0X2D,1,0XF4,0XF0,
	// 				0X3B,0,130,0XF4,0XF0,0X3C,1,40,0XF4,0XF0,0X3D,1,0,0XF4,0XF0,0X3E,1,230,0XF2,0XE8};//鎴浘
	
	//	char 	Buf20[10]={8,0XC6,4,0,0XFF,0XF1,0X9C,0X10,0XFD,0XC};// AIM 亮度
		
	//	char	Buf21[10]={8,0XC6,4,0,0XFF,0XF1,0X34,3,0XFC,0XE6};// 对比度
		
	//	char	Buf22[10]={8,0XC6,4,0,0XFF,0XF0,0X32,2,0XFC,0XE6};// decode aim
	// char	Buf23[9]={7,0XC6,4,0,0XFF,0X80,0X00,0XFD,0XB0};// 扫描中心识别	
	char setparam[40] = {0x26, 0xC6, 0x04, 0x00, 0xFF, 0xF0, 0x31, 0x40, 0xF0, 0x30, 0x01, 0x88, 0x14, 0x80, 0x00, 0xF0,
		0x2D, 0x01, 0xF4, 0xF0, 0x3B, 0x00, 0x82, 0xF4, 0xF0, 0x3C, 0x01, 0x28, 0xF4, 0xF0, 0x3D, 0x01, 0x00, 0xF4,
		0xF0, 0x3E, 0x01, 0xE6, 0xEF, 0x40};

	Isinitscanban = 1;

	tcflush(fd, TCOFLUSH);

	write(fd, &Buf0, 1);   //激活扫描头
	usleep(200000);

	#if newxin
	write(fd, &setparam, sizeof(setparam));	//激活扫描头
	if (!sc_getcsanbanack(fd, setparam, sizeof(setparam)))
			return false;	
	#else

	write(fd, &Buf23,sizeof(Buf23));	//激活扫描头
	if (!sc_getcsanbanack(fd, Buf23,sizeof(Buf23)))
	return false;	

	write(fd, &Buf3,sizeof(Buf3));  //激活扫描头
	if (!sc_getcsanbanack(fd, Buf3,sizeof(Buf3)))
			return false;
		
	write(fd, &Buf2,sizeof(Buf2));  //激活扫描头
	if (!sc_getcsanbanack(fd, Buf2,sizeof(Buf2)))
			return false;


	write(fd, &Buf4,sizeof(Buf4));  //激活扫描头
	if (!sc_getcsanbanack(fd, Buf4,sizeof(Buf4)))
			return false;
	write(fd, &Buf5,sizeof(Buf5));  //激活扫描头
	if (!sc_getcsanbanack(fd, Buf5,sizeof(Buf5)))
			return false;


	write(fd, &Buf19,sizeof(Buf19));	//激活扫描头
	if (!sc_getcsanbanack(fd, Buf19,sizeof(Buf19)))
			return false;	
	#endif			
	
	Isinitscanban = 0;			

	return true;	
}



bool sc_getscanerhoney(const unsigned char plate_num, bool IsReagent)
{
	char TCP_Buffer[50000] = {0};
	int write_len = 0;
	int fd	= serail_hand[port_scanner];
	int res_scan = 0, counter=0;
	char eenetevent = NO_EVENT;
	const char Buf1[3] = {0x16, 0x54, 0x0D};
	const char Buf3[3] = {0x16, 0x55, 0x0D};
	//	const char	  Buf4[11]={22,77,13,'S','C','N','A','I','M','0','.'};
	// char	Buf4[20]={0};
	//	const char	  Buf5[14]={0x16, 0x4D, 0x0D, 0x52, 0x45, 0x56, 0x5F, 0x43, 0x43, 0x3F, 0x2E, 0x16, 0x54, 0x0D};
	bool res = true;
	char try_again = 0;
	reagentoutside_list * reagentoutside_head_tmp = reagentoutside_head; //, *reagentoutside_head_lst = NULL;
	//	char * image_snap_cmd = "IMGSNP2P50E255D150W1L.";
	//char * image_snap_cmd = "IMGSNP1L.";
	char * image_snap_cmd = "IMGSNP0T.";

	//	char * image_cmd = "IMGSHP178L463R163T318B14E8F1D3P0~.";
	//	char * image_cmd = "IMGSHP1D14E8F180L460R150T380B3P0~.";//MY
		
	//	char * image_cmd = "IMGSHP1D5E8F0IR0IF0T450B3P0~.";
	//	char * image_cmd = "IMGSHP3P1D8F0~.";

	//	char * image_cmd = "IMGSHP1D14E8F135L505R150T310B3P0~.";//TST
	//char * image_cmd ="IMGSHP6F2P.";
	char * image_cmd ="IMGSHP1D6F220L420R150T280B2P.";
	char Buf2[50] = {0};
	/*
	const char	Buf2[42] = {0x16,0x4D,0x0D,0x49,0x4D,0x47,0x53,0x48,0x50,0x38,0x46,0x31,
	0x44,0x33,0x50,0x31,0x32,0x35,0x4C,0x35,0x52,0x35,0x42,0x35,0x4D,0x32,0x49,0x52,0x33,0x30,0x4B,0x30,0x55,0x31,0x35,0x45,0x30,0x49,
	0x46,0x30,0x7E,0x2E};
	*/ 
	static char lat_scan_OCR_data[20] = {0};
	char retry_cnt = 0;
	char get_image_cnt = 0;
	int i=0;

	while (flg_scanerworking == 0) 
	usleep(1000);

	image_cmd_len = strlen(image_cmd);

SCAN_AGAIN:
	flg_scanerworking = 0;
	tcflush( fd,TCOFLUSH);
		//ɨ��ָ��
	if (write(fd, Buf1, sizeof(Buf1)) < 0)    
		printf("send scaner activate trigger failed fd= %d\n", fd);
	else
		printf("send scaner activate trigger success fd= %d\n", fd);

	while(!flg_scanerworking)
	{
		counter++;
			
		usleep(1000);
		if (counter > 1000) 	//��ʱ˵��OCR ��û�յ� �����Ӵ�1�7?(���Ӵ����ɽ���ͼ�񴦄1�7?
		{
			printf("recived data from scaner time out \n");
			printf("\n");
			counter = 0;
			try_again++;
			if (IsReagent)	//ɨ���Լ�ֻɨ��������	ûɨ��ֱ��return
			{
				printf("send scaner reactivate trigger success\n");
				flg_scanerworking = 1;//ȷ���´�ɨ��
				tcflush( fd,TCOFLUSH);
				write(fd, Buf3, sizeof(Buf3));
				return false;		
			}
			else 
			{
				tcflush( fd,TCOFLUSH);
				write(fd, Buf3, sizeof(Buf3));
				res = false;
				break;
			}			
		}
	}
	
	res_scan = 0;
	printf("cmp %s %s\n", lat_scan_OCR_data, scan_OCR_data);

	if (!IsReagent)
	{
		if ( (strcmp((const char*)scan_OCR_data, (const char*)lat_scan_OCR_data) == 0) && scan_OCR_data[0] != 0 && retry_cnt < 2)
		{
			retry_cnt++;
			sleep(1);
			goto	SCAN_AGAIN;
		}
			
		if ( (strcmp((const char*)scan_OCR_data, (const char*)lat_scan_OCR_data) == 0))
		{
			memset((uint8_t*)scan_OCR_data, 0, sizeof(scan_OCR_data));
			scan_OCR_len = 0;
		}
	}
	if (IsReagent)
		eenetevent = REAGENTCODE_SEND;
	else
		eenetevent = SCANNER_SEND;
		
	//		eenetevent = eenetevent | 0x05000000;
	eenetevent = eenetevent | ((unsigned int )plate_num << 24);
	TCP_Buffer[0] = plate_num;
	memcpy(&TCP_Buffer[1], (uint8_t *)scan_OCR_data, scan_OCR_len);
	strcpy(lat_scan_OCR_data, (const char*)scan_OCR_data);
		
	//	if (scan_OCR_len != 0)	//scanner get OCR					//OCR�����ά��������1�7?
	{
		if (IsReagent)
		{
			strcpy(reagent_code[plate_num].code, (const char*)scan_OCR_data);
			while(strcmp(reagentoutside_head_tmp->reagent_info.code, (char*)scan_OCR_data) != 0)//Ѱ������ɨ���Լ��и��Լ��Ƿ����
			{
				reagentoutside_head_tmp = reagentoutside_head_tmp->next;
				if (reagentoutside_head_tmp == NULL)
					break;
			}
			
			if ( reagentoutside_head_tmp != NULL)//���� FALSE ����ȥ�����Լ�
			{
				memcpy(&reagent_code[plate_num], &reagentoutside_head_tmp->reagent_info, sizeof(reagent_t));
				res = false;
			}
			nt_sendpacketdata(eenetevent, TCP_Buffer, scan_OCR_len + 1);//�������ַ� 0X0A,0X0B
			memset((uint8_t*)scan_OCR_data, 0, sizeof(scan_OCR_data));
			scan_OCR_len = 0;
		}	
	}

	if (!IsReagent)
	{
		counter = 0;

	GET_IMAGE:
		Buf2[0]=0x16;
		Buf2[1]=0x4D;
		Buf2[2]=0x0D;
		memset(&Buf2[3], 0, strlen(image_snap_cmd));
		memcpy(&Buf2[3],image_cmd,strlen(image_cmd));
		
		write_len = write(fd, Buf2, (strlen(image_cmd) + 3));
		printf("send scaner get image success fd= %d\n", fd);
		for(i = 0; i<(strlen(image_cmd) + 3);i++)	
			printf("%c",Buf2[i]);
			
		//	tcflush( fd,TCOFLUSH);
		//	write(fd, Buf3, sizeof(Buf3));
		IsGetImage = 1;
		flg_scanerworking = 0;
		counter = 0;
		while (flg_scanerworking == 0)
		{
			counter++;
			usleep(10000);
			if (counter > 500)
			{
				flg_scanerworking = 1;//ȷ���´�ɨ��
				printf("CONNECT_ERROR_SCANNER\n");
				mb_seterrorcode(CONNECT_ERROR_SCANNER);
				return false;
			}
		}
		printf("scan_OCR_data = %s\n", scan_OCR_data);
		
		memcpy(&TCP_Buffer[1], (uint8_t*)scan_OCR_data, scan_OCR_len);
		if (scan_OCR_data[0] == 0 && plate_num == 0XED)//��һ�λ��վûɨ�1�7?
			{}
		else
			nt_sendpacketdata(eenetevent, TCP_Buffer, scan_OCR_len + 1);
				
		memset((uint8_t*)scan_OCR_data, 0, sizeof(scan_OCR_data));
		scan_OCR_len = 0;

		if (scan_data_len == 0 && get_image_cnt == 0)
		{
			get_image_cnt++;
			goto GET_IMAGE;
		}
		TCP_Buffer[0] = plate_num;
		if (plate_num == 0XED)
			TCP_Buffer[0] = 0XEE;
		memcpy(&TCP_Buffer[1], (const uint8_t*)scan_data, scan_data_len);
		//	printf("*******************scan_data_len = %d**********************\n", scan_data_len);
		//	while(nt_sendpacketdata(SCANNER_IMAGE_SEND, TCP_Buffer, scan_data_len + 1) < 0) sleep(1);
		//	if (plate_num >=20)
		if ((counter = nt_sendpacketdata(SCANNER_IMAGE_SEND, TCP_Buffer, scan_data_len + 1)) > 0)
		{
			//		printf("send scanner data to tcp success len = %d*******************\n", counter);
		}

		memset((uint8_t*)scan_data, 0, sizeof(scan_data));
		scan_data_len = 0;
	}
	write(fd, Buf3, sizeof(Buf3));
	//	printf("send scaner stop cmd success fd= %d\n", fd);
		
	return res;
}


