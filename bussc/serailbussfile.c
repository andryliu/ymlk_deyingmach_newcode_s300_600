

#include "serailbussfile.h"
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/time.h>
#include "serailminibd.h"
#include "midbus.h"
#include "log.h"
// #include "../../../../../../usr/local/arm/arm-2009q3/arm-none-linux-gnueabi/libc/usr/include/linux/hdlc/ioctl.h"
// #include <linux/hdlc/ioctl.h>
// #include <asm-generic/termbits.h>
#include <sys/socket.h>
#include <linux/if.h>



/// @brief 
int serail_hand[10];//串口句柄数据
int port_arm, port_pump, port_scanner, cabin_port, mix_port;
 bool flg_startrecvpump = false;
bool flg_recvpump = false;
volatile unsigned char flg_armack = 0;//主臂接收到ack标志
volatile unsigned char sr_armansercode = 0;//主臂接收到answer标志
static char *parminfo = NULL; //读取主臂信息指针
static volatile unsigned char inwait_init_flag = 0; //在受阻后的再次初始化中
volatile bool flg_getedserailarr = false;


unsigned char serail_recvbuf[SEARIAL_BUFEER_SIZE];  //Com Device Read Buffer

pthread_mutex_t armlock_ack = PTHREAD_MUTEX_INITIALIZER; //主臂ack同步锁
pthread_cond_t armcondtion_ack = PTHREAD_COND_INITIALIZER; //主臂ack条件变量
pthread_mutex_t armlock_answer = PTHREAD_MUTEX_INITIALIZER; //主臂answer同步锁
pthread_cond_t armcondtion_answer = PTHREAD_COND_INITIALIZER; //主臂answer条件变量


extern volatile bool flg_mainproexit;
extern volatile bool flg_intosleep;
extern unsigned volatile char pump_readbuf[21];




/******************************************************************************
* Function Name  : sb_serailcreat
* Description    : GPIO initialization, configuration GPIO 0~19 output, 20~31 input mode.
*                      
* Input          : None
* Output         : None
* Return         :  error return -1, success return 0
*******************************************************************************/
int sb_serailcreat(emb_comid com, int baud, enum emcom_bytesize bytes, enum emcom_parity parity)
{
	int fd = 0;
	char serailhand[16];	
	struct termios new_opt;
	int baud_rate;

	sprintf(serailhand,	"/dev/ttyS%d", com );

	if( ( fd = open(serailhand, O_RDWR | O_NOCTTY | O_NONBLOCK) ) < 0 )
	{
		printf("can't open szDevice %s\n", serailhand);
		return -1;
	}
	else
	{
		tcgetattr(fd, &new_opt);
		memset(&new_opt, 0, sizeof(new_opt));
	
		switch (baud)
		{
			case 4800:
				 baud_rate = B4800;
	
			case 9600:
				baud_rate = B9600;
				break;
			case 19200:
				baud_rate = B19200;
				break;
			case 38400:
				baud_rate = B38400;
				break;
			case 57600:
				baud_rate = B57600;
				break;
			case 115200:
				baud_rate = B115200;
				break;
			default:
				baud_rate = 0;
		}
	
		tcflush(fd, TCIOFLUSH);
		cfsetispeed(&new_opt, baud_rate);
		cfsetospeed(&new_opt, baud_rate);
		if (tcsetattr(fd, TCSANOW, &new_opt) != 0)
		{
			perror("tcsetattr::set baud rate failed\n");
			return -1;
		}
	
		new_opt.c_cflag |= CLOCAL;
		new_opt.c_cflag |= CREAD;
		new_opt.c_cflag |= HUPCL;
		new_opt.c_cflag &= ~CRTSCTS;
		new_opt.c_cflag &= ~CSIZE;
		switch(bytes)
		{
		case '5':
			new_opt.c_cflag |= CS5;
			break;
		case '6':
			new_opt.c_cflag |= CS6;
			break;
		case '7':
			new_opt.c_cflag |= CS7;
			break;
		case '8':
			new_opt.c_cflag |= CS8;
			break;
		default:
			new_opt.c_cflag |= CS8;
		}

		switch(parity)
		{
		case MYPARITY_NONE:
			new_opt.c_cflag &= ~PARENB;   /* Clear parity enable */
			new_opt.c_iflag &= ~INPCK;	   /* Enable parity checking */
			break;
	
		case MYPARITY_ODD:
			new_opt.c_cflag |= (PARODD | PARENB);	/* ����Ϊ��Ч��*/
			new_opt.c_iflag |= INPCK;				/* Disable parity checking */
			break;
	
		case MYPARITY_EVEN:
			new_opt.c_cflag |= PARENB;		/* Enable parity */
			new_opt.c_cflag &= ~PARODD; 	/* ת��ΪżЧ��*/
			new_opt.c_iflag |= INPCK;		/* Disable parity checking */
			break;
	
		default:
			fprintf(stderr, "Unsupported parity\n");
			return -1;
		}

		new_opt.c_cflag &= ~CSTOPB;
		if ((parity != PARITY_NONE))
		{
			new_opt.c_iflag |= INPCK;
		}
		new_opt.c_lflag &= ~(ICANON | ECHO | ISIG); 			/*Input*/
		new_opt.c_oflag &= ~OPOST;								/*Output*/
		new_opt.c_cc[VMIN] = 1;
		new_opt.c_cc[VTIME] = 1;
		tcflush(fd, TCIFLUSH);
	
		if(tcsetattr(fd, TCSANOW, &new_opt) != 0)
		{
			perror("Cannot set the serial port parameters");
			return -1;
		}
	}

	printf("serail COM%i bauld:%i, fd = %d is opened.\n", com + 1, baud, fd);

	serail_hand[com] = fd; 
	
	return 1;
}
	
	

/******************************************************************************
*
* Function Name  : CloseAllDevice
* Description    :  GPIO initialization, configuration GPIO 0~19 output, 20~31 input mode.
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void sb_serialclose(uint8_t comnum)
{
	uint8_t i;

	for (i = 0; i < comnum; i++)
	{
		if (serail_hand[i] > 0)
			close(serail_hand[i]);
	}
}



/******************************************************************************
*
* Function Name  : RSP_OME_Command
* Description	 : The string according to the RSP9000 OME communication protocol format function.
*					   
* Input 		 : None
* Output		 : None
* Return		 : None
*******************************************************************************/
int sb_frameconver2rsp9000(char* cmdStr, int cmdStrLength, unsigned char DeviceAddr, unsigned char seq, unsigned char isCmdRepeat, unsigned char *WriteBuffer)
{
	int i;
	int WDataLength = 0;
	unsigned char control = 0;
	unsigned char checksum = 0;

	//数据帧开始字节
	WriteBuffer[0] = STX_FLAG;

	if (isCmdRepeat)
		control = 0x08;	// control the Req is 1, then Command repeated

	control |= 0x40;	// control, 4,5,67 bit is 0100

	switch (seq)
	{
		case 1:
			control |= 0x01;	// seq#0
			break;
		case 2:
			control |= 0x02;	// seq#1
			break;
		case 3:
			control |= 0x03;	// seq#0,#1
			break;
		case 4:
			control |= 0x04;	// seq#2
			break;
		case 5:
			control |= 0x05;	// seq#2,#0				
			break;
		case 6:			
			control |= 0x06;	// seq#2,#1
			break;
		case 7:
			control |= 0x07;	// seq#0,#1,#2
			break;
		default:
			break;
	}
	
	WriteBuffer[1] = control;
	WriteBuffer[2] = 0X31;
	WriteBuffer[3] = DeviceAddr;

	WDataLength += 4;
	
	//Do copy the Message block
	if ((cmdStr != NULL) && (cmdStrLength != 0)) {
		memcpy((void *)&WriteBuffer[4], (void *)cmdStr, cmdStrLength);
		WDataLength += cmdStrLength;
	}

	// The communication data end byte
	WriteBuffer[WDataLength] = ETX_FLAG;
	WDataLength += 1;
	
	// calculate the VRC
	for (i = 0;i < WDataLength; i++) 
		checksum ^= WriteBuffer[i];	// XOR acceptance value

	// The VRC byte
	WriteBuffer[WDataLength] = checksum;
	WDataLength += 1;
	
#if (USE_PRINT_LOG == 1)
	lprintf(log_my, INFO, "[sb_frameconver2rsp9000].Send To Comm:\n");
	for(i = 0;i < WDataLength; i++)
		lprintf(log_my, INFO, " %x ",WriteBuffer[i]);
#endif

	return WDataLength;
}


int CanverFrameContentToXLP6000(char* cmdStr, int cmdStrLength, unsigned char cmdSeqNum, unsigned char isCmdRepeat, unsigned char *WriteBuffer)
{
	int i;
	int WDataLength = 0;
	unsigned char SeqNumber = 0;
	unsigned char checksum = 0;

	//数据帧开始字节
	WriteBuffer[0] = STX_FLAG;
	
	WriteBuffer[1] = PUMP_ADDR;

	if (isCmdRepeat) {			
		SeqNumber |= 0X08;
	}
	
	SeqNumber |= 0x30;	// control the 4,5,6,7 bit is 0011

	switch (cmdSeqNum)	// Here need to comford
	{
		case 1:
			SeqNumber |= 0x01;	// seq#0
			break;
		case 2:
			SeqNumber |= 0x02;	// seq#1
			break;
		case 3:
			SeqNumber |= 0x03;	// seq#0,#1
			break;
		case 4:
			SeqNumber |= 0x04;	// seq#2
			break;
		case 5:
			SeqNumber |= 0x05;	// seq#2,#0 			
			break;
		case 6: 		
			SeqNumber |= 0x06;	// seq#2,#1
			break;
		case 7:
			SeqNumber |= 0x07;	// seq#0,#1,#2
			break;
		default:
			break;
	}

	WriteBuffer[2] = SeqNumber;
	WDataLength += 3;

	//Do copy the Data block
	memcpy((void *)&WriteBuffer[3], cmdStr, cmdStrLength);
	WDataLength += cmdStrLength;
	

	// The communication data end byte
	WriteBuffer[WDataLength] = ETX_FLAG;
	WDataLength += 1;
	
	// calculate the VRC
	for (i = 0;i < WDataLength; i++) 
		checksum ^= WriteBuffer[i];	// XOR acceptance value

	// The VRC byte
	WriteBuffer[WDataLength] = checksum;
	WDataLength += 1;
	
	return WDataLength;
}



/******************************************************************************
*
* Function Name  : sb_findframeheadtailflg
* Description    : .
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int sb_findframeheadtailflg(unsigned char *framebuf, int *FrameLen, int ReadLen)
{
	char *pframedata = NULL;
	int pCnt = 0;
	unsigned char found = 0, start_flag = 0;
	 int counter = 0;
	
	pframedata = (char *)framebuf;

	while (!found && (pframedata != NULL))
	{
		if (counter >= ReadLen) {
			counter = 0;
			return 0;
		}	
		pCnt++;
		
		if (!start_flag && (*pframedata == STX_FLAG))
			start_flag = 1;
		if (start_flag && (*pframedata == ETX_FLAG))
		{
			found = 1;
			*FrameLen = pCnt + 1;
			counter += *FrameLen;
			
			if (counter >= ReadLen) 
				counter = 0;

			return found;
		}	
		pframedata++;
		counter++;
	}

	return found;
}


/******************************************************************************
*
* Function Name  : sb_checkbytesbit
* Description    : Gets one GPIO pin status.
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char sb_checkbytesbit(char x, char n)
{
	assert(n < 8);
	
	return ( x & (0x01 << n) );   //n = 0-7 如要n用 1-8表示则表达式改为( x | (0x01<<(n-1)) ); 
}



/******************************************************************************
*
* Function Name  : RSP_OME_Command
* Description	 : The string according to the RSP9000 OME communication protocol format function.
*					   
* Input 		 : None
* Output		 : None
* Return		 : None
*******************************************************************************/
sr_tecanoemstruct_t* sb_getoemframe(FRAME_TYPE frametype, uint8_t *pframebuf, uint8_t readlen, uint8_t *errorframeflg, int *rspframelen)
{
	int end, LengthCount = 0;  
	int length_needed = 0; 
	uint8_t fcs = 0x00; 
	int flag_found = 0;
	uint8_t is_error_code = 0;

	sr_tecanoemstruct_t *frame = NULL; 	

	// Find start flag and end flag 
	flag_found = sb_findframeheadtailflg(pframebuf, &LengthCount, readlen);
	if (!flag_found) {// no frame started	  and end
		printf("The recieve data is invaild.\n");
		return NULL;	
	}
	
	switch (frametype) {
		case RSP9000_FRAME: 
			length_needed = 6;	// STX, control, arm, device, ETX, fcs  ACK frame

			if (LengthCount >= length_needed)	
			{	  
				while((frame = malloc(sizeof(sr_tecanoemstruct_t))) == NULL)
				{
					sleep(1);
					printf("malloc error sr_tecanoemstruct_t\n");
				}
				
				fcs ^= *pframebuf;	// STX
				pframebuf++;
				
				frame->control = *pframebuf;	
				fcs ^= *pframebuf;	
				pframebuf++;
				
				frame->arm_addr = *pframebuf;    
				fcs ^= *pframebuf;	
				pframebuf++;

				frame->device_addr = *pframebuf;    
				fcs ^= *pframebuf;	
				pframebuf++;
								
				//extract data
				is_error_code = sb_checkbytesbit(frame->control, 4);
				memcpy(frame->data, pframebuf, (LengthCount - length_needed));
				
				if ((!is_error_code) && 	// Is not ack frame, error frame
					( LengthCount - length_needed > 0 )) {

					*errorframeflg = 1;	// 出错
					//if ((frame->data = (unsigned char *)malloc(sizeof(char)*(LengthCount - length_needed)))) 
					memcpy(frame->data, pframebuf, (LengthCount - length_needed));
					fcs ^= frame->data[0]; 
					pframebuf++;
				}
				
				// ack frame or answer frame ETX flag
				//		printf("[sb_getoemframe]: ----------------->>>>>1.\n");
				//*isErrFrame = 2;	// 正常回复
				while(*pframebuf != 0x03)
				{
					fcs ^= *pframebuf;
					pframebuf++;
				}
				fcs ^= *pframebuf;

				if (fcs != *(++pframebuf)) {
					printf("[sb_getoemframe]: Dropping frame: FCS[%02x <---> %02x] doesn't match.\n", fcs, *pframebuf);
					return NULL;
				}
				
				*rspframelen += LengthCount;

				if (*pframebuf++ != NULL)
					parminfo = (char*)pframebuf;
				else
					parminfo = NULL;
			
				//		printf("[sb_getoemframe]: is_error_code = %x, isErrFrame = %d, parminfo = %x.\n", is_error_code, *isErrFrame, parminfo);
			}
			break;
			
		case XLP6000_FRAME:
			length_needed = 5;	// STX, Master address, Status code, ETX, fcs. The pump responing with ACK

			if (readlen >= length_needed)	
			{	 
				while((frame = malloc(sizeof(sr_tecanoemstruct_t))) == NULL)
				{
					sleep(1);
					printf("malloc error sr_tecanoemstruct_t\n");
				}
				fcs ^= *pframebuf;	// STX
				pframebuf++;

				frame->arm_addr = *pframebuf;   // master address 
				fcs ^= *pframebuf;	
				pframebuf++;

				frame->control = *pframebuf;		// status 
				fcs ^= *pframebuf;	
				pframebuf++;
								
				//extract data	 
				if ( readlen - length_needed > 0 ) { // data block frame
						memcpy(frame->data, pframebuf, (LengthCount - length_needed));
						
						for (end = 0; end < (LengthCount - length_needed); end++)	//计算整帧fcs
							fcs ^= (frame->data[end]);	 
				} 
				
				// answer frame ETX flag
				fcs ^= *pframebuf;						
				pframebuf++;

				// check FCS	 
				if (fcs != *pframebuf) 
					printf("Dropping frame: FCS doesn't match\n");	  
			}			
			break;
		default:
			break;
	}

	return frame;
}

int sb_parseframe(FRAME_TYPE frametype, int recvdatlen)
{
	int returncode = -1, pLength = 0;
	unsigned char errframeflg = 0;
	unsigned char frameseq = 0;
	char str_array[5] = {0};
	
	sr_tecanoemstruct_t *frame =  sb_getoemframe(RSP9000_FRAME, serail_recvbuf, recvdatlen, &errframeflg, &pLength);
 
	parminfo = (char*)serail_recvbuf;
	while (frame != NULL) 
	{
		frameseq = frame->control & 0x07;
		if ((frame->control == 0x40) && (frameseq == 0)) 
		{	// ACK 响应帧		
			#if(USE_LOG_INFO == 1)
			printf("sb_parseframe in ack\n");
			#endif

			sr_armansercode = 0;	//防止先接收到answer信息出现commendoverflow现象
			flg_armack = 1;
			pthread_mutex_lock(&armlock_ack);
			pthread_cond_signal(&armcondtion_ack);
			if (pthread_mutex_unlock(&armlock_ack) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error armlock_ack");
		} 
		else 
		{	// 应答帧
			//确保只收到anwser继续执行
			flg_armack = 1;
			pthread_mutex_lock(&armlock_ack);
			pthread_cond_signal(&armcondtion_ack);
			if (pthread_mutex_unlock(&armlock_ack) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error armlock_ack");
			#if(USE_LOG_INFO == 1)
			printf("in answer\n");
			#endif

			switch (errframeflg)
			{				
				case 0:	// answer frame 
				//		printf("[sb_parseframe]: Answer frame %d. \n", isErrorFrame);
					frameseq = 0;
					if ((returncode = sb_armpumpsend(port_arm, NULL, frame->device_addr, frameseq, 0, 1)) < 0)
						printf("[sb_parseframe]: Send ack to RSP9000 error.\n");
					
					//		printf("[sb_parseframe]: Send ack to RSP9000 successful.\n");
				
					if (flg_getedserailarr)
					{
						//		printf("frame->data[0]=%d,frame->data[1]=%d,frame->data[2]=%d,frame->data[3]=%d",
						//			frame->data[0],frame->data[1],frame->data[2],frame->data[3]);
						//	printf("\n");
						
						memcpy(str_array,(serail_recvbuf + 4), recvdatlen - 6 );
						serail_dataarr = atoi(str_array) ;//selfarm -20
					//serail_dataarr = (frame->data[0]-48) * 1000 + (frame->data[1]-48) * 100 + (frame->data[2]-48)* 10  + frame->data[3]-48;
				
					//	flg_getedserailarr = FALSE;
					}
			
					sr_armansercode = 1;
					/*
					pthread_mutex_lock(&armlock_answer);
					pthread_cond_signal(&armcondtion_answer);
					if (pthread_mutex_unlock(&armlock_answer) != 0)
						lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error armlock_answer");
					*/
					break;
					
				case 1:	// error freame
				//	printf("[sb_parseframe]: Error frame %d, send the error code to PC. \n", isErrorFrame);
					frameseq = 0;
					if ((returncode = sb_armpumpsend(port_arm, NULL, frame->device_addr, frameseq, 0, 1)) < 0)
						printf("[sb_parseframe]: Send ack to RSP9000 error.\n");
					
					//		printf("[sb_parseframe]: Send ack to RSP9000 successful.\n");
			
					if ((frame->control & 0xF0) == 0x50)//正常恢复
					{
						sr_armansercode = 1;
						pthread_mutex_lock(&armlock_answer);
						pthread_cond_signal(&armcondtion_answer);
						if (pthread_mutex_unlock(&armlock_answer) != 0)
							lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error armlock_answer");
					}
					else
					{
						if ((frame->data[0] == 0x49) || (frame->data[0] == 0x4b))  //测液体错误在具体步骤中报错
						{
							sr_armansercode = frame->data[0] - 0x40;
							pthread_mutex_lock(&armlock_answer);
							pthread_cond_signal(&armcondtion_answer);
							if (pthread_mutex_unlock(&armlock_answer) != 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error armlock_answer");
							
						}
						else
						{
							if (frame->data[0] == 0x54)
								sr_armansercode = 4;
							else if (frame->data[0] == 0x55)
								sr_armansercode = 5;
							else if (frame->data[0] == 0x56)
								sr_armansercode = 6;
							else
								sr_armansercode = 3;
							printf("sr_armansercode=%d@@@@@@@@@@@@@22/n", sr_armansercode);
							lprintf(log_my, INFO, "sr_armansercode=%d@/n", sr_armansercode);

							pthread_mutex_lock(&armlock_answer);	
							pthread_cond_signal(&armcondtion_answer);
							if (pthread_mutex_unlock(&armlock_answer) != 0)
								lprintf(log_my, ERROR,"%s","pthread_mutex_unlock error armlock_answer");

							//		if (frame->data[0] == 0X54 || frame->data[0] == 0X55 || frame->data[0] == 0X56)
							//			sr_armansercode = 4;
							
							//受阻在具体执行位置发送//主臂命令不可以不发
							if (frame->data[0] != 0X42 && frame->data[0] != 0X43  && frame->data[0] !=  0X48 &&
										frame->data[0] != 0X54 && frame->data[0] != 0X55 && frame->data[0] != 0X56)
								mb_seterrorcode(frame->data[0] | 0X20);	
						}
					}
					break;
				default:
					break;
			}
		}
		
		//	printf("[sb_parseframe]: control = %x, frameseq =  %x, pLength = %d\n", frame->control, frameseq, pLength);
		if (pLength == recvdatlen)
			break;
		else if (frame->control != 0x40)//当多个帧粘连时接受到回复消息时不再处理同个帧后续字节
			break;		
	}
	free(frame);
	frame = NULL;

	return returncode;
}





/******************************************************************************
*
* Function Name  : sb_waitingframeaswer
* Description    : .等待知道主臂动作完成
* 					 
* Input		   :sr_cmdstruct_t * CommandElemt 
* Output		   :None
* Return		   :  BOOL 错误FALSE 正确 TRUE
*******************************************************************************/
int sb_waitingframeaswer(sr_cmdstruct_t * pcmddat)
{
	unsigned int timeout_counter=0,timeout_counter_an = 0,pump_error = 0;
	unsigned char isReaptSnd = 0;
	struct timeval ack_now;
	struct timespec ack_timeout;
	int res_ack=0,pi_cnt = 0,errortmp=0;
	int last_pump_error=0;
	sr_cmdstruct_t cmdtmp;
	bool PUMP_ACK = false;
	int timeout_overall = 0;

	if(flg_intosleep)
		return 0;
	memcpy(&cmdtmp, pcmddat, sizeof(sr_cmdstruct_t));
	//	BOOL res = TRUE;
	if (pcmddat->srdevaddr == PUMP_ADDR)
	{
		while(!flg_mainproexit)
		{
	PUMP_AGAIN:
			usleep(200000);
			// if (sb_armpumpsend(port_pump, "QR", 
			// 	(*CommandElemt).srdevaddr , 0, 0, 2) > 0)
			if (sb_armpumpsend(port_pump, (uint8_t*)("QR"), pcmddat->srdevaddr, 0, 0, 2) > 0)
			{
			//	printf("[sb_waitingframeaswer]: Send message [%d] to tecan device [%x]successful.\n" , 
			//		0, (*CommandElemt).srdevaddr);
			}
			else
			{
				printf("[sb_waitingframeaswer]: Send message [%d] to tecan device [%x]failed.\n", 0, (*pcmddat).srdevaddr);
			}
			
			//	while(!flg_mainproexit)
			{
				if(timeout_overall++ > 500)
					lprintf(log_my, INFO, "allwayssaomepump=%x\n", pump_readbuf[2]);
			
				usleep(100000);
				if (flg_recvpump)
				{
					timeout_counter = 0;
					flg_recvpump = false;
					//	printf("flg_recvpump pump_readbuf[2]=%x\n",pump_readbuf[2]);
	
					if (pump_readbuf[2] == 0x60) 
					{
						lprintf(log_my, INFO, "waitanser pump=0x%d.\n", pump_readbuf[2]);
						return 0;	
					}
					else if ( (pump_readbuf[1] == 0x40) )
					{
						//PUMP_ACK = TRUE;
					}
					else if ( (pump_readbuf[2] & 0x0F) > 0 && pump_readbuf[2] != 0x31) 
					{
						lprintf(log_my, INFO, "pump=%x\n", pump_readbuf[2]);
						//	printf("flg_recvpump pump_readbuf[2]=%x\n",pump_readbuf[2]);
						if ((pump_readbuf[2] & 0x0F) == 9 || (pump_readbuf[2] & 0x0F) == 0x0A
								|| (pump_readbuf[2] & 0x0F) == 7 || (pump_readbuf[2] & 0x0F) == 0x0C)
						{
							if (pump_error == 0)
								last_pump_error = pump_readbuf[2];
							pump_error++;
							if (pump_error > 5)
							{
								mb_seterrorcode(last_pump_error);
								mb_seterrorcode(pump_readbuf[2]);
							}
							else
							{
								sprintf(cmdtmp.cmdbuf, "ZR");
								sb_armpumpsend(port_pump, (uint8_t *)(cmdtmp.cmdbuf), pcmddat->srdevaddr , 0, 0, 2);
								sleep(10);	
								
								lprintf(log_my, INFO,"send pump cmd again cmd=%s\n", pcmddat->cmdbuf);
								printf("send pump cmd again cmd=%s\n", pcmddat->cmdbuf);

								sb_armpumpsend(port_pump, (unsigned char*)(pcmddat->cmdbuf), 
									pcmddat->srdevaddr , 0, 0, 2);
									goto PUMP_AGAIN;
							}
						}
						else 
						{
							mb_seterrorcode(pump_readbuf[2]);
							//	mb_seterrorcode(last_pump_error);
							return -1;
						}	
					}
					memset((uint8_t*)pump_readbuf, 0, sizeof(pump_readbuf));
				}
				else
				{
					printf("timeout_counter %d\n", timeout_counter);	
					if (timeout_counter++ > 80 )
					{	
						mb_seterrorcode(CONNECT_ERROR_PUMPA);
						return -1;
					}
					else if (!PUMP_ACK && timeout_counter % 10 == 0)
					{
						//	sb_armpumpsend(port_pump, CommandElemt->cmdbuf, 
						//	(*CommandElemt).srdevaddr , 0, 1, 2);
					}
				}		
			}
		}	  // end of while(!flg_mainproexit)
	}  // end of if (pcmddat->srdevaddr == PUMP_ADDR)
	else  // srdevaddr is arm_address
	{
		while(!flg_mainproexit)
		{
			gettimeofday(&ack_now, NULL);
			ack_timeout.tv_sec = ack_now.tv_sec + 2;
			ack_timeout.tv_nsec = ack_now.tv_usec * 1000;;
			pthread_mutex_lock(&armlock_ack);
			
			while((flg_armack != 1) && (res_ack != ETIMEDOUT))		
				res_ack = pthread_cond_timedwait(&armcondtion_ack, &armlock_ack, &ack_timeout);
			if (pthread_mutex_unlock(&armlock_ack) != 0)
				lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error armlock_ack");
			flg_armack = 0;
			if (res_ack != ETIMEDOUT)
			{	
				pthread_mutex_lock(&armlock_answer);

				while(sr_armansercode != 1 && (!flg_mainproexit))
				{
					if (strcmp(pcmddat->cmdbuf, "PI")==0 && sr_armansercode == 4)
					{
						mb_seterrorcode(0x70 + sr_armansercode);  //初始化时X轴失步直接报错不能重新初始化
						{

						}
						return -1;
					}

					//	printf("Wile Waiting arm counte[%d] anser[%d].\n", timeout_counter_an, sr_armansercode);

					if ((sr_armansercode == 2) || (sr_armansercode == 3 ) || sr_armansercode == 4 || sr_armansercode == 5 
							|| sr_armansercode == 6 || sr_armansercode == 9 || sr_armansercode == 0x0B)	 //测液体错误 
					{
						break;
					}
					//	res_answer = pthread_cond_wait(&armcondtion_answer, &armlock_answer);
					usleep(100000);
					timeout_counter_an++;
					if (timeout_counter_an > 500)
					{
						if(flg_intosleep)
							return 0;
						mb_seterrorcode(CONNECT_ERROR_ARM);
						if (pthread_mutex_unlock(&armlock_answer) != 0)
							lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error armlock_answer");
						lprintf(log_my, FATAL, "CONNECT_ERROR_ARM sr_armansercode");
						sleep(1);
						return -1;
					}
				}
				//	if (res_answer == 0)
				//		printf("answer recieved\n");
				//	else
				//		perror("armcondtion_answer error\n");
				
				if (pthread_mutex_unlock(&armlock_answer) != 0)
					lprintf(log_my, ERROR, "%s", "pthread_mutex_unlock error armlock_answer");
				#if(USE_LOG_INFO == 1)
				lprintf(log_my, INFO, "sr_armansercode=%d\n", sr_armansercode);
				printf("Sb arm anser code [%d].\n", sr_armansercode);
				#endif
				if (sr_armansercode == 1 || sr_armansercode == 0)
				{
					inwait_init_flag = 0;
					sr_armansercode = 0;
					return 0;
				}
				else if (sr_armansercode == 3)
				{
					inwait_init_flag = 0;	
					sr_armansercode = 0;	
					return -3;
				}
				else if (sr_armansercode == 9)
				{
					sr_armansercode = 0;
					inwait_init_flag = 0;
					return -9;
				}
				else if (sr_armansercode == 11)
				{
					sr_armansercode = 0;
					inwait_init_flag = 0;
					return -11;
				}
				else
				{
					if (inwait_init_flag && (strcmp(pcmddat->cmdbuf,  "PI")==0 || strcmp(pcmddat->cmdbuf, "ZI")==0))
					{
						mb_seterrorcode(0X70 + sr_armansercode);
						sr_armansercode = 0;								
						inwait_init_flag = 0;
						return -3;
					}
					errortmp = sr_armansercode;
					printf("sr_arm anser code=%d!!!!!!!!!!!!!!!/n", sr_armansercode);
					pi_cnt++;
					inwait_init_flag = 1;
					if (pcmddat->cmdbuf[0] == 'Z')
					{
						strcpy(cmdtmp.cmdbuf, "ZI");
						if(sm_serailsenddat(port_arm, &cmdtmp, 1, 0, 1) < 0)
						{
							return -3;
						}
					}
					else
					{
						strcpy(cmdtmp.cmdbuf, "PI");
						if(sm_serailsenddat(port_arm, &cmdtmp, 1, 0, 1) < 0)
						{
							return -3;
						}
					}
					printf("error send again#####################\n");
					lprintf(log_my, INFO, "error send again%s\n", (unsigned char *)(*pcmddat).cmdbuf);

					if (sb_armpumpsend(port_arm, (unsigned char	*)(*pcmddat).cmdbuf, (*pcmddat).srdevaddr, 1, 0, 1) > 0)
					{
						// printf("[sb_waitingframeaswer]: Send message [%d] to tecan device [%x]successful.\n" , 
						// 	(unsigned char	*)(*CommandElemt).cmdbuf, (*CommandElemt).srdevaddr);
					}
					if (pi_cnt == 2)
					{
						inwait_init_flag = 0;
						mb_seterrorcode(0x70 + errortmp);
						return -3;
					}
				}	
			}
			else if(res_ack  == ETIMEDOUT)
			{
				isReaptSnd = 1;	
				timeout_counter++;
				if (timeout_counter > 20)
				{
					if(flg_intosleep)
						return 0;
					mb_seterrorcode(CONNECT_ERROR_ARM);
					lprintf(log_my,FATAL,"CONNECT_ERROR_ARM ");
					sleep(1);
					return -1;
				}

				printf("ack recieve timeout send cmd again seqNum = %d\n", 0 );

				lprintf(log_my, INFO, "ack recieve timeout send cmd again seqNum = %d\n", 0 );
				isReaptSnd = 1;

				sm_serailsenddat(port_arm, pcmddat, 0, 1, 2);
			}
			res_ack = 0;
		}
	}
	return 0;
}



/******************************************************************************
*
* Function Name  : Tecan_OEM_Command
* Description    : RSP 9000 II and XLP6000 command frame .
*                      
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int sb_armpumpsend(emb_comid port, uint8_t *cmdbuf, uint8_t devaddr, uint8_t seq, uint8_t flgreport, uint8_t connmod)
{
	unsigned char WDataLength = 0;	//写入串口的数据长度
	unsigned char WriteBuffer[256]; //Com Device Write Buffer
	int write_len = 0;
	int cmdStrLength = 0;
	int fd = 0;
	static unsigned char seq_num = 1; 

	if (flg_mainproexit)
		return -1;

	if(flg_intosleep)
		return 1;

	if (seq_num > 7)
	{	
		seq_num = 1;
	}
	if (devaddr == ARM_ADDR)
	{
		port = port_arm;
	}
	else
	{
		port = port_pump;
	}
	
	fd =  serail_hand[port];

	if (cmdbuf == NULL)	//接收到answer后 发送ACK
	{
		cmdStrLength = 0;
	}
	else{
		cmdStrLength = strlen((char *)cmdbuf);
	}
		//	if (comPort == port_arm)
		//	if (srdevaddr == ARM_ADDR)
	if(cmdStrLength != 2 || cmdbuf[0] == 'Z')
	{
		#if(USE_LOG_INFO == 1)
		printf("[armpumpsend]: Write cmd [%s]\n ", cmdbuf);
		lprintf(log_my, INFO, "[armpumpsend] Write cmd [%s]\n ", cmdbuf);
		#endif
	}

	/* Pack the frame */
	if (devaddr == ARM_ADDR)
	{
		// the pump connect with RSP9000
		if (cmdbuf != NULL)	//接收到answer后 发送ACK
		{
			WDataLength = sb_frameconver2rsp9000((char *)cmdbuf, cmdStrLength, devaddr, seq_num, flgreport, WriteBuffer);
		}
		else
		{
			WDataLength = sb_frameconver2rsp9000((char *)cmdbuf, cmdStrLength, devaddr, 0, 0, WriteBuffer);
			if (seq != 0)	//seq == 0为 超时重发 
				seq_num++;			//SndCmdStr == NULL发送ACK后 为下一命令准备 自加
		}
	}
	else // the pump connect the RS232 serial independence
	{	
		WDataLength = CanverFrameContentToXLP6000((char *)cmdbuf, cmdStrLength, seq_num, flgreport, WriteBuffer);
		if (seq != 0)	//seq == 0为 超时重发 
			seq_num++;	
	}
	
	flg_armack = 0;
	sr_armansercode = 0;
	tcflush(fd, TCOFLUSH);
	write_len = write(fd, WriteBuffer, WDataLength);

	return write_len;
}






