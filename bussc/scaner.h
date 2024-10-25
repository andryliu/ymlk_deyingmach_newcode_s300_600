



#ifndef  _SCANNER_H__
#define __SCANNER_H__

#include "sttype.h"


#define SCAN_OFFSET 20


typedef struct 
{
    /* data */
    uint8_t packetlen;
    uint8_t opcode;
    uint8_t mesagsource;
    uint8_t status;
    uint8_t beepcode;
    uint8_t *parambuf;
    uint16_t checksum;
}stscanner_ssipaket;


typedef enum{
    EMSCANNER_
}emscanner_resppcode;


extern volatile volatile bool IMGSNP_ACK;
extern volatile char scanner_acknum;
extern volatile bool IsGetCode;
extern volatile bool IsGetImage, InGetImage; //IsGetImage要扫描标识，为了区别识别图像命令ACK
extern volatile unsigned char image_cmd_len;
extern volatile unsigned int scan_data_len;//扫描器返回数据长度
extern volatile unsigned char scan_OCR_data[50];//扫描器OCR和条形码数据
extern volatile unsigned int scan_OCR_len;//扫描器OCR和条形码数据长度
extern volatile unsigned char flg_scanerworking;	//防止还没接收完全又发送命令
extern volatile unsigned char flg_scannerrecivimage;
extern volatile unsigned char flg_scannerlastimage;
extern volatile unsigned char scan_data[500000];
extern unsigned char ScanBuffer[50000];
extern char Isinitscanban;
extern int new_version;










extern bool sc_getscanerhoney(const unsigned char plate_num, bool IsReagent);
extern bool sc_getcsanbanack(const int fd, const char *Buf1, int len);
extern bool sc_scanergetban(const unsigned char plate_num, bool IsReagent);
extern bool sc_getscode(const unsigned char plate_num, bool IsReagent);
extern bool sc_scanerinition(void);
extern void sc_listenscanhoney(void);




#endif



