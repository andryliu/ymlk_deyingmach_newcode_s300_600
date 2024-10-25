


#ifndef __NET_BUSS_H__
#define __NET_BUSS_H__


#include "sttype.h"
#include "string.h"

#define NET_USE_MACLLO_SENDBUF  1  //  1

#define NET_PACKET_HEAD "header"
#define NET_LEN_HEAD   strlen(NET_PACKET_HEAD)
#define NET_LEN_PACKET  4
#define NET_LEN_HEADFIED  (NET_LEN_HEAD + NET_LEN_PACKET + 1)  // head field + cmd field + data len



#define NETRDBUF_LEN   10000 
#define  NETWRBUF_LEN  512

#pragma pack(1)

typedef struct 
{
    /* data */
    char head[NET_LEN_HEAD];
    uint32_t datalen;
    uint8_t cmdmain;
    char *databuf;
}stnet_packet;




extern volatile bool flg_mainproexit;
extern reagent_t reagent_code[36]; //试剂架上的试剂
extern mix_t mix_DAB[3];
extern mix_t mix_SECEND[3];
extern unsigned int dispense_mixreagent_timeA_next;	//流程中到滴加混合试剂那一步还有多少时间
extern unsigned int dispense_mixreagent_timeB_next;
extern unsigned int dispense_mixreagent_timeC_next;
extern  operate_head_list* operate_head1;//A架操作头指针
extern  operate_head_list* operate_head2; //B架操作头指针
extern  operate_head_list* operate_head3;
extern unsigned int dispense_mixreagent_timeA;	//流程中到滴加混合试剂那一步还有多少时间
extern unsigned int dispense_mixreagent_timeB;
extern unsigned int dispense_mixreagent_timeC;
extern unsigned int liquid_valA; //试剂的分配量 100 or 150
extern unsigned int liquid_valB;
extern unsigned int liquid_valC;
extern  int mulnum_glb[3];
extern int mulnum_glb2[3];
extern unsigned char netbuf_read[NETRDBUF_LEN];
extern unsigned char netbuf_write[NETWRBUF_LEN];




void nt_thread_listensocket(void *arg);
int nt_isnetmiireg(const char *if_name, int phy_id, int reg_num );
/*  如果机器在扫描试剂扫描到相同的试剂 ，就会发送 PECAIL_REAGENT 命令来，执行 nt_reacreagentpara()  */
void nt_chargespecialreagent(char *netbuf_read);
void nt_thread_reportnet(void *arg);
int nt_sendpacketdata(char cmd, char * data, uint32_t data_len);
int nt_recreagentpara(operate_head_list* operate_head, const char *netbuf_read, int netread_len, uint8_t shelf_num);	//一个tcp包为一片玻片的操作玻片最后操作需为停止操作
int nt_recvchangereagentinfo( char *netbuf_read, int netread_len);
extern bool nt_createsocket( void);




#endif


