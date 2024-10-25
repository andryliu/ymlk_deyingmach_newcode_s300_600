

#ifndef __BUSS_H__
#define __BUSS_H__

#include "sttype.h"
#include "serailminibd.h"
#include "netbuss.h"


extern volatile uint8_t mini_recieve_codeACK;
extern volatile bool In_bs_minishelfwork;
extern volatile bool mini_work_finished[3];
extern volatile bool mini_finished;
extern volatile uint8_t mini_recieve_code_all[3];

extern volatile char shelf_statue[3];




extern volatile stminibd_cmdlist* pcmd_head;
extern volatile stminibd_cmdlist* pcmd_head2;
extern volatile stminibd_cmdlist* pcmd_head3;
extern void bs_stanbywork(void);
void bs_minishelfwork(char shelf, char pos, char cnt);
extern int bs_packetshelfstreach(uint8_t plate_num, stminibd_sendpacket* cmd);
extern void bs_writeconfiginfo(char* data);


#endif



