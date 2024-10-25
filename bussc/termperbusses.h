


#ifndef  __TEMPER_BUSSES_H__
#define  __TEMPER_BUSSES_H__


#include "sttype.h"
#include "temper_control.h"



#define TEMPER_NUM  30




typedef int (*htemper_ctrfunc)(void);


extern volatile bool get_temvale;  
extern volatile bool tem_over_load;
extern volatile bool tem_over_loadA;
extern volatile bool tem_over_loadB;
extern volatile bool tem_over_loadC;
extern volatile bool flg_temperheating1;  // 玻片正在加热中  andry  
extern volatile bool flg_temperheating2;
extern volatile bool flg_temperheating3; 
extern volatile unsigned int temp_value[30];
extern htemper_ctrfunc TemperControl_p[3];
extern volatile bool NEED_851;
extern volatile bool NEED_852;
extern volatile bool NEED_853;
extern volatile bool tem_over_load;
// extern unsigned int temp_value[];//全部30片玻片温度数据


extern void EnvTemp(void);
extern int tb_tempctrolc(void);
extern int tb_tempctrolb(void);
extern int tb_tempctrola(void);
extern int tb_temperinitnew(void);
extern int tp_tempctrnew(char sel, unsigned char i, bool isctrl);






#endif




