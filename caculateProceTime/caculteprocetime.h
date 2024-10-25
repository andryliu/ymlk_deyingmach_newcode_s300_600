

#ifndef __CACULTEPROCETIME_H__
#define __CACULTEPROCETIME_H__

#include "progameconf.h"
#include "sttype.h"






extern int ct_calateimeonehshelftime(operate_head_list* operate_head_p, unsigned char last_reagent,
	time_cal_t* work_time);

extern int ct_getdesttimestp(operate_head_list* comp_head_p,int des_time);
extern int ct_patchstptime(const time_cal_t des_time, const time_cal_t comp_time1,
	const time_cal_t comp_time2, operate_head_list* comp_head_p1, operate_head_list* comp_head_p2);
extern int ct_reportcaculateprocetime(void);

extern bool ct_patchtime(operate_head_list* operate_head);
extern void ct_addtabtime(operate_head_list* new_head, int time);
extern void ct_readstarttime(void);


#endif