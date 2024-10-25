




#ifndef __config_h__
#define __config_h__

#define USE_LOG_INFO   1

#define USE_REAGENT_VAL_80    1  // 别家有120ul和80ul拉伸滴加动作，于是要求增加 120ul， 80ul 2 种滴加不同试剂。

#define USE_TEMPER_ADD_105  1    // 增加上位机发送105度设置温度业务。


#define  HI_LO_


#define  LAIYUE 1  // 澳泉医疗公司跟我们跟莱悦公司的玻片拉伸搅拌是不一样的，我们需要搅拌，澳泉是不需要搅拌的, 我们的程序流程是莱阅设定的（他有专利，不让别人用，所以澳泉和贝晶的程序不搅拌版）
//------------------------------------------------------------------------------




#define USE_PRINT_LOG  0 //  1 


/*    //   #define  LAIYUE 1  澳泉医疗公司跟我们跟莱悦公司的玻片拉伸搅拌是不一样的，我们需要搅拌，澳泉是不需要搅拌的  // */

  
#define  SOFT_VER   "5.0.2.1122"  //  "5.0.3.0221"

// #define MACH_S300   1


#define 	USER_DEBUF_INFO  1

#define   DIFF_DISCHARGE_L_H  1

#define  CHANGE_SHELF_ORDER    1   

#if(CHANGE_SHELF_ORDER == 1)
    #define CAL_TIME_NEWCAL  1
#endif



#if( MACH_S300 == 1)
 #define MACH_S600  0
  #define  S600_CABIN_IN_0     0 // 1
#else
#define MACH_S600 1
 #define  S600_CABIN_IN_0     1
#endif

  

#if(MACH_S600 == 1)

#define S600_CONFIG_DIR    0  // 1 // 0  // 更新measur路径
#define S600_MANUAL_PUMPVAL    1    //  上位机手动打一定容量的大容量试剂值调整
#define S600_WRONING_ASPIRATE_WRONG    1  //  S600 参展机做跑水试验到 汲取混合好的 DAB 试剂时报错 ASPIRATE_WRONG，而增加的下降探液高度2mm后解决
#define S600_DABCLEAR_DEPTH       0 //   1  //   增加初始化时探测混合杯内有100ul以下残余液体检测不到的问题而增加探测深度
// #define S600_SHOWN_DOWN    1  // 展示机下层上电初始化探测到到200ul混合杯液体后清洗不干净


#define S600_SHWON_DOWN_DABCNG  1  // 展示机上电初始化清洗DAB混合杯内残余200UL时清洗完后有100多UL残余， 上面的机器却是好的。



#if(S600_MANUAL_PUMPVAL == 1)
  #define S600_MANUPUMP_VAL   103 
#endif


#endif



#define UserinfoPath "/mnt/nandflash/userinfo.txt"
  #if(1 == S600_CONFIG_DIR)
    #define MeasureConfigPath  "/usr/libfile/dev_measconf.ini"     //  怎么上位机校准保存数据后会擦除掉文件内容？？？ 因为以前DefPath名为dev_measconf.ini.bk所以在MAIN_ARM_T枚举中MIAN_SAVE命令会打开DefPath而擦除里面的内容
    #define MeasureConfigDefPath "/usr/libfile/dev_measconf.inibk"
    #define S600log_path    "/usr/libfile/log_1"
    #define g_szConfigPath	"/usr/libfile/dev_initconf.ini"
  #else
    #define MeasureConfigPath "/mnt/nandflash/device_measure_config.ini"
    #define MeasureConfigDefPath "/mnt/nandflash/device_measure_config.inibak"
    #define g_szConfigPath	"/mnt/nandflash/device_init_config.ini"
  #endif

#if(MACH_S600 == 1)  // #if(S600_CHANGE_REGEAT_PUMPTIME_ADD == 1)
  #define SYSTEP_MOR	  102/100  //	 101 / 100	//吸试剂多吸的倍数   103
#elif(MACH_S300 == 1)
  #define SYSTEP_MOR		 103 / 100	//吸试剂多吸的倍数   103
#endif





#endif

