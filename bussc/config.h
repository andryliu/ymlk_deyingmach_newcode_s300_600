




#ifndef __config_h__
#define __config_h__

#define USE_LOG_INFO   1

#define USE_REAGENT_VAL_80    1  // �����120ul��80ul����μӶ���������Ҫ������ 120ul�� 80ul 2 �ֵμӲ�ͬ�Լ���

#define USE_TEMPER_ADD_105  1    // ������λ������105�������¶�ҵ��


#define  HI_LO_


#define  LAIYUE 1  // ��Ȫҽ�ƹ�˾�����Ǹ����ù�˾�Ĳ�Ƭ��������ǲ�һ���ģ�������Ҫ���裬��Ȫ�ǲ���Ҫ�����, ���ǵĳ��������������趨�ģ�����ר�������ñ����ã����԰�Ȫ�ͱ����ĳ��򲻽���棩
//------------------------------------------------------------------------------




#define USE_PRINT_LOG  0 //  1 


/*    //   #define  LAIYUE 1  ��Ȫҽ�ƹ�˾�����Ǹ����ù�˾�Ĳ�Ƭ��������ǲ�һ���ģ�������Ҫ���裬��Ȫ�ǲ���Ҫ�����  // */

  
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

#define S600_CONFIG_DIR    0  // 1 // 0  // ����measur·��
#define S600_MANUAL_PUMPVAL    1    //  ��λ���ֶ���һ�������Ĵ������Լ�ֵ����
#define S600_WRONING_ASPIRATE_WRONG    1  //  S600 ��չ������ˮ���鵽 ��ȡ��Ϻõ� DAB �Լ�ʱ���� ASPIRATE_WRONG�������ӵ��½�̽Һ�߶�2mm����
#define S600_DABCLEAR_DEPTH       0 //   1  //   ���ӳ�ʼ��ʱ̽���ϱ�����100ul���²���Һ���ⲻ�������������̽�����
// #define S600_SHOWN_DOWN    1  // չʾ���²��ϵ��ʼ��̽�⵽��200ul��ϱ�Һ�����ϴ���ɾ�


#define S600_SHWON_DOWN_DABCNG  1  // չʾ���ϵ��ʼ����ϴDAB��ϱ��ڲ���200ULʱ��ϴ�����100��UL���࣬ ����Ļ���ȴ�Ǻõġ�



#if(S600_MANUAL_PUMPVAL == 1)
  #define S600_MANUPUMP_VAL   103 
#endif


#endif



#define UserinfoPath "/mnt/nandflash/userinfo.txt"
  #if(1 == S600_CONFIG_DIR)
    #define MeasureConfigPath  "/usr/libfile/dev_measconf.ini"     //  ��ô��λ��У׼�������ݺ��������ļ����ݣ����� ��Ϊ��ǰDefPath��Ϊdev_measconf.ini.bk������MAIN_ARM_Tö����MIAN_SAVE������DefPath���������������
    #define MeasureConfigDefPath "/usr/libfile/dev_measconf.inibk"
    #define S600log_path    "/usr/libfile/log_1"
    #define g_szConfigPath	"/usr/libfile/dev_initconf.ini"
  #else
    #define MeasureConfigPath "/mnt/nandflash/device_measure_config.ini"
    #define MeasureConfigDefPath "/mnt/nandflash/device_measure_config.inibak"
    #define g_szConfigPath	"/mnt/nandflash/device_init_config.ini"
  #endif

#if(MACH_S600 == 1)  // #if(S600_CHANGE_REGEAT_PUMPTIME_ADD == 1)
  #define SYSTEP_MOR	  102/100  //	 101 / 100	//���Լ������ı���   103
#elif(MACH_S300 == 1)
  #define SYSTEP_MOR		 103 / 100	//���Լ������ı���   103
#endif





#endif

