



1��  /*  ��λ�����͹�̵�Э���� ǰ���ֽڵ�ͷ��head��+ 6�ֽڵĳ��� + 1�ֽڵ����� + n�ֽ��������ݣ����� A �ܹ����ͨ�� WORKA��0x03)����һ����Ƭ�Ĺ�̣�
��1�ν��յ�һ�ܲ�Ƭ���ʱ���������Ƭ��ʹ���ԭ��λ�ã�����յ�������̵����ݶε� reagent�ֶ���STOP_OPERATE��ʾһ����ɵ�1Ƭ��Ƭ���     */




2:  /*  ɨ�躯��������  bool sc_getscanerhoney(const unsigned char plate_num, bool IsReagent)  
�����ɨ���Լ�ƿ(IsReagent == TRUE)������ɨ�赽�����ݵ���λ�������浽 memcpy(&reagent_code[plate_num], &reagentoutside_head_tmp->reagent_info, sizeof(reagent_t));





*/

3: "
    tp_thread_mainarm() [dowork_MainArm()] �̺߳���



"


4:  operate_head1, 2, 3 ��ȫ�ֺ����б��ڳ����ִ��ʱ�Ϳ����˵�ַ�ռ䣬������λ����WORKA��WORKB�� WORKC�������Ĳ�Ƭ�����Ϣ��ˮ�ϣ��ȶ���Ҳ�Ǹ��ݸ��б�����ȡ�ġ�


5:  operate_pri : ��λ����������Ե� A�� B�� C �ļܹ�̺��·������һ����̣�operate_pri = operate_head1; �� operator_head2, operator_head3.
  \ wkeventA = BUSY_WORK; , wkeventB = BUSY_WORK; , wkeventC = BUSY_WORK; \
  \ ISmini_cmdSETEDA = false; , ISmini_cmdSETEDB = false; , ISmini_cmdSETEDC = false;  \
  \ flg_opwork1ready = true;  , flg_opwork2ready = true; , flg_opwork2ready = true;
