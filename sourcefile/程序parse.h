



1：  /*  上位机发送规程的协议是 前个字节的头（head）+ 6字节的长度 + 1字节的命令 + n字节数据内容，发送 A 架规程是通过 WORKA（0x03)发送一个玻片的规程，
第1次接收到一架玻片规程时发送命令到玻片架使其回原点位置，如果收到操作规程的数据段的 reagent字段是STOP_OPERATE表示一个完成的1片玻片规程     */




2:  /*  扫描函数解析：  bool sc_getscanerhoney(const unsigned char plate_num, bool IsReagent)  
如果是扫描试剂瓶(IsReagent == TRUE)，发送扫描到的数据到上位机并保存到 memcpy(&reagent_code[plate_num], &reagentoutside_head_tmp->reagent_info, sizeof(reagent_t));





*/

3: "
    tp_thread_mainarm() [dowork_MainArm()] 线程函数



"


4:  operate_head1, 2, 3 的全局函数列表在程序刚执行时就开辟了地址空间，保存上位机名WORKA，WORKB， WORKC发送来的玻片规程信息。水合，等动作也是根据该列表来获取的。


5:  operate_pri : 上位机设置需测试的 A， B， C 哪架规程后下发到最后一个规程（operate_pri = operate_head1; ， operator_head2, operator_head3.
  \ wkeventA = BUSY_WORK; , wkeventB = BUSY_WORK; , wkeventC = BUSY_WORK; \
  \ ISmini_cmdSETEDA = false; , ISmini_cmdSETEDB = false; , ISmini_cmdSETEDC = false;  \
  \ flg_opwork1ready = true;  , flg_opwork2ready = true; , flg_opwork2ready = true;
