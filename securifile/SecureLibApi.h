#ifndef _SECURE_LIB_API_H_
#define _SECURE_LIB_API_H_

#include "sttype.h"



//ͨѶ����Կ
extern uint8_t uaCommMainKey[16];
//EEPROM��Կ
extern uint8_t EEPKey[16];
//IC����ԿA
extern uint8_t ICCKeyA[16];
//IC����ԿA
extern uint8_t ICCKeyB[16];
//��Ӧ����Կ
extern uint8_t uaRFPwd[8];

/* CRC-8  */
uint8_t   Crc8(uint8_t *data, uint16_t size);
/* CRC-16/CCITT */	
uint16_t  Crc16(void *data, uint16_t size);
/* CRC-32/IEEE  */
uint32_t  Crc32(void *data, uint32_t size);

/**************************************************
ͨ�ü��ܺ���
****************************************************/
void Encode(uint8_t Text[8],uint8_t Key[16]);
/**************************************************
ͨ�ý��ܺ���
****************************************************/
void Decode(uint8_t Text[8],uint8_t Key[16]);
/******************************************************************
��MD5��ָ�����ȵ����ݽ���HASH
******************************************************************/
void MD5(uint8_t *Text,uint16_t Len,uint8_t *MAC);

#endif
