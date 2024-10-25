#ifndef _SECURE_LIB_API_H_
#define _SECURE_LIB_API_H_

#include "sttype.h"



//通讯主密钥
extern uint8_t uaCommMainKey[16];
//EEPROM密钥
extern uint8_t EEPKey[16];
//IC卡密钥A
extern uint8_t ICCKeyA[16];
//IC卡密钥A
extern uint8_t ICCKeyB[16];
//感应卡密钥
extern uint8_t uaRFPwd[8];

/* CRC-8  */
uint8_t   Crc8(uint8_t *data, uint16_t size);
/* CRC-16/CCITT */	
uint16_t  Crc16(void *data, uint16_t size);
/* CRC-32/IEEE  */
uint32_t  Crc32(void *data, uint32_t size);

/**************************************************
通用加密函数
****************************************************/
void Encode(uint8_t Text[8],uint8_t Key[16]);
/**************************************************
通用解密函数
****************************************************/
void Decode(uint8_t Text[8],uint8_t Key[16]);
/******************************************************************
用MD5对指定长度的数据进行HASH
******************************************************************/
void MD5(uint8_t *Text,uint16_t Len,uint8_t *MAC);

#endif
