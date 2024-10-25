#include "stdafx.h"

//#define IDEA_CRYPT
uint8_t uaCommMainKey[16]= {0x9F,0x03,0xC5,0xBE,0x7C,0xE6,0x79,0x77,0xAA,0x59,0x66,0xB0,0xAE,0xC5,0xFA,0x1F};		//通讯主密钥
uint8_t EEPKey[16]= {0x9F,0x03,0xC5,0xBE,0x7C,0xE6,0x79,0x77,0xAA,0x59,0x66,0xB0,0xAE,0xC5,0xFA,0x1F};		//设备中EEPROM中的存储密钥
uint8_t ICCKeyB[16]={0xEB,0xE6,0x79,0x1E,0x75,0xE1,0x37,0xBC,0x88,0x1E,0xC8,0x94,0x60,0x11,0x20,0x94};		//驾训明细的加解密密钥

/******************************************************************************/
/*   IDEA CIPHER                                                              */
/******************************************************************************/

/***************************IDEA Definition************************************/
#define IDEAnofRound                 8 /* number of rounds                   */
#define IDEAuserKeyLen               8 /* user key length (8 or larger)      */

#define mulMod        0x10001 /* 2**16 + 1                                    */
#define ones           0xFFFF /* 2**16 - 1                                    */
#define Mul(a, b)                                                              \
  if (a == 0) a = mulMod - b;                                                  \
  else if (b == 0) a = mulMod - a;                                             \
  else {                                                                       \
	a *= b;                                                                    \
	if ((a & ones) >= (b = a >> 16)) a -= b;                                   \
	else a += mulMod - b;                                                      \
  } /* Mul */

/******************************************************************************/
/* Do not change the lines below.                                             */
#define IDEAdataLen                       4 /* plain-/ciphertext block length*/
#define IDEAkeyLen    (IDEAnofRound * 6 + 4) /* en-/decryption key length   */

#define IDEAdataSize       (IDEAdataLen * 2) /* 8 bytes = 64 bits           */
#define IDEAuserKeySize (IDEAuserKeyLen * 2) /* 16 bytes = 128 bits         */
#define IDEAkeySize         (IDEAkeyLen * 2) /* 104 bytes = 832 bits        */

typedef uint16_t IDEAData[IDEAdataLen];       //8    bytes
typedef uint16_t IDEAUserKey[IDEAuserKeyLen]; //16   bytes
typedef uint16_t IDEAKey[IDEAkeyLen];         //104  bytes
/**********************************************************************************/

/******************************************************************************/
/* Encryption and decryption algorithm ECC. Depending on the value of 'key'  */
/* 'IDEACrypt' either encrypts or decrypts 'dataIn'. The result is stored    */
/* in 'dataOut'.                                                              */
/* pre:  'dataIn'  contains the plain/cipher-text block.                      */
/*       'key'     contains the encryption/decryption key.                    */
/* post: 'dataOut' contains the cipher/plain-text block.                      */

void IDEACrypt (IDEAData dataIn, IDEAData dataOut, IDEAKey key)
{
  uint32_t x0, x1, x2, x3, t0, t1, t2;
  uint16_t round;

  x0 = (uint32_t)*dataIn++; x1 = (uint32_t)*dataIn++;
  x2 = (uint32_t)*dataIn++; x3 = (uint32_t)*dataIn;
  for (round = IDEAnofRound; round > 0; round--) {
	t1 = (uint32_t)*key++;
	x1 += (uint32_t)*key++;
	x2 += (uint32_t)*key++; x2 &= ones;
	t2 = (uint32_t)*key++;
	Mul(x0, t1); x0 &= ones;
	Mul(x3, t2);
	t0 = (uint32_t)*key++;
	t1 = x0 ^ x2;
	Mul(t0, t1); t0 &= ones;
	t1 = (uint32_t)*key++;
	t2 = (x1 ^ x3) + t0 & ones;
	Mul(t1, t2); t1 &= ones;
	t0 += t1;
	x0 ^= t1; x3 ^= t0; x3 &= ones;
	t0 ^= x1; x1 = x2 ^ t1; x2 = t0;
  }
  t0 = (uint32_t)*key++;
  Mul(x0, t0);
  *dataOut++ = (uint16_t)(x0 & ones);
  *dataOut++ = (uint16_t)((uint32_t)*key++ + x2 & ones);
  *dataOut++ = (uint16_t)((uint32_t)*key++ + x1 & ones);
  t0 = (uint32_t)*key;
  Mul(x3, t0);
  *dataOut = (uint16_t)(x3 & ones);
} /* IDEACrypt */

/******************************************************************************/
/* Multiplicative Inverse by Extended Stein Greatest Common Divisor Algorithm.*/
/* pre:  0 <= x <= 0xFFFF.                                                    */
/* post: x * MulInv(x) == 1, where '*' is multiplication in the               */
/*                           multiplicative group.                            */

static uint16_t MulInv (uint16_t x)
{
  INT32S n1, n2, N, a1, a2, b1, b2;

  if (x <= 1) return x;
  n1 = N = (INT32S)x; n2 = mulMod;
  a1 = b2 = 1; a2 = b1 = 0;
  do {
	while ((n1 & 1) == 0) {
	  if (a1 & 1)
		if (a1 < 0) { a1 += mulMod; b1 -= N; }
		else { a1 -= mulMod; b1 += N; }
	  n1 >>= 1; a1 >>= 1; b1 >>= 1;
	}
	if (n1 < n2)
	  do {
		n2 -= n1; a2 -= a1; b2 -= b1;
		if (n2 == 0) return (uint16_t)(a1 < 0 ? a1 + mulMod : a1);
		while ((n2 & 1) == 0) {
		  if (a2 & 1)
			if (a2 < 0) { a2 += mulMod; b2 -= N; }
			else { a2 -= mulMod; b2 += N; }
		  n2 >>= 1; a2 >>= 1; b2 >>= 1;
		}
	  } while (n1 <= n2);
	n1 -= n2; a1 -= a2; b1 -= b2;
  } while (n1);
  return (uint16_t)(a2 < 0 ? a2 + mulMod : a2);
} /* MulInv */

/******************************************************************************/
/* Additive Inverse.                                                          */
/* pre:  0 <= x <= 0xFFFF.                                                    */
/* post: x + AddInv(x) == 0, where '+' is addition in the additive group.     */

#define AddInv(x)  (-x & ones)

/******************************************************************************/
/* Inverts a decryption/encrytion key to a encrytion/decryption key.          */
/* pre:  'key'    contains the encryption/decryption key.                     */
/* post: 'invKey' contains the decryption/encryption key.                     */

void IDEAInvertKey (IDEAKey key, IDEAKey invKey)
{
  uint16_t t, *in, *out;
  uint16_t lo, hi, i;

  in = key; out = invKey;
  lo = 0; hi = 6 * IDEAnofRound;
  t = MulInv(in[lo]); out[lo++] = MulInv(in[hi]); out[hi++] = t;
  t = AddInv(in[lo]); out[lo++] = AddInv(in[hi]); out[hi++] = t;
  t = AddInv(in[lo]); out[lo++] = AddInv(in[hi]); out[hi++] = t;
  t = MulInv(in[lo]); out[lo++] = MulInv(in[hi]); out[hi] = t;
  for (i = (IDEAnofRound - 1) / 2 ; i != 0 ; i --) {
	t = in[lo]; out[lo++] = in[hi -= 5]; out[hi ++] = t;
	t = in[lo]; out[lo++] = in[hi]; out[hi] = t;
	t = MulInv(in[lo]); out[lo++] = MulInv(in[hi -= 5]); out[hi++] = t;
	t = AddInv(in[lo]); out[lo++] = AddInv(in[++hi]); out[hi--] = t;
	t = AddInv(in[lo]); out[lo++] = AddInv(in[hi]); out[hi++] = t;
	t = MulInv(in[lo]); out[lo++] = MulInv(in[++hi]); out[hi] = t;
  }
#if (IDEAnofRound % 2 == 0)
  t = in[lo]; out[lo++] = in[hi -= 5]; out[hi++] = t;
  t = in[lo]; out[lo++] = in[hi]; out[hi] = t;
  out[lo] = MulInv(in[lo]); lo++;
  t = AddInv(in[lo]); out[lo] = AddInv(in[lo + 1]); lo++; out[lo++] = t;
  out[lo] = MulInv(in[lo]);
#else
  out[lo] = in[lo]; lo++;
  out[lo] = in[lo];
#endif
} /* IDEAInvertKey */

/******************************************************************************/
/* Expands a user key of 128 bits to a full encryption key                    */
/* pre:  'userKey' contains the 128 bit user key                              */
/* post: 'key'     contains the encryption key                                */

void IDEAExpandUserKey (IDEAUserKey userKey, IDEAKey key)
{
  uint16_t i;

#if (IDEAkeyLen <= IDEAuserKeyLen)
  for (i = 0; i < IDEAkeyLen; i++) key[i] = userKey[i];
#else
  for (i = 0; i < IDEAuserKeyLen; i++) key[i] = userKey[i];
  for (i = IDEAuserKeyLen; i < IDEAkeyLen; i++)
	if ((i & 7) < 6)
	  key[i] = (key[i - 7] & 127) << 9 | key[i - 6] >> 7;
	else if ((i & 7) == 6)
	  key[i] = (key[i - 7] & 127) << 9 | key[i - 14] >> 7;
	else
	  key[i] = (key[i - 15] & 127) << 9 | key[i - 14] >> 7;
#endif
} /* IDEAExpandUserKey */

/******************************************************************************/

void IDEACopyKey(IDEAKey key,uint8_t *text)
{
	uint16_t i;
	for(i=0;i<8;i++) key[i]=text[2*i]*256+text[(2*i)+1];
}

void IDEACopyData(unsigned char *text,IDEAData data)
{
	uint16_t i;
	for(i=0;i<4;i++)
	{
		 text[2*i]=data[i]/256;
		 text[2*i+1]=data[i]%256;
	}
}

void IDEACopyChar(IDEAData data,uint8_t *text)
{
	uint16_t i;
	for(i=0;i<4;i++) data[i]=text[2*i]*256+text[(2*i)+1];
}

void IDEA(uint8_t Data[8],uint8_t Key[16])
{
	IDEAData DataIn,DataOut;
	IDEAUserKey User_Key;
	IDEAKey     IDEAkey;

	IDEACopyKey(User_Key,Key);
	IDEACopyChar(DataIn,Data);
	IDEAExpandUserKey(User_Key,IDEAkey);
	IDEACrypt(DataIn,DataOut,IDEAkey);
	IDEACopyData(Data,DataOut);
}

void IDEA_1(uint8_t Data[8],uint8_t Key[16])
{
	IDEAData DataIn,DataOut;
	IDEAUserKey User_Key;
	IDEAKey     IDEAkey,inv_key;

	IDEACopyKey(User_Key,Key);
	IDEACopyChar(DataIn,Data);
	IDEAExpandUserKey(User_Key,IDEAkey);
	IDEAInvertKey(IDEAkey,inv_key);
	IDEACrypt(DataIn,DataOut,inv_key);
	IDEACopyData(Data,DataOut);
}

/******************************************************************************/
/*   DES/3DES CIPHER                                                          */
/******************************************************************************/
void TEXT_IPrank(uint8_t TEXT[8])
{
    uint16_t  count;
    uint8_t   buffer[8];
    uint8_t   set;
    uint16_t      IP[64]={    58,50,42,34,26,18,10, 2,
                    60,52,44,36,28,20,12, 4,
                    62,54,46,38,30,22,14, 6,
                    64,56,48,40,32,24,16, 8,
                    57,49,41,33,25,17, 9, 1,
                    59,51,43,35,27,19,11, 3,
                    61,53,45,37,29,21,13, 5,
                    63,55,47,39,31,23,15, 7 };

    for(count=0;count<8;count++){
      buffer[count]=TEXT[count];
      TEXT[count]=0x00;
    }

    for(count=0;count<64;count++){
      if((buffer[(IP[count]-1)/8]&(0x80>>(IP[count]-1)%8))!=0x00)
        set=0x80;else   set=0x00;
        TEXT[count/8]=TEXT[count/8]|(set>>count%8);
    }
}

void    TEXT_MULchange( uint8_t Ln[4], uint8_t Rn[4], uint8_t Kn[6])
{
    uint16_t  count;
    uint8_t   buffer1[6];
    uint8_t   buffer2[4],buffer3[4];
    uint8_t   set;

    uint8_t   E[48]={ 32, 1, 2, 3, 4, 5,
                 4, 5, 6, 7, 8, 9,
                 8, 9,10,11,12,13,
                12,13,14,15,16,17,
                16,17,18,19,20,21,
                20,21,22,23,24,25,
                24,25,26,27,28,29,
                28,29,30,31,32, 1   };

    uint8_t   S[8][64]={
          14, 4,13, 1, 2,15,11, 8, 3,10, 6,12, 5, 9, 0, 7,
           0,15, 7, 4,14, 2,13, 1,10, 6,12,11, 9, 5, 3, 8,
           4, 1,14, 8,13, 6, 2,11,15,12, 9, 7, 3,10, 5, 0,
          15,12, 8, 2, 4, 9, 1, 7, 5,11, 3,14,10, 0, 6,13,

          15, 1, 8,14, 6,11, 3, 4, 9, 7, 2,13,12, 0, 5,10,
           3,13, 4, 7,15, 2, 8,14,12, 0, 1,10, 6, 9,11, 5,
           0,14, 7,11,10, 4,13, 1, 5, 8,12, 6, 9, 3, 2,15,
          13, 8,10, 1, 3,15, 4, 2,11, 6, 7,12, 0, 5,14, 9,

          10, 0, 9,14, 6, 3,15, 5, 1,13,12, 7,11, 4, 2, 8,
          13, 7, 0, 9, 3, 4, 6,10, 2, 8, 5,14,12,11,15, 1,
          13, 6, 4, 9, 8,15, 3, 0,11, 1, 2,12, 5,10,14, 7,
           1,10,13, 0, 6, 9, 8, 7, 4,15,14, 3,11, 5, 2,12,

           7,13,14, 3, 0, 6, 9,10, 1, 2, 8, 5,11,12, 4,15,
          13, 8,11, 5, 6,15, 0, 3, 4, 7, 2,12, 1,10,14, 9,
          10, 6, 9, 0,12,11, 7,13,15, 1, 3,14, 5, 2, 8, 4,
           3,15, 0, 6,10, 1,13, 8, 9, 4, 5,11,12, 7, 2,14,

           2,12, 4, 1, 7,10,11, 6, 8, 5, 3,15,13, 0,14, 9,
          14,11, 2,12, 4, 7,13, 1, 5, 0,15,10, 3, 9, 8, 6,
           4, 2, 1,11,10,13, 7, 8,15, 9,12, 5, 6, 3, 0,14,
          11, 8,12, 7, 1,14, 2,13, 6,15, 0, 9,10, 4, 5, 3,

          12, 1,10,15, 9, 2, 6, 8, 0,13, 3, 4,14, 7, 5,11,
          10,15, 4, 2, 7,12, 9, 5, 6, 1,13,14, 0,11, 3, 8,
           9,14,15, 5, 2, 8,12, 3, 7, 0, 4,10, 1,13,11, 6,
           4, 3, 2,12, 9, 5,15,10,11,14, 1, 7, 6, 0, 8,13,

           4,11, 2,14,15, 0, 8,13, 3,12, 9, 7, 5,10, 6, 1,
          13, 0,11, 7, 4, 9, 1,10,14, 3, 5,12, 2,15, 8, 6,
           1, 4,11,13,12, 3, 7,14,10,15, 6, 8, 0, 5, 9, 2,
           6,11,13, 8, 1, 4,10, 7, 9, 5, 0,15,14, 2, 3,12,

          13, 2, 8, 4, 6,15,11, 1,10, 9, 3,14, 5, 0,12, 7,
           1,15,13, 8,10, 3, 7, 4,12, 5, 6,11, 0,14, 9, 2,
           7,11, 4, 1, 9,12,14, 2, 0, 6,10,13,15, 3, 5, 8,
           2, 1,14, 7, 4,10, 8,13,15,12, 9, 0, 3, 5, 6,11,  };

    uint8_t   P[32]={ 16, 7,20,21,
                29,12,28,17,
                 1,15,23,26,
                 5,18,31,10,
                 2, 8,24,14,
                32,27, 3, 9,
                19,13,30, 6,
                22,11, 4,25 };

    for(count=0;count<6;count++){
       buffer1[count]=0x00;
    }

    for(count=0;count<4;count++){
      buffer2[count]=0x00;
      buffer3[count]=0x00;
    }

//***************************************************************************
//Expand:
//***************************************************************************
    for(count=0;count<48;count++){
      if((Rn[(E[count]-1)/8]&(0x80>>(E[count]-1)%8))!=0x00)
        set=0x80;else set=0x00;
      buffer1[count/8]=buffer1[count/8]|(set>>count%8);
    }

    for(count=0;count<6;count++){
      buffer1[count]=buffer1[count]^Kn[count];
    }

//***************************************************************************
//Compressed Encoding
//***************************************************************************
    for(count=0;count<48;count++){
      if(count%6==0)set=0x00;
      if((buffer1[count/8]&(0x80>>count%8))!=0x00)
        set|=0x01;
      else 
        set|=0x00;
      if((count+1)%6==0){
        buffer2[count/12]<<=4;
        buffer2[count/12]|=S[count/6][(set&0x20)+((set&0x01)<<4)+((set&0x1e)>>1)];
      }
      set<<=1;
    }

//***************************************************************************
//Permutation
//***************************************************************************
    for(count=0;count<32;count++){
    if((buffer2[(P[count]-1)/8]&(0x80>>(P[count]-1)%8))!=0x00)
      set=0x80;else set=0x00;
      buffer3[count/8]=buffer3[count/8]|(set>>count%8);
    }

    for(count=0;count<4;count++){
      set=Rn[count];
      Rn[count]=buffer3[count]^Ln[count];
      Ln[count]=set;
    }
}

void TEXT_IPworsen(uint8_t TEXT[8])
{
    uint16_t      count;
    uint8_t   buffer[8];
    uint8_t   set;
    uint16_t      IP_1[64]={  40, 8,48,16,56,24,64,32,
                    39, 7,47,15,55,23,63,31,
                    38, 6,46,14,54,22,62,30,
                    37, 5,45,13,53,21,61,29,
                    36, 4,44,12,52,20,60,28,
                    35, 3,43,11,51,19,59,27,
                    34, 2,42,10,50,18,58,26,
                    33, 1,41, 9,49,17,57,25 };

    for(count=0;count<8;count++){
      buffer[count]=TEXT[count];
      TEXT[count]=0x00;
    }

    for(count=0;count<64;count++){
      if((buffer[(IP_1[count]-1)/8]&(0x80>>(IP_1[count]-1)%8))!=0x00)
        set=0x80;else   set=0x00;
      TEXT[count/8]=TEXT[count/8]|(set>>count%8);
    }
}

void KEY_PC1(uint8_t CnDn[7], uint8_t KEY[8])
{
    uint16_t  count;
    uint8_t   keybuf[8],buffer[7];
    uint8_t   set;
    uint8_t   PC1[56]={   57,49,41,33,25,17, 9,
                     1,58,50,42,34,26,18,
                    10, 2,59,51,43,35,27,
                    19,11, 3,60,52,44,36,
                    63,55,47,39,31,23,15,
                     7,62,54,46,38,30,22,
                    14, 6,61,53,45,37,29,
                    21,13, 5,28,20,12, 4    
                    };

    for(count=0;count<7;count++){
      CnDn[count]=0x00;
    }
    for(count=0;count<8;count++){
      keybuf[count]=KEY[count];
    }

    set=0x80;
    for(count=0;count<7;count++){
      keybuf[count]&=0xfe;
      buffer[count]=(keybuf[count]<<count)|((keybuf[count+1]&set)>>(7-count));
      set=(set>>1)|0x80;
    }

    for(count=0; count<56; count++){
      PC1[count]=(PC1[count]-PC1[count]/8)-1;
      if((buffer[PC1[count]/8]&(0x80>>PC1[count]%8))!=0x00)
        set=0x80;else set=0x00;
      CnDn[count/8]=CnDn[count/8]|(set>>count%8);
    }
}

void KEY_LS(uint8_t CnDn[7])
{
    uint16_t  count;
    uint8_t   set;

    set=CnDn[0]&0x80;
    for(count=0;count<6;count++){
      CnDn[count]=(CnDn[count]<<1)|((CnDn[count+1]&0x80)>>7);
    }
    CnDn[6]<<=1;

    if((CnDn[3]&0x10)!=0x00)
      CnDn[6]|=0x01;
    CnDn[3]=CnDn[3]&0xef|(set>>3);
}

void KEY_PC2(uint8_t CnDn[7], uint8_t SKEYn[6])
{
    uint16_t  count;
    uint8_t   set;
    uint8_t   PC2[48]={   14,17,11,24, 1, 5,
                     3,28,15, 6,21,10,
                    23,19,12, 4,26, 8,
                    16, 7,27,20,13, 2,
                    41,52,31,37,47,55,
                    30,40,51,45,33,48,
                    44,49,39,56,34,53,
                    46,42,50,36,29,32   
                    };

    for(count=0;  count<6;  count++){
      SKEYn[count]  = 0x00;
    }

    for(count=0; count<48; count++){
      if((CnDn[(PC2[count]-1)/8] & (0x80>>(PC2[count]-1)%8)) != 0x00)
        set=0x80;
      else set=0x00;
      SKEYn[count/8]=SKEYn[count/8]|(set>>count%8);
    }
}

void SonKey(uint8_t SKEY[16][6], uint8_t KEY[8])
{
    uint16_t  count;
    uint8_t   CnDn[7];
    uint8_t   LS[16]  ={1, 1, 2, 2, 2, 2, 2, 2,
                     1, 2, 2, 2, 2, 2, 2, 1 };

    KEY_PC1(CnDn, KEY);
    for(count=0; count<16; count++){
      KEY_LS(CnDn);
      if(LS[count] == 2) 
        KEY_LS(CnDn);
        KEY_PC2(CnDn, SKEY[count]);
    }
}

void DES(uint8_t TEXT[8], uint8_t KEY[8])
{
    uint16_t  count;
    uint8_t   Ln[4], Rn[4];
    uint8_t   SKEY[16][6];

//***************************************************************************
//DES deal
//***************************************************************************
    SonKey(SKEY, KEY);

    TEXT_IPrank(TEXT);

    Ln[0] = TEXT[0];  Rn[0] = TEXT[4];
    Ln[1] = TEXT[1];  Rn[1] = TEXT[5];
    Ln[2] = TEXT[2];  Rn[2] = TEXT[6];
    Ln[3] = TEXT[3];  Rn[3] = TEXT[7];


    for(count=0; count<16; count++){
      TEXT_MULchange(Ln, Rn, SKEY[count]);
    }

    TEXT[0] = Rn[0];  TEXT[4] = Ln[0];
    TEXT[1] = Rn[1];  TEXT[5] = Ln[1];
    TEXT[2] = Rn[2];  TEXT[6] = Ln[2];
    TEXT[3] = Rn[3];  TEXT[7] = Ln[3];

    TEXT_IPworsen(TEXT);
}

void DES_1(uint8_t TEXT[8], uint8_t KEY[8])
{
    uint16_t  count;
    uint8_t   Ln[4], Rn[4];
    uint8_t   SKEY[16][6];

//***************************************************************************
//Get son key by key
//***************************************************************************
    SonKey(SKEY, KEY);

//***************************************************************************
//DES_1 deal
//***************************************************************************
    TEXT_IPrank(TEXT);

    Ln[0] = TEXT[0];  Rn[0] = TEXT[4];
    Ln[1] = TEXT[1];  Rn[1] = TEXT[5];
    Ln[2] = TEXT[2];  Rn[2] = TEXT[6];
    Ln[3] = TEXT[3];  Rn[3] = TEXT[7];


    for(count = 0; count < 16; count++){
      TEXT_MULchange(Ln, Rn, SKEY[15 - count]);
    }

    TEXT[0] = Rn[0];  TEXT[4] = Ln[0];
    TEXT[1] = Rn[1];  TEXT[5] = Ln[1];
    TEXT[2] = Rn[2];  TEXT[6] = Ln[2];
    TEXT[3] = Rn[3];  TEXT[7] = Ln[3];

    TEXT_IPworsen(TEXT);

}

void  DES3(uint8_t *TEXT, uint8_t *KEY)
{
    uint8_t key[8], key1[8];
    memcpy(key, KEY, 8);
    memcpy(key1, KEY+8, 8);
    DES(TEXT, key);
    DES_1(TEXT, key1);
    DES(TEXT, key);
}

void  DES3_1(uint8_t *TEXT,uint8_t *KEY)
{
    uint8_t key[8], key1[8];
    memcpy(key, KEY, 8);
    memcpy(key1, KEY+8, 8);
    DES_1(TEXT, key);
    DES(TEXT, key1);
    DES_1(TEXT, key);
}

/**************************************************
通用加密函数
****************************************************/
void Encode(uint8_t Text[8], uint8_t Key[16])
{
#ifdef	IDEA_CRYPT
	IDEA(Text,Key);
#else
	DES3(Text,Key);
#endif
}
/**************************************************
通用解密函数
****************************************************/
void Decode(uint8_t Text[8], uint8_t Key[16])
{
#ifdef	IDEA_CRYPT
	IDEA_1(Text, Key);
#else
	DES_1(Text, Key);
#endif
}



////身份证阅读器加密算法
//void IDReaderEncode(void* data, int size,  unsigned char key[4])
//{
//	uint8_t uaKey[16],i;
//
//	memcpy(uaKey,"\x00\x00\x00\x00",4);
//
//	memcpy(uaKey+4,key,4);
//	for(i=0;i<size/8;i++)
//		Encode((uint8_t*)data+i*8,uaKey);
//}


//身份证阅读器解密算法
void IDReaderDecode(void* data, int size,  unsigned char key[4])
{
	uint8_t uaKey[16], TEXT[8], i;

	memset(uaKey, 0x00, sizeof(uaKey));

	memcpy(uaKey + 4, key, 4);
	for(i = 0; i < size/8; i++)
	    DES3_1((uint8_t*)data+8*i, uaKey);
}