#ifndef __LORA_CRC_H__
#define __LORA_CRC_H__

typedef char 			BOOL;
typedef char 			CHAR;
typedef unsigned char 	UCHAR;
typedef unsigned int 	USHORT;


USHORT usLORACRC16( UCHAR * pucFrame, USHORT usLen );


#endif