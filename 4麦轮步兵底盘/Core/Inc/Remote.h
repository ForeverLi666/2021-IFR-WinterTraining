#ifndef  __REMOTE_H__
#define __REMOTE_H__
#include "main.h"
/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
	struct 
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;
		
}RC_Ctl_t;
void RemoteDataProcess(uint8_t *pData);	
#endif

