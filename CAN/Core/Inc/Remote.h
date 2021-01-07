#ifndef  __REMOTE_H__
#define __REMOTE_H__
#include "main.h"
#include "robo_base.h"
#include "can.h"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)
#define RC_FRAME_LENGTH 18u
/*--------------------------------------------------------------------------*/
#define USART1_RX_LEN_MAX 18
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
		
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
		}mouse;
		
		struct
		{
			uint16_t vl;
			uint16_t vh;  
		}key;
		
		uint16_t SW;
		
		uint8_t update;
		
}RC_Ctl_t;

typedef struct
{
	uint8_t* Buffer[2];
	uint8_t Buffer_Num;
	uint16_t Length_Max;
}UART_RX_BUFFER;

void RemoteDataProcess(uint8_t *pData);
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData));
void Usart_Init(void);
void My_Motor_control(Speed_System *Speed,Pos_System *Pos,uint8_t *RxData,uint8_t *TxData);
	
#endif

