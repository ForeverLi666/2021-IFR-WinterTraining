#include "Remote.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart1;
extern uint16_t Motor_Num;
RC_Ctl_t RC_CtrlData=
{
	{1024,1024,1024,1024,2,2},
	{0},
	{0},
	1024,
	0,
};

void RemoteDataProcess(uint8_t *pData) 
{     	
	if(pData == 0)     
	{         
		return;     
	}          
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;      
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;    
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;     
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;          
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;     
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003); 
 
  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);     
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);     
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);     
 
  RC_CtrlData.mouse.press_l = pData[12];     
	RC_CtrlData.mouse.press_r = pData[13];       
	RC_CtrlData.key.vh = ((int16_t)pData[14]);
	RC_CtrlData.key.vl = ((int16_t)pData[15]);      
	RC_CtrlData.SW = (uint16_t)((pData[17]<<8)|pData[16]);
	
	RC_CtrlData.update = 1;	
}

void My_Motor_Tar(Speed_System *Speed,Pos_System *Pos)
{
	switch (RC_CtrlData.rc.s1)
	{
		case 1:
			switch (Motor_Num)
			{
				case 0x201:
					Speed->Tar_Speed=(RC_CtrlData.rc.ch0-1024)*5-(RC_CtrlData.rc.ch1-1024)*5;
					break;
				case 0x202:
					Speed->Tar_Speed=(RC_CtrlData.rc.ch0-1024)*5+(RC_CtrlData.rc.ch1-1024)*5;
					break;
				case 0x203:
					Speed->Tar_Speed=(RC_CtrlData.rc.ch0-1024)*5-(RC_CtrlData.rc.ch1-1024)*5;
					break;
				case 0x204:
					Speed->Tar_Speed=(RC_CtrlData.rc.ch0-1024)*5+(RC_CtrlData.rc.ch1-1024)*5;
					break;
				default:
					break;
			}
			break;
		case 2:
			Pos->Tar_Pos=(RC_CtrlData.rc.ch1-1024)*10;
			break;
		default:
			break;
	}
}
void My_Motor_control(Speed_System *Speed,Pos_System *Pos,uint8_t *RxData,uint8_t *TxData)
{
	switch (RC_CtrlData.rc.s1)
	{
		case 1:
			My_Motor_Tar(Speed,Pos);
			My_Speed_Info_Analysis(Speed,RxData);
			PID_Speed_Cal(Speed,TxData);
			Send_To_Motor(&hcan1,TxData);
			break;
		case 2:
			My_Motor_Tar(Speed,Pos);
			My_Pos_Info_Analysis(Pos,RxData);
			PID_Pos_Cal(Pos,TxData);
			Send_To_Motor(&hcan1,TxData);
			break;
		default:
			break;
	}
}
