
//---------头文件引用部分---------//
#include "robo_base.h"
#include "Remote.h"
//--------------------------------//

//---------变量声明部分-----------//
uint8_t TxData[8];
//--------------------------------//
//---------外部变量声明部分-------//
extern CAN_HandleTypeDef hcan1;
extern RC_Ctl_t RC_CtrlData;
//--------------------------------//

/**********************************************************电机pid控制系统****************************************************************************************************/

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max)         
{
	pid->Kp = Kp;                      
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->error_max = error_max;      
	pid->output_max = output_max;        
	pid->dead_line = dead_line;         
	
	pid->intergral_max = intergral_max;         
	
	pid->error = 0;                 
	pid->error_last = 0;              
	pid->intergral = 0;               
	pid->derivative = 0;            
	pid->output = 0;                  
}
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg)
{

	Speed_Motor->Speed_PID.error =  Speed_Motor->Tar_Speed - Speed_Motor->Info.Speed;
	if(Speed_Motor->Speed_PID.error > Speed_Motor->Speed_PID.error_max)
		Speed_Motor->Speed_PID.error = Speed_Motor->Speed_PID.error_max;
	if(Speed_Motor->Speed_PID.error < -Speed_Motor->Speed_PID.error_max)
		Speed_Motor->Speed_PID.error = -Speed_Motor->Speed_PID.error_max;
	if(Speed_Motor->Speed_PID.error > 0 && Speed_Motor->Speed_PID.error < Speed_Motor->Speed_PID.dead_line)
		Speed_Motor->Speed_PID.error = 0;
	if(Speed_Motor->Speed_PID.error < 0 && -Speed_Motor->Speed_PID.error < Speed_Motor->Speed_PID.dead_line)
		Speed_Motor->Speed_PID.error = 0;
	
	Speed_Motor->Speed_PID.intergral = Speed_Motor->Speed_PID.intergral + Speed_Motor->Speed_PID.error;
	if(Speed_Motor->Speed_PID.intergral > Speed_Motor->Speed_PID.intergral_max)
		Speed_Motor->Speed_PID.intergral = Speed_Motor->Speed_PID.intergral_max;
	if(Speed_Motor->Speed_PID.intergral < -Speed_Motor->Speed_PID.intergral_max)
		Speed_Motor->Speed_PID.intergral = -Speed_Motor->Speed_PID.intergral_max;
	
	Speed_Motor->Speed_PID.derivative = Speed_Motor->Speed_PID.error - Speed_Motor->Speed_PID.error_last;
	Speed_Motor->Speed_PID.error_last = Speed_Motor->Speed_PID.error;
	
	Speed_Motor->Speed_PID.output = Speed_Motor->Speed_PID.Kp*Speed_Motor->Speed_PID.error + Speed_Motor->Speed_PID.Ki*Speed_Motor->Speed_PID.intergral + Speed_Motor->Speed_PID.Kd*Speed_Motor->Speed_PID.derivative;
	
	if(Speed_Motor->Speed_PID.output > Speed_Motor->Speed_PID.output_max)
		Speed_Motor->Speed_PID.output = Speed_Motor->Speed_PID.output_max;
	if(Speed_Motor->Speed_PID.output < -Speed_Motor->Speed_PID.output_max)
		Speed_Motor->Speed_PID.output = -Speed_Motor->Speed_PID.output_max;
	
	Tx_msg[Speed_Motor->Motor_Num*2] = ((int16_t)Speed_Motor->Speed_PID.output)>>8;
	Tx_msg[Speed_Motor->Motor_Num*2+1] = (int16_t)Speed_Motor->Speed_PID.output;
}

void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox; 

  TxHeader.RTR = 0;
  TxHeader.IDE = 0;            
  TxHeader.StdId=0x200;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
        
  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, Tx_Data, &TxMailbox) != HAL_OK)
  {
     Error_Handler();
  }
}
void My_Speed_Info_Analysis(Speed_System *Speed,uint8_t *RxData)
{

	Speed->Info.Angle=RxData[0];Speed->Info.Angle<<=8;Speed->Info.Angle|=RxData[1];
	Speed->Info.Speed=RxData[2];Speed->Info.Speed<<=8;Speed->Info.Speed|=RxData[3];
	Speed->Info.Current=RxData[4];Speed->Info.Current<<=8;Speed->Info.Current|=RxData[5];
	Speed->Info.Temperature=RxData[6];
}
void My_Info_Receive(ROBO_BASE *Base,uint8_t *RxData,uint16_t Motor_Num)
{
	switch (Motor_Num) 
	{
		case 0x201:
			My_Speed_Info_Analysis(&Base->Speed_MotorLF,RxData);
			break;
		case 0x202:
			My_Speed_Info_Analysis(&Base->Speed_MotorRF,RxData);
			break;
		case 0x203:
			My_Speed_Info_Analysis(&Base->Speed_MotorRB,RxData);
			break;
		case 0x204:
			My_Speed_Info_Analysis(&Base->Speed_MotorLB,RxData);
			break;
		default :
			break;
	}
}
void My_Base_Init(ROBO_BASE *Base)
{
	Base->Speed_MotorLF.Motor_Num=0;
	PID_Init(&Base->Speed_MotorLF.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Speed_MotorRF.Motor_Num=1;
	PID_Init(&Base->Speed_MotorRF.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Speed_MotorRB.Motor_Num=2;
	PID_Init(&Base->Speed_MotorRB.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Speed_MotorLB.Motor_Num=3;
	PID_Init(&Base->Speed_MotorLB.Speed_PID,5,0,0,5000,0,5000,5000);
	
}
void My_Speed_Limit(float Tar_Speed)
{
	if(Tar_Speed>5000)
	{
		Tar_Speed=5000;
	}else if(Tar_Speed<-5000)
	{
		Tar_Speed=-5000;
	}
}
void My_Motor_Tar(Speed_System *Speed)
{
	switch (Speed->Motor_Num)
	{
		case 0:
			Speed->Tar_Speed=(RC_CtrlData.rc.ch1-1024)*5-(RC_CtrlData.rc.ch0-1024)*5;
			My_Speed_Limit(Speed->Tar_Speed);
			break;
		case 1:
			Speed->Tar_Speed=-(RC_CtrlData.rc.ch1-1024)*5-(RC_CtrlData.rc.ch0-1024)*5;
			My_Speed_Limit(Speed->Tar_Speed);
			break;
		case 2:
			Speed->Tar_Speed=-(RC_CtrlData.rc.ch1-1024)*5+(RC_CtrlData.rc.ch0-1024)*5;
			My_Speed_Limit(Speed->Tar_Speed);
			break;
		case 3:
			Speed->Tar_Speed=(RC_CtrlData.rc.ch1-1024)*5+(RC_CtrlData.rc.ch0-1024)*5;
			My_Speed_Limit(Speed->Tar_Speed);
			break;
	}
}
void My_Motor_Control(Speed_System *Speed,uint8_t *TxData)
{
		My_Motor_Tar(Speed);
		PID_Speed_Cal(Speed,TxData);
		Send_To_Motor(&hcan1,TxData);
}
void My_Motor_Interface(ROBO_BASE *Base)
{
	My_Motor_Control(&Base->Speed_MotorLF,TxData);
	My_Motor_Control(&Base->Speed_MotorRF,TxData);
	My_Motor_Control(&Base->Speed_MotorRB,TxData);
	My_Motor_Control(&Base->Speed_MotorLB,TxData);
}

