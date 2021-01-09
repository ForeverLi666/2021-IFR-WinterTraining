
//---------头文件引用部分---------//
#include "Remote.h"
#include "can.h"
//--------------------------------//

//---------变量声明部分-----------//
uint8_t TxData[8];
CAN_RxHeaderTypeDef RxMessage;
uint16_t Motor_Num;
//--------------------------------//
//---------外部变量声明部分-------//
extern CAN_HandleTypeDef hcan1;
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
void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg)       
{
	Pos_Motor->Pos_PID.error =  Pos_Motor->Tar_Pos - Pos_Motor->Info.Abs_Angle;
	if(Pos_Motor->Pos_PID.error > Pos_Motor->Pos_PID.error_max)
		Pos_Motor->Pos_PID.error = Pos_Motor->Pos_PID.error_max;
	if(Pos_Motor->Pos_PID.error < -Pos_Motor->Pos_PID.error_max)
		Pos_Motor->Pos_PID.error = -Pos_Motor->Pos_PID.error_max;
	if(Pos_Motor->Pos_PID.error > 0 && Pos_Motor->Pos_PID.error < Pos_Motor->Pos_PID.dead_line)
		Pos_Motor->Pos_PID.error = 0;
	if(Pos_Motor->Pos_PID.error < 0 && Pos_Motor->Pos_PID.error > Pos_Motor->Pos_PID.dead_line)
		Pos_Motor->Pos_PID.error = 0;
	
	Pos_Motor->Pos_PID.intergral = Pos_Motor->Pos_PID.intergral + Pos_Motor->Pos_PID.error;
	if(Pos_Motor->Pos_PID.intergral > Pos_Motor->Pos_PID.intergral_max)
		Pos_Motor->Pos_PID.intergral = Pos_Motor->Pos_PID.intergral_max;
	if(Pos_Motor->Pos_PID.intergral < -Pos_Motor->Pos_PID.intergral_max)
		Pos_Motor->Pos_PID.intergral = -Pos_Motor->Pos_PID.intergral_max;
	
	Pos_Motor->Pos_PID.derivative = Pos_Motor->Pos_PID.error - Pos_Motor->Pos_PID.error_last;
	Pos_Motor->Pos_PID.error_last = Pos_Motor->Pos_PID.error;
	
	Pos_Motor->Pos_PID.output = Pos_Motor->Pos_PID.Kp*Pos_Motor->Pos_PID.error + Pos_Motor->Pos_PID.Ki*Pos_Motor->Pos_PID.intergral + Pos_Motor->Pos_PID.Kd*Pos_Motor->Pos_PID.derivative;
	
	if(Pos_Motor->Pos_PID.output > Pos_Motor->Pos_PID.output_max)
		Pos_Motor->Pos_PID.output = Pos_Motor->Pos_PID.output_max;
	if(Pos_Motor->Pos_PID.output < -Pos_Motor->Pos_PID.output_max)
		Pos_Motor->Pos_PID.output = -Pos_Motor->Pos_PID.output_max;
	
		Pos_Motor->Speed_PID.error =  Pos_Motor->Pos_PID.output - Pos_Motor->Info.Speed;
	if(Pos_Motor->Speed_PID.error > Pos_Motor->Speed_PID.error_max)
		Pos_Motor->Speed_PID.error = Pos_Motor->Speed_PID.error_max;
	if(Pos_Motor->Speed_PID.error < -Pos_Motor->Speed_PID.error_max)
		Pos_Motor->Speed_PID.error = -Pos_Motor->Speed_PID.error_max;
	if(Pos_Motor->Speed_PID.error > 0 && Pos_Motor->Speed_PID.error < Pos_Motor->Speed_PID.dead_line)
		Pos_Motor->Speed_PID.error = 0;
	if(Pos_Motor->Speed_PID.error < 0 && Pos_Motor->Speed_PID.error > Pos_Motor->Speed_PID.dead_line)
		Pos_Motor->Speed_PID.error = 0;
	
	Pos_Motor->Speed_PID.intergral = Pos_Motor->Speed_PID.intergral + Pos_Motor->Speed_PID.error;
	if(Pos_Motor->Speed_PID.intergral > Pos_Motor->Speed_PID.intergral_max)
		Pos_Motor->Speed_PID.intergral = Pos_Motor->Speed_PID.intergral_max;
	if(Pos_Motor->Speed_PID.intergral < -Pos_Motor->Speed_PID.intergral_max)
		Pos_Motor->Speed_PID.intergral = -Pos_Motor->Speed_PID.intergral_max;
	
	Pos_Motor->Speed_PID.derivative = Pos_Motor->Speed_PID.error - Pos_Motor->Speed_PID.error_last;
	Pos_Motor->Speed_PID.error_last = Pos_Motor->Speed_PID.error;
	
	Pos_Motor->Speed_PID.output = Pos_Motor->Speed_PID.Kp*Pos_Motor->Speed_PID.error + Pos_Motor->Speed_PID.Ki*Pos_Motor->Speed_PID.intergral + Pos_Motor->Speed_PID.Kd*Pos_Motor->Speed_PID.derivative;
	
	if(Pos_Motor->Speed_PID.output > Pos_Motor->Speed_PID.output_max)
		Pos_Motor->Speed_PID.output = Pos_Motor->Speed_PID.output_max;
	if(Pos_Motor->Speed_PID.output < -Pos_Motor->Speed_PID.output_max)
		Pos_Motor->Speed_PID.output = -Pos_Motor->Speed_PID.output_max;

	
	Tx_msg[Pos_Motor->Motor_Num*2]=((int16_t)Pos_Motor->Speed_PID.output)>>8;Tx_msg[Pos_Motor->Motor_Num*2+1]=(int16_t)Pos_Motor->Speed_PID.output;
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
void My_Pos_Info_Analysis(Pos_System *Pos,uint8_t *RxData)
{
	int16_t Error;
	Pos->Info.Angle=RxData[0];Pos->Info.Angle<<=8;Pos->Info.Angle|=RxData[1];
	Pos->Info.Speed=RxData[2];Pos->Info.Speed<<=8;Pos->Info.Speed|=RxData[3];
	Pos->Info.Current=RxData[4];Pos->Info.Current<<=8;Pos->Info.Current|=RxData[5];
	Pos->Info.Temperature=RxData[6];
	if(Pos->Info.Speed!=0)
	{
		Error=Pos->Info.Angle-Pos->Info.Last_Angle;
		Pos->Info.Abs_Angle+=Error;
		if (Error < -4096)Pos->Info.Abs_Angle += 8192;
    else if (Error > 4096)Pos->Info.Abs_Angle -= 8192;
	}
		Pos->Info.Last_Angle=Pos->Info.Angle;
}
void My_Info_Receive(ROBO_BASE *Base,uint8_t *RxData)
{
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,RxData);
	Motor_Num=RxMessage.StdId;
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
void My_Motor_Interface(ROBO_BASE *Base)
{
	My_Motor_Control(&Base->Speed_MotorLF,TxData);
	My_Motor_Control(&Base->Speed_MotorRF,TxData);
	My_Motor_Control(&Base->Speed_MotorRB,TxData);
	My_Motor_Control(&Base->Speed_MotorLB,TxData);
}

