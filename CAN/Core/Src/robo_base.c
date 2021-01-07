
//---------头文件引用部分---------//
#include "Remote.h"
#include "can.h"
//--------------------------------//

//---------变量声明部分-----------//
ROBO_BASE Robo_Base;
//--------------------------------//
//---------外部变量声明部分-------//
extern CAN_HandleTypeDef hcan1;
extern uint8_t RxData[8];
extern uint8_t TxData[8];
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

void PID_General_Cal(PID *pid, float fdbV, float tarV,uint8_t moto_num,uint8_t *Tx_msg)
{

	pid->error =  tarV - fdbV;
	if(pid->error > pid->error_max)
		pid->error = pid->error_max;
	if(pid->error < -pid->error_max)
		pid->error = -pid->error_max;
	if(pid->error > 0 && pid->error < pid->dead_line)
		pid->error = 0;
	if(pid->error < 0 && pid->error > pid->dead_line)
		pid->error = 0;
	
	pid->intergral = pid->intergral + pid->error;
	if(pid->intergral > pid->intergral_max)
		pid->intergral = pid->intergral_max;
	if(pid->intergral < -pid->intergral_max)
		pid->intergral = -pid->intergral_max;
	
	pid->derivative = pid->error - pid->error_last;
	pid->error_last = pid->error;
	
	pid->output = pid->Kp*pid->error + pid->Ki*pid->intergral + pid->Kd*pid->derivative;
	
	if(pid->output > pid->output_max)
		pid->output = pid->output_max;
	if(pid->output < -pid->output_max)
		pid->output = -pid->output_max;
	
	Tx_msg[moto_num*2]=((int16_t)pid->output)>>8;Tx_msg[moto_num*2+1]=(int16_t)pid->output;
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
	if(Speed_Motor->Speed_PID.error < 0 && Speed_Motor->Speed_PID.error > Speed_Motor->Speed_PID.dead_line)
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
void My_Base_Init(ROBO_BASE *Base)
{
	Base->Pos_MotorLF.Motor_Num=0;
	PID_Init(&Base->Pos_MotorLF.Pos_PID,0.5,0,0,5000,0,5000,5000);
	PID_Init(&Base->Pos_MotorLF.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Pos_MotorRF.Motor_Num=1;
	PID_Init(&Base->Pos_MotorRF.Pos_PID,0.5,0,0,5000,0,5000,5000);
	PID_Init(&Base->Pos_MotorRF.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Pos_MotorRB.Motor_Num=2;
	PID_Init(&Base->Pos_MotorRB.Pos_PID,0.5,0,0,5000,0,5000,5000);
	PID_Init(&Base->Pos_MotorRB.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Pos_MotorLB.Motor_Num=3;
	PID_Init(&Base->Pos_MotorLB.Pos_PID,0.5,0,0,5000,0,5000,5000);
	PID_Init(&Base->Pos_MotorLB.Speed_PID,5,0,0,5000,0,5000,5000);
	
	
	Base->Speed_MotorLF.Motor_Num=0;
	PID_Init(&Base->Speed_MotorLF.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Speed_MotorRF.Motor_Num=1;
	PID_Init(&Base->Speed_MotorRF.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Speed_MotorRB.Motor_Num=2;
	PID_Init(&Base->Speed_MotorRB.Speed_PID,5,0,0,5000,0,5000,5000);
	Base->Speed_MotorLB.Motor_Num=3;
	PID_Init(&Base->Speed_MotorLB.Speed_PID,5,0,0,5000,0,5000,5000);
	
}
void My_Motor_Analysis(ROBO_BASE *Base,uint16_t Motor_Num)
{
	switch (Motor_Num)
	{
		case 0x201:
			My_Motor_control(&Base->Speed_MotorLF,&Base->Pos_MotorLF,RxData,TxData);
			break;
		case 0x202:
			My_Motor_control(&Base->Speed_MotorRF,&Base->Pos_MotorRF,RxData,TxData);
			break;
		case 0x203:
			My_Motor_control(&Base->Speed_MotorRB,&Base->Pos_MotorRB,RxData,TxData);
			break;
		case 0x204:
			My_Motor_control(&Base->Speed_MotorLB,&Base->Pos_MotorLB,RxData,TxData);
			break;
		default :
			break;
	}
}

