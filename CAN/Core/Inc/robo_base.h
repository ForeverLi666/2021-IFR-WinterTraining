#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------头文件包含部分----------//
#include "main.h"
//---------------------------------//

//---------底盘结构体部分----------//
typedef struct Motor
{
  uint16_t Angle;
	int16_t Speed;
	int16_t Current;
	uint8_t Temperature;
	uint16_t Last_Angle;
	int32_t Abs_Angle;
}Motor;
typedef struct pid_init_val{		//电机PID参数结构体
	
	float Kp;
	float Ki;
	float Kd;
	
	float error;					//误差
	float error_last;				//上一次误差
	float error_max;				//最大误差
	float dead_line;				//死区
	
	float intergral;				//误差积分
	float intergral_max;			//误差积分最大值
	
	float derivative;				//误差微分
	
	float output;					//输出
	float output_max;				//输出最大值
	
}PID;
typedef struct Speed_System			//速度环系统
{
  Motor Info;			//速度环电机信息
  PID Speed_PID;					//速度环PID参数
  float Tar_Speed;					//目标速度
  uint8_t Motor_Num;				//电机号码
}Speed_System;

typedef struct Robo_Base			//底盘结构体
{
	Speed_System Speed_MotorLF;		//速度环--左前轮
	Speed_System Speed_MotorLB;		//速度环--左后轮
	Speed_System Speed_MotorRF;		//速度环--右前轮
	Speed_System Speed_MotorRB;		//速度环--右后轮
}ROBO_BASE;
//---------------------------------//

//-------------函数声明------------//
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);							
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg);																
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);							
void My_Speed_Info_Analysis(Speed_System* Speed,uint8_t *RxData);
void My_Base_Init(ROBO_BASE *Base);
void My_Info_Receive(ROBO_BASE *Base,uint8_t *RxData,uint16_t Motor_Num);
void My_Motor_Interface(ROBO_BASE *Base);
void My_Motor_Tar(Speed_System *Speed);
void My_Speed_Limit(float Tar_Speed);
void My_Motor_Control(Speed_System *Speed,uint8_t *TxData);
//---------------------------------//
#endif


