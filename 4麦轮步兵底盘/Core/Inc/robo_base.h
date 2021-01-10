#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------ͷ�ļ���������----------//
#include "main.h"
//---------------------------------//

//---------���̽ṹ�岿��----------//
typedef struct Motor
{
  uint16_t Angle;
	int16_t Speed;
	int16_t Current;
	uint8_t Temperature;
	uint16_t Last_Angle;
	int32_t Abs_Angle;
}Motor;
typedef struct pid_init_val{		//���PID�����ṹ��
	
	float Kp;
	float Ki;
	float Kd;
	
	float error;					//���
	float error_last;				//��һ�����
	float error_max;				//������
	float dead_line;				//����
	
	float intergral;				//������
	float intergral_max;			//���������ֵ
	
	float derivative;				//���΢��
	
	float output;					//���
	float output_max;				//������ֵ
	
}PID;
typedef struct Speed_System			//�ٶȻ�ϵͳ
{
  Motor Info;			//�ٶȻ������Ϣ
  PID Speed_PID;					//�ٶȻ�PID����
  float Tar_Speed;					//Ŀ���ٶ�
  uint8_t Motor_Num;				//�������
}Speed_System;

typedef struct Robo_Base			//���̽ṹ��
{
	Speed_System Speed_MotorLF;		//�ٶȻ�--��ǰ��
	Speed_System Speed_MotorLB;		//�ٶȻ�--�����
	Speed_System Speed_MotorRF;		//�ٶȻ�--��ǰ��
	Speed_System Speed_MotorRB;		//�ٶȻ�--�Һ���
}ROBO_BASE;
//---------------------------------//

//-------------��������------------//
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


