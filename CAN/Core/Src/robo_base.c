//�ļ�����:		robo_base.c
//��Ӧͷ�ļ�:	robo_base.h
//��Ҫ����:
//		���ڴ�C620�ĵ����M3508�����װ�����ĵ��̺�����.
//		�ܹ�ʵ�ֵ�����Ϣ�ĳ�ʼ��, ���������Ϣ�ķ���, ��PID����
//
//ʱ��:
//		2020/11/13
//
//�汾:	1.0V

//---------ͷ�ļ����ò���---------//
#include "Remote.h"
#include "can.h"
//--------------------------------//

//---------������������-----------//
ROBO_BASE Robo_Base;
CAN_HandleTypeDef hcan2;
extern uint8_t RxData[8];
extern uint8_t TxData[8];
//--------------------------------//

//---------�ⲿ������������-------//
//--------------------------------//

/**********************************************************���pid����ϵͳ****************************************************************************************************/
//--------------------------------------------------------------------------------------------------//
//1.��ʼ������
//
//��������:
//		���̲�����ʼ��
//
//��������:
//		��ʼ���������е���Ϣ
//
//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//
//��ֲ����:
//		��ʲô״̬, ���, ���״̬���Ȱ����ݷ�װ��ROBO_BASE�ṹ����, Ȼ��ֱ�ӳ�ʼ���ͺ���
//
//--------------------------------------------------------------------------------------------------//
void BASE_Init(ROBO_BASE *Robo)       
{
  Pos_System* P_Pos=NULL;           //λ�û���Ϣ��pid
  P_Pos=&Robo->Pos_MotorLF; PID_Init(&P_Pos->Pos_PID,			0.3,	0,	0,	5000,	0,	0,	7000);
  P_Pos->Motor_Num=0;		PID_Init(&P_Pos->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000); 
  P_Pos=&Robo->Pos_MotorRF; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=1;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos=&Robo->Pos_MotorRB; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=2;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos=&Robo->Pos_MotorLB; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=3;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 

  Speed_System* P_Speed=NULL;      //�ٶȻ���Ϣ��pid
  P_Speed=&Robo->Speed_MotorLF; PID_Init(&P_Speed->Speed_PID,	5,	0,	0,	5000,	0,	0,	7000); P_Pos->Motor_Num=0;
  P_Speed=&Robo->Speed_MotorRF; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Pos->Motor_Num=1;
  P_Speed=&Robo->Speed_MotorRB; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Pos->Motor_Num=2;
  P_Speed=&Robo->Speed_MotorLB; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Pos->Motor_Num=3;
}

//--------------------------------------------------------------------------------------------------//
//2.��ȡ��ǰλ�û�λ�����ٶ����ݣ�pos��ڣ�


//��������:
//		λ�û�������ݷ����Ľӿں���

//��������:
//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.

//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
//		uint32_t �������

//��ֲ����:
//		ֱ�Ӷ�case�����ݽ����޸�, �м���λ�û������ӾͼӼ���, Ȼ����ָ��ָ���Ӧ�����Ӿ���.

//--------------------------------------------------------------------------------------------------//
//void Motor_Pos_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num)
//{
//  Pos_System* P_Motor=NULL;
//  switch(Motor_Num)
//  {
//    case 0x201:P_Motor=&Robo->Pos_MotorLF;break;
//    case 0x202:P_Motor=&Robo->Pos_MotorRF;break;
//    case 0x203:P_Motor=&Robo->Pos_MotorRB;break;
//    case 0x204:P_Motor=&Robo->Pos_MotorLB;break;
//	default:break;
//  }if(P_Motor!=NULL) Pos_Info_Analyqsis(&P_Motor->Info,RX_Data);
//}

//--------------------------------------------------------------------------------------------------//
//3.��ȡ��ǰ�ٶȻ��ٶ����ݣ�speed��ڣ�
//
//��������:
//		�ٶȻ�������ݷ����Ľӿں���
//
//��������:
//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
//
//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
//		uint32_t �������
//
//��ֲ����:
//		ֱ�Ӷ�case�����ݽ����޸�, �м����ٶȻ������ӾͼӼ���, Ȼ����ָ��ָ���Ӧ�����Ӿ���.
//
//--------------------------------------------------------------------------------------------------//
//void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num)
//{
//  Speed_System* S_Motor=NULL;
//  switch(Motor_Num)
//  {
//    case 0x201:S_Motor=&Robo->Speed_MotorLF;break;
//    case 0x202:S_Motor=&Robo->Speed_MotorRF;break;
//    case 0x203:S_Motor=&Robo->Speed_MotorRB;break;
//    case 0x204:S_Motor=&Robo->Speed_MotorLB;break;
//	default:break;
//  }if(S_Motor!=NULL) Speed_Info_Analysis(&S_Motor->Info,RX_Data);
//}

//--------------------------------------------------------------------------------------------------//
//4.���ͽ������������ڣ�
//
//��������:
//		PID���ͺ���
//
//��������:
//		���͵��PID
//
//��������:
//		ROBO_BASE* ���̽ṹ��ָ��
//
//��ֲ����:
//		����Ҫɶ���Ŀ��ƾ���ָ��ָ�����ϵͳ, Ȼ����ö�Ӧ��PID���㺯�����д���
//
//--------------------------------------------------------------------------------------------------//
void PID_Send(ROBO_BASE* Robo)
{
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo->Pos_MotorLB; PID_Pos_Cal(P_Pos,Robo->Tx_CAN2);
  P_Pos=&Robo->Pos_MotorRB; PID_Pos_Cal(P_Pos,Robo->Tx_CAN2);
  P_Pos=&Robo->Pos_MotorLF; PID_Pos_Cal(P_Pos,Robo->Tx_CAN2);
  P_Pos=&Robo->Pos_MotorRF; PID_Pos_Cal(P_Pos,Robo->Tx_CAN2);
  Send_To_Motor(&hcan2,Robo->Tx_CAN2);

  Speed_System* P_Speed=NULL;
  P_Speed=&Robo->Speed_MotorLB; PID_Speed_Cal(P_Speed,Robo->Tx_CAN1);
  P_Speed=&Robo->Speed_MotorRB; PID_Speed_Cal(P_Speed,Robo->Tx_CAN1);
  P_Speed=&Robo->Speed_MotorLF; PID_Speed_Cal(P_Speed,Robo->Tx_CAN1);
  P_Speed=&Robo->Speed_MotorRF;PID_Speed_Cal(P_Speed,Robo->Tx_CAN1);
  Send_To_Motor(&hcan1,Robo->Tx_CAN1);
}

//--------------------------------------------------------------------------------------------------//
//5.�������ݸ���λ��
//
//��������:
//		ϵͳ�������ֵת��Ϊ�ַ�������
//
//��������:
//		���ٶȻ�/λ�û�ϵͳ���������ֵת�����ַ���, �Ա�ͨ�����ڷ��͵���λ��
//
//��������:
//		int32_t ϵͳ���ֵ
//		int32_t ϵͳ����ֵ
//
//��ֲ����:
//		������������ر��, ���԰�temp�������ٵ���һ��, ������Ҫ��֤�������ĳ��ȹ���
//--------------------------------------------------------------------------------------------------//
void SystemIO_Usart_ToString(int32_t System_Out,int32_t System_In)
{
  TX_BUFFER Usart_Tx;
  int32_t temp=1000000;
  uint8_t flag1=0;

  //ת��ϵͳ���ֵ
  Usart_Tx.length=0;
  if(System_Out<0) Usart_Tx.Tx_buffer[Usart_Tx.length++]='-',System_Out=-System_Out;
  else if(System_Out==0) Usart_Tx.Tx_buffer[Usart_Tx.length++]='0',temp=0;

  while(temp!=0)
  {
    if(System_Out/temp!=0)
    {
	  flag1=1;
	  Usart_Tx.Tx_buffer[Usart_Tx.length++]='0'+System_Out/temp;
	  System_Out-=System_Out/temp*temp;
	  if(System_Out==0)
	  {
		temp/=10;
	    while(temp!=0)
		{
		  Usart_Tx.Tx_buffer[Usart_Tx.length++]='0';
		  temp/=10;
		}break;
	  }
    }else if(flag1) Usart_Tx.Tx_buffer[Usart_Tx.length++]='0';
	temp/=10;
  }Usart_Tx.Tx_buffer[Usart_Tx.length++]=' ';
  temp=10000;
  flag1=0;

  //ת��ϵͳ����ֵ
  if(System_In<0) Usart_Tx.Tx_buffer[Usart_Tx.length++]='-',System_In=-System_In;
  else if(System_In==0) Usart_Tx.Tx_buffer[Usart_Tx.length++]='0',temp=0;

  while(temp!=0)
  {
    if(System_In/temp!=0)
    {
	  flag1=1;
	  Usart_Tx.Tx_buffer[Usart_Tx.length++]='0'+System_In/temp;
	  System_In-=System_In/temp*temp;
	  if(System_In==0)
	  {
		temp/=10;
	    while(temp!=0)
		{
		  Usart_Tx.Tx_buffer[Usart_Tx.length++]='0';
		  temp/=10;
		}break;
	  }
    }else if(flag1)  Usart_Tx.Tx_buffer[Usart_Tx.length++]='0';
	temp/=10;
  }
  Usart_Tx.Tx_buffer[Usart_Tx.length++]='\r';
  Usart_Tx.Tx_buffer[Usart_Tx.length]='\n';
}


/*********************************************************************************************************************************************************************/


//--------------------------------------------------------------------------------------------------//
//��������:
//		λ�û�������ݷ����Ĳ�������
//
//��������:
//		�������ݴ����������ݽ��н���, ���Ҽ�������ԽǶȺ���ԽǶ�.
//		Ĭ��ͨ��Э������:  	Data[0]----����ٶȸ�8λ
//							Data[1]----����ٶȵ�8λ
//							Data[2]----ת�ӽǶȸ�8λ
//							Data[3]----ת�ӽǶȵ�8λ
//							Data[4]----������С��8λ
//							Data[5]----������С��8λ
//							Data[6]----�¶�
//							Data[7]----NULL
//
//��������:
//		Motor_Pos_Info* λ�û������Ϣָ��
//		uint8_t* �����Ϣ������
//
//��ֲ����:
//		���ܲ���Ҫ��, Ҫ�ĵĻ�, ��Ϣ�����ĵط�����ͨ��Э�����ľ���.
//--------------------------------------------------------------------------------------------------//
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data)
{
  //���ݽ���
  Motor->Angle=(uint16_t)RX_Data[0]<<8|RX_Data[1];
  Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
  Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
  Motor->Temperature=RX_Data[6];

  //���ԽǶȼ���
  if (Motor->Speed!=0)
  {
    int16_t Error=Motor->Angle-Motor->Last_Angle;
    Motor->Abs_Angle += Error;
    if (Error < -4096)Motor->Abs_Angle += 8192;
    else if (Error > 4096)  Motor->Abs_Angle -= 8192;
  }Motor->Last_Angle=Motor->Angle;

  //��ԽǶȼ���, Ĭ�Ϸ�Χ0-360
  if(Motor->Abs_Angle>=0) Motor->Relative_Angle=(Motor->Abs_Angle%ONE_CIRCLE)*360.0/ONE_CIRCLE;
  else Motor->Relative_Angle=360-((-Motor->Abs_Angle)%ONE_CIRCLE)*360.0/ONE_CIRCLE;
}


//--------------------------------------------------------------------------------------------------//
//��������:
//		�ٶȻ�������ݷ����Ĳ�������
//
//��������:
//		�������ݴ����������ݽ��н���.
//		Ĭ��ͨ��Э������:  	Data[0]----����ٶȸ�8λ
//							Data[1]----����ٶȵ�8λ
//							Data[2]----ת�ӽǶȸ�8λ
//							Data[3]----ת�ӽǶȵ�8λ
//							Data[4]----������С��8λ
//							Data[5]----������С��8λ
//							Data[6]----�¶�
//							Data[7]----NULL
//
//��������:
//		Motor_Speed_Info* �ٶȻ������Ϣָ��
//		uint8_t* �����Ϣ������
//
//��ֲ����:
//		���ܲ���Ҫ��, Ҫ�ĵĻ�, ��Ϣ�����ĵط�����ͨ��Э�����ľ���.
//--------------------------------------------------------------------------------------------------//
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data)
{
  Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
  Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
  Motor->Temperature=RX_Data[6];
}



//--------------------------------------------------------------------------------------------------//
//��������:
//		PID������ʼ������
//
//��������:
//		��ʼ��ϵͳPID����
//
//��������:
//		PID* PIDָ��
//		float PID��Kp
//		float PID��Ki
//		float PID��Kd
//		float ������ֵ
//		float ����
//		float ����ۻ����ֵ
//		float ������ֵ
//
//--------------------------------------------------------------------------------------------------//
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max)           //������ֵ���ʼ��
{
	pid->Kp = Kp;                      
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->error_max = error_max;       //������
	pid->output_max = output_max;         //������
	pid->dead_line = dead_line;          //??
	
	pid->intergral_max = intergral_max;         //�������    
	
	pid->error = 0;                  //���
	pid->error_last = 0;               //��һ�����
	pid->intergral = 0;                //����
	pid->derivative = 0;              //��
	pid->output = 0;                  //���
}


//--------------------------------------------------------------------------------------------------//
//��������:
//		PID���㺯��
//
//��������:
//		����PID����
//
//��������:
//		PID* PIDָ��
//		float ��ǰֵ
//		float Ŀ��ֵ
//		uint8_t �������
//		uint8_t �������ݵ�����
//
//��ֲ����:
//		���ܷ�װ����ô��, ���鱣���ú�����Ҫ�޸�, ��Ϊ���¼��ݻ��߼�����ĺ���
//
//--------------------------------------------------------------------------------------------------//
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

//--------------------------------------------------------------------------------------------------//
//��������:
//		λ�û�PID���㺯��
//
//��������:
//		����λ�û�PID����
//
//��������:
//		Pos_System* λ�û�ϵͳָ��
//		uint8_t* �������ݵ�����
//
//--------------------------------------------------------------------------------------------------//
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

//--------------------------------------------------------------------------------------------------//
//��������:
//		�ٶȻ�PID���㺯��
//
//��������:
//		�����ٶȻ�PID����
//
//��������:
//		Speed_System* �ٶȻ�ϵͳָ��
//		uint8_t* �������ݵ�����
//
//--------------------------------------------------------------------------------------------------//
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



//--------------------------------------------------------------------------------------------------//
//��������:
//		//CANͨ�ŷ��ͺ���
//
//��������:
//		��������
//
//��������:
//		CAN_HandleTypeDef* CAN�ľ��
//		uint8_t* �������ݵ�����
//
//��ֲ����:
//		����Ҫ���޸ı�ʶ������
//--------------------------------------------------------------------------------------------------//
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
   /* Transmission request Error */
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
	Base->Speed_MotorLB.Motor_Num=2;
	PID_Init(&Base->Speed_MotorLB.Speed_PID,5,0,0,5000,0,5000,5000);
	
}
void My_Motor_Analysis(ROBO_BASE *Base,uint8_t Motor_Num)
{
	switch (Motor_Num)
	{
		case 0:
			My_Motor_control(&Base->Speed_MotorLF,&Base->Pos_MotorLF,RxData,TxData);
		case 1:
			My_Motor_control(&Base->Speed_MotorRF,&Base->Pos_MotorRF,RxData,TxData);
		case 2:
			My_Motor_control(&Base->Speed_MotorRB,&Base->Pos_MotorRB,RxData,TxData);
		case 3:
			My_Motor_control(&Base->Speed_MotorLB,&Base->Pos_MotorLB,RxData,TxData);
		default :
			break;
	}
}