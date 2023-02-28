#include "exti.h"


#include "oled.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "uart.h"
#include "stdio.h"
#include "stm32f10x.h"
#include "valuepack.h"
#include "encoder.h"

  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/


	float Med=-0.50		;			//�������
	float balance_UP_KP=180;			//��������kp				//	kp��Χ��0~700��
	float balance_UP_KD=1.08
		;			//��������kd			// kdΪ��ֵΪ�����
	int pid_pwm	;					//pwm�����
	int pid_banlance=0;				//��������������


	float velocity_KP=1.09;	//�ٶȻ�KP��KI
	float velocity_KI=0.00545;
	int Encoder_Left;		//���ֱ�����
	int Encoder_Right;		//���ֱ�����
	int pid_velocity=0;				//����ٶȻ��������
	

	float Turn_Kp=0;
	int pid_Turn=0;				//����ٶȻ��������
	int MOTO1,MOTO2;								//���װ�ر���


	 float pitch,roll,yaw; 		//ŷ����
	short gyrox,gyroy,gyroz;				//������--���ٶ�
	short aacx,aacy,aacz;						//���ٶ�
	float temp;		
	

	extern TxPack txpack;				//����ͨѶ���͵ĵĽṹ��
	extern RxPack rxpack;				//����ͨѶ���յĵĽṹ��
	




void MiniBalance_EXTI_Init(void)
{  
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIO�˿�ʱ��
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	            //�˿�����
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
	
	
		EXTI_InitStructure.EXTI_Line=EXTI_Line12;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
	
	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//��ռ���ȼ�0�� 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure); 
}


/*****************  
ֱ����PD��������Kp*Ek+Kd*Ek_D
��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
���ڣ�ֱ�������
******************/

int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;												//�Ƕ����
   int balance;												//ֱ������������ĵ������pwm
   Bias=Angle-Mechanical_balance;                   
   //===���ƽ��ĽǶ���ֵ�ͻ�е���
   balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  
   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
   return balance;
}




/**************************************
��ڲ����������������ֵ
����  ֵ���ٶȿ���PWM
�����ı�������ֵ���ҵ���ı���ֵ
**************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
    static float Encoder_Integral;
   //=============�ٶ�PI������=======================//  
    Encoder_Least =(Encoder_Left+Encoder_Right)-0;      
    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
    Encoder *= 0.7;          //===һ�׵�ͨ�˲���       
    Encoder += Encoder_Least*0.3;   //===һ�׵�ͨ�˲���    
    Encoder_Integral +=Encoder; //===���ֳ�λ�� ����ʱ�䣺10ms
    if(Encoder_Integral>10000)    Encoder_Integral=10000;   
    //===�����޷�
    if(Encoder_Integral<-10000)    Encoder_Integral=-10000;   
    //===�����޷�  
    Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;  
    //===�ٶȿ���  
    if(pitch<-40||pitch>40)   Encoder_Integral=0;   
    //===����رպ��������
    return Velocity;
}




/*********************
ת�򻷣�ϵ��*Z����ٶ�
*********************/
int Turn(int gyro_Z)
{
	int PWM_out;
	
	PWM_out=Turn_Kp*gyro_Z;
	return PWM_out;
}





/*****************  
���PWM���ƣ�������PWM
******************/
void Limit(int pwm_left)
{
				if(pid_pwm > 7200)
				{
					pid_pwm=7200;
				}
				if(pid_pwm < -7200)
				{
					pid_pwm=-7200;
				}
	
}

/*****************  
���������ƣ��жϵ��ת�������
******************/
void motor(int pwm)

{
	if(pid_pwm > 0)
				{
					TIM_SetCompare1(TIM3,0);	
					TIM_SetCompare3(TIM3,0);
					TIM_SetCompare2(TIM3,pid_pwm);	
					TIM_SetCompare4(TIM3,pid_pwm);
			}
			else{
					pid_pwm = -pid_pwm;
					TIM_SetCompare2(TIM3,0);	
					TIM_SetCompare4(TIM3,0);
					TIM_SetCompare1(TIM3,pid_pwm);	
					TIM_SetCompare3(TIM3,pid_pwm);
				}		
	
}


void EXTI15_10_IRQHandler(void) 
{
	
	

	
	 if(EXTI_GetITStatus(EXTI_Line12)!=0)//һ���ж�==0)		
	{   
	
			EXTI_ClearITPendingBit(EXTI_Line12);//����жϱ�־λ
		
			Encoder_Left=-Read_Encoder(2);                                       //===��ȡ��������ֵ
			Encoder_Right=Read_Encoder(4);    									//===��ȡ��������ֵ
			mpu_dmp_get_data(&pitch,&roll,&yaw);
		
//			temp=MPU_Get_Temperature();							  //�õ��¶�ֵ
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//������
	
			
				pid_banlance=balance_UP(pitch,Med, gyroy);		//������	
				pid_velocity=velocity( Encoder_Left,Encoder_Right);
//			pid_Turn=Turn( gyroz);
		
				pid_pwm=pid_banlance-balance_UP_KP*pid_velocity;//�������
		
				MOTO1=pid_pwm-pid_Turn;//����
				MOTO2=pid_pwm+pid_Turn;//�ҵ��	
				Limit(pid_pwm);								//���pwm������
				motor(pid_pwm);								//���ת����ж�
				
			}
	}










