/****************************************
PA15:AD0   PB10:IIC_SCL   PB11:IIC_SDA
PA1:IIC_SCL_OLED      PA2:IIC_SDA_OLED

2021
10
14
****************************************/
#include "led.h"
#include "beep.h"
#include "exti.h"
#include "timer.h"
#include "oled.h"
#include "delay.h"
#include "adc.h"
#include "sys.h"
#include "usart.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "uart.h"
#include "stdio.h"
#include "encoder.h"
#include "bsp_pid.h"


u32 Distance;
int Encoder_Left;
int Encoder_Right;
u16 adcx;
float temp;
int balance(float Angle,float Gyro);
 int main(void)
 {	 
	u16 t;  			//����3ר��
	u16 len;	
	u16 times=0;
	u8 dir=1;
	float tem;
	u16 led0pwmval=0; 
	int pid_pwm = 0;
	 
	 
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		  //���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;								//�¶�	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�

	 
	LED_Init();						    //LED�˿ڳ�ʼ��
	BEEP_Init();						//BEEP�˿ڳ�ʼ��
	EXTIX_Init();						//EXTIX�жϳ�ʼ��
	delay_init();				       //��ʱ��ʼ��
//	Adc_Init();		  					//ADC��ʼ��
	OLED_Init();
	OLED_ColorTurn(0);         //0������ʾ��1 ��ɫ��ʾ
	OLED_DisplayTurn(0);       //0������ʾ 1 ��Ļ��ת��ʾ
	MPU_Init();					       //��ʼ��MPU6050
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
  PID_param_init();
	 
 	TIM3_PWM1_Init(4999,99);	 //����Ƶ��PWMƵ��=72000000/7200*9=902hz
	TIM_SetCompare1(TIM3,0);		
 	TIM3_PWM2_Init(4999,99);	 //����Ƶ��PWMƵ��=72000000/7200*9=902hz
	TIM_SetCompare2(TIM3,0);		
//	GPIO_SetBits(GPIOA,GPIO_Pin_7); 						 //PA.7 �����  
	 
	TIM3_PWM4_Init(4999,99);
	TIM_SetCompare4(TIM3,0);
	TIM3_PWM3_Init(4999,99);
	TIM_SetCompare3(TIM3,0);	       //1��3һ�������2��4һ�����
//24����13����	������ֵ��0�Ƚϣ�
//��ƽ�⳵����Ϊ���棬1��2ռ�ձ�һ�飬3��4ռ�ձ�һ��
//pid����Ϊ��������



//	 GPIO_SetBits(GPIOB,GPIO_Pin_0); 						 //PB.0 ����� 
	 
	OLED_ShowString(15,0,"Waiting",24,1);
	OLED_ShowString(0,28,"Initialise",24,1);
	OLED_Refresh();
	
//	TIM1_Cap_Init(0xFFFF,71);
//	 Encoder_Init_TIM2();            //=====�������ӿ�
//    Encoder_Init_TIM4();            //=====��ʼ��������2

	while(mpu_dmp_init())
 	{
		OLED_Clear();
		OLED_Refresh();
	}
	OLED_Clear();
	OLED_Refresh();
	OLED_ShowString(0,0,"PI:",12,1);
	OLED_ShowString(0,15,"RO:",12,1);
	OLED_ShowString(0,31,"YA:",12,1);
	OLED_ShowString(0,47,"TEMP:",12,1);
//	OLED_ShowNum(90,47,22,3,12,1);
	OLED_Refresh();
	BEEP=0;
 
 	while(1)
	{


//		LED0=!LED0;
//		led0pwmval=100;
//		TIM_SetCompare1(TIM3,led0pwmval);
//    delay_ms(10);
//		TIM_SetCompare3(TIM3,led0pwmval);	


//����ת��	
		
		

		
		
		Encoder_Left=Read_Encoder(2);                                       //===��ȡ��������ֵ
		Encoder_Right=Read_Encoder(4);    

		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
			LED0=!LED0;
			temp=MPU_Get_Temperature();							  //�õ��¶�ֵ
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������

			OLED_ShowFloat(16, 0,pitch,1,12,1);
			OLED_ShowFloat(16,15,roll,5,12,1);
			OLED_ShowFloat(16,31,yaw,5,12,1);
			OLED_ShowFloat(75, 0,gyrox,1,12,1);
			OLED_ShowFloat(75,15,gyroy,5,12,1);
			OLED_ShowFloat(75,31,gyroz,5,12,1);
			OLED_Refresh();
         ADC_OLED();
		}
//if(pitch>30||pitch<-30)
//{
//	TIM_SetCompare1(TIM3,0);	
//	TIM_SetCompare3(TIM3,0);
//	TIM_SetCompare2(TIM3,0);	
//	TIM_SetCompare4(TIM3,0);
//}
//else if(pitch>0)
//{
// led0pwmval=balance(pitch,gyrox);
//	TIM_SetCompare1(TIM3,0);	
//	TIM_SetCompare3(TIM3,0);
//	TIM_SetCompare2(TIM3,led0pwmval);	
//	TIM_SetCompare4(TIM3,led0pwmval);
//}
//else
//{
//led0pwmval=balance(pitch,gyrox);
//led0pwmval=-led0pwmval;
//	TIM_SetCompare2(TIM3,0);	
//	TIM_SetCompare4(TIM3,0);
//	TIM_SetCompare1(TIM3,led0pwmval);	
//	TIM_SetCompare3(TIM3,led0pwmval);
//}

	pid_pwm = PID_realize(pitch);
	if(pid_pwm > 0){
		TIM_SetCompare1(TIM3,0);	
	  TIM_SetCompare3(TIM3,0);
	  TIM_SetCompare2(TIM3,pid_pwm);	
  	TIM_SetCompare4(TIM3,pid_pwm);
	}else{
		pid_pwm = -pid_pwm;
		TIM_SetCompare2(TIM3,0);	
  	TIM_SetCompare4(TIM3,0);
	  TIM_SetCompare1(TIM3,pid_pwm);	
  	TIM_SetCompare3(TIM3,pid_pwm);
	}



 } 	
}


/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias,kp=100,kd=0;
	 int balance;
	 Bias=Angle;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
   Gyro=Gyro+57;
	 balance=kp*Bias+Gyro*kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}
// 	TIM3_PWM1_Init(71,999);	 //����Ƶ��PWMƵ��=72000000/7200*9=902hz
//	TIM_SetCompare1(TIM3,500);		
// 	TIM3_PWM2_Init(71,999);	 //����Ƶ��PWMƵ��=72000000/7200*9=902hz
//	TIM_SetCompare2(TIM3,500);		
//	TIM3_PWM4_Init(71,999);
//	TIM_SetCompare4(TIM3,500);
//	TIM3_PWM3_Init(71,999);
	


