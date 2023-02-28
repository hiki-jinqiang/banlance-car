/****************************************
PA15:AD0   PB10:IIC_SCL   PB11:IIC_SDA
PA1:IIC_SCL_OLED      PA2:IIC_SDA_OLED

		2021 11 19�������
		

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
#include "stm32f10x.h"
#include "valuepack.h"
#include "exti.h"


int balance_UP(float Angle,float Mechanical_balance,float Gyro);	//�������ĺ�������	


 TxPack txpack;
 RxPack rxpack;


extern	float pitch,roll,yaw; 		//ŷ����
extern	short gyrox,gyroy,gyroz;				//������--���ٶ�
extern	short aacx,aacy,aacz;						//���ٶ�
extern	float temp;	

u32 Distance;			//�������ľ������

extern int Encoder_Left;		//���ֱ�����
extern int Encoder_Right;		//���ֱ�����

u16 adcx;				//��ѹ����



extern float Med;		;			//�������
extern float balance_UP_KP;			//��������kp				//	kp��Χ��0~700��
extern float balance_UP_KD ;			//��������kd			// kdΪ��ֵΪ�����
extern int pid_pwm	;					//pwm�����




 int main(void)
 {	 
//	u16 t;  			//����3ר��
//	u16 len;	
//	u16 times=0;
//	u8 dir=1;
//	 
//	u16 led0pwmval=0; 
	 
	

	 
	 initValuePack(115200);	 			//�������ĳ�ʼ��
	 
	LED_Init();						    //LED�˿ڳ�ʼ��
	BEEP_Init();						//BEEP�˿ڳ�ʼ��
	delay_init();				       //��ʱ��ʼ��
	Adc_Init();		  					//ADC��ʼ��
	OLED_Init();
	OLED_ColorTurn(0);         //0������ʾ��1 ��ɫ��ʾ
	OLED_DisplayTurn(0);       //0������ʾ 1 ��Ļ��ת��ʾ
	MPU_Init();					       //��ʼ��MPU6050
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�


	 
	OLED_ShowString(15,0,"Waiting",24,1);
	OLED_ShowString(0,28,"Initialise",24,1);
	OLED_Refresh();
	
//	TIM1_Cap_Init(0xFFFF,71);				//��������ʼ��

	Encoder_Init_TIM2();            //=====��ʼ��������2
    Encoder_Init_TIM4();            //=====��ʼ��������4
	
	delay_ms(100);					//��ʱ�������ǵĳ�ʼ��

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
	OLED_Refresh();
	
	 
	TIM3_PWM1_Init(7199,0);	 //����Ƶ��PWMƵ��=72000000/5000*100=902hz
	TIM_SetCompare1(TIM3,0);		
 	TIM3_PWM2_Init(7199,0);	 //����Ƶ��PWMƵ��=72000000/7200*9=902hz
	TIM_SetCompare2(TIM3,0);		

	 
	TIM3_PWM4_Init(7199,0);
	TIM_SetCompare4(TIM3,0);
	TIM3_PWM3_Init(7199,0);
	TIM_SetCompare3(TIM3,0);	       //1��3һ�������2��4һ�����
		
	 MiniBalance_EXTI_Init();	//�ⲿ�жϳ�ʼ��
	
 	while(1)
	{
	
		
//			LED0=!LED0;
//			Encoder_Left=Read_Encoder(2);                                       //===��ȡ��������ֵ
//			Encoder_Right=Read_Encoder(4);  
			OLED_ShowNum(90,47,Encoder_Left,4,12,1);				//��ʾpwmout��ֵ	
			OLED_ShowNum(30,47,Encoder_Right,4,12,1);				//��ʾpwmout��ֵ		
//			OLED_ShowFloat(90,47,Encoder_Left,5,12,1);
//			OLED_ShowFloat(30,47,Encoder_Right,5,12,1);
		

			
			
			
			
			


			
				OLED_ShowFloat(16, 0,pitch,1,12,1);
//				OLED_ShowFloat(16,15,roll,5,12,1);
//				OLED_ShowFloat(16,31,yaw,5,12,1);
//				OLED_ShowFloat(75, 0,(float)gyrox,5,12,1);				//���ٶ�
				OLED_ShowFloat(75,0,(float)gyroy,5,12,1);
//				OLED_ShowFloat(75,31,(float)gyroz,5,12,1);
//				OLED_ShowFloat(30,47,(float)temp/100,5,12,1);
				OLED_Refresh();
				OLED_ShowNum(75,15,pid_pwm,5,12,1);				//��ʾpwmout��ֵ

				
				
			/****			������ʽ�ķ���  ******/	
				if(readValuePack(&rxpack))
			{
			
				// �ڴ˶�ȡ�ֻ�����������
				
				// �����ǽ����յ�����ԭ���ش�
//				txpack.bools[0] = rxpack.bools[0];
//				txpack.bytes[0] = rxpack.bytes[0];
//				txpack.shorts[0] = rxpack.shorts[0];
//				txpack.integers[0] = rxpack.integers[0];
				
				balance_UP_KP=rxpack.floats[0];
				balance_UP_KD=rxpack.floats[1];
				txpack.floats[0] =balance_UP_KP;
				txpack.floats[1] = balance_UP_KD;
				
				txpack.floats[2]=pitch;
				// Ҳ���԰� sendValuePack�����⣬������ֻ�е����յ��ֻ����������ݰ���Żش�����
				
				
			}
			
			// �ڴ˶����ݰ���ֵ�������ݷ��͵��ֻ�
      
			sendValuePack(&txpack);	
				
				
			LED0=!LED0;
			delay_ms(50);
				
		
	} 	
}
	



/*****************  
ֱ����PD��������Kp*Ek+Kd*Ek_D
��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
���ڣ�ֱ�������
******************/

//int balance_UP(float Angle,float Mechanical_balance,float Gyro)
//{  
//   float Bias;												//�Ƕ����
//   int balance;												//ֱ������������ĵ������pwm
//   Bias=Angle-Mechanical_balance;                   
//   //===���ƽ��ĽǶ���ֵ�ͻ�е���
//   balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  
//   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
//   return balance;
//}




/****	����ԭ�ӵ�usatr3���ڵĽ��� ****/



//		
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			printf("\r\n�����͵���ϢΪ:\r\n\r\n");
//			for(t=0;t<len;t++)
//			{
//				USART_SendData(USART3, USART_RX_BUF[t]);//�򴮿�1��������
//				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
//			}
//			printf("\r\n\r\n");//���뻻��
//			USART_RX_STA=0;
//		}else
//		{
//			times++;
//			if(times%5000==0)
//			{
//				printf("\r\n��ӢSTM32������ ����ʵ��\r\n");
//				printf("����ԭ��@ALIENTEK\r\n\r\n");
//			}
//			if(times%200==0)printf("����������,�Իس�������\n");  
//			if(times%30==0)LED0=!LED0;//��˸LED,��ʾϵͳ��������.
//			delay_ms(10);   
//		}
		


	//		Read_TIM1Distane();			//��������������






//					aacx = (2*9.8*aacx)/32768;				//
//					aacy = (2*9.8*aacy)/32768;
//					aacz = (2*9.8*aacz)/32768;
