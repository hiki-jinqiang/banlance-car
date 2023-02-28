/****************************************
PA15:AD0   PB10:IIC_SCL   PB11:IIC_SDA
PA1:IIC_SCL_OLED      PA2:IIC_SDA_OLED

		2021 11 19日整理版
		

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


int balance_UP(float Angle,float Mechanical_balance,float Gyro);	//自立环的函数声明	


 TxPack txpack;
 RxPack rxpack;


extern	float pitch,roll,yaw; 		//欧拉角
extern	short gyrox,gyroy,gyroz;				//陀螺仪--角速度
extern	short aacx,aacy,aacz;						//加速度
extern	float temp;	

u32 Distance;			//超声波的距离参数

extern int Encoder_Left;		//左轮编码器
extern int Encoder_Right;		//右轮编码器

u16 adcx;				//电压参数



extern float Med;		;			//极限零点
extern float balance_UP_KP;			//自立环的kp				//	kp范围（0~700）
extern float balance_UP_KD ;			//自立环的kd			// kd为负值为正向的
extern int pid_pwm	;					//pwm的输出




 int main(void)
 {	 
//	u16 t;  			//串口3专用
//	u16 len;	
//	u16 times=0;
//	u8 dir=1;
//	 
//	u16 led0pwmval=0; 
	 
	

	 
	 initValuePack(115200);	 			//串口三的初始化
	 
	LED_Init();						    //LED端口初始化
	BEEP_Init();						//BEEP端口初始化
	delay_init();				       //延时初始化
	Adc_Init();		  					//ADC初始化
	OLED_Init();
	OLED_ColorTurn(0);         //0正常显示，1 反色显示
	OLED_DisplayTurn(0);       //0正常显示 1 屏幕翻转显示
	MPU_Init();					       //初始化MPU6050
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级


	 
	OLED_ShowString(15,0,"Waiting",24,1);
	OLED_ShowString(0,28,"Initialise",24,1);
	OLED_Refresh();
	
//	TIM1_Cap_Init(0xFFFF,71);				//超声波初始化

	Encoder_Init_TIM2();            //=====初始化编码器2
    Encoder_Init_TIM4();            //=====初始化编码器4
	
	delay_ms(100);					//延时给陀螺仪的初始化

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
	
	 
	TIM3_PWM1_Init(7199,0);	 //不分频。PWM频率=72000000/5000*100=902hz
	TIM_SetCompare1(TIM3,0);		
 	TIM3_PWM2_Init(7199,0);	 //不分频。PWM频率=72000000/7200*9=902hz
	TIM_SetCompare2(TIM3,0);		

	 
	TIM3_PWM4_Init(7199,0);
	TIM_SetCompare4(TIM3,0);
	TIM3_PWM3_Init(7199,0);
	TIM_SetCompare3(TIM3,0);	       //1和3一个电机，2和4一个电机
		
	 MiniBalance_EXTI_Init();	//外部中断初始化
	
 	while(1)
	{
	
		
//			LED0=!LED0;
//			Encoder_Left=Read_Encoder(2);                                       //===读取编码器的值
//			Encoder_Right=Read_Encoder(4);  
			OLED_ShowNum(90,47,Encoder_Left,4,12,1);				//显示pwmout的值	
			OLED_ShowNum(30,47,Encoder_Right,4,12,1);				//显示pwmout的值		
//			OLED_ShowFloat(90,47,Encoder_Left,5,12,1);
//			OLED_ShowFloat(30,47,Encoder_Right,5,12,1);
		

			
			
			
			
			


			
				OLED_ShowFloat(16, 0,pitch,1,12,1);
//				OLED_ShowFloat(16,15,roll,5,12,1);
//				OLED_ShowFloat(16,31,yaw,5,12,1);
//				OLED_ShowFloat(75, 0,(float)gyrox,5,12,1);				//加速度
				OLED_ShowFloat(75,0,(float)gyroy,5,12,1);
//				OLED_ShowFloat(75,31,(float)gyroz,5,12,1);
//				OLED_ShowFloat(30,47,(float)temp/100,5,12,1);
				OLED_Refresh();
				OLED_ShowNum(75,15,pid_pwm,5,12,1);				//显示pwmout的值

				
				
			/****			蓝牙格式的发收  ******/	
				if(readValuePack(&rxpack))
			{
			
				// 在此读取手机传来的数据
				
				// 这里是将接收的数据原样回传
//				txpack.bools[0] = rxpack.bools[0];
//				txpack.bytes[0] = rxpack.bytes[0];
//				txpack.shorts[0] = rxpack.shorts[0];
//				txpack.integers[0] = rxpack.integers[0];
				
				balance_UP_KP=rxpack.floats[0];
				balance_UP_KD=rxpack.floats[1];
				txpack.floats[0] =balance_UP_KP;
				txpack.floats[1] = balance_UP_KD;
				
				txpack.floats[2]=pitch;
				// 也可以把 sendValuePack放在这，这样就只有当接收到手机传来的数据包后才回传数据
				
				
			}
			
			// 在此对数据包赋值并将数据发送到手机
      
			sendValuePack(&txpack);	
				
				
			LED0=!LED0;
			delay_ms(50);
				
		
	} 	
}
	



/*****************  
直立环PD控制器：Kp*Ek+Kd*Ek_D
入口：Med:机械中值(期望角度)，Angle:真实角度，gyro_Y:真实角速度
出口：直立环输出
******************/

//int balance_UP(float Angle,float Mechanical_balance,float Gyro)
//{  
//   float Bias;												//角度误差
//   int balance;												//直立环计算出来的电机控制pwm
//   Bias=Angle-Mechanical_balance;                   
//   //===求出平衡的角度中值和机械相关
//   balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  
//   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
//   return balance;
//}




/****	正点原子的usatr3串口的接收 ****/



//		
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
//			printf("\r\n您发送的消息为:\r\n\r\n");
//			for(t=0;t<len;t++)
//			{
//				USART_SendData(USART3, USART_RX_BUF[t]);//向串口1发送数据
//				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
//			}
//			printf("\r\n\r\n");//插入换行
//			USART_RX_STA=0;
//		}else
//		{
//			times++;
//			if(times%5000==0)
//			{
//				printf("\r\n精英STM32开发板 串口实验\r\n");
//				printf("正点原子@ALIENTEK\r\n\r\n");
//			}
//			if(times%200==0)printf("请输入数据,以回车键结束\n");  
//			if(times%30==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
//			delay_ms(10);   
//		}
		


	//		Read_TIM1Distane();			//超声波测量函数






//					aacx = (2*9.8*aacx)/32768;				//
//					aacy = (2*9.8*aacy)/32768;
//					aacz = (2*9.8*aacz)/32768;
