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
	u16 t;  			//串口3专用
	u16 len;	
	u16 times=0;
	u8 dir=1;
	float tem;
	u16 led0pwmval=0; 
	int pid_pwm = 0;
	 
	 
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		  //加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;								//温度	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级

	 
	LED_Init();						    //LED端口初始化
	BEEP_Init();						//BEEP端口初始化
	EXTIX_Init();						//EXTIX中断初始化
	delay_init();				       //延时初始化
//	Adc_Init();		  					//ADC初始化
	OLED_Init();
	OLED_ColorTurn(0);         //0正常显示，1 反色显示
	OLED_DisplayTurn(0);       //0正常显示 1 屏幕翻转显示
	MPU_Init();					       //初始化MPU6050
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
  PID_param_init();
	 
 	TIM3_PWM1_Init(4999,99);	 //不分频。PWM频率=72000000/7200*9=902hz
	TIM_SetCompare1(TIM3,0);		
 	TIM3_PWM2_Init(4999,99);	 //不分频。PWM频率=72000000/7200*9=902hz
	TIM_SetCompare2(TIM3,0);		
//	GPIO_SetBits(GPIOA,GPIO_Pin_7); 						 //PA.7 输出高  
	 
	TIM3_PWM4_Init(4999,99);
	TIM_SetCompare4(TIM3,0);
	TIM3_PWM3_Init(4999,99);
	TIM_SetCompare3(TIM3,0);	       //1和3一个电机，2和4一个电机
//24正向，13反向。	（以正值与0比较）
//以平衡车读法为正面，1和2占空比一组，3和4占空比一组
//pid正向为正数增大



//	 GPIO_SetBits(GPIOB,GPIO_Pin_0); 						 //PB.0 输出高 
	 
	OLED_ShowString(15,0,"Waiting",24,1);
	OLED_ShowString(0,28,"Initialise",24,1);
	OLED_Refresh();
	
//	TIM1_Cap_Init(0xFFFF,71);
//	 Encoder_Init_TIM2();            //=====编码器接口
//    Encoder_Init_TIM4();            //=====初始化编码器2

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


//正向转动	
		
		

		
		
		Encoder_Left=Read_Encoder(2);                                       //===读取编码器的值
		Encoder_Right=Read_Encoder(4);    

		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
			LED0=!LED0;
			temp=MPU_Get_Temperature();							  //得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据

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
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias,kp=100,kd=0;
	 int balance;
	 Bias=Angle;       //===求出平衡的角度中值 和机械相关
   Gyro=Gyro+57;
	 balance=kp*Bias+Gyro*kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
// 	TIM3_PWM1_Init(71,999);	 //不分频。PWM频率=72000000/7200*9=902hz
//	TIM_SetCompare1(TIM3,500);		
// 	TIM3_PWM2_Init(71,999);	 //不分频。PWM频率=72000000/7200*9=902hz
//	TIM_SetCompare2(TIM3,500);		
//	TIM3_PWM4_Init(71,999);
//	TIM_SetCompare4(TIM3,500);
//	TIM3_PWM3_Init(71,999);
	


