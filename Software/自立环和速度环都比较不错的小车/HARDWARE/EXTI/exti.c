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
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：外部中断初始化
入口参数：无
返回  值：无 
**************************************************************************/


	float Med=-0.50		;			//极限零点
	float balance_UP_KP=180;			//自立环的kp				//	kp范围（0~700）
	float balance_UP_KD=1.08
		;			//自立环的kd			// kd为负值为正向的
	int pid_pwm	;					//pwm的输出
	int pid_banlance=0;				//电机自立环的输出


	float velocity_KP=1.09;	//速度环KP、KI
	float velocity_KI=0.00545;
	int Encoder_Left;		//左轮编码器
	int Encoder_Right;		//右轮编码器
	int pid_velocity=0;				//电机速度环环的输出
	

	float Turn_Kp=0;
	int pid_Turn=0;				//电机速度环环的输出
	int MOTO1,MOTO2;								//电机装载变量


	 float pitch,roll,yaw; 		//欧拉角
	short gyrox,gyroy,gyroz;				//陀螺仪--角速度
	short aacx,aacy,aacz;						//加速度
	float temp;		
	

	extern TxPack txpack;				//蓝牙通讯发送的的结构体
	extern RxPack rxpack;				//蓝牙通讯接收的的结构体
	




void MiniBalance_EXTI_Init(void)
{  
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIO端口时钟
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	            //端口配置
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
	
	
		EXTI_InitStructure.EXTI_Line=EXTI_Line12;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键所在的外部中断通道
	
	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//抢占优先级0， 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure); 
}


/*****************  
直立环PD控制器：Kp*Ek+Kd*Ek_D
入口：Med:机械中值(期望角度)，Angle:真实角度，gyro_Y:真实角速度
出口：直立环输出
******************/

int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;												//角度误差
   int balance;												//直立环计算出来的电机控制pwm
   Bias=Angle-Mechanical_balance;                   
   //===求出平衡的角度中值和机械相关
   balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  
   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
   return balance;
}




/**************************************
入口参数：电机编码器的值
返回  值：速度控制PWM
左电机的编码器的值和右电机的编码值
**************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
    static float Encoder_Integral;
   //=============速度PI控制器=======================//  
    Encoder_Least =(Encoder_Left+Encoder_Right)-0;      
    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
    Encoder *= 0.7;          //===一阶低通滤波器       
    Encoder += Encoder_Least*0.3;   //===一阶低通滤波器    
    Encoder_Integral +=Encoder; //===积分出位移 积分时间：10ms
    if(Encoder_Integral>10000)    Encoder_Integral=10000;   
    //===积分限幅
    if(Encoder_Integral<-10000)    Encoder_Integral=-10000;   
    //===积分限幅  
    Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;  
    //===速度控制  
    if(pitch<-40||pitch>40)   Encoder_Integral=0;   
    //===电机关闭后清除积分
    return Velocity;
}




/*********************
转向环：系数*Z轴角速度
*********************/
int Turn(int gyro_Z)
{
	int PWM_out;
	
	PWM_out=Turn_Kp*gyro_Z;
	return PWM_out;
}





/*****************  
电机PWM限制，控制在PWM
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
电机方向控制，判断电机转向的正否
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
	
	

	
	 if(EXTI_GetITStatus(EXTI_Line12)!=0)//一级判定==0)		
	{   
	
			EXTI_ClearITPendingBit(EXTI_Line12);//清除中断标志位
		
			Encoder_Left=-Read_Encoder(2);                                       //===读取编码器的值
			Encoder_Right=Read_Encoder(4);    									//===读取编码器的值
			mpu_dmp_get_data(&pitch,&roll,&yaw);
		
//			temp=MPU_Get_Temperature();							  //得到温度值
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//陀螺仪
	
			
				pid_banlance=balance_UP(pitch,Med, gyroy);		//自立环	
				pid_velocity=velocity( Encoder_Left,Encoder_Right);
//			pid_Turn=Turn( gyroz);
		
				pid_pwm=pid_banlance-balance_UP_KP*pid_velocity;//最终输出
		
				MOTO1=pid_pwm-pid_Turn;//左电机
				MOTO2=pid_pwm+pid_Turn;//右电机	
				Limit(pid_pwm);								//电机pwm的限制
				motor(pid_pwm);								//电机转向的判定
				
			}
	}










