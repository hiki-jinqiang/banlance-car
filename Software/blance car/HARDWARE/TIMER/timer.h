#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/4
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//********************************************************************************

extern u32 Distance;


void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM1_Init(u16 arr,u16 psc);
void TIM3_PWM2_Init(u16 arr,u16 psc);
void TIM3_PWM3_Init(u16 arr,u16 psc);
void TIM3_PWM4_Init(u16 arr,u16 psc);
void TIM1_Cap_Init(u16 arr,u16 psc);
void Read_TIM1Distane(void);

#endif