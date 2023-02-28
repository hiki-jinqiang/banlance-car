#ifndef __BSP_PID_H
#define	__BSP_PID_H
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
    float target_val;           //Ŀ��ֵ
    float actual_val;        		//ʵ��ֵ
    float err;             			//����ƫ��ֵ
		float err_next;             //������һ��ƫ��ֵ  ����ʽ����Ҫ          
    float err_last;          		//������һ��ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
}_pid;

extern _pid pid; //С��ƽ��pid

extern void PID_param_init(void);
extern void set_pid_target(float temp_val);
extern float get_pid_target(void);
extern void set_p_i_d(float p, float i, float d);
extern float PID_realize(float actual_val);
extern void time_period_fun(void);

#endif
