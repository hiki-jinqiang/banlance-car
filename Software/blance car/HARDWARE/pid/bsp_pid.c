#include "bsp_pid.h"
 #include "delay.h"

//����ȫ�ֱ���

_pid pid;

/**
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */
void PID_param_init()
{
		/* ��ʼ������ */
    pid.target_val=0.0;				
    pid.actual_val=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.integral=0.0;

		pid.Kp = 50.0;
		pid.Ki = 0.0; //0.005
		pid.Kd = 0.0;
#if defined(PID_ASSISTANT_EN)
    float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // ��ͨ�� 1 ���� P I D ֵ
#endif
}

/**
  * @brief  ����Ŀ��ֵ
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ��
  */
void set_pid_target(float temp_val)
{
  pid.target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(void)
{
  return pid.target_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ���ñ��������֡�΢��ϵ��
  * @param  p������ϵ�� P
  * @param  i������ϵ�� i
  * @param  d��΢��ϵ�� d
	*	@note 	��
  * @retval ��
  */
void set_p_i_d(float p, float i, float d)
{
  	pid.Kp = p;    // ���ñ���ϵ�� P
		pid.Ki = i;    // ���û���ϵ�� I
		pid.Kd = d;    // ����΢��ϵ�� D
}

/**
  * @brief  PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */

//λ��ʽpid
float PID_realize(float actual_val)
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid.err=pid.target_val-(-actual_val);
    /*����ۻ�*/
    pid.integral += (-pid.err);
	 
//		/*�����޷�*/
//		if(pid.integral > 300.0f){
//			pid.integral = 300.0;
//		}else if(pid.integral < -300.0f){
//			pid.integral = -300.0;
//		}
		
		if(actual_val >= 35 || actual_val<= -35){
			/*PID�㷨ʵ��*/
			pid.actual_val=pid.Kp*pid.err+pid.Kd*(pid.err-pid.err_last);
		}else{
			pid.actual_val=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
		}
		
		/*����*/
    pid.err_last=pid.err;	
		
//		/*1700Ϊ�е�λ�ã�����޷�*/
//		pid.actual_val += 1700; 
//		if(pid.actual_val >2200){
//			pid.actual_val = 2200;
//		}else if(pid.actual_val < 1200){
//			pid.actual_val = 1200;
//		}
//		else if(pid.actual_val < 1750 && pid.actual_val >1650){
//			pid.actual_val = 1700;
//		}		
		/*���ص�ǰʵ��ֵ*/
    return pid.actual_val;
}

////����ʽpid
//float PID_realize(float actual_val)
//{
//	/*����Ŀ��ֵ��ʵ��ֵ�����*/
//  pid.err=pid.target_val-actual_val;
//	/*PID�㷨ʵ��*/
//	pid.actual_val += pid.Kp*(pid.err - pid.err_next) 
//                 + pid.Ki*pid.err 
//                 + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);
//	/*�������*/
//	pid.err_last = pid.err_next;
//	pid.err_next = pid.err;
//	/*���ص�ǰʵ��ֵ*/
//	return pid.actual_val;
//}