#include "main.h"


extern int vt_1[accel_steps]; //S型加速的速度数组
extern int vt_2[accel_steps]; //S型减速的速度数组

extern volatile int velocity[5];
extern volatile int encoder[5];

extern TIM_HandleTypeDef htim7;

extern volatile PID pid_1_L;
extern volatile PID pid_2_L;
extern volatile PID pid_3_L;
extern volatile PID pid_4_L;

extern volatile PID pid_1_V;
extern volatile PID pid_2_V;
extern volatile PID pid_3_V;
extern volatile PID pid_4_V;

//extern volatile PID pid_A;
//extern volatile float Target_Angle;
//extern volatile float Angle;

volatile int num;
volatile int time;



/**
  * @brief  定时器回调函数中断
  * @param  定时器类别
  * @retval 无
  */

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==htim7.Instance)						//5ms周期
//	{
//		Sensor_Get();

////		move_encoder();//编码器位置信息读取
////		move_velocity();//编码器速度读取
//////		HWT101_getAngle();//角度读取
////		PID_calc_L(&pid_1_L,1000000,encoder[1]);
////		if(time<1000)
////		PID_calc_V(&pid_1_V,vt_1[time/5],velocity[1]);
////		else if (time>=1000&time<3000)
////		PID_calc_V(&pid_1_V,1000,velocity[1]);
////		else if(time>=3000&time<4000)
////		PID_calc_V(&pid_1_V,vt_2[(time-3000)/5],velocity[1]);
////		else 
////		PID_calc_V(&pid_1_V,0,velocity[1]);
//////		PID_calc_V(&pid_1_V,1000,velocity[1]);

////		move_control(pid_1_V.output,0,0,0);
//////		move_control(900,0,0,0);
////		num++;
////		time = num * 5;
//		
//	}
//}

