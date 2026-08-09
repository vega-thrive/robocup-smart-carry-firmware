#include "main.h"


//		Mcan_Vel_Control(2, 0, 1, 0, 1);      //控制左轮转动
//		HAL_Delay(10);
//		Mcan_Vel_Control(1, 1, 1, 0, 1);      //控制右轮转动
//		HAL_Delay(10);
//		Mcan_Synchronous_motion(0); 	

void step_control()
	{
		Sensor_Get();

		LCD_DrawNumber(state,80,80,LCD_WHITE,LCD_BLACK,ALIGN_LEFT);
		

 
		   switch(state) {
				case -7:        //0000 0001
			Mcan_Vel_Control(2, 0, 0, 0, 1);      //控制左轮转动 方向为0（顺时针） 转速为每分钟0转 加速度为0（直接启动）
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 100, 0, 1);      //控制右轮转动 方向为1（逆时针） 转速为每分钟100转 加速度为0（直接启动）
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case -6:  //0000 0011
			Mcan_Vel_Control(2, 0, 20, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 100, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case -5:  //0000 0010
			Mcan_Vel_Control(2, 0, 30, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 80, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case -4:  //0000 0110
			Mcan_Vel_Control(2, 0, 40, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 80, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case -3:  //0000 0100
			Mcan_Vel_Control(2, 0, 50, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 80, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case -2:  //0000 1100
			Mcan_Vel_Control(2, 0, 70, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 80, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case -1:  //0000 1000
			Mcan_Vel_Control(2, 0, 70, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 80, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 0:  //0001 1000
			Mcan_Vel_Control(2, 0, 80, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 80, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 1:  //0001 0000
			Mcan_Vel_Control(2, 0, 80, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 70, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 2:  //0011 0000
			Mcan_Vel_Control(2, 0, 80, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 70, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 3:  //0010 0000
			Mcan_Vel_Control(2, 0, 80, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 50, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 4:  //0110 0000
			Mcan_Vel_Control(2, 0, 80, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 40, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 5:  //0100 0000
			Mcan_Vel_Control(2, 0, 80, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 30, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 6:  //1100 0000
			Mcan_Vel_Control(2, 0, 100, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 20, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;
				case 7:  //1000 0000
			Mcan_Vel_Control(2, 0, 100, 0, 1);      //控制左轮转动
			HAL_Delay(10);
			Mcan_Vel_Control(1, 1, 0, 0, 1);      //控制右轮转动
			HAL_Delay(10);
			Mcan_Synchronous_motion(0);
                break;

        }       
       HAL_Delay(10);  // 10ms延迟
}
