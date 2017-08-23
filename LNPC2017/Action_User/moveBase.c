/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2017/07/24
  * @brief	 2017省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/
#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "math.h"
#include "adc.h"
#include "movebase.h"
/* Typedef ------------------------------------------------------------------------------------*/
struct areaInfo
{
	int angle;
	int distance;
}areaInfo[3][3] = {0};
/* Define -------------------------------------------------------------------------------------*/
#define PI 														3.141592f
//上下左右四条轨道的转弯提前量
#define TRACK_UP_ADVANCE 							3200.0f
#define TRACK_DOWN_ADVANCE 						1200.0f
#define	TRACK_LEFT_ADVANCE						-1000.0f
#define	TRACK_RIGHT_ADVANCE 					700.0f
//上下左右四条轨道的代号
#define TRACK_UP_NUMBER 							2
#define TRACK_DOWN_NUMBER 						0
#define	TRACK_LEFT_NUMBER							3
#define	TRACK_RIGHT_NUMBER 						1
//上下左右四条轨道的起始值
#define TRACK_UP_ORIGIN 							4300.0f
#define TRACK_DOWN_ORIGIN 						400.0f
#define	TRACK_LEFT_ORIGIN							-2000.0f
#define	TRACK_RIGHT_ORIGIN 						1900.0f
//角度闭环PID参数
#define ANGEL_PID_P 									180
#define ANGEL_PID_D 									2
//距离闭环PID参数
#define DISTANCE_PID_P 								10
#define DISTANCE_PID_D 								2
//将摄像头视野分成三行三列九个部分，每部分中心相对于车的极坐标
#define LINE0_ANGLE										-17.0f
#define LINE1_ANGLE										0.0f
#define LINE2_ANGLE										17.0f
#define ROW0_DISTANCE									25.0f
#define ROW1_DISTANCE									75.0f
#define ROW2_DISTANCE									125.0f
/* Macro --------------------------------------------------------------------------------------*/
//激光测距返回参数与距离的关系式
#define DISTANCE(x) 									(x * 0.9445f + 380.2f)
//角度数转弧度数
#define ANGLE2RAD(x) 									(x * PI / 180.0f)
//速度转脉冲
#define VEL2IMP(x) 										(x * COUNTS_PER_ROUND / (PI * WHEEL_DIAMETER))
//极坐标转直角坐标
#define POLAR2RIGHT_X(a,d)						(float)(x + d * cos(ANGLE2RAD(90 + a)))
#define POLAR2RIGHT_Y(a,d)						(float)(y + d * sin(ANGLE2RAD(90 + a)))
/* Variables ----------------------------------------------------------------------------------*/
float angle = 0.0, x = 0.0, y = 0.0;
float dir1 = 0.0, dir0 = 0.0;
float	dir1_1 = 0.0, dir1_0 = 0.0;
float targetAngle = 90.0;
float targetX = 0.0f, targetY = 0.0f;
int i = 0;
int u = 0;
int u1 = 0, u2 = 0, p1 = 0, d1 = 0, d2 = 0, p2 = 0; 
int j = 5;
int n = 0, m = 0, q = 0;
int back = 0;
float recordX = 0.0, recordY = 0.0;

float dirX = 0.0f, dirY = 0.0f, dirAngle = 0.0f;
//定位系统返回的坐标角度值
float fakeX = 0.0f, fakeY = 0.0f, fakeAngle = 0.0f;
int ifcorrect = 0;
int restart = 0;
int leftLaser = 0, rightLaser = 0;
int stopRecord = 0;
float dir2_0 = 0.0, dir2_1 = 0.0;
int mirror = 1;
int ifMirror = 1;

int startReceive = 0;
int endReceive = 0xc9;
//int ball[3] = {0,0,0};
int turnLock = 0;
int ball[50][2] = {0};
int directionBallNum[9] = {0};
int ballAmount = 0;
int max = 0;
int arrive = 0;

int tim = 0,r = 0;
/* Function prototypes ------------------------------------------------------------------------*/
/* Functions ----------------------------------------------------------------------------------*/






float GetPosx(void)
{
	return fakeX;
}

float GetPosy(void)
{
	return fakeY;
}


float GetAngleZ(void)
{
	return fakeAngle;
}




int IfMirror(int x)
{
	 if(x == 1)
	 {
		if(Get_Adc_Average(ADC_Channel_15,100) < 1000)
		{	 
			ifMirror = 0;	
			return -1;
		} 
		if(Get_Adc_Average(ADC_Channel_14,100) < 1000)
		{
			ifMirror = 0;
			return 1;			
		}	
	}
}
void SquareClosed_LoopNew()
{
	if(fabs(y - 0.0f) <= 10.0f && j == 5)
	{
		i = 0;
		j = 2;
	}
	//if(fabs(y - 1100.0f - 400 * n) <= 20.0f && j == 1)
	if((y < (1200.0f + 400 * n)) && j == 1)
	{
		i = 0;
		j++;
		n++;
		if(n == 4)
		{
			n = 1;
			back = 1;
			j = 6;
		}			
	}
	//if(fabs(x - 700.0f + 300 * n) <= 20.0f && j == 2)
	if(( x > (700.0f - 300 * n)) && j == 2)
	{
		i = 1;
		j++;
	}	
	//if(fabs(y - 3400.0f + 300 * n) <= 20.0f && j == 3)
	if((y > (3400.0f - 300 * n)) && j == 3)
	{
		i = 2;
		j++;
	}	
	//if((fabs(x + 1200.0f - 350 * n) <= 20.0f) && j == 4 && (fabs(x) > 100.0f))
	if(( x < (-1000.0f + 400 * n)) && j == 4 && (fabs(x) > 100.0f))
	{
		i = 3;
		j = 1;
	}	
	if(restart == 1)
	{
			i = 0;
			j = 2;
			n = 0;
			restart = 0;
	}
	switch(i)
	{
		case 0:
			dir1_1 = 400.0f + 400 * n - y;
			targetAngle = -90.0f ;
			break;
		case 1:
			dir1_1 = mirror * (1900.0f - 300 * n) - x;
			targetAngle = 0.0f;
			back = 0;
			break;
		case 2:
			dir1_1 = 4400.0f - 300 * n - y;
			targetAngle = 90.0f ;
			break;
		case 3:
			dir1_1 = mirror * (-2000.0f + 400 * n) - x;
			targetAngle = 180.0f;
			break;			
		case 4:
			dir1_1 = 0.0f - y;
			targetAngle = 0.0f;
			break;
	}
	dir1 = targetAngle - angle;		
	if(dir1 >= 180.0f)	
	{
		dir1 -= 360.0f;
	}
	else if(dir1 <= -180.0f)
	{
		dir1 += 360.0f;
	}
	dir2_1 = 0.0f - x;
	u = 180 * dir1 + 2 * (dir1 - dir0);
	u1 = 10 * dir1_1 + 2 * (dir1_1 - dir1_0);
	u2 = 10 * dir2_1 + 2 * (dir2_1 - dir2_0);
	dir0 = dir1;
	dir1_0 = dir1_1;
	dir2_0 = dir2_1;
	if(fabs(dir1_1) < 2.0f)
	{
		u1 = 0;
	}
	if(i == 1 || i == 2)
	{
		VelCrl(CAN1, 1, VEL2IMP(500) + u - u1);//you
		VelCrl(CAN1, 2, -VEL2IMP(500) + u - u1);
	}
	if(i == 0 || i == 3)
	{
		VelCrl(CAN1, 1, VEL2IMP(500) + u + u1);//you
		VelCrl(CAN1, 2, -VEL2IMP(500) + u + u1);
	}	
	if(back == 1 && fabs(y - 1300.0f) < 50.0f)
	{
		i = 4;
	}
	if(i == 4)
	{
		if(fabs(angle) > 0.5f)
		{
			VelCrl(CAN1, 1, -VEL2IMP(500) + u + u1 + u2);//you
			VelCrl(CAN1, 2, VEL2IMP(500) + u - u1 + u2);
		}	
		else
		{
			VelCrl(CAN1, 1, -VEL2IMP(500) + u1 + u2);//you
			VelCrl(CAN1, 2, VEL2IMP(500) - u1 + u2);
		}
		if(fabs(y - recordY) < 0.001f)
		{
			m = 0;
			stopRecord++;
			if(stopRecord == 10)
			{
				ifcorrect = 1;
				stopRecord = 0;
			}	
			//restart = 1;
		}
	}
	rightLaser = Get_Adc_Average(ADC_Channel_14,100);
	leftLaser = Get_Adc_Average(ADC_Channel_15,100);
	if(ifcorrect == 1)
	{
		dirAngle = fakeAngle;
		dirY = -fakeX * sinf(dirAngle / 180.0f * 3.141592f) + fakeY * cosf(dirAngle / 180.0f * 3.141592f);
		dirX = fakeX * cosf(dirAngle / 180.0f * 3.141592f) + fakeY * sinf(dirAngle / 180.0f * 3.141592f) - (Get_Adc_Average(ADC_Channel_15,100) * 0.9445f + 380.2f - 2400.0f);
		ifcorrect = 0;
		restart = 1;
	}
	if(fabs(x - recordX) < 0.001f && fabs(y - recordY) < 0.001f)
	{
		m++;
	}
	else
	{
		m = 0;
		recordX = x;
		recordY = y;
	}
	//USART_OUT(USART1, (uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", (int)angle, (int)x, (int)y, (int)i, (int)leftLaser, (int)rightLaser, (int)dirX, (int)dirY, (int)dirAngle, (int)fakeAngle, (int)fakeX, (int)fakeY);	
		USART_OUT(USART1, (uint8_t*)"%d\t%d\t%d\r\n", (int)fakeAngle, (int)fakeX, (int)fakeY);	
}

int AnglePID(int p, int d, float targetAngle)
{
	dir1 = targetAngle - angle;
	if(dir1 >= 180.0f)	
	{
		dir1 -= 360.0f;
	}
	else if(dir1 <= -180.0f)
	{
		dir1 += 360.0f;
	}
	u = p * dir1 + d * (dir1 - dir0);
	dir0 = dir1;
	return u;
}
int LinePID(int p, int d, float target, float x, float y)
{
	dir1_1 = target - x - y;;
	if(fabs(dir1_1) < 2.0f)
	{
		u1 = 0;
	}
	u1 = p * dir1_1 + d * (dir1_1 - dir1_0);
	dir1_0 = dir1_1;
	return u1;
}
void TrackMovingX(float originTrack, float trackDistance, int trackNumber, int n, float targetAngle, int xP, int xD, int angleP, int angleD)
{
	int u = 0, u1 = 0;
	float targetX = 0;
	targetX = originTrack + trackDistance * n;
	u = AnglePID(angleP, angleD, targetAngle);
	u1 = LinePID(xP, xD, targetX, x, 0);
	if(trackNumber == 1)
	{
		VelCrl(CAN1, 1, 8192 + 4096 + 2048 * 3 + u - u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 - 2048 * 3 + u - u1);
	}
	else
	{
		VelCrl(CAN1, 1, 8192 + 4096 + 2048 * 3 + u + u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 - 2048 * 3 + u + u1);
	}
}

void TrackMovingY(float originTrack, float trackDistance, int trackNumber, int n, float targetAngle, int yP, int yD, int angleP, int angleD)
{
	int u = 0, u1 = 0;
	float targetY = 0;
	targetY = originTrack + trackDistance * n;
	u = AnglePID(angleP, angleD, targetAngle);
	u1 = LinePID(yP, yD, targetY, 0, y);
	if(trackNumber == 2)
	{
		VelCrl(CAN1, 1, 8192 + 4096 + 2048 * 3 + u - u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 - 2048 * 3 + u - u1);
	}
	else
	{
		VelCrl(CAN1, 1, 8192 + 4096 + 2048 * 3 + u + u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 - 2048 * 3 + u + u1);
	}
}

////未完成
//void F4MoveBase(void)
//{
//	if(fabs(y) < 10.0f && j == 5)
//	{
//		TrackMovingY(700.0f, 720.0f, TRACK_DOWN_NUMBER, n, -90.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
//	}
//	if(x > 300.0f && (j == 5 || j == 1))
//	{
//		TrackMovingX(1000.0f, 700.0f, TRACK_RIGHT_NUMBER, n, 0.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
//		j = 1;
//	}
//	if(y > 3100.0f && (j == 1 || j == 2))
//	{
//		TrackMovingY(3800.0f, 700.0f, TRACK_UP_NUMBER, n, 90.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
//		j = 2;
//	}
//	if(x < -300.0f && (j == 2 || j == 3))
//	{
//		TrackMovingX(-1000.0f, 700.0f, TRACK_LEFT_NUMBER, n, 180.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
//		j = 3;
//	}
//}
//NewNew增加了避障功能
void SquareClosed_LoopNewNew(void)
{
	if(m < 15)
	{
		//UseCamera();
		SquareClosed_LoopNew();
	}
	else
	{
		if(q <= 150)
		{
			VelCrl(CAN1, 1, -8192);
			VelCrl(CAN1, 2, 8192);
		}
		else if(q > 150 && q <= 250)
		{
			if(fabs(x) <= 1500.0f && y >= 900.0f && y <= 3900.0f)
			{
				VelCrl(CAN1, 1, 4096);
				VelCrl(CAN1, 2, -8192);
			}
			else
			{
				VelCrl(CAN1, 1, 8192);
				VelCrl(CAN1, 2, -4096);
			}
		}
		q++;
		if(q > 250)
		{
				m = 0;
				q = 0;
		}
		USART_OUT(USART1, (uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", (int)angle, (int)x, (int)y, (int)i, (int)leftLaser, (int)rightLaser, (int)dirX, (int)dirY, (int)dirAngle);	
	}
}
void LNPCSTANDARD()
{
	if(fabs(y - 0.0f) <= 10.0f && j == 5)
	{
		i = 0;
		j = 2;
	}
	if((y < TRACK_DOWN_ADVANCE + 400 * n) && j == 1)
	{
		i = 0;
		j++;
		n++;
		if(n == 4)
		{
			n = 1;
			back = 1;
			j = 6;
		}			
	}
	if((x > (TRACK_RIGHT_ADVANCE - 300 * n)) && j == 2)
	{
		i = 1;
		j++;
	}	
	if((y > (TRACK_UP_ADVANCE - 300 * n)) && j == 3)
	{
		i = 2;
		j++;
	}	
	if((x < TRACK_LEFT_ADVANCE + 400 * n) && j == 4 && (fabs(x) > 100.0f))
	{
		i = 3;
		j = 1;
	}	
	if(restart == 1)
	{
			i = 0;
			j = 2;
			n = 0;
			restart = 0;
	}
	switch(i)
	{
		case 0:
			TrackMovingY(TRACK_DOWN_ORIGIN, 400.0f, TRACK_DOWN_NUMBER, n, -90.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
			break;
		case 1:
			TrackMovingX(TRACK_RIGHT_ORIGIN, -300.0f, TRACK_RIGHT_NUMBER, n, 0.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
			back = 0;
			break;
		case 2:
			TrackMovingY(TRACK_UP_ORIGIN, -300.0f, TRACK_UP_NUMBER, n, 90.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
			break;
		case 3:
			TrackMovingX(TRACK_LEFT_ORIGIN, 400.0f, TRACK_LEFT_NUMBER, n, 180.0f, DISTANCE_PID_P, DISTANCE_PID_D, ANGEL_PID_P, ANGEL_PID_D);
			break;			
		case 4:
			dir1_1 = 0.0f - y;
			targetAngle = 0.0f;
			break;
	}
	dir2_1 = 0.0f - x;
	u2 = 10 * dir2_1 + 2 * (dir2_1 - dir2_0);
	dir2_0 = dir2_1;
	if(back == 1 && fabs(y - 1300.0f) < 50.0f)
	{
		i = 4;
	}
	if(i == 4)
	{
		if(fabs(angle) > 0.5f)
		{
			VelCrl(CAN1, 1, -4096 + u + u1 + u2);//you
			VelCrl(CAN1, 2, 4096 + u - u1 + u2);
		}	
		else
		{
			VelCrl(CAN1, 1, -4096 + u1 + u2);//you
			VelCrl(CAN1, 2, 4096 - u1 + u2);
		}
		if(fabs(y - recordY) < 0.001f)
		{
			m = 0;
			stopRecord++;
			if(stopRecord == 10)
			{
				ifcorrect = 1;
				stopRecord = 0;
			}	
			//restart = 1;
		}
	}
	rightLaser = Get_Adc_Average(ADC_Channel_14,100);
	leftLaser = Get_Adc_Average(ADC_Channel_15,100);
	if(ifcorrect == 1)
	{
		dirAngle = fakeAngle;
		dirY = -fakeX * sinf(ANGLE2RAD(dirAngle)) + fakeY * cosf(ANGLE2RAD(dirAngle));
		dirX = fakeX * cosf(ANGLE2RAD(dirAngle)) + fakeY * sinf(ANGLE2RAD(dirAngle)) - (Get_Adc_Average(ADC_Channel_15,100) * 0.9445f + 380.2f - 2400.0f);
		ifcorrect = 0;
		restart = 1;
	}
	if(fabs(x - recordX) < 0.001f && fabs(y - recordY) < 0.001f)
	{
		m++;
	}
	else
	{
		m = 0;
		recordX = x;
		recordY = y;
	}
	USART_OUT(USART1, (uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", (int)angle, (int)x, (int)y, (int)i, (int)leftLaser, (int)rightLaser, (int)dirX, (int)dirY, (int)dirAngle, (int)fakeAngle, (int)fakeX, (int)fakeY);	
}
//由内向外扫场
void LNPC_Plan_A()
{
	if(fabs(y - 0.0) <= 15.0 && j == 5)
	{
		i = 0;
		j = 2;
	}
	if(fabs(x - 300.0 - 400 * n) <= 15.0 && j == 1)
	{
		i = 0;
		j++;
		n++;
		if(n == 4)
		{
			n = 1;
			back = 1;
			j = 6;
		}			
	}
	if(fabs(y - 2600.0 - 400 * n) <= 15.0 && j == 2)
	{
		i = 1;
		j++;
	}	
	if(fabs(x + 300.0 + 400 * n) <= 15.0 && j == 3)
	{
		i = 2;
		j++;
	}	
	if((fabs(y - 2000.0 + 400 * n) <= 15.0) && j == 4 && (fabs(x) > 100.0))
	{
		i = 3;
		j = 1;
	}	
	switch(i)
	{
		case 0:
			dir1_1 = 600.0f + 400.0f * n - x;
			targetAngle = 0.0f;
			break;
		case 1:
			dir1_1 = 3300.0f + 400.0f * n - y;
			targetAngle = 90.0f;
			break;
		case 2:
			dir1_1 = -600.0f - 400.0f * n - x;
			targetAngle = 180.0f;
			break;
		case 3:
			dir1_1 = 1400.0f - 400.0f * n - y;
			targetAngle = -90.0f;
			break;			
		case 4:
			dir1_1 = 0.0f - y;
			targetAngle = 0.0f;
			break;
	}
	dir1 = targetAngle - angle;		
	if(dir1 >= 180)	
	{
		dir1 -= 360;
	}
	else if(dir1 <= -180)
	{
		dir1 += 360;
	}
	u = 100 * dir1 + 2 * (dir1 - dir0);
	u1 = 6 * dir1_1 + 2 * (dir1_1 - dir1_0);
	dir0 = dir1;
	dir1_0 = dir1_1;
	if(fabs(dir1_1) < 2.0f)
	{
		u1 = 0;
	}
	if(i == 0 || i == 3)
	{
		VelCrl(CAN1, 1, 8192 + 4096 * 1 + u - u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 * 1 + u - u1);
	}
	if(i == 1 || i == 2)
	{
		VelCrl(CAN1, 1, 8192 + 4096 * 1 + u + u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 * 1 + u + u1);
	}	
	if(back == 1 && fabs(y - 1000.0) < 50.0f)
	{
		i = 4;
	}
	if(i == 4)
	{
		if(fabs(angle) > 0.5f)
		{
			VelCrl(CAN1, 1,  u + u1);//you
			VelCrl(CAN1, 2,  u - u1);
		}	
		else
		{
			VelCrl(CAN1, 1,  u1);//you
			VelCrl(CAN1, 2, -u1);
		}
		if(fabs(y) < 0.01f)
		{
			m = 0;
		}
	}
	if(fabs(x - recordX) < 0.001f && fabs(y - recordY) < 0.001f)
	{
		m++;
	}
	else
	{
		m = 0;
		recordX = x;
		recordY = y;
	}
	USART_OUT(USART1, (uint8_t*)"%d\t%d\r\n", (int)x, (int)y);	
}
//由外向内扫场，轨迹有所调整，不是以前的正方形
void LNPC_Plan_B()
{
	if(fabs(y - 0.0) <= 10.0 && j == 5)
	{
		i = 0;
		j = 2;
	}
	if(fabs(y - 850.0 - 400 * n) <= 20.0 && j == 1)
	{
		i = 0;
		j++;
		n++;
		if(n == 4)
		{
			n = 1;
			back = 1;
			j = 6;
		}			
	}
	if(fabs(x - 1100.0 + 400 * n) <= 20.0 && j == 2)
	{
		i = 1;
		j++;
	}	
	if(fabs(y - 3850.0 + 400 * n) <= 20.0 && j == 3)
	{
		i = 2;
		j++;
	}	
	if((fabs(x + 1600.0 - 400 * n) <= 20.0) && j == 4 && (fabs(x) > 100.0))
	{
		i = 3;
		j = 1;
	}	
	switch(i)
	{
		case 0:
			dir1_1 = 400.0 + 400 * n - y;
			targetAngle = -90.0;
			break;
		case 1:
			dir1_1 = 2000.0 - 400 * n - x;
			targetAngle = 0.0;
			break;
		case 2:
			dir1_1 = 4400.0 - 400 * n - y;
			targetAngle = 90.0;
			break;
		case 3:
			dir1_1 = -2000.0 + 400 * n - x;
			targetAngle = 180.0;
			break;			
		case 4:
			dir1_1 = 0.0 - y;
			targetAngle = 0.0;
			break;
	}
	dir1 = targetAngle - angle;		
	if(dir1 >= 180)	
	{
		dir1 -= 360;
	}
	else if(dir1 <= -180)
	{
		dir1 += 360;
	}
	u = 160 * dir1 + 2 * (dir1 - dir0);
	u1 = 5 * dir1_1 + 2 * (dir1_1 - dir1_0);
	dir0 = dir1;
	dir1_0 = dir1_1;
	if(fabs(dir1_1) < 2.0f)
	{
		u1 = 0;
	}
	if(i == 1 || i == 2)
	{
		VelCrl(CAN1, 1, 8192 + 4096 * 2 + u - u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 * 2 + u - u1);
	}
	if(i == 0 || i == 3)
	{
		VelCrl(CAN1, 1, 8192 + 4096 * 2 + u + u1);//you
		VelCrl(CAN1, 2, -8192 - 4096 * 2 + u + u1);
	}	
	if(back == 1 && fabs(y - 1000.0) < 50.0f)
	{
		i = 4;
	}
	if(i == 4)
	{
		if(fabs(angle) > 0.5f)
		{
			VelCrl(CAN1, 1,  u + u1);//you
			VelCrl(CAN1, 2,  u - u1);
		}	
		else
		{
			VelCrl(CAN1, 1,  u1);//you
			VelCrl(CAN1, 2, -u1);
		}
		if(fabs(y) < 0.01f)
		{
			m = 0;
		}
	}
	if(fabs(x - recordX) < 0.000001f && fabs(y - recordY) < 0.000001f)
	{
		m++;
	}
	else
	{
		m = 0;
		recordX = x;
		recordY = y;
	}
	USART_OUT(USART1, (uint8_t*)"%d\t%d\r\n", (int)x, (int)y);	
}

//顺时针为1，逆时针为0
void GivenCircle(int vel,float targetX,float targetY,float r,int direction)
{
	float R = 0.0f, impulse = 0.0f, impulse1 = 0.0f, impulse2 = 0.0f;
	float targetAngle = 0.0f, dir1 = 0.0f, dir0 = 0.0f,dir1_1 = 0.0f, dir1_0 = 0.0f;
	int u = 0, u1 = 0;
	if (direction == 1) 
	{
		if(atan2((y - targetY) , (x - targetX)) <= 0.0f)
		{
			targetAngle = atan2((y - targetY) , (x - targetX)) / 3.141592f * 180 - 180;
		}
		else
		{
			targetAngle = atan2((y - targetY) , (x - targetX)) / 3.141592f * 180 + 180;
		}	
	}
	else
	{
		targetAngle = atan2((y - targetY) , (x - targetX)) / 3.141592f * 180;
	}
	dir1 = targetAngle - angle;
	if(dir1 > 180)
	{
		dir1 -= 360;
	}
	else if(dir1 < -180)
	{
		dir1 += 360;
	}
	u = 60 * dir1 + 2 * (dir1 - dir0);
	R = (float)sqrt((x - targetX) * (x - targetX) + (y - targetY) * (y - targetY));
	dir1_1 = r - R;
	u1 = 5 * dir1_1 + 2 * (dir1_1 - dir1_0);
	impulse = vel * COUNTS_PER_ROUND / (PI * WHEEL_DIAMETER); 
	impulse1 = impulse * (r + WHEEL_TREAD/2) / r;
	impulse2 = impulse * (r - WHEEL_TREAD/2) / r;
	if(direction == 1)
	{
		VelCrl(CAN1, 1, (int)impulse2 + u + u1);
		VelCrl(CAN1, 2, -(int)impulse1 + u + u1);
	}
	else
	{
		VelCrl(CAN1, 1, (int)impulse1 + u - u1);
		VelCrl(CAN1, 2, -(int)impulse2 + u - u1);
	}
	dir1 = dir0;
	dir1_1 = dir1_0;
}
void GivenPoint(float targetX, float targetY)
{
	float k = 0.0f, R = 0.0f;
	if(fakeX != targetX)
	{	
		k = (targetY - fakeY) / (targetX - fakeX);
		if(k > 0.0f)
		{	
			if(fakeX > targetX)
			{
				targetAngle = atan(k) * 180 / PI + 90.0f;
			}
			else if(fakeX < targetX)
			{
				targetAngle = atan(k) * 180 / PI - 90.0f;
			}
		}
		else if(k < 0.0f)
		{
			if(fakeX > targetX)
			{
				targetAngle = atan(k) * 180 / PI + 90.0f;
			}
			else if(fakeX < targetX)
			{
				targetAngle = atan(k) * 180 / PI - 90.0f;
			}
		}
		else if(fabs(k) < 0.001f)
		{
			if(fakeX > targetX)
			{
				targetAngle = 90.0f;
			}
			else
			{
				targetAngle = -90.0f;
			}
		}
	}	
	else
	{
		if(fakeY > targetY)
		{
			targetAngle = 180.0f;
		}
		else if(fakeY < targetY)
		{
			targetAngle = 0.0f;
		}
	}
	//若是运用atan2()函数来求targetAngle
	//if(x > targetX && y >= targetY)
	//{
	//	targetAngle = 90 + atan2((y - targetY),(x - targetX)) / 3.141592 * 180;
	//}
	//else if(x <= targetX && y > targetY)
	//{
	//	targetAngle = 270 - atan2((y - targetY),(x - targetX)) / 3.141592 * 180;
	//}
	//else if(x < targetX && y <= targetY)
	//{
	//	targetAngle = 90 + atan2((y - targetY),(x - targetX)) / 3.141592 * 180;
	//}
	//else if(x >= targetX && y < targetY)
	//{
	//	targetAngle = 90 + atan2((y - targetY),(x - targetX)) / 3.141592 * 180;
	//}
	R = (float)sqrt((fakeX - targetX) * (fakeX - targetX) + (fakeY - targetY) * (fakeY - targetY));
	dir1_1 = 0.0f - (float)sqrt((fakeX - targetX) * (fakeX - targetX) + (fakeY - targetY) * (fakeY - targetY));
	u1 = 2 * dir1_1 + 2 * (dir1_1 - dir1_0);				
	if(fabs(fakeAngle - targetAngle) > 2.0f)
	{
		dir1 = targetAngle - fakeAngle;
		if(dir1 > 180)
		{
			dir1 -= 360;
		}
		else if(dir1 < -180)
		{
			dir1 += 360;
		}
		u = 200 * dir1 + 2 * (dir1 - dir0);
		dir0 = dir1;	
	}
	else
	{		
		u = 0;
	}	
	if(R <= 80)
	{
		VelCrl(CAN1, 1, VEL2IMP(1000));
		VelCrl(CAN1, 2, -VEL2IMP(1000));
		arrive = 1;
	}
	else
	{
		VelCrl(CAN1, 1,VEL2IMP(1000) + u - u1);
		VelCrl(CAN1, 2,-VEL2IMP(1000) + u + u1);
	}
}

float K2Angle(float point1X, float point1Y, float point2X, float point2Y)
{
	float k = 0;
	float returnAngle = 0;
	if(point1X != point2X)
	{
		k = (point2Y - point1Y) / (point2X - point1X);
		returnAngle = atan(k);
	}
	else
	{
		returnAngle = PI / 2;
	}
	return returnAngle;
}

int RealX(int error)
{
	float realX = 0.0f;
	int wall = 1;
	if(DISTANCE(leftLaser) < (2400.0f - error))
	{
		if(DISTANCE(rightLaser) > (2400.0f - error))
		{
			realX = 2400.0f - DISTANCE(rightLaser);
		}
		else
			;
	}
}

int ChangeWall()
{
	
}

float DistancePoint2Line(int lineAngle, int8_t pointAngle, int pointDistance)
{
	float distance = 0.0f;
	distance = 10 * pointDistance * sin(ANGLE2RAD(abs(lineAngle - pointAngle)));
	return distance;
}
//给定方向哪个球最多
void UseCamera()
{
	if(ball[0] > ball[1] && ball[0] > ball[2])
	{
		u1 = 2048 + 1024 * 2;
	}
	else if(ball[2] > ball[1] && ball[2] > ball[0])
	{
		u1 = -2048 - 1024 * 2;
	}
	else if(ball[1] > ball[0] && ball[1] > ball[2])
	{
		u1 = 0;
	}	
	if(fabs(y - 0.0f) <= 10.0f && j == 5)
	{
		i = 0;
		j = 2;
		turnLock = 1;
	}
	//if((y < 700.0f) && j == 1)
	if((y < 700.0f + 700.0f * n) && j == 1)
	{
		i = 0;
		j++;
		if(n == 0)
		{
			n = 1;
		}
		else
		{
			back = 1;
			j = 6;
		}
		turnLock = 1;
	}
	//if((x > 1700.0f) && j == 2)
	if((x > 1000.0f - 700.0f * n) && j == 2)
	{
		i = 1;
		j++;
		turnLock = 1;
	}	
	//if((y > 4100.0f) && j == 3)
	if((y > 3800.0f - 700.0f * n) && j == 3)
	{
		i = 2;
		j++;
		turnLock = 1;
	}	
	//if((x < -1700.0f) && j == 4 && (fabs(x) > 100.0f))
	if((x < -1000.0f + 700.0f * n) && j == 4 && (fabs(x) > 100.0f))
	{
		i = 3;
		j = 1;
		turnLock = 1;
	}	
	if(j == 6 && fabs(x) < 20.0f)
	{
		i = 4;
		turnLock = 1;
	}
	switch(i)
	{
		case 0:
			targetAngle = -90.0f;
			break;
		case 1:
			targetAngle = 0.0f;
			break;
		case 2:
			targetAngle = 90.0f;
			break;
		case 3:
			targetAngle = 180.0f;
			break;		
		case 4:
			targetAngle = 0.0f;
			break;
	}
	dir1 = targetAngle - angle;		
	if(dir1 >= 180.0f)	
	{
		dir1 -= 360.0f;
	}
	else if(dir1 <= -180.0f)
	{
		dir1 += 360.0f;
	}
	u = 180 * dir1 + 2 * (dir1 - dir0);
	dir0 = dir1;
	if(fabs(angle - targetAngle) < 5.0f)
	{
		turnLock = 0;
	}
	if(turnLock == 0 && fabs(dir1) < 90.0f)
	{
		u = 0;
	}
	if(turnLock == 1)
	{
		u1 = 0;
	}
	if(i != 4)
	{
		VelCrl(CAN1, 1, VEL2IMP(500) + u + u1);//you
		VelCrl(CAN1, 2, -1 * VEL2IMP(500) + u + u1);
	}
	else
	{
		if(fabs(angle) > 0.5f)
		{
			VelCrl(CAN1, 1, -1 * VEL2IMP(500) + u);//you
			VelCrl(CAN1, 2, VEL2IMP(500) + u);
		}	
		if(fabs(y - 0.0f) < 0.001f)
		{
			m = 0;
			stopRecord++;
			if(stopRecord == 10)
			{
				ifcorrect = 1;
				stopRecord = 0;
			}		
		}
		rightLaser = Get_Adc_Average(ADC_Channel_14,100);
		leftLaser = Get_Adc_Average(ADC_Channel_15,100);
		if(ifcorrect == 1)
		{
			dirAngle = fakeAngle;
			dirY = -fakeX * sinf(dirAngle / 180.0f * 3.141592f) + fakeY * cosf(dirAngle / 180.0f * 3.141592f);
			dirX = fakeX * cosf(dirAngle / 180.0f * 3.141592f) + fakeY * sinf(dirAngle / 180.0f * 3.141592f) - (Get_Adc_Average(ADC_Channel_15,100) * 0.9445f + 380.2f - 2400.0f);
			ifcorrect = 0;
			j = 5;
		}
	}	
	if(fabs(x - recordX) < 0.001f && fabs(y - recordY) < 0.001f)
	{
		m++;
	}
	else
	{
		m = 0;
		recordX = x;
		recordY = y;
	}
	//USART_OUT(USART1, (uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", (int)angle, (int)x, (int)y, (int)i, (int)leftLaser, (int)rightLaser, (int)dirX, (int)dirY, (int)dirAngle, (int)fakeAngle, (int)fakeX, (int)fakeY);	
		USART_OUT(USART1, (uint8_t*)"%d\t%d\t%d\r\n", (int)fakeAngle, (int)fakeX, (int)fakeY);	
}
void AnotherUseCamera()
{
	static int i = 0, j = 0, k = 0;
	int ballNum = 0;
	int scanAngle = 0;
//	USART_OUT(USART1, (uint8_t*)"%x\t\r\n", (int)startReceive);
//	for(ballNum = 0; ballNum < ballAmount; ballNum++)
//	{
//		USART_OUT(USART1, (uint8_t*)"%d\t%d\r\n", (int8_t)ball[ballNum][0], (int)ball[ballNum][1]);
//	}
	for(scanAngle = -20; scanAngle < 25; scanAngle += 5)
	{
		for(ballNum = 0; ballNum < ballAmount; ballNum++)
		{
			if(DistancePoint2Line(scanAngle, ball[ballNum][0], ball[ballNum][1]) < (MOVEBASE_WIDTH / 2))
			{
				directionBallNum[i]++;
			}
		}
		i++;
	}
	for(i = 0; i < 9; i++)
	{
		if(directionBallNum[i] > directionBallNum[max])
		{
			max = i;
		}
	}
	targetAngle = 5.0f * max - 20.0f;
	if(targetAngle > 180.0f)
	{
		targetAngle -= 360.0f;
	}
	if(targetAngle < -180.0f)
	{
		targetAngle += 360.0f;
	}
	for(k = 0; k < 9; k++)
	{
		directionBallNum[k] = 0;
	}
	USART_OUT(USART1, (uint8_t*)"%d\t\r\n", (int)targetAngle);
}
void BroChenUseCamera()
{
	int i = 0, ballNum = 0, k = 0;
	int scanAngle = 0;
	int testValue1 = 0;
	int targetDirection[51] = {0};
	for(k = 0;k<51;k++)
	{
		targetDirection[k] = 0;
	}
	for(ballNum = 0; ballNum < ballAmount; ballNum++)
	{
		for(i = 0; i < 51; i++)
		{
			scanAngle = i - 25;
			if(DistancePoint2Line(scanAngle, ball[ballNum][0], ball[ballNum][1]) < (MOVEBASE_WIDTH / 2))
			{
				testValue1 = DistancePoint2Line(scanAngle, ball[ballNum][0], ball[ballNum][1]);
				targetDirection[i]++;
			}
		}
	}	
	for(k = 0; k < 51; k++)
	{
		if(directionBallNum[k] > directionBallNum[max])
		{
			max = k;
		}
	}
	targetAngle = k - 25;
	if(targetAngle > 180.0f)
	{
		targetAngle -= 360.0f;
	}
	if(targetAngle < -180.0f)
	{
		targetAngle += 360.0f;
	}
}
void YaoyiyaoUseCamera()
{
	int areaBallAmount[3][3] = {0};
	int route[3] = {0};
	int ballNum = 0;
	int i = 0, j = 0, ifDone = 0;
	
	areaInfo[0][0].angle = LINE0_ANGLE;
	areaInfo[1][0].angle = LINE0_ANGLE;
	areaInfo[2][0].angle = LINE0_ANGLE;
	areaInfo[0][1].angle = LINE1_ANGLE;
	areaInfo[1][1].angle = LINE1_ANGLE;
	areaInfo[2][1].angle = LINE1_ANGLE;
	areaInfo[0][2].angle = LINE2_ANGLE;
	areaInfo[1][2].angle = LINE2_ANGLE;
	areaInfo[2][2].angle = LINE2_ANGLE;
	
	areaInfo[0][0].distance = ROW0_DISTANCE;
	areaInfo[0][1].distance = ROW0_DISTANCE;
	areaInfo[0][2].distance = ROW0_DISTANCE;
	areaInfo[1][0].distance = ROW1_DISTANCE;
	areaInfo[1][1].distance = ROW1_DISTANCE;
	areaInfo[1][2].distance = ROW1_DISTANCE;
	areaInfo[2][0].distance = ROW2_DISTANCE;
	areaInfo[2][1].distance = ROW2_DISTANCE;
	areaInfo[2][2].distance = ROW2_DISTANCE;
	
//	struct route
//	{
//		int row1;
//		int row2;
//		int row3;
//		int ballNumber;
//	}walkRoute[];
	
	struct chosenBallArea
	{
		int maxBallAmount[3];
		struct chosenArea
		{
			int row;
			int line;
		}chosenArea[3];
		struct location
		{
			float angle;
			float distance;
		}location[3];
	}chosenBallArea={0};

	//if(ifDone == 1) 
	for(ballNum = 0; ballNum < ballAmount; ballNum++)
	{
		if(ball[ballNum][0] >= -25 && ball[ballNum][0] < -8)
		{
			if(ball[ballNum][1] >= 0 && ball[ballNum][1] < 50)
			{
				areaBallAmount[0][0]++;
			}
			else if(ball[ballNum][1] >= 50 && ball[ballNum][1] < 100)
			{
				areaBallAmount[1][0]++;
			}
			else if(ball[ballNum][1] >= 100 && ball[ballNum][1] < 150)
			{
				areaBallAmount[2][0]++;
			}
		}
		else if(ball[ballNum][0] >= -8 && ball[ballNum][0] < 8)
		{
			if(ball[ballNum][1] >= 0 && ball[ballNum][1] < 50)
			{
				areaBallAmount[0][1]++;
			}
			else if(ball[ballNum][1] >= 50 && ball[ballNum][1] < 100)
			{
				areaBallAmount[1][1]++;
			}
			else if(ball[ballNum][1] >= 100 && ball[ballNum][1] < 150)
			{
				areaBallAmount[2][1]++;
			}
		}
		else if(ball[ballNum][0] >= 8 && ball[ballNum][0] <= 25)
		{
			if(ball[ballNum][1] >= 0 && ball[ballNum][1] < 50)
			{
				areaBallAmount[0][2]++;
			}
			else if(ball[ballNum][1] >= 50 && ball[ballNum][1] < 100)
			{
				areaBallAmount[1][2]++;
			}
			else if(ball[ballNum][1] >= 100 && ball[ballNum][1] < 150)
			{
				areaBallAmount[2][2]++;
			}
		}
	}
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 3; j++)
		{
			if(areaBallAmount[i][j] > chosenBallArea.maxBallAmount[i])
			{
				chosenBallArea.maxBallAmount[i] = areaBallAmount[i][j];
				chosenBallArea.chosenArea[i].line = j;
				chosenBallArea.chosenArea[i].row = i;
			}
		}
	}
	chosenBallArea.location[0].angle = areaInfo[chosenBallArea.chosenArea[0].row][chosenBallArea.chosenArea[0].line].angle;
	chosenBallArea.location[1].angle = areaInfo[chosenBallArea.chosenArea[1].row][chosenBallArea.chosenArea[1].line].angle;
	chosenBallArea.location[2].angle = areaInfo[chosenBallArea.chosenArea[2].row][chosenBallArea.chosenArea[2].line].angle;
	chosenBallArea.location[0].distance = areaInfo[chosenBallArea.chosenArea[0].row][chosenBallArea.chosenArea[0].line].distance;
	chosenBallArea.location[1].distance = areaInfo[chosenBallArea.chosenArea[1].row][chosenBallArea.chosenArea[1].line].distance;
	chosenBallArea.location[2].distance = areaInfo[chosenBallArea.chosenArea[2].row][chosenBallArea.chosenArea[2].line].distance;
	while(i < 3)
	{
		if(chosenBallArea.maxBallAmount[i] != 0)
		{
			GivenPoint(POLAR2RIGHT_X(chosenBallArea.location[i].angle,chosenBallArea.location[i].distance),POLAR2RIGHT_Y(chosenBallArea.location[i].angle,chosenBallArea.location[i].distance));
			if(arrive == 1)
			{
				i++;
				arrive = 0;
			}
		}
		else
		{
			i++;
		}
	}
	if(i == 3)
	{
		dir1 = 0.0f - angle;
		dir1_1 = 0.0f - x;
		u = 160 * dir1 + 2 * (dir1 - dir0);
		u1 = 10 * dir1_1 + 2 * (dir1_1 - dir1_0);
		dir0 = dir1;
		dir1_0 = dir1_1;
		VelCrl(CAN1, 1,VEL2IMP(500) + u - u1);
		VelCrl(CAN1, 2,VEL2IMP(500) + u + u1);
		//接扫场
	}
}
void C0Connection(float point1X, float point1Y, float point2X, float point2Y, float originAngle)
{
	float distance = 0.0f;
	float lineAngle = 0.0f;
	float r = 0.0f;
	distance = sqrtf((point1X - point2X) * (point1X - point2X) + (point1Y - point2Y) * (point1Y - point2Y));
	lineAngle = K2Angle(point1X, point1Y, point2X, point2Y);
	if((PI / 2 + originAngle * PI / 180.0f - lineAngle) != 0)
	{
		targetY = point1Y - ((distance / 2) / sin(PI / 2 + originAngle * PI / 180.0f - lineAngle)) * cosf(PI / 2 + originAngle * PI / 180.0f);
		targetX = point1X + ((distance / 2) / sin(PI / 2 + originAngle * PI / 180.0f - lineAngle)) * sinf(PI / 2 + originAngle * PI / 180.0f);
		r = sqrtf((targetX - point2X) * (targetX - point2X) + (targetY - point2Y) * (targetY - point2Y));
		if((PI / 2 + originAngle * PI / 180.0f - lineAngle) < 0)
		{
			GivenCircle(1000, targetX, targetY, r, 1);
		}
		else if((PI / 2 + originAngle * PI / 180.0f - lineAngle) > 0)
		{
			GivenCircle(1000, targetX, targetY, r, 0);
		}
	}
	else
	{
		VelCrl(CAN1, 1, VEL2IMP(1000));
		VelCrl(CAN1, 2, VEL2IMP(1000));
	}
////与之前的目标点坐标结构体衔接
//	if(sqrtf((fakeX - point2X) * (fakeX - point2X) + (fakeY - point2Y) * (fakeY - point2Y)) < 20.0f)
//	{
//		C0Connection(point2X, point2Y, point3X, point3Y, fakeAngle);
//	}
}
struct Point
{
	float x;
	float y;
}pnt={0};
void TowardsPoint(struct Point pnt, float vel)
{
	float k = 0.0f, R = 0.0f;
	if(x != pnt.x)
	{	
		k = (pnt.y - y) / (pnt.x - x);
		if(k > 0.0f)
		{	
			if(x > pnt.x)
			{
				targetAngle = atan(k) * 180 / PI + 90.0f;
			}
			else if(x < pnt.x)
			{
				targetAngle = atan(k) * 180 / PI - 90.0f;
			}
		}
		else if(k < 0.0f)
		{
			if(x > pnt.x)
			{
				targetAngle = atan(k) * 180 / PI + 90.0f;
			}
			else if(x < pnt.x)
			{
				targetAngle = atan(k) * 180 / PI - 90.0f;
			}
		}
		else if(fabs(k) < 0.001f)
		{
			if(x > pnt.x)
			{
				targetAngle = 90.0f;
			}
			else
			{
				targetAngle = -90.0f;
			}
		}
	}	
	else
	{
		if(y > pnt.y)
		{
			targetAngle = 180.0f;
		}
		else if(y < pnt.y)
		{
			targetAngle = 0.0f;
		}
	}
	//若是运用atan2()函数来求targetAngle，还没经过验证，上面那个复杂的已经验证好用
	//if(x > pnt.x && y >= pnt.y)
	//{
	//	targetAngle = 90 + atan2((y - pnt.y),(x - pnt.x)) / 3.141592 * 180;
	//}
	//else if(x <= pnt.x && y > pnt.y)
	//{
	//	targetAngle = 270 - atan2((y - pnt.y),(x - pnt.x)) / 3.141592 * 180;
	//}
	//else if(x < pnt.x && y <= pnt.y)
	//{
	//	targetAngle = 90 + atan2((y - pnt.y),(x - pnt.x)) / 3.141592 * 180;
	//}
	//else if(x >= pnt.x && y < pnt.y)
	//{
	//	targetAngle = 90 + atan2((y - pnt.y),(x - pnt.y)) / 3.141592 * 180;
	//}
	R = (float)sqrt((x - pnt.x) * (x - pnt.x) + (y - pnt.y) * (y - pnt.y));
	dir1_1 = 0.0f - (float)sqrt((x - pnt.x) * (x - pnt.x) + (y - pnt.y) * (y - pnt.y));
	u1 = 2 * dir1_1 + 2 * (dir1_1 - dir1_0);				
	if(fabs(angle - targetAngle) > 2.0f)
	{
		dir1 = targetAngle - angle;
		if(dir1 > 180)
		{
			dir1 -= 360;
		}
		else if(dir1 < -180)
		{
			dir1 += 360;
		}
		u = 200 * dir1 + 2 * (dir1 - dir0);
		dir0 = dir1;	
	}
	else
	{		
		u = 0;
	}	
////如果需要他到达位置停下来的话
//	if(R < )
//	{
//		VelCrl(CAN1, 1,0);
//		VelCrl(CAN1, 2,0);
//	}
//	else
//	{
		VelCrl(CAN1, 1,VEL2IMP(vel) + u - u1);
		VelCrl(CAN1, 2,-VEL2IMP(vel) + u + u1);
//	}
}
/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */



/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/