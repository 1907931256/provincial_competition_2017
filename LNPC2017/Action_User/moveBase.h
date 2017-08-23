/**
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H



/* Includes ------------------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */

//#define 

//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (4096)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (120.0f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (492.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (490.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (40.0f)
//两个轮子中心距离（单位：mm）
#define WHEEL_TREAD (439.14f)

#define PI 														3.141592f
#define TRACK_UP_ADVANCE 							3200.0f
#define TRACK_DOWN_ADVANCE 						1200.0f
#define	TRACK_LEFT_ADVANCE						-1000.0f
#define	TRACK_RIGHT_ADVANCE 					700.0f
#define TRACK_UP_NUMBER 							2
#define TRACK_DOWN_NUMBER 						0
#define	TRACK_LEFT_NUMBER							3
#define	TRACK_RIGHT_NUMBER 						1
#define TRACK_UP_ORIGIN 							4300.0f
#define TRACK_DOWN_ORIGIN 						400.0f
#define	TRACK_LEFT_ORIGIN							-2000.0f
#define	TRACK_RIGHT_ORIGIN 						1900.0f
#define ANGEL_PID_P 									180
#define ANGEL_PID_D 									2
#define DISTANCE_PID_P 								10
#define DISTANCE_PID_D 								2
#define LINE0_ANGLE										-17.0f
#define LINE1_ANGLE										0.0f
#define LINE2_ANGLE										17.0f
#define ROW0_DISTANCE									25.0f
#define ROW1_DISTANCE									75.0f
#define ROW2_DISTANCE									125.0f

#define DISTANCE(x) 									(x * 0.9445f + 380.2f)
#define ANGLE2RAD(x) 									(x * PI / 180.0f)
#define VEL2IMP(x) 										(x * COUNCTS_PER_ROUND / (PI * WHEEL_DIAMETER))
#define POLAR2RIGHT_X(a,d)						(float)(x + d * cos(ANGLE2RAD(90 + a)))
#define POLAR2RIGHT_Y(a,d)						(float)(y + d * sin(ANGLE2RAD(90 + a)))
/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void SquareClosed_LoopNewNew(void);
void SquareClosed_LoopNew(void);
void UseCamera(void);
void AnotherUseCamera(void);
void BroChenUseCamera(void);
void YaoyiyaoUseCamera(void);
void GivenPoint(float targetX, float targetY);
int IfMirror(int x);



#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

