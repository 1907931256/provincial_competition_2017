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
#include "motionCard.h"


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

extern float fakeX, fakeY, fakeAngle;


void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/ 
	PeriodSem = OSSemCreate(0);

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//1ms定时器用于控制WalkTask周期
	TIM_Init(TIM2, 99, 839, 0, 0);
	BufferZizeInit(400);
	//行程开关初始化
	KeyInit();
	//CAN初始化
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	//串口初始化
	USART3_Init(115200);
	USART1_Init(115200);
	UART5_Init(115200);
	//驱动器初始化
	elmo_Init(CAN1);
	elmo_Enable(CAN1,1);
	elmo_Enable(CAN1,2);
	//ADC初始化
	Adc_Init();
	//配置速度环
	Vel_cfg(CAN1, 1, 50000, 50000);
	Vel_cfg(CAN1, 2, 50000, 50000);

	delay_ms(10000); //00

	OSTaskSuspend(OS_PRIO_SELF);
}
extern int mirror;
extern int ifMirror;
extern int arrive;
int step = 0;
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
//	vel=(355.4/r+1)*8192;
	//PeriodSem = 0;
//	vel = 500;	
//	impulse = vel * 4096 / (3.14 * 106.8);
//	t = 200000 / vel;
//	targetAngle = 0.0;
//	r = 1000;
	OSSemSet(PeriodSem, 0, &os_err);	
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);	
		
		
		//if(ifMirror == 1)
		//{
		//	mirror = IfMirror(ifMirror);
		//}	
		//if(ifMirror == 0)
		//{
		//SquareClosed_LoopNew();
		//}
		 
		//AnotherUseCamera();
		
		//BroChenUseCamera();
//		if(arrive == 1)
//		{
//			arrive = 0;
//			step++;
//		}
//		switch(step)
//		{
//			case 0:
//				GivenPoint(500.0f, 1000.0f);
//				break;
//			case 1:
//				GivenPoint(1500.0f, 2000.0f);
//				break;
//			case 2:
//				GivenPoint(2000.0f, 3000.0f);
//				break;
//			case 3:
//				GivenPoint(1000.0f, 4000.0f);
//				break;
//			case 4:

//				float rightLaser = 0.0f,leftLaser = 0.0f;
//				rightLaser = Get_Adc_Average(ADC_Channel_14,100);
//				leftLaser = Get_Adc_Average(ADC_Channel_15,100);
//				break;
//		}
//		USART_OUT(USART1, (uint8_t*)"%d\t%d\t%d\t%d\t%d\t%d\r\n", (int)fakeAngle, (int)fakeX, (int)fakeY, (int)leftLaser, (int)rightLaser);	
	}
}
		

		
