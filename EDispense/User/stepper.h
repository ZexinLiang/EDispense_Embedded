#ifndef __STEPPER_H
#define __STEPPER_H
#include "main.h"
#include "tim.h"
#include "math.h"

/*******************************************************************************
note :
	中断中实际用整形来运算
	带入计算重新赋参数时使用浮点数的接口，位置，速度 ，加速度 ，减速度

裸机 食用方法：
	定义步进电机对象


	main中初始化
	eg.Init_Stepper(&Stepper1 , GPIOB , GPIO_PIN_4  , &htim4 , TIM_CHANNEL_3 , 0.1125);
					结构体     方向端口    方向引脚   stp定时器		定时器通道      步距角


任意位置调用，目前加速度和速度参数没调好，分别设为50 500是比较合适的
void StpDistanceSetNonBlocking( struct Stepper* stepper , 	float angdistance , 	float accel 	, 	float tagv );
   void StpDistanceSetBlocking( struct Stepper* stepper , 	float angdistance , 	float accel 	, 	float tagv );
																			结构体									角位移，有正负				角加速度（绝对值）		角速度（绝对值）

输出比较中断中添加如下语句
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)------>定时器
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)------>定时器通道
			StepperInOC(&Stepper1);------>定时器通道对应的步进电机
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)------>定时器通道
			StepperInOC(&Stepper2);------>定时器通道对应的步进电机
	}
}

 *******************************************************************************/
//------------------------------------------------全局变量&常量------------------------------------------------------//
extern const int Freq_t;				  // 计算预分频之后时基单元的倒数
extern const uint16_t Tim_Period;		  // 定时器周期
extern const uint16_t Tim_Prescaler;	  // 定时器预分频系数
extern const uint16_t PulseWide_Negetive; // 负半周时间
extern const uint16_t Min_Period;		  // 最短周期
extern const uint16_t Max_Period;		  // 最长周期

//------------------------------------------------步进电机参数结构体------------------------------------------------------//
typedef struct Stepper
{
	__IO uint8_t CC_FLAG;
	__IO uint8_t motor_state; //电机状态寄存各个位分别有不同意义 \
																						MSB（方向1cw0ccw）\
																						[X]		预留	 \
																						[6 :3]  运动模态\
																						[2:1]   运动阶段\
																						[0]			运动或静止
	GPIO_TypeDef *StpPort;			   //
	uint16_t StpPin;		  //																						
	__IO uint16_t CCR_ValSetter;
	__IO uint16_t EnPort; //
	uint16_t EnPin;		  //

	GPIO_TypeDef *DirPort;			   //
	TIM_HandleTypeDef *StpTim;		   //
	uint16_t StpChannel;			   //
	uint16_t DirPin;				   //
	__IO int32_t acceldistancebuffer; // 缓存加减速段角位移
	__IO int32_t accletperiod0;	   // 缓存加减速段第一个周期

	__IO int32_t period_buffer;	// 预算的初速度，在加速度相同时不需要重新计算
	__IO int32_t period_now;		// 当前周期
	__IO int32_t period_rest;		// 余数，在运算完新的周期后被赋值
	__IO int32_t step_counter;		// 当前阶段的步数计数
	__IO int32_t step_threshold;	// 当前阶段的总步数
	__IO int32_t stepff_memory[6]; // 计算各个步骤点 [0]加速 [1]匀速 [2]减速

	int position_ctnow; // 以计数器的形式计算的当前位置，转换成角度需要乘步距角

	//	float								angv_now;
	float position_ang; // 以浮点角度的形式表示的位置，
	float stepangle;	// 步距角
	float TagAngV;		// 目标角速度
	float AcceAng;		// 角加速度
	
	
	__IO uint32_t If_End;


 
	uint32_t			StpMode;//步进电机模式 1 梯形加减速  2 速度跟随器 记得更改
	uint32_t      OverLoadN;//需要的溢出次数
	__IO uint32_t	V_Duty;//速度周期 
	__IO uint32_t	V_HalfDutyl;//速度半周期
	__IO uint32_t ToRunCnt;//要转的步数
	__IO uint16_t TimCntScale;//定时器预装载值

} Stepper;

void Init_Stepper(Stepper *stp, GPIO_TypeDef *stpdir_gpio_port, uint16_t stpdir_gpio_pin, GPIO_TypeDef *stp_gpio_port, uint16_t stp_gpio_pin,
				  TIM_HandleTypeDef *stps_timer, uint16_t stps_pin, float angpp); // 初始化是配置步进电机的相关通道与IO等

//---------------------------------------------------逻辑部分----------------------------------------------------\\
void			StopSetBit(uint8_t* flag)       {*flag  &= 0xfe ;}								//停止

/*
0 0000 00 0
0 : 方向
000 0：单向平顶 000 1：单向尖顶 100 0：变向平顶 100 1：变向尖顶

00 阶段 01 10 11 00
0  运动还是静止

*/
#define MOVINGFLAG (uint8_t)0x01
#define DIRFLAG (uint8_t)0x80
#define TYPEFLAG (uint8_t)0x08
#define STEPFLAG (uint8_t)0x06
#define DIRCHANGEFLAG (uint8_t)0x40

#define FULLSTEP (uint8_t)0x00
#define MIDPOINT (uint8_t)0x08

#define ACCELING (uint8_t)0x02
#define RUNNING (uint8_t)0x04
#define DECELING (uint8_t)0x06
#define STOPING (uint8_t)0x00

#define STATETOFULLSTEP(flag)   \
	{                           \
		(*flag) &= (~TYPEFLAG); \
		(*flag) |= FULLSTEP;    \
	} // 切换到停止状态
#define STATETOMIDPOINT(flag)   \
	{                           \
		(*flag) &= (~TYPEFLAG); \
		(*flag) |= MIDPOINT;    \
	} // 切换到停止状态
#define DIRCHANGENEEDED(flag)       \
	{                               \
		(*flag) |= (DIRCHANGEFLAG); \
	} // 切换到要变向状态
#define DIRCHANGECANCEL(flag)        \
	{                                \
		(*flag) &= (~DIRCHANGEFLAG); \
	} // 切换到不需要要变向状态

#define MoveSetBit(flag)          \
	{                             \
		(*flag) &= (~MOVINGFLAG); \
		(*flag) |= MOVINGFLAG;    \
	} // 切换到运动状态							//
#define StateToAcc(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= ACCELING;    \
	} // 切换到加速状态
#define StateToRun(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= RUNNING;     \
	} // 切换到匀速状态
#define StateToDec(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= DECELING;    \
	} // 切换到减速状态
#define StateToStp(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= STOPING;     \
	} // 切换到停止状态

#define IFDIRCHANGE(flag) ((flag) & (uint8_t)DIRCHANGEFLAG)
#define IFMOVING(flag) ((flag) & (uint8_t)MOVINGFLAG)
#define GETDIRECT(flag) ((flag) & (uint8_t)DIRFLAG)
#define GETTYPE(flag) ((flag) & (uint8_t)TYPEFLAG)
#define GETRUNSTEP(flag) ((flag) & (uint8_t)STEPFLAG)

#define CW (uint8_t)0x80
#define CCW (uint8_t)0x00
#define IfCW(flag) ((flag) & (uint8_t)CW)
#define DIRTOCW(flag)          \
	{                          \
		(*flag) &= (~DIRFLAG); \
		(*flag) |= CW;         \
	} // 切换到顺时针状态
#define DIRTOCCW(flag)         \
	{                          \
		(*flag) &= (~DIRFLAG); \
		(*flag) |= CCW;        \
	} // 切换到逆时针状态
// #define			CHANGEDIR(flag)				{(IfCW(*flag))? {DIRTOCCW(flag)}:{DIRTOCW(flag)}}//换向

/*----------------------- 引脚定义，注意一定要先在hal库里配置好 -----------------------------------*/

/*----------------------- 方向引脚控制 -----------------------------------*/

/*----------------------- 失能引脚控制 -----------------------------------*/
/* 使能失能 x = 1 有效，x = 0时无效*/

/******************************************************************************************/
/* 外部接口函数*/
//extern struct Stepper Stepper1;

float _Cal_PositionAng(struct Stepper *stepper);
// 利用计数器计算当前坐标（浮点表示）并返回

void StpDistanceSetNonBlocking(struct Stepper *stepper, float angdistance, float accel, float tagv);
// 设置新的运动时，新运动于旧运动同方向，将需要多走的步数添加到当前的运动上

uint8_t StpDistanceSetBlocking(struct Stepper *stepper, float angdistance, float accel, float tagv);
// 设置新的运动时，新运动于旧运动同方向，将需要多走的步数添加到当前的运动上

void StepperInOC(Stepper *stp);
float _Cal_AngV(struct Stepper* stepper);
void Stepper_Pause(struct Stepper *stepper);
void Stepper_Stop(Stepper *stepper);
void StpVFollowerStart(Stepper *s  ,   float newV);
void Init_StpVMode(Stepper *s   ,  uint16_t TimCntScale );
void NewStpV_CalCulator(Stepper *s  ,  float angletorun , float newV);
void StpVSFollowerStart(Stepper *s  ,  float angletorun , float newV);
void StepperInOC_ForVfollower(Stepper *stp);//duty的值在别的地方给
#endif

