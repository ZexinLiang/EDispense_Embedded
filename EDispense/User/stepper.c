/*        	┌─┐      ┌─┐
#      ┌──┘ ┴ ───┘ ┴ ┐
#      │                 		│
#      │       ───       	│
#      │  ─┬┘      └┬─  │
#      │                 		│
#      │       ─┴─       	│
#      │                 		│
#      └───┐         ┌─┘
#          		│         │
#         		│         │
#          		│         │
#          		│         └──────────────┐
#         		│                        								│
#         		│                        								├─┐
#          		│                        								┌─┘
#          		│                        								│
#         		 └─┐  ┐  ┌───────┬──┐  ┌┘
#         		   ─┤ ─┤       					│ ─┤ ─┤
#         		 └──┴──┘       			└──┴──┘
#                神兽保佑
#                代码无BUG!
*/

#include "stepper.h"
//struct Stepper Stepper1;
//速度与加速度的运算存在一些问题，还需要再测试改进
uint32_t Accel_Max=65535;
uint32_t Velocity_Max=65535;

int  midway=0 ; // 计算所需要的中间量

//acc 2700 vmax 3800 x

//------------------------------------------------全局变量&常量------------------------------------------------------//

const int  Freq_t=170000000/170;						//计算预分频之后时基单元的倒数
const uint16_t Tim_Period=65535;				//定时器周期
const uint16_t Tim_Prescaler=170-1;				//定时器预分频系数
const uint16_t PulseWide_Negetive=35;		//负半周时间
const uint16_t Min_Period=20;						//最短周期
const uint16_t Max_Period=50000;				//最长周期


//------------------------------------------------初始化函数------------------------------------------------------//
/*******************************************************************************
 * @brief       生成步进电机对象
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 *******************************************************************************/
void Init_Stepper(Stepper* stp , GPIO_TypeDef* stpdir_gpio_port , uint16_t stpdir_gpio_pin , GPIO_TypeDef *stp_gpio_port, uint16_t stp_gpio_pin,\
									TIM_HandleTypeDef* stps_timer , uint16_t stps_ch , float angpp)
{
	stp->CC_FLAG 				= 0; 
	stp->AcceAng 				=	0 ;
	stp->StpPort				= stp_gpio_port;
	stp->StpPin					= stp_gpio_pin;
	stp->DirPin	 				=	stpdir_gpio_pin ;
	stp->DirPort 				=	stpdir_gpio_port;
	stp->EnPin					= 0;
	stp->EnPort					=	0;
	stp->motor_state 		= 0;
	stp->period_now 		= 0;
	stp->period_rest 		= 0;
	stp->position_ang 	= 0;
	stp->position_ctnow 	= 0;
	stp->stepangle 			= angpp;
	stp->step_counter 	= 0;
	stp->step_threshold = 0;
	stp->StpChannel 		= stps_ch;
	stp->StpTim 				= stps_timer;
	stp->TagAngV 				= 0;						
//	stp->angv_now 			= 0;
	stp->position_ctnow = 0;
	stp->CCR_ValSetter	= 0;
	stp->accletperiod0	= 0;
	for(int i=0;i<6;i++){
		stp->stepff_memory [i]=0;
	}
	stp->acceldistancebuffer=0;
//	switch(stps_ch)
//	{
//		case TIM_CHANNEL_1 : {
//													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR1));
//													}
//		case TIM_CHANNEL_2 : {
//													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR2));
//													}		
//		case TIM_CHANNEL_3 : {
//													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR3));
//													}		
//		case TIM_CHANNEL_4 : {
//													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR4));
//													}		
//	}
}


//------------------------------------------------工具函数------------------------------------------------------//
void StepperInOC(Stepper* stp);

/*******************************************************************************
 * @brief       取绝对值
 * @param       要取绝对值的数
 * @retval      无
 *******************************************************************************/
uint32_t myabsi(int a)
{
	if(a>=0){
		return a;
	}else
	{
		return -a;
	}
}
float myabsf(float a)
{
	if(a>=0){
		return a;
	}else
	{
		return -a;
	}
}



/**
 * @brief       利用计数器计算当前坐标（浮点表示）并返回
 * @param       Stepper* stepper: 步进电机
 * @retval      当前角度（绝对位置）
 */
float _Cal_PositionAng(struct Stepper* stepper)
{
	stepper->position_ang = stepper->stepangle * stepper->position_ctnow  ;
	return  stepper->position_ang;
}
/*******************************************************************************
 * @brief       计算当前角速度（浮点表示）
 * @param       Stepper* stepper: 步进电机
 * @retval      计算角速度
 *******************************************************************************/
float _Cal_AngV(struct Stepper* stepper)
{
	if((IFMOVING(stepper->motor_state))){
		if(IfCW((stepper->motor_state)))//CW
		{
			return (stepper->stepangle * Freq_t)/(stepper->period_now) ; 
		}
		else//CCW
		{
			return	-(stepper->stepangle * Freq_t)/(stepper->period_now) ;
		}
	}else{
		return  0;
	}
}



/*******************************************************************************
 * @brief       关闭步进电机
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 *******************************************************************************/
void Stepper_Stop(struct Stepper* stepper)
{
		HAL_TIM_OC_Stop_IT(stepper->StpTim, stepper->StpChannel); 
//		stepper->AcceAng					= 0; 
		stepper->motor_state 			= 0;
		stepper->period_now 			= 0;
		stepper->period_rest 			= 0;
		stepper->step_counter 		= 0;	 
}
/*******************************************************************************
 * @brief       暂停步进电机
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 *******************************************************************************/
void	Stepper_Pause(struct Stepper* stepper)
{
	HAL_TIM_OC_Stop_IT(stepper->StpTim, stepper->StpChannel); 
}
/*******************************************************************************
 * @brief       计算电机从初速度加速到目标速度需要的角位移,返回的是脉冲步数
 * @param				struct Stepper* stepper 对应的步进电机
 * @retval      int类型，目标位置
 *******************************************************************************/
uint32_t Calculator_AngleAcc(struct Stepper* stepper , float startv , float tagv )//计算电机从初速度加速到目标速度需要的角位移
{
	static float mid = 0;
	mid = (tagv - startv) ;
	return fabsf(mid*(startv + 0.5*mid))/(stepper->AcceAng * stepper->stepangle);   
	
}
/*
					先固定死速度和加速度好了				
					uint32_t stepstoacc1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );
					uint32_t stepstodec  = Calculator_AngleAcc(stepper , _Cal_AngV(stepper) , stepper->TagAngV );
					Calculator_AngleAcc(stepper , _Cal_AngV(stepper) , stepper->TagAngV );
	*/
/*******************************************************************************
 * @brief       计算电机加速度过程中需要的周期
								第一个脉冲不在这里计算
* @param				struct Stepper* stepper 对应的步进电机
								steps 当前运动的总步数
 * @retval      none
 *******************************************************************************/
void CalCulator_NextCount_Accel(struct Stepper* stepper )//计算电机加速度过程中需要的周期
{
	static uint32_t rest = 0;
	
	rest = ((2 *stepper->period_now)+stepper->period_rest)%(4 * stepper->step_counter  + 1);//计算当前的余数
	
	stepper->period_now=(stepper->period_now - ( 2*stepper->period_now + stepper->period_rest )/( 4*stepper->step_counter  + 1));//计算新的周期，使用的是上一次的余数
	
	stepper->period_rest  =  rest ;  //余数赋值
	
	if(stepper->period_now<=Min_Period){//周期限幅
		
		stepper->period_now=Min_Period;
		
	}
	
}


/*******************************************************************************
 * @brief       计算电机减速度过程中需要的周期
								第一个脉冲不在这里计算
 * @param				struct Stepper* stepper 对应的步进电机
								steps 当前运动的总步数
 * @retval      none
 *******************************************************************************/
void CalCulator_NextCount_Deccel(struct Stepper* stepper )//计算电机减速度过程中需要的周期
{
	static uint32_t rest = 0;
	
	rest = ((/*2 **/stepper->period_now)+stepper->period_rest)%(4 * stepper->step_counter  - 1);//计算当前的余数
	
	stepper->period_now=(stepper->period_now + ( 2*stepper->period_now + stepper->period_rest )/( 4*stepper->step_counter  - 1));//计算新的周期，使用的是上一次的余数
	
	stepper->period_rest  =  rest ;  //余数赋值
		
	if(stepper->period_now>=Max_Period){//周期限幅
		
		stepper->period_now=Max_Period;
		
	}
	
}


/*******************************************************************************
 * @brief       初始化要运动的步进电机运动参数
								大于步距角，否者不会动
 * @param       Stepper* stepper 电机结构体
 * @retval      无
*******************************************************************************/
uint8_t c=0;
uint8_t StpDistanceSetBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv )
{
	if(myabsf(angdistance)>(stepper->stepangle))//大于步距角，否者不会动
	{
		if(!(IFMOVING(stepper->motor_state)))  //任务下达时电机静止	
		{
			if(angdistance<=0)
			{
				DIRTOCW(&stepper->motor_state) //顺时针
			}
			else
			{
				DIRTOCCW(&stepper->motor_state)//逆时针	
				
			}					
			stepper->AcceAng = accel;
			stepper->TagAngV = tagv;		
			stepper->stepff_memory[0]= Calculator_AngleAcc(stepper , 0 , stepper->TagAngV);
			if(stepper->stepff_memory[0]<1)
			{
				return 0; 
			}
			stepper->period_now = 0.676*Freq_t*sqrt(2*stepper->stepangle/stepper->AcceAng);	
			if(stepper->period_now<=Min_Period){//周期限幅
				stepper->period_now=Min_Period;
			}else if(stepper->period_now>=Max_Period){
				stepper->period_now=Max_Period;
			}
			stepper->accletperiod0=stepper->period_now;
			
			

			stepper->stepff_memory[1]= myabsi(angdistance/(stepper->stepangle));
			stepper->stepff_memory[5]=stepper->stepff_memory[1];//终点计数器
			if( stepper->stepff_memory[0] < (stepper->stepff_memory[1]/2) )
			{//可以加到最大速度
				stepper->stepff_memory[1]	=	stepper->stepff_memory[1]-2*stepper->stepff_memory[0];
				stepper->stepff_memory[2] = stepper->stepff_memory[0];
//				if(stepper->stepff_memory[0]==0)
//				{
//					stepper->stepff_memory[0]=1;
//				}
//				if(stepper->stepff_memory[1]==0)
//				{
//					stepper->stepff_memory[1]=1;					
//				}			
//				if(stepper->stepff_memory[2]==0)
//				{
//					stepper->stepff_memory[2]=1;					
//				}								
				StateToAcc(&stepper->motor_state);
				stepper->motor_state&=(~TYPEFLAG);
				stepper->step_threshold = stepper->stepff_memory[0] ;
			}
			else
			{//中点减速
				stepper->stepff_memory[0] = stepper->stepff_memory[1]/2 + (stepper->stepff_memory[1])%2 ;		
				stepper->stepff_memory[1] = stepper->stepff_memory[1]/2  ;
//				if(stepper->stepff_memory[0]==0)
//				{
//					stepper->stepff_memory[0]=1;
//				}
//				if(stepper->stepff_memory[0]==0)
//				{
//					stepper->stepff_memory[1]=1;					
//				}
				stepper->step_threshold = stepper->stepff_memory[0]  ;
				stepper->motor_state&=(~TYPEFLAG);
				stepper->motor_state|=MIDPOINT; //尖峰运动
				StateToAcc(&stepper->motor_state);			
			
			}		
			MoveSetBit(&stepper->motor_state);//置位到运动状态
			if((GETDIRECT(stepper->motor_state))==CW){//运动方向
				HAL_GPIO_WritePin(stepper->DirPort , stepper->DirPin  , GPIO_PIN_SET);			
			}else{
				HAL_GPIO_WritePin(stepper->DirPort , stepper->DirPin  , GPIO_PIN_RESET);			
			}
		}
	

		//StepperInOC(stepper);
		stepper->If_End=0;
		stepper->CC_FLAG=0;
		uint16_t ccrval= __HAL_TIM_GET_COMPARE(stepper->StpTim , stepper->StpChannel)+PulseWide_Negetive;
		__HAL_TIM_SET_COMPARE(stepper->StpTim , stepper->StpChannel  , ccrval);
		HAL_TIM_OC_Start_IT(stepper->StpTim , stepper->StpChannel);	
	}		
	else{
		Stepper_Stop(stepper);
	}
	return 0;
}



/*******************************************************************************
 * @brief       在定时器中断中修改参数
								在运动步骤状态机中查询运动类型
								在状态跳变时触发
								只负责修改状态
 * @param       Stepper* stepper 电机结构体
 * @retval      无
 *******************************************************************************/
//uint8_t value=0;
void Stepper_Handler(struct Stepper* stepper)
{

							if(GETDIRECT(stepper->motor_state)){
								stepper->position_ctnow +=1;			
							}else{
								stepper->position_ctnow -=1;
							}							
							
//					value=		
				if(GETRUNSTEP(stepper->motor_state)==ACCELING)	
				{			
							stepper->step_counter ++ ;						 								
							if(stepper->step_counter < stepper->step_threshold)
							{
								CalCulator_NextCount_Accel(stepper);	
							}
							else if (stepper->step_counter >= stepper->step_threshold)
							{//状态机即将结束加速
								CalCulator_NextCount_Accel(stepper);
								if((GETTYPE(stepper->motor_state))==FULLSTEP)
								{//平顶							
									StateToRun(&stepper->motor_state);
									//stepper->period_now = stepper->period_buffer ;
									stepper->step_threshold = stepper->stepff_memory [1] ;	
									stepper->step_counter 	= 1 ;				
								}
								else if((GETTYPE(stepper->motor_state))==MIDPOINT)
								{//尖顶
									StateToDec(&stepper->motor_state);//状态机到减速
									stepper->step_counter 	= stepper->stepff_memory [1]-1 ;
									if(stepper->step_counter==0)
									{
										StateToStp(&stepper->motor_state);//状态机到停止
									}
								}
							}

							
						}

				else if(GETRUNSTEP(stepper->motor_state)==RUNNING)	
				{		
								stepper->step_counter ++ ; 														
							if(stepper->step_counter>= stepper->step_threshold){//状态机即将结束匀速
								if((GETTYPE(stepper->motor_state))==FULLSTEP)
								{//平顶或尖顶
									StateToDec(&stepper->motor_state);//状态机到减速
									stepper->step_counter = stepper->stepff_memory [2] ;
									if(stepper->step_counter<=0)
									{
										StateToStp(&stepper->motor_state);//状态机到停止
									}
								}								
							}	

							
					}
				else if(GETRUNSTEP(stepper->motor_state)==DECELING)	
				{						
							CalCulator_NextCount_Deccel(stepper);
							if(stepper->step_counter<=1){
								if((GETTYPE(stepper->motor_state))==FULLSTEP||(GETTYPE(stepper->motor_state))==MIDPOINT)
								{//平顶或尖顶
									StateToStp(&stepper->motor_state);//状态机到停止
									//Stepper_Stop(stepper);		
																		
								}									
							}							
							stepper->step_counter -- ;	
						}
				else if(GETRUNSTEP(stepper->motor_state)==STOPING)	
				{	

							stepper->If_End=1;
							//Stepper_Stop(stepper);								

				}
}


void Stepper_Handler_Stop(struct Stepper* stepper)
{
	if(stepper->If_End)
	{
		Stepper_Stop(stepper);
		stepper->If_End	=0;	
	}
	
}

/*******************************************************************************
 * @brief       在定时器中断中修改参数
								在运动步骤状态机中查询运动类型
								在状态跳变时触发
								只负责修改状态
 * @param       Stepper* stepper 电机结构体
 * @retval      无
 *******************************************************************************/
void StepperInOC(Stepper* stp)
{
		uint16_t Comparenow=__HAL_TIM_GET_COMPARE(stp->StpTim , stp->StpChannel);
	 if (!stp->CC_FLAG){
			Stepper_Handler(stp);
//			stp->CCR_ValSetter=(Tim_Period-stp->period_now+PulseWide_Negetive);
//			if(Comparenow < stp->CCR_ValSetter ) {
//				 __HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel , Comparenow+(stp->period_now-PulseWide_Negetive));
//			}else{
//				__HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel ,(stp->period_now-stp->CCR_ValSetter));
//			}
		 stp->CCR_ValSetter=Comparenow+stp->period_now/2;
		 __HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel , stp->CCR_ValSetter);
		}
		else {
			Stepper_Handler_Stop(stp);
//			if(Comparenow <stp->CCR_ValSetter ) {
//			 __HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel ,(Comparenow+PulseWide_Negetive));
//			}else{
//				__HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel , (PulseWide_Negetive-stp->CCR_ValSetter));
//			}	
			
			stp->CCR_ValSetter=Comparenow+stp->period_now/2;
			__HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel , stp->CCR_ValSetter);
		}
		HAL_GPIO_TogglePin(stp->StpPort , stp->StpPin);
		stp->CC_FLAG = !stp->CC_FLAG;		
}











/*速度控制模式添加*/






















/*速度控制模式添加*/

/*******************************************************************************
 * @brief       定时器比较中断回调
 * @param       定时器通道
 * @retval      无
*******************************************************************************/






uint32_t Rounding(float num)
{
	uint32_t num_uint=fabs(num);
	if(fabs(num-num_uint)<=0.5)
	{
		return num_uint;
	}else{
		return (num_uint+1);
	}
}


void Init_StpVMode(Stepper *s   ,  uint16_t TimCntScale )
{
	s->StpMode = 2 ;
	//s->StpMode = 2 ;
	s->V_Duty=0;
	s->V_HalfDutyl=0;
	s->TimCntScale=TimCntScale;
	s->ToRunCnt=0;
}
//不带角度需求的
void NewStpV_Cal(Stepper *s  ,   float newV)
{
	s->StpMode=2;
	//s->ToRunCnt=Rounding((angletorun/s->stepangle));			//fabs(angletorun)/s->stepangle;
	s->V_Duty=(Freq_t*s->stepangle)/newV;
	s->V_HalfDutyl=s->V_Duty/2.f;
}
void NewStpV_CalCulator(Stepper *s  ,  float angletorun , float newV)
{
	s->StpMode=2;
	s->ToRunCnt=Rounding((angletorun/s->stepangle));			//fabs(angletorun)/s->stepangle;
	s->V_Duty=(Freq_t*s->stepangle)/newV;
	s->V_HalfDutyl=s->V_Duty/2.f;
}
void StpVFollowerStart(Stepper *s  ,   float newV)
{
		MoveSetBit(&(s->motor_state));
		if (newV >= 0)
		{
			DIRTOCW(&(s->motor_state))// 顺时针
			HAL_GPIO_WritePin(s->DirPort , s->DirPin , GPIO_PIN_RESET);
		}
		else
		{
			DIRTOCCW(&(s->motor_state)) // 逆时针
			HAL_GPIO_WritePin(s->DirPort , s->DirPin , GPIO_PIN_SET);
			newV = -newV;				
		}
		NewStpV_Cal(s ,  newV );
		uint16_t Comparenow = __HAL_TIM_GET_COMPARE(s->StpTim, s->StpChannel);
		s->CCR_ValSetter = ( Comparenow + s->V_HalfDutyl );
		if (s->CCR_ValSetter < s->TimCntScale)
		{
			__HAL_TIM_SET_COMPARE(s->StpTim, s->StpChannel, s->CCR_ValSetter);//(stp->period_now - PulseWide_Negetive));
		}
		else
		{
			__HAL_TIM_SET_COMPARE(s->StpTim, s->StpChannel, ( s->CCR_ValSetter - s->TimCntScale));
		}
		HAL_TIM_OC_Start_IT(s->StpTim, s->StpChannel);
}
void StpVSFollowerStart(Stepper *s  ,  float angletorun , float newV)
{
		MoveSetBit(&(s->motor_state));
		if (angletorun >= 0)
		{
			DIRTOCW(&(s->motor_state))// 顺时针
			HAL_GPIO_WritePin(s->DirPort , s->DirPin , GPIO_PIN_SET);
		}
		else
		{
			DIRTOCCW(&(s->motor_state)) // 逆时针
			HAL_GPIO_WritePin(s->DirPort , s->DirPin , GPIO_PIN_RESET);				
		}
		NewStpV_CalCulator(s , angletorun , newV );
		uint16_t Comparenow = __HAL_TIM_GET_COMPARE(s->StpTim, s->StpChannel);
		s->CCR_ValSetter = ( Comparenow + s->V_HalfDutyl );
		if (s->CCR_ValSetter < s->TimCntScale)
		{
			__HAL_TIM_SET_COMPARE(s->StpTim, s->StpChannel, s->CCR_ValSetter);//(stp->period_now - PulseWide_Negetive));
		}
		else
		{
			__HAL_TIM_SET_COMPARE(s->StpTim, s->StpChannel, ( s->CCR_ValSetter - s->TimCntScale));
		}
		HAL_TIM_OC_Start_IT(s->StpTim, s->StpChannel);
}

void Stepper_Handler_ForVfollower(struct Stepper *stepper)
{
	stepper->step_counter++;
	if (GETDIRECT(stepper->motor_state))
	{
		stepper->position_ctnow += 1;
	}
	else
	{
		stepper->position_ctnow -= 1;
	}
	if(stepper->step_counter==stepper->ToRunCnt)
	{
		Init_StpVMode(stepper , stepper->TimCntScale);
		Stepper_Stop(stepper);
	}
}

void StepperInOC_ForVfollower(Stepper *stp)//duty的值在别的地方给
{
	uint16_t Comparenow = __HAL_TIM_GET_COMPARE(stp->StpTim, stp->StpChannel);
	stp->CCR_ValSetter = ( Comparenow + stp->V_HalfDutyl );
	if (stp->CC_FLAG)
	{
		if (stp->CCR_ValSetter < stp->TimCntScale)
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, stp->CCR_ValSetter);//(stp->period_now - PulseWide_Negetive));
		}
		else
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, ( stp->CCR_ValSetter - stp->TimCntScale));
		}
	}
	else
	{
		Stepper_Handler_ForVfollower(stp);
		
		if (stp->CCR_ValSetter < stp->TimCntScale)
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, stp->CCR_ValSetter);//(stp->period_now - PulseWide_Negetive));
		}
		else
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, ( stp->CCR_ValSetter - stp->TimCntScale));
		}
		
		
	}
	
	stp->CC_FLAG = !stp->CC_FLAG;
	HAL_GPIO_TogglePin(stp->StpPort , stp->StpPin);
}

void NewStpV_CalC(Stepper *s  , float newV)
{

	s->StpMode=2;
	s->V_Duty=(Freq_t*s->stepangle)/newV;
	s->V_HalfDutyl=s->V_Duty/2.f;
	if(s->V_HalfDutyl>65535)
	{
		s->V_HalfDutyl=65535;
	}
	else if(s->V_HalfDutyl<60)
	{
		s->V_HalfDutyl=60;
	}
}























