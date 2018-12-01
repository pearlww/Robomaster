#include "main.h"
#include "test_can.h"
#include "RemoteTask.h"
#include "can.h"
#include "CanBusTask.h"
#include "ControlTask.h"
#include "PID.h"
#include "gun.h"

extern RC_Ctl_t RC_CtrlData;
uint32_t Period=4;
float Ramp_count=0.0f;
/*
typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	float kp_offset;
	float ki_offset;
	float kd_offset;
	void (*Calc)(struct PID_Regulator_t *pid);//函数指针
	void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;
void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
*/
//对PID结构体进行初始化，赋值为默认值
PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;   //云台俯仰位置PID
PID_Regulator_t GMPSpeedPID = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;//云台俯仰速度PID
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;			//云台偏航位置PID
PID_Regulator_t GMYSpeedPID = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;//云台偏航速度PID

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; //底盘四个电机旋转PID
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//底盘电机1的速度PID
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//底盘电机2的速度PID
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//底盘电机3的速度PID
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//底盘电机4的速度PID

PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //拨轮位置PID shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;//拨轮速度PID

/*--------------------------------------------CTRL Variables----------------------------------------*/
/*
typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态
    NORMAL_STATE,			//无输入状态
    STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;
*/
WorkState_e lastWorkState = PREPARE_STATE;//上一次检测时，系统所处的工作状态
WorkState_e workState = PREPARE_STATE;//系统当前所处的工作状态

/*
typedef struct RampGen_t
{
	int32_t count;
	int32_t XSCALE;
	float out;
	void (*Init)(struct RampGen_t *ramp, int32_t XSCALE);
	float (*Calc)(struct RampGen_t *ramp);
	void (*SetCounter)(struct RampGen_t *ramp, int32_t count);
	void (*ResetCounter)(struct RampGen_t *ramp);
	void (*SetScale)(struct RampGen_t *ramp, int32_t scale);
	uint8_t (*IsOverflow)(struct RampGen_t *ramp);
}RampGen_t;
*/

//RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;//斜坡，响应要处理成斜坡
//RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;//斜坡，响应要处理成斜坡
	
/*
*********************************************************************************************************
*                                            FUNCTIONS 
*********************************************************************************************************
*/
//设置系统的工作状态
static void SetWorkState(WorkState_e state)
{
    workState = state;
}


//获取当前系统所处的工作状态
WorkState_e GetWorkState()
{
	return workState;
}


//底盘控制任务
void CMControlLoop(void)
{  
	//底盘旋转量计算，此函数决定了底盘会跟随云台转动，检测监控
	if(GetWorkState()==PREPARE_STATE) //启动阶段，底盘不旋转
	{
		ChassisSpeedRef.rotate_ref = 0;	 
	}
	else
	{
		 //底盘跟随编码器旋转PID计算
		 CMRotatePID.ref=0.02*(RC_CtrlData.rc.ch2-(int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		// CMRotatePID.fdb = GMYawEncoder.ecd_angle;
		 CMRotatePID.Calc(&CMRotatePID);   
		 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
	}
	//
	//如果遥控器丢失，强制将速度设定值reset
//	if(Is_Lost_Error_Set(LOST_ERROR_RC))      
//	{
//		ChassisSpeedRef.forward_back_ref = 0;
//		ChassisSpeedRef.left_right_ref = 0;
//	}


	//由于底盘是麦轮，因此这里的reference分为三个分量，前后，左右，自旋。这四句话相当于三个未知数，四个方程。因此这四个ref并不是能够通过遥控器遍历所有值
	//其次，这里的正负号的问题，与麦轮安装的方式有关，O型，X型等。
	//确定公式的时候，可让底盘直走，左右平移，自旋，分别记录四个电机的正反转，然后判断这里的正负号
	//这里的ref并不遥控器的直接值，而是经过处理过，把遥控器中值设置为0，则左为负，右为正
	//如若电机抖动，减小PID的P	


	//CMRotatePID.ref = (RC_CtrlData.rc.ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
	CM1SpeedPID.ref = 27*( ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075); //+ ChassisSpeedRef.rotate_ref);
	CM2SpeedPID.ref = 27*(-ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 );//+ ChassisSpeedRef.rotate_ref);
	CM3SpeedPID.ref = 27*(-ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 );//+ ChassisSpeedRef.rotate_ref);
	CM4SpeedPID.ref = 27*( ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 );//+ ChassisSpeedRef.rotate_ref);
/*
	CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*10+ ChassisSpeedRef.left_right_ref*10;
	CM2SpeedPID.ref = -ChassisSpeedRef.forward_back_ref*10 + ChassisSpeedRef.left_right_ref*10 ;
	CM3SpeedPID.ref = -ChassisSpeedRef.forward_back_ref*10 - ChassisSpeedRef.left_right_ref*10 ;
	CM4SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*10 - ChassisSpeedRef.left_right_ref*10 ;
 */
	//每1ms进中断，每4ms进行底盘控制 CM1Encoder.filter_rate;是最近四次编码器差值的平均值，即4ms一次的速度的平均值
	
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
	
	//进行PID计算，RM把这部分封装起来了
	
	CM1SpeedPID.Calc(&CM1SpeedPID);
	CM2SpeedPID.Calc(&CM2SpeedPID);
	CM3SpeedPID.Calc(&CM3SpeedPID);
	CM4SpeedPID.Calc(&CM4SpeedPID);
	/*if(CM1SpeedPID.output>2000)
	{
		BEEP_ARR = 3033;
				BEEP_CH = 3033 / 2;
	}
	else
	{	
		BEEP_ARR = 0;
				BEEP_CH = 0;
	}*/
	
	//PID_Calc(&CM1SpeedPID);
	//PID_Calc(&CM1SpeedPID);
	//PID_Calc(&CM1SpeedPID);
	//PID_Calc(&CM1SpeedPID);
//如果检测到当前工作状态是 停止，校准，准备 状态，或者检测监控到 出现了严重错误 Is_Serious_Error() 则强制设置速度为0
	 if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE)    //|| dead_lock_flag == 1紧急停车，编码器校准，无控制输入时都会使底盘控制停止
	 {
		 Set_CM_Speed(&hcan1, 0,0,0,0);
	 }
	 else
	 {
		 //PID计算后的 CM1SpeedPID.output 乘以一个比例系数CHASSIS_SPEED_ATTENUATION 作为设定速度
		//Set_CM_Speed(CAN1,CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, 0, 0, 0);		
			//Set_CM_Speed(&hcan1,CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, 0, 0, 0);
	    //CAN_Send_Msg(&hcan1, 0, 0x200, 8);
		 Set_CM_Speed(&hcan1, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);		 
	 } 
}
//发射机构射击电机任务
int16_t pwm_ccr = 0;
void ShooterMControlLoop(void)	//波轮
{		
  //检测射击状态是否是 射击	
	if(GetShootState() == SHOOTING)
	{
		ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED;//PID_SHOOT_MOTOR_SPEED=10 	
	}
	else
	{
		ShootMotorSpeedPID.ref = 0;		
	}
	
	//ShootMotorSpeedPID.fdb = GetQuadEncoderDiff();   //裁判系统会利用反馈的速度 检测是否射弹的频率太快
	ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);
	//根据反馈速度和设定速度 进行PID计算得到PWM赋值给定时器
	PWM3 = ShootMotorSpeedPID.output;	
}

static uint32_t time_tick_1ms = 0;//1ms定时计数，系统会在指定的毫秒时刻进行不同的控制
float last_ref=0;
//控制任务，放在timer6 1ms定时中断中执行
void Control_Task(void)
{
	time_tick_1ms++;//每进一次中断就计数一次
	
	WorkStateFSM();//每1ms进中断都要判断是否需要进行状态切换
	
	WorkStateSwitchProcess();//当前系统的状态与上次系统的状态不同，并且当前系统的状态是准备状态就需要重新初始化控制任务和遥控器了
	
	//在前100ms内进行四元数的初始化
//	if(time_tick_1ms <100)	
//	{
//		//启动后根据磁力计的数据初始化四元数
//		Init_Quaternion();
//	}
	
	//平台稳定启动后，复位陀螺仪模块
//	if(time_tick_1ms == PREPARE_TIME_TICK_MS/2)// PREPARE_TIME_TICK_MS=4000
//	{
//		GYRO_RST();//云台偏航角的数据来源，贴在云台上面的一个小正方形，与MCU通过CAN1通信
//	}
//		
	//step 1: 云台控制
	GimbalYawControlModeSwitch();   //YAW工作模式切换处理，得到位置环的设定值和给定值
	//下三句
	
	GMPitchControlLoop();//Pitch控制
	GMYawControlLoop();//Yaw控制
	SetGimbalMotorOutput();
	
	//CalibrateLoop();   //校准任务，当接收到校准命令后才有效执行，否则直接跳过
	//chassis motor control
	if(time_tick_1ms%4 == 0)         //motor control frequency 4ms
	{
		//监控任务
		//SuperviseTask();    
		//底盘控制任务
		CMControlLoop();			 
		ShooterMControlLoop();       //发射机构控制任务
	}
}
/**********************************************************
*工作状态切换状态机,与1ms定时中断同频率
**********************************************************/
/*
typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态，底盘平移运动
    NORMAL_STATE,			//无输入状态
    STOP_STATE,        	//停止运动状态，遥控器S2拨到最下面，遥控器S2的中间是键鼠控制模式
    CALI_STATE,    			//校准状态，上位机进行校准
}WorkState_e;
系统运行的时候只会在这五种状态中切换
遥控器右手的S2，最上面是NORMAL_STATE,	中间是键鼠控制模式，最下面是STOP_STATE
遥控器左手边S1，最上面是拨轮转，中间是停止发射机构，最下面是摩擦轮
*/



void WorkStateFSM(void)
{
	lastWorkState = workState;//把此时的工作状态覆盖到上一次的工作状态
	
	switch(workState)
	{
		//准备状态
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP )//如果从遥控S2传回的控制模式是 STOP或者 出现严重的错误，系统的状态就转换成STOP
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())//如果是校准模式，则进行校准状态   
//			{
//				workState = CALI_STATE;
//			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)//如果当前系统时间大于系统初始化时间后，工作状态转化成NORMAL_STATE
			{
				workState = NORMAL_STATE;
			}			
		}break;
		
		//正常状态
		case NORMAL_STATE: 
		{
			if(GetInputMode() == STOP)//如果从遥控S2传回的控制模式是 STOP或者 出现严重的错误，系统的状态就转换成STOP
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())//如果是校准模式，则进行校准状态    
//			{
//				workState = CALI_STATE;
//			}
			else if((!IsRemoteBeingAction() /*||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC)*/) && GetShootState() != SHOOTING)//如果（没有遥控器输入量或者检测到遥控器丢失）&&（不在射击状态）系统进入备用状态
			{
				workState = STANDBY_STATE;      
			}			
		}break;
		
		//备用状态
		case STANDBY_STATE:     
		{
			if(GetInputMode() == STOP )//如果从遥控S2传回的控制模式是 STOP或者 出现严重的错误，系统的状态就转换成STOP
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())//如果是校准模式，则进行校准状态      
//			{
//				workState = CALI_STATE;
//			}
			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)//如果有遥控器输入量 或者 炮管在射击 或者 摩擦轮在转动，系统进入正常状态
			{
				workState = NORMAL_STATE;
			}				
		}break;
		
		//停止状态
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP )//如果遥控器S2的状态不是停止 并且没有检测监控到出现重大错误就进入准备状态
			{
				workState = PREPARE_STATE;   
			}
		}break;
		
		//校准状态
//		case CALI_STATE:      
//		{
//			if(GetInputMode() == STOP )//如果从遥控S2传回的控制模式是 STOP或者 出现严重的错误，系统的状态就转换成STOP
//			{
//				workState = STOP_STATE;
//			}
//		}break;	    
		default:
		{
			
		}
	}	
}

static void WorkStateSwitchProcess(void)
{
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	//当前系统的状态与上次系统的状态不同，并且当前系统的状态是准备状态就需要重新初始化控制任务和遥控器了
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		//初始化控制任务
		ControtLoopTaskInit();
	
		//遥控器初始化，检测是否变成键盘鼠标模式
		RemoteTaskInit();
	}
}

/*
************************************************************************************************************************
*Name        : GimbalYawControlModeSwitch
* Description: This function process the yaw angle ref and fdb according to the WORKSTATE.
* Arguments  : void     
* Returns    : void
* Note(s)    : 1) from NORMAL to STANDBY it need to delay a few seconds to wait for the IMU to be steady.  
                  STATE_SWITCH_DELAY_TICK represents the delay time.
************************************************************************************************************************
*/
//云台yaw轴 模式 控制程序，这个IMU不是MPU6050而是控制YAW轴电机上的电子罗盘这个IMU
void GimbalYawControlModeSwitch(void)
{
	static uint8_t normalFlag = 0;   //正常工作模式标志
	static uint8_t standbyFlag = 1;  //IMU工作模式标志
	static uint32_t modeChangeDelayCnt = 0;
	static float angleSave = 0.0f;    //用于切换模式时保存切换前的角度值，用于角度给定值切换
	switch(GetWorkState())
	{
		//准备状态
		case PREPARE_STATE:   //启动过程，加入斜坡
		{
			GMYPositionPID.ref = 0.0f;//启动过程的设定值
			//GMYPositionPID.fdb = -GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);//云台的yaw轴电机编码器反馈值
			//angleSave = ZGyroModuleAngle;	//云台上的电子罗盘测出的角度，储存到angleSave这个变量中	
		}break;
		
		//正常状态
		case NORMAL_STATE:
		{
			if(standbyFlag == 1)//如果IMU在工作
			{
				standbyFlag = 0;
				normalFlag = 1;
				
				//修改设定值为STANDBY状态下记录的最后一个ZGYROMODULEAngle值
				GimbalRef.yaw_angle_dynamic_ref = angleSave;   //云台上的电子罗盘把YAW角度给GimbalRef.yaw_angle_dynamic_ref 因为在转动的过程中，YAW角在不断的运动，所以叫做动态的
				
				modeChangeDelayCnt = 0;   //delay清零
			}
			
			GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //设定给定值，让底盘随着YAW角变化而旋转
			//GMYPositionPID.fdb = ZGyroModuleAngle; 					//设定反馈值
			//angleSave = yaw_angle;   //时刻保存IMU的值用于从NORMAL向STANDBY模式切换，把MPU6050的Yaw角赋值给angleSave
		}break;
		
		//备用状态
		case STANDBY_STATE:   //IMU模式，没有任何输入，云台此时还要控制自己的炮管保持一定的角度
		{
			modeChangeDelayCnt++;
			if(modeChangeDelayCnt < STATE_SWITCH_DELAY_TICK)    //STATE_SWITCH_DELAY_TICK=1000000 delay的这段时间与NORMAL_STATE一样
			{
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //设定给定值
				//GMYPositionPID.fdb = ZGyroModuleAngle; 					//设定反馈值
				//angleSave = yaw_angle;
			}
			else     //delay时间到，切换模式到IMU
			{ 
				if(normalFlag == 1)   //修改模式标志
				{
					normalFlag = 0;
					standbyFlag = 1;
					GimbalRef.yaw_angle_dynamic_ref = angleSave;    //保存的是delay时间段内保存的
				}
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //设定给定值
				//GMYPositionPID.fdb = yaw_angle; 					//设定反馈值	
			//	angleSave = ZGyroModuleAngle;           //IMU模式时，保存ZGyro的值供模式切换时修改给定值使用						
			}
		}break;
		
		//停止和校准状态不进行云台YAW控制
		case STOP_STATE:    //停止工作模式
		{
			
		}break;
		case CALI_STATE:    //校准模式
		{
			
		}break;
	}	
}

//云台pitch轴控制程序
void GMPitchControlLoop(void)
{
	//云台Pitch位置PID赋值
	GMPPositionPID.kp = 0.9;//PITCH_POSITION_KP_DEFAULTS + PitchPositionSavedPID.kp_offset;
	GMPPositionPID.ki = 0;//PITCH_POSITION_KI_DEFAULTS + PitchPositionSavedPID.ki_offset;
	GMPPositionPID.kd = 0;//PITCH_POSITION_KD_DEFAULTS + PitchPositionSavedPID.kd_offset;
	
  //云台Pitch速度PID赋值	
	GMPSpeedPID.kp = PITCH_SPEED_KP_DEFAULTS;// + PitchSpeedSavedPID.kp_offset;
	GMPSpeedPID.ki = PITCH_SPEED_KI_DEFAULTS;// + PitchSpeedSavedPID.ki_offset;
	GMPSpeedPID.kd = PITCH_SPEED_KD_DEFAULTS;// + PitchSpeedSavedPID.kd_offset;
	
	//设定指定位置参数
	GMPPositionPID.ref = GimbalRef.pitch_angle_dynamic_ref;
	GMPPositionPID.fdb = GMPitchEncoder.ecd_angle; //* GMPitchRamp.Calc(&GMPitchRamp);    //加入斜坡函数
	GMPPositionPID.Calc(&GMPPositionPID);   //得到pitch轴位置环输出控制量

	//pitch speed control
//	GMPSpeedPID.ref = GMPPositionPID.output;
	//GMPSpeedPID.fdb = MPU6050_Real_Data.Gyro_Y;
//	GMPSpeedPID.Calc(&GMPSpeedPID);
}

//云台Yaw轴控制程序
void GMYawControlLoop(void)
{
	//云台Yaw位置PID赋值
	GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS; //+ YawPositionSavedPID.kp_offset;//  gAppParamStruct.YawPositionPID.kp_offset;  //may be bug if more operation  done
	GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS; //+ YawPositionSavedPID.ki_offset;
	GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS; //+ YawPositionSavedPID.kd_offset;
	 
	//云台Yaw速度PID赋值	
	GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS; //+ YawSpeedSavedPID.kp_offset;
	GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS; //+ YawSpeedSavedPID.ki_offset;
	GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS; //+ YawSpeedSavedPID.kd_offset;
		
	//设定指定位置参数
	GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //设定给定值
	GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
	GMYPositionPID.Calc(&GMYPositionPID);
	
	//yaw speed control
  //GMYSpeedPID.ref = GMYPositionPID.output;
	//GMYSpeedPID.fdb = MPU6050_Real_Data.Gyro_Z;
//	GMYSpeedPID.Calc(&GMYSpeedPID);			
}

//云台控制输出,把控制量通过CAN通信给电机，使电机转动到指定位置
void SetGimbalMotorOutput(void)
{
	//检测监控到当前系统状态是停止,校准或者发生重大错误时强制把云台置0 						
	if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE)  
	{
		Set_Gimbal_Current(&hcan1, 0, 0);     //yaw + pitch			
	}
	else
	{		
		//把PID计算得到的输出量通过CAN1发送给电机，从而控制YAW和PITCH
	  Set_Gimbal_Current(&hcan1, -(int16_t)GMYPositionPID.output, -(int16_t)GMPPositionPID.output);     //yaw + pitch			
	}		
}


//控制任务初始化程序，当工作状态发生变化，变成了 准备 时候需要用此段代码初始化
void ControtLoopTaskInit(void)
{
	//计数初始化，中断中的计数清零
	time_tick_1ms = 0;   

	//用于从flash中读取校准数据，初始化各个参数的校准值
	//AppParamInit();
	
	//校准后参数偏差值初始化
	//Sensor_Offset_Param_Init(&gAppParamStruct);
	
	//设置工作模式为准备阶段
	SetWorkState(PREPARE_STATE);
	
	//斜坡初始化
//	GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
//	GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
//	GMPitchRamp.ResetCounter(&GMPitchRamp);
//	GMYawRamp.ResetCounter(&GMYawRamp);
	
	
	//云台给定角度初始化
//	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
//	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
    
	//监控任务初始化
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
//  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_NOCALI));
    
	//PID初始化
//	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
//	GMPPositionPID.Reset(&GMPPositionPID);
//	GMPSpeedPID.Reset(&GMPSpeedPID);
//	GMYPositionPID.Reset(&GMYPositionPID);
//	GMYSpeedPID.Reset(&GMYSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
