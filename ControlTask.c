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
	void (*Calc)(struct PID_Regulator_t *pid);//����ָ��
	void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;
void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
*/
//��PID�ṹ����г�ʼ������ֵΪĬ��ֵ
PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;   //��̨����λ��PID
PID_Regulator_t GMPSpeedPID = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;//��̨�����ٶ�PID
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;			//��̨ƫ��λ��PID
PID_Regulator_t GMYSpeedPID = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;//��̨ƫ���ٶ�PID

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; //�����ĸ������תPID
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//���̵��1���ٶ�PID
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//���̵��2���ٶ�PID
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//���̵��3���ٶ�PID
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//���̵��4���ٶ�PID

PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //����λ��PID shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;//�����ٶ�PID

/*--------------------------------------------CTRL Variables----------------------------------------*/
/*
typedef enum
{
    PREPARE_STATE,     		//�ϵ���ʼ��״̬ 4s������
    STANDBY_STATE,			//��ֹ̨ͣ��ת״̬
    NORMAL_STATE,			//������״̬
    STOP_STATE,        	//ֹͣ�˶�״̬
    CALI_STATE,    			//У׼״̬
}WorkState_e;
*/
WorkState_e lastWorkState = PREPARE_STATE;//��һ�μ��ʱ��ϵͳ�����Ĺ���״̬
WorkState_e workState = PREPARE_STATE;//ϵͳ��ǰ�����Ĺ���״̬

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

//RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;//б�£���ӦҪ�����б��
//RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;//б�£���ӦҪ�����б��
	
/*
*********************************************************************************************************
*                                            FUNCTIONS 
*********************************************************************************************************
*/
//����ϵͳ�Ĺ���״̬
static void SetWorkState(WorkState_e state)
{
    workState = state;
}


//��ȡ��ǰϵͳ�����Ĺ���״̬
WorkState_e GetWorkState()
{
	return workState;
}


//���̿�������
void CMControlLoop(void)
{  
	//������ת�����㣬�˺��������˵��̻������̨ת���������
	if(GetWorkState()==PREPARE_STATE) //�����׶Σ����̲���ת
	{
		ChassisSpeedRef.rotate_ref = 0;	 
	}
	else
	{
		 //���̸����������תPID����
		 CMRotatePID.ref=0.02*(RC_CtrlData.rc.ch2-(int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		// CMRotatePID.fdb = GMYawEncoder.ecd_angle;
		 CMRotatePID.Calc(&CMRotatePID);   
		 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
	}
	//
	//���ң������ʧ��ǿ�ƽ��ٶ��趨ֵreset
//	if(Is_Lost_Error_Set(LOST_ERROR_RC))      
//	{
//		ChassisSpeedRef.forward_back_ref = 0;
//		ChassisSpeedRef.left_right_ref = 0;
//	}


	//���ڵ��������֣���������reference��Ϊ����������ǰ�����ң����������ľ仰�൱������δ֪�����ĸ����̡�������ĸ�ref�������ܹ�ͨ��ң������������ֵ
	//��Σ�����������ŵ����⣬�����ְ�װ�ķ�ʽ�йأ�O�ͣ�X�͵ȡ�
	//ȷ����ʽ��ʱ�򣬿��õ���ֱ�ߣ�����ƽ�ƣ��������ֱ��¼�ĸ����������ת��Ȼ���ж������������
	//�����ref����ң������ֱ��ֵ�����Ǿ������������ң������ֵ����Ϊ0������Ϊ������Ϊ��
	//���������������СPID��P	


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
	//ÿ1ms���жϣ�ÿ4ms���е��̿��� CM1Encoder.filter_rate;������Ĵα�������ֵ��ƽ��ֵ����4msһ�ε��ٶȵ�ƽ��ֵ
	
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
	
	//����PID���㣬RM���ⲿ�ַ�װ������
	
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
//�����⵽��ǰ����״̬�� ֹͣ��У׼��׼�� ״̬�����߼���ص� ���������ش��� Is_Serious_Error() ��ǿ�������ٶ�Ϊ0
	 if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE)    //|| dead_lock_flag == 1����ͣ����������У׼���޿�������ʱ����ʹ���̿���ֹͣ
	 {
		 Set_CM_Speed(&hcan1, 0,0,0,0);
	 }
	 else
	 {
		 //PID������ CM1SpeedPID.output ����һ������ϵ��CHASSIS_SPEED_ATTENUATION ��Ϊ�趨�ٶ�
		//Set_CM_Speed(CAN1,CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, 0, 0, 0);		
			//Set_CM_Speed(&hcan1,CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, 0, 0, 0);
	    //CAN_Send_Msg(&hcan1, 0, 0x200, 8);
		 Set_CM_Speed(&hcan1, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);		 
	 } 
}
//�����������������
int16_t pwm_ccr = 0;
void ShooterMControlLoop(void)	//����
{		
  //������״̬�Ƿ��� ���	
	if(GetShootState() == SHOOTING)
	{
		ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED;//PID_SHOOT_MOTOR_SPEED=10 	
	}
	else
	{
		ShootMotorSpeedPID.ref = 0;		
	}
	
	//ShootMotorSpeedPID.fdb = GetQuadEncoderDiff();   //����ϵͳ�����÷������ٶ� ����Ƿ��䵯��Ƶ��̫��
	ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);
	//���ݷ����ٶȺ��趨�ٶ� ����PID����õ�PWM��ֵ����ʱ��
	PWM3 = ShootMotorSpeedPID.output;	
}

static uint32_t time_tick_1ms = 0;//1ms��ʱ������ϵͳ����ָ���ĺ���ʱ�̽��в�ͬ�Ŀ���
float last_ref=0;
//�������񣬷���timer6 1ms��ʱ�ж���ִ��
void Control_Task(void)
{
	time_tick_1ms++;//ÿ��һ���жϾͼ���һ��
	
	WorkStateFSM();//ÿ1ms���ж϶�Ҫ�ж��Ƿ���Ҫ����״̬�л�
	
	WorkStateSwitchProcess();//��ǰϵͳ��״̬���ϴ�ϵͳ��״̬��ͬ�����ҵ�ǰϵͳ��״̬��׼��״̬����Ҫ���³�ʼ�����������ң������
	
	//��ǰ100ms�ڽ�����Ԫ���ĳ�ʼ��
//	if(time_tick_1ms <100)	
//	{
//		//��������ݴ����Ƶ����ݳ�ʼ����Ԫ��
//		Init_Quaternion();
//	}
	
	//ƽ̨�ȶ������󣬸�λ������ģ��
//	if(time_tick_1ms == PREPARE_TIME_TICK_MS/2)// PREPARE_TIME_TICK_MS=4000
//	{
//		GYRO_RST();//��̨ƫ���ǵ�������Դ��������̨�����һ��С�����Σ���MCUͨ��CAN1ͨ��
//	}
//		
	//step 1: ��̨����
	GimbalYawControlModeSwitch();   //YAW����ģʽ�л������õ�λ�û����趨ֵ�͸���ֵ
	//������
	
	GMPitchControlLoop();//Pitch����
	GMYawControlLoop();//Yaw����
	SetGimbalMotorOutput();
	
	//CalibrateLoop();   //У׼���񣬵����յ�У׼��������Чִ�У�����ֱ������
	//chassis motor control
	if(time_tick_1ms%4 == 0)         //motor control frequency 4ms
	{
		//�������
		//SuperviseTask();    
		//���̿�������
		CMControlLoop();			 
		ShooterMControlLoop();       //���������������
	}
}
/**********************************************************
*����״̬�л�״̬��,��1ms��ʱ�ж�ͬƵ��
**********************************************************/
/*
typedef enum
{
    PREPARE_STATE,     		//�ϵ���ʼ��״̬ 4s������
    STANDBY_STATE,			//��ֹ̨ͣ��ת״̬������ƽ���˶�
    NORMAL_STATE,			//������״̬
    STOP_STATE,        	//ֹͣ�˶�״̬��ң����S2���������棬ң����S2���м��Ǽ������ģʽ
    CALI_STATE,    			//У׼״̬����λ������У׼
}WorkState_e;
ϵͳ���е�ʱ��ֻ����������״̬���л�
ң�������ֵ�S2����������NORMAL_STATE,	�м��Ǽ������ģʽ����������STOP_STATE
ң�������ֱ�S1���������ǲ���ת���м���ֹͣ�����������������Ħ����
*/



void WorkStateFSM(void)
{
	lastWorkState = workState;//�Ѵ�ʱ�Ĺ���״̬���ǵ���һ�εĹ���״̬
	
	switch(workState)
	{
		//׼��״̬
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP )//�����ң��S2���صĿ���ģʽ�� STOP���� �������صĴ���ϵͳ��״̬��ת����STOP
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())//�����У׼ģʽ�������У׼״̬   
//			{
//				workState = CALI_STATE;
//			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)//�����ǰϵͳʱ�����ϵͳ��ʼ��ʱ��󣬹���״̬ת����NORMAL_STATE
			{
				workState = NORMAL_STATE;
			}			
		}break;
		
		//����״̬
		case NORMAL_STATE: 
		{
			if(GetInputMode() == STOP)//�����ң��S2���صĿ���ģʽ�� STOP���� �������صĴ���ϵͳ��״̬��ת����STOP
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())//�����У׼ģʽ�������У׼״̬    
//			{
//				workState = CALI_STATE;
//			}
			else if((!IsRemoteBeingAction() /*||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC)*/) && GetShootState() != SHOOTING)//�����û��ң�������������߼�⵽ң������ʧ��&&���������״̬��ϵͳ���뱸��״̬
			{
				workState = STANDBY_STATE;      
			}			
		}break;
		
		//����״̬
		case STANDBY_STATE:     
		{
			if(GetInputMode() == STOP )//�����ң��S2���صĿ���ģʽ�� STOP���� �������صĴ���ϵͳ��״̬��ת����STOP
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())//�����У׼ģʽ�������У׼״̬      
//			{
//				workState = CALI_STATE;
//			}
			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)//�����ң���������� ���� �ڹ������ ���� Ħ������ת����ϵͳ��������״̬
			{
				workState = NORMAL_STATE;
			}				
		}break;
		
		//ֹͣ״̬
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP )//���ң����S2��״̬����ֹͣ ����û�м���ص������ش����ͽ���׼��״̬
			{
				workState = PREPARE_STATE;   
			}
		}break;
		
		//У׼״̬
//		case CALI_STATE:      
//		{
//			if(GetInputMode() == STOP )//�����ң��S2���صĿ���ģʽ�� STOP���� �������صĴ���ϵͳ��״̬��ת����STOP
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
	//���������ģʽ�л���prapareģʽ��Ҫ��һϵ�в�����ʼ��
	//��ǰϵͳ��״̬���ϴ�ϵͳ��״̬��ͬ�����ҵ�ǰϵͳ��״̬��׼��״̬����Ҫ���³�ʼ�����������ң������
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		//��ʼ����������
		ControtLoopTaskInit();
	
		//ң������ʼ��������Ƿ��ɼ������ģʽ
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
//��̨yaw�� ģʽ ���Ƴ������IMU����MPU6050���ǿ���YAW�����ϵĵ����������IMU
void GimbalYawControlModeSwitch(void)
{
	static uint8_t normalFlag = 0;   //��������ģʽ��־
	static uint8_t standbyFlag = 1;  //IMU����ģʽ��־
	static uint32_t modeChangeDelayCnt = 0;
	static float angleSave = 0.0f;    //�����л�ģʽʱ�����л�ǰ�ĽǶ�ֵ�����ڽǶȸ���ֵ�л�
	switch(GetWorkState())
	{
		//׼��״̬
		case PREPARE_STATE:   //�������̣�����б��
		{
			GMYPositionPID.ref = 0.0f;//�������̵��趨ֵ
			//GMYPositionPID.fdb = -GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);//��̨��yaw��������������ֵ
			//angleSave = ZGyroModuleAngle;	//��̨�ϵĵ������̲���ĽǶȣ����浽angleSave���������	
		}break;
		
		//����״̬
		case NORMAL_STATE:
		{
			if(standbyFlag == 1)//���IMU�ڹ���
			{
				standbyFlag = 0;
				normalFlag = 1;
				
				//�޸��趨ֵΪSTANDBY״̬�¼�¼�����һ��ZGYROMODULEAngleֵ
				GimbalRef.yaw_angle_dynamic_ref = angleSave;   //��̨�ϵĵ������̰�YAW�Ƕȸ�GimbalRef.yaw_angle_dynamic_ref ��Ϊ��ת���Ĺ����У�YAW���ڲ��ϵ��˶������Խ�����̬��
				
				modeChangeDelayCnt = 0;   //delay����
			}
			
			GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //�趨����ֵ���õ�������YAW�Ǳ仯����ת
			//GMYPositionPID.fdb = ZGyroModuleAngle; 					//�趨����ֵ
			//angleSave = yaw_angle;   //ʱ�̱���IMU��ֵ���ڴ�NORMAL��STANDBYģʽ�л�����MPU6050��Yaw�Ǹ�ֵ��angleSave
		}break;
		
		//����״̬
		case STANDBY_STATE:   //IMUģʽ��û���κ����룬��̨��ʱ��Ҫ�����Լ����ڹܱ���һ���ĽǶ�
		{
			modeChangeDelayCnt++;
			if(modeChangeDelayCnt < STATE_SWITCH_DELAY_TICK)    //STATE_SWITCH_DELAY_TICK=1000000 delay�����ʱ����NORMAL_STATEһ��
			{
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //�趨����ֵ
				//GMYPositionPID.fdb = ZGyroModuleAngle; 					//�趨����ֵ
				//angleSave = yaw_angle;
			}
			else     //delayʱ�䵽���л�ģʽ��IMU
			{ 
				if(normalFlag == 1)   //�޸�ģʽ��־
				{
					normalFlag = 0;
					standbyFlag = 1;
					GimbalRef.yaw_angle_dynamic_ref = angleSave;    //�������delayʱ����ڱ����
				}
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //�趨����ֵ
				//GMYPositionPID.fdb = yaw_angle; 					//�趨����ֵ	
			//	angleSave = ZGyroModuleAngle;           //IMUģʽʱ������ZGyro��ֵ��ģʽ�л�ʱ�޸ĸ���ֵʹ��						
			}
		}break;
		
		//ֹͣ��У׼״̬��������̨YAW����
		case STOP_STATE:    //ֹͣ����ģʽ
		{
			
		}break;
		case CALI_STATE:    //У׼ģʽ
		{
			
		}break;
	}	
}

//��̨pitch����Ƴ���
void GMPitchControlLoop(void)
{
	//��̨Pitchλ��PID��ֵ
	GMPPositionPID.kp = 0.9;//PITCH_POSITION_KP_DEFAULTS + PitchPositionSavedPID.kp_offset;
	GMPPositionPID.ki = 0;//PITCH_POSITION_KI_DEFAULTS + PitchPositionSavedPID.ki_offset;
	GMPPositionPID.kd = 0;//PITCH_POSITION_KD_DEFAULTS + PitchPositionSavedPID.kd_offset;
	
  //��̨Pitch�ٶ�PID��ֵ	
	GMPSpeedPID.kp = PITCH_SPEED_KP_DEFAULTS;// + PitchSpeedSavedPID.kp_offset;
	GMPSpeedPID.ki = PITCH_SPEED_KI_DEFAULTS;// + PitchSpeedSavedPID.ki_offset;
	GMPSpeedPID.kd = PITCH_SPEED_KD_DEFAULTS;// + PitchSpeedSavedPID.kd_offset;
	
	//�趨ָ��λ�ò���
	GMPPositionPID.ref = GimbalRef.pitch_angle_dynamic_ref;
	GMPPositionPID.fdb = GMPitchEncoder.ecd_angle; //* GMPitchRamp.Calc(&GMPitchRamp);    //����б�º���
	GMPPositionPID.Calc(&GMPPositionPID);   //�õ�pitch��λ�û����������

	//pitch speed control
//	GMPSpeedPID.ref = GMPPositionPID.output;
	//GMPSpeedPID.fdb = MPU6050_Real_Data.Gyro_Y;
//	GMPSpeedPID.Calc(&GMPSpeedPID);
}

//��̨Yaw����Ƴ���
void GMYawControlLoop(void)
{
	//��̨Yawλ��PID��ֵ
	GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS; //+ YawPositionSavedPID.kp_offset;//  gAppParamStruct.YawPositionPID.kp_offset;  //may be bug if more operation  done
	GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS; //+ YawPositionSavedPID.ki_offset;
	GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS; //+ YawPositionSavedPID.kd_offset;
	 
	//��̨Yaw�ٶ�PID��ֵ	
	GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS; //+ YawSpeedSavedPID.kp_offset;
	GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS; //+ YawSpeedSavedPID.ki_offset;
	GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS; //+ YawSpeedSavedPID.kd_offset;
		
	//�趨ָ��λ�ò���
	GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //�趨����ֵ
	GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
	GMYPositionPID.Calc(&GMYPositionPID);
	
	//yaw speed control
  //GMYSpeedPID.ref = GMYPositionPID.output;
	//GMYSpeedPID.fdb = MPU6050_Real_Data.Gyro_Z;
//	GMYSpeedPID.Calc(&GMYSpeedPID);			
}

//��̨�������,�ѿ�����ͨ��CANͨ�Ÿ������ʹ���ת����ָ��λ��
void SetGimbalMotorOutput(void)
{
	//����ص���ǰϵͳ״̬��ֹͣ,У׼���߷����ش����ʱǿ�ư���̨��0 						
	if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE)  
	{
		Set_Gimbal_Current(&hcan1, 0, 0);     //yaw + pitch			
	}
	else
	{		
		//��PID����õ��������ͨ��CAN1���͸�������Ӷ�����YAW��PITCH
	  Set_Gimbal_Current(&hcan1, -(int16_t)GMYPositionPID.output, -(int16_t)GMPPositionPID.output);     //yaw + pitch			
	}		
}


//���������ʼ�����򣬵�����״̬�����仯������� ׼�� ʱ����Ҫ�ô˶δ����ʼ��
void ControtLoopTaskInit(void)
{
	//������ʼ�����ж��еļ�������
	time_tick_1ms = 0;   

	//���ڴ�flash�ж�ȡУ׼���ݣ���ʼ������������У׼ֵ
	//AppParamInit();
	
	//У׼�����ƫ��ֵ��ʼ��
	//Sensor_Offset_Param_Init(&gAppParamStruct);
	
	//���ù���ģʽΪ׼���׶�
	SetWorkState(PREPARE_STATE);
	
	//б�³�ʼ��
//	GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
//	GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
//	GMPitchRamp.ResetCounter(&GMPitchRamp);
//	GMYawRamp.ResetCounter(&GMYawRamp);
	
	
	//��̨�����Ƕȳ�ʼ��
//	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
//	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
    
	//��������ʼ��
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
    
	//PID��ʼ��
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
