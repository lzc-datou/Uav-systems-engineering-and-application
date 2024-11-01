#include "control.h"

float pitch, roll, yaw;
float pitch_des, roll_des, yaw_des;
int Motor_INIT, Motor_MAX;

//---------------------------------------------------------------------------------------------------
// Control variants
float Kp_OuterLoop_Pitch, Ki_OuterLoop_Pitch, Kd_OuterLoop_Pitch;			// Outer loop PID para
float Kp_InnerLoop_Pitch, Ki_InnerLoop_Pitch, Kd_InnerLoop_Pitch;			// Inner loop PID para

float Angle_Rad_Pitch, Angular_Velocity_Rad_Pitch;		// Current angle & angluar velocity // radius
float Angle_Rad_Last_Pitch = 0;		// Last time angle storage // radius
float Angle_Rad_Acc_Pitch;		// Angle integral // radius
float Angle_Rad_Des_Pitch;

float Output_OuterLoop_Pitch;		// Outer loop output - Angular velocity expectation

float Angular_Velocity_Rad_Acc_Pitch;		// Angular velocity integral // radius
float Angular_Velocity_Rad_Last_Pitch = 0;	// Last time angular velocity
float Output_OuterLoop_Last_Pitch = 0;			// Last time outerloop output - Angular velocity expectation

float f_ut_Pitch;								// Control quantity output - Inner loop output
float f_Lambda_Pitch;						// Control coefficient
float f_Lock_Pitch;							// Yaw lock pitch angle

////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Kp_OuterLoop_Roll, Ki_OuterLoop_Roll, Kd_OuterLoop_Roll;			// Outer loop PID para
float Kp_InnerLoop_Roll, Ki_InnerLoop_Roll, Kd_InnerLoop_Roll;			// Inner loop PID para

float Angle_Rad_Roll, Angular_Velocity_Rad_Roll;		// Current angle & angluar velocity // radius
float Angle_Rad_Last_Roll = 0;		// Last time angle storage // radius
float Angle_Rad_Acc_Roll;		// Angle integral // radius
float Angle_Rad_Des_Roll;

float Output_OuterLoop_Roll;		// Outer loop output - Angular velocity expectation

float Angular_Velocity_Rad_Acc_Roll;		// Angular velocity integral // radius
float Angular_Velocity_Rad_Last_Roll = 0;	// Last time angular velocity
float Output_OuterLoop_Last_Roll = 0;			// Last time outerloop output - Angular velocity expectation

float f_ut_Roll;								// Control quantity output - Inner loop output
float f_Lambda_Roll;						// Control coefficient
float f_Lock_Roll;							// Yaw lock roll angle


// Control variants
float Kp_OuterLoop_Yaw, Ki_OuterLoop_Yaw, Kd_OuterLoop_Yaw;			// Outer loop PID para
float Kp_InnerLoop_Yaw, Ki_InnerLoop_Yaw, Kd_InnerLoop_Yaw;			// Inner loop PID para

float Angle_Rad_Yaw, Angular_Velocity_Rad_Yaw;		// Current angle & angluar velocity // radius
float Angle_Rad_Last_Yaw = 0;		// Last time angle storage // radius
float Angle_Rad_Acc_Yaw;		// Angle integral // radius
float Angle_Rad_Des_Yaw;

float Output_OuterLoop_Yaw;		// Outer loop output - Angular velocity expectation

float Angular_Velocity_Rad_Acc_Yaw;		// Angular velocity integral // radius
float Angular_Velocity_Rad_Last_Yaw = 0;	// Last time angular velocity
float Output_OuterLoop_Last_Yaw = 0;			// Last time outerloop output - Angular velocity expectation

float f_ut_Yaw;								// Control quantity output - Inner loop output
float f_Lambda_Yaw;						// Control coefficient
float f_Lambda_Yaw_Co;				// Yaw lock ratio counterback


float PWM_1, PWM_2, PWM_3, PWM_4;

float CCR_Pitch_Motor_1,CCR_Pitch_Motor_2,CCR_Pitch_Motor_3,CCR_Pitch_Motor_4;
float CCR_Roll_Motor_1,CCR_Roll_Motor_2,CCR_Roll_Motor_3,CCR_Roll_Motor_4;
float CCR_Yaw_Motor_1,CCR_Yaw_Motor_2,CCR_Yaw_Motor_3,CCR_Yaw_Motor_4;

long Set_Width;

///////////////////////////////////////////////////////////////////////////////////
//////////////////////// CONTROL PARA INITIALIZATION //////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void PID_Init(void)
{
	Motor_INIT = 3600;
	Motor_MAX = 4200;
	
	CCR_Pitch_Motor_1 = Motor_INIT;
	CCR_Pitch_Motor_2 = Motor_INIT;
	CCR_Pitch_Motor_3 = Motor_INIT;
	CCR_Pitch_Motor_4 = Motor_INIT;
	
	CCR_Roll_Motor_1 = Motor_INIT;
	CCR_Roll_Motor_2 = Motor_INIT;
	CCR_Roll_Motor_3 = Motor_INIT;
	CCR_Roll_Motor_4 = Motor_INIT;

	CCR_Yaw_Motor_1 = Motor_INIT;
	CCR_Yaw_Motor_2 = Motor_INIT;
	CCR_Yaw_Motor_3 = Motor_INIT;
	CCR_Yaw_Motor_4 = Motor_INIT;
	
	Kp_OuterLoop_Pitch = 6;
	Ki_OuterLoop_Pitch = 0.1;		
	Kd_OuterLoop_Pitch = 0.0;
	
  Kp_InnerLoop_Pitch = 4;
	Ki_InnerLoop_Pitch = 0.1;		
	Kd_InnerLoop_Pitch = 0.1;
	
	Angle_Rad_Acc_Pitch = 0.0;
	Angle_Rad_Last_Pitch = 0.0;
	Angular_Velocity_Rad_Acc_Pitch = 0.0;
	
	f_Lambda_Pitch = 15000.0;		//60
	
	////////////////////////////////////////////
	
	Kp_OuterLoop_Roll = 6;
	Ki_OuterLoop_Roll = 0.1;
	Kd_OuterLoop_Roll = 0.0;
	
  Kp_InnerLoop_Roll = 4;	
	Ki_InnerLoop_Roll = 0.1;
	Kd_InnerLoop_Roll = 0.1;	
	
	Angle_Rad_Acc_Roll = 0.0;
	Angle_Rad_Last_Roll = 0.0;
	Angular_Velocity_Rad_Acc_Roll = 0.0;
	
	f_Lambda_Roll = 15000;		//60
	
////////////////////////////////////////////
	
	Kp_OuterLoop_Yaw = 3.0;	
	Ki_OuterLoop_Yaw = 0.6;
	Kd_OuterLoop_Yaw = 0;
	
	Kp_InnerLoop_Yaw = 15;	
	Ki_InnerLoop_Yaw = 0.3;
	Kd_InnerLoop_Yaw = 0.3;
	
	Angle_Rad_Acc_Yaw = 0.0;
	Angle_Rad_Last_Yaw = 0.0;
	Angular_Velocity_Rad_Acc_Yaw = 0.0;
	
	f_Lambda_Yaw = 15000.0;	//60
	
	Set_Width = (Motor_INIT - Motor_PWM_MIN) * (Motor_INIT - Motor_PWM_MIN);
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////// MAIN PID CONTROL FUNCTION ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
///////////////////////// MAIN PID CONTROL FUNCTION ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void PID_Control(float filter_gyro_x, float filter_gyro_y, float filter_gyro_z)
{
	float f_Pos;
	float f_Neg;
	///////////////////////////////// Control law starts
	//  Add your own codes below
	
	///// Pitch control
	Angle_Rad_Pitch = pitch * 3.1416 / 180.0;
	Angular_Velocity_Rad_Pitch = filter_gyro_y;
	Angle_Rad_Des_Pitch = pitch_des * 3.1416 / 180.0;
	
	Angle_Rad_Acc_Pitch += 0.5*(Angle_Rad_Pitch + Angle_Rad_Last_Pitch)*CONTROL_INTER;
	Output_OuterLoop_Pitch = -Kp_OuterLoop_Pitch * Angle_Rad_Pitch-Ki_OuterLoop_Pitch*Angle_Rad_Acc_Pitch;
	
	Angular_Velocity_Rad_Acc_Pitch += 0.5 * (Angular_Velocity_Rad_Pitch - Output_OuterLoop_Pitch + Angular_Velocity_Rad_Last_Pitch - Output_OuterLoop_Last_Pitch) * CONTROL_INTER;
	
	f_ut_Pitch = - Kp_InnerLoop_Pitch * (Angular_Velocity_Rad_Pitch - Output_OuterLoop_Pitch)
								- Ki_InnerLoop_Pitch * Angular_Velocity_Rad_Acc_Pitch
								- Kd_InnerLoop_Pitch * (Angular_Velocity_Rad_Pitch - Output_OuterLoop_Pitch
								-Angular_Velocity_Rad_Last_Pitch+ Output_OuterLoop_Last_Pitch) / CONTROL_INTER;
	f_ut_Pitch *= f_Lambda_Pitch;
	
	f_Pos = Set_Width + f_ut_Pitch;
	f_Neg = Set_Width - f_ut_Pitch;
	value_limit(f_Pos, 0, f_Pos);
	value_limit(f_Neg, 0, f_Neg);
	
	CCR_Pitch_Motor_2 = sqrt(f_Pos) + Motor_PWM_MIN;
	CCR_Pitch_Motor_3 = sqrt(f_Pos) + Motor_PWM_MIN;
	CCR_Pitch_Motor_1 = sqrt(f_Neg) + Motor_PWM_MIN;
	CCR_Pitch_Motor_4 = sqrt(f_Neg) + Motor_PWM_MIN;

///// Roll control
	Angle_Rad_Roll = roll * 3.1416 / 180.0;
	Angular_Velocity_Rad_Roll = filter_gyro_x;
	Angle_Rad_Des_Roll = roll_des * 3.1416 / 180.0;
	
	
	Angle_Rad_Acc_Roll += 0.5*(Angle_Rad_Roll + Angle_Rad_Last_Roll)*CONTROL_INTER;
	Output_OuterLoop_Roll = -Kp_OuterLoop_Roll * Angle_Rad_Roll-Ki_OuterLoop_Roll*Angle_Rad_Acc_Roll;
	
	Angular_Velocity_Rad_Acc_Roll += 0.5 * (Angular_Velocity_Rad_Roll - Output_OuterLoop_Roll + Angular_Velocity_Rad_Last_Roll - Output_OuterLoop_Last_Roll) * CONTROL_INTER;
	
	f_ut_Roll = - Kp_InnerLoop_Roll * (Angular_Velocity_Rad_Roll - Output_OuterLoop_Roll)
								- Ki_InnerLoop_Roll * Angular_Velocity_Rad_Acc_Roll
								- Kd_InnerLoop_Roll * (Angular_Velocity_Rad_Roll - Output_OuterLoop_Roll
								-Angular_Velocity_Rad_Last_Roll+ Output_OuterLoop_Last_Roll) / CONTROL_INTER;
	f_ut_Roll *= f_Lambda_Roll;
	
	f_Pos = Set_Width + f_ut_Roll;
	f_Neg = Set_Width - f_ut_Roll;
	value_limit(f_Pos, 0, f_Pos);
	value_limit(f_Neg, 0, f_Neg);
	CCR_Roll_Motor_3 = sqrt(f_Neg) + Motor_PWM_MIN;
	CCR_Roll_Motor_1 = sqrt(f_Pos) + Motor_PWM_MIN;
	CCR_Roll_Motor_4 = sqrt(f_Neg) + Motor_PWM_MIN;
	CCR_Roll_Motor_2 = sqrt(f_Pos) + Motor_PWM_MIN;
	
///// Yaw control
	Angle_Rad_Yaw = yaw * 3.1416 / 180.0;
	Angular_Velocity_Rad_Yaw = filter_gyro_z;
	Angle_Rad_Des_Yaw = yaw_des * 3.1416 / 180.0;

	Angle_Rad_Acc_Yaw += 0.5*(Angle_Rad_Yaw + Angle_Rad_Last_Yaw)*CONTROL_INTER;
	Output_OuterLoop_Yaw = -Kp_OuterLoop_Yaw * Angle_Rad_Yaw-Ki_OuterLoop_Yaw*Angle_Rad_Acc_Yaw;
	
	Angular_Velocity_Rad_Acc_Yaw += 0.5 * (Angular_Velocity_Rad_Yaw - Output_OuterLoop_Yaw + Angular_Velocity_Rad_Last_Yaw - Output_OuterLoop_Last_Yaw) * CONTROL_INTER;
	
	f_ut_Yaw = - Kp_InnerLoop_Yaw * (Angular_Velocity_Rad_Yaw - Output_OuterLoop_Yaw)
								- Ki_InnerLoop_Yaw * Angular_Velocity_Rad_Acc_Yaw
								- Kd_InnerLoop_Yaw * (Angular_Velocity_Rad_Yaw - Output_OuterLoop_Yaw
								-Angular_Velocity_Rad_Last_Yaw+ Output_OuterLoop_Last_Yaw) / CONTROL_INTER;
	f_ut_Yaw *= f_Lambda_Yaw;
	   
	//  Add your own codes above


	
	//Æ«º½
	f_Pos = Set_Width + f_ut_Yaw;
	f_Neg = Set_Width - f_ut_Yaw;
	value_limit(f_Pos, 0, f_Pos);
	value_limit(f_Neg, 0, f_Neg);
	CCR_Yaw_Motor_2 = sqrt(f_Neg) + Motor_PWM_MIN;
	CCR_Yaw_Motor_1 = sqrt(f_Pos) + Motor_PWM_MIN;
	CCR_Yaw_Motor_3 = sqrt(f_Pos) + Motor_PWM_MIN;
	CCR_Yaw_Motor_4 = sqrt(f_Neg) + Motor_PWM_MIN;
	
	//×îÖÕ¿ØÖÆ
	PWM_1 = CCR_Pitch_Motor_1 + CCR_Roll_Motor_1 + CCR_Yaw_Motor_1 - 2 * Motor_INIT;
	PWM_2 = CCR_Pitch_Motor_2 + CCR_Roll_Motor_2 + CCR_Yaw_Motor_2 - 2 * Motor_INIT;
	PWM_3 = CCR_Pitch_Motor_3 + CCR_Roll_Motor_3 + CCR_Yaw_Motor_3 - 2 * Motor_INIT;
	PWM_4 = CCR_Pitch_Motor_4 + CCR_Roll_Motor_4 + CCR_Yaw_Motor_4 - 2 * Motor_INIT;

	Angle_Rad_Last_Pitch = Angle_Rad_Pitch;
	Angular_Velocity_Rad_Last_Pitch = Angular_Velocity_Rad_Pitch;
	Output_OuterLoop_Last_Pitch = Output_OuterLoop_Pitch;
	
	Angle_Rad_Last_Roll = Angle_Rad_Roll;
	Angular_Velocity_Rad_Last_Roll = Angular_Velocity_Rad_Roll;
	Output_OuterLoop_Last_Roll = Output_OuterLoop_Roll;
	
	Angle_Rad_Last_Yaw = Angle_Rad_Yaw;
	Angular_Velocity_Rad_Last_Yaw = Angular_Velocity_Rad_Yaw;
	Output_OuterLoop_Last_Yaw = Output_OuterLoop_Yaw;
	
	///// Evoke motors
	PID_Limit();
	
	M1 = PWM_1;
	M2 = PWM_2;
	M3 = PWM_3;
	M4 = PWM_4;
	
}

void PID_Limit(void)
{
	int PWM_LOW, PWM_HIGH;
	
	PWM_LOW = Motor_PWM_MIN + Motor_PWM_IDLE;
	PWM_HIGH = Motor_MAX;
	
	value_limit(PWM_1, PWM_LOW, PWM_HIGH);
	value_limit(PWM_2, PWM_LOW, PWM_HIGH);
	value_limit(PWM_3, PWM_LOW, PWM_HIGH);
	value_limit(PWM_4, PWM_LOW, PWM_HIGH);
	
}


