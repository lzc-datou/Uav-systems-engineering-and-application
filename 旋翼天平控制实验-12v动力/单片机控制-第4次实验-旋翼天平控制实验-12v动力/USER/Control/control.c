#include "control.h"
#include "pid.h"

// IMU variables
float pitch, roll, yaw, gyroGx, gyroGy, gyroGz;
float scope_angle, scope_gyro;
short ax, ay, az, gx, gy, gz;

// Control variables
int n_L_Motor_Width=STOP_WIDTH, n_R_Motor_Width=STOP_WIDTH;
unsigned char Stop_Status = STOP;
int n_Motor_Counter;			// Motor speed control variable

// Control variables
float Kp_OuterLoop, Ki_OuterLoop, Kd_OuterLoop;			// Outer loop PID para
float Kp_InnerLoop, Ki_InnerLoop, Kd_InnerLoop;			// Inner loop PID para

float Angle_Rad, Angular_Velocity_Rad;		// Current angle & angluar velocity // radius
float Angle_Rad_Last;		// Last time angle storage // radius
float Angle_Rad_Acc;		// Angle integral // radius
float Angle_Des, Angle_Rad_Des, Angle_Des_Last;			// Angle expectation

float Output_OuterLoop;		// Outer loop output - Angular velocity expectation

float Angular_Velocity_Rad_Acc;		// Angular velocity integral // radius
float Angular_Velocity_Rad_Last;	// Last time angular velocity
float Output_OuterLoop_Last;			// Last time outerloop output - Angular velocity expectation

float f_ut;								// Control quantity output - Inner loop output
float f_Lambda;						// Control coefficient
float f_Control_Limit;		// Control quantity limit

float f_AV_Error;
float f_Int_Sat = 0.087;		// 5 degree
float f_Angle_Rad;

///////////////////////////////////////////////////////////////////////
// PID control parameter initialization
void PIDInit(void)
{
	Kp_OuterLoop = 7.0;	//7
	Ki_OuterLoop = 0.7;	//0.7
	Kd_OuterLoop = 0.1;	
	
  Kp_InnerLoop = 7.0;	//3.0
	Ki_InnerLoop = 0.0;
	Kd_InnerLoop = 0.35;	//0.35
	Angle_Rad_Acc = 0.0;
	Angle_Rad_Last = 0.0;
	Angular_Velocity_Rad_Acc = 0.0;
	
	f_Lambda = 60000.0;
	f_Control_Limit = 2000;
	
	Angle_Des = 0.0;
	
}

//////////////////////////////////////////////////////////////////////////////////
// Main control law
void RotorBalanceControlLoop(void)
{
	// Axis x = roll     left + / right -
	MPU_Get_Accelerometer(&ax, &ay, &az);
	MPU_Get_Gyroscope(&gx, &gy, &gz);
	gyroGx=(gx - f_gx_error)/16.384;
	gyroGy=(gy - f_gy_error)/16.384;
	gyroGz=(gz - f_gz_error)/16.384;
	
	LPFUpdate6axis();				// Butterworth lowpass filtering
	
	IMUupdate(Filters.GyroxLPF.output/57.3,
						Filters.GyroyLPF.output/57.3,
						Filters.GyrozLPF.output/57.3,
						Filters.AccxLPF.output,
						Filters.AccyLPF.output,
						Filters.AcczLPF.output,
						&pitch, &roll, &yaw);			// Mahony algorithm
	
	scope_angle = roll; // Red curve on touch screen£¬+-30¡ã
	scope_gyro  = Filters.GyroxLPF.output; // Yellow curve on touch screen£¬+-210¡ã/s	

	// IMU feedback variants
	// "roll" - Rotor level current angle
	// "Filters.GyroxLPF.output" - Rotor level current angular velocity
	// "Angle_Rad" - Rotor level current angle // radius
	// "Angular_Velocity_Rad" - Rotor level current angular velocity // radius
	Angle_Rad = roll * 3.1416 / 180.0;
	Angle_Rad_Des = Angle_Des * 3.1416 / 180.0;
	Angular_Velocity_Rad = Filters.GyroxLPF.output * 3.1416 / 180.0;
	
	/////////////////////////////////////////////////////////////////////////////
	//  Control Law Begins
	/////////////////////////////////////////////////////////////////////////////

		Angle_Rad_Acc += 0.5*(Angle_Rad + Angle_Rad_Last) * CONTROL_INTER;
    Output_OuterLoop = -Angle_Rad * Kp_OuterLoop -Angle_Rad_Acc * Ki_OuterLoop;
    
    Angular_Velocity_Rad_Acc += 0.5 * (Angular_Velocity_Rad - Output_OuterLoop + Angular_Velocity_Rad_Last - Output_OuterLoop_Last) * CONTROL_INTER;
    f_ut = -Kp_InnerLoop * (Angular_Velocity_Rad - Output_OuterLoop) - Kd_InnerLoop * (Angular_Velocity_Rad - Output_OuterLoop - Angular_Velocity_Rad_Last + Output_OuterLoop_Last) / CONTROL_INTER - Ki_InnerLoop * Angular_Velocity_Rad_Acc;
    f_ut *= f_Lambda;
    
    n_R_Motor_Width = sqrt((INIT_WIDTH_1 - STOP_WIDTH) * (INIT_WIDTH_1-STOP_WIDTH) + f_ut) + STOP_WIDTH;
    n_L_Motor_Width = sqrt((INIT_WIDTH_2 - STOP_WIDTH) * (INIT_WIDTH_2-STOP_WIDTH) - f_ut) + STOP_WIDTH;
    
    Angle_Rad_Last = Angle_Rad;
    Angular_Velocity_Rad_Last = Angular_Velocity_Rad;
    Output_OuterLoop_Last = Output_OuterLoop;

	/////////////////////////////////////////////////////////////////////////////
	//  Control Law Ends
	/////////////////////////////////////////////////////////////////////////////
	
	value_limit(n_L_Motor_Width, WIDTH_MIN, WIDTH_MAX);
	value_limit(n_R_Motor_Width, WIDTH_MIN, WIDTH_MAX);

	// Evoke motors
	if (Stop_Status == STOP) 
	{
		M1 = STOP_WIDTH;		
		M2 = STOP_WIDTH;
		Angle_Rad_Acc = 0;
		Angular_Velocity_Rad_Acc = 0;
	}
	else
	{
		M1 = n_L_Motor_Width;
		M2 = n_R_Motor_Width;
	}	
}

///////////////////////////////////////////////////////////////////////////////
// Motor speed control experiment
void Rotor_Motor_Loop(void)
{
	n_Motor_Counter ++;
	
	if (n_Motor_Counter < 400)
	{
		n_L_Motor_Width = STOP_WIDTH + (WIDTH_MAX - STOP_WIDTH) * 0.8 * n_Motor_Counter / 400;
		n_R_Motor_Width = n_L_Motor_Width;
	}
	
	else if (n_Motor_Counter < 600)
	{
		n_L_Motor_Width = STOP_WIDTH + (WIDTH_MAX - STOP_WIDTH) * 0.8;
		n_R_Motor_Width = n_L_Motor_Width;
	}
	
	else if (n_Motor_Counter < 800)
	{
		n_L_Motor_Width = STOP_WIDTH + (WIDTH_MAX - STOP_WIDTH) * 0.6;
		n_R_Motor_Width = n_L_Motor_Width;
	}		
	
	else if (n_Motor_Counter < 1000)
	{
		n_L_Motor_Width = STOP_WIDTH + (WIDTH_MAX - STOP_WIDTH) * 0.4;
		n_R_Motor_Width = n_L_Motor_Width;
	}		
	
	else if (n_Motor_Counter < 1200)
	{
		n_L_Motor_Width = STOP_WIDTH + (WIDTH_MAX - STOP_WIDTH) * 0.4 * (1200 - n_Motor_Counter) / 200;
		n_R_Motor_Width = n_L_Motor_Width;
	}		
	
	else
	{
		n_L_Motor_Width = STOP_WIDTH;
		n_R_Motor_Width = STOP_WIDTH;
		n_Motor_Counter = 0;
	}
	
// Evoke the motors
	if (Stop_Status == STOP) 
	{
		M1 = STOP_WIDTH;		
		M2 = STOP_WIDTH;
		n_L_Motor_Width = STOP_WIDTH;
		n_R_Motor_Width = STOP_WIDTH;
		n_Motor_Counter = 0;
	}
	else
	{
		M1 = n_L_Motor_Width;
		M2 = n_R_Motor_Width;
	}	
	
}

/////////////////////////////////////////////////
// Low pass filtering
void LPFUpdate6axis(void)
{
	Filters.GyroxLPF.input = gyroGx;
	Filters.GyroyLPF.input = gyroGy;
	Filters.GyrozLPF.input = gyroGz;
	Butterworth50HzLPF(&Filters.GyroxLPF);
	Butterworth50HzLPF(&Filters.GyroyLPF);
	Butterworth50HzLPF(&Filters.GyrozLPF);
	
	Filters.AccxLPF.input = ax;
	Filters.AccyLPF.input = ay;
	Filters.AcczLPF.input = az;
	Butterworth30HzLPF(&Filters.AccxLPF);
	Butterworth30HzLPF(&Filters.AccyLPF);
	Butterworth30HzLPF(&Filters.AcczLPF);
}

