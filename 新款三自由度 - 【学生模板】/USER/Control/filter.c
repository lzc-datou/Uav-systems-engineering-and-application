#include "filter.h"
#include "control.h"


float Kp 	=	5.0f;//10.0f					// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki	=	0.001f;//0.01f				// integral gain governs rate of convergence of gyroscope biases

//---------------------------------------------------------------------------------------------------
// Variable definitions
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// quaternion elements representing the estimated orientation
float q0_last = 1, q1_last = 0, q2_last = 0, q3_last = 0;
float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error

extern short ax, ay, az, gx, gy, gz;
extern float pitch, roll, yaw, gyroGx, gyroGy, gyroGz;

#define GAINBtw50hz   (3.450423889e+02f)
void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF)
{ 
  pLPF->xv[0] = pLPF->xv[1];
	pLPF->xv[1] = pLPF->xv[2];
	pLPF->xv[2] = pLPF->xv[3]; 
  pLPF->xv[3] = pLPF->input / GAINBtw50hz;
  pLPF->yv[0] = pLPF->yv[1]; 
	pLPF->yv[1] = pLPF->yv[2];
	pLPF->yv[2] = pLPF->yv[3]; 
  pLPF->yv[3] = (pLPF->xv[0] + pLPF->xv[3]) 
									+ 3 * (pLPF->xv[1] + pLPF->xv[2])
                  + (0.5320753683f * pLPF->yv[0])
									+ (-1.9293556691f * pLPF->yv[1])
                  + (2.3740947437f * pLPF->yv[2]);
        
	pLPF->output = pLPF->yv[3];
}


#define GAINBtw30Hz   1.278738361e+02f
void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF)
{
	pLPF->xv[0] = pLPF->xv[1];
	pLPF->xv[1] = pLPF->xv[2]; 
  pLPF->xv[2] = pLPF->input / GAINBtw30Hz;
  pLPF->yv[0] = pLPF->yv[1];
	pLPF->yv[1] = pLPF->yv[2]; 
  pLPF->yv[2] = (pLPF->xv[0] 
									+ pLPF->xv[2]) 
									+ 2 * pLPF->xv[1]
                  + (-0.7660066009f * pLPF->yv[0])
									+ (1.7347257688f * pLPF->yv[1]);
	
  pLPF->output = pLPF->yv[2];
}

void MovingAverageFilter(MovingAverageFilterTypeDef* pFilter)
{
	pFilter->buf[4] = pFilter->buf[3];
	pFilter->buf[3] = pFilter->buf[2];
	pFilter->buf[2] = pFilter->buf[1];
	pFilter->buf[1] = pFilter->buf[0];
	pFilter->buf[0] = pFilter->input;
	pFilter->output = (pFilter->buf[0]
											+ pFilter->buf[1]
											+ pFilter->buf[2]
											+ pFilter->buf[3]
											+ pFilter->buf[4])
										/5.0f;
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
	
// Then use moving average filtering to further process gyro data
	Filters.GyroxFinal.input = Filters.GyroxLPF.output;
	Filters.GyroyFinal.input = Filters.GyroyLPF.output;
	Filters.GyrozFinal.input = Filters.GyrozLPF.output;
	MovingAverageFilter(&Filters.GyroxFinal);
	MovingAverageFilter(&Filters.GyroyFinal);
	MovingAverageFilter(&Filters.GyrozFinal);
	
	Filters.AccxLPF.input = ax;
	Filters.AccyLPF.input = ay;
	Filters.AcczLPF.input = az;
	Butterworth30HzLPF(&Filters.AccxLPF);
	Butterworth30HzLPF(&Filters.AccyLPF);
	Butterworth30HzLPF(&Filters.AcczLPF);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);   
	if(norm >= 0.01)
	{	
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;  
	}
	else
	{
		ax=0; ay=0; az=0;
	}
	
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki*2.0*halfT;
	eyInt = eyInt + ey*Ki*2.0*halfT;
	ezInt = ezInt + ez*Ki*2.0*halfT;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
	q1 = q1_last + (q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
	q2 = q2_last + (q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
	q3 = q3_last + (q0_last*gz + q1_last*gy - q2_last*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if (norm >= 0.01)
	{
		q0 = q0 / norm;
		q1 = q1 / norm;
		q2 = q2 / norm;
		q3 = q3 / norm;
	}
	else
	{
		q0=1; q1=0; q2=0; q3=0;
	}

	// Calculate Euler angles
	*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
	*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
	*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;

	q0_last = q0;
	q1_last = q1;
	q2_last = q2;
	q3_last = q3;
	
}

// Restart Mahony algorithm
void IMUReset(void)
{
	q0 = 1; q1 = 0; q2 = 0; q3 = 0;
	q0_last = 1; q1_last = 0; q2_last = 0; q3_last = 0;

	exInt = 0; eyInt = 0;	ezInt = 0;

}

