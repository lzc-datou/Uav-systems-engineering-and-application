#include "pid.h"




/*
PIDTypeDef RollPID = {
	.Kp = 2.8,//2.8
	.Ki = 0.03,//0.03
	.Kd = 20,//20
	
	.UMax = 200,
	.UpMax = 200,
	.UiMax = 20,//20
	.UdMax = 40,//40
	.SumEMax = 1000,
	.EMin = 10,//10
};      

PIDTypeDef GyroxPID = {
	.Kp = 5,// 5
	.Ki = 0.05,//0.05
	.Kd = 100,// 100
	
	.UMax = 700,//700 
	.UpMax = 700,//700
	.UiMax = 30,//30
	.UdMax = 320,//320
	.SumEMax = 1000,
	.EMin = 10,//10
}; 

// 积分分离、积分限幅、P I D 输出限幅、总输出限幅
void PIDUpdate(PIDTypeDef *pPID)
{
	pPID->E = pPID->Des - pPID->FB;//计算当前偏差

	if(((pPID->U <= pPID->UMax && pPID->E > 0) || (pPID->U >= -pPID->UMax && pPID->E < 0)) \
		    && abs(pPID->E) < pPID->EMin)//积分分离
	{
		pPID->SumE += pPID->E;//计算偏差积分
	}
	value_limit( pPID->SumE , -pPID->SumEMax , pPID->SumEMax );//积分限幅
	pPID->Ui = pPID->Ki * pPID->SumE;
	value_limit( pPID->Ui , -pPID->UiMax , pPID->UiMax );
	
	pPID->Up = pPID->Kp * pPID->E;
	value_limit( pPID->Up , -pPID->UpMax , pPID->UpMax );
	
	pPID->Ud = pPID->Kd * ( pPID->E - pPID->PreE );
	value_limit( pPID->Ud , -pPID->UdMax , pPID->UdMax );
	
	pPID->U = pPID->Up + pPID->Ui + pPID->Ud; //位置式PID计算公式
  value_limit( pPID->U , -pPID->UMax , pPID->UMax );  // PID运算输出限幅
	
	pPID->PreE = pPID->E ;//保存本次偏差
}

float alpha= 0.75;
float zeta = 2.0 ;//0.75   1.7

// Kp非线性
void NLPIDUpdate(PIDTypeDef *pPID)
{
	pPID->E = pPID->Des - pPID->FB;//计算当前偏差

	if(((pPID->U <= pPID->UMax && pPID->E > 0) || (pPID->U >= -pPID->UMax && pPID->E < 0)) \
		    && abs(pPID->E) < pPID->EMin)//积分分离
	{
		pPID->SumE += pPID->E;//计算偏差积分
	}
	value_limit( pPID->SumE , -pPID->SumEMax , pPID->SumEMax );//积分限幅
	pPID->Ui = pPID->Ki * pPID->SumE;
	value_limit( pPID->Ui , -pPID->UiMax , pPID->UiMax );
	
	pPID->Up = pPID->Kp * Fal_ADRC(pPID->E,alpha,zeta);
	value_limit( pPID->Up , -pPID->UpMax , pPID->UpMax );
	
	pPID->Ud = pPID->Kd * ( pPID->E - pPID->PreE );
	value_limit( pPID->Ud , -pPID->UdMax , pPID->UdMax );
	
	pPID->U = pPID->Up + pPID->Ui + pPID->Ud; //位置式PID计算公式
  value_limit( pPID->U , -pPID->UMax , pPID->UMax );  //PID运算输出限幅
	
	pPID->PreE = pPID->E ;//保存本次偏差
}

void Clear_Structure(void)
{
	RollPID.SumE=0;
	GyroxPID.SumE=0;
}
*/





