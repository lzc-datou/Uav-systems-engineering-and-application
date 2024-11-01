#include "mpu6050.h"
#include "sys.h"
#include "delay.h"

#define MPU_STATIC_ERROR_SAMPLE_TIME	500.0

void MPU_IIC_Init(void)
{					     
  GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		
// PB6 -> SCL			PB7 -> SDA
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
}

u8 MPU_Init(void)
{ 
	u8 res;
	MPU_IIC_Init();
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	// Restart MPU6050
  delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	// Wakeup
	MPU_Set_Gyro_Fsr(3);										// Gyro scale, ¡À2000dps
	MPU_Set_Accel_Fsr(0);										// Acc scale, ¡À2g
	MPU_Set_Rate(500);											// Sampling rate: 500Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);
		MPU_Set_Rate(500);
 	}else return 1;
	return 0;
}

/////////////////////////////////////////////////////////////////////////////
// Function to acquire gyro raw date from MPU6050
/////////////////////////////////////////////////////////////////////////////
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
// Add your own codes below
// GYRO-X register address of MPU6050 is defined as MPU_GYRO_XOUTH_REG
// MPU6050 device address is defined as MPU_ADDR
// Use MPU_Read_Len() function to acquire data from serial resgisters from MPU6050
// Store raw date into *gx, *gy, *gz
  u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
  return res;
}

/////////////////////////////////////////////////////////////////////////////
// Function to acquire acc raw date from MPU6050
/////////////////////////////////////////////////////////////////////////////
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
// Add your own codes below
// ACC-X register address of MPU6050 is defined as MPU_ACCEL_XOUTH_REG
// MPU6050 device address is defined as MPU_ADDR
// Use MPU_Read_Len() function to acquire data from serial resgisters from MPU6050
// Store raw date into *ax, *ay, *az
  u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
  return res;

}

u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
  MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
	
  MPU_IIC_Send_Byte(reg);
  MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(data);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
  MPU_IIC_Stop();	 
	return 0;
}

u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
}

u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
}

u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);
}

u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
 	return MPU_Set_LPF(rate/2);
}

short MPU_Get_Temperature(void)
{
  u8 buf[2]; 
  short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
  raw=((u16)buf[0]<<8)|buf[1];  
  temp=36.53+((double)raw)/340;  
  return temp*100;;
}


u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
  MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);
		if(MPU_IIC_Wait_Ack())
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 

u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
  MPU_IIC_Send_Byte(reg);
  MPU_IIC_Wait_Ack();
  MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);
  MPU_IIC_Wait_Ack();
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);
		else *buf=MPU_IIC_Read_Byte(1);
		len--;
		buf++; 
	}    
  MPU_IIC_Stop();
	return 0;	
}

u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
  MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);
	MPU_IIC_Wait_Ack();
  MPU_IIC_Send_Byte(reg);
  MPU_IIC_Wait_Ack();
  MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);
  MPU_IIC_Wait_Ack();
	res=MPU_IIC_Read_Byte(0);
  MPU_IIC_Stop();
	return res;		
}

void MPU_IIC_Delay(void)
{
	delay_us(2);
}

void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;	  	  
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
 	MPU_IIC_SDA=0;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}	  

void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;
 	MPU_IIC_Delay();
	MPU_IIC_SCL=1; 
	MPU_IIC_SDA=1;
	MPU_IIC_Delay();							   	
}

u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();
	MPU_IIC_SDA=1;MPU_IIC_Delay();	   
	MPU_IIC_SCL=1;MPU_IIC_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL=0;
	return 0;  
} 

void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=0;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}

void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}					

void MPU_IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
	MPU_SDA_OUT(); 	    
  MPU_IIC_SCL=0;
  for(t=0;t<8;t++)
  {              
    MPU_IIC_SDA=(txd&0x80)>>7;
    txd<<=1; 	  
		MPU_IIC_SCL=1;
		MPU_IIC_Delay(); 
		MPU_IIC_SCL=0;	
		MPU_IIC_Delay();
  }	 
} 	   

u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();
  for(i=0;i<8;i++ )
	{
    MPU_IIC_SCL=0; 
    MPU_IIC_Delay();
		MPU_IIC_SCL=1;
    receive<<=1;
    if(MPU_READ_SDA)receive++;   
		MPU_IIC_Delay(); 
  }					 
  if (!ack)
		MPU_IIC_NAck();
  else
    MPU_IIC_Ack();
  return receive;
}

float f_gx_error, f_gy_error, f_gz_error;
void Get_Gyro_Static_Error(void)
{
	int i=0;
	int gyrox_add=0;
	int gyroy_add=0;
	int gyroz_add=0;
	short gx, gy, gz;
	
	for(i=0;i<MPU_STATIC_ERROR_SAMPLE_TIME;i++)
	{
		MPU_Get_Gyroscope(&gx,&gy,&gz);
		
		delay_ms(5);
		gyrox_add+=gx;
		gyroy_add+=gy;
		gyroz_add+=gz;
		
	}
	f_gx_error = (float)(gyrox_add/MPU_STATIC_ERROR_SAMPLE_TIME);
	f_gy_error = (float)(gyroy_add/MPU_STATIC_ERROR_SAMPLE_TIME);
	f_gz_error = (float)(gyroz_add/MPU_STATIC_ERROR_SAMPLE_TIME);
}
