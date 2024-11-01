#include "i2c_MPU.h"
#include "delay.h"

//���̼���IIC�⺯���������������ļ�������ͷ�ļ�����·����������������ע�͵��ļ���Ϳ�����
//#include "i2c_MPU.h"
 //mpu_initialize();
//MPU6050_GetRawAccelGyro(data);
//printf("%d %d %d %d %d %d\r\n",data[0],data[1],data[2],data[3],data[4],data[5]);

short data[6];
unsigned char receive_data;//��һ���Ĵ������Զ������������
void mpu_initialize(void)
{
	I2C_Configuration();
	MPU6050_Initialize();
}          

void I2C_Configuration(void)
{
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(IIC_RCC, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin =IIC_IO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;       //�������Ŷ��� 4.7K ��������
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 =0xc0; //  STM32 �������ַ�������������ͬ����
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;  

    I2C_Init(IIC_Channel, &I2C_InitStructure);
    I2C_Cmd(IIC_Channel, ENABLE);
}
           
           
int MPU6050_Initialize()             //��ʼ������ ����ʵ����д 5���Ĵ���
{        
    MPU6050_I2C_ByteWrite(0xd0,0x80,MPU6050_RA_PWR_MGMT_1);  
		delay_ms(100);
	 MPU6050_I2C_ByteWrite(0xd0,0x00,MPU6050_RA_PWR_MGMT_1);
    MPU6050_I2C_ByteWrite(0xd0,0x07,MPU6050_RA_SMPLRT_DIV);         //����Ƶ�� 1000
   // MPU6050_I2C_ByteWrite(0xd0,0x06,MPU6050_RA_CONFIG);                 
    MPU6050_I2C_ByteWrite(0xd0,0,MPU6050_RA_ACCEL_CONFIG);     //���ٶ����� 2g
    MPU6050_I2C_ByteWrite(0xd0,3<<3,MPU6050_RA_GYRO_CONFIG);          //���ٶ����� 2000��/s
	
		MPU6050_I2C_BufferRead(0xd0,&receive_data,MPU_DEVICE_ID_REG, 1);
	if(receive_data==0x68)
	{
		MPU6050_I2C_ByteWrite(0xd0,0x01,MPU_PWR_MGMT1_REG);  
		MPU6050_I2C_ByteWrite(0xd0,0x00,MPU_PWR_MGMT2_REG);  
			
		MPU6050_I2C_ByteWrite(0xd0,2,MPU_SAMPLE_RATE_REG);
		MPU6050_I2C_ByteWrite(0xd0,1,MPU_CFG_REG);
 	}else return 1;
	return 0;
}
         
         
         
void MPU6050_I2C_ByteWrite(u8 slaveAddr, u8 pBuffer, u8 writeAddr)
{
  /* Send START condition */
  I2C_GenerateSTART(IIC_Channel, ENABLE);          //���Ϳ�ʼ�ź�
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(IIC_Channel, slaveAddr, I2C_Direction_Transmitter);          // ���� MPU6050 ��ַ��״̬��д��
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(IIC_Channel, writeAddr);                   //���� MPU6050�ڲ�ĳ����д�Ĵ�����ַ
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(IIC_Channel, pBuffer);                     //����Ҫд�������
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(IIC_Channel, ENABLE);          //���ͽ����ź�
}

void MPU6050_GetRawAccelGyro(short* AccelGyro)        //�����ٶ�ֵ �� ���ٶ�ֵ
{
    u8 tmpBuffer[14],i; 
    MPU6050_I2C_BufferRead(0xd0, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14); 
    /* Get acceleration */
    for(i=0; i<3; i++)                              
    AccelGyro[i]=(unsigned short)(((u16)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
     /* Get Angular rate */
    for(i=4; i<7; i++)                                             //�ڴ������¶ȼĴ���������Ҫ�¶�ֵ
    AccelGyro[i-1]=((s16)((u16)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);      
}
       
       
       
void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
  /* While the bus is busy */  
  while(I2C_GetFlagStatus(IIC_Channel, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(IIC_Channel, ENABLE);
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(IIC_Channel, slaveAddr, I2C_Direction_Transmitter); 
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(IIC_Channel, ENABLE);
  /* Send the MPU6050's internal address to write to */
  I2C_SendData(IIC_Channel, readAddr);
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

/* Send STRAT condition a second time */
  I2C_GenerateSTART(IIC_Channel, ENABLE);
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for read */
  I2C_Send7bitAddress(IIC_Channel, slaveAddr, I2C_Direction_Receiver);
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(IIC_Channel, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(IIC_Channel, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(IIC_Channel, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      *pBuffer = I2C_ReceiveData(IIC_Channel);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }
  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(IIC_Channel, ENABLE);
}

