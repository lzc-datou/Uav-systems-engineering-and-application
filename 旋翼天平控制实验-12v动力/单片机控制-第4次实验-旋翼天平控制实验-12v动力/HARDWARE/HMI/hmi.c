#include "hmi.h"
#include "pid.h"
#include "control.h"
#include "delay.h"
#include "motor.h"

/************
2019-11-4
新屏幕设置波特率  baud=512000       0xff 0xff 0xff
**/

extern unsigned int USART1_REC_CNT;
char Pointer_str[20];		// pointer

//为了保证字符串都能被发出去，将各步骤产生的指令拼接起来，一次发出去
//这个变量保存了上一个步骤产生指令后，串口发送缓冲区被填充的位置
unsigned short p_HMI_USART_TX_BUF;

#define  Wmax   27600			// 2300KV  12V

#define  Color_Angle_Omega_CYAN         34815
#define  Color_Width_W_YELLOW   	      65504
#define  Color_Init          						63488
#define  Color_Speed_Str   						  Color_Angle_Omega_CYAN

#define   BG_Pic_ID    0   //背景图片ID，刷新页面使用,HMI上位机中查看

#define   OBJNAME_ANGLE						"t5"
#define   OBJNAME_OMEGA						"t6"
#define   OBJNAME_WIDTH1  			  "t7"
#define   OBJNAME_WIDTH2 		 		  "t8"
#define   OBJNAME_SPEED_WIDTH  	  "t9"
#define   OBJNAME_SPEED_ROTOR			"t10"


unsigned char LastPage,isPageInited=0,StopRunConvertStatus=0;
unsigned short touchx,touchy,touchEvent=0;
unsigned char touchArea=0,UpdateWhich;
unsigned short pushdownPage,pushdownId,pushdownEvent=0;
unsigned short pushupPage,pushupId,pushupEvent=0,CurrentPageId;
unsigned char slidePos_Left=20, slidePos_Right=20;
unsigned int HMI_Rx_Int_Cnt,HMI_RX_Frame_Cnt,HMI_RX_Bytes,HMI_RX_Fail_Cnt;
u8 set_test;

///////////////////////////////////////////////////////////////////
// Screen functions
void HMI_Print_Init(void)
{
	p_HMI_USART_TX_BUF = 0;
	HMI_PicQ(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_Pic_ID);
	USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);
	delay_ms(1800);
	
	p_HMI_USART_TX_BUF = 0;
	HMI_printf_String(LCD_WIDTH/2-52, LCD_HEIGHT/2-16, 104, 16, Color_Init,
					"Initialzing..");
	HMI_printf_String(LCD_WIDTH/2-76, LCD_HEIGHT/2+16, 152, 16, Color_Init,
					"Don`t move the arm!!!..");
	USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);
	delay_ms(150);

}

void HMI_Update(void)
{
	if(CurrentPageId != LastPage)
	{
		isPageInited = 0;
	}
	
	if(LastPage == MOTOR && CurrentPageId != MOTOR)
	{
		StopRunConvertStatus = RUN2STOP;
		Stop_Status = STOP;
		n_L_Motor_Width = STOP_WIDTH;
		n_R_Motor_Width = STOP_WIDTH;
	}
	
	if(LastPage != MOTOR && CurrentPageId == MOTOR)
	{
		StopRunConvertStatus = RUN2STOP;
		Stop_Status = STOP;
		n_L_Motor_Width = STOP_WIDTH;
		n_R_Motor_Width = STOP_WIDTH;		
	}
	
	switch (CurrentPageId)
	{
		case 0:HMI_Update_Page0();break;
		case 1:HMI_Update_Page1(4);break;
		case 2:HMI_Update_Page2();break;
	}
	LastPage = CurrentPageId;
}

void HMI_RefreshPage(void)
{
	p_HMI_USART_TX_BUF = 0;
	HMI_Refresh_CurrentPage();
	USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);
	delay_ms(30);
}

void HMI_Update_Page0(void)
{
	float f_Pointer_Angle_1, f_Pointer_Angle_2;
	f_Pointer_Angle_1 = MOTOR_SPEED * 
						(n_L_Motor_Width - WIDTH_MIN) / (WIDTH_MAX - WIDTH_MIN) - 
						45.0;
	f_Pointer_Angle_2 = MOTOR_SPEED * 
						(n_R_Motor_Width - WIDTH_MIN) / (WIDTH_MAX - WIDTH_MIN) - 
						45.0;

	if(isPageInited == 0)
	{
//		HMI_RefreshPage();
		isPageInited = 1;
	}
	
	if(StopRunConvertStatus == RUN2STOP)
	{
		HMI_Run2Stop_Deal(0);
		StopRunConvertStatus = 0;
	}
	
	if(StopRunConvertStatus == STOP2RUN)
	{
		HMI_Stop2Run_Deal(0);
		StopRunConvertStatus = 0;
	}
	
// Reveal the balance's status
	p_HMI_USART_TX_BUF = 0;
	HMI_Draw_Pointer(-1.0*roll);
	HMI_Draw_Pointer_LR(f_Pointer_Angle_1, f_Pointer_Angle_2);
	HMI_Print_AngleOmega();
	HMI_Print_Width();
	if (LastPage != 0)
		HMI_Set_Slidebar(slidePos_Left, slidePos_Right);
	
	if(USART1_DMA_TX_CHANNEL->CNDTR == 0 )
		USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);	

}

void HMI_Update_Page1(unsigned char dt)
{
	static unsigned char cntPage1;
	
	if(isPageInited == 0)
	{
//		HMI_RefreshPage();
		isPageInited = 1;
	}
	
	if(cntPage1 >= dt)
	{
		p_HMI_USART_TX_BUF=0;
		
	//add 5,0,120   add 5,1,120       0-255
		HMI_Add_Curve(0, scope_angle, 30);
		HMI_Add_Curve(1, scope_gyro, 210);
		if(USART1_DMA_TX_CHANNEL->CNDTR == 0)
			USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);
		cntPage1=0;
	}
	cntPage1++;
	
}

void HMI_Update_Page2(void)
{
	float f_Pointer_Angle;
	int n_Rotor_Speed;
	
	f_Pointer_Angle = MOTOR_SPEED * 
						(n_L_Motor_Width - STOP_WIDTH) / (WIDTH_MAX - STOP_WIDTH) - 
						45.0;
	
	n_Rotor_Speed = MOTOR_KV * MOTOR_VOLTAGE * 
								((float)(WIDTH_MAX - STOP_WIDTH)/(FULL_WIDTH - STOP_WIDTH)) *
								((float) (n_L_Motor_Width - STOP_WIDTH) / (WIDTH_MAX - STOP_WIDTH));

	
	if(isPageInited == 0)
	{
//		HMI_RefreshPage();
		isPageInited = 1;
	}
	
	if(StopRunConvertStatus == RUN2STOP)
	{
		HMI_Run2Stop_Deal(0);
		StopRunConvertStatus = 0;
	}
	
	if(StopRunConvertStatus == STOP2RUN)
	{
		HMI_Stop2Run_Deal(0);
		StopRunConvertStatus = 0;
	}
	
// Reveal the motor's speed
	p_HMI_USART_TX_BUF = 0;
	HMI_Draw_Pointer(f_Pointer_Angle);
	HMI_Print_Speed(n_Rotor_Speed);

	if(USART1_DMA_TX_CHANNEL->CNDTR == 0 )
		USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);	

}


void HMI_Run2Stop_Deal(unsigned char page)
{
	delay_ms(50);
	/*
	if(page == 0)
	{
		p_HMI_USART_TX_BUF=0;			
		HMI_printf_String((LCD_WIDTH-48)/2,300,48,16,Color_Angle_Omega_CYAN,"已停车");
		if(USART1_DMA_TX_CHANNEL->CNDTR == 0)
			USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);
		p_HMI_USART_TX_BUF=0;	*/

/*		
		delay_ms(50);
		HMI_printf_String(  (LCD_WIDTH-48)/2,300,48,16,Color_Angle_Omega_CYAN,"已停车");
		if(USART1_DMA_TX_CHANNEL->CNDTR==0 )
			USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);
		p_HMI_USART_TX_BUF=0;	
	}*/
}

void HMI_Stop2Run_Deal(unsigned char page)
{
	delay_ms(50);
//	if(page == 0)
//		HMI_RefreshPage();
}

//z0.val=225
void HMI_Set_Pointer(unsigned short val)
{
	char SetPointerStr[80];
	Clear_String(SetPointerStr,sizeof(SetPointerStr));
	sprintf(SetPointerStr,"z0.val=%d",val);
	ADD_Instruction_To_BUF(SetPointerStr,strlen(SetPointerStr));
	
	Copy_BUF(SetPointerStr,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(SetPointerStr));
	p_HMI_USART_TX_BUF += strlen(SetPointerStr);
}

//h0.val=0
void HMI_Set_Slider(unsigned char val)
{
	char SetSliderStr[80];
	Clear_String(SetSliderStr,sizeof(SetSliderStr));
	sprintf(SetSliderStr,"h0.val=%d",val);
	ADD_Instruction_To_BUF(SetSliderStr,strlen(SetSliderStr));
	
	Copy_BUF(SetSliderStr,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(SetSliderStr));
	p_HMI_USART_TX_BUF += strlen(SetSliderStr);
}

void HMI_Refresh_CurrentPage(void)
{
	char RefreshPageStr[80];
	Clear_String(RefreshPageStr, sizeof(RefreshPageStr));
	sprintf(RefreshPageStr, "page %d", CurrentPageId);
	ADD_Instruction_To_BUF(RefreshPageStr, strlen(RefreshPageStr));
	
	Copy_BUF(RefreshPageStr,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(RefreshPageStr));
	p_HMI_USART_TX_BUF += strlen(RefreshPageStr);
}

void HMI_Print_AngleOmega(void)
{
	char printAngleStr[80], printOmegaStr[80];

//					p_HMI_USART_TX_BUF = 0;

	sprintf(printAngleStr," %.2f", roll);
	HMI_change_String(OBJNAME_ANGLE, printAngleStr);
/*	if(USART1_DMA_TX_CHANNEL->CNDTR == 0 )
		USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);	*/


//						p_HMI_USART_TX_BUF = 0;

	sprintf(printOmegaStr, " %.2f", Filters.GyroxLPF.output);
	HMI_change_String(OBJNAME_OMEGA, printOmegaStr);
//	if(USART1_DMA_TX_CHANNEL->CNDTR == 0 )
//		USART1_DMA_TX_Enable(p_HMI_USART_TX_BUF);		

}

void HMI_Print_Width(void)
{
	char printWidth1Str[80], printWidth2Str[80];
	
	sprintf(printWidth1Str, "%d", n_L_Motor_Width);
	HMI_change_String(OBJNAME_WIDTH1, printWidth1Str);
	
	sprintf(printWidth2Str, "%d", n_R_Motor_Width);
	HMI_change_String(OBJNAME_WIDTH2, printWidth2Str);
}

void HMI_Print_Speed(int n_Rotor_Speed)
{
	char printWidth1Str[80], printWidth2Str[80];
	
	sprintf(printWidth1Str, "%d", n_L_Motor_Width);
	HMI_change_String(OBJNAME_SPEED_WIDTH, printWidth1Str);
	
	sprintf(printWidth2Str, "%d", n_Rotor_Speed);
	HMI_change_String(OBJNAME_SPEED_ROTOR, printWidth2Str);
}

void HMI_RX_Deal(void)
{
	for(int i=0; i<USART1_REC_CNT; i++)
		HMI_RX_GetOneByte(HMI_USART_RXBUF[i]);
	HMI_Rx_Int_Cnt++;
	HMI_RX_Bytes+=USART1_REC_CNT;

}

char Change_String_str[80];
void HMI_change_String(char* objname, char* string_to_change)
{
	Clear_String(Change_String_str, sizeof(Change_String_str));
	sprintf(Change_String_str, "%s.txt=\"%s\"", objname, string_to_change);
	ADD_Instruction_To_BUF(Change_String_str, strlen(Change_String_str));
	
	Copy_BUF(Change_String_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Change_String_str));
	p_HMI_USART_TX_BUF += strlen(Change_String_str);
}

//add 5,0,120   add 5,1,120       0-255
char Add_Curve_str[80];
//#define  Curve_ID        4
void HMI_Add_Curve(unsigned char chx, float data, float range)
{
	unsigned char chData;
	float chDataf;
	
	chDataf = data/range*127.0 + Curve_MID;
	if(chDataf>255)chDataf=255;if(chDataf<0)chDataf=0;
	chData = chDataf;
	Clear_String(Add_Curve_str,sizeof(Add_Curve_str));
	sprintf(Add_Curve_str,"add 2,%d,%d",chx,chData);
	ADD_Instruction_To_BUF(Add_Curve_str,strlen(Add_Curve_str));
	
	Copy_BUF(Add_Curve_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Add_Curve_str));
	p_HMI_USART_TX_BUF += strlen(Add_Curve_str);
}

char Print_String_str[80];
void HMI_printf_String(u16 x,u16 y,u16 w,u16 h,u16 color,char* print_string)
{
	Clear_String(Print_String_str,sizeof(Print_String_str));
	sprintf(Print_String_str,"xstr %d,%d,%d,%d,0,%d,BLACK,0,0,3,\"%s\"",x,y,w,h,color,print_string);
	ADD_Instruction_To_BUF(Print_String_str,strlen(Print_String_str));
	
	Copy_BUF(Print_String_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Print_String_str));
	p_HMI_USART_TX_BUF += strlen(Print_String_str);
}

char PicQ_str[80];
void HMI_PicQ(u16 sx,u16 sy,u16 pic_w,u16 pic_h,u16 pic_id)
{
	Clear_String(PicQ_str,sizeof(PicQ_str));
	sprintf(PicQ_str,"picq %d,%d,%d,%d,%d",sx,sy,pic_w,pic_h,pic_id);
	ADD_Instruction_To_BUF(PicQ_str,strlen(PicQ_str));
	
	Copy_BUF(PicQ_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(PicQ_str));
	p_HMI_USART_TX_BUF += strlen(PicQ_str);
}

char Fill_Str[80];
void HMI_Fill(char* color,u16 x,u16 y,u16 w,u16 h)
{
	Clear_String(Fill_Str,sizeof(Fill_Str));
	sprintf(Fill_Str,"fill %d,%d,%d,%d,%s",x,y,w,h,color);
	ADD_Instruction_To_BUF(Fill_Str,strlen(Fill_Str));
	
	Copy_BUF(Fill_Str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Fill_Str));
	p_HMI_USART_TX_BUF += strlen(Fill_Str);
}

void HMI_Draw_Pointer(short theta)//330-360   0-30     if<0 +=360   转整数
{
	Clear_String(Pointer_str, sizeof(Pointer_str));
	if(theta < 0)
		theta += 360;
	sprintf(Pointer_str, "z0.val=%d", theta);
	ADD_Instruction_To_BUF(Pointer_str, strlen(Pointer_str));
	Copy_BUF(Pointer_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Pointer_str));
	p_HMI_USART_TX_BUF += strlen(Pointer_str);
}

void HMI_Draw_Pointer_LR(short theta1, short theta2)
{
	Clear_String(Pointer_str, sizeof(Pointer_str));
	if(theta1 < 0)
		theta1 += 360;
	sprintf(Pointer_str, "z1.val=%d", theta1);
	ADD_Instruction_To_BUF(Pointer_str, strlen(Pointer_str));
	Copy_BUF(Pointer_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Pointer_str));
	p_HMI_USART_TX_BUF += strlen(Pointer_str);
	
	Clear_String(Pointer_str, sizeof(Pointer_str));
	if(theta2 < 0)
		theta2 += 360;
	sprintf(Pointer_str, "z2.val=%d", theta2);
	ADD_Instruction_To_BUF(Pointer_str, strlen(Pointer_str));
	Copy_BUF(Pointer_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Pointer_str));
	p_HMI_USART_TX_BUF += strlen(Pointer_str);
	
}

void HMI_Set_Slidebar(u8 pos1, u8 pos2)
{
	Clear_String(Pointer_str, sizeof(Pointer_str));
	sprintf(Pointer_str, "h0.val=%d", pos1);
	ADD_Instruction_To_BUF(Pointer_str, strlen(Pointer_str));
	Copy_BUF(Pointer_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Pointer_str));
	p_HMI_USART_TX_BUF += strlen(Pointer_str);
	
	Clear_String(Pointer_str, sizeof(Pointer_str));
	sprintf(Pointer_str, "h1.val=%d", pos2);
	ADD_Instruction_To_BUF(Pointer_str, strlen(Pointer_str));
	Copy_BUF(Pointer_str,(char*)HMI_USART_TXBUF+p_HMI_USART_TX_BUF,strlen(Pointer_str));
	p_HMI_USART_TX_BUF += strlen(Pointer_str);	
}


void ADD_Instruction_To_BUF(char* pBUF,unsigned short str_len)
{
	*(pBUF+str_len+0)=0xFF;
	*(pBUF+str_len+1)=0xFF;
	*(pBUF+str_len+2)=0xFF;
}

void Copy_BUF(char* pBUF_Src,char* pBUF_Dst,unsigned short buf_len)
{
	unsigned short i;
	for(i=0;i<buf_len;i++)
	{
		pBUF_Dst[i] = pBUF_Src[i];
	}
}

void Clear_String(char* pBUF,unsigned short size_of_str)
{
	unsigned short i;
	for(i=0;i<size_of_str;i++)
	{
		pBUF[i] = 0;
	}
}

void HMI_RX_GetOneByte(uint8_t data)
{
	static unsigned char _data_len = 0;
	static unsigned char state = 0;
	static unsigned char  _datatemp[50];
	static unsigned char _data_cnt;

	if( state==0 &&  (data==0x65 || data==0x66 || data==0x67 || data==0x55) )	//header
	{
		state++;
		_datatemp[0] = data;
		if(	data==0x65	)_data_len = 3;	//65 page id 01 ff ff ff  // button down
																		//65 page id 00 ff ff ff  // button up
		
		if(	data==0x66	)_data_len = 1;	//66 page FF FF FF  			// sendme
		
		if(	data==0x67	)_data_len = 5;	//67 xh xl yh yl 1/0 FF FF FF  1press   0up
																		//67 touch down/up

		if(	data==0x55	)_data_len = 3;	//55 aa id pos FF FF FF  // slide bar

		_data_cnt = 0;
	}
	else if(state==1&&_data_len>0)	//data
	{
		_data_len--;
		_datatemp[1+_data_cnt++]=data;
		if(_data_len==0) state++;
	}
	else if(state==2 && data==0xFF)//0xFF
	{
		_datatemp[1+_data_cnt++]=data;
		state++;
	}
	else if(state==3 && data==0xFF)//0xFF
	{
		_datatemp[1+_data_cnt++]=data;
		state++;
	}
	else if(state==4 && data==0xFF)//0xFF
	{
		state = 0;
		_datatemp[1+_data_cnt++]=data;
		
		HMI_RX_Frame_Cnt++;
		if(_datatemp[0]  ==0x65)
		{
			if (_datatemp[3]==0x01)		// Button push down
			{
				pushdownPage = _datatemp[1];
				pushdownId   = _datatemp[2];
				pushdownEvent= 1;
			}
			else if (_datatemp[3]==0x00)
			{
				pushupPage = _datatemp[1];
				pushupId   = _datatemp[2];
				pushupEvent= 1;
			}
		}
		else if( _datatemp[0]==0x66 )//66 01 FF FF FF   page1
		{
			CurrentPageId = _datatemp[1];
		}
		else if( _datatemp[0]==0x67 && _datatemp[5]==0x01)// 67 xh xl yh yl 1/0 FF FF FF    1press   0up    ???? 
		{
			touchx= (_datatemp[1]<<8) | _datatemp[2] ;
			touchy= (_datatemp[3]<<8) | _datatemp[4] ;
			touchEvent=1;
		}
		else if ( _datatemp[0]==0x55 && _datatemp[1]==0xAA)//55 aa id pos FF FF FF  // slide bar
		{
			if (_datatemp[2] == 1)
				slidePos_Left = _datatemp[3];
			else if (_datatemp[2] == 2)
				slidePos_Right = _datatemp[3];
		}
	}
	else
	{
		state = 0;
		HMI_RX_Fail_Cnt++;
	}
}

////////////////////////////////////////////////////////////////////////
void HMI_Check_Button_Push(void)
{
// Some button is released on screen
	if(pushupEvent)
	{
		if (pushupPage == 0)
		{
			switch (pushupId)
			{
				case BUTTON_PUSH_LEFT_PAGE_0:		
				case BUTTON_PUSH_RIGHT_PAGE_0:
																				Angle_Des = Angle_Des_Last;
																				break;
			}
		}
		
		pushupEvent = 0;
	}
	
// Some button is pushed on screen
	if(pushdownEvent)
	{
	// Page 0 deal
		if (pushdownPage == 0)
		{
			switch (pushdownId)
			{
				case BUTTON_RUN_PAGE_0:		if(Stop_Status == STOP)
																	{
																		StopRunConvertStatus = STOP2RUN;
																		Stop_Status = RUN;
																	}
																	break;
																	
				case BUTTON_STOP_PAGE_0:	if(Stop_Status == RUN)
																	{
																		StopRunConvertStatus = RUN2STOP;
																		Stop_Status = STOP;
																	}
																	break;
											
				case BUTTON_PUSH_LEFT_PAGE_0:
						if (Angle_Des >= 0 && Angle_Des < ANGLE_RANGE)
						{
							Angle_Des_Last = Angle_Des;
							Angle_Des += (ANGLE_RANGE - Angle_Des) * (100.0 - slidePos_Left) / 100.0;
						}
						else if (Angle_Des < 0)
						{
							Angle_Des_Last = Angle_Des;
							Angle_Des += ANGLE_RANGE * (100.0 - slidePos_Left) / 100.0;
						}
						break;
						
				case BUTTON_PUSH_RIGHT_PAGE_0:
						if (Angle_Des < 0 && -Angle_Des < ANGLE_RANGE)
						{
							Angle_Des_Last = Angle_Des;
							Angle_Des -= (ANGLE_RANGE + Angle_Des) * (100.0 - slidePos_Right) / 100.0;
						}
						else if (Angle_Des >= 0)
						{
							Angle_Des_Last = Angle_Des;
							Angle_Des -= ANGLE_RANGE * (100.0 - slidePos_Right) / 100.0;
						}
						break;
				
			}
		}
		
	// Page 1 deal
		if (pushdownPage == 1)
		{
			switch(pushdownId)
			{
				case BUTTON_RUN_PAGE_1:		if(Stop_Status == STOP)
																	{
																		StopRunConvertStatus = STOP2RUN;
																		Stop_Status = RUN;
																	}
																	break;
																	
				case BUTTON_STOP_PAGE_1:	if(Stop_Status == RUN)
																	{
																		StopRunConvertStatus = RUN2STOP;
																		Stop_Status = STOP;
																	}	
																	break;
			}
		}
		
	// Page 2 deal
		if (pushdownPage == 2)
		{
			switch(pushdownId)
			{
				case BUTTON_RUN_PAGE_2:		if(Stop_Status == STOP)
																	{
																		StopRunConvertStatus = STOP2RUN;
																		Stop_Status = RUN;
																	}
																	break;
																	
				case BUTTON_STOP_PAGE_2:	if(Stop_Status == RUN)
																	{
																		StopRunConvertStatus = RUN2STOP;
																		Stop_Status = STOP;
																	}	
																	break;
			}
		}

		pushdownEvent = 0;
	}
}

