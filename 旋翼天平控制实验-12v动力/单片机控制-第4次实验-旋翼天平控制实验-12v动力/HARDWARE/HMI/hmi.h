#ifndef __HMI_H
#define __HMI_H
#include "usart1.h"
#include <math.h>
#include "string.h"
#include "stdio.h"

#define HMI_USART_TXBUF     USART1_TX_BUF
#define HMI_USART_RXBUF     USART1_RX_BUF

#define LCD_WIDTH           480			// X2 touch screen
#define LCD_HEIGHT          800

#define MAX_X        			  LCD_WIDTH-1
#define MAX_Y         			LCD_HEIGHT-1
#define ConstrainX(x)       if(x>MAX_X)x=MAX_X;if(x<0)x=0;
#define ConstrainY(y)       if(y>MAX_Y)y=MAX_Y;if(y<0)y=0;

#define Curve_MID    127


// Colors macro
#define  Color_Angle_Omega_CYAN         34815

#define  Color_RED     63488
#define  Color_BLUE    31
#define  Color_GRAY    33840
#define  Color_BLACK   0
#define  Color_WHITE   65535 
#define  Color_GREEN   2016
#define  Color_BROWN   48192
#define  Color_YELLOW  65504

#define STOP2RUN    1
#define RUN2STOP    2

// Object ID on touch screen -> use HMI software to modify
#define BUTTON_RUN_PAGE_0					14
#define BUTTON_STOP_PAGE_0				15
#define BUTTON_PUSH_LEFT_PAGE_0		23
#define BUTTON_PUSH_RIGHT_PAGE_0	22
#define BUTTON_RUN_PAGE_1					8
#define BUTTON_STOP_PAGE_1				9
#define BUTTON_RUN_PAGE_2					12
#define BUTTON_STOP_PAGE_2				13

// Touch screen operation data -> CRITICAL !!!
extern unsigned short p_HMI_USART_TX_BUF,touchx,touchy,touchEvent;
extern unsigned short pushdownPage,pushdownId,pushdownEvent;
extern unsigned short pushupPage,pushupId,pushupEvent,CurrentPageId;
extern unsigned char slidePos_Left, slidePos_Right;

// Touch screen functions declaration
void HMI_Update(void);
void HMI_Print_Init(void);
void HMI_RX_GetOneByte(uint8_t data);
void HMI_RX_Deal(void);

void HMI_PicQ(u16 sx,u16 sy,u16 pic_w,u16 pic_h,u16 pic_id);
void HMI_printf_String(u16 sx,u16 sy,u16 ex,u16 ey,u16 color,char* print_string);
void HMI_Fill(char* color,u16 sx,u16 sy,u16 dx,u16 dy);
void HMI_Draw_Pointer(short theta);
void HMI_Draw_Pointer_LR(short theta1, short theta2);
void HMI_Set_Slidebar(u8 pos1, u8 pos2);

void ADD_Instruction_To_BUF(char* pBUF,unsigned short strlen);
void Copy_BUF(char* pBUF_Src,char* pBUF_Dst,unsigned short buf_len);
void Clear_String(char* pBUF,unsigned short size_of_str);

void HMI_Update_Page0(void);
void HMI_Update_Page1(unsigned char dt);
void HMI_Update_Page2(void);

void HMI_Print_AngleOmega(void);
void HMI_Print_Width(void);
void HMI_Print_Speed(int n_Rotor_Speed);
void HMI_Refresh_CurrentPage(void);
void HMI_RefreshPage(void);
void HMI_change_String(char* objname,char* string_to_change);
void HMI_Add_Curve(unsigned char chx,float data,float range);
void HMI_Set_Slider(unsigned char val);
void HMI_Set_Pointer(unsigned short val);

void HMI_Check_Button_Push(void);
void HMI_Stop2Run_Deal(unsigned char page);
void HMI_Run2Stop_Deal(unsigned char page);

#endif
