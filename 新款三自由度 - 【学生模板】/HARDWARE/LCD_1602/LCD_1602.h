#include "stm32f10x.h"    		//±ØÒªµÄ 
#include "delay.h"

// LCD GPIO
#define HC595_SHcp 			GPIO_Pin_15		// PORTB
#define HC595_STcp			GPIO_Pin_14		// PORTB
#define HC595_Ds				GPIO_Pin_13		// PORTB
#define LCD_Rs					GPIO_Pin_8		// PORTA
#define LCD_E						GPIO_Pin_11		// PORTA

// LCD command
#define DISPLAY_CLEAR           1
#define RETURN_HOME             2
#define ENTRY_SET               3
#define DISPLAY_SET             4
#define SHIFT_DISPLAY           5
#define FUNCTION_SET            6   
#define WRITE_ADDRESS           10
#define WRITE_DATA              11 
#define ON                      1
#define OFF                     0

void Push_Data(unsigned char c_Data);
void LCD_write(unsigned char c_RS, unsigned char c_Data);
void LCD_Disp(unsigned char c_Mode, unsigned char c_Para1, unsigned char c_Para2, unsigned char c_Para3, unsigned char c_Data);
void LCD_Write_Char(unsigned char x, unsigned char y, char s_Dis);
void LCD_Write_String(unsigned char x, unsigned char y, char *s_Dis);
void LCD_Init(void);
void Dis_HOP(unsigned int n_No, float f_HOP);
void Dis_Width(unsigned int n_No, float f_Width);


