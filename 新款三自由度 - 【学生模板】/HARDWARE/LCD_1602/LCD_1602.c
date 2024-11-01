#include "LCD_1602.h"

void Push_Data(unsigned char c_Data)
{
 unsigned int i; 
	
// Clear 74HC595 shift & store clock   
 GPIO_ResetBits(GPIOB, HC595_SHcp);  //HC595_SHcp = 0
 GPIO_ResetBits(GPIOB, HC595_STcp);  //HC595_STcp = 0
                  
 for (i = 0; i < 8; i++)                
 {
// Set the 74HC595 serial data	 
	if ((c_Data >> (7 -i)) & 0x01) 
	 GPIO_SetBits(GPIOB, HC595_Ds);  //HC595_Ds = 1
  else
	 GPIO_ResetBits(GPIOB, HC595_Ds);  //HC595_Ds = 0
	
// Shift the 74HC595 data	
  GPIO_SetBits(GPIOB, HC595_SHcp);  //HC595_SHcp = 1
  GPIO_ResetBits(GPIOB, HC595_SHcp);  //HC595_SHcp = 0
 }          
 
// Store the 74HC595 data & output
 GPIO_SetBits(GPIOB, HC595_STcp);  //HC595_STcp = 1
 GPIO_ResetBits(GPIOB, HC595_STcp);  //HC595_STcp = 0

}  

void LCD_write(unsigned char c_RS, unsigned char c_Data)
{
// LCD_RS = c_RS;
 if (c_RS) 
	GPIO_SetBits(GPIOA, LCD_Rs);  //RS = 1
 else
	GPIO_ResetBits(GPIOA, LCD_Rs);  //RS = 0

// LCD_E = 1;
 GPIO_SetBits(GPIOA, LCD_E);  //E = 1

 Push_Data(c_Data);	
 delay_us(50);
 
// LCD_E = 0;
 GPIO_ResetBits(GPIOA, LCD_E); // E = 0
}   

void LCD_Disp(unsigned char c_Mode, unsigned char c_Para1, unsigned char c_Para2, unsigned char c_Para3, unsigned char c_Data)
{                     
 unsigned char c_Temp;
 switch (c_Mode)
 {
  case DISPLAY_CLEAR: {
                       LCD_write(0, 0x01); 
                       break;
                      }  
  case RETURN_HOME:
                      {
                       LCD_write(0, 0x02); 
                       break;                      
                      }  
  case ENTRY_SET:
                      {     
                       c_Temp = 0x04 | (c_Para1 * 0x02);
                       c_Temp = c_Temp | c_Para2;
                       LCD_write(0, c_Temp); 
                       break;             
                      }  
  case DISPLAY_SET:
                      {     
                       c_Temp = 0x08 | (c_Para1 * 0x04);
                       c_Temp = c_Temp | (c_Para2 * 0x02);
                       c_Temp = c_Temp | c_Para3;
                       LCD_write(0, c_Temp); 
                       break;             
                      }  
  case SHIFT_DISPLAY:
                      {     
                       c_Temp = 0x10 | (c_Para1 * 0x08);
                       c_Temp = c_Temp | (c_Para2 * 0x04);
                       LCD_write(0, c_Temp); 
                       break;             
                      }  
  case FUNCTION_SET:
                      {     
                       c_Temp = 0x20 | (c_Para1 * 0x10);
                       c_Temp = c_Temp | (c_Para2 * 0x08);
                       c_Temp = c_Temp | (c_Para3 * 0x04);
                       LCD_write(0, c_Temp); 
                       break;             
                      }  
  case WRITE_ADDRESS:
                      {     
                       c_Temp = 0x80 | c_Data;
                       LCD_write(0, c_Temp); 
                       break;             
                      }  
  case WRITE_DATA:
                      {     
                       LCD_write(1, c_Data); 
                       break;             
                      }  
 }
} 

void LCD_Write_Char(unsigned char x, unsigned char y, char s_Dis)
{               
 if (y < 2)                                
  LCD_write(0, 0x7F + x);
 else
  LCD_write(0, 0xBF + x);
 LCD_write(1, s_Dis);
}

void LCD_Write_String(unsigned char x, unsigned char y, char *s_Dis)
{               
 if (y < 2)                                
  LCD_write(0, 0x7F + x);
 else
  LCD_write(0, 0xBF + x);
 while(*s_Dis)
 {
  LCD_write(1, *s_Dis++);
 }
}

void LCD_Init(void)
{        
 LCD_Disp(FUNCTION_SET, ON, ON, OFF, 0);
 LCD_Disp(DISPLAY_SET, OFF, OFF, OFF, 0); 
 LCD_Disp(DISPLAY_CLEAR, OFF, OFF, OFF, 0);
 LCD_Disp(ENTRY_SET, ON, ON, OFF, 0);
 delay_ms(10);
 LCD_Disp(DISPLAY_SET, ON, OFF, OFF, 0);  
}   
