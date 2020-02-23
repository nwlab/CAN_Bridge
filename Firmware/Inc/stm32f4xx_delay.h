/***************************************************************
* File     : fm_stm32f4_delay.h
* website  : www.firmcodes.com
* email_id : support@firmcodes.com
* IDE used : The files are Compiled in coocox ide and 
			 tested on stm32f4 discovery board.
***************************************************************/
 
#ifndef __FM_STM32F4_DELAY_H
#define __FM_STM32F4_DELAY_H 
 
/***************************************************************
// Header Files Includes
***************************************************************/
#include "stm32f4xx.h"			   
 
 
 
/***************************************************************
* Gloable Function
***************************************************************/	 
void delay_init(void);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
 
/***************************************************************/
#endif // __FM_STM32F4_DELAY_H
