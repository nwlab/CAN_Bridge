#include "stm32f4xx_delay.h"
 
static uint8_t  fac_us=0;  // micro second count
static uint16_t fac_ms=0;  // mili second count
 
/***************************************************************
* delay Initilize 
***************************************************************/
void delay_init()	 
{
//	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);	//HSE  HCLK/8
	fac_us = SystemCoreClock / 8000000;							//SYSCLK/8  
	fac_ms = (uint16_t)fac_us*1000;   
}								    
 
/***************************************************************
* delay in micro second
***************************************************************/		    								   
void delay_us(uint32_t micro_sec)
{		
	uint32_t temp;	    	 
	SysTick->LOAD = micro_sec*fac_us; //Load	  		 
	SysTick->VAL = 0x00;        		//Clear
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;        
	do
	{
		temp = SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       
	SysTick->VAL = 0X00;       
}
 
/***************************************************************
* Delay in mili second
***************************************************************/
void delay_ms(uint16_t mili_sec)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)mili_sec*fac_ms;
	SysTick->VAL =0x00;           
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;         
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       
	SysTick->VAL =0X00;       
}
