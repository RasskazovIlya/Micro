#include "stm32f10x.h"

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

uint32_t count_tic = 0;
uint32_t freq;
volatile uint16_t cntr = 0;

void SysTick_Handler(void)
{
	if(cntr > 0)
		cntr--;
}

void delay_ms(uint16_t delay_temp)
{
	cntr = delay_temp;
	while(cntr){}
}

int main(void)
{
	//uint32_t i;
	GPIO_InitTypeDef GPIO_LED;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	SysTick_Config(SystemCoreClock/1000);

	GPIO_LED.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_LED.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_LED.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_LED);
	
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT_CYCCNT  = 0;
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
	
	while(1) 
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		delay_ms(500);
		
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		delay_ms(500);
		
		count_tic =  DWT_CYCCNT;
		freq = SystemCoreClock/DWT_CYCCNT;
		DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk; 
		DWT_CYCCNT  = 0;

		/*GPIO_SetBits(GPIOC, GPIO_Pin_9);
		delay_ms(1000);
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		delay_ms(1000);*/
	}
}
