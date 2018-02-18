#include "stm32f10x.h"

int main(void)
{
	uint32_t i;
	GPIO_InitTypeDef GPIO_LED;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_LED.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_LED.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_LED.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOC, &GPIO_LED);
	
	while(1) {
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		
		for (i = 0; i < 100000; i++);
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		
		for (i = 0; i < 1000000; i++);
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		
		for (i = 0; i < 1000000; i++);
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		
		for (i = 0; i < 1000000; i++);
	}
}
