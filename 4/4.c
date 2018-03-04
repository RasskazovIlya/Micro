#include "stm32f10x.h"

#define PRESCALER_PWM 960
#define PERIOD_PWM 15000
#define PULSE_PWM 0
#define COUNTER 5000

GPIO_InitTypeDef GPIO_LED;
TIM_TimeBaseInitTypeDef Tim;
TIM_OCInitTypeDef TimPWM;

void initALL(void);

int main(void)
{
	volatile uint16_t i = 0;
	volatile double freq;
	
  initALL();
	
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	freq = clocks.PCLK1_Frequency/PRESCALER_PWM;
	
	while(1)
	{
		for (i = 0; i < COUNTER; i++)
				TIM_SetCompare3(TIM3, i);
		
		for (i = COUNTER; i >= 1; i--)
				TIM_SetCompare3(TIM3, i);
	}
}

void initALL()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_StructInit(&GPIO_LED);
	GPIO_LED.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_LED.GPIO_Pin = GPIO_Pin_8;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_LED);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	TIM_TimeBaseStructInit(&Tim);
	Tim.TIM_ClockDivision = TIM_CKD_DIV2;
	Tim.TIM_CounterMode = TIM_CounterMode_Up;
	Tim.TIM_Prescaler = PRESCALER_PWM;
	Tim.TIM_Period = PERIOD_PWM;
	TIM_TimeBaseInit(TIM3, &Tim);
	
	TIM_OCStructInit(&TimPWM);
	TimPWM.TIM_Pulse = PULSE_PWM;
	TimPWM.TIM_OCMode = TIM_OCMode_PWM1;
	TimPWM.TIM_OutputState = TIM_OutputState_Enable;
	TimPWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TimPWM);
	
	TIM_Cmd(TIM3, ENABLE);
}
