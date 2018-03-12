#include "stm32f10x.h"

#define PRESCALER_PWM 960 //предделитель таймера
#define PERIOD_PWM 17500 //период ШИМ
#define PULSE_PWM 0 //скважность ШИМ

GPIO_InitTypeDef GPIO_LED; 
TIM_TimeBaseInitTypeDef Tim;
TIM_OCInitTypeDef TimPWM;

uint32_t delay_count=0;

void initALL(void); //включаем всё
void initTACT(void); //включаем тактирование
void delay_ms(uint32_t delay_temp); //задержка

int main(void)
{
	volatile uint16_t i = PULSE_PWM;
	volatile double freq;
	
	//включаем всё
	SystemInit();
	initTACT();
  initALL();
	
	SystemCoreClockUpdate(); 
	SysTick_Config(SystemCoreClock/1000); //запускаем системный таймер
	
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	freq = clocks.PCLK1_Frequency/PRESCALER_PWM; //определяем частоту ШИМ
	
	while(1)
	{
		for (i = PULSE_PWM; i < PERIOD_PWM-1; i++)
		{
			if (i < (PERIOD_PWM/2))
				{
					TIM_SetCompare3(TIM3, i); // яркость увеличивается
				}
			else if (PULSE_PWM > PERIOD_PWM)
			{
				if (i > PULSE_PWM && i < (PERIOD_PWM-1))
				{
					TIM_SetCompare3(TIM3, (PERIOD_PWM - i - 1)); // яркость уменьшается
				}
			}
			else if (i > (PERIOD_PWM/2) && i < (PERIOD_PWM-1))
			{
				TIM_SetCompare3(TIM3, (PERIOD_PWM - i - 1)); // яркость уменьшается
			}
		}
		//-----------
		// Затухание светодиода
		/*TIM_SetCompare3(TIM3, i);
		if (i >  PULSE_PWM)
			i--;
		else i = PERIOD_PWM;*/
		//-----------
	}
}

//включаем всё
void initALL()
{
	//включаем GPIOC, AFIO, TIM3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//включаем 8 ножку на GPIOC в альтернативном режиме
	GPIO_StructInit(&GPIO_LED);
	GPIO_LED.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_LED.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_LED);
	
	//ремапим ножку, чтобы канал ШИМ был на 8 ножке
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	//настраиваем таймер: считает вверх, предделитель и период заданы в начале
	TIM_TimeBaseStructInit(&Tim);
	Tim.TIM_ClockDivision = TIM_CKD_DIV1;
	Tim.TIM_CounterMode = TIM_CounterMode_Up;
	Tim.TIM_Prescaler = PRESCALER_PWM - 1;
	Tim.TIM_Period = PERIOD_PWM - 1;
	TIM_TimeBaseInit(TIM3, &Tim);
	
	//настраиваем генерацию ШИМ на таймере
	TIM_OCStructInit(&TimPWM);
	TimPWM.TIM_Pulse = PULSE_PWM;
	TimPWM.TIM_OCMode = TIM_OCMode_PWM1;
	TimPWM.TIM_OutputState = TIM_OutputState_Enable;
	TimPWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TimPWM);
	TIM_OC4Init(TIM3, &TimPWM);
	
	//включаем таймер
	TIM_Cmd(TIM3, ENABLE);
}

//включаем тактирование
void initTACT(void)
{
	RCC_DeInit();

	RCC_PLLCmd(DISABLE); // Выключаем PLL
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12); //Источник частоты в PLL - HSI, деленый на 2, с множителем 12 (8/2*12=48 МГц)
	RCC_PLLCmd(ENABLE); // Включаем PLL
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); // Ждем включения PLL
	
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // Источник системной частоты - PLL
	RCC_HCLKConfig(RCC_SYSCLK_Div2); //ставим делитель 2 для шины AHB, частота на ней 48/2=24 МГц
	RCC_PCLK1Config(RCC_HCLK_Div1); //ставим делитель 1 для шины APB1, частота на ней 24/1=24 МГц
	RCC_PCLK2Config(RCC_HCLK_Div1); //ставим делитель 1 для шины APB2, частота на ней 24/1=24 МГц
}

// задержка 1 мс
void delay_ms(uint32_t delay_temp)
{
	delay_count = delay_temp;
	
	while(delay_count){}
}

//обработчик прерываний системного таймера
void SysTick_Handler(void)
{
	if (delay_count > 0)
		delay_count--;
}

