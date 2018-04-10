#include "stm32f10x.h"

#define PRESCALER_PWM 1 //предделитель таймера
#define PERIOD_PWM 960 //период ШИМ
#define PULSE_PWM 0 //скважность ШИМ

GPIO_InitTypeDef GPIO_LED; 
TIM_TimeBaseInitTypeDef Tim;
TIM_OCInitTypeDef TimPWM;
ADC_InitTypeDef ADC_1;

uint32_t delay_count = 0;
float temp;
uint16_t result;

void initALL(void); //включаем всё
void initTACT(void); //включаем тактирование
void delay_ms(uint32_t delay_temp); //задержка

int main(void)
{
	__enable_irq();
	SystemInit();
	initTACT();
  initALL();
	
	while(1){	
	}
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

void initALL(void)
{
	//включаем тактирование АЦП1, порта входа-выхода С и таймера 3
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//включаем 8 ножку на GPIOC в альтернативном режиме
	GPIO_StructInit(&GPIO_LED);
	GPIO_LED.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_LED.GPIO_Pin = GPIO_Pin_8;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_LED);
	
	//ремапим ножку, чтобы канал ШИМ был на 8 ножке
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	//настраиваем таймер: считает вверх, предделитель и период заданы в начале
	TIM_TimeBaseStructInit(&Tim);
	Tim.TIM_ClockDivision = TIM_CKD_DIV1;
	Tim.TIM_CounterMode = TIM_CounterMode_Up;
	Tim.TIM_Prescaler = PRESCALER_PWM - 1;
	Tim.TIM_Period = PERIOD_PWM;
	TIM_TimeBaseInit(TIM3, &Tim);
	
	//настраиваем генерацию ШИМ на таймере
	TIM_OCStructInit(&TimPWM);
	TimPWM.TIM_Pulse = PULSE_PWM;
	TimPWM.TIM_OCMode = TIM_OCMode_PWM1;
	TimPWM.TIM_OutputState = TIM_OutputState_Enable;
	TimPWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TimPWM);
	
	//включаем таймер
	TIM_Cmd(TIM3, ENABLE);
	
	//настраиваем АЦП1
	ADC_1.ADC_ContinuousConvMode = ENABLE; //сигнал измеряем постоянно
	ADC_1.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //внешних событий для активации АЦП нет
	ADC_1.ADC_DataAlign = ADC_DataAlign_Right; //выравнивание по правому краю (первые 12 бит)
	ADC_1.ADC_Mode = ADC_Mode_Independent; //независимый режим работы
	ADC_1.ADC_NbrOfChannel = 1; //число каналов - 1 (16 канал, где висит датчик температуры)
	ADC_1.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_1);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);//перезапуск калибровки
	while(ADC_GetResetCalibrationStatus(ADC1)); //ждем, пока калибровка пройдет
	ADC_StartCalibration(ADC1);//запуск калибровки
	while(ADC_GetCalibrationStatus(ADC1)); //ждем, пока калибровка пройдет
	ADC_TempSensorVrefintCmd(ENABLE);//включаем датчик температуры
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_28Cycles5); //настраиваем 16 канал (датчик температуры)
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//запускаем АЦП
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	
	//включаем обработчик прерываний АЦП1
	NVIC_EnableIRQ(ADC1_IRQn);
	ADC_ITConfig(ADC1, ADC_IT_EOC,ENABLE);
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

//обработчик прерываний АЦП1
void ADC1_IRQHandler(void)
{
	float V25 = 1.41, Avg_Slope = 4.3*10e-3, Vref = 3.3, Vsense;
	
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
	{
		result = ADC_GetConversionValue(ADC1);
		Vsense = result/4096.0*Vref;
		temp = (V25 - Vsense)/Avg_Slope + 25.0; //считаем температуру по формуле
		ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
	}
	
	if (temp < 30 && temp > 15)
		TIM_SetCompare3(TIM3, PERIOD_PWM/15*(uint16_t)(temp-15.0)); //изменяем яркость диода в зависимости от температуры
	else if (temp > 30)
		TIM_SetCompare3(TIM3, PERIOD_PWM); //если температура больше 30, то яркость максимальная
	else if (temp < 15)
		TIM_SetCompare3(TIM3, 0); //если температура меньше 15, то яркость 0, диод не горит
}
