#include "stm32f10x.h"

#define PRESCALER_PWM 1 //������������ �������
#define PERIOD_PWM 960 //������ ���
#define PULSE_PWM 0 //���������� ���

GPIO_InitTypeDef GPIO_LED; 
TIM_TimeBaseInitTypeDef Tim;
TIM_OCInitTypeDef TimPWM;
ADC_InitTypeDef ADC_1;

uint32_t delay_count = 0;
float temp;
uint16_t result;

void initALL(void); //�������� ��
void initTACT(void); //�������� ������������
void delay_ms(uint32_t delay_temp); //��������

int main(void)
{
	__enable_irq();
	SystemInit();
	initTACT();
  initALL();
	
	while(1){	
	}
}

//�������� ������������
void initTACT(void)
{
	RCC_DeInit();

	RCC_PLLCmd(DISABLE); // ��������� PLL
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12); //�������� ������� � PLL - HSI, ������� �� 2, � ���������� 12 (8/2*12=48 ���)
	RCC_PLLCmd(ENABLE); // �������� PLL
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); // ���� ��������� PLL
	
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // �������� ��������� ������� - PLL
	RCC_HCLKConfig(RCC_SYSCLK_Div2); //������ �������� 2 ��� ���� AHB, ������� �� ��� 48/2=24 ���
	RCC_PCLK1Config(RCC_HCLK_Div1); //������ �������� 1 ��� ���� APB1, ������� �� ��� 24/1=24 ���
	RCC_PCLK2Config(RCC_HCLK_Div1); //������ �������� 1 ��� ���� APB2, ������� �� ��� 24/1=24 ���
}

void initALL(void)
{
	//�������� ������������ ���1, ����� �����-������ � � ������� 3
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//�������� 8 ����� �� GPIOC � �������������� ������
	GPIO_StructInit(&GPIO_LED);
	GPIO_LED.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_LED.GPIO_Pin = GPIO_Pin_8;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_LED);
	
	//������� �����, ����� ����� ��� ��� �� 8 �����
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	//����������� ������: ������� �����, ������������ � ������ ������ � ������
	TIM_TimeBaseStructInit(&Tim);
	Tim.TIM_ClockDivision = TIM_CKD_DIV1;
	Tim.TIM_CounterMode = TIM_CounterMode_Up;
	Tim.TIM_Prescaler = PRESCALER_PWM - 1;
	Tim.TIM_Period = PERIOD_PWM;
	TIM_TimeBaseInit(TIM3, &Tim);
	
	//����������� ��������� ��� �� �������
	TIM_OCStructInit(&TimPWM);
	TimPWM.TIM_Pulse = PULSE_PWM;
	TimPWM.TIM_OCMode = TIM_OCMode_PWM1;
	TimPWM.TIM_OutputState = TIM_OutputState_Enable;
	TimPWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TimPWM);
	
	//�������� ������
	TIM_Cmd(TIM3, ENABLE);
	
	//����������� ���1
	ADC_1.ADC_ContinuousConvMode = ENABLE; //������ �������� ���������
	ADC_1.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //������� ������� ��� ��������� ��� ���
	ADC_1.ADC_DataAlign = ADC_DataAlign_Right; //������������ �� ������� ���� (������ 12 ���)
	ADC_1.ADC_Mode = ADC_Mode_Independent; //����������� ����� ������
	ADC_1.ADC_NbrOfChannel = 1; //����� ������� - 1 (16 �����, ��� ����� ������ �����������)
	ADC_1.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_1);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);//���������� ����������
	while(ADC_GetResetCalibrationStatus(ADC1)); //����, ���� ���������� �������
	ADC_StartCalibration(ADC1);//������ ����������
	while(ADC_GetCalibrationStatus(ADC1)); //����, ���� ���������� �������
	ADC_TempSensorVrefintCmd(ENABLE);//�������� ������ �����������
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_28Cycles5); //����������� 16 ����� (������ �����������)
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//��������� ���
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	
	//�������� ���������� ���������� ���1
	NVIC_EnableIRQ(ADC1_IRQn);
	ADC_ITConfig(ADC1, ADC_IT_EOC,ENABLE);
}

// �������� 1 ��
void delay_ms(uint32_t delay_temp)
{
	delay_count = delay_temp;
	
	while(delay_count){}
}

//���������� ���������� ���������� �������
void SysTick_Handler(void)
{
	if (delay_count > 0)
		delay_count--;
}

//���������� ���������� ���1
void ADC1_IRQHandler(void)
{
	float V25 = 1.41, Avg_Slope = 4.3*10e-3, Vref = 3.3, Vsense;
	
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
	{
		result = ADC_GetConversionValue(ADC1);
		Vsense = result/4096.0*Vref;
		temp = (V25 - Vsense)/Avg_Slope + 25.0; //������� ����������� �� �������
		ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
	}
	
	if (temp < 30 && temp > 15)
		TIM_SetCompare3(TIM3, PERIOD_PWM/15*(uint16_t)(temp-15.0)); //�������� ������� ����� � ����������� �� �����������
	else if (temp > 30)
		TIM_SetCompare3(TIM3, PERIOD_PWM); //���� ����������� ������ 30, �� ������� ������������
	else if (temp < 15)
		TIM_SetCompare3(TIM3, 0); //���� ����������� ������ 15, �� ������� 0, ���� �� �����
}
