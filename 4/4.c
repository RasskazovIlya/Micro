#include "stm32f10x.h"

#define PRESCALER_PWM 960 //������������ �������
#define PERIOD_PWM 17500 //������ ���
#define PULSE_PWM 0 //���������� ���

GPIO_InitTypeDef GPIO_LED; 
TIM_TimeBaseInitTypeDef Tim;
TIM_OCInitTypeDef TimPWM;

uint32_t delay_count=0;

void initALL(void); //�������� ��
void initTACT(void); //�������� ������������
void delay_ms(uint32_t delay_temp); //��������

int main(void)
{
	volatile uint16_t i = PULSE_PWM;
	volatile double freq;
	
	//�������� ��
	SystemInit();
	initTACT();
  initALL();
	
	SystemCoreClockUpdate(); 
	SysTick_Config(SystemCoreClock/1000); //��������� ��������� ������
	
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	freq = clocks.PCLK1_Frequency/PRESCALER_PWM; //���������� ������� ���
	
	while(1)
	{
		for (i = PULSE_PWM; i < PERIOD_PWM-1; i++)
		{
			if (i < (PERIOD_PWM/2))
				{
					TIM_SetCompare3(TIM3, i); // ������� �������������
				}
			else if (PULSE_PWM > PERIOD_PWM)
			{
				if (i > PULSE_PWM && i < (PERIOD_PWM-1))
				{
					TIM_SetCompare3(TIM3, (PERIOD_PWM - i - 1)); // ������� �����������
				}
			}
			else if (i > (PERIOD_PWM/2) && i < (PERIOD_PWM-1))
			{
				TIM_SetCompare3(TIM3, (PERIOD_PWM - i - 1)); // ������� �����������
			}
		}
		//-----------
		// ��������� ����������
		/*TIM_SetCompare3(TIM3, i);
		if (i >  PULSE_PWM)
			i--;
		else i = PERIOD_PWM;*/
		//-----------
	}
}

//�������� ��
void initALL()
{
	//�������� GPIOC, AFIO, TIM3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//�������� 8 ����� �� GPIOC � �������������� ������
	GPIO_StructInit(&GPIO_LED);
	GPIO_LED.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_LED.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_LED);
	
	//������� �����, ����� ����� ��� ��� �� 8 �����
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	//����������� ������: ������� �����, ������������ � ������ ������ � ������
	TIM_TimeBaseStructInit(&Tim);
	Tim.TIM_ClockDivision = TIM_CKD_DIV1;
	Tim.TIM_CounterMode = TIM_CounterMode_Up;
	Tim.TIM_Prescaler = PRESCALER_PWM - 1;
	Tim.TIM_Period = PERIOD_PWM - 1;
	TIM_TimeBaseInit(TIM3, &Tim);
	
	//����������� ��������� ��� �� �������
	TIM_OCStructInit(&TimPWM);
	TimPWM.TIM_Pulse = PULSE_PWM;
	TimPWM.TIM_OCMode = TIM_OCMode_PWM1;
	TimPWM.TIM_OutputState = TIM_OutputState_Enable;
	TimPWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TimPWM);
	TIM_OC4Init(TIM3, &TimPWM);
	
	//�������� ������
	TIM_Cmd(TIM3, ENABLE);
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

