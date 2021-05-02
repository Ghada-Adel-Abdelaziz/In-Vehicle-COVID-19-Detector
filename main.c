#include "Common_Macros.h"
#include "stm32f4xxx.h"

#include "GPIO_lcfg.h"
#include "GPIO.h"
#include "GPIO_cfg.h"

#include "UART_cfg.h"
#include "UART_Lcfg.h"
#include "UART_Driver.h"

#include "ADC_Cfg.h"
#include "ADC.h"
#include "ADC_Lcfg.h"

#include "timer.h"
#include "timer_Cfg.h"
#include "timer_Lcfg.h"

//char msg[1024] = "UART Tx testing...\n\r";
char msg[9] = {1,2,3,4,5,6,7,8,9};
char msgg[10] = {0};

char RX_Buffer[20];
char TX_Buffer[1000] = "SENDING WITH INTERRUPT ";

uint16_t ADC_Readings[NUMBER_OF_CONFIGURED_CHANNEL];
extern uint16_t x;

void delay_ms(uint16_t delay)
{
	x = 0;
	while(x < (delay));
}


int main(void)
{
	long i = 0;

	GPIO_Init();

	USART_Init();
	Uart_IntControl(USART2_, UART_INT_RXNE, ENABLE);   // enable interrupt when receive data through uart
	UART_IRQConfig(38, ENABLE);  // 38 for USART2_IRQ enable uart2 interrupt in NVIC


	Timer_Init();
	TIM_OC_Init();
	TIM_IRQConfig(28, ENABLE);    // 28 is the number of timer2 in the NVIC
	TIM_IRQConfig(29, ENABLE);	  // 29 is the number of timer2 in the NVIC
	TIM_IntControl(TIMER2_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer2 overflow
	TIM_IntControl(TIMER3_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer3 overflow
	Timer_Cmd(TIMER2_, START);   // to start the timer2
	Timer_Cmd(TIMER3_, START);    // to start the timer3

	ADC_Init();
	ADC_RegularChannelConfig();
	ADC_IntControl(ADC_1, ADC_IT_EOC, ENABLE);    // enable interrupt when ADC conversion complete
	ADC_IRQConfig(18, ENABLE);     // 18 is ID for ADC in NVIC
	ADC_SoftwareStartConv(ADC_1);  // to start ADC conversion

	CAN_TxMsg.id = 60;                              //initialise message to send
	for (i = 0; i < 8; i++) CAN_TxMsg.data[i] = 0;
	CAN_TxMsg.len = 1;
	CAN_TxMsg.format = STANDARD_FORMAT;
	CAN_TxMsg.type = DATA_FRAME;

	CAN_wrMsg      (&(CAN_TxMsg));
	while (1)
	{

		while (CAN_RxRdy == 0);

		if (CAN_RxRdy)
		{
			CAN_RxRdy = 0;
			//read message here from array

			CAN_RxMsg;
		}
	}
	return 0;

}
