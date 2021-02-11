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


//char msg[1024] = "UART Tx testing...\n\r";
char msg[9] = {1,2,3,4,5,6,7,8,9};
char msgg[10] = {0};

char RX_Buffer[20];
char TX_Buffer[30] = "SENDING WITH INTERRUPT";

int main(void)
{
	//int i = 0;

		GPIO_Init();

		USART_Init();

		Uart_IntControl(USART2_, UART_INT_RXNE, ENABLE);
		UART_IRQConfig(38, ENABLE);  // 38 for USART2_IRQ enable uart2 interrupt in NVIC


		ADC_Init();
		ADC_RegularChannelConfig();
		ADC_IntControl(ADC_1, ADC_IT_EOC, ENABLE);
		ADC_IRQConfig(18, ENABLE);    // 18 is ID for ADC in NVIC


	while (1)
	{

	}
	return 0;

}


