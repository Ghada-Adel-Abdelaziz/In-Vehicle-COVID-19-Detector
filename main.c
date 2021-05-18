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
#include "Buzzer.h"

#include "HR_sensor.h"
#include "HR_sensor_lcfg.h"

//char msg[1024] = "UART Tx testing...\n\r";
char msg[9] = {1,2,3,4,5,6,7,8,9};
char msgg[10] = {0};

char RX_Buffer[20];
char TX_Buffer[1000] = "SENDING WITH INTERRUPT ";

extern uint16_t ADC_Readings[NUMBER_OF_CONFIGURED_CHANNEL];
extern uint16_t x;

uint16_t ADC_Value;
uint16_t ADC_Value_in_Volt;

void delay_ms(uint16_t delay)
{
	x = 0;
	while(x < (delay));
}



int main(void)
{
	long i = 0;

	GPIO_Init();

	//	USART_Init();
	//	Uart_IntControl(USART2_, UART_INT_RXNE, ENABLE);   // enable interrupt when receive data through uart
	//	UART_IRQConfig(38, ENABLE);  // 38 for USART2_IRQ enable uart2 interrupt in NVIC

	//
	//		Timer_Init();
	//		TIM_OC_Init();
	//		TIM_IRQConfig(28, ENABLE);    // 28 is the number of timer2 in the NVIC
	//		TIM_IRQConfig(29, ENABLE);	  // 29 is the number of timer2 in the NVIC
	//		TIM_IntControl(TIMER2_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer2 overflow
	//		TIM_IntControl(TIMER3_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer3 overflow
	//		Timer_Cmd(TIMER2_, START);    // to start the timer2
	//		Timer_Cmd(TIMER3_, START);    // to start the timer3


	Buzzer_Init();

	//		ADC_Init();
	//		ADC_RegularChannelConfig();
	//		ADC_IntControl(ADC_1, ADC_IT_EOC, ENABLE);    // enable interrupt when ADC conversion complete
	//		ADC_IRQConfig(18, ENABLE);     // 18 is ID for ADC in NVIC
	//ADC_SoftwareStartConv(ADC_1);  // to start ADC conversion

	Buzzer_Req(  2 ,  1 ,  1 ,   LOW_POWER);
	//Buzzer_Req(  3 ,  4 ,  1 ,   MEDIUM_POWER);

CAN_init();
	CAN_IRQConfig(19, ENABLE);
	CAN_IRQConfig(20, ENABLE);
	CAN_msg CAN_TxMsg;
	CAN_TxMsg.id = 60;                              //initialise message to send
	for (i = 0; i < 8; i++) CAN_TxMsg.data[i] = 3;
	CAN_TxMsg.len = 8;
	CAN_TxMsg.format = STANDARD_FORMAT;
	CAN_TxMsg.type = DATA_FRAME;

	CAN_wrMsg      (&(CAN_TxMsg));
	//HR_Sensor_Cmd(HR_SENSOR_PREPH_ID, 1);
	while (1)
	{
		
		//can test 
		while (CAN_RxRdy == 0);

		if (CAN_RxRdy)
		{
			CAN_RxRdy = 0;

		//PWM_Set_Duty(TIMER2_,100);

		// ADC test
		//ADC_Value = (3 * ADC_Readings[0]) / 1023;



		// HR_Sensor_Read(&ADC_Value);
		//
		//				        ADC_Value_in_Volt = (3 * ADC_Value ) / 1023;
		//
		//				        if(  ADC_Value_in_Volt < 1 )
		//						{
		//							GPIO_WriteOutputPin(BLUE_LED,1);    // green led
		//							//for(i=0; i<1000000; i++);
		//						}
		//						else
		//						{
		//							GPIO_WriteOutputPin(BLUE_LED,0);
		//							//for(i=0; i<1000000; i++);
		//						}
		//
		//		if( ( (3*ADC_Readings[1]) / 1023) < 1 )
		//		{
		//			GPIO_WriteOutputPin(LED1,1);   // orange led
		//			for(i=0; i<1000000; i++);
		//		}
		//		else
		//		{
		//			GPIO_WriteOutputPin(LED1,0);
		//			for(i=0; i<1000000; i++);
		//		}





		// UART test
		//Uart_SendDataAsync(USART2_,TX_Buffer,sizeof(TX_Buffer));

		//Uart_ReceiveDataASync(USART2_, RX_Buffer, sizeof(RX_Buffer));

		//		if( RX_Buffer[0] == 5 )
		//		{
		//			GPIO_WriteOutputPin(LED0,1);
		//		}
		//		else
		//		{
		//			GPIO_WriteOutputPin(LED0,0);
		//		}



		//		GPIO_ToggleOutputPin(GPIOD_,LED1);
		//		delay_ms(1000);


		// PWM test

		int i,j = 0;

		//				for(i=0; i<2000; i++)
		//				{
		//
		//					CC1R_SetValue(TIMER2_,i);
		//
		//					delay_ms(2);
		//
		//				}
		//
		//				//for(j=0; j<1000000; j++);
		//
		//				for(i=2000; i>0; i--)
		//				{
		//					CC1R_SetValue(TIMER2_,i);
		//
		//					delay_ms(2);
		//				}
		//	}

		//		for(i=0; i<100; i++)
		//		{
		//
		//			PWM_Set_Duty(TIMER2_,i);
		//
		//			delay_ms(10);
		//
		//		}
		//
		//		//for(j=0; j<1000000; j++);
		//
		//		for(i=100; i>0; i--)
		//		{
		//			PWM_Set_Duty(TIMER2_,i);
		//
		//			delay_ms(10);
		//		}
	}
	return 0;

}


