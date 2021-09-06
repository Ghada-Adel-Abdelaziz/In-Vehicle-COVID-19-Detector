#include "Common_Macros.h"
#include "stm32f4xxx.h"

#include "GPIO_lcfg.h"
#include "GPIO.h"
#include "GPIO_cfg.h"

#include "UART_cfg.h"
#include "UART_Lcfg.h"
#include "UART_Driver.h"

#include "timer.h"
#include "timer_Cfg.h"
#include "timer_Lcfg.h"


#include "TFT.h"
#include "Communication_Manager.h"

#include "CAN.h"
#include "CAN_cfg.h"
#include "CAN_Lcfg.h"

#include "stdio.h"

char Txt[9] = {"ESLAM GP"};
//char msg[1024] = "UART Tx testing...\n\r";
char msg[9] = {1,2,3,4,5,6,7,8,9};
char msgg[10] = {0};

uint8_t RX_Buffer[JETSON_PACKET_LENGTH] = {0};
char TX_Buffer[4] = "ABCD";

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
	GPIO_Init();


	USART_Init();
	Uart_IntControl(USART2_, UART_INT_RXNE, ENABLE);   // enable interrupt when receive data through uart
	Uart_IntControl(USART3_, UART_INT_RXNE, ENABLE);   // enable interrupt when receive data through uart
	Uart_IntControl(USART4_, UART_INT_RXNE, ENABLE);

	UART_IRQConfig(38, ENABLE);  // 38 for USART2_IRQ enable uart2 interrupt in NVIC
	UART_IRQConfig(39, ENABLE);  // 38 for USART2_IRQ enable uart2 interrupt in NVIC
	UART_IRQConfig(52, ENABLE);

//	CAN_IRQConfig(19, ENABLE);
//	CAN_IRQConfig(20, ENABLE);
//	CAN_init();

	//	TFT_Request_Writing( REG , 0x03 , 0x0000);
	//	TFT_Tx_Manage();
	//
	//	TFT_Display_Text_Req(Txt2 , 0x0011 , 0X50);


	Uart_ReceiveDataASync(USART2_, RX_Buffer, JETSON_PACKET_LENGTH);
	Uart_ReceiveDataASync(USART4_, RXbuffer, 8);

	Uart_SendDataAsync(USART2_,TX_Buffer,4);



	Timer_Init();
	//			TIM_OC_Init();
	//			TIM_IRQConfig(28, ENABLE);    // 28 is the number of timer2 in the NVIC
	TIM_IRQConfig(29, ENABLE);	  // 29 is the number of timer2 in the NVIC
	TIM_IRQConfig(30, ENABLE);	  // 29 is the number of timer2 in the NVIC

	//			TIM_IntControl(TIMER2_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer2 overflow
	TIM_IntControl(TIMER3_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer3 overflow
	TIM_IntControl(TIMER4_, TIM_UPDATE_EVENT_IE, ENABLE);  // enable interrupt when timer3 overflow

	//			Timer_Cmd(TIMER2_, START);    // to start the timer2
	Timer_Cmd(TIMER3_, START);    // to start the timer3
	Timer_Cmd(TIMER4_, START);    // to start the timer3



	//	Buzzer_Init();
	//
	//			ADC_Init();
	//			ADC_RegularChannelConfig();
	//			ADC_IntControl(ADC_1, ADC_IT_EOC, ENABLE);    // enable interrupt when ADC conversion complete
	//			ADC_IRQConfig(18, ENABLE);     // 18 is ID for ADC in NVIC
	//	//ADC_SoftwareStartConv(ADC_1);  // to start ADC conversion
	//
	//	//Buzzer_Req(  2 ,  1 ,  2 ,   LOW_POWER);
	//	//HR_Sensor_Init();
	//	HR_Sensor_Cmd(HR_Sensor_ConfigArray->Sensor_PrephID , 1);
	//HR_Sensor_Cmd(HR_SENSOR_PREPH_ID, 1);

	//	I2C_Init();

	//I2C_IRQConfig(31 , ENABLE);
	//I2C_IRQConfig(72 , ENABLE);    // I2C3
	//I2C_SendDataAsync(I2C_1 , TX_Buffer , 10 , 0xA0 , ( 0x00 ));
	//I2C_ReceiveDataASync(I2C_1 , RX_Buffer , 2 , 0xA0 , ( 0x00 ) );
	//I2C_ReceiveDataASync(I2C_1 , RX_Buffer , 3 , ( 0xB4) , ( MLX90614_OBJECT_GET_TEMP_CMD ) );

	//I2C_Master_Write( I2C_1 , TX_Buffer , 16 , (0xA0) , 0x00);
	//I2C_Master_Read(I2C_1 , RX_Buffer , 16 , (0xA0) , 0x00);

	//	I2C_Master_Read( I2C_1 , RX_Buffer , 3 , ( 0xB4 ) , 0x07 );
	//
	//
	//	float temp = ( RX_Buffer[1] << 8 ) | RX_Buffer[0];
	//	temp = temp * 0.02 - 273.15;

	//Uart_SendDataAsync(USART3_,TX_Buffer,4);

	//TFT_Request_Reading();
	//Uart_ReceiveDataASync(USART3_, RX_Buffer, 4);

	while (1)
	{
		//		I2C_Master_Read( I2C_1 , RX_Buffer , 3 , ( 0xB4 ) , 0x07 );
		//
		//
		//		float temp = ( RX_Buffer[1] << 8 ) | RX_Buffer[0];
		//		temp = temp * 0.02 - 273.15;


		//		printf("Temperature = %f, \n", temp);
		//
		//		for(i=0; i<1000000; i++);

		//PWM_Set_Duty(TIMER2_,100);

		// ADC test
		//ADC_Value = (3 * ADC_Readings[0]) / 1023;



		//		HR_Sensor_Read(&ADC_Value);
		//
		//		//ADC_Value_in_Volt = (3 * ADC_Value ) / 1023;
		//
		//		if(  ADC_Value < 1 )
		//		{
		//			GPIO_WriteOutputPin(BLUE_LED,1);    // green led
		//			//for(i=0; i<1000000; i++);
		//		}
		//		else
		//		{
		//			GPIO_WriteOutputPin(BLUE_LED,0);
		//			//for(i=0; i<1000000; i++);
		//		}


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
		//	TFT_Tx_Manage();
		//	TFT_Request_Writing( REG , 0x03 , 0x0004);
		//
		//		TFT_Request_Writing( REG , 0x03 , 0x0005);
		//		TFT_Tx_Manage();
		//		TFT_Request_Writing( REG , 0x03 , 0x0006);
		//		TFT_Tx_Manage();
		//
		//	TFT_Display_Text_Req(Txt2 , 0x0011 , 0X50);

		//TFT_Tx_Manage();
		//		TFT_Request_Writing( REG , 0x03 , 0x0004);
		//		TFT_Tx_Manage();
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

		//int i,j = 0;

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
