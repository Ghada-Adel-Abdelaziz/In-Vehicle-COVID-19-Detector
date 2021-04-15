/*
 * I2C_lcfg.c
 *
 *  Created on: Apr 2, 2021
 *      Author: esraa
 */

#include "I2C_lcfg.h"
#include "I2C_cfg.h"
void LED1_ON(void)   // I2C TX complete
{
	static char i = 0;
	i++;

	if(i == 1)
	{
		GPIO_WriteOutputPin(ORANGE_LED,1);
	}
	else if(i == 2)
	{
		i = 0;
		GPIO_WriteOutputPin(ORANGE_LED,0);
	}

}

void LED2_ON(void)   // UART RX complete
{
	static char i = 0;
	i++;

	if(i == 1)
	{
		GPIO_WriteOutputPin(GREEN_LED,1);
	}
	else if(i == 2)
	{
		i = 0;
		GPIO_WriteOutputPin(GREEN_LED,0);
	}

	Uart_SendDataAsync(USART2_,RX_Buffer,sizeof(RX_Buffer));
}


extern I2C_Config_t I2C_ConfigArray[NUMBER_OF_CONFIGURED_I2C]=
{
		{I2C_1, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable, I2C_Address_7bit},
};
