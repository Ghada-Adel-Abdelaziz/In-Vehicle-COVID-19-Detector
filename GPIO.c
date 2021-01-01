/*
 * GPIO.c
 *
 *  Created on: Dec 30, 2020
 *      Author: esraa
 */


#include "GPIO.h"


/************************************************************ NEW ************************************/
GPIO_regdef_t *GPIO_Arr[NUM_OF_GPIO] = {GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG,GPIOH,GPIOI};
/*****************************************************************************************************/



//void GPIO_PeriClockControl(GPIO_regdef_t *pGPIOx,uint8_t EnorDi)
static void GPIO_PeriClockControl(uint8_t PORT_num,uint8_t EnCLK)
{
	switch(EnCLK)
	{
		case ENABLE:

		for(char i=0; i<NUM_OF_GPIO; i++)          //new
		{
			if(i == PORT_num)
			{
				GPIO_PCLK_EN |= (1 << PORT_num);
				
			}
		}

	case DISALBE:

		for(char i=0; i<NUM_OF_GPIO; i++)
		{
			if(i == PORT_num)
			{
				GPIO_PCLK_EN &= ~(1 << PORT_num);
			}
		}

	}
	return;
}
/*init and De-init
fn   -   GPIO_initialization
brief -  This function initialize the given GPIO port
Parameter  -  Base address of the gpio peripheral or configuration settings
Parameter  -
Parameter  -
return     -
Note       =
 */



// Modified
void GPIO_Init(uint8_t PORT_num , GPIO_pinconfig_t GPIO_pinconfig_1)
{
	uint32_t temp = 0;  //temp register

	//enable the peripheral clock

	GPIO_handle_t GPIO_handle;                 // these 2 variables used to void replacing all -> to . (To facilitate)
	GPIO_handle_t *pGPIOHandle = &GPIO_handle;


	pGPIOHandle-> pGPIOx = GPIO_Arr[PORT_num];     // new
	pGPIOHandle-> GPIO_pinconfig = GPIO_pinconfig_1;

	GPIO_PeriClockControl(PORT_num, ENABLE);

	//configure the mode of gpio pin
	if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		//this part for interrupt mode
		if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
			//clear the correspnding RISR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure the FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);

			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
		}

		//configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		SYSCFG_PCLK_EN;
		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
	}

	temp = 0;

	//configure the speed
	temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//configure the pupd setting
	temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);  //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//configure the optype
	temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinOPType << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//configure the alternate functionality
	if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//alternate function
		uint8_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] = (pGPIOHandle->GPIO_pinconfig.GPIO_PinAltFunMode << (4 * temp2) ); //setting
	}
}
/*init and De-init
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */

//void GPIO_DeInit(GPIO_regdef_t *pGPIOx)
void GPIO_(uint8_t PORT_num)
{

    for(char i=0; i< NUM_OF_GPIO; i++)         // new
    {
    	if( i == PORT_num )
    	{
    		GPIO_RESET_REG |= (1 << PORT_num);
    		GPIO_RESET_REG &= ~(1 << PORT_num);

    		return;
    	}
    }

//	if(pGPIOx == GPIOA)
//	{
//		GPIOA_REG_RESET();
//	}
//	else if(pGPIOx == GPIOB)
//	{
//		GPIOB_REG_RESET();
//	}
//	else if(pGPIOx == GPIOC)
//	{
//		GPIOC_REG_RESET();
//	}
//	else if(pGPIOx == GPIOD)
//	{
//		GPIOD_REG_RESET();
//	}
//	else if(pGPIOx == GPIOE)
//	{
//		GPIOE_REG_RESET();
//	}
//	else if(pGPIOx == GPIOF)
//	{
//		GPIOF_REG_RESET();
//	}
//	else if(pGPIOx == GPIOG)
//	{
//		GPIOG_REG_RESET();
//	}
//	else if(pGPIOx == GPIOH)
//	{
//		GPIOH_REG_RESET();
//	}
//	else if(pGPIOx == GPIOI)
//	{
//		GPIOI_REG_RESET();
//	}
}

/*init and De-init
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */

//uint8_t GPIO_ReadInputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber)
PIN_STATE GPIO_ReadInputPin(uint8_t PORT_num, uint8_t PinNumber)
{
	uint8_t value;

	GPIO_regdef_t *pGPIOx = GPIO_Arr[PORT_num];     // new

	value = (uint8_t)((pGPIOx->IDR  >> PinNumber) & 0x00000001 );

	return value;
}
/*init and De-init
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */

//uint16_t GPIO_ReadInputPort(GPIO_regdef_t *pGPIOx)
uint16_t GPIO_ReadInputPort(uint8_t PORT_num)
{
	uint16_t value;
	GPIO_regdef_t *pGPIOx = GPIO_Arr[PORT_num];     // new

	value = (uint16_t)pGPIOx->IDR;

	return value;
}
/*init and De-init
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */

//void GPIO_WriteOutputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
void GPIO_WriteOutputPin(uint8_t PORT_num, uint8_t PinNumber, uint8_t Value)
{

	GPIO_regdef_t *pGPIOx = GPIO_Arr[PORT_num];     // new

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field coreesponding to the pin
		pGPIOx->ODR |= ( 1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
/*init and De-init
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */

//void GPIO_WriteOutputPort(GPIO_regdef_t *pGPIOx, uint16_t Value)
void GPIO_WriteOutputPort(uint8_t PORT_num, uint16_t Value)
{
	GPIO_regdef_t *pGPIOx = GPIO_Arr[PORT_num];       //new
	pGPIOx->ODR = Value;
}
/*init and De-init
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */

/*init and De-init
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */


/*IRQ configuration and ISR handling
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
Parameter  -
return     -
Note       =
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= ( 1 << (IRQNumber % 32) );
		}

		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
		else
		{
			if(IRQNumber <= 31)
			{
				//Program ICER0 register
				*NVIC_ICER0 |= ( 1 << (IRQNumber % 32) );
			}

			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				//Program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				//Program ICER2 register
				*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
			}
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount );

}


//GPIO_IRQ handling

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
