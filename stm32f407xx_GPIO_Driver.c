/*
 *File Name: stm32f407xx_GPIO_Driver.c
 *Description: Source file for the GPIO Stm32f407 driver
 *Created on: 19/12/2020
 *Author: Toaa mahmoud
 *
  *******************************************************************************/
#include "stm32f407xx_GPIO_Driver.h"
/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/
/*GPIO peripheral clock control
fn   -   GPIO_periclockcontrol
brief -  This function enables or disables peripheral clock for the given GPIO port
Parameter  -  Base address of the gpio peripheral
Parameter  -  Enable or disable macros
return     -
Note       =uint8_t--->unsigned integer type with width of exactly 8bits 
*/
void GPIO_PeriClockControl(GPIO_regdef_t *pGPIOx,uint8_t EnCLK)//DONE1
{
   if(EnCLK == ENABLE)
   {
	   if(pGPIOx == GPIOA)
	   {
		   GPIOA_PCLK_EN;
	   }
	   else if(pGPIOx == GPIOB)
	   {
		   GPIOB_PCLK_EN;
	   }
	   else if(pGPIOx == GPIOC)
	   	   {
	   		   GPIOC_PCLK_EN;
	   	   }
	   else if(pGPIOx == GPIOD)
	   	   {
	   		   GPIOD_PCLK_EN;
	   	   }
	   else if(pGPIOx == GPIOE)
	   	   {
	   		   GPIOE_PCLK_EN;
	   	   }
	   else if(pGPIOx == GPIOF)
	   	   {
	   		   GPIOF_PCLK_EN;
	   	   }
	   else if(pGPIOx == GPIOG)
	   	   {
	   		   GPIOG_PCLK_EN;
	   	   }
	   else if(pGPIOx == GPIOH)
	   	   {
	   		   GPIOH_PCLK_EN;
	   	   }
	   else if(pGPIOx == GPIOI)
	   	   {
	   		   GPIOI_PCLK_EN;
	   	   }
   }
   else
   {
	   if(pGPIOx == GPIOA)
	   	   {
	   		   GPIOA_PCLK_DI;
	   	   }
	   	   else if(pGPIOx == GPIOB)
	   	   {
	   		   GPIOB_PCLK_DI;
	   	   }
	   	   else if(pGPIOx == GPIOC)
	   	   	   {
	   	   		   GPIOC_PCLK_DI;
	   	   	   }
	   	   else if(pGPIOx == GPIOD)
	   	   	   {
	   	   		   GPIOD_PCLK_DI;
	   	   	   }
	   	   else if(pGPIOx == GPIOE)
	   	   	   {
	   	   		   GPIOE_PCLK_DI;
	   	   	   }
	   	   else if(pGPIOx == GPIOF)
	   	   	   {
	   	   		   GPIOF_PCLK_DI;
	   	   	   }
	   	   else if(pGPIOx == GPIOG)
	   	   	   {
	   	   		   GPIOG_PCLK_DI;
	   	   	   }
	   	   else if(pGPIOx == GPIOH)
	   	   	   {
	   	   		   GPIOH_PCLK_DI;
	   	   	   }
	   	   else if(pGPIOx == GPIOI)
	   	   	   {
	   	   		   GPIOI_PCLK_DI;
	   	   	   }

   }
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

void GPIO_Init(GPIO_handle_t *pGPIOHandle)
{
	uint32_t temp = 0;  //temp register

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//configure the mode of gpio pin
	if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber );//
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
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber );
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

void GPIO_DeInit(GPIO_regdef_t *pGPIOx)//DONE2
{
		   if(pGPIOx == GPIOA)
		   {
			   GPIOA_REG_RESET();
		   }
		   else if(pGPIOx == GPIOB)
		   {
			   GPIOB_REG_RESET();
		   }
		   else if(pGPIOx == GPIOC)
		   	   {
			   GPIOC_REG_RESET();
		   	   }
		   else if(pGPIOx == GPIOD)
		   	   {
			   GPIOD_REG_RESET();
		   	   }
		   else if(pGPIOx == GPIOE)
		   	   {
			   GPIOE_REG_RESET();
		   	   }
		   else if(pGPIOx == GPIOF)
		   	   {
			   GPIOF_REG_RESET();
		   	   }
		   else if(pGPIOx == GPIOG)
		   	   {
			   GPIOG_REG_RESET();
		   	   }
		   else if(pGPIOx == GPIOH)
		   	   {
			   GPIOH_REG_RESET();
		   	   }
		   else if(pGPIOx == GPIOI)
		   	   {
			   GPIOI_REG_RESET();
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
//uint8_t--->unsigned integer type with width of exactly 8bits 
uint8_t GPIO_ReadInputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value; //Variable to Read Data

    value = (uint8_t)((pGPIOx->IDR  >> PinNumber) & 0x00000001 ); ??

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

uint16_t GPIO_ReadInputPort(GPIO_regdef_t *pGPIOx)//DONE3
{
	 uint16_t value;

	    value = (uint16_t)pGPIOx->IDR;//(dot or row)

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

void GPIO_WriteOutputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)//DONE4
{
	if(Value == GPIO_PIN_SET)
	{
   //write 1 to the output data register at the bit field coresponding to the pinNumber
		pGPIOx->ODR |= ( 1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_WriteOutputPort(GPIO_regdef_t *pGPIOx, uint16_t Value)//DONE5
{
     pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber)//DONE6
{
       pGPIOx->ODR ^= (1 << PinNumber);
}