/******************************************************************************
 * Module: 		GPIO
 * File Name: 	GPIO.c
 * Description: GPIO Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa&Ghada
 * Date:		9/1/2021
 ******************************************************************************/

#include "Common_Macros.h"
#include "GPIO_Cfg.h"
#include "GPIO.h"
#include "GPIO_Lcfg.h"


#define PORT_NUMBER_OF_BITS_IN_REG         16



/*NVIC IRQ interrupt priority number*/
#define NVIC_IRQ_PRI0   0
#define NVIC_IRQ_PRI1   1
#define NVIC_IRQ_PRI2   2
#define NVIC_IRQ_PRI3   3
#define NVIC_IRQ_PRI4   4
#define NVIC_IRQ_PRI5   5


#define last_regBit_num_31                  31
#define ISER_reg_32Bits                     32
#define ICER_reg_32Bits                     32

#define Double_32Bits_for_ISER_reg          64
#define Double_32Bits_for_ICER_reg          64

#define Four_times_32Bits_FOR_ISER_reg      96
#define Four_times_32Bits_FOR_ICER_reg      96


// new

/************************************************************ NEW ************************************/
GPIO_regdef_t *GPIO_Arr[NUM_OF_GPIO] = {GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG,GPIOH,GPIOI};
/*****************************************************************************************************/


//handle structure for GPIO


static void GPIO_PeriClockControl(uint8_t PORT_num,uint8_t EnorDi);

//void GPIO_PeriClockControl(GPIO_regdef_t *pGPIOx,uint8_t EnorDi)
static void GPIO_PeriClockControl(uint8_t PORT_num,uint8_t EnCLK)
{
	//	GPIO_PCLK_EN &=~ (One_bit_shift << PORT_num);
	//	GPIO_PCLK_EN |= (EnCLK << PORT_num);

	GPIO_PCLK_EN =(GPIO_PCLK_EN & ~(One_bit_shift << PORT_num))
			|(EnCLK << PORT_num);
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

void GPIO_Init(void)
{

	uint8_t counter=0;
	uint8_t PortNumber=0;
	uint8_t PinActualNumber=0;


	for( counter=0; counter<NUMBER_OF_CONFIGURED_PINS; counter++ )
	{
		PortNumber = ( GPIO_PinConfigArray[counter].GPIO_PinNumber) / PORT_NUMBER_OF_BITS_IN_REG;
		PinActualNumber = (GPIO_PinConfigArray[counter].GPIO_PinNumber) % PORT_NUMBER_OF_BITS_IN_REG;
		uint32_t temp = 0;  //temp register
		//enable the peripheral clock
		GPIO_PeriClockControl(PortNumber, ENABLE);
		GPIO_regdef_t *pGPIOx = GPIO_Arr[PortNumber];
		//configure the mode of gpio pin
		if(GPIO_PinConfigArray[counter].GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
			//			temp = (GPIO_PinConfigArray[counter].GPIO_PinMode << (Two_bits_shift * PinActualNumber) );
			//			pGPIOx->MODER &= ~( (Two_bits_shift *Two_consecutive_bits_mask_by_HEX)<< PinActualNumber );
			//			pGPIOx->MODER |= temp;
			//			temp = 0;
			temp = pGPIOx->MODER;
			temp &= ~(0x3UL << (PinActualNumber * 2u));
			temp |= ((GPIO_PinConfigArray[counter].GPIO_PinMode & 0x00000003U) << (PinActualNumber * 2u));
			pGPIOx->MODER = temp;
		}
		else
		{
			//this part for interrupt mode
			if(GPIO_PinConfigArray[counter].GPIO_PinMode == GPIO_MODE_IT_FT)
			{
				//configure the FTSR
				EXTI->FTSR |= ( One_bit_shift << PinActualNumber);
				//clear the corresponding RTSR bit
				EXTI->RTSR &= ~(One_bit_mask << PinActualNumber);
			}
			else if(GPIO_PinConfigArray[counter].GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				//configure the RTSR
				EXTI->RTSR |= ( One_bit_shift << PinActualNumber);
				//clear the correspnding RISR bit
				EXTI->FTSR &= ~( One_bit_mask << PinActualNumber);
			}
			else if(GPIO_PinConfigArray[counter].GPIO_PinMode == GPIO_MODE_IT_RFT)
			{
				//configure the FTSR and RTSR
				EXTI->RTSR |= ( One_bit_shift << PinActualNumber);

				EXTI->FTSR |= ( One_bit_shift << PinActualNumber);
			}

			//configure the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1 = PinActualNumber / Four_Pins_for_SYSCFG_EXTICR;
			uint8_t temp2 = PinActualNumber % Four_Pins_for_SYSCFG_EXTICR;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOx);
			SYSCFG->EXTICR[temp1] = portcode << (temp2 * Four_bits_shift);
			SYSCFG_PCLK_EN;
			//enable the exti interrupt delivery using IMR
			EXTI->IMR |= (One_bit_shift << GPIO_PinConfigArray[counter]	.GPIO_PinNumber);
		}

		temp = 0;

		//configure the speed
		temp = pGPIOx->OSPEEDR;
		temp &= ~(0x3UL << (PinActualNumber * 2u));
		temp |= ((GPIO_PinConfigArray[counter].GPIO_PinSpeed & 0x00000003U) << (PinActualNumber * 2u));
		pGPIOx->OSPEEDR = temp;

		//configure the pupd setting
		temp = pGPIOx->PUPDR;
		temp &= ~(0x3UL << (PinActualNumber * 2u));
		temp |= ((GPIO_PinConfigArray[counter].GPIO_PinPuPdControl & 0x00000003U) << (PinActualNumber * 2u));
		pGPIOx->PUPDR = temp;

		//configure the optype
		temp = pGPIOx->OTYPER;
		temp &= ~(0x1UL << (PinActualNumber * 1u));
		temp |= ((GPIO_PinConfigArray[counter].GPIO_PinOPType & 0x00000001U) << (PinActualNumber * 1u));
		pGPIOx->OTYPER = temp;

		//configure the alternate functionality
		if(GPIO_PinConfigArray[counter].GPIO_PinMode == GPIO_MODE_ALTFN)
		{
			//alternate function
			temp = pGPIOx->AFR[PinActualNumber >> 3u];
			temp &= ~(0xFu << ((PinActualNumber & 0x07u) * 4u));
			temp |= ((GPIO_PinConfigArray[counter].GPIO_PinAltFunMode ) << ((PinActualNumber & 0x07u) * 4u));
			pGPIOx->AFR[PinActualNumber >> 3u] = temp;
		}

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
void GPIO_RESET(uint8_t PORT_num)
{
	GPIO_RESET_REG |= (One_bit_shift << PORT_num);
	GPIO_RESET_REG &= ~(One_bit_mask << PORT_num);

	return;

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
PIN_STATE GPIO_ReadInputPin(uint8_t Pin)
{
	uint8_t value;

	uint8_t PortNumber=0;
	uint8_t PinActualNumber=0;

	PortNumber = ((Pin / PORT_NUMBER_OF_BITS_IN_REG));
	PinActualNumber = (Pin % PORT_NUMBER_OF_BITS_IN_REG);

	GPIO_regdef_t *pGPIOx = GPIO_Arr[PortNumber];     // new

	value = (uint8_t)((pGPIOx->IDR  >> PinActualNumber) & Number_one_in_HEX );

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
void GPIO_WriteOutputPin(uint8_t Pin, uint8_t Value)
{

	uint8_t PortNumber=0;
	uint8_t PinActualNumber=0;

	PortNumber = ((Pin / PORT_NUMBER_OF_BITS_IN_REG));
	PinActualNumber = (Pin % PORT_NUMBER_OF_BITS_IN_REG);

	GPIO_regdef_t *pGPIOx = GPIO_Arr[PortNumber];     // new

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field coreesponding to the pin
		pGPIOx->ODR |= ( One_bit_shift << PinActualNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~( One_bit_mask << PinActualNumber);
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

//void GPIO_ToggleOutputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber);
void GPIO_ToggleOutputPin(uint8_t PORT_num, uint8_t PinNumber)
{
	uint8_t PortNumber=0;
	uint8_t PinActualNumber=0;

	PortNumber = ((PinNumber / PORT_NUMBER_OF_BITS_IN_REG));
	PinActualNumber = (PinNumber % PORT_NUMBER_OF_BITS_IN_REG);

	GPIO_regdef_t *pGPIOx = GPIO_Arr[PortNumber];     // new

	pGPIOx->ODR ^= (One_bit_shift << PinActualNumber);
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
		if(IRQNumber <= last_regBit_num_31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= ( One_bit_shift << (IRQNumber % ISER_reg_32Bits) );
		}

		else if(IRQNumber > last_regBit_num_31 && IRQNumber < Double_32Bits_for_ISER_reg)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= ( One_bit_shift << (IRQNumber % ISER_reg_32Bits) );
		}
		else if(IRQNumber >= Double_32Bits_for_ISER_reg && IRQNumber < Four_times_32Bits_FOR_ISER_reg)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= ( One_bit_shift << (IRQNumber % Double_32Bits_for_ISER_reg) );
		}
		else
		{
			if(IRQNumber <= last_regBit_num_31)
			{
				//Program ICER0 register
				*NVIC_ICER0 |= ( One_bit_shift << (IRQNumber % ICER_reg_32Bits) );
			}

			else if(IRQNumber > last_regBit_num_31 && IRQNumber < Double_32Bits_for_ICER_reg)
			{
				//Program ICER1 register
				*NVIC_ICER1 |= ( One_bit_shift << (IRQNumber % ICER_reg_32Bits) );
			}
			else if(IRQNumber >= Double_32Bits_for_ICER_reg && IRQNumber < Four_times_32Bits_FOR_ICER_reg)
			{
				//Program ICER2 register
				*NVIC_ICER2 |= ( One_bit_shift << (IRQNumber % Double_32Bits_for_ICER_reg) );
			}
		}
	}
}



void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//find out the ipr register
	uint8_t iprx = IRQNumber / Four_Pins_for_IPR_reg;
	uint8_t iprx_section = IRQNumber % Four_Pins_for_IPR_reg;

	uint8_t shift_amount = (Eight_reg_bits * iprx_section) + (Eight_reg_bits - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDR + (iprx * Four_Pins_for_IPR_reg)) |= ( IRQPriority << shift_amount );

}


//GPIO_IRQ handling

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( One_bit_mask << PinNumber))
	{
		//clear
		EXTI->PR |= (One_bit_mask << PinNumber);
	}
}
