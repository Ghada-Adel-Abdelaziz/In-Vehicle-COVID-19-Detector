/******************************************************************************
 * Module: 		ADC
 * File Name: 	ADC.c
 * Description: ADC Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa&Ghada
 * Date:		17/1/2021
 ******************************************************************************/

#include "Common_Macros.h"

#include "ADC_Cfg.h"
#include "ADC.h"
#include "ADC_Lcfg.h"

static void ADC_PeripheralControl(uint8_t ADC_ID, uint8_t Cmd);
static void ADC_PeriClockControl(uint8_t ADC_ID,uint8_t EnorDi);
static void ADC_PeripheralControl(uint8_t ADC_ID, uint8_t Cmd)
{

	ADC_EN =(ADC_EN & ~(One_bit_shift << ADC_CR1_ADON))
																		|(Cmd << ADC_CR1_ADON);

}
static void ADC_PeriClockControl(uint8_t ADC_ID,uint8_t EnCLK)
{
	ADC_PCLK_EN =(ADC_PCLK_EN & ~(One_bit_shift << ADC_ID))
																	|(EnCLK << ADC_ID);
}

void ADC_Init(void)
{

	//Temporary variable
	uint32_t TempReg=0;
	uint8_t counter=0;
	ADC_RegDef_t *pADCx;
	uint32_t temp = 0;  //temp register

	for(;counter<NUMBER_OF_CONFIGURED_adc;counter++)
	{
		/***Implement the code to enable the Clock for given ADC peripheral***/
		ADC_PeriClockControl(ADC_ConfigArray[counter].ADC_ID,ENABLE);
		ADC_PeripheralControl(ADC_ConfigArray[counter].ADC_ID,ENABLE);

		/***Implement the code to configure the Word length configuration item**/
		//		/******************************** Configuration of CR2******************************************/
		TempReg |= ADC_ConfigArray[counter].Data_alignment << ADC_CR2_ALIGN ;
		//configure the PRESCALER

		temp = (ADC_ConfigArray[counter].ADC_Prescaler << (Two_bits_shift * ADC_CCR_PRE));
		pADCx->ADC_CCR &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CCR_PRE );
		pADCx->ADC_CCR |= temp;
		//configure the PRESCALER
		temp = (ADC_ConfigArray[counter].ADC_Resolution << (Two_bits_shift * ADC_CCR_RES));
		pADCx->ADC_CR1 &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CCR_RES );
		pADCx->ADC_CR1 |= temp;
		temp=0;
		//configure the mode of ADC pin
		switch(ADC_ConfigArray[counter].ADC_Mode)
		{
		case REGULAR :
			pADCx->ADC_CR1 |= ( One_bit_shift << ADC_CR1_JEOCIE);
			temp = (ADC_ConfigArray[counter].External_trigger << (Two_bits_shift * ADC_CR1_EXTEN));
								pADCx->ADC_CR2 &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CR1_EXTEN );
								pADCx->ADC_CR2 |= temp;
			break;
		case INJECTED:
			pADCx->ADC_CR1 |= ( One_bit_shift << ADC_CR1_EOCIE);
			temp = (ADC_ConfigArray[counter].External_trigger << (Two_bits_shift * ADC_CR1_JEXTEN));
					pADCx->ADC_CR2 &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CR1_JEXTEN );
					pADCx->ADC_CR2 |= temp;

			break;
		}

	}
}

