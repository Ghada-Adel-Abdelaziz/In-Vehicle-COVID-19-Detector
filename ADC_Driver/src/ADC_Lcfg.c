/******************************************************************************
 * Module: 		ADC
 * File Name: 	ADC_Lcfg.c
 * Description: Link time Configuration Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa&Ghada
 * Date:		17/1/2021
 ******************************************************************************/
#include "ADC_Cfg.h"
#include "ADC.h"
#include "ADC_Lcfg.h"


void LED_ON(void)   // I2C TX complete
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

ADC_Config_t ADC_ConfigArray[ NUMBER_OF_CONFIGURED_ADC]=
		/*	ADC_ID    ADC MODE   ADC_Prescaler         ADC_Resolution     Data_alignment       */
{
		{ADC_1, ADC_RESOLUTION_10_BIT, SCAN_MODE_ENABLE, CONTINOUS_CONVERSION_MODE_ENABLE, Trigger_detection_disabled, ADC_ExternalTrigConv_T1_CC1, Right_alignment, 1, ADC_PCLK2_divided_by_2, LED_ON}
};


ADC_RegularCH_Config_t RegCH_ConfigArray[NUMBER_OF_CONFIGURED_CHANNEL] =
{
		//{ADC_1, ADC_Channel_0, RANK_1, ADC_SampleTime_3Cycles},
		{ADC_1, ADC_Channel_1, RANK_2, ADC_SampleTime_3Cycles}
};
