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
// descriptive macros for magic numbers
#define One_bit_shift                        1
#define Two_bits_shift                       2
#define Two_consecutive_bits_mask_by_HEX    0x3


#define RESOLUTION_BIT_MASK                              				 0xFCFFFFFF
#define RESOLUTION_BIT_LOCATION_IN_REG 				     				 24

#define EXTERNAL_TRIGGER_BIT_MASK_FOR_REGULAR_GROUP      				 0xCFFFFFFF
#define EXTERNAL_TRIGGER_BIT_LOCATION_FOR_REGULAR_GROUP  				 28

#define EXTERNAL_TRIGGER_SELECTION_BIT_MASK_FOR_REG_GROUP 			     0xF0FFFFFF
#define EXTERNAL_TRIGGER_SELECTION_BIT_LOCATION_IN_REG_FOR_REG_GROUP     24

#define DATA_ALIGN_BIT_MASK					  	 0xFFFFF7FF
#define DATA_ALIGN_BIT_LOCATION	 			   	 11


#define ADC_PRESCALE_BIT_MASK          			 0xFFFCFFFF
#define ADC_PRESCALE_BIT_LOCATION  				 16

#define SCAN_BIT_MASK		   					 0xFFFFFEFF
#define SCAN_BIT_LOCATION_IN_REG				 8

#define CONTINOUS_BIT_MASK					     0xFFFFFFFD
#define CONTINOUS_BIT_LOCATION_IN_REG			 1


#define NUM_OF_CONVERSION_BIT_MASK        		 0xFF0FFFFF
#define NUM_OF_CONVERSION_BIT_LOCATION_IN_REG    20


#define SW_START_CONV	((uint32_t)0x40000000)
#define ADC_CR2_EOCS 	((uint32_t)0x00000400)

ADC_RegDef_t* ADC_Arr[3] = {ADC1_BASE, ADC2_BASE, ADC3_BASE};

static void ADC_PeripheralControl(uint8_t ADC_ID, uint8_t Cmd);
static void ADC_PeriClockControl(uint8_t ADC_ID,uint8_t EnorDi);

static void ADC_PeripheralControl(uint8_t ADC_ID, uint8_t Cmd)
{
	//ADC_EN =(ADC_EN & ~(One_bit_shift << ADC_CR1_ADON)) | (Cmd << ADC_CR1_ADON);
	ADC_RegDef_t *pADCx;
	pADCx = ADC_Arr[ADC_ID];

	pADCx->ADC_CR2 = ( pADCx->ADC_CR2 & ~(One_bit_shift << ADC_CR1_ADON) ) | (Cmd << ADC_CR1_ADON);

}
static void ADC_PeriClockControl(uint8_t ADC_ID,uint8_t EnCLK)
{
	ADC_PCLK_EN =(ADC_PCLK_EN & ~(One_bit_shift << ADC_ID)) | (EnCLK << ADC_ID);
}

void ADC_Init(void)
{

	//Temporary variable
	uint32_t TempReg=0;
	uint8_t counter=0;
	ADC_RegDef_t *pADCx;
	uint32_t temp = 0;  //temp register

	for(counter = 0; counter<NUMBER_OF_CONFIGURED_ADC; counter++)
	{
		/***Implement the code to enable the Clock for given ADC peripheral***/
		ADC_PeriClockControl(ADC_ConfigArray[counter].ADC_ID,ENABLE);
		ADC_PeripheralControl(ADC_ConfigArray[counter].ADC_ID,ENABLE);

		pADCx = ADC_Arr[ADC_ConfigArray[counter].ADC_ID];

		/* configure resolution */
		temp = ( ADC_ConfigArray[counter].ADC_Resolution << RESOLUTION_BIT_LOCATION_IN_REG);
		pADCx->ADC_CR1 &= ( RESOLUTION_BIT_MASK );
		pADCx->ADC_CR1 |= temp;

		/* configure scan mode */
		temp = ( ADC_ConfigArray[counter].ADC_ScanConvMode << SCAN_BIT_LOCATION_IN_REG);
		pADCx->ADC_CR1 &= ( SCAN_BIT_MASK );
		pADCx->ADC_CR1 |= temp;

		/* continous mode configuration */
		temp = ( ADC_ConfigArray[counter].ADC_ContinuousConvMode << CONTINOUS_BIT_LOCATION_IN_REG);
		pADCx->ADC_CR2 &= ( CONTINOUS_BIT_MASK );
		pADCx->ADC_CR2 |= temp;

		/* external trigger configuration for regular channel */
		temp = ( ADC_ConfigArray[counter].External_trigger << EXTERNAL_TRIGGER_BIT_LOCATION_FOR_REGULAR_GROUP);
		pADCx->ADC_CR2 &= ( EXTERNAL_TRIGGER_BIT_MASK_FOR_REGULAR_GROUP );
		pADCx->ADC_CR2 |= temp;

		/* external trigger selection for regular channel*/
		temp = ( ADC_ConfigArray[counter].External_trigger_selection << EXTERNAL_TRIGGER_SELECTION_BIT_LOCATION_IN_REG_FOR_REG_GROUP);
		pADCx->ADC_CR2 &= ( EXTERNAL_TRIGGER_SELECTION_BIT_MASK_FOR_REG_GROUP );
		pADCx->ADC_CR2 |= temp;

		/* data alignment configuration */
		temp = ( ADC_ConfigArray[counter].Data_alignment << DATA_ALIGN_BIT_LOCATION);
		pADCx->ADC_CR2 &= ( DATA_ALIGN_BIT_MASK );
		pADCx->ADC_CR2 |= temp;


		/*********************** CCR configuration *****************************/
		/* Prescaller configuration */
		temp = ( ADC_ConfigArray[counter].ADC_Prescaler << ADC_PRESCALE_BIT_LOCATION);
		pADCx->ADC_CCR &= ( ADC_PRESCALE_BIT_MASK );
		pADCx->ADC_CCR |= temp;


		/*************************** SQR1 configuration ****************************************/
		/* number of conversion configuration */
		temp = ( ADC_ConfigArray[counter].ADC_NbrOfConversion << NUM_OF_CONVERSION_BIT_LOCATION_IN_REG);
		pADCx->ADC_SQR1 &= ( NUM_OF_CONVERSION_BIT_MASK );
		pADCx->ADC_SQR1 |= temp;

		/*************************************** END OF ADC CONFIGURATION **************************************/



		//		/***Implement the code to configure the Word length configuration item**/
		//		//		/******************************** Configuration of CR2******************************************/
		//		TempReg |= ADC_ConfigArray[counter].Data_alignment << ADC_CR2_ALIGN ;
		//		//configure the PRESCALER
		//
		//		temp = (ADC_ConfigArray[counter].ADC_Prescaler << (Two_bits_shift * ADC_CCR_PRE));
		//		pADCx->ADC_CCR &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CCR_PRE );
		//		pADCx->ADC_CCR |= temp;
		//		//configure the PRESCALER
		//		temp = (ADC_ConfigArray[counter].ADC_Resolution << (Two_bits_shift * ADC_CCR_RES));
		//		pADCx->ADC_CR1 &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CCR_RES );
		//		pADCx->ADC_CR1 |= temp;
		//		temp=0;
		//		//configure the mode of ADC pin
		//		switch(ADC_ConfigArray[counter].ADC_Mode)
		//		{
		//		case REGULAR :
		//			pADCx->ADC_CR1 |= ( One_bit_shift << ADC_CR1_JEOCIE);
		//			temp = (ADC_ConfigArray[counter].External_trigger << (Two_bits_shift * ADC_CR1_EXTEN));
		//			pADCx->ADC_CR2 &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CR1_EXTEN );
		//			pADCx->ADC_CR2 |= temp;
		//			break;
		//		case INJECTED:
		//			pADCx->ADC_CR1 |= ( One_bit_shift << ADC_CR1_EOCIE);
		//			temp = (ADC_ConfigArray[counter].External_trigger << (Two_bits_shift * ADC_CR1_JEXTEN));
		//			pADCx->ADC_CR2 &= ~( Two_consecutive_bits_mask_by_HEX << ADC_CR1_JEXTEN );
		//			pADCx->ADC_CR2 |= temp;
		//
		//			break;
		//		}

	}
}



//void ADC_RegularChannelConfig(uint8_t ADC_ID, uint8_t ch_num, uint8_t rank, uint8_t sample_time)
//{
//	ADC_RegDef_t *pADCx;
//	uint32_t temp = 0;
//
//	uint8_t counter = 0;
//
//	for( counter=0; counter<NUMBER_OF_CONFIGURED_CHANNEL; counter++ )
//	{
//		/* Smaple time configurations */
//		if( ch_num <= 9 )
//		{
//			temp = sample_time << (3 * ch_num);
//			pADCx->ADC_SMPR2 &= ~( 7 << (3 * ch_num) );  // masking
//			pADCx->ADC_SMPR2 |= temp;
//		}
//		else if( ch_num > 9 )
//		{
//			temp = sample_time << (3 * (ch_num-10));
//			pADCx->ADC_SMPR2 &= ~( 7 << (3 * (ch_num-10)) );  // masking
//			pADCx->ADC_SMPR2 |= temp;
//		}
//
//		temp = 0;
//
//		/* Sequence configurations */
//		if( rank < 7 )
//		{
//			temp = ch_num << (5 * (rank-1));
//			pADCx->ADC_SQR3 &= ~(31 << (rank-1));
//			pADCx->ADC_SQR3 |= temp;
//		}
//		else if( rank < 13 )
//		{
//			temp = ch_num << (5 * (rank-7));
//			pADCx->ADC_SQR2 &= ~(31 << (rank-7));
//			pADCx->ADC_SQR2 |= temp;
//		}
//		else
//		{
//			temp = ch_num << (5 * (rank-13));
//			pADCx->ADC_SQR1 &= ~(31 << (rank-13));
//			pADCx->ADC_SQR1 |= temp;
//		}
//	}
//
//}

void ADC_RegularChannelConfig(void)
{
	ADC_RegDef_t *pADCx;
	uint32_t temp = 0;

	uint8_t counter = 0;

	for( counter=0; counter<NUMBER_OF_CONFIGURED_CHANNEL; counter++ )
	{

		pADCx = ADC_Arr[RegCH_ConfigArray[counter].ADC_ID];
		/* Smaple time configurations */
		if( RegCH_ConfigArray[counter].CH_ID <= 9 )
		{
			temp = RegCH_ConfigArray[counter].Sample_Time << (3 * RegCH_ConfigArray[counter].CH_ID);
			pADCx->ADC_SMPR2 &= ~( 7 << (3 * RegCH_ConfigArray[counter].CH_ID) );  // masking
			pADCx->ADC_SMPR2 |= temp;
		}
		else if( RegCH_ConfigArray[counter].CH_ID > 9 )
		{
			temp = RegCH_ConfigArray[counter].Sample_Time << (3 * (RegCH_ConfigArray[counter].CH_ID-10));
			pADCx->ADC_SMPR1 &= ~( 7 << (3 * (RegCH_ConfigArray[counter].CH_ID-10)) );  // masking
			pADCx->ADC_SMPR1 |= temp;
		}

		temp = 0;

		/* Sequence configurations */
		if( RegCH_ConfigArray[counter].CH_Rank < 7 )
		{
			temp = RegCH_ConfigArray[counter].CH_ID << (5 * (RegCH_ConfigArray[counter].CH_Rank-1));
			pADCx->ADC_SQR3 &= ~(31 << (RegCH_ConfigArray[counter].CH_Rank-1));
			pADCx->ADC_SQR3 |= temp;
		}
		else if( RegCH_ConfigArray[counter].CH_Rank < 13 )
		{
			temp = RegCH_ConfigArray[counter].CH_ID << (5 * (RegCH_ConfigArray[counter].CH_Rank-7));
			pADCx->ADC_SQR2 &= ~(31 << (RegCH_ConfigArray[counter].CH_Rank-7));
			pADCx->ADC_SQR2 |= temp;
		}
		else
		{
			temp = RegCH_ConfigArray[counter].CH_ID << (5 * (RegCH_ConfigArray[counter].CH_Rank-13));
			pADCx->ADC_SQR1 &= ~(31 << (RegCH_ConfigArray[counter].CH_Rank-13));
			pADCx->ADC_SQR1 |= temp;
		}
	}

}


void ADC_SoftwareStartConv(uint8_t ADC_ID)
{
	ADC_RegDef_t *pADCx;

	pADCx = ADC_Arr[ADC_ID];

	pADCx->ADC_CR2 |= SW_START_CONV;

}


void ADC_EOCOnEachRegularChannelCmd(uint8_t ADC_ID, uint8_t State)
{
	ADC_RegDef_t *pADCx;

	pADCx = ADC_Arr[ADC_ID];

	switch(State)
	{
	case ENABLE:
		pADCx->ADC_CR2 |= (ADC_CR2_EOCS);
		break;
	case DISABLE:
		pADCx->ADC_CR2 &= ~(ADC_CR2_EOCS);
		break;
	}
}

uint16_t ADC_GetConversionValue(uint8_t ADC_ID)
{
	ADC_RegDef_t *pADCx;
	pADCx = ADC_Arr[ADC_ID];

	return (uint16_t)pADCx->ADC_DR;
}


void ADC_IntControl(uint8_t ADC_ID , uint8_t IntSource , uint8_t State)
{
	ADC_RegDef_t *pADCx;
	pADCx = ADC_Arr[ADC_ID];

	switch(State)
	{
	case ENABLE:
		//Implement the code to enable interrupt for IntSource
		pADCx->ADC_CR1 |= ( One_bit_shift << IntSource);
		break;
	case DISABLE:
		//Implement the code to disable interrupt for IntSource
		pADCx->ADC_CR1 &= ~( One_bit_shift << IntSource);
		break;
	}

}


void ADC_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	uint8_t ISER_Num=0;
	uint8_t IRQActualNumber=0;


	ISER_Num = IRQNumber / 32;
	IRQActualNumber = IRQNumber % 32;


	switch(EnorDi)
	{
	case ENABLE:
		NVIC_ISER_Base_Addr[ISER_Num] = 1<< IRQActualNumber;
		break;
	case DISABLE:
		NVIC_ICER_Base_Addr[ISER_Num] = 1<< IRQActualNumber;
		break;
	}
}

void ADC1_IRQHandler(void)
{
	uint32_t data = 0;

	data = ADC_GetConversionValue(ADC_1);
}
