/******************************************************************************
 * Module: 		ADC
 * File Name: 	ADC.c
 * Description: ADC Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa & Ghada
 * Date:		26/3/2021
 ******************************************************************************/

#include "Common_Macros.h"
#include "ADC_Cfg.h"
#include "ADC.h"
#include "ADC_Lcfg.h"

// descriptive macros for magic numbers

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

#define CH_ID_CONTROLED_BY_ADC_SMPR2_EACH_3_BITS_FOR_EACH_CH    9
#define ADC_SMPR2_BITS_STEP_SHIFT_LEFT                          3
#define ADC_SMPR1_BITS_STEP_SHIFT_LEFT                          3
#define THREE_BITS_MASK                                         7
#define NUM_OF_1st_10_CHs_CONTROLED_BY_ADC_SMPR2                10

#define FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR2           7
#define FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR1           13
#define ADC_SQR1_BITS_STEP_SHIFT_LEFT                           5
#define ADC_SQR2_BITS_STEP_SHIFT_LEFT                           5
#define ADC_SQR3_BITS_STEP_SHIFT_LEFT                           5
#define FIVE_BITS_MASKING                                       31

#define NUM_OF_REG_BITS                                         32

#define ADC_EOC_BIT_LOCATION_IN_SR                              1

float analog_value = 0;

uint16_t ADC_Readings[NUMBER_OF_CONFIGURED_CHANNEL] = {0};


ADC_RegDef_t* ADC_Arr[3] = {ADC1_BASE, ADC2_BASE, ADC3_BASE};

static void ADC_PeripheralControl(uint8_t ADC_ID, uint8_t Cmd);
static void ADC_PeriClockControl(uint8_t ADC_ID,uint8_t EnorDi);

static void ADC_PeripheralControl(uint8_t ADC_ID, uint8_t Cmd)
{
	ADC_RegDef_t *pADCx;
	pADCx = ADC_Arr[ADC_ID];

	pADCx->ADC_CR2 = ( pADCx->ADC_CR2 & ~(One_bit_shift << ADC_CR1_ADON) ) | (Cmd << ADC_CR1_ADON);

}
static void ADC_PeriClockControl(uint8_t ADC_ID,uint8_t EnCLK)
{
	ADC_PCLK_EN =(ADC_PCLK_EN & ~(One_bit_shift << ADC_ID+8)) | (EnCLK << ADC_ID+8);
}

void ADC_Init(void)
{

	//Temporary variable
	uint32_t TempReg=0;
	uint8_t counter=0;
	ADC_RegDef_t *pADCx;
	uint32_t temp = 0;  //temp register


	ADC_Common_RegDef_t *p = ADC_BASE;    // pointer to common ADC base address

	for(counter = 0; counter<NUMBER_OF_CONFIGURED_ADC; counter++)
	{
		/***Implement the code to enable the Clock for given ADC peripheral***/
		ADC_PeriClockControl(ADC_ConfigArray[counter].ADC_ID,ENABLE);     // enable ADC clock
		ADC_PeripheralControl(ADC_ConfigArray[counter].ADC_ID,ENABLE);    // enable ADC module

		pADCx = ADC_Arr[ADC_ConfigArray[counter].ADC_ID];

		/* configure resolution */
		temp = ( ADC_ConfigArray[counter].ADC_Resolution << RESOLUTION_BIT_LOCATION_IN_REG);
		pADCx->ADC_CR1 &= ( RESOLUTION_BIT_MASK );
		pADCx->ADC_CR1 |= temp;

		temp = 0;

		/* configure scan mode */
		temp = ( ADC_ConfigArray[counter].ADC_ScanConvMode << SCAN_BIT_LOCATION_IN_REG);
		pADCx->ADC_CR1 &= ( SCAN_BIT_MASK );
		pADCx->ADC_CR1 |= temp;

		temp = 0;

		/* continous mode configuration */
		temp = ( ADC_ConfigArray[counter].ADC_ContinuousConvMode << CONTINOUS_BIT_LOCATION_IN_REG);
		pADCx->ADC_CR2 &= ( CONTINOUS_BIT_MASK );
		pADCx->ADC_CR2 |= temp;

		temp = 0;

		/* external trigger configuration for regular channel */
		temp = ( ADC_ConfigArray[counter].External_trigger << EXTERNAL_TRIGGER_BIT_LOCATION_FOR_REGULAR_GROUP);
		pADCx->ADC_CR2 &= ( EXTERNAL_TRIGGER_BIT_MASK_FOR_REGULAR_GROUP );
		pADCx->ADC_CR2 |= temp;

		temp = 0;

		/* external trigger selection for regular channel*/
		temp = ( ADC_ConfigArray[counter].External_trigger_selection << EXTERNAL_TRIGGER_SELECTION_BIT_LOCATION_IN_REG_FOR_REG_GROUP);
		pADCx->ADC_CR2 &= ( EXTERNAL_TRIGGER_SELECTION_BIT_MASK_FOR_REG_GROUP );
		pADCx->ADC_CR2 |= temp;

		temp = 0;

		/* data alignment configuration */
		temp = ( ADC_ConfigArray[counter].Data_alignment << DATA_ALIGN_BIT_LOCATION);
		pADCx->ADC_CR2 &= ( DATA_ALIGN_BIT_MASK );
		pADCx->ADC_CR2 |= temp;


		temp = 0;

		/*************************** SQR1 configuration ****************************************/
		/* number of conversion configuration */
		temp = ( ADC_ConfigArray[counter].ADC_NbrOfConversion << NUM_OF_CONVERSION_BIT_LOCATION_IN_REG);
		pADCx->ADC_SQR1 &= ( NUM_OF_CONVERSION_BIT_MASK );
		pADCx->ADC_SQR1 |= temp;



		temp = 0;


		/*********************** CCR configuration *****************************/
		/* Prescaller configuration */
		temp = ( ADC_ConfigArray[counter].ADC_Prescaler << ADC_PRESCALE_BIT_LOCATION);
		p->ADC_CCR &= ( ADC_PRESCALE_BIT_MASK );
		p->ADC_CCR |= temp;

		/*************************************** END OF ADC CONFIGURATION **************************************/

	}
}

void ADC_RegularChannelConfig(void)
{
	ADC_RegDef_t *pADCx;
	uint32_t temp = 0;

	uint8_t counter = 0;

	for( counter=0; counter<NUMBER_OF_CONFIGURED_CHANNEL; counter++ )
	{

		pADCx = ADC_Arr[RegCH_ConfigArray[counter].ADC_ID];

		/* Smaple time configurations */

		if( RegCH_ConfigArray[counter].CH_ID <= CH_ID_CONTROLED_BY_ADC_SMPR2_EACH_3_BITS_FOR_EACH_CH )
		{
			temp = RegCH_ConfigArray[counter].Sample_Time << (ADC_SMPR2_BITS_STEP_SHIFT_LEFT * RegCH_ConfigArray[counter].CH_ID);
			pADCx->ADC_SMPR2 &= ~( THREE_BITS_MASK << (ADC_SMPR2_BITS_STEP_SHIFT_LEFT * RegCH_ConfigArray[counter].CH_ID) );  // masking
			pADCx->ADC_SMPR2 |= temp;
		}
		else if( RegCH_ConfigArray[counter].CH_ID > CH_ID_CONTROLED_BY_ADC_SMPR2_EACH_3_BITS_FOR_EACH_CH )
		{
			temp = RegCH_ConfigArray[counter].Sample_Time << (ADC_SMPR1_BITS_STEP_SHIFT_LEFT * (RegCH_ConfigArray[counter].CH_ID - NUM_OF_1st_10_CHs_CONTROLED_BY_ADC_SMPR2));
			pADCx->ADC_SMPR1 &= ~( THREE_BITS_MASK << (ADC_SMPR2_BITS_STEP_SHIFT_LEFT * (RegCH_ConfigArray[counter].CH_ID - NUM_OF_1st_10_CHs_CONTROLED_BY_ADC_SMPR2)) );  // masking
			pADCx->ADC_SMPR1 |= temp;
		}

		temp = 0;

		/* Sequence configurations */
		if( RegCH_ConfigArray[counter].CH_Rank < FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR2 )
		{
			temp = RegCH_ConfigArray[counter].CH_ID << (ADC_SQR3_BITS_STEP_SHIFT_LEFT * (RegCH_ConfigArray[counter].CH_Rank-1));
			pADCx->ADC_SQR3 &= ~(FIVE_BITS_MASKING << (RegCH_ConfigArray[counter].CH_Rank-1));
			pADCx->ADC_SQR3 |= temp;
		}
		else if( RegCH_ConfigArray[counter].CH_Rank < FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR1 )
		{
			temp = RegCH_ConfigArray[counter].CH_ID << (ADC_SQR2_BITS_STEP_SHIFT_LEFT * (RegCH_ConfigArray[counter].CH_Rank - FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR2));
			pADCx->ADC_SQR2 &= ~(FIVE_BITS_MASKING << (RegCH_ConfigArray[counter].CH_Rank - FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR2));
			pADCx->ADC_SQR2 |= temp;
		}
		else
		{
			temp = RegCH_ConfigArray[counter].CH_ID << (ADC_SQR1_BITS_STEP_SHIFT_LEFT * (RegCH_ConfigArray[counter].CH_Rank - FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR1));
			pADCx->ADC_SQR1 &= ~(FIVE_BITS_MASKING << (RegCH_ConfigArray[counter].CH_Rank - FIRST_REGULAR_SEQ_CONVERSION_RANK_IN_ADC_SQR1));
			pADCx->ADC_SQR1 |= temp;
		}
	}

}


void ADC_SoftwareStartConv(uint8_t ADC_ID)
{
	ADC_RegDef_t *pADCx;

	pADCx = ADC_Arr[ADC_ID];

	pADCx->ADC_CR2 |= (SW_START_CONV);

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
	// HA Review: Conversion state should be checked before returning ressults 
	// MOK-> conversion not requested
	//Busy-> Conversion requested but not done.
	//OK -> conversion result updated
	ADC_RegDef_t *pADCx;
	pADCx = ADC_Arr[ADC_ID];

	return (uint16_t)pADCx->ADC_DR;
}


uint16_t ADC_getValue(uint8_t Ch_Num)
{
	uint8_t i = 0;
	// HA Review: Conversion state should be checked before returning ressults 
	// MOK-> conversion not requested
	//Busy-> Conversion requested but not done.
	//OK -> conversion result updated - ISR called
	ADC_RegDef_t *pADCx;

	for(i=0; i<NUMBER_OF_CONFIGURED_CHANNEL; i++)
	{
		if( RegCH_ConfigArray[i].CH_ID == Ch_Num)
		{
			return ADC_Readings[RegCH_ConfigArray[i].CH_Rank - 1];
		}
	}
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


	ISER_Num = IRQNumber / NUM_OF_REG_BITS;
	IRQActualNumber = IRQNumber % NUM_OF_REG_BITS;


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

void ADC_IRQHandler(void)
{
	static uint32_t i = 0;

	ADC_RegDef_t *pADCx;
	pADCx = ADC_Arr[0];

    //HA Review: To search for which channel fired the IRQ. Then update the reading buffer by the channel index
	if( pADCx->ADC_SR & (One_bit_shift << ADC_EOC_BIT_LOCATION_IN_SR) )
	{
		//first check the end of conversion flag
		ADC_Readings[i] = pADCx->ADC_DR;

		if( i == NUMBER_OF_CONFIGURED_CHANNEL - 1 )
		{
			i = 0;
		}
		else
		{
			i++;
		}
	}

}