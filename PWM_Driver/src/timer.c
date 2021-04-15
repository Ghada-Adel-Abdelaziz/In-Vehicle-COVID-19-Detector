/*
 * timer.c
 *
 *  Created on: Feb 27, 2021
 *      Author: Toqa & Ghada
 */

#include "timer.h"
#include "timer_Cfg.h"
#include "timer_Lcfg.h"
#include "Common_Macros.h"


#include "GPIO_Lcfg.h"



#define TIM_MODE_BIT_LOC_IN_REG     4
#define TIM_MODE_BIT_MASK			0xFF8F

#define CLK_DIV_BIT_LOC_IN_REG		8
#define CLOK_DIV_BIT_MASK           0xFCFF

#define TIM_ENABLE_BIT_LOC_IN_REG	0
#define TIM_ENABLE_BIT_MASK			0xFFFD


// output compare1 bits mask
#define OC1M_BIT_LOC_IN_REG			4
#define OC1M_BIT_MASK				0xFF8F

#define OC1S_BIT_LOC_IN_REG			0
#define OC1S_BIT_MASK				0xFFFC

#define OC1_POL_BIT_LOC_IN_REG		1
#define OC1_PLO_BIT_MASK			0xFFFD

#define OC1_EN_BIT_LOC_IN_REG		0
#define OC1_EN_BIT_MASK				0xFFFE


// output compare2 bits mask
#define OC2M_BIT_LOC_IN_REG			12
#define OC2M_BIT_MASK				0x8FFF

#define OC2S_BIT_LOC_IN_REG			8
#define OC2S_BIT_MASK				0xFCFF

#define OC2_POL_BIT_LOC_IN_REG		5
#define OC2_PLO_BIT_MASK			0xFFDF

#define OC2_EN_BIT_LOC_IN_REG		4
#define OC2_EN_BIT_MASK				0xFFEF

// output compare3 bits mask
#define OC3M_BIT_LOC_IN_REG			4
#define OC3M_BIT_MASK				0xFF8F

#define OC3S_BIT_LOC_IN_REG			0
#define OC3S_BIT_MASK				0xFFFC

#define OC3_POL_BIT_LOC_IN_REG		9
#define OC3_PLO_BIT_MASK			0xFDFF

#define OC3_EN_BIT_LOC_IN_REG		8
#define OC3_EN_BIT_MASK				0xFEFF

// output compare4 bits mask
#define OC4M_BIT_LOC_IN_REG			12
#define OC4M_BIT_MASK				0x8FFF

#define OC4S_BIT_LOC_IN_REG			8
#define OC4S_BIT_MASK				0xFCFF

#define OC4_POL_BIT_LOC_IN_REG		13
#define OC4_PLO_BIT_MASK			0xDFFF

#define OC4_EN_BIT_LOC_IN_REG		12
#define OC4_EN_BIT_MASK				0xEFFF




// flag bit masks
#define UPD_EVENT_INT_FLAG		0



uint16_t x = 0;

void (*TIM2_ptr[NUM_OF_TIM])(void);
void (*TIM3_ptr[NUM_OF_TIM])(void);


TIM_RegDef_t *Timer_Arr[NUM_OF_TIM] = {TIMER2,TIMER3,TIMER4,TIMER5};


static void Timer_PeriClockControl(uint8_t TIM_ID,uint8_t EnCLK)
{
	TIM_PCLK_EN = (TIM_PCLK_EN & ~(One_bit_shift << TIM_ID)) | (EnCLK << TIM_ID);
}


static char TIM_GetFlagStatus(uint8_t TIM_ID , uint32_t FlagName)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[TIM_ID];

	return ((pTIMx->SR & (One_bit_shift << FlagName)) >> FlagName );
}


//HA Review: function areguments type should be added explicitly (done)
void Timer_Init(void)
{
	//Temporary variable
	uint32_t TempReg=0;
	uint8_t counter=0;
	TIM_RegDef_t *pTIMx;
	uint32_t temp = 0;  //temp register

    //HA Review: Timer to be switched of before starting init (done)
	for(counter = 0; counter<NUMBER_OF_CONFIGURED_TIMER; counter++)
	{
		Timer_Cmd(TIM_ConfigArray[counter].TIMER_ID, STOP);
		/***Implement the code to enable the Clock for given ADC peripheral***/
		Timer_PeriClockControl(TIM_ConfigArray[counter].TIMER_ID,ENABLE);     // enable TIM clock

		pTIMx = Timer_Arr[TIM_ConfigArray[counter].TIMER_ID];

		/* configure prescaler */

		pTIMx->PSC = TIM_ConfigArray[counter].TIM_Prescaler;


		/* configure timer mode */
		temp = ( TIM_ConfigArray[counter].TIM_CounterMode << TIM_MODE_BIT_LOC_IN_REG);
		pTIMx->CR1 &= ( TIM_MODE_BIT_MASK );
		pTIMx->CR1 |= temp;

		temp = 0;

		// set the time periode
		pTIMx->ARR = TIM_ConfigArray[counter].TIM_Period;

		/* configure clock division */
		temp = ( TIM_ConfigArray[counter].TIM_ClockDivision << CLK_DIV_BIT_LOC_IN_REG);
		pTIMx->CR1 &= ( CLOK_DIV_BIT_MASK );
		pTIMx->CR1 |= temp;


		/*we may need to generate an update event here to activate
		 *
		the prescaler reg initialization*/
		
		TIM2_ptr[TIM_ConfigArray[counter].TIMER_ID] = TIM_ConfigArray[counter].TIMER2_CompleteFunptr;
		TIM3_ptr[TIM_ConfigArray[counter].TIMER_ID] = TIM_ConfigArray[counter].TIMER3_CompleteFunptr;

	}
}




void TIM_OC_Init(void)
{
	//Temporary variable
	uint32_t TempReg=0;
	uint8_t counter=0;
	TIM_RegDef_t *pTIMx;
	uint32_t temp = 0;  //temp register

	for(counter = 0; counter<NUMBER_OF_CONFIGURED_CHANNEL; counter++)
	{
		pTIMx = Timer_Arr[OC_ConfigArray[counter].TIMER_ID];


		switch( OC_ConfigArray[counter].CH_ID )
		{
		case CH1_ID:

			// Select the required CH as output compare
			temp = 0;
			pTIMx->CCMR1 &= (OC1S_BIT_MASK);
			pTIMx->CCMR1 |= temp;

			temp = 0;

			// output compare mode configurations
			temp = OC_ConfigArray[counter].TIM_OCMode << OC1M_BIT_LOC_IN_REG;
			pTIMx->CCMR1 &= (OC1M_BIT_MASK);
			pTIMx->CCMR1 |= temp;

			temp = 0;

			// polarity configurations
			temp = OC_ConfigArray[counter].TIM_OCPolarity << OC1_POL_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC1_PLO_BIT_MASK);
			pTIMx->CCER |= temp;

			temp = 0;

			// Set compare value
			pTIMx->CCR1 = OC_ConfigArray[counter].TIM_Pulse;

			temp = 0;

			// output compare enable
			temp = OC_ConfigArray[counter].TIM_OutputState << OC1_EN_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC1_EN_BIT_MASK);
			pTIMx->CCER |= temp;

			break;

		case CH2_ID:
			// Select the required CH as output compare
			temp = 0;
			pTIMx->CCMR1 &= (OC2S_BIT_MASK);
			pTIMx->CCMR1 |= temp;

			temp = 0;

			// output compare mode configurations
			temp = OC_ConfigArray[counter].TIM_OCMode << OC2M_BIT_LOC_IN_REG;
			pTIMx->CCMR1 &= (OC2M_BIT_MASK);
			pTIMx->CCMR1 |= temp;

			temp = 0;

			// polarity configurations
			temp = OC_ConfigArray[counter].TIM_OCPolarity << OC2_POL_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC2_PLO_BIT_MASK);
			pTIMx->CCER |= temp;

			temp = 0;

			// Set compare value
			pTIMx->CCR2 = OC_ConfigArray[counter].TIM_Pulse;

			temp = 0;

			// output compare enable
			temp = OC_ConfigArray[counter].TIM_OutputState << OC2_EN_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC2_EN_BIT_MASK);
			pTIMx->CCER |= temp;
			break;

		case CH3_ID:
			// Select the required CH as output compare
			temp = 0;
			pTIMx->CCMR2 &= (OC3S_BIT_MASK);
			pTIMx->CCMR2 |= temp;

			temp = 0;

			// output compare mode configurations
			temp = OC_ConfigArray[counter].TIM_OCMode << OC3M_BIT_LOC_IN_REG;
			pTIMx->CCMR2 &= (OC3M_BIT_MASK);
			pTIMx->CCMR2 |= temp;

			temp = 0;

			// polarity configurations
			temp = OC_ConfigArray[counter].TIM_OCPolarity << OC3_POL_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC3_PLO_BIT_MASK);
			pTIMx->CCER |= temp;

			temp = 0;

			// Set compare value
			pTIMx->CCR3 = OC_ConfigArray[counter].TIM_Pulse;

			temp = 0;

			// output compare enable
			temp = OC_ConfigArray[counter].TIM_OutputState << OC3_EN_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC3_EN_BIT_MASK);
			pTIMx->CCER |= temp;
			break;

		case CH4_ID:
			// Select the required CH as output compare
			temp = 0;
			pTIMx->CCMR2 &= (OC4S_BIT_MASK);
			pTIMx->CCMR2 |= temp;

			temp = 0;

			// output compare mode configurations
			temp = OC_ConfigArray[counter].TIM_OCMode << OC4M_BIT_LOC_IN_REG;
			pTIMx->CCMR2 &= (OC4M_BIT_MASK);
			pTIMx->CCMR2 |= temp;

			temp = 0;

			// polarity configurations
			temp = OC_ConfigArray[counter].TIM_OCPolarity << OC4_POL_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC4_PLO_BIT_MASK);
			pTIMx->CCER |= temp;

			temp = 0;

			// Set compare value
			pTIMx->CCR4 = OC_ConfigArray[counter].TIM_Pulse;

			temp = 0;

			// output compare enable
			temp = OC_ConfigArray[counter].TIM_OutputState << OC4_EN_BIT_LOC_IN_REG;
			pTIMx->CCER &= (OC4_EN_BIT_MASK);
			pTIMx->CCER |= temp;
			break;
		}
	}

}




void Timer_SetCounter(uint8_t TIM_ID, uint32_t Counter)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[TIM_ID];

	/* Set the Counter Register value */
	pTIMx->CNT = Counter;
}

void Timer_SetAutoreload(uint8_t TIM_ID, uint32_t Autoreload)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[TIM_ID];

	/* Set the Autoreload Register value */
	pTIMx->ARR = Autoreload;
}

void Timer_Cmd(uint8_t TIM_ID, TIMER_STATE cmd)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[TIM_ID];

	if (cmd == START)
	{
		/* Enable the TIM Counter */
		pTIMx->CR1 |= (One_bit_shift << TIM_ENABLE_BIT_LOC_IN_REG);
	}
	else
	{
		/* Disable the TIM Counter */
		pTIMx->CR1 &= ~(One_bit_shift << TIM_ENABLE_BIT_LOC_IN_REG);
	}
}



void CC1R_SetValue(uint8_t TIM_ID, uint32_t Counter)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[TIM_ID];

	/* Set the Counter Register value */
	pTIMx->CCR1 = Counter;        // note: preload is disabled meaning that the value will have effect immediately

}

void TIM_IntControl(uint8_t TIM_ID , uint8_t IntSource , uint8_t State)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[TIM_ID];

	switch(State)
	{
	case ENABLE:
		//Implement the code to enable interrupt for IntSource
		pTIMx->DIER |= ( One_bit_shift << IntSource);
		break;
	case DISABLE:
		//Implement the code to disable interrupt for IntSource
		pTIMx->DIER &= ~( One_bit_shift << IntSource);
		break;
	}

}


void TIM_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


void TIM2_IRQHandler(void)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[0];


	if( TIM_GetFlagStatus(TIMER2_ , UPD_EVENT_INT_FLAG) == 1)
	{
		//HA review:callback to be added (done)
		
		TIM2_ptr[TIMER2_]();

		pTIMx->SR &= ~(1 << UPD_EVENT_INT_FLAG);     // this flag must be cleared here as per data sheet page 634

	}

}


void TIM3_IRQHandler(void)
{
	TIM_RegDef_t *pTIMx;
	pTIMx = Timer_Arr[1];


	if( TIM_GetFlagStatus(TIMER3_ , UPD_EVENT_INT_FLAG) == 1)
	{
		TIM3_ptr[TIMER3_]();
		x++;
		pTIMx->SR &= ~(1 << UPD_EVENT_INT_FLAG);     // this flag must be cleared here as per data sheet page 634
	}
}
