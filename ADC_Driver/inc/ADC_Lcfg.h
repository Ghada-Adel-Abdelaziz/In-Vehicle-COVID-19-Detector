/******************************************************************************
 * Module: 		ADC
 * File Name: 	ADC_Lcfg.h
 * Description: Link time  Configuration Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa & Ghada
 * Date:		17/1/2021
 ******************************************************************************/

#ifndef ADC_LCFG_H_
#define ADC_LCFG_H_

#include "stm32f4xxx.h"
#include "ADC_Cfg.h"
/*************************************************************/
/******************** link time configuration parameters*************************************/



typedef struct
{
	uint8_t ADC_ID;        				// POSSIBLE VALUES FROM @GPIO ADC IDs
	uint8_t ADC_Resolution;         	// possible values from @ADC RESOLUTION MODES
	uint8_t ADC_ScanConvMode;       	// possible values from @ADC SCAN_MODES
	uint8_t ADC_ContinuousConvMode;  	// possible values from @ADC CONTINOUS_MODES
	uint8_t External_trigger;		    // Possible options for External_trigger
	uint8_t External_trigger_selection; // Select the external event used to trigger the start of conversion of a regular group.
	uint8_t Data_alignment;   			//possible values from @GPIO PUSHPULL CONTROL

	uint8_t  ADC_NbrOfConversion;        /*!< Specifies the number of ADC conversions
                                               that will be done using the sequencer for
                                               regular channel group.
                                               This parameter must range from 1 to 16. */

	uint8_t ADC_Prescaler;          //possible values from @ADC PRESCALER MODES
} ADC_Config_t;




// @ADC IDs
#define ADC_1   0
#define ADC_2   1
#define ADC_3   2


//ADC pin possible output RESOLUTION
//@ADC RESOLUTION MODES

#define ADC_RESOLUTION_12_BIT    0
#define ADC_RESOLUTION_10_BIT    1
#define ADC_RESOLUTION_8_BIT     2
#define ADC_RESOLUTION_6_BIT     3



//@SCAN_MODES

#define SCAN_MODE_ENABLE	1
#define SCAN_MODE_DISABLE	0

//@ADC CONTINOUS_MODES
#define CONTINOUS_CONVERSION_MODE_ENABLE	1
#define CONTINOUS_CONVERSION_MODE_DISABLE	0

/**@Possible options for External_trigger for regular channel ***/
#define Trigger_detection_disabled    							   0
#define Trigger_detection_on_the_rising_edge   					   1
#define Trigger_detection_on_the_FALLING_edge   			       2
#define Trigger_detection_on_both_the_rising_and_falling_edges     3

// external trigger selection for regular group
#define ADC_ExternalTrigConv_T1_CC1                ((uint32_t)0)
#define ADC_ExternalTrigConv_T1_CC2                ((uint32_t)1)
#define ADC_ExternalTrigConv_T1_CC3                ((uint32_t)2)
#define ADC_ExternalTrigConv_T2_CC2                ((uint32_t)3)
#define ADC_ExternalTrigConv_T2_CC3                ((uint32_t)4)
#define ADC_ExternalTrigConv_T2_CC4                ((uint32_t)5)
#define ADC_ExternalTrigConv_T2_TRGO               ((uint32_t)6)
#define ADC_ExternalTrigConv_T3_CC1                ((uint32_t)7)
#define ADC_ExternalTrigConv_T3_TRGO               ((uint32_t)8)
#define ADC_ExternalTrigConv_T4_CC4                ((uint32_t)9)
#define ADC_ExternalTrigConv_T5_CC1                ((uint32_t)10)
#define ADC_ExternalTrigConv_T5_CC2                ((uint32_t)11)
#define ADC_ExternalTrigConv_T5_CC3                ((uint32_t)12)
#define ADC_ExternalTrigConv_T8_CC1                ((uint32_t)13)
#define ADC_ExternalTrigConv_T8_TRGO               ((uint32_t)14)
#define ADC_ExternalTrigConv_Ext_IT11              ((uint32_t)15)



//GPIO pin output types
//@GPIO PIN OP TYPE//

#define Right_alignment   0
#define Left_alignment    1


//ADC pin possible output PRESCALER
//@ADC PRESCALER MODES

#define ADC_PCLK2_divided_by_2    0
#define ADC_PCLK2_divided_by_4    1
#define ADC_PCLK2_divided_by_6    2
#define ADC_PCLK2_divided_by_8    3






//GPIO pin output types
//@GPIO PIN OP TYPE

#define CONVERSION_1   0
#define CONVERSION_2   1
#define CONVERSION_3   2
#define CONVERSION_4   3
#define CONVERSION_5   4
#define CONVERSION_6   5
#define CONVERSION_7   6
#define CONVERSION_8   7
#define CONVERSION_9   8
#define CONVERSION_10  9
#define CONVERSION_11   10
#define CONVERSION_12   11
#define CONVERSION_13   12
#define CONVERSION_14   13
#define CONVERSION_15   14
#define CONVERSION_16   15


//GPIO pin output types
//@ADC Injected sequence length

#define INJECTED_CONVERSION_1   0
#define INJECTED_CONVERSION_2   1
#define INJECTED_CONVERSION_3   2
#define INJECTED_CONVERSION_4   3



/* regular channel data structure */
typedef struct
{
	uint8_t ADC_ID;
	uint8_t CH_ID;        // available values exists @ CH IDs
    uint8_t CH_Rank;
    uint8_t Sample_Time;
} ADC_RegularCH_Config_t;


// @ CH IDs
#define ADC_Channel_0             ((uint8_t)0)
#define ADC_Channel_1             ((uint8_t)1)
#define ADC_Channel_2             ((uint8_t)2)
#define ADC_Channel_3             ((uint8_t)3)
#define ADC_Channel_4             ((uint8_t)4)
#define ADC_Channel_5             ((uint8_t)5)
#define ADC_Channel_6             ((uint8_t)6)
#define ADC_Channel_7             ((uint8_t)7)
#define ADC_Channel_8             ((uint8_t)8)
#define ADC_Channel_9             ((uint8_t)9)
#define ADC_Channel_10            ((uint8_t)10)
#define ADC_Channel_11            ((uint8_t)11)
#define ADC_Channel_12            ((uint8_t)12)
#define ADC_Channel_13            ((uint8_t)13)
#define ADC_Channel_14            ((uint8_t)14)
#define ADC_Channel_15            ((uint8_t)15)
#define ADC_Channel_16            ((uint8_t)16)
#define ADC_Channel_17            ((uint8_t)17)
#define ADC_Channel_18            ((uint8_t)18)


//@Rank
#define RANK_1		1
#define RANK_2		2
#define RANK_3		3
#define RANK_4		4
#define RANK_5		5
#define RANK_6		6
#define RANK_7		7
#define RANK_8		8
#define RANK_9		9
#define RANK_10		10
#define RANK_11		11
#define RANK_12		12
#define RANK_13		13
#define RANK_14		14
#define RANK_15 	15
#define RANK_16		16


/** @defgroup ADC_sampling_times
  * @{
  */
#define ADC_SampleTime_3Cycles                    ((uint8_t)0x00)
#define ADC_SampleTime_15Cycles                   ((uint8_t)0x01)
#define ADC_SampleTime_28Cycles                   ((uint8_t)0x02)
#define ADC_SampleTime_56Cycles                   ((uint8_t)0x03)
#define ADC_SampleTime_84Cycles                   ((uint8_t)0x04)
#define ADC_SampleTime_112Cycles                  ((uint8_t)0x05)
#define ADC_SampleTime_144Cycles                  ((uint8_t)0x06)
#define ADC_SampleTime_480Cycles                  ((uint8_t)0x07)



extern ADC_Config_t ADC_ConfigArray[NUMBER_OF_CONFIGURED_ADC];
extern ADC_RegularCH_Config_t RegCH_ConfigArray[NUMBER_OF_CONFIGURED_CHANNEL];

#endif /* ADC_LCFG_H_ */
