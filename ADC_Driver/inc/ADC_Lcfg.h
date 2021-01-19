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
#define ADC_1   8
#define ADC_2   9
#define ADC_3   10


typedef struct
{
	uint8_t ADC_ID;        //POSSIBLE VALUES FROM @GPIO ADC IDs
	uint8_t ADC_Mode;          //possible values from @GPIO PIN MODES
	uint8_t ADC_Prescaler;          //possible values from @ADC PRESCALER MODES
	uint8_t ADC_Resolution;         //possible values from @ADC RESOLUTION MODES
	uint8_t Data_alignment;   //possible values from @GPIO PUSHPULL CONTROL
	uint8_t  External_trigger;
	uint8_t Regular_channel_sequence_length;        //possible values from @GPIO PIN OP TYPE
	uint8_t Injected_channel_sequence_length;      //possible values from    @ADC Injected sequence length

} ADC_Config_t;

//ADC pin possible output PRESCALER
//@ADC PRESCALER MODES

#define ADC_PCLK2_divided_by_2    0
#define ADC_PCLK2_divided_by_4    1
#define ADC_PCLK2_divided_by_6    2
#define ADC_PCLK2_divided_by_8    3

//ADC pin possible output RESOLUTION
//@ADC RESOLUTION MODES

#define ADC_RESOLUTION_12_BIT    0
#define ADC_RESOLUTION_10_BIT    1
#define ADC_RESOLUTION_8_BIT     2
#define ADC_RESOLUTION_6_BIT     3

//GPIO pin output types
//@GPIO PIN OP TYPE

#define Right_alignment   0
#define Left_alignment    1

//ADC MODE
#define REGULAR   0
#define INJECTED  1
/**Possible options for External_trigger***/
#define Trigger_detection_disabled    0
#define Trigger_detection_on_the_rising_edge    1
#define Trigger_detection_on_the_FALLING_edge     2
#define Trigger_detection_on_both_the_rising_and_falling_edges     3

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




extern ADC_Config_t ADC_ConfigArray[NUMBER_OF_CONFIGURED_adc];

#endif /* ADC_LCFG_H_ */
