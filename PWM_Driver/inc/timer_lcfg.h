/*
 * timer_lcfg.h
 *
 *  Created on: Feb 27, 2021
 *      Author: esraa
 */

#ifndef TIMER_LCFG_H_
#define TIMER_LCFG_H_


typedef struct
{
	uint8_t TIMER_ID;				  /*!< Specifies the id of timer to be configured.
                                       This parameter can be a value of @ref TIM_ID */

	uint16_t TIM_Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                       This parameter can be a number between 0x0000 and 0xFFFF */

	uint16_t TIM_CounterMode;       /*!< Specifies the counter mode.
                                       This parameter can be a value of @ref TIM_Counter_Mode */

	uint32_t TIM_Period;            /*!< Specifies the period value to be loaded into the active
                                       Auto-Reload Register at the next update event.
                                       This parameter must be a number between 0x0000 and 0xFFFF.  */

	uint16_t TIM_ClockDivision;     /*!< Specifies the clock division.
                                      This parameter can be a value of @ref TIM_Clock_Division_CKD */

} TIM_Config_t;


typedef struct
{

	uint8_t  CH_ID;					/*!< Specifies the channel ID.
                                   This parameter can be a value of @ref CH_ID */


	uint8_t  TIMER_ID;


	//uint8_t  OC_Selection;        /*!< Specifies the channel output compare selection.
                                   //This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */

	uint16_t TIM_OCMode;        /*!< Specifies the TIM mode.
                                   This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */

	uint32_t TIM_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                                   This parameter can be a number between 0x0000 and 0xFFFF */

	uint16_t TIM_OCPolarity;    /*!< Specifies the output polarity.
                                   This parameter can be a value of @ref TIM_Output_Compare_Polarity */

	uint16_t TIM_OutputState;   /*!< Specifies the TIM Output Compare state.
                                   This parameter can be a value of @ref TIM_Output_Compare_State */

} TIM_OC_Config_t;


/** @ref TIM_ID
 * @{
 */

#define   TIMER2_	0
#define   TIMER3_   1
#define   TIMER4_   2
#define   TIMER5_   3



/** @defgroup TIM_Counter_Mode
 * @{
 */
#define TIM_CounterMode_Up                 ((uint16_t)0x0000)
#define TIM_CounterMode_Down               ((uint16_t)0x0001)
#define TIM_CounterMode_CenterAligned1     ((uint16_t)0x0002)
#define TIM_CounterMode_CenterAligned2     ((uint16_t)0x0004)
#define TIM_CounterMode_CenterAligned3     ((uint16_t)0x0006)


/** @defgroup TIM_Clock_Division_CKD
 * @{
 */

#define TIM_CKD_DIV1                       ((uint16_t)0x0000)
#define TIM_CKD_DIV2                       ((uint16_t)0x0001)
#define TIM_CKD_DIV4                       ((uint16_t)0x0002)




//////////////////// output compare configurations /////////////////

/** @defgroup TIM_Output_Compare_and_PWM_modes
 * @{
 */

#define TIM_OCMode_Timing                  ((uint16_t)0x0000)
#define TIM_OCMode_Active                  ((uint16_t)0x0001)
#define TIM_OCMode_Inactive                ((uint16_t)0x0002)
#define TIM_OCMode_Toggle                  ((uint16_t)0x0003)
#define TIM_OCMode_PWM1                    ((uint16_t)0x0006)
#define TIM_OCMode_PWM2                    ((uint16_t)0x0007)



/** @defgroup TIM_Output_Compare_State
 * @{
 */

#define TIM_OutputState_Disable            ((uint16_t)0x0000)
#define TIM_OutputState_Enable             ((uint16_t)0x0001)


/** @defgroup TIM_Output_Compare_Polarity
 * @{
 */

#define TIM_OCPolarity_High                ((uint16_t)0x0000)
#define TIM_OCPolarity_Low                 ((uint16_t)0x0002)


/**
@ref CH_ID
*/

#define CH1_ID	1
#define CH2_ID	2
#define CH3_ID	3
#define CH4_ID	4



extern TIM_Config_t TIM_ConfigArray[NUMBER_OF_CONFIGURED_TIMER];

extern TIM_OC_Config_t OC_ConfigArray[NUMBER_OF_CONFIGURED_CHANNEL];

#endif /* TIMER_LCFG_H_ */
