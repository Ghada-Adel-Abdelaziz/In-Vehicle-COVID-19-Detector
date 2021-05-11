/******************************************************************************
 * Module: 		GPIO
 * File Name: 	GPIO_Lcfg.c
 * Description: Link time Configuration Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa&Ghada
 * Date:		9/1/2021
 ******************************************************************************/

#include "GPIO_lcfg.h"
#include "GPIO.h"
/* GPIO Pre-Compile Configuration Header file */
#include "GPIO_cfg.h"



/************************************************************************
 *      creating instance and initializing configuration structure      *
 ************************************************************************/

GPIO_pinconfig_t GPIO_PinConfigArray[NUMBER_OF_CONFIGURED_PINS] =
/*	PinNumber       PinMode           PinSpeed         PinPuPdControl    PinOPType   AltFunMode */
{
	{GREEN_LED,    GPIO_MODE_OUT,    GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	{ORANGE_LED,   GPIO_MODE_OUT,    GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	{RED_LED,      GPIO_MODE_OUT,    GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	//{BLUE_LED,   GPIO_MODE_OUT,    GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	{USART2_Tx,    GPIO_MODE_ALTFN,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   7},
	{USART2_Rx,    GPIO_MODE_ALTFN,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   7},
	{ADC_CH1,      GPIO_MODE_ANALOG, GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   0},
	{TIMER2_CH1,   GPIO_MODE_ALTFN,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP,   1},   // OC1 pin

};
