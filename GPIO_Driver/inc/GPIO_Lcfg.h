/******************************************************************************
 * Module: 		GPIO
 * File Name: 	UART_Lcfg.h
 * Description: Link time  Configuration Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa & Ghada
 * Date:		9/1/2021
 ******************************************************************************/

#ifndef GPIO_LCFG_H_
#define GPIO_LCFG_H_

#include "stm32f4xxx.h"

#include "GPIO_cfg.h"
#include "stdint.h"
/*************************************************************/
/******************** link time configuration parameters*************************************/
#define GPIO_CHANNEL_A0              (uint8_t)0
#define GPIO_CHANNEL_A1              (uint8_t)1
#define GPIO_CHANNEL_A2              (uint8_t)2
#define GPIO_CHANNEL_A3              (uint8_t)3
#define GPIO_CHANNEL_A4              (uint8_t)4
#define GPIO_CHANNEL_A5              (uint8_t)5
#define GPIO_CHANNEL_A6              (uint8_t)6
#define GPIO_CHANNEL_A7              (uint8_t)7
#define GPIO_CHANNEL_A8              (uint8_t)8
#define GPIO_CHANNEL_A9              (uint8_t)9
#define GPIO_CHANNEL_A10             (uint8_t)10
#define GPIO_CHANNEL_A11             (uint8_t)11
#define GPIO_CHANNEL_A12             (uint8_t)12
#define GPIO_CHANNEL_A13             (uint8_t)13
#define GPIO_CHANNEL_A14             (uint8_t)14
#define GPIO_CHANNEL_A15             (uint8_t)15


#define GPIO_CHANNEL_B0              (uint8_t)16
#define GPIO_CHANNEL_B1              (uint8_t)17
#define GPIO_CHANNEL_B2              (uint8_t)18
#define GPIO_CHANNEL_B3              (uint8_t)19
#define GPIO_CHANNEL_B4              (uint8_t)20
#define GPIO_CHANNEL_B5              (uint8_t)21
#define GPIO_CHANNEL_B6              (uint8_t)22
#define GPIO_CHANNEL_B7              (uint8_t)23
#define GPIO_CHANNEL_B8              (uint8_t)24
#define GPIO_CHANNEL_B9              (uint8_t)25
#define GPIO_CHANNEL_B10             (uint8_t)26
#define GPIO_CHANNEL_B11             (uint8_t)27
#define GPIO_CHANNEL_B12             (uint8_t)28
#define GPIO_CHANNEL_B13             (uint8_t)29
#define GPIO_CHANNEL_B14             (uint8_t)30
#define GPIO_CHANNEL_B15             (uint8_t)31

#define GPIO_CHANNEL_D12             (uint8_t)60




#define GPIOA_		0
#define GPIOB_		1
#define GPIOD_		3

/*************************************************************/
/*************************************************************/




#define GPIO_MODE_IN      0
#define GPIO_MODE_OUT     1
#define GPIO_MODE_ALTFN   2
#define GPIO_MODE_ANALOG  3
#define GPIO_MODE_IT_FT   4
#define GPIO_MODE_IT_RT   5
#define GPIO_MODE_IT_RFT  6

//GPIO pin output types
//@GPIO PIN OP TYPE

#define GPIO_TYPE_PP   0       //push pull
#define GPIO_TYPE_OD   1       //open drain

//GPIO pin possible output speed
//@GPIO SPEED MODES

#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

//GPIO pin pull up and pull down configuration macros
//@GPIO PUSHPULL CONTROL

#define GPIO_NO_PUPD  2
#define GPIO_PIN_PU   1
#define GPIO_PIN_PD   0

typedef struct
{
	uint8_t GPIO_PinNumber;        //POSSIBLE VALUES FROM @GPIO PIN NUMBERS
	uint8_t GPIO_PinMode;          //possible values from @GPIO PIN MODES
	uint8_t GPIO_PinSpeed;         //possible values from @GPIO SPEED MODES
	uint8_t GPIO_PinPuPdControl;   //possible values from @GPIO PUSHPULL CONTROL
	uint8_t GPIO_PinOPType;        //possible values from @GPIO PIN OP TYPE
	uint8_t GPIO_PinAltFunMode;

} GPIO_pinconfig_t;


extern GPIO_pinconfig_t GPIO_PinConfigArray[NUMBER_OF_CONFIGURED_PINS];

#endif /* GPIO_LCFG_H_ */
