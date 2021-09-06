/******************************************************************************
 * Module:      GPIO
 * File Name:   GPIO_Cfg.h
 * Description: Pre-Compile Configuration Header file for
 *              STM32F407 Microcontroller
 * Author:      Toqa&Ghada
 * Date:        9/1/2021
 ******************************************************************************/


#ifndef GPIO_CFG_H_
#define GPIO_CFG_H_


/* number of configured pins which is the size of the configuration array of structures */
#define NUMBER_OF_CONFIGURED_PINS  9


#define GREEN_LED      	GPIO_CHANNEL_D12
#define ORANGE_LED      GPIO_CHANNEL_D13
#define RED_LED         GPIO_CHANNEL_D14
#define BLUE_LED      	GPIO_CHANNEL_D15
#define USART2_Tx    	GPIO_CHANNEL_A2
#define USART2_Rx    	GPIO_CHANNEL_A3

#define USART3_Tx    	GPIO_CHANNEL_B10
#define USART3_Rx    	GPIO_CHANNEL_B11
#define USART4_Tx    	GPIO_CHANNEL_A0
#define USART4_Rx    	GPIO_CHANNEL_A1
#define CAN_Tx          GPIO_CHANNEL_D1
#define CAN_Rx          GPIO_CHANNEL_D0


#define Mask_Request    GPIO_CHANNEL_B0
#define Covid_Request   GPIO_CHANNEL_B1
#define API_Request    GPIO_CHANNEL_B2


/*********************/

/*********************/

#endif /* GPIO_CFG_H_ */

