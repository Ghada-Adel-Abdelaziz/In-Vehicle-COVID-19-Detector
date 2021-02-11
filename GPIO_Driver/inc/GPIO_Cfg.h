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
#define NUMBER_OF_CONFIGURED_PINS  4

#define LED0      GPIO_CHANNEL_D12
#define LED1      GPIO_CHANNEL_D13
//#define LED2      GPIO_CHANNEL_D15
//#define LED3      GPIO_CHANNEL_D15
#define trial_Tx    GPIO_CHANNEL_A2
#define trial_Rx    GPIO_CHANNEL_A3
/*********************/

/*********************/

#endif /* GPIO_CFG_H_ */

