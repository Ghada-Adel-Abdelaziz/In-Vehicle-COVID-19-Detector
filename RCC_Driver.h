/*
 * RCC_Driver.h
 *
 *  Created on: Dec 30, 2020
 *      Author: esraa
 */

#ifndef RCC_DRIVER_H_
#define RCC_DRIVER_H_


#include "stm32f4xxx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);


#endif /* RCC_DRIVER_H_ */
