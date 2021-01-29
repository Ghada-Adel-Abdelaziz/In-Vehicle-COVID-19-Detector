/******************************************************************************
 * Module:      GPIO
 * File Name:   GPIO.h
 * Description: Header file for GPIO Module on STM32F407 Microcontroller
 * Author:      Toqa & Ghada
 * Date:        9/1/2021
 ******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

/* GPIO Pre-Compile Configuration Header file */
#include "GPIO_cfg.h"
//HA 10/1/2020: To be removed

//configuration structure for GPIO

typedef enum
{
	HIGH,
	LOW
}PIN_STATE;


//GPIO pin modes
//@GPIO PIN MODES

void GPIO_Init(void);
void GPIO_RESET(uint8_t PORT_num);
PIN_STATE GPIO_ReadInputPin(uint8_t Pin);
uint16_t GPIO_ReadInputPort(uint8_t PORT_num);
void GPIO_WriteOutputPin(uint8_t Pin, uint8_t Value);
void GPIO_WriteOutputPort(uint8_t PORT_num, uint16_t Value);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* GPIO_H_ */
