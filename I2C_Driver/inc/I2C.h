/*
 * I2C.h
 *
 *  Created on: Apr 2, 2021
 *      Author: esraa
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xxx.h"

/********************* interrupt sources *************************/


typedef enum
{
	I2C_E_OK,
	I2C_E_NOT_OK
}I2C_ERROR_STATUS;


typedef enum
{
	I2C_EVENT_INTERRUPT = 0x09,
	I2C_BUFFER_INTERRUPT = 0x0A
}I2C_INTERRUPT_SOURCE;

typedef enum
{
	READ,
	WRITE
}DIRECTION;

typedef enum
{
   IDLE,
   TRANSMIT,
   RECEIVE
}I2C_STATUS;

void I2C_Init(void);
I2C_ERROR_STATUS I2C_SendDataAsync(uint8_t I2C_ID , uint8_t* Data , uint16_t DataSize, uint8_t slave_address , uint16_t loc_address);
I2C_ERROR_STATUS I2C_ReceiveDataASync(uint8_t I2C_ID , uint8_t* Data, uint16_t DataSize, uint8_t slave_address, uint16_t loc_address);

void I2C_IntControl(uint8_t I2C_ID , uint8_t IntSource , uint8_t State);
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
#endif /* I2C_H_ */
