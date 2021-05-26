/*
 * I2C_lcfg.h
 *
 *  Created on: Apr 2, 2021
 *      Author: esraa
 */

#ifndef I2C_LCFG_H_
#define I2C_LCFG_H_

#include "stm32f4xxx.h"
#include "I2C_cfg.h"
#include "stdint.h"


typedef struct
{
	uint8_t I2C_ID;        				// POSSIBLE VALUES FROM @GPIO I2C IDs

	uint32_t I2C_ClockSpeed;          /*!< Specifies the clock frequency.
                                         This parameter must be set to a value lower than 400kHz */

  uint16_t I2C_Mode;                /*!< Specifies the I2C mode.
                                         This parameter can be a value of @ref I2C_mode */

  uint16_t I2C_FastModeDutyCycle;           /*!< Specifies the I2C fast mode duty cycle.
                                         This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

  uint16_t I2C_OwnAddress1;         /*!< Specifies the first device own address.
                                         This parameter can be a 7-bit or 10-bit address. */


  uint16_t I2C_AdressingMode; /*!< Specifies if 7-bit or 10-bit address is acknowledged.
                                         This parameter can be a value of @ref I2C_acknowledged_address */

  void (*TX_CompleteFunptr)(void);
}I2C_Config_t;






// @I2C IDs
#define I2C_1   0
#define I2C_2   1
#define I2C_3   2


/** @defgroup I2C_mode
  * @{
  */

#define I2C_Mode_I2C                    ((uint16_t)0x0000)
#define I2C_Mode_SMBusDevice            ((uint16_t)0x0002)
#define I2C_Mode_SMBusHost              ((uint16_t)0x000A)


/**
  * @}
  */

/** @defgroup I2C_duty_cycle_in_fast_mode
  * @{
  */

#define I2C_DutyCycle_16_9              ((uint16_t)0x01) /*!< I2C fast mode Tlow/Thigh = 16/9 */
#define I2C_DutyCycle_2                 ((uint16_t)0x00) /*!< I2C fast mode Tlow/Thigh = 2 */



/** @defgroup I2C_acknowledgement
  * @{
  */

#define I2C_Ack_Enable                  ((uint16_t)0x0400)
#define I2C_Ack_Disable                 ((uint16_t)0x0000)

/** @defgroup I2C_acknowledged_address
  * @{
  */

#define I2C_Address_7bit    ((uint16_t)0x4000)
#define I2C_Address_10bit   ((uint16_t)0xC000)



extern I2C_Config_t I2C_ConfigArray[NUMBER_OF_CONFIGURED_I2C];

#endif /* I2C_LCFG_H_ */
