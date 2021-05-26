/*
 *  CAN.h
 *
 *  Created on: April 23, 2021
 *      Author: Ghada & Toaa
 */
/*----------------------------------------------------------------------------
 * Name:    CAN.h
 * Purpose: CAN interface for STM32
 * Version: V1.00
 */

#ifndef _CAN_H_
#define _CAN_H_

typedef struct  {
	uint32_t  id;                 // 29 bit identifier
	uint8_t  data[8];            // Data field
	uint8_t  len;                // Length of data field in bytes
	uint8_t  format;             // 0 - STANDARD, 1- EXTENDED IDENTIFIER
	uint8_t  type;               // 0 - DATA FRAME, 1 - REMOTE FRAME
				
} CAN_msg;

typedef enum
{
	NOT_OK,
	OK
}CAN_Transmission_STATUS;
typedef enum
{
	NOT_OK,
	OK
}CAN_Get_MSG_STATUS;



/* Functions defined in module CAN.c */

void CAN_init          (void);
void CAN_wrMsg (CAN_msg *msg); 
void CAN_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void get_MSG(CAN_msg *MSG);






#endif // _CAN_H_
