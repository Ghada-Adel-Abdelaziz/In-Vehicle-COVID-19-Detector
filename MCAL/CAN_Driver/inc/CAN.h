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

typedef enum
{
	NOT_OK,
	OK
}CAN_Transmission_STATUS;



/* Functions defined in module CAN.c */

void CAN_init          (void);
void CAN_wrMsg      (CAN_msg *msg);
void CAN_rdMsg      (CAN_msg *msg);


extern CAN_msg       CAN_RxMsg[3];                          // CAN message for receiving




#endif // _CAN_H_
