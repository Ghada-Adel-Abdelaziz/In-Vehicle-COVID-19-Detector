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




/* Functions defined in module CAN.c */

void CAN_init          (void);
void CAN_wrMsg      (CAN_msg *msg, u8 u8MailBox);
void CAN_rdMsg        (CAN_msg *msg);
void CAN_wrFilter      (void);
u8 CAN_u8GetNumberOfPendingMessage(void);
void CAN_vid_filter_list(u8 no_of_filters);




#endif // _CAN_H_
