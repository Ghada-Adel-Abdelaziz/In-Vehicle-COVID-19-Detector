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
void CAN_start         (void);
void CAN_wrMsg      (CAN_msg *msg, u8 u8MailBox);
void CAN_rdMsg_0        (CAN_msg *msg);
void CAN_rdMsg_1        (CAN_msg *msg);
void CAN_wrFilter      (void);

void CAN_vid_filter_list(u8 no_of_filters);

extern CAN_msg       CAN_TxMsg[3];      // CAN messge for sending
extern CAN_msg       CAN_RxMsg[3];      // CAN message for receiving
extern u8  CAN_TxRdy[3];      // CAN HW ready to transmit a message
extern u8  CAN_RxRdy;      // CAN HW received a message



extern u8 value_0[8];
extern u8 value_1[8];
extern u8 value_2[8];

extern u8 u8MailBox0Flag;
extern u8 u8MailBox1Flag;
extern u8 u8MailBox2Flag;


#endif // _CAN_H_