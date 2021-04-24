/*
 * CAN.C
 *
 *  Created on: APRIL 23, 2021
 *      Author: Toqa & Ghada
 */
#include "CAN.h"

CAN_msg       CAN_TxMsg[3];                          // CAN messge for sending
CAN_msg       CAN_RxMsg[3];                          // CAN message for receiving

u8  CAN_TxRdy[3] = {0};                      // CAN HW ready to transmit a message
u8  CAN_RxRdy = {0};                      // CAN HW received a message
u32  filt_counter;

u8 value_0[8];
u8 u8MailBox0Flag;
u8 value_1[8];
u8 u8MailBox1Flag;
u8 value_2[8];
u8 u8MailBox2Flag;

void CAN_init(void)  {

}



/*----------------------------------------------------------------------------
  leave initialisation mode
 ----------------------------------------------------------------------------/
void CAN_start (void)  {


}


/*----------------------------------------------------------------------------
  wite a message to CAN peripheral and transmit it
 ----------------------------------------------------------------------------/
void CAN_wrMsg (CAN_msg *msg, u8 u8MailBox)  {


}


/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it
 ----------------------------------------------------------------------------/
void CAN_rdMsg_0 (CAN_msg *msg)  {
	// Read identifier information
	
}

void CAN_wrFilter (filter_type *pstrfilter)  {
}


void CAN_vid_filter_list(filter_type *pstrfilters,u8 no_of_filters)
{
	

}


/*----------------------------------------------------------------------------
  CAN transmit interrupt handler
 ----------------------------------------------------------------------------/
void USB_HP_CAN1_TX_IRQHandler (void) {

	
}


/*----------------------------------------------------------------------------
  CAN receive interrupt handler
 ----------------------------------------------------------------------------/
void USB_LP_CAN1_RX0_IRQHandler (void) {
	
}