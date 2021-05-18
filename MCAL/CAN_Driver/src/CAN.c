/*
 * CAN.C
 *
 *  Created on: May 5, 2021
 *      Author: Ghada & Toaa
 */

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xxx.h"
#include "CAN.h"
#include "CAN_cfg.h"
#include "CAN_Lcfg.h"

#include "GPIO_lcfg.h"
#include "GPIO.h"
#include "GPIO_cfg.h"

#define CAN_BUS_TIME_REG_VALUE                                                          0X40060004
#define NUM_OF_MAIL_BOXES                                                                    3

#define STANDARD_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_TX_MAIL_BOX_IDENTIFIER_REG               21
#define EXTENDED_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_TX_MAIL_BOX_IDENTIFIER_REG               3

#define STANDARD_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_RECEIVE_FIFO_MAIL_BOX_IDENTIFIER_REG     21
#define EXTENDED_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_RECEIVE_FIFO_MAIL_BOX_IDENTIFIER_REG     3

#define 1ST_DATA_BYTE_IN_CAN_TDLR_REG                                                        0
#define 2ND_DATA_BYTE_IN_CAN_TDLR_REG                                                        1
#define 3RD_DATA_BYTE_IN_CAN_TDLR_REG                                                        2
#define 4TH_DATA_BYTE_IN_CAN_TDLR_REG                                                        3

#define DATA_BYTE_3_BIT_POSITION_IN_CAN_TDLR_REG                                             24
#define DATA_BYTE_2_BIT_POSITION_IN_CAN_TDLR_REG                                             16
#define DATA_BYTE_1_BIT_POSITION_IN_CAN_TDLR_REG                                             8

#define 1ST_DATA_BYTE_IN_CAN_TDHR_REG                                                        4
#define 2ND_DATA_BYTE_IN_CAN_TDHR_REG                                                        5
#define 3RD_DATA_BYTE_IN_CAN_TDHR_REG                                                        6
#define 4TH_DATA_BYTE_IN_CAN_TDHR_REG                                                        7

#define DATA_BYTE_7_BIT_POSITION_IN_CAN_TDHR_REG                                             24
#define DATA_BYTE_6_BIT_POSITION_IN_CAN_TDHR_REG                                             16
#define DATA_BYTE_5_BIT_POSITION_IN_CAN_TDHR_REG                                             8

#define 1ST_DATA_BYTE_IN_CAN_RDLR_REG                                                        0
#define 2ND_DATA_BYTE_IN_CAN_RDLR_REG                                                        1
#define 3RD_DATA_BYTE_IN_CAN_RDLR_REG                                                        2
#define 4TH_DATA_BYTE_IN_CAN_RDLR_REG                                                        3

#define DATA_BYTE_3_BIT_POSITION_IN_CAN_RDLR_REG                                             24
#define DATA_BYTE_2_BIT_POSITION_IN_CAN_RDLR_REG                                             16
#define DATA_BYTE_1_BIT_POSITION_IN_CAN_RDLR_REG                                             8

#define 1ST_DATA_BYTE_IN_CAN_RDHR_REG                                                        4
#define 2ND_DATA_BYTE_IN_CAN_RDHR_REG                                                        5
#define 3RD_DATA_BYTE_IN_CAN_RDHR_REG                                                        6
#define 4TH_DATA_BYTE_IN_CAN_RDHR_REG                                                        7

#define DATA_BYTE_7_BIT_POSITION_IN_CAN_RDHR_REG                                             24
#define DATA_BYTE_6_BIT_POSITION_IN_CAN_RDHR_REG                                             16
#define DATA_BYTE_5_BIT_POSITION_IN_CAN_RDHR_REG                                             8

#define LAST_FILTER_ID                                                                       13
//HA Review: All variables to be static
CAN_msg       CAN_TxMsg[3];                          // CAN messge for sending
CAN_msg       CAN_RxMsg[3];                          // CAN message for receiving
uint8_t CAN_TxRdy[3] = {0};                      // CAN HW ready to transmit a message
uint8_t  CAN_RxRdy = {0};                      // CAN HW received a message
uint32_t  filt_counter;

uint8_t value_0[8];
uint8_t u8MailBox0Flag;
uint8_t value_1[8];
uint8_t u8MailBox1Flag;
uint8_t value_2[8];
uint8_t u8MailBox2Flag;

static CAN_TypeDef* CAN_Arr[NUM_OF_CAN] = {CAN_1, CAN_2};

static void CAN_waitReady (void);
static void CAN_vidReleaseMessage(void);

static void CAN_wrFilter (void);


static void CAN_PeriClockControl(uint8_t CAN_ID,uint8_t EnCLK)
{
	TIM_PCLK_EN = (TIM_PCLK_EN & ~(One_bit_shift << CAN_ID+25)) | (EnCLK << CAN_ID+25);
}
/*----------------------------------------------------------------------------
  setup CAN interface
 *----------------------------------------------------------------------------*/
void CAN_init(void)  {


	CAN_PeriClockControl( CAN_1, 1);

	CAN1->MCR = (CAN_MCR_INRQ);       				// init mode, enable auto. retransmission                                         // Note: only FIFO 0,
	//transmit mailbox 0 used
	CAN1->MCR |= (1<<CAN_MCR_TXFP); 					//Transmit priority by request order
	CAN1->IER = (CAN_IER_FMPIE0 | CAN_IER_TMEIE);    // FIFO 0 msg pending, Transmit mbx empty (enable interrupts)

	/* Note: this calculations fit for PCLK1 = 36MHz */
	                  

	/* set BTR register so that sample point is at about 72% bit time from bit start */
	/* TSEG1 = 12, TSEG2 = 5, SJW = 4 => 1 CAN bit = 18 TQ, sample at 72%    */

	CAN1->BTR = CAN_BUS_TIME_REG_VALUE;

	CAN_wrFilter( );

	CAN1->MCR &= ~(CAN_MCR_INRQ);                      // normal operating mode, reset INRQ
	while (CAN1->MSR & CAN_MCR_INRQ);
	GPIO_WriteOutputPin(GREEN_LED,1);

	CAN_waitReady ();
}

/*----------------------------------------------------------------------------
  check if transmit mailbox is empty
 *----------------------------------------------------------------------------*/
static void CAN_waitReady (void)  {

//HA Review: no wait loops without timeout
	while ((CAN1->TSR & CAN_TSR_TME0) == 0);         // Transmit mailbox 0 is empty
	while ((CAN1->TSR & CAN_TSR_TME1) == 0);         // Transmit mailbox 0 is empty
	while ((CAN1->TSR & CAN_TSR_TME2) == 0);         // Transmit mailbox 0 is empty

	CAN_TxRdy[0] = 1;
	CAN_TxRdy[1] = 1;
	CAN_TxRdy[2] = 1;

}

/*----------------------------------------------------------------------------
  wite a message to CAN peripheral and transmit it
 ----------------------------------------------------------------------------*/
 //HA Review: function should not return void
void CAN_wrMsg (CAN_msg *msg, u8 u8MessageID, u8 u8Frame, u8 u8DataLength)  {

	uint8_t u8MailBoxIndex = 0;
	uint8_t u8DataCounter = 0;
	uint8_t u8PendingMsgID = 0;
	uint8_t u8Counter = 0;
	CAN_Transmission_STATUS returnValue ;
	for (u8MailBoxIndex = 0; u8MailBoxIndex < NUM_OF_MAIL_BOXES; u8MailBoxIndex++)
	{
		if (CAN_TxRdy[u8MailBoxIndex] == 1)
		{
			returnValue = OK;
			break;
		}

		else
		{
			returnValue = NOT_OK;
		}
	}

	if (u8MailBoxIndex < NUM_OF_MAIL_BOXES)
	{
		CAN1->sTxMailBox[u8MailBoxIndex].TIR  = (uint32_t)0;      // Reset TIR register
		// Setup identifier information
		if (msg->format == STANDARD_FORMAT)
		{
			CAN1->sTxMailBox[u8MailBoxIndex].TIR |= (uint32_t)(msg->id << STANDARD_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_TX_MAIL_BOX_IDENTIFIER_REG) | (CAN_ID_STD);
		}
		else
		{    // Extended ID

			CAN1->sTxMailBox[u8MailBoxIndex].TIR |= (uint32_t)(msg->id <<  EXTENDED_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_TX_MAIL_BOX_IDENTIFIER_REG) | (CAN_ID_EXT);
		}
		// Setup type information
		if (msg->type == DATA_FRAME)
		{   // DATA FRAME
			CAN1->sTxMailBox[u8MailBoxIndex].TIR |= (CAN_RTR_DATA);
			CAN1->sTxMailBox[u8MailBoxIndex].TDLR = (((uint32_t)msg->data[4TH_DATA_BYTE_IN_CAN_TDLR_REG] << DATA_BYTE_3_BIT_POSITION_IN_CAN_TDLR_REG) |
					((uint32_t)msg->data[3RD_DATA_BYTE_IN_CAN_TDLR_REG] << DATA_BYTE_2_BIT_POSITION_IN_CAN_TDLR_REG) |
					((uint32_t)msg->data[2ND_DATA_BYTE_IN_CAN_TDLR_REG] << DATA_BYTE_1_BIT_POSITION_IN_CAN_TDLR_REG) |
					((uint32_t)msg->data[1ST_DATA_BYTE_IN_CAN_TDLR_REG])        );

			CAN1->sTxMailBox[u8MailBoxIndex].TDHR = (((uint32_t)msg->data[4TH_DATA_BYTE_IN_CAN_RDHR_REG] << DATA_BYTE_7_BIT_POSITION_IN_CAN_TDHR_REG) |
					((uint32_t)msg->data[3RD_DATA_BYTE_IN_CAN_RDHR_REG] << DATA_BYTE_6_BIT_POSITION_IN_CAN_TDHR_REG) |
					((uint32_t)msg->data[2ND_DATA_BYTE_IN_CAN_RDHR_REG] << DATA_BYTE_5_BIT_POSITION_IN_CAN_TDHR_REG) |
					((uint32_t)msg->data[1ST_DATA_BYTE_IN_CAN_RDHR_REG])        );

			/* Send Message */
			switch (u8Frame)
			{
			case CAN_u8REMOTEFRAME:
				CAN_TxMsg[u8MailBoxIndex].type = REMOTE_FRAME;
				CAN_TxMsg[u8MailBoxIndex].id = u8MessageID;                              // initialise message to send
				CAN_TxMsg[u8MailBoxIndex].len = u8DataLength;
				CAN_TxMsg[u8MailBoxIndex].format = STANDARD_FORMAT;
				CAN_TxRdy[u8MailBoxIndex] = 0;
				CAN_wrMsg(&(CAN_TxMsg[u8MailBoxIndex]), u8MailBoxIndex);
				break;

			case CAN_u8DATAFRAME:
				CAN_TxMsg[u8MailBoxIndex].type = DATA_FRAME;
				CAN_TxMsg[u8MailBoxIndex].id = u8MessageID;                              // initialise message to send

				for (u8DataCounter = 0; u8DataCounter < u8DataLength; u8DataCounter++)
				{
					CAN_TxMsg[u8MailBoxIndex].data[u8DataCounter] = msg[u8DataCounter];
				}

				CAN_TxMsg[u8MailBoxIndex].len = u8DataLength;
				CAN_TxMsg[u8MailBoxIndex].format = STANDARD_FORMAT;
				CAN_TxRdy[u8MailBoxIndex] = 0;
				CAN_wrMsg(&(CAN_TxMsg[u8MailBoxIndex]), u8MailBoxIndex);
				break;

			}
		}
		else
		{                  // REMOTE FRAME
			CAN1->sTxMailBox[u8MailBoxIndex].TIR |= (CAN_RTR_REMOTE);
		}
		// Setup length
		CAN1->sTxMailBox[u8MailBoxIndex].TDTR &= ~CAN_TDTxR_DLC;
		CAN1->sTxMailBox[u8MailBoxIndex].TDTR |=  (msg->len & CAN_TDTxR_DLC);

		CAN1->IER |= CAN_IER_TMEIE;                      // enable  TME interrupt                     // enable  TME interrupt
		CAN1->sTxMailBox[u8MailBoxIndex].TIR |=  CAN_TIxR_TXRQ;       // transmit message


	}
	return returnValue;
}


/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it
 ----------------------------------------------------------------------------*/
 //HA to be static and to add a new function to read received data
void CAN_rdMsg (CAN_msg *msg)  {
	// Read identifier information
	if ((CAN1->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0) { // Standard ID

		msg->format = STANDARD_FORMAT;


		msg->id     = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> STANDARD_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_RECEIVE_FIFO_MAIL_BOX_IDENTIFIER_REG);
	}  else  {                                          // Extended ID
		msg->format = EXTENDED_FORMAT;

		msg->id     = (uint32_t)0x0003FFFF & (CAN1->sFIFOMailBox[0].RIR >> EXTENDED_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_RECEIVE_FIFO_MAIL_BOX_IDENTIFIER_REG);
	}
	// Read type information
	if ((CAN1->sFIFOMailBox[0].RIR & CAN_RTR_REMOTE) == 0) {

		msg->type =   DATA_FRAME;                     // DATA   FRAME

	}  else  {
		msg->type = REMOTE_FRAME;                   // REMOTE FRAME
	}
	// Read length (number of received bytes)
	msg->len = (uint8_t)0x0000000F & CAN1->sFIFOMailBox[0].RDTR;
	// Read data bytes
	msg->data[1ST_DATA_BYTE_IN_CAN_RDLR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
	msg->data[2ND_DATA_BYTE_IN_CAN_RDLR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> DATA_BYTE_1_BIT_POSITION_IN_CAN_RDLR_REG);
	msg->data[3RD_DATA_BYTE_IN_CAN_RDLR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> DATA_BYTE_2_BIT_POSITION_IN_CAN_RDLR_REG);
	msg->data[4TH_DATA_BYTE_IN_CAN_RDLR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> DATA_BYTE_3_BIT_POSITION_IN_CAN_RDLR_REG);

	msg->data[1ST_DATA_BYTE_IN_CAN_RDHR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
	msg->data[2ND_DATA_BYTE_IN_CAN_RDHR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> DATA_BYTE_5_BIT_POSITION_IN_CAN_RDHR_REG);
	msg->data[3RD_DATA_BYTE_IN_CAN_RDHR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> DATA_BYTE_6_BIT_POSITION_IN_CAN_RDHR_REG);
	msg->data[4TH_DATA_BYTE_IN_CAN_RDHR_REG] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> DATA_BYTE_7_BIT_POSITION_IN_CAN_RDHR_REG);
	CAN_vidReleaseMessage();

}

void CAN_wrFilter ()  {

	static unsigned short CAN_filterIdx;

	uint32_t   CAN_msgId     = 0;


	CAN1->FMR  |=  CAN_FMR_FINIT;						// set Initialisation mode for filter banks
	for (CAN_filterIdx = 0 ; CAN_filterIdx < NUMBER_OF_CONFIGURED_CAN_FILTERS; CAN_filterIdx++)
	{
		// Enable reception of messages
		if (CAN_filterIdx > LAST_FILTER_ID) {                       // check if Filter Memory is full
			return;
		}
		// Setup identifier information
		if (CAN_filters_Array[CAN_filterIdx].u8Type == STANDARD_FORMAT)  {               // Standard ID
			CAN_msgId  |= (uint32_t)(CAN_filters_Array[CAN_filterIdx].u8Id << STANDARD_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_RECEIVE_FIFO_MAIL_BOX_IDENTIFIER_REG ) | CAN_ID_STD;

		}  else  {                                      // Extended ID
			CAN_msgId  |= (uint32_t)(CAN_filters_Array[CAN_filterIdx].u8Id << EXTENDED_IDENTIFIER_1ST_BIT_POSITION_IN_CAN_RECEIVE_FIFO_MAIL_BOX_IDENTIFIER_REG ) | CAN_ID_EXT;
		}
		if (CAN_filters_Array[CAN_filterIdx].u8Frame == REMOTE_FRAME)  {               // Standard ID
			CAN_msgId  |= CAN_RTR_REMOTE;
		}
		else
		{

		}
		CAN1->FA1R &=  ~(uint32_t)(1 << CAN_filterIdx); // deactivate filter

		// initialize filter
		CAN1->FS1R |= (uint32_t)(1 << CAN_filterIdx);// set 32-bit scale configuration
		CAN1->FM1R |= (uint32_t)(1 << CAN_filterIdx);// set 2 32-bit identifier list mode

		CAN1->sFilterRegister[CAN_filterIdx].FR1 = CAN_msgId; //  32-bit identifier
		CAN1->sFilterRegister[CAN_filterIdx].FR2 = CAN_msgId; //  32-bit identifier

		CAN1->FFA1R &= ~(uint32_t)(1 << CAN_filterIdx);  // assign filter to FIFO 0
		CAN1->FA1R  |=  (uint32_t)(1 << CAN_filterIdx);  // activate filter

	}

	CAN1->FMR &= ~(CAN_FMR_FINIT);						// reset Initialisation mode for filter banks
}

static void CAN_vidReleaseMessage(void)
{
	CAN1->RF0R |= CAN_RF0R_RFOM0;
}
/**********NVIC******/
void CAN_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint8_t ISER_Num=0;
	uint8_t IRQActualNumber=0;

	ISER_Num = IRQNumber / 32;
	IRQActualNumber = IRQNumber % 32;

	switch(EnorDi)
	{
	case 1:
		NVIC_ISER_Base_Addr[ISER_Num] = 1<< IRQActualNumber;
		break;
	case 0:
		NVIC_ICER_Base_Addr[ISER_Num] = 1<< IRQActualNumber;
		break;
	}
}
/*****END****/

/*----------------------------------------------------------------------------
CAN transmit interrupt handler
 *----------------------------------------------------------------------------*/
//HA To add a Tx done callback notification
void CAN1_TX_IRQHandler (void) {

	if (CAN1->TSR & (CAN_TSR_RQCP0)) 						  // request completed mbx 0
	{
		CAN1->TSR |= (CAN_TSR_RQCP0);                    // reset request complete mbx 0
		CAN1->IER &= ~(CAN_IER_TMEIE);                   // disable  TME interrupt
		CAN_TxRdy[0] = 1;
	}
	if (CAN1->TSR & (CAN_TSR_RQCP1)) 						// request completed mbx 1
	{
		CAN1->TSR |= (CAN_TSR_RQCP1);                    // reset request complete mbx 1
		CAN1->IER &= ~(CAN_IER_TMEIE);                   // disable  TME interrupt
		CAN_TxRdy[1] = 1;

	}


	if (CAN1->TSR & (CAN_TSR_RQCP2)) 					 // request completed mbx 2
	{
		CAN1->TSR |= (CAN_TSR_RQCP2);                    // reset request complete mbx 2
		CAN1->IER &= ~(CAN_IER_TMEIE);                   // disable  TME interrupt
		CAN_TxRdy[2] = 1;
	}
}

/*----------------------------------------------------------------------------
  CAN receive interrupt handler
 *----------------------------------------------------------------------------*/
//HA RX Notific to be added.
void CAN1_RX0_IRQHandler (void) {
	uint8_t u8RxMsgIndex = 0;
	if (CAN1->RF0R & CAN__Msg_Pending)
	{			      // message pending ?
		for (u8RxMsgIndex = 0; u8RxMsgIndex<3; u8RxMsgIndex++)
		{
			if (CAN_RxMsg[u8RxMsgIndex].u8ActiveFlag == 0)
			{
				break;
			}
		}

		CAN_rdMsg_0 (&(CAN_RxMsg[u8RxMsgIndex]));                     // read the message
		CAN_RxMsg[u8RxMsgIndex].u8ActiveFlag = 1;
		CAN_RxRdy = 1;                                // set receive flag

	}
}
