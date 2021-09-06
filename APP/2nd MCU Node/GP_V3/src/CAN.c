/*
 * CAN.C
 *
 *  Created on: May 5, 2021
 *      Author: Toqa & Ghada
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

//#define NUM_OF_CAN                           1
#define One_bit_shift                        1

static uint8_t CAN_TxRdy[3] = {0};                      // CAN HW ready to transmit a message
uint8_t  CAN_RxRdy = {0};                      // CAN HW received a message
void (*RX_ptr)(void);

static void CAN_waitReady (void);
static void CAN_vidReleaseMessage(void);

static void CAN_wrFilter (void);



static void CAN_PeriClockControl(uint8_t CAN_ID,uint8_t EnCLK)
{
	TIM_PCLK_EN = (TIM_PCLK_EN & ~(One_bit_shift << ( CAN_ID+25) )) | (EnCLK << ( CAN_ID+25 ) );
}

/*----------------------------------------------------------------------------
  setup CAN interface
 *----------------------------------------------------------------------------*/
void CAN_init(void)  {

	uint8_t counter = 0 ;
	for(counter = 0 ; counter<NUMBER_OF_CONFIGURED_CAN;counter++)
		{

   CAN_PeriClockControl( CAN_1, 1);
	//uint32_t brp =  36000000;			//clock 36Mhz

	CAN1->MCR = (CAN_MCR_INRQ);
	// init mode, enable auto. retransmission                                         // Note: only FIFO 0,
	//transmit mailbox 0 used

	CAN1->MCR |= (1<<CAN_MCR_TXFP); 					//Transmit priority by request order

	CAN1->IER = (CAN_IER_FMPIE0 | CAN_IER_TMEIE);    // FIFO 0 msg pending, Transmit mbx empty (enable interrupts)

	/* Note: this calculations fit for PCLK1 = 36MHz */
//	brp  = (brp / 18) / 500000;                     // baudrate is set to 500k bit/s

	/* set BTR register so that sample point is at about 72% bit time from bit start */
	/* TSEG1 = 12, TSEG2 = 5, SJW = 4 => 1 CAN bit = 18 TQ, sample at 72%    */
//	CAN1->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF));
//	CAN1->BTR |=  ((((4-1) & 0x03) << 24) | (((5-1) & 0x07) << 20) | (((12-1) & 0x0F) << 16) | ((brp-1) & 0x1FF));
//	normal_mode_test
	CAN1->BTR =0x00060004;
	CAN_wrFilter( );

	//CAN1->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);     // set testmode

	//CAN1->BTR |=  (3 & (CAN_BTR_SILM | CAN_BTR_LBKM));

	CAN1->MCR &= ~(CAN_MCR_INRQ);                     // normal operating mode, reset INRQ
//	while (CAN1->MSR & CAN_MCR_INRQ);
	//GPIO_WriteOutputPin(GREEN_LED,1);

	CAN_waitReady ();
	// callback functions initialization
	//TX_ptr[CAN_Arr[counter]] = CAN_filters_Array[counter].TX_CompleteFunptr;
//	RX_ptr[CAN_Arr[counter]] = CAN_filters_Array[counter].RX_CompleteFunptr;
	RX_ptr = CAN_filters_Array[counter].RX_CompleteFunptr;
		}
}

/*----------------------------------------------------------------------------
  check if transmit mailbox is empty
 *----------------------------------------------------------------------------*/
static void CAN_waitReady (void)  {

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
void CAN_wrMsg (CAN_msg *msg)  {

	uint8_t u8MailBoxIndex = 0;
		for (u8MailBoxIndex = 0; u8MailBoxIndex < 3; u8MailBoxIndex++)
	{
		if (CAN_TxRdy[u8MailBoxIndex] == 1)
		{
			break;
		}

		else
		{

		}
	}
	CAN1->IER |=0x02;

	if (u8MailBoxIndex < 3)
	{
		CAN1->sTxMailBox[u8MailBoxIndex].TIR  = (uint32_t)0;      // Reset TIR register
		// Setup identifier information
		if (msg->format == STANDARD_FORMAT)
		{
			CAN1->sTxMailBox[u8MailBoxIndex].TIR |= (uint32_t)(msg->id << 21) | (CAN_ID_STD);
		}
		else
		{    // Extended ID

			CAN1->sTxMailBox[u8MailBoxIndex].TIR |= (uint32_t)(msg->id <<  3) | (CAN_ID_EXT);
		}
		// Setup type information
		if (msg->type == DATA_FRAME)
		{   // DATA FRAME
			CAN1->sTxMailBox[u8MailBoxIndex].TIR |= (CAN_RTR_DATA);
			CAN1->sTxMailBox[u8MailBoxIndex].TDLR = (((uint32_t)msg->data[3] << 24) |
					((uint32_t)msg->data[2] << 16) |
					((uint32_t)msg->data[1] <<  8) |
					((uint32_t)msg->data[0])        );
			CAN1->sTxMailBox[u8MailBoxIndex].TDHR = (((uint32_t)msg->data[7] << 24) |
					((uint32_t)msg->data[6] << 16) |
					((uint32_t)msg->data[5] <<  8) |
					((uint32_t)msg->data[4])        );
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
}


/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it
 ----------------------------------------------------------------------------*/
void CAN_rdMsg (CAN_msg *msg)  {
	// Read identifier information
	if ((CAN1->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0) { // Standard ID

		msg->format = STANDARD_FORMAT;


		msg->id     = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
	}  else  {                                          // Extended ID
		msg->format = EXTENDED_FORMAT;

		msg->id     = (uint32_t)0x0003FFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
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
	msg->data[0] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
	msg->data[1] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
	msg->data[2] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
	msg->data[3] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 24);

	msg->data[4] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
	msg->data[5] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
	msg->data[6] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
	msg->data[7] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 24);
	CAN_vidReleaseMessage();

}

void CAN_wrFilter ()  {

	static unsigned short CAN_filterIdx;


	CAN1->FMR  |=  CAN_FMR_FINIT;			// set Initialisation mode for filter banks
	for (CAN_filterIdx = 0 ; CAN_filterIdx < NUMBER_OF_CONFIGURED_CAN_FILTERS; CAN_filterIdx++)
	{
		// Enable reception of messages
//		if (CAN_filterIdx > 13) {                       // check if Filter Memory is full
//			return;
//		}
//		// Setup identifier information
//		if (CAN_filters_Array[CAN_filterIdx].u8Type == STANDARD_FORMAT)  {               // Standard ID
//			CAN_msgId  |= (uint32_t)(CAN_filters_Array[CAN_filterIdx].u8Id << 21) | CAN_ID_STD;
//
//		}  else  {                                      // Extended ID
//			CAN_msgId  |= (uint32_t)(CAN_filters_Array[CAN_filterIdx].u8Id <<  3) | CAN_ID_EXT;
//		}
//		if (CAN_filters_Array[CAN_filterIdx].u8Type == REMOTE_FRAME)  {               // Standard ID
//			CAN_msgId  |= CAN_RTR_REMOTE;
//		}
//		else
//		{
//
//		}
		CAN1->FA1R &=  ~(uint32_t)(1 << CAN_filterIdx); // deactivate filter

		// initialize filter
		CAN1->FS1R |= (uint32_t)(1 << CAN_filterIdx);// set 32-bit scale configuration
		CAN1->FM1R &= (uint32_t)~(1 << CAN_filterIdx);// set 2 32-bit identifier list mode

		CAN1->sFilterRegister[CAN_filterIdx].FR1 = CAN_filters_Array[CAN_filterIdx].u8Id; //  32-bit identifier
		CAN1->sFilterRegister[CAN_filterIdx].FR2 = 0; //  32-bit identifier

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
	GPIO_WriteOutputPin(BLUE_LED,1);

}

/*----------------------------------------------------------------------------
  CAN receive interrupt handler
 *----------------------------------------------------------------------------*/
void CAN1_RX0_IRQHandler (void) {

	if (CAN1->RF0R & CAN__Msg_Pending)
	{			      // message pending ?

		                     // read the message
		//CAN_RxMsg.u8ActiveFlag = 1;
		//CAN_RxRdy = 1;                                // set receive flag

	//	GPIO_WriteOutputPin(BLUE_LED,1);
		RX_ptr();
		//while(1);


	}
}

//CAN_Get_MSG_STATUS get_MSG(CAN_msg *MSG){
//	static uint8_t Rx_index=0;
//	if(Rx_Counter>0){
//		MSG->id = CAN_RxMsg[Rx_index].id
//		MSG->data = CAN_RxMsg[Rx_index].data
//		MSG->len = CAN_RxMsg[Rx_index].len
//		MSG->format = CAN_RxMsg[Rx_index].format
//		MSG->type = CAN_RxMsg[Rx_index].type
//
//	Rx_index++;
//	Rx_Counter--;
//	if(Rx_index==3){
//				Rx_index=0;
//			}
//	}
//
//}

int CanTest(void)
{
	if((CAN1->MSR & (1 << 11)) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
