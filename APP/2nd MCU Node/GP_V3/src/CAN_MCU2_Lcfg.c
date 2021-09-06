/*
 *  CAN_Lcfg.c
 *
 *  Created on: April 23, 2021
 *      Author: Ghada & Toaa
 */

#include <stdio.h>
#include <stdlib.h>
#include "CAN.h"
#include "CAN_cfg.h"
#include "CAN_Lcfg.h"
#include "timer.h"
#include "timer_Lcfg.h"
#include "Mode_manager.h"
#include "UART_Driver.h"
#include "UART_Cfg.h"
#include "UART_Lcfg.h"

uint8_t RXbuffer[STM_packet_size];

static CAN_msg       CAN_RxMsg;                          // CAN message for receiving

uint8_t Seat_reading[8];
void CAN_TX_Callback(void)
{
	//GPIO_WriteOutputPin(BLUE_LED,1);

	GPIO_WriteOutputPin(GREEN_LED,1);
}
void CAN_RX_Callback(void)
{
	int i;
	uint8_t first_byte;
	//GPIO_WriteOutputPin(BLUE_LED,1);
	switch (RXbuffer[0])
	{
	case seats_ID:
		for (i = 0; i < 4; i++)
			Seat_reading[i] = RXbuffer[i+1];
		for (i = 4; i < 8; i++)
		{
		Seat_reading[i] = 0;

		}
		Set_Seat_Sate(Seat_reading);

		break;

	case ignition_ID:
		first_byte = RXbuffer[1];
		Set_Ignition_Sate(first_byte);
		break;
	case Temp_ID:
		first_byte = RXbuffer[1];
		Set_Temp(first_byte);
		break;

	default:
		break;
	}
	Uart_ReceiveDataASync(USART4_, RXbuffer, STM_packet_size);

	//GPIO_WriteOutputPin(BLUE_LED,1);

}
filter_type CAN_filters_Array [ NUMBER_OF_CONFIGURED_CAN_FILTERS ] =
{

		//{60, DATA_FRAME, STANDARD_FORMAT,CAN_Callback,CAN_RX_Callback},
		//{30, DATA_FRAME, STANDARD_FORMAT,CAN_Callback,CAN_RX_Callback},
		{seats_ID, DATA_FRAME, STANDARD_FORMAT,CAN_TX_Callback,CAN_RX_Callback},
		{ignition_ID, DATA_FRAME, STANDARD_FORMAT,CAN_TX_Callback,CAN_RX_Callback},
		{Temp_ID, DATA_FRAME, STANDARD_FORMAT,CAN_TX_Callback,CAN_RX_Callback},

};
