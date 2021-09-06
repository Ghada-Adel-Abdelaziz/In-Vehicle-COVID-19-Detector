/*
 * TFT.c
 *
 *  Created on: 14 Jul 2021
 *      Author:
 */

#include "TFT.h"
#include "UART_Lcfg.h"
#include "UART_Driver.h"


#define EMPTY 0
#define FULL  1

static uint8_t TFT_Tx_notification = TFT_FALSE;
static uint8_t TFT_Rx_notification = TFT_FALSE;

static uint8_t TFT_Status = TFT_NOT_BUSY;

static uint8_t TFT_Reg_Address;
static uint16_t TFT_Variable_Addrss;
static uint16_t TFT_Data;

static TFT_DATA_TYPE_t tft_data_type;

static TFT_STATE_t TFT_State = TFT_IDLE;

static uint8_t TFT_New_Request = 0;

static uint8_t write_var_arr[8];
static uint8_t write_reg_arr[7];

/**************** Text Data Struct ******************/
static uint16_t TFT_Txt_Length;
static uint8_t write_text_arr[100];   // define a buffer of expexted max num of characters will be displayed
static uint8_t *text_ptr;

/**********************************/


static uint8_t TFT_ReadDataArr[9] = {0};
uint16_t Data;
static TFT_STATE_t TFT_RxState = TFT_IDLE;

static uint8_t TFT_RxBuffer = EMPTY;

typedef enum
{
	NULLl,
	WRITING,
	READING
}TFT_CURRENT_STATE_t;

TFT_CURRENT_STATE_t TFT_current_state = NULLl;


void TFT_Writing_Complete_CallBack(void)
{
	TFT_Tx_notification = TFT_TRUE;
}

void TFT_Reading_Complete_CallBack(void)
{
	TFT_RxBuffer = FULL;
	TFT_Rx_notification = TFT_TRUE;
}

TFT_REQUEST_STATE_t TFT_Request_Writing(TFT_DATA_TYPE_t data_type , uint16_t address , uint16_t data )
{

	TFT_REQUEST_STATE_t return_value = TFT_E_OK;

	if( TFT_Status == TFT_NOT_BUSY )
	{
		switch( data_type )
		{
		case VARIABLE:

			TFT_Variable_Addrss = address;
			TFT_Data = data;
			tft_data_type = VARIABLE;
			break;
		case TEXT:
			TFT_Variable_Addrss = address;
			TFT_Data = data;
			tft_data_type = TEXT;
			break;
		case REG:

			TFT_Reg_Address = (uint8_t)address;
			TFT_Data = data;
			tft_data_type = REG;
			break;

		}

		TFT_New_Request = TFT_TRUE;
		TFT_Status = TFT_BUSY;
		TFT_current_state = WRITING;
	}
	else
	{
		return_value = TFT_E_NOT_OK;
	}

	return return_value;
}

TFT_REQUEST_STATE_t TFT_Request_Reading(void)
{
	Uart_ReceiveDataASync(USART3_,TFT_ReadDataArr,sizeof(TFT_ReadDataArr));
	return TFT_E_OK;
}


/**************************************************************/

void TFT_Display_Text_Req(uint8_t *ptr , uint16_t address , uint8_t text_length)
{

	if( TFT_Status == TFT_NOT_BUSY )
	{
		TFT_Variable_Addrss = address;
		text_ptr = ptr;
		TFT_Txt_Length = text_length;
		tft_data_type = TEXT;


		TFT_New_Request = TFT_TRUE;
		TFT_Status = TFT_BUSY;
		TFT_current_state = WRITING;
	}
	else
	{
	}

}

/**************************************************************/

void TFT_Tx_Manage(void)
{
	unsigned char i;
	switch( TFT_State )
	{
	case TFT_IDLE:

		if( TFT_New_Request == TFT_TRUE )
		{
			TFT_New_Request = TFT_FALSE;
			if (TFT_current_state == WRITING)
			{
				TFT_State = TFT_WRITING;
			}

		}

		break;

	case TFT_WRITING:

		switch( tft_data_type )
		{
		case VARIABLE:

			write_var_arr[0] = TFT_FRAM_HEADER1;
			write_var_arr[1] = TFT_FRAM_HEADER2;
			write_var_arr[2] = 5;   // data length
			write_var_arr[3] = TFT_VAR_WRITE_CMD;    // variable writing command
			write_var_arr[4] = (uint8_t)( TFT_Variable_Addrss >> 8 );
			write_var_arr[5] = (uint8_t)( TFT_Variable_Addrss & 0x00FF );
			write_var_arr[6] = (uint8_t)( TFT_Data >> 8 );
			write_var_arr[7] = (uint8_t)( TFT_Data & 0x00FF );

			Uart_SendDataAsync(USART3_,write_var_arr,sizeof(write_var_arr));


			break;

		case TEXT:

			write_text_arr[0] = TFT_FRAM_HEADER1;
			write_text_arr[1] = TFT_FRAM_HEADER2;
			write_text_arr[2] = (uint8_t)( 3 + TFT_Txt_Length );   // 3( 1 byte for command + 2 byte for address ) + text length
			write_text_arr[3] = TFT_VAR_WRITE_CMD;    // variable writing command
			write_text_arr[4] = (uint8_t)( TFT_Variable_Addrss >> 8 );
			write_text_arr[5] = (uint8_t)( TFT_Variable_Addrss & 0x00FF );

			for( i=0; i<TFT_Txt_Length; i++ )
			{
				write_text_arr[i+6] = text_ptr[i];    // fill the buffer with the text data starting from index 6
			}

			Uart_SendDataAsync( USART3_, write_text_arr, TFT_Txt_Length + 6 );  // 6 is the length of basic data to be sent to TFT

			TFT_Request_Writing( REG , 0x03 , 0x0003);

			break;


		case REG:

			write_reg_arr[0] = TFT_FRAM_HEADER1;
			write_reg_arr[1] = TFT_FRAM_HEADER2;
			write_reg_arr[2] = 4;   // data length
			write_reg_arr[3] = TFT_REG_WRITE_CMD;    // variable writing command
			write_reg_arr[4] = (uint8_t)( TFT_Reg_Address);
			write_reg_arr[5] = (uint8_t)( TFT_Data >> 8 );
			write_reg_arr[6] = (uint8_t)( TFT_Data & 0x00FF );

			Uart_SendDataAsync(USART3_, write_reg_arr, sizeof(write_reg_arr));

		//	TFT_Request_Writing( REG , 0x03 , 0x0003);
			break;
		}

		TFT_State = TFT_Tx_WAITING;
		break;

		case TFT_Tx_WAITING:

			if (TFT_Tx_notification == TFT_TRUE)
			{
				TFT_Tx_notification = TFT_FALSE;
				TFT_Status = TFT_NOT_BUSY;

				TFT_State = TFT_IDLE;
			}

			break;
	}
}

void TFT_Rx_Manage(void)
{
	unsigned char i;
	switch( TFT_RxState )
	{
	case TFT_IDLE:

		if( TFT_RxBuffer != EMPTY )
		{
			if( TFT_ReadDataArr[0] == 0x5A && TFT_ReadDataArr[0] == 0xA5 )
				TFT_RxState = TFT_READING;
		}

		break;

	case TFT_READING:
		TFT_State =TFT_Tx_WAITING;

		Data = ( ( (uint16_t)TFT_ReadDataArr[4] << 8 ) | TFT_ReadDataArr[5] );


		for(i=0; i<9; i++)
		{
			TFT_ReadDataArr[i] = 0;    // clear the buffer
		}
		TFT_RxBuffer = EMPTY;
		TFT_Request_Reading();

		break;

	case TFT_Rx_WAITING:

		if (TFT_Rx_notification == TFT_TRUE)
		{

			TFT_Rx_notification = TFT_FALSE;
			TFT_State = TFT_IDLE;

		}


		break;

	}
}
