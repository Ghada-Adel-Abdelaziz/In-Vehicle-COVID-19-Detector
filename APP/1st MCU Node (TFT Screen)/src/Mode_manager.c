/*
 * 	Mode_manger.c
 *
 *  Created on: 22 Jul 2021
 *      Author:
 */

#include "Common_Macros.h"
#include "stm32f4xxx.h"

#include "GPIO_lcfg.h"
#include "GPIO.h"
#include "GPIO_cfg.h"

#include "UART_cfg.h"
#include "UART_Lcfg.h"
#include "UART_Driver.h"

#include "timer.h"
#include "timer_Cfg.h"

#include "TFT.h"

#include "Mode_manager.h"
#include "Communication_Manager.h"
#include "stdio.h"


#define Num_Of_Seats      4
#define Num_Of_Signals    4
#define NON_RESERVED_SEAT 0
#define RESERVED_SEAT     1
#define txt_length        50

#define BUTTON_PRESSED    0x0005
MASK_REQUEST_STATE_t Seat_returnValue = Mode_NOT_OK;

static STATE_t state = IDLE_;
static uint8_t Ignition_Signal ;
static uint8_t COVID_detection_Data ;
static uint8_t Mask_detection_Data;
static uint8_t API_Adress_txt[txt_length] = {"12 Murad Street Manial Giza 12513 Egypt"};
static uint8_t Seat[Num_Of_Seats] ;
static uint8_t Seat_[Num_Of_Signals];
static uint8_t Set_temp_var;
static uint8_t TFT_Rx_pressed;
static MASK_REQUEST_STATE_t ReadSeats_State(void);
MASK_REQUEST_STATE_t Mask_returnValue;
/**Flags**/
uint8_t COVID_received;
uint8_t MASK_received = 1;
uint8_t API_received;


extern uint16_t Data;


void TFT_button_pressed(void)
{
	TFT_Rx_pressed = 1;
}

void CAM_SetMaskState (uint8_t MASK_Received_)
{
	MASK_received = 1;

	Mask_detection_Data = MASK_Received_;
}

void Jetson_Set_COVID_State (uint8_t COVID_Received_)
{
	COVID_received = 1;

	COVID_detection_Data = COVID_Received_;
}

void Jetson_Set_API_State (uint8_t *API_Received_ptr)
{

	API_received = 1;//{USART3_Tx,   GPIO_MODE_ALTFN,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP, 7},
	//{USART3_Rx,   GPIO_MODE_ALTFN,  GPIO_SPEED_MEDIUM,  GPIO_NO_PUPD,  GPIO_TYPE_PP, 7},
	uint8_t i = 0;
	do{
		API_Adress_txt[i]= API_Received_ptr[i];
		i++;
	}while(API_Received_ptr[i] != '\0' );

}

void Set_Temp(uint8_t Tempratur_Received_)
{
	Set_temp_var = Tempratur_Received_;

}

void Set_Seat_Sate(uint8_t *Seat_Received_)
{
	int i;
	for( i=0; i<4; i++ )
	{
		Seat_[i] = Seat_Received_[i];
	}
}

void Set_Ignition_Sate(uint8_t Ignition_Received_)
{
	Ignition_Signal = Ignition_Received_;

}


static MASK_REQUEST_STATE_t ReadSeats_State(void)
{

	if ( (Seat_[0] ==0) && (Seat_[1] ==0) && (Seat_[2] ==0) && (Seat_[3] == 0) )// check if all seats is empty
	{
		Seat_returnValue = Mode_NOT_OK;

	}

	else
	{

		Seat_returnValue = Mode_OK;
	}
	return Seat_returnValue;
}

void Mode_Manager(void)
{
	switch( state )
	{
	case Mode_IDLE:
		//		TFT_Request_Writing( REG , 0x03 , 0x0000);
		state = Background;

		break;

	case Background :
		Seat_returnValue = ReadSeats_State(); // return array of 4 seats


		if (  Seat_returnValue == Mode_NOT_OK )
		{
			//	TFT_Request_Writing( REG , 0x03 , 0x0000);

			state = Background;

		}
		else
		{
			TFT_Request_Writing( REG , 0x03 , 0x0001);
			Jetson_Req_COVID_Detection();

			Jetson_Req_Mask_Detection ();

			state = HandlingActions;
		}
		break;

	case HandlingActions :


		Seat_returnValue = ReadSeats_State(); // return array of 4 seats

		if (  Seat_returnValue == Mode_NOT_OK )
		{
			TFT_Request_Writing( REG , 0x03 , 0x0000);

			state = Background;
		}
		else
		{


			//			if (Mask_detection_Data == 0)
			//			{
			//				TFT_Request_Writing( REG , 0x03 , 0x0002);
			MASK_received = 1;
			Mask_detection_Data = 0;
			if( Ignition_Signal == 1)
			{
				CanWriteBuzzer();
			}
			//
			//			}
			//			else
			//		{
			TFT_Request_Writing( REG , 0x03 , 0x0000);

			//		}
			state = HandlingState;

			if( ( Ignition_Signal == 1 ) && (Mask_detection_Data == 0))
			{
				CanWriteBuzzer();
			}

		}


		break;

	case HandlingState :

		Seat_returnValue = ReadSeats_State(); // return array of 4 seats


		if (  Seat_returnValue == Mode_NOT_OK )
		{
			TFT_Request_Writing( REG , 0x03 , 0x0000);

			state = Background;
		}
		else

		{
			if( ( Ignition_Signal == 1 ) && (Mask_detection_Data == 0))
			{
				CanWriteBuzzer();
			}

		}

		if ( ( ( COVID_received == 1 ) && ( COVID_detection_Data == 1 ) ) || ( Set_temp_var > 39) )
		{
			COVID_received =0;

			{
				TFT_Request_Writing( REG , 0x03 , 0x0003);
				Jetson_Req_API_Detection();
				state = HandlingCOVID;
			}

		}
		break;

	case HandlingCOVID:

		Seat_returnValue = ReadSeats_State(); // return array of 4 seats


		if (  Seat_returnValue == Mode_NOT_OK )
		{
			TFT_Request_Writing( REG , 0x03 , 0x0000);

			state = Background;
		}
		else
		{
			API_received = 1;
			if (API_received == 1)
			{
				API_received = 0;

				TFT_Request_Writing( REG , 0x03 , 0x0006); //API Screean

				state = HandlingAdress;


			}
		}
		break;
	case HandlingAdress:

		Seat_returnValue = ReadSeats_State(); // return array of 4 seats


		if (  Seat_returnValue == Mode_NOT_OK )
		{
			TFT_Request_Writing( REG , 0x03 , 0x0000);

			state = Background;
		}
		else

		{
			TFT_Display_Text_Req(API_Adress_txt , 0x0111 , txt_length);
			TFT_Request_Reading();

			state = Waiting_Press;
		}

		break;
	case Waiting_Press:

		Seat_returnValue = ReadSeats_State(); // return array of 4 seats


		if (  Seat_returnValue == Mode_NOT_OK )
		{
			TFT_Request_Writing( REG , 0x03 , 0x0000);

			state = Background;
		}
		else

		{
			if (TFT_Rx_pressed == 1)
			{
				TFT_Rx_pressed =0;

				TFT_Request_Writing( REG , 0x03 , 0x0006);

				state = Background_COVID;


			}
		}
		break;

	case Background_COVID:
		Seat_returnValue = ReadSeats_State(); // return array of 4 seats


		if (  Seat_returnValue == Mode_NOT_OK )
		{
			TFT_Request_Writing( REG , 0x03 , 0x0000);

			state = Background;
		}


	}
}


//		if (Ignition_Signal == 1 && Mask_detection == 0)
//			//		TFT_Request_Writing( REG , 0x03 , 0x0003); //Display screen 3 with mask icon
//			Jetson_Req_Mask_Detection();




