/*
 * 	Mode_manger.h
 *
 *  Created on: 22 Jul 2021
 *      Author:
 */

typedef enum
{
	Mode_IDLE,
	Background,
	HandlingActions,
	HandlingState,
	HandlingCOVID,
	HandlingAdress,
	Waiting_Press,
	Background_COVID

}STATE_t;

typedef enum
{

	Mode_NOT_OK,
	Mode_OK,
	Mode_NOT_found
}
MASK_REQUEST_STATE_t;

void CAM_SetMaskState (uint8_t MASK_Received_);

void Jetson_Set_COVID_State (uint8_t COVID_Received_);

void Jetson_Set_API_State (uint8_t *API_Received_);

void Set_Seat_Sate(uint8_t *Seat_Received_);

void Set_Ignition_Sate(uint8_t Ignition_Received_);

void Set_Temp(uint8_t Tempratur_Received_);



void Mode_Manager(void);




