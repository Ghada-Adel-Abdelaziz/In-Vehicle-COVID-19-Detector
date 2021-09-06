/*
 * Communication_Manager.c
 *
 *  Created on: Jul 20, 2021
 *      Author: mas
 */
#include "UART_cfg.h"
#include "UART_Lcfg.h"
#include "UART_Driver.h"
#include "GPIO_cfg.h"
#include "GPIO_Lcfg.h"
#include "GPIO.h"

#include "Communication_Manager.h"
#include "Mode_manager.h"
#define RX_BUFFER_LENGTH               12

#define Request_DONE                   1
#define Request_NOT_DONE               0

extern unsigned char RX_Buffer[JETSON_PACKET_LENGTH];

static const uint8_t GPIO_PIN_CONFIG[3] ={Mask_Request,Covid_Request,API_Request};
uint8_t Request_State[3];
uint8_t GPIO_STATE[3];
#define MAX_COUNT 2
uint8_t Data[20]={"HELLOWORLDAA"};

#define Reception_DONE 		1
#define Reception_NOT_DONE		0
#define Transmission_DONE 		1
#define Transmission_NOT_DONE		0
#define LENGTH               12
#define One_Byte               1


void Communication_Manager_RX_Notification(void)
{
	switch (RX_Buffer[0])
	{
	case Camera_ID:
		CAM_SetMaskState (RX_Buffer[1]);
		break;

	case Cough_ID:
		Jetson_Set_COVID_State (RX_Buffer[1]);

		break;
	case API_ID:
		Jetson_Set_API_State (&RX_Buffer[1]);

		break;

	default:
		break;
	}
	Uart_ReceiveDataASync(USART2_, RX_Buffer, JETSON_PACKET_LENGTH);
}

void Communication_Handling(uint8_t GPIO_ID_SIGNAL)
{
	static uint8_t GPIO_Counter[3];
	switch (GPIO_STATE[GPIO_ID_SIGNAL])
	{
	case LOW:
		GPIO_WriteOutputPin(GPIO_PIN_CONFIG[GPIO_ID_SIGNAL],0);
		if( Request_State[GPIO_ID_SIGNAL] == Request_DONE)
		{
			Request_State[GPIO_ID_SIGNAL] =  Request_NOT_DONE;
			GPIO_STATE[GPIO_ID_SIGNAL] = HIGH;
		}
		break;

	case HIGH:
		GPIO_WriteOutputPin(GPIO_PIN_CONFIG[GPIO_ID_SIGNAL],1);


		if (GPIO_Counter[GPIO_ID_SIGNAL] < MAX_COUNT){
			GPIO_Counter[GPIO_ID_SIGNAL]++;
		}
		else
		{
			GPIO_Counter[GPIO_ID_SIGNAL] = 0;
			GPIO_STATE[GPIO_ID_SIGNAL] = LOW;
		}


	break;

	default:
		break;
}
}

void Jetson_Req_Mask_Detection (void)
{
	Request_State[0] = Request_DONE;
}

void Jetson_Req_COVID_Detection (void)
{
	Request_State[1] = Request_DONE;
}

void Jetson_Req_API_Detection (void)
{
	Request_State[2] = Request_DONE;
}

void Communication_Manage(void)
{
	Communication_Handling(0);
	Communication_Handling(1);
	Communication_Handling(2);
}
