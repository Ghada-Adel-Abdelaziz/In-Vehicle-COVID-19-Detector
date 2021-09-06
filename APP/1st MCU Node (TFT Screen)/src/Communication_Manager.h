/*
 * Communication_Manager.h
 *
 *  Created on: Jul 20, 2021
 *      Author: mas
 */

#ifndef COMMUNICATION_MANAGER_H_
#define COMMUNICATION_MANAGER_H_

#define JETSON_PACKET_LENGTH  8

#define EMPTY       0
#define FULL        1

#define BUSY 		1
#define NOT_BUSY    0

#define TRUE	1
#define FALSE   0

typedef enum
{
	E_NOT_OK,
	E_OK
}REQUEST_STATE_t;

typedef enum
{
	IDLE_,
	Wait_RX,
	Handle_TX,
	NOISE_WAIT
}Communication_STATE;


REQUEST_STATE_t Request_Reading(void);

void Jetson_Req_Mask_Detection (void);
void Jetson_Req_COVID_Detection (void);
void Jetson_Req_API_Detection (void);
void Communication_Manage(void);
void Rx_Manage(void);


#endif /* COMMUNICATION_MANAGER_H_ */
//Communication_Manager_TX_Notification
//Communication_Manager_RX_Notification
