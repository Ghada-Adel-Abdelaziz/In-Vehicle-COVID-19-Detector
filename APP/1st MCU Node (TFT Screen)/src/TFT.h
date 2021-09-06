/*
 * TFT.h
 *
 *  Created on: 14 Jul 2021
 *      Author:
 */

#ifndef TFT_H_
#define TFT_H_

#include "stm32f4xxx.h"


#define TFT_FRAM_HEADER1		0x5A
#define TFT_FRAM_HEADER2 		0xA5
#define TFT_REG_READ_CMD		0x81
#define TFT_REG_WRITE_CMD		0x80
#define TFT_VAR_READ_CMD		0x83
#define TFT_VAR_WRITE_CMD		0x82


#define TFT_BUSY 		1
#define TFT_NOT_BUSY    0

#define TFT_TRUE	1
#define TFT_FALSE   0


typedef enum
{
	VARIABLE,
	TEXT,
	REG
}TFT_DATA_TYPE_t;


typedef enum
{
	TFT_E_NOT_OK,
	TFT_E_OK
}TFT_REQUEST_STATE_t;

typedef enum
{
	TFT_IDLE,
	TFT_WRITING,
	TFT_READING,
	TFT_Tx_WAITING,
	TFT_Rx_WAITING
}TFT_STATE_t;



TFT_REQUEST_STATE_t TFT_Request_Writing(TFT_DATA_TYPE_t data_type , uint16_t address , uint16_t data);
void TFT_Display_Text_Req(uint8_t *ptr , uint16_t address , uint8_t text_length);
void TFT_Tx_Manage(void);
void TFT_Rx_Manage(void);

void TFT_Writing_Complete_CallBack(void);
void TFT_Reading_Complete_CallBack(void);
TFT_REQUEST_STATE_t TFT_Request_Reading(void);




#endif /* TFT_H_ */
