/*
 *  CAN_Lcfg.c
 *
 *  Created on: April 23, 2021
 *      Author: Ghada & Toaa
 */

#ifndef CANHANDLER_CFG_H_
#define CANHANDLER_CFG_H_

/* Description:	Maximum Number of Filters ion filters array					*/
/* Range:		1 ~ 13														*/
#include "stm32f4xxx.h"
#define CANHANDLER_u8MAXFILTERNUMBERS		13

#define NUMBER_OF_CAN                       1

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1

typedef struct
{
	uint8_t u8Id;
	uint8_t u8Frame;
	uint8_t u8Type;
}filter_type;


typedef struct  {
	void (*TX_CompleteFunptr)(void);
	void (*RX_CompleteFunptr)(void);
} CAN_config;



extern filter_type CAN_filters_Array [ NUMBER_OF_CONFIGURED_CAN_FILTERS ];

#endif /* CANHANDLER_CFG_H_ */
