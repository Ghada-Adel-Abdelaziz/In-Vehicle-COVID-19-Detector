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
#define CANHANDLER_u8MAXFILTERNUMBERS		13

#define NUMBER_OF_CAN                       1

typedef struct  {
  uint32_t  id;                 // 29 bit identifier
  uint8_t  data[8];            // Data field
  uint8_t  len;                // Length of data field in bytes
  uint8_t  format;             // 0 - STANDARD, 1- EXTENDED IDENTIFIER
  uint8_t  type;               // 0 - DATA FRAME, 1 - REMOTE FRAME
  uint8_t u8ActiveFlag;			//active flag by sondos
} CAN_msg;


typedef struct
{
	uint8_t u8Id;
	uint8_t u8Frame;
	uint8_t u8Type;
}filter_type;

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1

extern CAN_msg CAN_ConfigArray[ NUMBER_OF_CONFIGURED_CAN  ];

filter_type CAN_FilterArray [ CANHANDLER_u8MAXFILTERNUMBERS ];

#endif /* CANHANDLER_CFG_H_ */