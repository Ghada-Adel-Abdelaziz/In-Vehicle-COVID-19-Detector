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

typedef struct
{
	uint32_t TIR;
	uint32_t TDTR;
	uint32_t TDLR;
	uint32_t TDHR;
} CAN_TxMailBox_TypeDef;


typedef struct
{
	uint32_t RIR;
	uint32_t RDTR;
	uint32_t RDLR;
	uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;


typedef struct
{
	uint32_t FR1;
	uint32_t FR2;
} CAN_FilterRegister_TypeDef;

typedef struct
{
	uint32_t MCR;
	uint32_t MSR;
	uint32_t TSR;
	uint32_t RF0R;
	uint32_t RF1R;
	uint32_t IER;
	uint32_t ESR;
	uint32_t BTR;
	uint32_t  RESERVED0[88];
	CAN_TxMailBox_TypeDef sTxMailBox[3];
	CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
	uint32_t  RESERVED1[12];
	uint32_t FMR;
	uint32_t FM1R;
	uint32_t  RESERVED2;
	uint32_t FS1R;
	uint32_t  RESERVED3;
	uint32_t FFA1R;
	uint32_t  RESERVED4;
	uint32_t FA1R;
	uint32_t  RESERVED5[8];
#ifndef STM32F10X_CL
	CAN_FilterRegister_TypeDef sFilterRegister[14];
#else
	CAN_FilterRegister_TypeDef sFilterRegister[28];
#endif /* STM32F10X_CL */
} CAN_TypeDef;


// @CAN IDs
#define CAN_1   0
#define CAN_2   1


#define CAN1                ((CAN_TypeDef *) 0x40006400)
#define Reset_Can_Remap		0xFFFF9FFF
#define Set_Can_Remap		0x00004000

/* register CAN_MCR ----------------------------------------------------------*/
#define CAN_MCR_INRQ          ((unsigned long)0x00000001)
#define CAN_MCR_NART          ((unsigned long)0x00000010)
#define CAN_MCR_TXFP		  ((unsigned long)0x00000002)
/* register CAN_FMR ----------------------------------------------------------*/
#define CAN_FMR_FINIT         ((unsigned long)0x00000001)

/* register CAN_TSR ----------------------------------------------------------*/
#define CAN_TSR_RQCP0         ((unsigned long)0x00000001)
#define CAN_TSR_RQCP1         ((unsigned long)0x00000100)
#define CAN_TSR_RQCP2         ((unsigned long)0x00010000)
#define CAN_TSR_TME0          ((unsigned long)0x04000000)
#define CAN_TSR_TME1          ((unsigned long)0x08000000)
#define CAN_TSR_TME2          ((unsigned long)0x10000000)

/* register CAN_RF0R ---------------------------------------------------------*/
#define CAN_RF0R_FMP0         ((unsigned long)0x00000003)
#define CAN_RF0R_RFOM0        ((unsigned long)0x00000020)

/* register CAN_IER ----------------------------------------------------------*/
#define CAN_IER_TMEIE         ((unsigned long)0x00000001)
#define CAN_IER_FMPIE0        ((unsigned long)0x00000002)

/* register CAN_BTR ----------------------------------------------------------*/
#define CAN_BTR_SILM          ((unsigned long)0x80000000)
#define CAN_BTR_LBKM          ((unsigned long)0x40000000)

/* register CAN_TIxR ---------------------------------------------------------*/
#define CAN_TIxR_TXRQ       ((unsigned long)0x00000001)
#define CAN_ID_STD			((unsigned long)0x00000000)
#define CAN_ID_EXT			((unsigned long)0x00000004)
#define CAN_RTR_DATA		((unsigned long)0x00000000)
#define CAN_RTR_REMOTE		((unsigned long)0x00000002)
/* register CAN_TDTxR --------------------------------------------------------*/
#define CAN_TDTxR_DLC         ((unsigned long)0x0000000F)

#define CAN__Msg_Pending	((unsigned long)0x00000003)


extern CAN_msg       CAN_RxMsg[3];                          // CAN message for receiving

extern filter_type CAN_filters_Array [ NUMBER_OF_CONFIGURED_CAN_FILTERS ];

#endif /* CANHANDLER_CFG_H_ */
