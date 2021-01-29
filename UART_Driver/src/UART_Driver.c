/*
 * stm32f407xx_uartdriver.c
 *
 *  Created on: 17-JAN-2021
 *      Author: TOQA&GHADA
 */

/*
 * Handle structure for USARTx peripheral
 */

//#include "Common_Macros.h"
#include "UART_Driver.h"
#include "UART_Cfg.h"
#include "UART_Lcfg.h"



// descriptive macros for magic numbers
#define One_bit_shift                        1
#define Two_bits_shift                       2
#define Four_bits_shift                      4
#define Eight_Pins_for_GPIOxAFRH_or_AFRL     8
#define Four_Pins_for_SYSCFG_EXTICR          4
#define Four_Pins_for_IPR_reg                4
#define Eight_reg_bits                       8


#define One_bit_mask                         1
#define One_bit_mask_by_HEX                 0x1
#define Two_consecutive_bits_mask_by_HEX    0x3
#define bits_mask_by_HEX                    0xF

/*new*/
#define UART_INT_EN_MASK                             0x0000001F
#define UART_INT_REG_SHIFT                           0x5
#define UART_INT_FLAG_SHIFT                          0x8
#define UART_NO_BYTES_PER_REG                        4
#define UART_INT_PTR_BASE_OFFSET                     4
#define UART_INT_PTR_REG_OFFSET                      3

/*baud rate macros*/
/*the actual values are mul=100 and samples=16 but for the top value not exceed the uint32 limit we substitute those values
 * with the current values*/
#define UART_FRACTION_GET_MUL_SUB                    25
#define UART_NUMBER_OF_SAMPLES_SUB_FOR_OVER8         2
#define UART_NUMBER_OF_SAMPLES_SUB_FOR_OVER16        4
#define UART_FRACTION_GET_MUL                        100
#define UART_FRACTION_GET_DIV                        100
#define UART_BAUDRATE_INT_OFFSET                     4
#define UART_BAUDRATE_FRACTION_MUL_FOR_OVER8		 8
#define UART_BAUDRATE_FRACTION_MUL_FOR_OVER16    	 16
#define UART_BAUDRATE_ROUND_VALUE                    50


#include "UART_Driver.h"
#define USART_2_TO_5_APB1ENR_REG_OFFEST 17
#define USART_1_APB2ENR_REG_OFFEST 4


/*
 * USART related status flag definition
 */

#define USART_FLAG_TXE (1 << USART_SR_TXE)
#define USART_FLAG_TC (1 << USART_SR_TC )
#define USART_FLAG_RXNE (1 << USART_SR_RXNE)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

/*****Variables for Transmission******/
static uint8_t gUSART_ID;
static const uint8_t *Global_pTxData;
static uint32_t Global_Len;
static uint8_t TransmitRequest = 0;
static uint8_t MemState = IDLE;

/*****Variables for Reception******/
static uint8_t gUSART_ID_R;
static uint8_t *Global_pRxData;
static uint8_t ReceivetRequest = 0;
static uint32_t Global_LenR;
static uint8_t MemState_R = IDLE;

USART_RegDef_t *USART_Arr[NUM_OF_UART] = {USART1,USART2,USART3,USART4,USART5,USART6};


static void USART_PeriClockControl(uint8_t USART_ID, uint8_t EnorDi);
static FLAG_STATUS USART_GetFlagStatus(uint8_t USART_ID , uint32_t FlagName);
static void USART_PeripheralControl(uint8_t USART_ID, uint8_t Cmd);


static void USART_PeripheralControl(uint8_t USART_ID, uint8_t Cmd)
{
	/*********NEW_BY_GHADA**************/
	USART_RegDef_t *pUSARTx = USART_Arr[USART_ID] ;

	pUSARTx->USART_CR1 = (pUSARTx->USART_CR1 & ~(One_bit_mask << USART_CR1_UE))
									|(Cmd << USART_CR1_UE);
	/**********************************/

}


static void USART_PeriClockControl(uint8_t USART_ID, uint8_t EnorDi)
{
	switch(USART_ID)
	{
	case USART1_ :
		USART_PCLK_1_6_EN = (USART_PCLK_1_6_EN & ~(One_bit_mask << (USART_ID+USART_1_APB2ENR_REG_OFFEST)))
		|(EnorDi <<(USART_ID+USART_1_APB2ENR_REG_OFFEST));
		break;
	case USART6_:
		USART_PCLK_1_6_EN =(USART_PCLK_1_6_EN & ~(One_bit_mask << USART_ID))
		|(EnorDi <<USART_ID);
		break;
	default:
		USART_PCLK_2_TO_5_EN =(USART_PCLK_2_TO_5_EN & ~(One_bit_mask << (USART_ID+USART_2_TO_5_APB1ENR_REG_OFFEST-1)))
		|(EnorDi << (USART_ID+USART_2_TO_5_APB1ENR_REG_OFFEST-1));
	}
}

/*Get flag status function */

static FLAG_STATUS USART_GetFlagStatus(uint8_t USART_ID , uint32_t FlagName)
{
	/* one shift left with bitNum_FlagName_
	00000000
	00001000
	 */
	USART_RegDef_t *pUSARTx = USART_Arr[USART_ID] ;

	return ((pUSARTx->USART_SR & (One_bit_shift << FlagName)) >> FlagName );

}

void USART_SetBaudRate(uint8_t USART_ID, uint32_t BaudRate)
{
	USART_RegDef_t *pUSARTx = USART_Arr[USART_ID] ;

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->USART_CR1 & (One_bit_shift << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((UART_FRACTION_GET_MUL_SUB * PCLKx) / (UART_NUMBER_OF_SAMPLES_SUB_FOR_OVER8 *BaudRate));
	}else
	{
		//over sampling by 16
		usartdiv = ((UART_FRACTION_GET_MUL_SUB * PCLKx) / (UART_NUMBER_OF_SAMPLES_SUB_FOR_OVER16 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/UART_FRACTION_GET_MUL;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << UART_BAUDRATE_INT_OFFSET;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * UART_FRACTION_GET_MUL));

	//Calculate the final fractional
	if(pUSARTx->USART_CR1 & ( One_bit_shift << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = ((( F_part * UART_BAUDRATE_FRACTION_MUL_FOR_OVER8)+ UART_BAUDRATE_ROUND_VALUE) / UART_FRACTION_GET_DIV)& ((uint8_t)0x07);

	}else
	{
		//over sampling by 16
		F_part = ((( F_part * UART_BAUDRATE_FRACTION_MUL_FOR_OVER16)+ UART_BAUDRATE_ROUND_VALUE) / UART_FRACTION_GET_DIV) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->USART_BRR = tempreg;
}


/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_Init(void)
{

	//Temporary variable
	uint32_t TempReg=0;
	uint8_t counter=0;
	USART_RegDef_t *pUSARTx;


	for(;counter<NUMBER_OF_CONFIGURED_UART;counter++)
	{
		/******************************** Configuration of CR1******************************************/
		//		pUSARTx = USART_Arr[UART_ConfigArray[counter].USART_ID] ;


		//Implement the code to enable the Clock for given USART peripheral
		USART_PeriClockControl(UART_ConfigArray[counter].USART_ID, ENABLE);


		//Enable USART Tx and Rx engines according to the USART_Mode configuration item
		switch(UART_ConfigArray[counter].USART_Mode)
		{
		case USART_MODE_ONLY_RX :
			//Implement the code to enable the Receiver bit field
			TempReg |= (One_bit_shift << USART_CR1_RE);

			break;

		case USART_MODE_ONLY_TX :
			//Implement the code to enable the Transmitter bit field
			TempReg |= ( One_bit_shift << USART_CR1_TE);
			break;

		case USART_MODE_TXRX :
			//Implement the code to enable the both Transmitter and Receiver bit fields
			TempReg |= ( ( One_bit_shift << USART_CR1_TE) | ( One_bit_shift << USART_CR1_RE) );
			break;
		}
		//Implement the code to configure the Word length configuration item
		TempReg |= UART_ConfigArray[counter].USART_WordLength << USART_CR1_M ;


		//Configuration of parity control bit fields
		switch(UART_ConfigArray[counter].USART_ParityControl)
		{
		case USART_PARITY_EN_EVEN:
			//Implement the code to enable the parity control
			//
			TempReg |= ( 1 << USART_CR1_PCE);
			break;

		case USART_PARITY_EN_ODD:
			//Implement the code to enable the parity control
			TempReg |= ( 1 << USART_CR1_PCE);
			//Implement the code to enable ODD parity
			TempReg |= ( 1 << USART_CR1_PS);

			break;
		}

		//Program the CR1 register
		pUSARTx->USART_CR1 = TempReg;


		USART_PeripheralControl(UART_ConfigArray[counter].USART_ID, ENABLE);

		/******************************** Configuration of CR2******************************************/

		TempReg=0;

		//Implement the code to configure the number of stop bits inserted during USART frame transmission
		TempReg |= UART_ConfigArray[counter].USART_NoOfStopBits << USART_CR2_STOP;

		//Program the CR2 register
		pUSARTx->USART_CR2 = TempReg;


		/******************************** Configuration of CR3******************************************/

		TempReg=0;

		//Configuration of USART hardware flow control
		switch (UART_ConfigArray[counter].USART_HWFlowControl)
		{
		case USART_HW_FLOW_CTRL_CTS:
			//Implement the code to enable CTS flow control
			TempReg |= ( 1 << USART_CR3_CTSE);
			break;

		case USART_HW_FLOW_CTRL_RTS:
			//Implement the code to enable RTS flow control
			TempReg |= (1 << USART_CR3_RTSE);
			break;

		case USART_HW_FLOW_CTRL_CTS_RTS:
			//Implement the code to enable both CTS and RTS Flow control
			TempReg |= ( ( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE) );
			break;
		}


		pUSARTx->USART_CR3 = TempReg;

		/******************************** Configuration of BRR(Baudrate register)******************************************/

		//USART_SetBaudRate(UART_ConfigArray[counter].USART_ID ,UART_ConfigArray[counter].USART_Baud);

		USART2->USART_BRR = 0x1117;
	}
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendDataRequest(uint8_t USART_ID , const uint8_t *pTxBuffer, uint32_t Len)
{
	gUSART_ID = USART_ID;
	Global_pTxData = pTxBuffer;
	Global_Len = Len;
	TransmitRequest = 1;
}

Tx_or_Rx_Feedback TransmitDoneFeedback(void)
{
	static USART_RegDef_t *Local_pUSARTx;
	Local_pUSARTx = USART_Arr[gUSART_ID];
	static const uint16_t *pTxBuffer;
	//static Tx_or_Rx_Feedback TC_FlagState = FALSE;
	static uint32_t TX_Counter = 0;

	pTxBuffer = (uint16_t *)Global_pTxData;

	uint32_t i = 0;

	switch(MemState)
	{
	case IDLE :
		if(TransmitRequest == 1)
		{
			TransmitRequest = 0;
			MemState = TX_IN_PROGRESS;
		}
		break;
	case TX_IN_PROGRESS :
		if(  USART_GetFlagStatus(UART_ConfigArray[i].USART_ID,USART_FLAG_TXE) )
		{
			if(UART_ConfigArray[i].USART_WordLength == USART_WORDLEN_9BITS)
			{
				//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
				Local_pUSARTx->USART_DR = (*pTxBuffer & (uint16_t)DR_2BITS_MASKING_TO_LOAD_9BITS);

				//check for USART_ParityControl
				if(UART_ConfigArray[i].USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used in this transfer. so, 9bits of user data will be sent
					//Implement the code to increment pTxBuffer twice
					Global_pTxData++;
					Global_pTxData++;
				}
				else
				{
					//Parity bit is used in this transfer . so , 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					Global_pTxData++;
				}
			}
			else
			{

				//This is 8bit data transfer
				Local_pUSARTx->USART_DR = (*Global_pTxData  & (uint8_t)TRANSFER_8BITS);

				//Implement the code to increment the buffer address
				Global_pTxData++;

			}
			TX_Counter ++;
		}
		break;
	}



	if(TX_Counter >= Global_Len)
	{
		TX_Counter = 0;
		MemState = IDLE;
		//TC_FlagState = TRUE;
		return TRUE;

	}
	return FALSE;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_ReceiveDataRequest(uint8_t USART_ID, const uint8_t *pRxBuffer, uint32_t LenR)
{
	gUSART_ID_R = USART_ID;
	Global_pRxData = pRxBuffer;
	Global_LenR = LenR;
	ReceivetRequest = 1;
}

Tx_or_Rx_Feedback ReceiveDoneFeedback(void)
{
	static USART_RegDef_t *Local_pUSARTx;
	Local_pUSARTx = USART_Arr[gUSART_ID_R];
	const uint16_t *pRxBuffer;
	static uint32_t RX_Counter = 0;
	pRxBuffer = (uint16_t *)Global_pRxData;


	uint32_t i = 0;

	switch(MemState_R)
	{
	case IDLE :
		if(ReceivetRequest == 1)
		{
			ReceivetRequest = 0;
			MemState_R = RX_IN_PROGRESS;
		}
		break;
	case RX_IN_PROGRESS :

		if ( USART_GetFlagStatus(UART_ConfigArray[i].USART_ID,USART_FLAG_RXNE))
		{
			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(UART_ConfigArray[i].USART_WordLength == USART_WORDLEN_9BITS)
			{
				//We are going to receive 9bit data in a frame

				//check are we using USART_ParityControl control or not
				if(UART_ConfigArray[i].USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used. so, all 9bits will be of user data

					//read only first 9 bits. so, mask the DR with 0x01FF

					*((uint16_t*) Global_pRxData) = (Local_pUSARTx->USART_DR  & (uint16_t)DR_2BITS_MASKING_TO_READ_9BITS);

					//Now increment the pRxBuffer two times
					Global_pRxData++;
					Global_pRxData++;
				}
				else
				{
					//Parity is used, so, 8bits will be of user data and 1 bit is parity
					*Global_pRxData = (Local_pUSARTx->USART_DR  & (uint8_t)RECEIVE_8BITS);

					//Increment the pRxBuffer
					Global_pRxData++;
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				//check are we using USART_ParityControl control or not
				if(UART_ConfigArray[i].USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data

					//read 8 bits from DR
					*Global_pRxData = Local_pUSARTx->USART_DR;
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					*Global_pRxData = (Local_pUSARTx->USART_DR & (uint8_t) DR_MASKING_TO_READ_7BITS);

				}

				//increment the pRxBuffer
				Global_pRxData++;
			}
			RX_Counter ++;
		}
		break;
	}
	if(RX_Counter >= Global_LenR)
	{
		RX_Counter = 0;
		MemState_R = IDLE;
		return TRUE;

	}
	return FALSE;

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t USART_SendDataIT(uint8_t USART_ID, uint8_t *pTxBuffer, uint32_t Len)
{
	USART_RegDef_t *pUSARTx = USART_Arr[USART_ID];
	uint8_t TxBusyState;
	uint8_t txstate = TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		uint32_t TxLen = Len;
		uint8_t pTxBuffer = pTxBuffer;
		uint8_t TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTx->USART_CR1 |= ( One_bit_shift << USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTx->USART_CR1 |= ( One_bit_shift << USART_CR1_TCIE);


	}

	return txstate;
}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t USART_ReceiveDataIT(uint8_t USART_ID,uint8_t *pRxBuffer, uint32_t Len)
{
	USART_RegDef_t *pUSARTx = USART_Arr[USART_ID];
	uint8_t RxBusyState;
	uint8_t rxstate = RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		uint32_t RxLen = Len;
		uint8_t pRxBuffer = pRxBuffer;
		uint8_t RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTx->USART_DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTx->USART_CR1 |= ( One_bit_shift << USART_CR1_RXNEIE);

	}

	return rxstate;
}



