/******************************************************************************
 * Module:      UART
 * File Name:   UART_Driver.c
 * Description: Source file for UART Module on STM32F407 Microcontroller
 * Author:      Toqa & Ghada
 * Date:        16/1/2021
 ******************************************************************************/

#include "Common_Macros.h"
#include "UART_Driver.h"
#include "UART_Cfg.h"
#include "UART_Lcfg.h"

/*new*/
#define UART_INT_EN_MASK                             0x0000001F
#define UART_INT_REG_SHIFT                           0x5
#define UART_INT_FLAG_SHIFT                          0x8
#define UART_NO_BYTES_PER_REG                        4
#define UART_INT_PTR_BASE_OFFSET                     4
#define UART_INT_PTR_REG_OFFSET                      3

#define UART_TXE_NOT_BUSY                            (uint8_t)0
#define UART_TXE_BUSY                                (uint8_t)1

/**************************** END*****************************************/
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

#define USART_2_TO_5_APB1ENR_REG_OFFEST              17
#define USART_1_APB2ENR_REG_OFFEST                   4

#define UART_SET_BAUDRATE_FOR_OVER8               (uint8_t)0x07
#define UART_SET_BAUDRATE_FOR_OVER16              (uint8_t)0x0F

/**************************** END*****************************************/

#define DR_2BITS_MASKING_TO_LOAD_9BITS             0x01FF
#define DR_2BITS_MASKING_TO_READ_9BITS             0x01FF
#define TRANSFER_8BITS                             0xFF
#define RECEIVE_8BITS                              0xFF
#define DR_MASKING_TO_READ_7BITS                   0x7F

#define IDLE                                        0
#define TX_IN_PROGRESS                              1
#define RX_IN_PROGRESS                              1


#define USART_2_TO_5_APB1ENR_REG_OFFEST 17
#define USART_1_APB2ENR_REG_OFFEST 4


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




/************************* IT ************************/

typedef struct
{
	volatile uint16_t DataSizeCounter;
	uint8_t*   Data;
	volatile uint16_t CurrentIndex;
	volatile uint8_t Flag;
}Uart_TxDetails_t;


typedef struct
{
	volatile uint8_t InsertIndex;
	volatile uint8_t CurrentIndex;
	uint8_t   Data[UART_RX_BUFFER_SIZE];
	volatile uint8_t CurrentSize;
}Uart_RxDetails_t;


Uart_TxDetails_t Uart_IntTxeDetails[NUMBER_OF_CONFIGURED_UART]={0};

Uart_RxDetails_t Uart_IntRxDetails[NUMBER_OF_CONFIGURED_UART]={0};


/************************   END **********************/

/*
 * USART related status flag definition
 */

#define USART_FLAG_TXE  (USART_SR_TXE)
#define USART_FLAG_TC (1 << USART_SR_TC )
#define USART_FLAG_RXNE (USART_SR_RXNE)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

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
		PCLKx = RCC_GetPCLK2Value();   // return APB2 bus frequency
	}else
	{
		PCLKx = RCC_GetPCLK1Value();   // return APB1 bus frequency
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
	M_part = usartdiv/UART_FRACTION_GET_DIV;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << UART_BAUDRATE_INT_OFFSET;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * UART_FRACTION_GET_MUL));

	//Calculate the final fractional
	if(pUSARTx->USART_CR1 & ( One_bit_shift << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = ((( F_part * UART_BAUDRATE_FRACTION_MUL_FOR_OVER8)+ UART_BAUDRATE_ROUND_VALUE) / UART_FRACTION_GET_DIV)& (UART_SET_BAUDRATE_FOR_OVER8);

	}else
	{
		//over sampling by 16
		F_part = ((( F_part * UART_BAUDRATE_FRACTION_MUL_FOR_OVER16)+ UART_BAUDRATE_ROUND_VALUE) / UART_FRACTION_GET_DIV) & (UART_SET_BAUDRATE_FOR_OVER16);

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
	uint8_t counter = 0 ;
	USART_RegDef_t *pUSARTx;
	pUSARTx = USART_Arr[UART_ConfigArray[counter].USART_ID];

	for(counter; counter<NUMBER_OF_CONFIGURED_UART;counter++)
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

		USART_SetBaudRate(UART_ConfigArray[counter].USART_ID ,UART_ConfigArray[counter].USART_Baud);

	}
}


/**************NEW(26/1/2021)***********/

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

/*************************** New with interrupt **************************/


uint8_t Uart_SendDataAsync(uint8_t Id , uint8_t* Data , uint16_t DataSize)
{
	uint8_t returnValue=UART_E_OK;

	if(Uart_IntTxeDetails[Id].Flag==UART_TXE_NOT_BUSY)
	{
		/*set values for tx */
		Uart_IntTxeDetails[Id].Data = Data;
		Uart_IntTxeDetails[Id].DataSizeCounter = DataSize;
		Uart_IntTxeDetails[Id].CurrentIndex = 0;
		Uart_IntTxeDetails[Id].Flag = UART_TXE_BUSY;

		/*enable interrupt*/
		Uart_IntControl(Id , UART_INT_TXE , ENABLE);

		/* No need to send the first byte here to fire the interrupt as
		 * after enabling the interrupt it will jump automatically to
		 * the ISR as DR is empty (Architecture do this)*/
	}
	else
	{
		returnValue=UART_E_NOT_OK;
	}

	return returnValue;
}




uint8_t Uart_ReceiveDataASync(uint8_t Id , uint8_t* Data, uint16_t DataSize)
{
	uint8_t returnValue = UART_E_OK;
	if(Uart_IntRxDetails[Id].Flag == UART_RXE_NOT_BUSY)
	{
		/*set values for tx */
		Uart_IntRxDetails[Id].Data = Data;
		Uart_IntRxDetails[Id].DataSizeCounter = DataSize;
		Uart_IntRxDetails[Id].CurrentIndex = 0;
		Uart_IntRxDetails[Id].Flag = UART_RXE_BUSY;
	}
	else
	{
		returnValue=UART_E_NOT_OK;
	}

	return returnValue;
}

/*******************************   END   *********************************/


void Uart_IntControl(uint8_t USART_ID , uint8_t IntSource , uint8_t State)
{
	USART_RegDef_t *pUSARTx = USART_Arr[USART_ID];

	switch(State)
	{
	case ENABLE:
		//Implement the code to enable interrupt for IntSource
		pUSARTx->USART_CR1 |= ( One_bit_shift << IntSource);
		break;
	case DISABLE:
		//Implement the code to disable interrupt for IntSource
		pUSARTx->USART_CR1 &= ~( One_bit_shift << IntSource);
		break;
	}

}


void UART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	uint8_t ISER_Num=0;
	uint8_t IRQActualNumber=0;


	ISER_Num = IRQNumber / 32;
	IRQActualNumber = IRQNumber % 32;


	switch(EnorDi)
	{
	case ENABLE:
		NVIC_ISER_Base_Addr[ISER_Num] = 1<< IRQActualNumber;
		break;
	case DISABLE:
		NVIC_ICER_Base_Addr[ISER_Num] = 1<< IRQActualNumber;
		break;
	}
}

/*************************** USART1 IRQ handler *****************************/


void USART2_IRQHandler(void)
{
//	volatile uint8_t Local_SR=0;
//	Local_SR=USART_Arr[USART2_]->USART_SR;

	GPIO_WriteOutputPin((uint8_t)61 , GPIO_PIN_SET);   // check if whether the code jump to ISR or not

	/* Handling RX */
	if ( USART_GetFlagStatus(USART2_,USART_FLAG_RXNE))
	{
		/*receive data in the current empty position*/
		Uart_IntRxDetails[0].Data[Uart_IntRxDetails[0].InsertIndex]=USART_Arr[USART2_]->USART_DR;

		if(Uart_IntRxDetails[0].CurrentSize!=UART_RX_BUFFER_SIZE)
		{
			++Uart_IntRxDetails[0].CurrentSize;
		}
		else
		{
			/*if the current element overwritten then take the next element*/
			Uart_IntRxDetails[0].CurrentIndex=(Uart_IntRxDetails[0].CurrentIndex+1)%(UART_RX_BUFFER_SIZE-1);
		}
		/*update the insert index*/
		Uart_IntRxDetails[0].InsertIndex=(Uart_IntRxDetails[0].InsertIndex+1)%(UART_RX_BUFFER_SIZE-1);
	}

	/* Handling TX */
	else if(USART_GetFlagStatus(USART2_,USART_FLAG_TXE))
		{
			if(Uart_IntTxeDetails[0].CurrentIndex<Uart_IntTxeDetails[0].DataSizeCounter)
			{
				USART_Arr[USART2_]->USART_DR=Uart_IntTxeDetails[0].Data[Uart_IntTxeDetails[0].CurrentIndex++];
			}
			else
			{
				/*Disable Interrupt*/
				Uart_IntControl(USART2_ , UART_INT_TXE , DISABLE);

				/*clear busy flag*/
				Uart_IntTxeDetails[0].Flag = UART_TXE_NOT_BUSY;
			}
		}




}


/****************************************************************************/

