/******************************************************************************
 * Module: 		I2C
 * File Name: 	I2C.c
 * Description: I2C Source file for
 * 				STM32F407 Microcontroller
 * Author: 		Toqa & Ghada
 * Date:		1/4/2021
 ******************************************************************************/

#include "I2C.h"
#include "Common_Macros.h"
#include "I2C_cfg.h"
#include "I2C_lcfg.h"


#define I2C_ENABLE_BIT_LOC_IN_REG		0
#define I2C_ENABLE_BIT_MASK				0xFFFE

#define I2C_FREQUENCY_BIT_LOC_IN_REG	0
#define I2C_FREQUENCY_BIT_MASK			0xFFE0

#define I2C_FAST_OR_STANDARD_BIT_LOC_IN_REG	   15
#define I2C_FAST_OR_STANDARD_BIT_MASK		   0x7FFF

#define I2C_FM_DUTY_CYCLE_BIT_LOC_IN_REG     14
#define I2C_DUTY_CYCLE_BIT_MASK				 0xBFFF

#define I2C_RISE_TIME_BIT_LOC_I_REG     0
#define I2C_RISE_TIME_BIT_MASK          0xFFC0

#define CLOCK_CONFIG_BIT_LOC_IN_REG		0
#define CLOCK_CONFIG_BIT_MASK			0xF000

#define I2C_ENABLE_BIT_LOC_IN_REG		0
#define I2C_ENABLE_BIT_MASK				0xFFFE

#define I2C_ADDRESS_MODE_BIT_LOC_IN_REG	15
#define I2C_ADDRESS_MODE_BIT_MASK		0x7FFF

#define I2C_ADDRESS_BIT_LOC_IN_REG		0
#define I2C_ADDRESS_BIT_MASK			0xFC00

#define I2C_ACK_BIT_LOC_IN_REG     10
#define I2C_ACK_BIT_MASK           0xFBFF

#define START_BIT_LOC_IN_REG       8
#define START_BIT_MASK			   0xFEFF

#define STOP_BIT_LOC_IN_REG       9
#define STOP_BIT_MASK			  0xFDFF


/* Flags definitions */

#define START_CONDITION_GENERATED_FLAG   0
#define ADDRESS_SENT_FLAG  				 1
#define TX_REG_EMPTY_FLAG  				 7
#define RX_REG_NOT_EMPTY_FLAG			 6
#define BYTE_TRANSFER_FINISHED_FLAG      2

#define NUM_OF_REG_BITS                  32

/* Descriptive Macros For Configurations */
#define STANDARD_MODE_CLK_SPEED_CONFIG               100000
#define I2C_BUS_FREQUENCY                            42
#define THE_MINIMUM_ALLOWED_VALUE_FOR_STANDARD_MODE  0x04
#define THE_MINIMUM_ALLOWED_VALUE_FOR_FAST_DUTY_MODE 0x01
#define I2C_BUS_FREQUENCY_CONVERTED_FROM_MHz_TO_Hz   42000000
#define THE_MAX_RISE_TIME_IN_FAST_SPEED_MOOD         300


#define I2C_TXE_NOT_BUSY                            (uint8_t)0
#define I2C_TXE_BUSY                                (uint8_t)1

#define I2C_RXE_NOT_BUSY                            (uint8_t)0
#define I2C_RXE_BUSY                                (uint8_t)1



static void I2C_PeriClockControl(uint8_t I2C_ID,uint8_t EnorDi);
static void I2C_PeripheralControl(uint8_t I2C_ID, uint8_t Cmd);
static void I2C_AcknoledgeControl(uint8_t I2C_ID, uint8_t Cmd);
static void I2C_SendAddress(uint8_t I2C_ID, uint8_t address,DIRECTION direction);


typedef struct
{
	volatile uint16_t DataSizeCounter;
	uint8_t*   Data;
	volatile uint8_t SlaveAddress;
	volatile uint16_t LocationAddress;
	volatile uint16_t CurrentIndex;
	volatile uint8_t Flag;
}I2C_TxDetails_t;



typedef struct
{
	volatile uint16_t DataSizeCounter;
	uint8_t*   Data;
	volatile uint8_t SlaveAddress;
	volatile uint16_t LocationAddress;
	volatile uint16_t CurrentIndex;
	volatile uint8_t Flag;
}I2C_RxDetails_t;

static void (*Transfer_Complete_Fptr[NUM_OF_I2C])(void);

static I2C_TxDetails_t I2C_IntTxeDetails[NUM_OF_I2C]={0};

static I2C_RxDetails_t I2C_IntRxDetails[NUM_OF_I2C]={0};

I2C_STATUS I2C_Current_Status = IDLE;


I2C_RegDef_t* I2C_Arr[NUM_OF_I2C] = {I2C1, I2C2, I2C3};


static char I2C_GetFlagStatus_SR1(uint8_t I2C_ID , uint32_t FlagName)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[I2C_ID];

	return ((pI2Cx->SR1 & (One_bit_shift << FlagName)) >> FlagName );
}

static char I2C_GetFlagStatus_SR2(uint8_t I2C_ID , uint32_t FlagName)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[I2C_ID];

	return ((pI2Cx->SR2 & (One_bit_shift << FlagName)) >> FlagName );
}

static void I2C_AcknowledgeControl(uint8_t I2C_ID, uint8_t Cmd)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[I2C_ID];

	pI2Cx->CR1 = ( pI2Cx->CR1 & ~(One_bit_shift << I2C_ACK_BIT_LOC_IN_REG) ) | (Cmd << I2C_ACK_BIT_LOC_IN_REG);

}


static void I2C_SendAddress(uint8_t I2C_ID, uint8_t address, DIRECTION direction)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[I2C_ID];

	if( direction == READ )
	{
		pI2Cx->DR = ( address | (0x01) );  // 0x01 for write
	}
	else
	{
		pI2Cx->DR = ( address & ~(0x01) );  // 0x00 for write
	}

	while( !( I2C_GetFlagStatus_SR1(I2C_ID , ADDRESS_SENT_FLAG) ) );

	(void)pI2Cx->SR1;   // must read SR1 then SR2 to clear the ADDRESS_SENT_FLAG
	(void)pI2Cx->SR2;

}



static void I2C_PeripheralControl(uint8_t I2C_ID, uint8_t Cmd)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[I2C_ID];

	pI2Cx->CR1 = ( pI2Cx->CR1 & ~(One_bit_shift << I2C_ENABLE_BIT_LOC_IN_REG) ) | (Cmd << I2C_ENABLE_BIT_LOC_IN_REG);

}

static void I2C_PeriClockControl(uint8_t I2C_ID,uint8_t EnCLK)
{
	I2C_PCLK_EN =(I2C_PCLK_EN & ~(One_bit_shift << I2C_ID+21)) | (EnCLK << I2C_ID+21);
}




void I2C_Init(void)
{

	//Temporary variable
	uint32_t TempReg=0;
	uint8_t counter=0;
	I2C_RegDef_t *pI2Cx;
	uint32_t temp = 0;  //temp register
	uint16_t result = 0;

	for(counter = 0; counter<NUMBER_OF_CONFIGURED_I2C; counter++)
	{
		pI2Cx = I2C_Arr[I2C_ConfigArray[counter].I2C_ID];


		/* Enable Clock of the I2C prepheal */
		I2C_PeriClockControl( I2C_ConfigArray[counter].I2C_ID , ENABLE);

		/* Disable the I2C  */
		I2C_PeripheralControl( I2C_ConfigArray[counter].I2C_ID, DISABLE ); // I2C must be disabled before clock configurations

		temp = 0;

		/* I2C Clock Configuration */
		temp = (I2C_BUS_FREQUENCY << I2C_FREQUENCY_BIT_LOC_IN_REG);   // 42 is the frequency of the bus the I2C connected to
		pI2Cx->CR2 &= I2C_FREQUENCY_BIT_MASK;
		pI2Cx->CR2 |= temp;


		temp = 0;

		/* Speed Configuration */
		if( I2C_ConfigArray[counter].I2C_ClockSpeed <= STANDARD_MODE_CLK_SPEED_CONFIG)    // Standard Mode configuration
		{
			/* Speed Mode Configuration */
			temp = (0 << I2C_FAST_OR_STANDARD_BIT_LOC_IN_REG);
			pI2Cx->CCR &= I2C_FAST_OR_STANDARD_BIT_MASK;  // choose standard mode
			pI2Cx->CCR |= temp;

			temp = 0;

			/* Rise time configuration */
			pI2Cx->TRISE |= I2C_BUS_FREQUENCY + 1;   // back to the data sheet for details
			/* Also back to the standard document the maximum rise time in standard mode is 1000ns
			 *
			 * So ( 1000ns / (1/42MHz) ) + 1 =~ 43
			 * */

			/* Clock Configuration */
			result = ( I2C_BUS_FREQUENCY_CONVERTED_FROM_MHz_TO_Hz / ( I2C_ConfigArray[counter].I2C_ClockSpeed << 1 ) );
		    if (result < THE_MINIMUM_ALLOWED_VALUE_FOR_STANDARD_MODE)
		    {
		      /* Set minimum allowed value */
		      result = THE_MINIMUM_ALLOWED_VALUE_FOR_STANDARD_MODE;    /*The minimum allowed value is 0x04, except in FAST DUTY mode where the minimum
								allowed value is 0x01*/
		    }
			/* back to data sheet page 870 CCR value can be computed as flolowing
			 *
			 * check photo
			 * note that duty cycle in standard mode is 50% so T high = T low
			 * */

			temp = (result << CLOCK_CONFIG_BIT_LOC_IN_REG);
			pI2Cx->CCR &= CLOCK_CONFIG_BIT_MASK;
			pI2Cx->CCR |= temp;

		}
		else    // Fast mode Configuration
		{
			/* Speed Mode Configuration */
			temp = (1 << I2C_FAST_OR_STANDARD_BIT_LOC_IN_REG);
			pI2Cx->CCR &= I2C_FAST_OR_STANDARD_BIT_MASK;  // choose fast mode
			pI2Cx->CCR |= temp;

			temp = 0;

			/* FM duty cycle configuration */
			temp = (I2C_ConfigArray[counter].I2C_FastModeDutyCycle << I2C_FM_DUTY_CYCLE_BIT_LOC_IN_REG);
			pI2Cx->CCR &= I2C_DUTY_CYCLE_BIT_MASK;
			pI2Cx->CCR |= temp;

			temp = 0;

			/* Rise time configuration */

			pI2Cx->TRISE |= (THE_MAX_RISE_TIME_IN_FAST_SPEED_MOOD / (1 / I2C_BUS_FREQUENCY) );
			/* Also back to the standard document the maximum rise time in FastSpeed mode is 300ns
			 *
			 * So ( 300 /(1/42MHz) ) + 1 =~ 13
			 *  */

			/* Clock Configuration */

			if( I2C_ConfigArray[counter].I2C_FastModeDutyCycle == I2C_DutyCycle_2)
			{
				result = ( I2C_BUS_FREQUENCY_CONVERTED_FROM_MHz_TO_Hz / ( I2C_ConfigArray[counter].I2C_ClockSpeed * 3 ) );
			    if (result < THE_MINIMUM_ALLOWED_VALUE_FOR_FAST_DUTY_MODE)
			    {
			      /* Set minimum allowed value */
			      result = THE_MINIMUM_ALLOWED_VALUE_FOR_FAST_DUTY_MODE;    /*The minimum allowed value is 0x04, except in FAST DUTY mode where the minimum
									allowed value is 0x01*/
			    }
				/* back to data sheet page 870 CCR value can be computed as flolowing
				 *
				 * */
				temp = (result << CLOCK_CONFIG_BIT_LOC_IN_REG);
				pI2Cx->CCR &= CLOCK_CONFIG_BIT_MASK;
				pI2Cx->CCR |= temp;
			}
			else
			{
				result = ( I2C_BUS_FREQUENCY_CONVERTED_FROM_MHz_TO_Hz / ( I2C_ConfigArray[counter].I2C_ClockSpeed * 25 ) );
			    if (result < THE_MINIMUM_ALLOWED_VALUE_FOR_FAST_DUTY_MODE)
			    {
			      /* Set minimum allowed value */
			      result = THE_MINIMUM_ALLOWED_VALUE_FOR_FAST_DUTY_MODE;    /*The minimum allowed value is 0x04, except in FAST DUTY mode where the minimum
									allowed value is 0x01*/
			    }
				/* back to data sheet page 870 CCR value can be computed as flolowing
				 *
				 * check photo
				 * */
				temp = (result << CLOCK_CONFIG_BIT_LOC_IN_REG);
				pI2Cx->CCR &= CLOCK_CONFIG_BIT_MASK;
				pI2Cx->CCR |= temp;
			}

		}

		/* Enable the I2C module after Clock Configuration */
		/* Enable the I2C  */
		I2C_PeripheralControl( I2C_ConfigArray[counter].I2C_ID, ENABLE );

		temp = 0;

		/* Address Mode Configuration */
        temp = ( I2C_ConfigArray[counter].I2C_AdressingMode << I2C_ADDRESS_MODE_BIT_LOC_IN_REG);
        pI2Cx->OAR1 &= I2C_ADDRESS_MODE_BIT_MASK;
        pI2Cx->OAR1|= temp;

        temp = 0;

        /* setting I2C Address in case working as slave */
        temp = ( I2C_ConfigArray[counter].I2C_OwnAddress1 << I2C_ADDRESS_BIT_LOC_IN_REG);
        pI2Cx->OAR1 &= I2C_ADDRESS_BIT_MASK;
        pI2Cx->OAR1|= temp;

        temp = 0;

        Transfer_Complete_Fptr[I2C_ConfigArray[counter].I2C_ID] = I2C_ConfigArray[counter].TX_CompleteFunptr;
	}


}


void I2C_IntControl(uint8_t I2C_ID , I2C_INTERRUPT_SOURCE IntSource , uint8_t State)
{
	I2C_RegDef_t *pI2Cx = I2C_Arr[I2C_ID];

	switch(State)
	{
	case ENABLE:
		//Implement the code to enable interrupt for IntSource
		pI2Cx->CR2 |= ( One_bit_shift << IntSource);
		break;
	case DISABLE:
		//Implement the code to disable interrupt for IntSource
		pI2Cx->CR2 &= ~( One_bit_shift << IntSource);
		break;
	}

}



void Send_Start_Bit_Assynchronous(uint8_t I2C_ID)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[I2C_ID];

	uint32_t temp = 0;

	temp = ( One_bit_shift << START_BIT_LOC_IN_REG );
	pI2Cx->CR1 &= START_BIT_MASK;
	pI2Cx->CR1 |= temp;

}

void Send_Stop_Bit_Assynchronous(uint8_t I2C_ID)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[I2C_ID];

	uint32_t temp = 0;

	temp = ( One_bit_shift << STOP_BIT_LOC_IN_REG );
	pI2Cx->CR1 &= STOP_BIT_MASK;
	pI2Cx->CR1 |= temp;

}

////////////////////// Asyncrounous Functions /////////////////////////

I2C_ERROR_STATUS I2C_SendDataAsync(uint8_t I2C_ID , uint8_t* Data , uint16_t DataSize, uint8_t slave_address,uint16_t loc_address)
{
	I2C_ERROR_STATUS returnValue = I2C_E_OK;

	if(I2C_IntTxeDetails[I2C_ID].Flag == I2C_TXE_NOT_BUSY)
	{
		/*set values for tx */
		I2C_IntTxeDetails[I2C_ID].Data = Data;
		I2C_IntTxeDetails[I2C_ID].SlaveAddress = slave_address;
		I2C_IntTxeDetails[I2C_ID].LocationAddress = loc_address;
		I2C_IntTxeDetails[I2C_ID].DataSizeCounter = DataSize;
		I2C_IntTxeDetails[I2C_ID].CurrentIndex = 0;
		I2C_IntTxeDetails[I2C_ID].Flag = I2C_TXE_BUSY;

		I2C_Current_Status = TRANSMIT;

		/*enable interrupt*/
		I2C_IntControl(I2C_ID , I2C_EVENT_INTERRUPT , ENABLE);
		I2C_IntControl(I2C_ID , I2C_BUFFER_INTERRUPT , ENABLE);

		Send_Start_Bit_Assynchronous(I2C_ID);
	}
	else
	{
		returnValue = I2C_E_NOT_OK;
	}

	return returnValue;
}


I2C_ERROR_STATUS I2C_ReceiveDataASync(uint8_t I2C_ID , uint8_t* Data , uint16_t DataSize, uint8_t slave_address,uint16_t loc_address)
{
	I2C_ERROR_STATUS returnValue = I2C_E_OK;

	if(I2C_IntTxeDetails[I2C_ID].Flag == I2C_RXE_NOT_BUSY)
	{
		/*set values for tx */
		I2C_IntRxDetails[I2C_ID].Data = Data;
		I2C_IntRxDetails[I2C_ID].SlaveAddress = slave_address;
		I2C_IntRxDetails[I2C_ID].LocationAddress = loc_address;
		I2C_IntRxDetails[I2C_ID].DataSizeCounter = DataSize;
		I2C_IntRxDetails[I2C_ID].CurrentIndex = 0;
		I2C_IntRxDetails[I2C_ID].Flag = I2C_RXE_BUSY;

		I2C_Current_Status = RECEIVE;

		if( DataSize > 1 )
		{
			I2C_AcknowledgeControl(I2C_ID, ENABLE);     // enable sending ACK after each received byte
		}
		else
		{
			I2C_AcknowledgeControl(I2C_ID, DISABLE);
		}

		/*enable interrupt*/
		I2C_IntControl(I2C_ID , I2C_EVENT_INTERRUPT , ENABLE);
		I2C_IntControl(I2C_ID , I2C_BUFFER_INTERRUPT , ENABLE);

		Send_Start_Bit_Assynchronous(I2C_ID);
	}
	else
	{
		returnValue = I2C_E_NOT_OK;
	}

	return returnValue;
}


void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	uint8_t ISER_Num=0;
	uint8_t IRQActualNumber=0;


	ISER_Num = IRQNumber / NUM_OF_REG_BITS;
	IRQActualNumber = IRQNumber % NUM_OF_REG_BITS;


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

void I2C1_EV_IRQHandler(void)
{
	I2C_RegDef_t *pI2Cx;
	pI2Cx = I2C_Arr[0];
	
	static uint16_t flag_clearing;
	if( I2C_GetFlagStatus_SR1(I2C_1 , START_CONDITION_GENERATED_FLAG) == 1)   // start bit transferred successfully
	{
		flag_clearing = (void)pI2Cx->SR1;    // read SR1 to clear the flag

		if( I2C_Current_Status == TRANSMIT )
		{
			pI2Cx->DR = I2C_IntTxeDetails[0].SlaveAddress;      // send the slave address  R/W bit is 0 mean writing
		}
		else if( I2C_Current_Status == RECEIVE )
		{
			if( receiving_sequence == 0 )
			{
				pI2Cx->DR = ( I2C_IntTxeDetails[0].SlaveAddress | 0x01 );
				receiving_sequence ++;
			}
			else
			{
				pI2Cx->DR = ( I2C_IntTxeDetails[0].SlaveAddress | 0x01 );      // send the slave address R/W bit is 1 mean reading
			}

		}



	}
	else if( I2C_GetFlagStatus_SR1(I2C_1 , ADDRESS_SENT_FLAG) == 1 )
	{
		(void)pI2Cx->SR1;   // must read SR1 then SR2 to clear the ADDRESS_SENT_FLAG
		(void)pI2Cx->SR2;

		if( I2C_Current_Status == TRANSMIT )     // send the first byte
		{
			pI2Cx->DR = I2C_IntTxeDetails[0].LocationAddress;

		}
		else if( I2C_Current_Status == RECEIVE )
		{
			if( receiving_sequence == 1 )
			{
				pI2Cx->DR = I2C_IntTxeDetails[0].LocationAddress;
				receiving_sequence ++;
			}
			else
			{

			}
		}

	}
	else if( I2C_GetFlagStatus_SR1(I2C_1 , TX_REG_EMPTY_FLAG) == 1 )
	{
		if( I2C_Current_Status == RECEIVE )      // this condition execute only once
		{
			Send_Start_Bit_Assynchronous(I2C_1);    // send a repeated start bit
			/* TXE flag is cleared after sending the start bit */
		}
		else
		{
			if(I2C_IntTxeDetails[0].CurrentIndex < I2C_IntTxeDetails[0].DataSizeCounter - 1)
			{
				pI2Cx->DR = I2C_IntTxeDetails[0].Data[I2C_IntTxeDetails[0].CurrentIndex];
				I2C_IntTxeDetails[0].CurrentIndex++;
			}
			else
			{
				Send_Stop_Bit_Assynchronous(I2C_1);   // send stop bit
				I2C_IntTxeDetails[I2C_1].Flag = I2C_TXE_NOT_BUSY;

				I2C_Current_Status = IDLE;
				Transfer_Complete_Fptr[I2C_1]();
			}

		}

	}
	else if( I2C_GetFlagStatus_SR1(I2C_1 , RX_REG_NOT_EMPTY_FLAG) == 1 )
	{
		I2C_IntRxDetails[I2C_1].Data[I2C_IntRxDetails[I2C_1].CurrentIndex] = pI2Cx->DR;


		if(I2C_IntRxDetails[I2C_1].CurrentIndex < I2C_IntRxDetails[I2C_1].DataSizeCounter - 1)
		{
			I2C_IntRxDetails[I2C_1].CurrentIndex ++;
		}
		else
		{
			Send_Stop_Bit_Assynchronous(I2C_1);   // send stop bit
			I2C_IntRxDetails[I2C_1].Flag = I2C_RXE_NOT_BUSY;

			I2C_Current_Status = IDLE;

			receiving_sequence = 0;

		}
	}

}
