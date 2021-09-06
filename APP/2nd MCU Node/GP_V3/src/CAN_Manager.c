
#include <stdint.h>
#include "CAN_cfg.h"
#include "CAN_Lcfg.h"
#include "CAN.h"
#include "UART_Driver.h"
#include "UART_Cfg.h"
#include "UART_Lcfg.h"

void CanWriteBuzzer(void)
{
	uint8_t TXbuffer[8]={0};
	TXbuffer[0] = Buzzer_ID;
	Uart_SendDataAsync(USART4_, TXbuffer, 8);

};
void CanWriteHeater(uint8_t Heater_Reading)
{
uint8_t TXbuffer[8]={0};
TXbuffer[0] = Heater_ID;
TXbuffer[1] = Heater_Reading;
Uart_SendDataAsync(USART4_, TXbuffer, 8);

};
