/*
 ******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.3.0   2020-12-30

The MIT License (MIT)
Copyright (c) 2019 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 ******************************************************************************
 */

/* Includes */
#include "GPIO_cfg.h"
#include "stm32f4xxx.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */


//#include "stm32f4xx.h"


#include "GPIO_lcfg.h"
#include "GPIO.h"

/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */
int main(void)
{
	int i = 0;
	GPIO_Init();
	while (1)
	{
		GPIO_WriteOutputPin(LED0 , GPIO_PIN_SET);
		GPIO_WriteOutputPin(LED1 , GPIO_PIN_SET);
		GPIO_WriteOutputPin(LED2 , GPIO_PIN_SET);
		GPIO_WriteOutputPin(LED3 , GPIO_PIN_SET);

		for(i=0; i<4000000; i++);

		GPIO_WriteOutputPin(LED0 , GPIO_PIN_RESET);
		GPIO_WriteOutputPin(LED1 , GPIO_PIN_RESET);
		GPIO_WriteOutputPin(LED2 , GPIO_PIN_RESET);
		GPIO_WriteOutputPin(LED3 , GPIO_PIN_RESET);

		for(i=0; i<4000000; i++);

	}
}
