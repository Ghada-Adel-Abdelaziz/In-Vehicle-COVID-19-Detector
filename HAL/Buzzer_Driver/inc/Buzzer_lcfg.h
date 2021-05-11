/*
 * HR_sensor_lcdg.h
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */
#include "Buzzer_cfg.h"

#include "timer_lcfg.h"
#include "Buzzer.h"





typedef struct
{
	uint8_t buzzer_id;
	uint8_t buzzer_state;
	uint8_t buzzer_mode;
} Buzzer_Config_t;



#define BUZZER1_DIMMING_ID   CH1_ID
#define BUZZER2_DIMMING_ID   CH2_ID
#define BUZZER3_DIMMING_ID   CH3_ID
#define BUZZER4_DIMMING_ID   CH4_ID



#define BUZZER1_NORMAL_ID   GPIO_CHANNEL_B0
#define BUZZER2_NORMAL_ID   GPIO_CHANNEL_B1
#define BUZZER3_NORMAL_ID   GPIO_CHANNEL_B2
#define BUZZER4_NORMAL_ID   GPIO_CHANNEL_B3





#define BUZZER_ON 		0
#define BUZZER_OFF 		1

#define NORMAL_MODE		0
#define DIMMING_MODE	1





extern Buzzer_Config_t Buzzer_ConfigArray[1];
