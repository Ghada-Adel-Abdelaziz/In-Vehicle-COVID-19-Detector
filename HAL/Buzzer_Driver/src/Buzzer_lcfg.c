/*
 * HR_sensor_lcdg.c
 *
 *  Created on: May 9, 2021
 *      Author: mas
 */

#include "Buzzer_cfg.h"
#include "Buzzer_lcfg.h"
#include "Buzzer.h"

Buzzer_Config_t Buzzer_ConfigArray[NUMBER_OF_CONFIGURED_BUZZER] =
{
		{ BUZZER1_DIMMING_ID, BUZZER_OFF , DIMMING_MODE}
};
