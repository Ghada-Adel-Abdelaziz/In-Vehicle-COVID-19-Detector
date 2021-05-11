/*
 *  CAN_Lcfg.c
 *
 *  Created on: April 23, 2021
 *      Author: Ghada & Toaa
 */

#include <stdio.h>
#include <stdlib.h>
#include "CAN.h"
#include "CAN_cfg.h"
#include "CAN_Lcfg.h"

filter_type CAN_filters_Array [ NUMBER_OF_CONFIGURED_CAN_FILTERS ] =
{

		{60, DATA_FRAME, STANDARD_FORMAT}
};

