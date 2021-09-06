/*
 *  CAN_Lcfg.c
 *
 *  Created on: April 23, 2021
 *      Author: Ghada & Toaa
 */

 #define NUMBER_OF_CONFIGURED_CAN_FILTERS	 3

 #define NUMBER_OF_CONFIGURED_CAN            1

#define Buzzer_ID		0x01	//remote from gui to main (user accept update)
#define Heater_ID		0x02	//remote from main to app (request SW version ID on ECU) , data from app to main (the SW version)
#define seats_ID	    0x03	//remote from app to main (feedback after receiving file)
#define ignition_ID	    0x04  //data from app to main
#define Temp_ID			0x05  //data from app to main


