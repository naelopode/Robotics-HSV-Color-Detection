/*
 * proximity.c
 *
 *  Created on: 11 mai 2022
 *      Author: naeld
 */

/*
 * led_anim.c
 *
 *  Created on: 10 mai 2022
 *      Author: naeld
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include "detect_obj.h"
#include "sensors/proximity.h"
#include "coordinate_motor.h"
#include "led_anim.h"
//#include <main.h>

#include "global.h"
//semaphore
bool DETECT_ON = FALSE;
bool detected_flag = FALSE;

uint8_t detected_obj(void){
	bool detected = FALSE;
	for (uint8_t i = 0; i <PROXIMITY_NB_CHANNELS;++i){
		if(get_prox(i)>THRESHOLD_PROXIMITY){
			chprintf((BaseSequentialStream *)&SD3, "value of proximity %i\n", get_prox(i));
			//set_semaphore_pause();
			detected = TRUE;
			return TRUE;//detected_flag=TRUE;
		}
	}
	if (!detected){
		return FALSE;
		detected_flag=FALSE;
		//reset_semaphore_pause();
	}
}


void detect_obj_start(void){
	proximity_start();
	calibrate_ir();
}

void set_detect_on(void){
	DETECT_ON = TRUE;
}

uint8_t get_detected_flag(void){
	return detected_flag;
}

