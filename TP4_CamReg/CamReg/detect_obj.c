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
static THD_WORKING_AREA(wadetect_obj, 512);
static THD_FUNCTION(detect_obj, arg) {
	//chprintf((BaseSequentialStream *)&SD3, "step1\n");
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    //chprintf((BaseSequentialStream *)&SD3, "step2\n");
    bool detected = FALSE;
    while(1){
    	if (DETECT_ON){
        	detected = FALSE;
        	for (uint8_t i = 0; i <PROXIMITY_NB_CHANNELS;++i){
        		if(get_prox(i)>THRESHOLD_PROXIMITY){
        			chprintf((BaseSequentialStream *)&SD3, "value of proximity %i\n", get_prox(i));
        			//set_semaphore_pause();
        			set_led_state(BLINK);
        			detected_flag=TRUE;
        			detected = TRUE;
        		}
        	}
        	if (!detected){
        		detected_flag=FALSE;
        		//reset_semaphore_pause();
    	}

   	}
    chThdSleepMilliseconds(100); //switch to windowed tWime ?
    }
}



void detect_obj_start(void){
	proximity_start();
	calibrate_ir();
	chThdCreateStatic(wadetect_obj, sizeof(wadetect_obj), NORMALPRIO+1, detect_obj, NULL);
}

void set_detect_on(void){
	DETECT_ON = TRUE;
}

uint8_t get_detected_flag(void){
	return detected_flag;
}

