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


static THD_WORKING_AREA(wadetect_obj, 512);
static THD_FUNCTION(detect_obj, arg) {
	chprintf((BaseSequentialStream *)&SD3, "step1\n");
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    chprintf((BaseSequentialStream *)&SD3, "step2\n");
    bool detected = FALSE;
    while(1){
    	detected = FALSE;
    	for (uint8_t i = 0; i <PROXIMITY_NB_CHANNELS;++i){
    		if(get_prox(i)>THRESHOLD_PROXIMITY){
    			set_semaphore_pause();
    			set_led_state(BLINK);
    			detected = TRUE;
    		}
    	}
    	if (!detected){
    		reset_semaphore_pause();
    	}
    chThdSleepMilliseconds(50); //switch to windowed tWime ?
    }

}


void detect_obj_start(void){

	proximity_start();
	calibrate_ir();

	//chThdCreateStatic(wadetect_obj, sizeof(wadetect_obj), NORMALPRIO, detect_obj, NULL);

}



