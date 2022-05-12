/*
 * usr_interface.c
 *
 *  Created on: 11 mai 2022
 *      Author: naeld
 */
#include "usr_interface.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#define WAITING_TIME 20
//semaphore
static BSEMAPHORE_DECL(button_pressed, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */


static THD_WORKING_AREA(waUsrInerface, 256);
static THD_FUNCTION(UsrInerface, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
   // static LED_STATES_t LED_STATE;
    bool previous_state = FALSE; //bool to prevent double signaling
    uint8_t wait = 0;
    while(1){
    	if (button_get_state()){
    		if (!previous_state){
    			chBSemSignal(&button_pressed);
    			previous_state = TRUE;
    			wait = WAITING_TIME;
    		}
    	}
    	chThdSleepMilliseconds(100); //check every 0.1s
    	--wait;
    	if (wait==0){
    		previous_state=FALSE;
    	}
    }
}

void usr_interface_start(void){
	//static LED_STATES_t LED_STATE;
	chThdCreateStatic(waUsrInerface, sizeof(waUsrInerface), NORMALPRIO, UsrInerface, NULL);
}

void wait_button_pressed(void) {
	//waits until capture is finished
	chBSemWait(&button_pressed);
	chprintf((BaseSequentialStream *)&SD3, "BUTTON_PRESSED \r");
	//chBSemResetI(&button_pressed, TRUE);
}






