#include "usr_interface.h"
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include "button.c"
#define WAITING_TIME 		20

static BSEMAPHORE_DECL(button_pressed, TRUE); //semaphore for when button is pressed


static THD_WORKING_AREA(waUsrInerface, 256);
static THD_FUNCTION(UsrInerface, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    bool previous_state = FALSE; //bool to prevent double signaling and eventual bounce-back
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

void usr_interface_start(void){ //init function
	chThdCreateStatic(waUsrInerface, sizeof(waUsrInerface), NORMALPRIO, UsrInerface, NULL);
}

void wait_button_pressed(void) {
	//waits until capture is finished
	chBSemWait(&button_pressed);
}






