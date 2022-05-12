#include "usr_interface.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#define WAITING_TIME 20 //number of time
//semaphore
static BSEMAPHORE_DECL(button_pressed, TRUE); //semaphore for when button is pressed

static THD_WORKING_AREA(waUsrInerface, 256);
static THD_FUNCTION(UsrInerface, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
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
    	--wait; //decrement wait
    	if (wait==0){
    		previous_state=FALSE; //reset state if counter is down
    	}
    }
}

void usr_interface_start(void){
	chThdCreateStatic(waUsrInerface, sizeof(waUsrInerface), NORMALPRIO, UsrInerface, NULL);
}

void wait_button_pressed(void) {
	//waits until capture is finished
	chBSemWait(&button_pressed);
}






