#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include "capture_color.h"
#include "led_anim.h"
#include "usr_interface.h"
#include "coordinate_motor.h"
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{
	enum FSM{ //States of the FSM
		INIT,
		WAIT_INPUT,
		CAPTURE_COLOR,
		MOVE_AND_TRACK,
	};
	enum FSM current_state; //Current state of our FSM
	current_state = INIT;
    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
	//stars the threads for the pi regulator and the processing of the image
	process_image_start(); //start image but also motor because of the architecture
	motor_coordinate_start();
	led_anim_start();
	usr_interface_start();
	current_state = INIT+1; //jump to next state
    /* Infinite loop. */
    while (1) {
       	switch (current_state){
    		case (WAIT_INPUT):
    			set_led_state(TURN_CW);
    			wait_button_pressed();
      			current_state = CAPTURE_COLOR;
    			break;
    		case (CAPTURE_COLOR):
				set_semaphore_capture();
				wait_capture_ready();
				current_state = MOVE_AND_TRACK;
    			break;
    		case (MOVE_AND_TRACK):
				set_semaphore_move_and_track();
				wait_move_finished();
				current_state=WAIT_INPUT;
    			break;
    	}
       	chThdSleepMilliseconds(400);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
