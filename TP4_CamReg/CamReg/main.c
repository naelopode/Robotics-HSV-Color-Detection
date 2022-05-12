#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <audio/play_melody.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
//#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
//#include <leds.h>
//#include <audio/audio_thread.h>
//#include <audio/play_melody.h>
//#include <spi_comm.h>
#include "capture_color.h"
#include "coordinate_motor.h"
#include "led_anim.h"
#include "usr_interface.h"
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
	enum FSM{
		INIT,
		WAIT_INPUT,
		CAPTURE_COLOR,
		MOVE_AND_TRACK,
	};
	enum FSM current_state;
	current_state = INIT;
    halInit();
    chSysInit();
    mpu_init();
    //initialise the bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();

	//inits the motors
	motors_init();
	//init motor thread
	motor_coordinate_start();
	//init image capture thread
	process_image_start();
	//init led animation thread
	led_anim_start();
	//init button thread
	usr_interface_start();

	current_state = INIT+1; //jump to next state
    /* Infinite loop. */

    while (1) {
    	//current_state = CAPTURE_COLOR;//for debug purposes
       	switch (current_state){
    		case (WAIT_INPUT):
				chprintf((BaseSequentialStream *)&SD3, "STATE: WAIT_INPUT \r");
    			set_led_state(TURN_CW);
    			wait_button_pressed();
      			current_state = CAPTURE_COLOR;
    			//chprintf((BaseSequentialStream *)&SD3, "button pushed \r");
    			break;
    		case (CAPTURE_COLOR):
				//chprintf((BaseSequentialStream *)&SD3, "enter capture color \r");
				chprintf((BaseSequentialStream *)&SD3, "STATE: CAPTURE_COLOR \r");
				set_semaphore_capture();
				wait_capture_ready();
				current_state = MOVE_AND_TRACK;
    			break;
    		case (MOVE_AND_TRACK):
				//chprintf((BaseSequentialStream *)&SD3, "enter move and track \r");
				chprintf((BaseSequentialStream *)&SD3, "STATE: MOVE AND TRACK \r");
				set_semaphore_move_and_track();

//				if (get_detected_flag()){
//					current_state=OBJ_DETECTED;
//				}
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
