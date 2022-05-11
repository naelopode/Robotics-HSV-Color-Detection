/*
 * led_anim.c
 *
 *  Created on: 10 mai 2022
 *      Author: naeld
 */
#include "led_anim.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>
#include <spi_comm.h>
//#include <main.h>
#include <math.h>
#include "global.h"
//semaphore

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
LED_STATES_t LED_STATE = NO_LEDS;
color_rgb_n_t COLOR_LED = {1,1,1};

static THD_WORKING_AREA(waLedAnim, 256);
static THD_FUNCTION(LedAnim, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
   // static LED_STATES_t LED_STATE;
    while(1){
    	switch (LED_STATE){
    		case (NO_LEDS):
				clear_leds();
    			break;
    		case (TURN_CW):
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				clear_leds();
    				set_led(i, 1);
    				chThdSleepMilliseconds(200);
    				clear_leds();
    				set_rgb_led(i, RGB_MAX_INTENSITY,0,0);
    				chThdSleepMilliseconds(200);
    				clear_leds();
    			}
    			break;
    		case (TURN_CCW):
    	  		for (uint8_t i = 1; i<=NUM_RGB_LED; ++i){
    	  			clear_leds();
    	    		set_rgb_led(NUM_RGB_LED-i, RGB_MAX_INTENSITY,0,0);
    	    		chThdSleepMilliseconds(200);
    	   			clear_leds();
    	   			set_led(NUM_LED-i, 1);
    	   			chThdSleepMilliseconds(200);
    	   			clear_leds();
    			}
    			break;
    		case (ALL_ON):
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				set_led(i, 1);
       				set_rgb_led(i, RGB_MAX_INTENSITY,0,0);
    			}
    			chThdSleepMilliseconds(1000);
    			break;
    		case (ALL_ON_COLOR):
    	   		for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    	   			set_rgb_led(i,(uint8_t) (COLOR_LED.red*RGB_MAX_INTENSITY),
    	   						  (uint8_t) (COLOR_LED.green*RGB_MAX_INTENSITY),
    	   						  (uint8_t) (COLOR_LED.blue*RGB_MAX_INTENSITY));
    	    	}
    			chThdSleepMilliseconds(500);
				break;
    		case (BLINK):
				clear_leds();
    			chThdSleepMilliseconds(200);
    	    	for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    	    		set_led(i, 1);
       				set_rgb_led(i, RGB_MAX_INTENSITY,0,0);
    			}
    			break;
    	}
    }
}

void led_anim_start(void){
	//static LED_STATES_t LED_STATE;
	clear_leds();
	set_led_state(NO_LEDS);
	chThdCreateStatic(waLedAnim, sizeof(waLedAnim), NORMALPRIO-1, LedAnim, NULL);

}

void set_led_state (LED_STATES_t INPUT){
	LED_STATE=INPUT;
}

void set_led_color(color_rgb_n_t COLOR_INPUT){
	COLOR_LED = COLOR_INPUT;
}
