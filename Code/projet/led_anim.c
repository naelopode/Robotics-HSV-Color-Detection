#include "led_anim.h"
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <leds.h>
#include <spi_comm.h>
#include <math.h>
#include "global.h"

#define RGB_MIN_INTENSITY			0
#define LED_MAX_INTENSITY			1
#define LED_MIN_INTENSITY			0

static LED_STATES_t LED_STATE = NO_LEDS;
static color_rgb_n_t COLOR_LED = {1,1,1};

static THD_WORKING_AREA(waLedAnim, 256);
static THD_FUNCTION(LedAnim, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){
    	switch (LED_STATE){
    		case (NO_LEDS): //Turn all leds off
				clear_leds();
    			break;
    		case (TURN_CW): //Clockwise led animation
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				clear_leds();
    				set_led(i, LED_MAX_INTENSITY);
    				chThdSleepMilliseconds(75);
    				clear_leds();
    				set_rgb_led(i, RGB_MAX_INTENSITY,LED_MIN_INTENSITY,LED_MIN_INTENSITY);
    				chThdSleepMilliseconds(75);
    				clear_leds();
    			}
    			break;
    		case (TURN_CCW): //Counter-Clockwise led animation
    	  		for (uint8_t i = 1; i<=NUM_RGB_LED; ++i){
    	  			clear_leds();
    	    		set_rgb_led(NUM_RGB_LED-i, RGB_MAX_INTENSITY,RGB_MIN_INTENSITY,RGB_MIN_INTENSITY);
    	    		chThdSleepMilliseconds(75);
    	   			clear_leds();
    	   			set_led(NUM_LED-i, LED_MAX_INTENSITY);
    	   			chThdSleepMilliseconds(75);
    	   			clear_leds();
    			}
    			break;
    		case (ALL_ON):
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				set_led(i, LED_MAX_INTENSITY);
       				set_rgb_led(i, RGB_MAX_INTENSITY,RGB_MIN_INTENSITY,RGB_MIN_INTENSITY);
    			}
    			chThdSleepMilliseconds(1000);
    			break;
    		case (ALL_ON_COLOR):

    	   		for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    	   			set_led(i, LED_MIN_INTENSITY);
    	   			set_rgb_led(i,(uint8_t) (COLOR_LED.red*RGB_MAX_INTENSITY),
    	   						  (uint8_t) (COLOR_LED.green*RGB_MAX_INTENSITY),
    	   						  (uint8_t) (COLOR_LED.blue*RGB_MAX_INTENSITY));
    	    	}
    			chThdSleepMilliseconds(500);

				break;
    		case (BLINK):
				clear_leds();
    			chThdSleepMilliseconds(50);
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				set_led(i, LED_MAX_INTENSITY);
    				set_rgb_led(i, RGB_MAX_INTENSITY,RGB_MIN_INTENSITY,RGB_MIN_INTENSITY);
    			}
    			chThdSleepMilliseconds(50);
    			break;
    	}
    }
}

void led_anim_start(void){ //init function
	spi_comm_start();
	clear_leds();
	set_led_state(NO_LEDS);
	chThdCreateStatic(waLedAnim, sizeof(waLedAnim), NORMALPRIO, LedAnim, NULL);
}

void set_led_state (LED_STATES_t INPUT){
	LED_STATE=INPUT;
}

void set_led_color(color_rgb_n_t COLOR_INPUT){
	COLOR_LED = COLOR_INPUT;
}
