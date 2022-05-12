/*
 * led_anim.c
 *
 *  Created on: 10 mai 2022
 *      Author: naeld
 */
#include "led_anim.h"
#include "ch.h"
#include "hal.h"
#include "global.h"
#include <usbcfg.h>
#include <leds.h>
#include <spi_comm.h>

//Useful definitions
#define MAX_LED_INTENSITY 1
#define MIN_LED_INTENSITY 0
#define RGB_MIN_INTENSITY 0

static LED_STATES_t LED_STATE = NO_LEDS;
static color_rgb_n_t COLOR_LED = {MAX_LED_INTENSITY,MAX_LED_INTENSITY,MAX_LED_INTENSITY};

static THD_WORKING_AREA(waLedAnim, 256);
static THD_FUNCTION(LedAnim, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){
    	switch (LED_STATE){
    		case (NO_LEDS): //All leds are off
				clear_leds();
    			break;
    		case (TURN_CW): //Clockwise led animation
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				clear_leds();
    				set_led(i, 1);
    				chThdSleepMilliseconds(75);
    				clear_leds();
    				set_rgb_led(i, RGB_MAX_INTENSITY,RGB_MIN_INTENSITY,RGB_MIN_INTENSITY);
    				chThdSleepMilliseconds(75);
    				clear_leds();
    			}
    			break;
    		case (TURN_CCW): //Counter clockwise led animation
    	  		for (uint8_t i = 1; i<=NUM_RGB_LED; ++i){
    	  			clear_leds();
    	    		set_rgb_led(NUM_RGB_LED-i, RGB_MAX_INTENSITY,RGB_MIN_INTENSITY,RGB_MIN_INTENSITY);
    	    		chThdSleepMilliseconds(75);
    	   			clear_leds();
    	   			set_led(NUM_LED-i, MAX_LED_INTENSITY);
    	   			chThdSleepMilliseconds(75);
    	   			clear_leds();
    			}
    			break;
    		case (ALL_ON): //All leds are ON and RED
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				set_led(i, MAX_LED_INTENSITY);
       				set_rgb_led(i, RGB_MAX_INTENSITY,RGB_MIN_INTENSITY,RGB_MIN_INTENSITY);
    			}
    			chThdSleepMilliseconds(1000);
    			break;
    		case (ALL_ON_COLOR)://All RGB leds are ON and with the latest defined RGB color
    	   		for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    	   			set_led(i, MIN_LED_INTENSITY);
    	   			set_rgb_led(i,(uint8_t) (COLOR_LED.red*RGB_MAX_INTENSITY),
    	   						  (uint8_t) (COLOR_LED.green*RGB_MAX_INTENSITY),
    	   						  (uint8_t) (COLOR_LED.blue*RGB_MAX_INTENSITY));
    	    	}
    			chThdSleepMilliseconds(500);
				break;
    		case (BLINK): //Blink very fast all leds
				clear_leds();
    			chThdSleepMilliseconds(50);
    			for (uint8_t i = 0; i<NUM_RGB_LED; ++i){
    				set_led(i, MAX_LED_INTENSITY);
    				set_rgb_led(i,RGB_MAX_INTENSITY,RGB_MIN_INTENSITY,RGB_MIN_INTENSITY);
    			}
    			chThdSleepMilliseconds(50);
    			break;
    	}
    }
}

void led_anim_start(void){
	//start SPI
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
