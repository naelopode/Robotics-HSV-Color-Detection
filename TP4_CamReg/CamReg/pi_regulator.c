#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <leds.h>
#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <audio/play_melody.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <audio/play_melody.h>
//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 512);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    calibrate_ir();
    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        //applies the speed from the PI regulator and the correction for the rotation
		//right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		//left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
        //100Hz
//        uint16_t proxy[8];
//        for (uint16_t i=0; i<=7; ++i){
//        	proxy[i]=get_prox(i);
//        }
//        bool detected = false;
//    	if (proxy[0]>RANGE){
//    		set_led(LED1,1);
//    		detected = true;
//    		set_rgb_led(LED2,100,100,100);
//    		set_rgb_led(LED4,100,100,100);
//    		set_rgb_led(LED6,100,100,100);
//    		set_rgb_led(LED8,100,100,100);
//    	}
//    	if (proxy[2]>RANGE){
//    		set_led(LED3,1);
//    		detected = true;
//		}
//    	if (proxy[3]>RANGE){
//    		set_led(LED5,1);
//    		detected = true;
//		}
//    	if (proxy[4]>RANGE){
//    		set_led(LED5,1);
//    		detected = true;
//		}
//    	if (proxy[5]>RANGE){
//    		set_led(LED7,1);
//    		detected = true;
//		}
//    	if (!detected){
//    		clear_leds();
//    	}
//        chprintf((BaseSequentialStream *)&SD3, "gyro x %f \r", (float)get_gyro_rate(0));
//        chprintf((BaseSequentialStream *)&SD3, "gyro y %f \r", (float)get_gyro_rate(1));
//        chprintf((BaseSequentialStream *)&SD3, "gyro z %f \r", (float)get_gyro_rate(2));
       // stopCurrentMelody();
     //   playNote(NOTE_C8,1000);
      //  chThdSleepUntilWindowed(time, time + MS2ST(10000));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
