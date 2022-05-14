#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <math.h>
#include <motors.h>
#include <arm_math.h>
#include "coordinate_motor.h"
#include <button.h>
#include "capture_color.h"
#include "detect_obj.h"
#include "led_anim.h"

#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]

static float r_cercle = 37.5;

static float robot_x = 0;
static float robot_y = 0;

static float x = 0;
static float y = 0;

static float angle = 0;

bool flag_pause_motor = FALSE;
// semaphore
static BSEMAPHORE_DECL(move_and_track, TRUE);
static BSEMAPHORE_DECL(move_finished, TRUE);

void set_semaphore_move_and_track(){
	chBSemSignal(&move_and_track);
}

int16_t convert_cm_step(float x){  //convert cm into steps
	int16_t x_step;
	x_step = x * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	return x_step;
}

void motor_set_pos(float pos_r, float pos_l, float vit_r, float vit_l){
	// functions in "motors.c" take steps as input
	// a cm -> steps conversion is then needed

	float pos_step_r = convert_cm_step(pos_r);
	float pos_step_l = convert_cm_step(pos_l);
	float vit_step_r = convert_cm_step(vit_r);
	float vit_step_l = convert_cm_step(vit_l);

	// the step counter of the robot stops at a given value wich corresponds to a given position
	if(pos_step_r != 0 && pos_step_l != 0){
		if(vit_step_r > 0 && vit_step_l > 0){
			while(right_motor_get_pos() <= pos_step_r && left_motor_get_pos() <= pos_step_l){
				if (detected_obj()){
					set_led_state(BLINK);
					right_motor_set_speed(0);
					left_motor_set_speed(0);
				} else {
					set_led_state(ALL_ON_COLOR);
					right_motor_set_speed(vit_step_r);
					left_motor_set_speed(vit_step_l);
				}
			}
		} else if(vit_step_r > 0 && vit_step_l < 0){
			left_motor_set_pos(pos_step_l);
			while(right_motor_get_pos() <= pos_step_r && left_motor_get_pos() >= 0){
				if (detected_obj()){
					set_led_state(BLINK);
					right_motor_set_speed(0);
					left_motor_set_speed(0);
				} else {
					set_led_state(ALL_ON_COLOR);
					right_motor_set_speed(vit_step_r);
					left_motor_set_speed(vit_step_l);
				}
			}
		} else if(vit_step_r < 0 && vit_step_l > 0){
			right_motor_set_pos(pos_step_r);
			while(right_motor_get_pos() >= 0 && left_motor_get_pos() <= pos_step_l){
				if (detected_obj()){
					set_led_state(BLINK);
					right_motor_set_speed(0);
					left_motor_set_speed(0);
				} else {
					set_led_state(ALL_ON_COLOR);
					right_motor_set_speed(vit_step_r);
					left_motor_set_speed(vit_step_l);
				}
			}
		}
	}

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

static THD_WORKING_AREA(waMotorCoordinate, 512);
static THD_FUNCTION(MotorCoordinate, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	// + + = forward
	// - - = backward
	// + - = ccw
	// - + = cw
    systime_t time;

    while(1){
    	chBSemWait(&move_and_track);

        time = chVTGetSystemTime();

		float delta_x = abs(robot_x-x);
		float delta_y = abs(robot_y-y);

		if(x == robot_x && y == robot_y){
			right_motor_set_speed(0);
			left_motor_set_speed(0);
    	} else if(x < robot_x && y < robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(delta_x,delta_x,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(delta_y,delta_y,8,8);
			motor_set_pos(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2,-8,8);

			robot_x=x;
			robot_y=y;

		} else if(x > robot_x && y < robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(delta_x,delta_x,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(delta_y,delta_y,8,8);
			motor_set_pos(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2,-8,8);

			robot_x=x;
			robot_y=y;

		} else if(x < robot_x && y > robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(delta_x,delta_x,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(delta_y,delta_y,8,8);

			robot_x=x;
			robot_y=y;

		} else if(x > robot_x && y > robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(delta_x,delta_x,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(delta_y,delta_y,8,8);

			robot_x=x;
			robot_y=y;
		} else {
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}
		chBSemSignal(&move_finished);
		chThdSleepMilliseconds(1000);
	}
}

void motor_coordinate_start(void){ //Motor thread set as low prio as it need to be interrupted sometimes
	chThdCreateStatic(waMotorCoordinate, sizeof(waMotorCoordinate), LOWPRIO, MotorCoordinate, NULL);
}

void pause_motor(void){
	flag_pause_motor=TRUE;
}

void resume_motor(void){
	flag_pause_motor=FALSE;
}

void set_robot_pos_x(float x_input){
	x = convert_coord_cm(x_input);
}

void set_robot_pos_y(float y_input){
	y = convert_coord_cm(y_input);
}

float convert_coord_cm(float coord){  //take x and y value and convert it
	float x = r_cercle*coord/100;
	return x;
}

void wait_move_finished(void){
	chBSemWait(&move_finished);
}

void set_angle(float angle_input){
	angle = angle_input;
}
