#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <math.h>

#include <motors.h>
#include <arm_math.h>
#include "coordinate_motor.h"
#include "process_image.h"

#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]

uint16_t convert_cm_step(float x){  //convert cm into steps
	uint16_t x_step;
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

	chprintf((BaseSequentialStream *)&SD3, "pos_r = %f \n", pos_r);
	chprintf((BaseSequentialStream *)&SD3, "pos_step_r = %f \n", pos_step_r);

	// the step counter of the robot stops at a given value wich corresponds to a given position
	if(pos_step_r != 0 && pos_step_l != 0){
		while(right_motor_get_pos() <= pos_step_r && left_motor_get_pos() <= pos_step_l){
			float pos = right_motor_get_pos();
			chprintf((BaseSequentialStream *)&SD3, "vit = %f \n", vit_step_r);
			right_motor_set_speed(vit_step_r);
			left_motor_set_speed(vit_step_l);
		}
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);

	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void motor_set_pos_2(float pos_r, float pos_l, float vit_r, float vit_l){
	float pos_step_r = convert_cm_step(pos_r);
	float pos_step_l = convert_cm_step(pos_l);
	float vit_step_r = convert_cm_step(vit_r);
	float vit_step_l = convert_cm_step(vit_l);

	if(pos_step_r != 0 && pos_step_l != 0){
		right_motor_set_speed(vit_step_r);
		left_motor_set_speed(vit_step_l);
		chThdSleepMilliseconds(pos_step_r/vit_step_r);
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

//void goto_position(float x,float y){ // take coordinates in cm as input to move the robot

static THD_WORKING_AREA(waMotorCoordinate, 512);
static THD_FUNCTION(MotorCoordinate, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    //chprintf((BaseSequentialStream *)&SD3, "Je suis dans le thread motor \n");
	// + + = forward
	// - - = backward
	// + - = ccw
	// - + = cw
    systime_t time;

    while(1){
        time = chVTGetSystemTime();
		//image_rdy = 1;
		//if(image_rdy == 1){
		//chBSemWait(&color_ready_sem);

		float robot_x = get_robot_pos_x();
		float robot_y = get_robot_pos_y();

		float x = get_pos_x();
		float y = get_pos_y();

		chprintf((BaseSequentialStream *)&SD3, " x= %f \n", x);

		if(robot_x == 0 && robot_y == 0){
			motor_set_pos(y,y,8,8);
			motor_set_pos(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, -8, 8);
			motor_set_pos(x,x,8,8);

			set_robot_pos_x(x);
			set_robot_pos_y(y);

		} else if(x < robot_x && y < robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(robot_y - y,robot_y - y,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(robot_x - x,robot_x - x,8,8);
			motor_set_pos(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2,-8,8);

			set_robot_pos_x(x);
			set_robot_pos_y(y);

		} else if(x > robot_x && y < robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(robot_y - y,robot_y - y,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(x - robot_x,x - robot_x,8,8);

			set_robot_pos_x(x);
			set_robot_pos_y(y);

		} else if(x < robot_x && y > robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(y - robot_y,y - robot_y,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(robot_x - x,robot_x - x,8,8);
			motor_set_pos(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2,-8,8);

			set_robot_pos_x(x);
			set_robot_pos_y(y);

		} else if(x > robot_x && y > robot_y){
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-8,8);
			motor_set_pos(y - robot_y,y - robot_y,8,8);
			motor_set_pos(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,8,-8);
			motor_set_pos(x - robot_x,x - robot_x,8,8);

			set_robot_pos_x(x);
			set_robot_pos_y(y);
		}
	    //image_rdy = 0;
		chprintf((BaseSequentialStream *)&SD3, " robot_x= %f \n", robot_x);

		chThdSleepUntilWindowed(time, time + MS2ST(10000));
		//chThdYield();
		//chThdSleepMilliseconds(1000);
	}
    //}
}

void motor_coordinate_start(void){
	chprintf((BaseSequentialStream *)&SD3, "Je suis dans le thread motor start \n");
	chThdCreateStatic(waMotorCoordinate, sizeof(waMotorCoordinate), NORMALPRIO+1, MotorCoordinate, NULL);
}
