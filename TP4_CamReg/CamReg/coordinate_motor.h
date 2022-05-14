#ifndef COORDINATE_MOTOR_H
#define COORDINATE_MOTOR_H

typedef enum { //which way the robot should go
	FW, // forward
	CW, // clockwise
	CCW, // counter clockwise
} robot_dir;

int16_t convert_cm_step(float x);
void motor_set_pos(float pos_to_reach, robot_dir way);
void motor_coordinate_start(void);
void set_semaphore_move_and_track(void);
void pause_motor(void);
void resume_motor(void);
void set_robot_pos_x(float);
void set_robot_pos_y(float);
float convert_coord_cm(float);
void wait_move_finished(void);
void set_angle(float angle_input);
void set_norm(float norm_input);
#endif /* COORDINATE_MOTOR_H */
