#ifndef COORDINATE_MOTOR_H
#define COORDINATE_MOTOR_H


int16_t convert_cm_step(float x);
void motor_set_pos(float pos_r, float pos_l, float vit_r, float vit_l);
void motor_coordinate_start(void);
void set_semaphore_move_and_track(void);
void pause_motor(void);
void resume_motor(void);
void set_robot_pos_x(float);
void set_robot_pos_y(float);
float convert_coord_cm(float);
void wait_move_finished(void);
void set_angle(float angle_input);
#endif /* COORDINATE_MOTOR_H */
