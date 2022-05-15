#ifndef COORDINATE_MOTOR_H
#define COORDINATE_MOTOR_H

void set_semaphore_move_and_track(void);
void motor_coordinate_start(void);
void wait_move_finished(void);
void set_angle(float);
void set_norm(float);
#endif /* COORDINATE_MOTOR_H */
