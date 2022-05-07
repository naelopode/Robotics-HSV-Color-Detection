#ifndef COORDINATE_MOTOR_H
#define COORDINATE_MOTOR_H

uint16_t convert_cm_step(float x);
void motor_set_pos(float pos_r, float pos_l, float vit_r, float vit_l);
void motor_set_pos_2(float pos_r, float pos_l, float vit_r, float vit_l);
//void goto_position(float x,float y);

void motor_coordinate_start(void);

#endif /* COORDINATE_MOTOR_H */
