#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
float convert_rgb_cm(uint8_t c);
float convert_cm_step(float x);
void motor_set_pos(float pos_r, float pos_l, float vit_r, float vit_l);
void goto_position(float x,float y);

#endif /* PROCESS_IMAGE_H */
