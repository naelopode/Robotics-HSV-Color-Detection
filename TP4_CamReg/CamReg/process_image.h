#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
float get_robot_pos_x(void);
float get_robot_pos_y(void);
void set_robot_pos_x(float x);
void set_robot_pos_y(float y);
float get_pos_x(void);
float get_pos_y(void);
uint16_t get_line_position(void);
void process_image_start(void);
float convert_rgb_cm(uint8_t c);

//static BSEMAPHORE_DECL(color_ready_sem, TRUE);

#endif /* PROCESS_IMAGE_H */
