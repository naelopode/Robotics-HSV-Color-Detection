#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define POSITION_NOT_REACHED 0
#define POSITION_REACHED 1

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
float convert_rgb_cm(uint8_t c);
void goto_position(float x,float y);

#endif /* PROCESS_IMAGE_H */
