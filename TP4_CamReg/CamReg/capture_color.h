#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define MAX_RGB_VALUE 255 //max value of RGB
#define NB_MESURES 10
#include "global.h"

typedef enum { //available color type for output
	RGB,
	HSV,
} color_type;

typedef struct RGB_l{ //RGB as usigned long int as it can reach 256�000
	unsigned long int red;
	unsigned long int green;
	unsigned long int blue;
}color_rgb_long;

//typedef struct RGB_n{ //normalised RGB
//	float red;
//	float green;
//	float blue;
//}color_rgb_n;

typedef struct HSV{ //HSV color
	float hue;
	float saturation;
	float value;
}color_hsv;

//void get_color(void);
//float get_robot_pos_x(void);
//float get_robot_pos_y(void);
//
//float get_pos_x(void);
//float get_pos_y(void);
void process_image_start(void);
void led_match(color_rgb_n_t);
void RGB2HSV(color_rgb_n_t, struct HSV *output);
float max(float a, float b, float c);
float min(float a, float b, float c);
void print_color(color_rgb_n_t, struct HSV input_HSV, color_type format);

void set_semaphore_capture(void);
int get_done_capture(void);
void wait_capture_ready(void);

#endif /* PROCESS_IMAGE_H */
