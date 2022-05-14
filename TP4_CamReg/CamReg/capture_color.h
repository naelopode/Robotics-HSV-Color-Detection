#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define MAX_RGB_VALUE 			255 //max value of RGB
#define NB_MESURES 				10 //Number of image to average color
#define IMAGE_BUFFER_SIZE		20 //Size of rectangles we want to capture
#include "global.h"

typedef enum { //available color type for output
	RGB,
	HSV,
} color_type;

typedef struct RGB_l{ //RGB as usigned long int as it can reach 252 000
	unsigned long int red;
	unsigned long int green;
	unsigned long int blue;
}color_rgb_long;

typedef struct HSV{ //HSV color
	float hue;
	float saturation;
	float value;
}color_hsv;

void process_image_start(void);
void led_match(color_rgb_n_t);
void RGB2HSV(color_rgb_n_t, struct HSV *output);
float max(float a, float b, float c);
float min(float a, float b, float c);

void print_color(color_rgb_n_t, struct HSV input_HSV, color_type format); //function for debug purposes, need to include chfprint

void set_semaphore_capture(void);
void wait_capture_ready(void);

#endif /* PROCESS_IMAGE_H */
