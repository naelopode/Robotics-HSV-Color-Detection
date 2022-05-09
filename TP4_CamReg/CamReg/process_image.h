#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define MAX_RGB_VALUE 255 //max value of RGB
#define NB_MESURES 10

typedef enum { //available color type for output
	RGB,
	HSV,
} color_type;

typedef struct RGB_l{ //RGB as usigned long int as it can reach 256�000
	unsigned long int red;
	unsigned long int green;
	unsigned long int blue;
}color_rgb_long;

//typedef struct RGB{ //RGB with uint8_t to save space
//	uint8_t red;
//	uint8_t green;
//	uint8_t blue;
//}color_rgb;

//typedef struct RGB_i{ //RGB with int because uint8_t or uint16_t is not enough
//	int red;
//	int green;
//	int blue;
//}color_rbg_i;

typedef struct RGB_n{ //normalised RGB
	float red;
	float green;
	float blue;
}color_rgb_n;

typedef struct HSV{ //HSV color
	float hue;
	float saturation;
	float value;
}color_hsv;
void get_color(void);
float get_robot_pos_x(void);
float get_robot_pos_y(void);
void set_robot_pos_x(float x);
void set_robot_pos_y(float y);
float get_pos_x(void);
float get_pos_y(void);
void process_image_start(void);
void led_match(struct RGB_n input);
void RGB2HSV(struct RGB_n input, struct HSV *output);
float max(float a, float b, float c);
float min(float a, float b, float c);
void print_color(struct RGB_n input_RGB, struct HSV input_HSV, color_type format);
float convert_coord_cm(float coord);

//static BSEMAPHORE_DECL(color_ready_sem, TRUE);

#endif /* PROCESS_IMAGE_H */
