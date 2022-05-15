#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define MAX_RGB_VALUE 			255 //max value of RGB
#define NB_MESURES 				10 //Number of image to average color
#define IMAGE_BUFFER_SIZE		20 //Size of rectangles we want to capture
#include "global.h"

void process_image_start(void);
void set_semaphore_capture(void);
void wait_capture_ready(void);

#endif /* PROCESS_IMAGE_H */
