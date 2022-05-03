#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>
#include <audio/play_melody.h>
#include <process_image.h>
#include <motors.h>
#include "motor.h"
#include "gpio.h"

#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE)

static float distance_cm = 0;
static float l_tot = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle

static float robot_x;
static float robot_y;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	//uint8_t imageR[IMAGE_BUFFER_SIZE] = {0};
	uint8_t imageG[IMAGE_BUFFER_SIZE] = {0};
	//uint8_t imageB[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE-1) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			//imageR[i/2] = (uint8_t)img_buff_ptr[i];
			imageG[i/2] = ((((uint8_t)img_buff_ptr[i]&0b00000111)<<3) +(((uint8_t)img_buff_ptr[i+1]&0b11100000)>>5));
			//imageG[i/2] = (uint8_t)img_buff_ptr[i+1]&11100000;
			//if(i < (2*IMAGE_BUFFER_SIZE)){
			//	imageB[i/2] = (uint8_t)img_buff_ptr[i+1]&0b00011111;
			//}
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(imageG);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}

		//uint8_t max = 0;
		//for(uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i+=2){
		//	if(image[i/2] > max){
		//		max = image[i/2];
		//	}
		//}
		//float moy_r = 0;
		//float moy_b = 0;
		//for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){
		//	moy_r = moy_r + imageR[i];
			//moy_b = moy_b + imageB[i];
		//}
		//moy_r = moy_r/IMAGE_BUFFER_SIZE;
		//moy_b = moy_b/IMAGE_BUFFER_SIZE;

		uint8_t rouge = 0;
		uint8_t vert = 0;
		uint8_t bleu = 0;
		for(uint16_t i = 0; i < 10; i++){
			rouge += ((uint8_t)img_buff_ptr[IMAGE_BUFFER_SIZE+i]&0b11111000)>>3;
			vert +=((((uint8_t)img_buff_ptr[IMAGE_BUFFER_SIZE+i]&0b00000111)<<3) +(((uint8_t)img_buff_ptr[IMAGE_BUFFER_SIZE+1+i]&0b11100000)>>5));
			bleu += (uint8_t)img_buff_ptr[(IMAGE_BUFFER_SIZE) + 1 + i]&0b00011111;
		}

		float moy_r = (float)rouge/10;
		float moy_g = (float)vert/10;
		float moy_b = (float)bleu/10;

		uint8_t rouge_888 = moy_r/31 * 255;
		uint8_t vert_888 = moy_g/63 * 255;
		uint8_t bleu_888 = moy_b/31 * 255;

		float x = convert_rgb_cm(rouge_888);
		float y = convert_rgb_cm(bleu_888);

		goto_position(x,y);

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(imageG, IMAGE_BUFFER_SIZE);
			//chprintf((BaseSequentialStream *)&SD3, "moy_r =  %f \r", (float)moy_r);
			//chprintf((BaseSequentialStream *)&SD3, "moy_b =  %i \r", (float)moy_b);
			//if(moy_r <= 75){
			//	chprintf((BaseSequentialStream *)&SD3, "bleu \r");
			//}
			//else if(moy_r >75){
			//	chprintf((BaseSequentialStream *)&SD3, "rouge \r");
			//}

			chprintf((BaseSequentialStream *)&SD3, " R= %i", rouge_888);
			chprintf((BaseSequentialStream *)&SD3, " G= %i", vert_888);
			chprintf((BaseSequentialStream *)&SD3, " B= %i\n", bleu_888);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

float convert_rgb_cm(uint8_t c){
	float x = l_tot*(float)c/255;
	return x;
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void goto_position(float x,float y){

	// + + = forward
	// - - = backward
	// + - = ccw
	// - + = cw

	if(robot_x == 0 && robot_y == 0){
		motor_set_position(y,y,5,5);
		while(motor_position_reached() != POSITION_REACHED);
	    motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, -5, 5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(x,x,5,5);
		while(motor_position_reached() != POSITION_REACHED);

		robot_x = x;
		robot_y = y;

	} else if(x < robot_x && y < robot_y){
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,5,-5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(robot_y - y,robot_y - y,5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(robot_x - x,robot_x - x,5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2,-5,5);
		while(motor_position_reached() != POSITION_REACHED);

		robot_x = x;
		robot_y = y;

	} else if(x > robot_x && y < robot_y){
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,5,-5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(robot_y - y,robot_y - y,5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,5,-5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(x - robot_x,x - robot_x,5,5);
		while(motor_position_reached() != POSITION_REACHED);

		robot_x = x;
		robot_y = y;

	} else if(x < robot_x && y > robot_y){
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(y - robot_y,y - robot_y,5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(robot_x - x,robot_x - x,5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2,-5,5);
		while(motor_position_reached() != POSITION_REACHED);

		robot_x = x;
		robot_y = y;

	} else if(x > robot_x && y > robot_y){
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,-5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(y - robot_y,y - robot_y,5,5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(PERIMETER_EPUCK/4,PERIMETER_EPUCK/4,5,-5);
		while(motor_position_reached() != POSITION_REACHED);
		motor_set_position(x - robot_x,x - robot_x,5,5);
		while(motor_position_reached() != POSITION_REACHED);

		robot_x = x;
		robot_y = y;
	}
}
