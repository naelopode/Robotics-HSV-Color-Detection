#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>
#include <main.h>
#include <camera/po8030.h>
#include <audio/play_melody.h>
#include <process_image.h>
#include <math.h>
#include <motors.h>
#include <coordinate_motor.h>
#include <button.h>

static float distance_cm = 0;
static float l_tot = 20;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle

static float robot_x = 0;
static float robot_y = 0;

static float x = 0;
static float y = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 310 to 330 of line 230 to 249
	po8030_advanced_config(FORMAT_RGB565, 310, 230, IMAGE_BUFFER_SIZE, IMAGE_BUFFER_SIZE, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_ae(0);
	po8030_set_awb(0);
	//po8030_set_brightness(0b00000000);
	//po8030_set_contrast(0x40);
	po8030_set_exposure(0x0080,0x00);//0x0080,0x00
	po8030_set_rgb_gain(0x5E,0x40,0x5D);//0x5E,0x40,0x5D
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
    	//chThdSleepMilliseconds(1000);
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
    static struct RGB_l color_rgb_long =  {0,0,0};
    uint8_t compte_mesures = NB_MESURES;
	uint8_t *img_buff_ptr;


    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//struct RGB_i color_rbg_i = {0,0,0};//value can reach 64*IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE so using int

		for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE; i++){
			color_rgb_long.red += (unsigned long int) (((uint8_t)img_buff_ptr[2*i]&0b11111000)>>3);
			color_rgb_long.green += (unsigned long int) ((((uint8_t)img_buff_ptr[2*i]&0b00000111)<<3) + (((uint8_t)img_buff_ptr[2*i+1]&0b11100000)>>5));
			color_rgb_long.blue +=  (unsigned long int) ((uint8_t)img_buff_ptr[2*i+1]&0b00011111);
		}

//		chprintf((BaseSequentialStream *)&SD3, "%lu, ", color_rgb_long.red);
//		chprintf((BaseSequentialStream *)&SD3, "%lu, ", color_rgb_long.green);
//		chprintf((BaseSequentialStream *)&SD3, "%lu \n\r", color_rgb_long.blue);
//		color_rgb_long.red = color_rgb_long.red/(IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE);
//		color_rgb_long.green = color_rgb_long.green/(IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE);
//		color_rgb_long.blue = color_rgb_long.blue/(IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE);

		compte_mesures -=1;

		if(compte_mesures==0){

			//uint8_t rouge_888 = (uint8_t) ((float)global_red/((float)(nombre_mesure)*32)*255);
			//uint8_t vert_888 = (uint8_t) ((float)global_green/((float)(nombre_mesure)*64)*255);
			//uint8_t bleu_888 = (uint8_t) ((float)global_blue/((float)(nombre_mesure)*32)*255);

			struct RGB_n color_rgb_n = {(float)color_rgb_long.red/((float)(NB_MESURES*IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE*31)),
								 	 	(float)color_rgb_long.green/((float)(NB_MESURES*IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE*63)),
										(float)color_rgb_long.blue/((float)(NB_MESURES*IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE*31))};
			//chprintf((BaseSequentialStream *)&SD3, "%.1f \n\r", color_rgb_n.red);
			struct HSV color_hsv = {0.,0.,0.};

//			uint8_t rouge_pc = (uint8_t) ((float)global_red/((float)(nombre_mesure)*32)*100);
//			uint8_t vert_pc = (uint8_t) ((float)global_green/((float)(nombre_mesure)*64)*100);
//			uint8_t bleu_pc = (uint8_t) ((float)global_blue/((float)(nombre_mesure)*32)*100);

			led_match(color_rgb_n);

			RGB2HSV(color_rgb_n, &color_hsv);

			print_color(color_rgb_n, color_hsv, RGB);
//			chprintf((BaseSequentialStream *)&SD3, "%f \n", color_hsv.hue);
//			chprintf((BaseSequentialStream *)&SD3, "%f \n", color_hsv.saturation);
//			chprintf((BaseSequentialStream *)&SD3, "%f \n\r", color_hsv.value);


			//RESET VALUE FOR NEXT ROUND OF MEASURMENTS
			color_rgb_long.red=0;
			color_rgb_long.green=0;
			color_rgb_long.blue=0;
			compte_mesures = NB_MESURES;
		}
		//*img_buff_ptr = NULL;
    }
}

void get_color(){

}

void led_match(struct RGB_n input){
	for (uint8_t i = 0; i<NUM_RGB_LED;++i){
		set_rgb_led(i,(uint8_t) (input.red*RGB_MAX_INTENSITY),
					  (uint8_t) (input.green*RGB_MAX_INTENSITY),
					  (uint8_t) (input.blue*RGB_MAX_INTENSITY));
	}
}


void RGB2HSV(struct RGB_n input, struct HSV *output){
	float cmax = max(input.red, input.green, input.blue); // maximum of r, g, b
	float cmin = min(input.red, input.green, input.blue); // minimum of r, g, b
	float diff = cmax-cmin; // diff of cmax and cmin.
	if (cmax == cmin)
		output->hue = 0;
	else if (cmax == input.red)
		output->hue = fmod((60 * ((input.green - input.blue) / diff) + 360), 360.0);
	else if (cmax == input.green)
		output->hue = fmod((60 * ((input.blue - input.red) / diff) + 120), 360.0);
	else if (cmax == input.blue)
		output->hue = fmod((60 * ((input.red - input.green) / diff) + 240), 360.0);
	// if cmax equal zero
	if (cmax == 0)
		output->saturation = 0;
	else
		output->saturation = (diff / cmax) * 100;
	// compute v
	output->value = cmax * 100;
}

float max(float a, float b, float c) {
   return ((a > b)? (a > c ? a : c) : (b > c ? b : c));
}

float min(float a, float b, float c) {
   return ((a < b)? (a < c ? a : c) : (b < c ? b : c));
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void print_color(struct RGB_n input_RGB, struct HSV input_HSV, color_type format){
	switch (format){
		case RGB:
			chprintf((BaseSequentialStream *)&SD3, "A%u\n", (uint8_t) (input_RGB.red*255.));
			chprintf((BaseSequentialStream *)&SD3, "%u\n", (uint8_t) (input_RGB.green*255.));
			chprintf((BaseSequentialStream *)&SD3, "%u\n", (uint8_t) (input_RGB.blue*255.));
			break;
		case HSV:
			chprintf((BaseSequentialStream *)&SD3, "%.0f, ", input_HSV.hue);
			chprintf((BaseSequentialStream *)&SD3, "%.2f, ", input_HSV.saturation);
			chprintf((BaseSequentialStream *)&SD3, "%.2f \n\r", input_HSV.value);
			break;
	}
}
