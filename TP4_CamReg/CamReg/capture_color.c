#include "capture_color.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>
#include <main.h>
#include <camera/po8030.h>
#include <math.h>
#include <coordinate_motor.h>
#include "led_anim.h"
#include "global.h"

static BSEMAPHORE_DECL(capture_start, TRUE); //Semaphore to start a capture
static BSEMAPHORE_DECL(image_ready_sem, TRUE); //Semaphore for when a capture is ready
static BSEMAPHORE_DECL(capture_finished,TRUE); //Semaphore to declare a capture finished

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 310 to 330 of line 230 to 249
	po8030_advanced_config(FORMAT_RGB565, 310, 230, IMAGE_BUFFER_SIZE, IMAGE_BUFFER_SIZE, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_ae(0); //Disable auto-exposure to get a more consistent color
	po8030_set_awb(0); //Disable auto white-balance to get a more consistent color
	//po8030_set_brightness(0b00000000); //Default value = 0 -> Tweek when necessary
	//po8030_set_contrast(0x40); //Default value = 0x40 -> Tweek when necessary
	po8030_set_exposure(0x0080,0x00);//Default value: 0x0080,0x00 -> Tweek when necessary
	po8030_set_rgb_gain(0x5E,0x40,0x5D);//Default value: 0x5E,0x40,0x5D -> Tweel when necessary
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
    	chThdSleepMilliseconds(500);
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
    	chBSemWait(&capture_start); //wait until a capture is necessary
    	set_led_state(TURN_CCW); //set leds as turning counter-clockwise
        chBSemWait(&image_ready_sem); //waits until an image has been captured

		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Bitwise operation to optain RGB565 values
		for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE; i++){
			color_rgb_long.red += (unsigned long int) (((uint8_t)img_buff_ptr[2*i]&0b11111000)>>3);
			color_rgb_long.green += (unsigned long int) ((((uint8_t)img_buff_ptr[2*i]&0b00000111)<<3) + (((uint8_t)img_buff_ptr[2*i+1]&0b11100000)>>5));
			color_rgb_long.blue +=  (unsigned long int) ((uint8_t)img_buff_ptr[2*i+1]&0b00011111);
		}

		compte_mesures -=1;
		if(compte_mesures==0){
			color_rgb_n_t color_rgb_n = {(float)color_rgb_long.red/((float)(NB_MESURES*IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE*31)),
								 	 	(float)color_rgb_long.green/((float)(NB_MESURES*IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE*63)),
										(float)color_rgb_long.blue/((float)(NB_MESURES*IMAGE_BUFFER_SIZE*IMAGE_BUFFER_SIZE*31))};

			struct HSV color_hsv = {0.,0.,0.};

			set_led_color(color_rgb_n); //set color of leds with value of detected colors
			set_led_state(ALL_ON_COLOR); //turning on colors of the leds

			RGB2HSV(color_rgb_n, &color_hsv); //Convert RGB values to HSV values

			//print_color(color_rgb_n, color_hsv, RGB); //chprintf color values for debug purposes

			color_hsv.hue = -color_hsv.hue + 90; //HSV to polar circle
			fmod(color_hsv.hue,360);

			set_robot_pos_x(color_hsv.saturation*cos(color_hsv.hue*M_PI/180)); //TODO mettre ça dans les coordinate
			set_robot_pos_y(color_hsv.saturation*sin(color_hsv.hue*M_PI/180));
			set_angle(color_hsv.hue);

			//RESET VALUE FOR NEXT ROUND OF MEASURMENTS
			color_rgb_long.red=0;
			color_rgb_long.green=0;
			color_rgb_long.blue=0;
			compte_mesures = NB_MESURES;
			chBSemSignal(&capture_finished); //Set semaphore to declared that all captures are finished
			chThdYield(); //End of thread
		} else {
			chBSemSignal(&capture_start); //Continue to capture as we need more capture for measurements
		}
    }
}

void led_match(color_rgb_n_t input){
	for (uint8_t i = 0; i<NUM_RGB_LED;++i){
		set_rgb_led(i,(uint8_t) (input.red*RGB_MAX_INTENSITY),
					  (uint8_t) (input.green*RGB_MAX_INTENSITY),
					  (uint8_t) (input.blue*RGB_MAX_INTENSITY));
	}
}

void RGB2HSV(color_rgb_n_t input, struct HSV *output){ //Code from https://www.tutorialspoint.com/c-program-to-change-rgb-color-model-to-hsv-color-model
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

float max(float a, float b, float c) {  //Code from https://www.tutorialspoint.com/c-program-to-change-rgb-color-model-to-hsv-color-model
   return ((a > b)? (a > c ? a : c) : (b > c ? b : c));
}

float min(float a, float b, float c) { //Code from https://www.tutorialspoint.com/c-program-to-change-rgb-color-model-to-hsv-color-model
   return ((a < b)? (a < c ? a : c) : (b < c ? b : c));
}

void process_image_start(void){ //Init thread
    //starts the camera
    dcmi_start();
	po8030_start();
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}

void print_color(color_rgb_n_t input_RGB, struct HSV input_HSV, color_type format){ //This function print values ! This is for debug purposes
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

void set_semaphore_capture(){ //Declare we need a capture
	chBSemSignal(&capture_start);
}

void wait_capture_ready(void) {
	//waits until capture is finished
	chBSemWait(&capture_finished);
}
