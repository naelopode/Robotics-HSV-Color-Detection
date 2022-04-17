#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <math.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <pi_regulator.h>


//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static float amp_diff = 0;

#define MIN_VALUE_THRESHOLD 10000

#define MIN_FREQ		10
#define FREQ_FORWARD	16
#define FREQ_LEFT		19
#define FREQ_RIGHT		23
#define FREQ_BACKWARD	26
#define MAX_FREQ		30

#define	FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		left_motor_set_speed(600);
		right_motor_set_speed(600);
	}
	else if(max_norm_index >=FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	else if(max_norm_index >=FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
			left_motor_set_speed(600);
			right_motor_set_speed(-600);
	}
	else if(max_norm_index >=FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
			left_motor_set_speed(-600);
			right_motor_set_speed(-600);
	}
	else {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
	}
}



/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

void processAudioData(int16_t *data, uint16_t num_samples){
	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	for(uint16_t i = 0; i < num_samples ; i+=4){
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//chprintf((BaseSequentialStream *)&SDU1, "hello \n");

		float max_norm = 1000;
		int16_t max_norm_index = -1;
		for(uint16_t i = 0 ; i <= FFT_SIZE-1 ; i++){
			if(micRight_output[i] > max_norm){
				max_norm = micRight_output[i];
				max_norm_index = i;
			}
		}
	//	for(uint16_t i = 0 ; i <= SAMPLES-1 ; i++){
	//		amplitude1[i] = micRight_output[max_norm_index];
	//		amplitude2[i] = micLeft_output[max_norm_index];
	//	}

		if(mustSend > SAMPLES){
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;

		//	chprintf((BaseSequentialStream *)&SDU1, "max = %d \r\n", (float)max(micBack_output));

			float moyenne_amplitude = 0;
			moyenne_amplitude = moyenne_amplitude+micRight_output[max_norm_index]-micLeft_output[max_norm_index];
			moyenne_amplitude=moyenne_amplitude/SAMPLES;
			amp_diff=moyenne_amplitude;

			//float realIn_R = micRight_cmplx_input[max_norm_index];
			//float imagIn_R = micRight_cmplx_input[max_norm_index+1];

			//float realIn_L = micLeft_cmplx_input[max_norm_index];
			//float imagIn_L = micLeft_cmplx_input[max_norm_index+1];

			//float phase1 = atan2f(imagIn_R,realIn_R);
			//float phase2 = atan2f(imagIn_L,realIn_L);
			//float delta_phase = phase2-phase1;


			//float amplitude2 = micLeft_output[max_norm_index];
			//float amplitude_delta = amplitude2-amplitude1;

		//	chprintf((BaseSequentialStream *)&SDU1, "delta phase = %f \r", (float)delta_phase);
			chprintf((BaseSequentialStream *)&SDU1, "moy amp = %f \r", (float)moyenne_amplitude);

		}

	nb_samples = 0;
	mustSend++;

	//sound_remote(micLeft_output);
	}
}

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

float get_amp_diff(){
	return amp_diff;
}
