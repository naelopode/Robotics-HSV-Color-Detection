#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include "detect_obj.h"
#include "sensors/proximity.h"
#include "global.h"

#define THRESHOLD_PROXIMITY 		700 //Proximity threshold (more than this value will pause the robot)
//semaphore
static bool detected_flag = FALSE;

uint8_t detected_obj(void){ //Simple pooling function that determine if an object is in range
	bool detected = FALSE;
	for (uint8_t i = 0; i <PROXIMITY_NB_CHANNELS;++i){
		if(get_prox(i)>THRESHOLD_PROXIMITY){
			detected = TRUE;
			return TRUE;
		}
	}
	if (!detected){
		return FALSE;
		detected_flag=FALSE;
	}
}

void detect_obj_start(void){
	proximity_start();
	calibrate_ir();
}


