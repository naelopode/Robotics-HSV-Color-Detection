#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		1000
#define ROTATION_COEFF			0.2
#define PXTOCM					1570.0f //experimental value
#define GOAL_AMP_DIFF			0
#define GOAL_DISTANCE 			0
#define MIC_RESOLUTION			500
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			1000	//[cm] because of the noise of the camera
#define KP						5.0f
#define KI 						0.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
