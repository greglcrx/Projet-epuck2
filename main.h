#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			35 //[cm]
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			2	//[cm] because of the noise of the camera
#define KP						8
#define KI 						3	//must not be zero 3.5
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#define SEARCH_MODE 0
#define EXE_MODE 1

#define END 0
#define ADVANCE 1
#define RETREAT 2
#define RIGHT 3
#define LEFT 4

#define MAX_CODE_LENGTH			5
#define VALID_CODE				1

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
