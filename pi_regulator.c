#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <sensors\VL53L0X\VL53L0X.h>
#include <pi_regulator.h>
#include <process_image.h>

//#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

#define POSITION_REACHED 1
#define POSITION_NOT_REACHED 0

static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;

static int32_t position_to_reach_right=0;
static int32_t position_to_reach_left=0;


//simple PI regulator implementation
int16_t pi_regulator(uint16_t distance, uint16_t goal){

	uint16_t error = 0;
	uint16_t speed = 0;

	static uint16_t sum_error = 0;

	error = distance-goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error ;//+ KI * sum_error;

    return (int16_t)speed;
}

// Set the target position
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{

	int16_t speed_r_step_s,speed_l_step_s;

	//reinit global variable
	right_motor_set_pos(0);
	left_motor_set_pos(0);

    position_right_reached = 0;
    position_left_reached = 0;

	//Set global variable with position to reach in step
	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	// Limit motor speed
	if (speed_r > MOTOR_SPEED_LIMIT) {
		speed_r = MOTOR_SPEED_LIMIT;
	} else if (speed_r < -MOTOR_SPEED_LIMIT) {
		speed_r = -MOTOR_SPEED_LIMIT;
	}
	if (speed_l > MOTOR_SPEED_LIMIT) {
		speed_l = MOTOR_SPEED_LIMIT;
	} else if (speed_l < -MOTOR_SPEED_LIMIT) {
		speed_l = -MOTOR_SPEED_LIMIT;
	}

	//transform the speed from cm/s into step/s
	speed_r_step_s = speed_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	speed_l_step_s = speed_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	//send to motor the speed in step/s
	right_motor_set_speed(speed_r_step_s);
	left_motor_set_speed(speed_l_step_s);
}

// Evaluate if the target position is reach
uint8_t motor_position_reached(void)
{
	int32_t pos_l = left_motor_get_pos();
	int32_t pos_r = right_motor_get_pos();
    if(abs(position_to_reach_left)<abs(pos_l) && abs(position_to_reach_right)<abs(pos_r)){
    	right_motor_set_speed(0);
    	left_motor_set_speed(0);
        return POSITION_REACHED;
    }else{
        return POSITION_NOT_REACHED;
    }
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;
    int16_t speed = 0;
    uint8_t mode = SEARCH_MODE;
    uint8_t i=0;
    uint8_t reverse=0;
    uint8_t stop=0;
    uint8_t tab_cmd[MAX_CODE_LENGTH];
    uint16_t distance =0;
    while(1){
    	time = chVTGetSystemTime();
        switch (mode){
        	// PI regulator active when image processor Thread search the code
        	case SEARCH_MODE :
        		while (mode == SEARCH_MODE)
				{
					//distance_mm is modified by VL53L0X sensors
					distance = VL53L0X_get_dist_mm();
					//chprintf((BaseSequentialStream *)&SD3,"Distance = %d", distance);
					speed = pi_regulator(distance, GOAL_DISTANCE);
					right_motor_set_speed(speed);
					left_motor_set_speed(speed);
					//Take back the mode in image processor Thread
	        		mode = get_mode();
				}

			//Execution mode : Execute the code
        	case EXE_MODE :
        		get_tab(tab_cmd);
        		for (int i = 0; i < MAX_CODE_LENGTH; i++ ) {
        					chprintf((BaseSequentialStream *)&SD3,"tab %d : %d - ", i, tab_cmd[i]);
        				}
        		while (!stop)
        		{
        			switch (tab_cmd[i]){
						case ADVANCE:
							tab_cmd[i]=RETREAT;
							motor_set_position(20, 20, 7, 7);
							break;
						case RETREAT:
							tab_cmd[i]=ADVANCE;
							motor_set_position(20, 20, -7, -7);
							break;
						case RIGHT:
							tab_cmd[i]=LEFT;
							motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, -7, 7);
							break;
						case LEFT:
							tab_cmd[i]=RIGHT;
							motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, 7, -7);
							break;
						case END:
							motor_set_position(PERIMETER_EPUCK, PERIMETER_EPUCK, 7, -7);
							break;
        			}
        			while(motor_position_reached() != POSITION_REACHED);

        			//Reversing the order of reading when it's the end
        			if ((tab_cmd[i]==END) | ((i+1==MAX_CODE_LENGTH) & (!reverse))){
        				reverse=1;
        			//Stop the EXE mode when reading takes place both ways
        			}else if(reverse & ((i)==0)){
        				stop = 1;
        			}
        			//Increment or decrement according to the order of reading
        			if (!reverse){
        				i++;
        			}else {
        				i--;
        			}
        		}

        		mode=SEARCH_MODE;
        		set_mode(SEARCH_MODE);
        		stop=0;
        		reverse=0;
        	}
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
//}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
