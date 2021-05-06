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
#include <leds.h>


#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define MM_TO_CM			0.1

#define POSITION_REACHED 1
#define POSITION_NOT_REACHED 0



static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;

static int32_t position_to_reach_right=0;
static int32_t position_to_reach_left=0;


//simple PI regulator implementation
int16_t pi_regulator(uint16_t distance, uint16_t goal){

	int16_t error = 0;
	int32_t speed = 0;

	int16_t sum_error = 0;

	error = distance-goal;
	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth

	speed = (KP * error) * NSTEP_ONE_TURN / WHEEL_PERIMETER;//+ KI * sum_error;
	/* limit motor speed */
	if (speed > MOTOR_SPEED_LIMIT) {
	   speed = MOTOR_SPEED_LIMIT;
	} else if (speed < -MOTOR_SPEED_LIMIT) {
	   speed = -MOTOR_SPEED_LIMIT;
	}

//	chprintf((BaseSequentialStream *)&SD3,"::::::: Speed = %d:::::::", speed);
    return  speed;
}

// Set the target position in cm
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


//
static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;

    uint8_t mode = SEARCH_MODE;

    //SEARCH_MODE variable
    uint16_t distance =0;
    int16_t speed = 0;

    //EXE_MODE variable
    uint8_t cnt=0;
    uint8_t reverse=false;
    uint8_t stop=false;
    uint8_t tab_cmd[MAX_CODE_LENGTH];

    while(1){
    	time = chVTGetSystemTime();
        switch (mode){
        	// PI regulator active when image processor Thread search the code
        	case SEARCH_MODE :
				//distance_mm is modified by VL53L0X sensors
				distance = VL53L0X_get_dist_mm()*MM_TO_CM;
				speed = pi_regulator(distance, GOAL_DISTANCE);
				right_motor_set_speed(speed);
				left_motor_set_speed(speed);
				//Bring back the new mode in image processor Thread
	        	mode = get_mode();

			//Execution mode : Execute the code
        	case EXE_MODE :
        		//Bring back the table of instructions
        		get_tab(tab_cmd);
        		//loop to decode instructions and make the action
        		while (!stop)
        		{
        			switch (tab_cmd[cnt]){
						case ADVANCE:
							tab_cmd[cnt]=RETREAT;
							motor_set_position(20, 20, 7, 7);
							set_led(LED1,1);
							break;
						case RETREAT:
							tab_cmd[cnt]=ADVANCE;
							motor_set_position(20, 20, -7, -7);
							set_led(LED5,1);
							break;
						case RIGHT:
							tab_cmd[cnt]=LEFT;
							motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, -7, 7);
							set_led(LED3,1);
							break;
						case LEFT:
							tab_cmd[cnt]=RIGHT;
							motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, 7, -7);
							set_led(LED7,1);
							break;
						case END:
							motor_set_position(PERIMETER_EPUCK, PERIMETER_EPUCK, 7, -7);
							set_body_led(1);
							break;
        			}
        			while(motor_position_reached() != POSITION_REACHED);
        			//clear all the leds after the action
        			clear_leds();
        			set_body_led(0);
        			//Reversing the order of reading when it's the end
        			if ((tab_cmd[cnt]==END) | ((cnt+1==MAX_CODE_LENGTH) & (!reverse))){
        				reverse=true;
        			//Stop the EXE mode when reading takes place both ways
        			} else if(reverse & ((cnt)==0)){
        				stop = true;
        			}
        			//Increment or decrement according to the order of reading
        			if (!reverse){
        				cnt++;
        			}else {
        				cnt--;
        			}
        		}
        		mode=SEARCH_MODE;
        		set_mode(SEARCH_MODE);
        		stop=false;
        		reverse=false;
        	}
    }

    //100Hz
    chThdSleepUntilWindowed(time, time + MS2ST(10));
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
