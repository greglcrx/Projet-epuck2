#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void motor_control_start(void);


uint8_t motor_position_reached(void);


void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);


uint8_t motor_position_reached(void);

#endif /* MOTOR_CONTROL_H */
