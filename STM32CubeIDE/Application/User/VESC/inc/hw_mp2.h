/*
 * hw_m365.h
 *
 *  Created on: 25.07.2022
 *      Author: jensk
 */

#ifndef HW_MP2_H_
#define HW_MP2_H_


#define HW_NAME					"MP2-ESC"

#define CURRENT_SHUNT_RES       0.00033333
#define AMPLIFICATION_GAIN      10.50

//Voltage Dividers
#define VBUS_R1 1000000.0
#define VBUS_R2 33000.0
#define VPHASE_R1 100000.0
#define VPHASE_R2 3300.0

#define HW_DEAD_TIME_NSEC		500.0

//Limits
// Setting limits
#define HW_LIM_CURRENT			-300.0, 300.0
#define HW_LIM_CURRENT_IN		-300.0, 300.0
#define HW_LIM_CURRENT_ABS		0.0, 300.0
#define HW_LIM_VIN				6.0, 100.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 50000.0



#endif /* APPLICATION_USER_VESC_INC_HW_M365_H_ */
