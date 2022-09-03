/*
 * hw_m365.h
 *
 *  Created on: 25.07.2022
 *      Author: jensk
 */

#ifndef HW_M365_H_
#define HW_M365_H_


#define HW_NAME					"GD32_m365_20s"

#define RSHUNT                  0.00200
#define AMPLIFICATION_GAIN      8.00

//Voltage Dividers
#define VBUS_R1 180000.0
#define VBUS_R2 6200.0
#define VPHASE_R1 39000.0
#define VPHASE_R2 2000.0

#define HW_DEAD_TIME_NSEC		500.0

//Limits
// Setting limits
#define HW_LIM_CURRENT			-70.0, 70.0
#define HW_LIM_CURRENT_IN		-70.0, 70.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_VIN				6.0, 56.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 50000.0

#endif /* APPLICATION_USER_VESC_INC_HW_M365_H_ */
