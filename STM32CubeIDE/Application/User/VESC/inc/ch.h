/*
 * ch.h
 *
 *  Created on: 07.08.2022
 *      Author: jensk
 */

#ifndef APPLICATION_USER_VESC_INC_CH_H_
#define APPLICATION_USER_VESC_INC_CH_H_

#include <stdint.h>

//Wrapper defines

typedef uint32_t systime_t;

#define chThdSleepMilliseconds(ms) 			vTaskDelay(MS_TO_TICKS(ms));
#define chVTGetSystemTime()					xTaskGetTickCount()
#define chVTGetSystemTimeX()				xTaskGetTickCount()
#define MS2ST(x)							MS_TO_TICKS(x)
#define chRegSetThreadName(x)
#define THD_FUNCTION(t_name,argument)		void t_name(void * argument)


#endif /* APPLICATION_USER_VESC_INC_CH_H_ */
