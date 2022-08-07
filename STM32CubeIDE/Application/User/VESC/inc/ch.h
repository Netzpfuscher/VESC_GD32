/*
 * ch.h
 *
 *  Created on: 07.08.2022
 *      Author: jensk
 */

#ifndef APPLICATION_USER_VESC_INC_CH_H_
#define APPLICATION_USER_VESC_INC_CH_H_

//Wrapper defines

typedef uint32_t systime_t;

#define chThdSleepMilliseconds(ms) 			vTaskDelay(MS_TO_TICKS(ms));


#endif /* APPLICATION_USER_VESC_INC_CH_H_ */
