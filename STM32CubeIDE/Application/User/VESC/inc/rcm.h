/*
 * rcm.h
 *
 *  Created on: 01.06.2022
 *      Author: jensk
 */

#ifndef RCM_H
#define RCM_H

#include "main.h"
#include "FreeRTOS.h"

typedef struct
{
  ADC_TypeDef * regADC;
  uint8_t  channel;
  uint32_t sampling_time;
  volatile uint16_t * result_ptr;
  TickType_t last_time;
  float dt;
} RegConv_t;

typedef void* rcm_handle;

void rcm_init(void);

rcm_handle rcm_add_conversion(ADC_TypeDef * adc, uint8_t channel, uint32_t sampling_time, volatile uint16_t * result_ptr);

#endif /* APPLICATION_USER_VESC_INC_RCM_H_ */
