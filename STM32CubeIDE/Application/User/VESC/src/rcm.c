/*
 * rcm.c
 *
 *  Created on: 01.06.2022
 *      Author: jensk
 */
#include "rcm.h"
#include "task.h"
#include "dll.h"
#include "stm32f1xx_ll_adc.h"


DLL_t dll;

void rcm_thread(void * arg);

void rcm_init(void){
	LL_ADC_REG_SetSequencerLength( ADC3, LL_ADC_REG_SEQ_SCAN_DISABLE );
	dll_init(&dll);
	xTaskCreate(rcm_thread, "tskRCM", 256, &dll, PRIO_NORMAL, NULL);
}

rcm_handle rcm_add_conversion(ADC_TypeDef * adc, uint8_t channel, uint32_t sampling_time, TickType_t sample_delay, volatile uint16_t * result_ptr){
	if(adc == NULL || result_ptr == NULL) return NULL;

	RegConv_t * conv = pvPortMalloc(sizeof(RegConv_t));
	if(conv == NULL) return NULL;

	conv->regADC = adc;
	conv->channel = channel;
	conv->sampling_time = sampling_time;
	conv->result_ptr = result_ptr;
	conv->dt = 0.0;
	conv->last_time = xTaskGetTickCount();
	conv->sample_delay = sample_delay;

	LL_ADC_SetChannelSamplingTime ( adc, __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel) ,sampling_time);

	return (rcm_handle)dll_insert_head(&dll, conv);
}


void rcm_thread(void * arg){
	DLL_t * dll_ptr = arg;

	while(1){

		Node_t * next = dll_get_next(dll_ptr);
		if(next){
			RegConv_t * conv = next->data;

			if(conv->result_ptr && ((conv->last_time + conv->sample_delay) < xTaskGetTickCount())){
				LL_ADC_REG_SetSequencerRanks(conv->regADC,LL_ADC_REG_RANK_1,__LL_ADC_DECIMAL_NB_TO_CHANNEL(conv->channel));
				LL_ADC_REG_ReadConversionData12(conv->regADC );
				LL_ADC_REG_StartConversionSWStart(conv->regADC);
				vTaskDelay(1);
				*conv->result_ptr = LL_ADC_REG_ReadConversionData12(conv->regADC);
				conv->dt =  (xTaskGetTickCount() - conv->last_time) * (1.0/configTICK_RATE_HZ);
				conv->last_time = xTaskGetTickCount();

			}
		}

	}
}
