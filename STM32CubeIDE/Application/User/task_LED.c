/*
 * m365
 *
 * Copyright (c) 2021 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <commands.h>
#include <mc_interface.h>
#include "task_LED.h"
#include "task_init.h"
#include "task_cli.h"
#include "main.h"
#include "conf_general.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lispif.h"


TaskHandle_t LEDHandle;



void prv_LED_blink(uint32_t speed){
	static uint16_t cnt=0;
	if(cnt>speed){
		cnt=0;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}else{
		cnt++;
	}



}

void startup_thread(void * arg){
	lispif_init();
	vTaskDelete(NULL);
	vTaskDelay(portMAX_DELAY);
}



void task_LED(void * argument)
{
	mc_interface_init();
	xTaskCreate(startup_thread, "tskStart", 3096, NULL, PRIO_NORMAL, NULL);

	/* Infinite loop */
	for(;;)
	{

			prv_LED_blink(50);

		vTaskDelay(MS_TO_TICKS(10));
	}
}

void task_LED_init(port_str * port){
	xTaskCreate(task_LED, "tskLED", 128, (void*)port, PRIO_NORMAL, &LEDHandle);
}
