/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <commands.h>
#include "shutdown.h"
#include "app.h"
#include "conf_general.h"
#include "timeout.h"
#include "FreeRTOS.h"
#include "hw.h"

//Debug
#include "task_init.h"

#ifdef HW_SHUTDOWN_HOLD_ON

// Private variables
bool volatile m_button_pressed = false;
static volatile float m_inactivity_time = 0.0;

SemaphoreHandle_t m_sample_mutex;

static volatile bool m_init_done = false;
static volatile bool m_sampling_disabled = false;

void shutdown_thread(void * arg);

void shutdown_init(void) {
	m_sample_mutex = xSemaphoreCreateMutex();
	xTaskCreate(shutdown_thread, "Shutdown", 256, NULL, PRIO_NORMAL, NULL);
	m_init_done = true;
}

void shutdown_reset_timer(void) {
	m_inactivity_time = 0.0;
}

bool shutdown_button_pressed(void) {
	return m_button_pressed;
}

float shutdown_get_inactivity_time(void) {
	return m_inactivity_time;
}

void shutdown_set_sampling_disabled(bool disabled) {
	if (!m_init_done) {
		return;
	}

	xSemaphoreTake(m_sample_mutex, portMAX_DELAY);
	m_sampling_disabled = disabled;
	xSemaphoreGive(m_sample_mutex);
}

//static bool do_shutdown(void) {
//	//conf_general_store_backup_data();
//	power_control(DEV_PWR_OFF);
//	return true;
//}

uint8_t buttonState() {
    static const uint32_t DEBOUNCE_MILLIS = MS_TO_TICKS(20) ;
    bool buttonstate = HAL_GPIO_ReadPin( PWR_BTN_GPIO_Port, PWR_BTN_Pin ) == GPIO_PIN_SET ;
    uint32_t buttonstate_ts = xTaskGetTickCount();

    uint32_t now = xTaskGetTickCount();
    if( now - buttonstate_ts > DEBOUNCE_MILLIS )
    {
        if( buttonstate != (HAL_GPIO_ReadPin( PWR_BTN_GPIO_Port, PWR_BTN_Pin ) == GPIO_PIN_SET))
        {
            buttonstate = !buttonstate ;
            buttonstate_ts = now ;
        }
    }
    return buttonstate ;
}

eButtonEvent getButtonEvent()
{
    static const uint32_t DOUBLE_GAP_MILLIS_MAX 	= MS_TO_TICKS(250);
    static const uint32_t SINGLE_PRESS_MILLIS_MAX 	= MS_TO_TICKS(300);
    static const uint32_t LONG_PRESS_MILLIS_MAX 	= MS_TO_TICKS(5000);

    static uint32_t button_down_ts = 0 ;
    static uint32_t button_up_ts = 0 ;
    static bool double_pending = false ;
    static bool button_down = false ; ;

    eButtonEvent button_event = NO_PRESS ;
    uint32_t now = xTaskGetTickCount();

    if( button_down != buttonState() ) {
        button_down = !button_down ;
        if( button_down ) {
            button_down_ts = now ;
        } else {
            button_up_ts = now ;
            if( double_pending ) {
                button_event = DOUBLE_PRESS ;
                double_pending = false ;
            }
            else {
                double_pending = true ;
            }
        }
    }

    uint32_t diff =  button_up_ts - button_down_ts;
    if (!button_down && double_pending && now - button_up_ts > DOUBLE_GAP_MILLIS_MAX) {
    	double_pending = false ;
    	button_event = SINGLE_PRESS ;
	} else if (!button_down && double_pending && diff >= SINGLE_PRESS_MILLIS_MAX && diff <= LONG_PRESS_MILLIS_MAX) {
		double_pending = false ;
		button_event = LONG_PRESS ;
	} else if (button_down && now - button_down_ts > LONG_PRESS_MILLIS_MAX) {
		double_pending = false ;
		button_event = VERY_LONG_PRESS ;
	}

    return button_event ;
}


void power_control(uint8_t pwr)
{
	if(pwr == DEV_PWR_ON) {
		/* Turn the PowerON line high to keep the board powered on even when
		 * the power button is released
		 */
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_SET);
	} else if(pwr == DEV_PWR_OFF) {

		vTaskDelay(1);

		while(HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin));
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_RESET);
		while(1);
	} else if(pwr == DEV_PWR_RESTART) {
		/* Restart the system */
		NVIC_SystemReset();
	}
}


void button_process(void) {

	switch( getButtonEvent() ){
		  case NO_PRESS : break ;
		  case SINGLE_PRESS : {
		  } break ;
		  case LONG_PRESS :   {
			  power_control(DEV_PWR_OFF);

		  } break ;
		  case VERY_LONG_PRESS :   {

		  } break ;
		  case DOUBLE_PRESS : {

		  } break ;
	 }
}


void shutdown_thread(void * arg){
	(void)arg;

	systime_t last_iteration_time = xTaskGetTickCount();

	for(;;) {
		float dt = (float)TimeElapsedSinceX(last_iteration_time) / (float)configTICK_RATE_HZ;
		last_iteration_time = xTaskGetTickCount();

		xSemaphoreTake(m_sample_mutex, portMAX_DELAY);

		button_process();

		xSemaphoreGive(m_sample_mutex);

		const app_configuration *conf = app_get_configuration();

		if (conf->shutdown_mode >= SHUTDOWN_MODE_OFF_AFTER_10S) {
			m_inactivity_time += dt;

			float shutdown_timeout = 0.0;
			switch (conf->shutdown_mode) {
			case SHUTDOWN_MODE_OFF_AFTER_10S: shutdown_timeout = 10.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_1M: shutdown_timeout = 60.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_5M: shutdown_timeout = 60.0 * 5.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_10M: shutdown_timeout = 60.0 * 10.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_30M: shutdown_timeout = 60.0 * 30.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_1H: shutdown_timeout = 60.0 * 60.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_5H: shutdown_timeout = 60.0 * 60.0 * 5.0; break;
			default: break;
			}

			if (m_inactivity_time >= shutdown_timeout) {
				power_control(DEV_PWR_OFF);
			}
		} else {
			m_inactivity_time = 0.0;
		}

		vTaskDelay(MS_TO_TICKS(10));
	}
}

#endif
