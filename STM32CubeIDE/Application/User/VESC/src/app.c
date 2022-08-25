#include "app.h"
#include "crc.h"
#include "shutdown.h"
#include "task_init.h"
#include "utils.h"
#include "timers.h"
#include "hw.h"

extern app_configuration appconf;
static volatile bool output_disabled_now = false;
TimerHandle_t app_disable;

app_configuration* app_get_configuration(void) {
	return &appconf;
}

void vEnableOutput( TimerHandle_t xTimer ){
	output_disabled_now = false;
}


/**
 * Reconfigure and restart all apps. Some apps don't have any configuration options.
 *
 * @param conf
 * The new configuration to use.
 */
void app_set_configuration(app_configuration *conf) {
	appconf = *conf;

	if(app_disable==NULL){
		app_disable = xTimerCreate("DIS_TMR",MS_TO_TICKS(20) , pdFALSE, ( void * ) 0,vEnableOutput );
	}

	VESC_USART_DMA.Init.BaudRate = appconf.app_uart_baudrate;

	HAL_UART_Init(&VESC_USART_DMA);


	utils_truncate_number_int((int*)&appconf.app_adc_conf.update_rate_hz, 1, 200);

	app_adc_configure(&appconf.app_adc_conf);
	app_nunchuk_configure(&appconf.app_chuk_conf);

#ifndef DEBUG_ISR
	if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
		app_adc_stop();
		app_nunchuk_stop();
		task_cli_kill(&aux_uart);
	}

	switch (appconf.app_to_use) {
		case APP_UART:
			task_cli_init(&aux_uart);
			break;
		case APP_ADC:
			app_adc_start(false);
			break;
		case APP_ADC_UART:
			app_adc_start(false);
			task_cli_init(&aux_uart);
			break;
		case APP_NUNCHUK:
			app_nunchuk_start();
			break;
		default:
			break;
	}
#endif
}



/**
 * Disable output on apps
 *
 * @param time_ms
 * The amount of time to disable output in ms
 * 0: Enable output now
 * -1: Disable forever
 * >0: Amount of milliseconds to disable output
 */
void app_disable_output(int time_ms) {
	if (time_ms == 0) {
		output_disabled_now = false;
	} else if (time_ms == -1) {
		output_disabled_now = true;
	} else {
		xTimerChangePeriod(app_disable, MS_TO_TICKS(time_ms),100);
		xTimerStart(app_disable, 100);
		output_disabled_now = true;
	}
}

bool app_is_output_disabled(void) {
	return output_disabled_now;
}

/**
 * Get app_configuration CRC
 *
 * @param conf
 * Pointer to app_configuration or NULL for current appconf
 *
 * @return
 * CRC16 (with crc field in struct temporarily set to zero).
 */
unsigned app_calc_crc(app_configuration* conf) {
	//if(null == conf)
	//	conf = &appconf;

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(app_configuration));
	conf->crc = crc_old;
	return crc_new;
}
