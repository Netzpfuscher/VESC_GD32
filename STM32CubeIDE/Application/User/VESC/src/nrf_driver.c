/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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

#include <string.h>
#include "nrf_driver.h"
//#include "rf.h"
//#include "rfhelp.h"
#include "conf_general.h"
#include "app.h"
#include "buffer.h"
#include "commands.h"
#include "crc.h"
#include "packet.h"
#include "mc_interface.h"
#include "app.h"

// Settings
#define MAX_PL_LEN				25
#define RX_BUFFER_SIZE			PACKET_MAX_PL_LEN

#define ALIVE_INTERVAL			100  // Send alive packets at this rate
#define NRF_RESTART_TIMEOUT		500  // Restart the NRF if nothing has been received or acked for this time

// Variables
//static THD_WORKING_AREA(rx_thread_wa, 2048);
//static THD_WORKING_AREA(tx_thread_wa, 256);
static mote_state mstate;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static int nosend_cnt;
static int nrf_restart_rx_time;
static int nrf_restart_tx_time;

static systime_t pairing_time_end = 0;
static volatile bool pairing_active = false;

static volatile bool tx_running = false;
static volatile bool tx_stop = true;
static volatile bool rx_running = false;
static volatile bool rx_stop = true;
static volatile bool ext_nrf = false;
static volatile int driver_paused = 0;

// This is a hack to prevent race conditions when updating the appconf
// from the nrf thread
static volatile bool from_nrf = false;

// Functions
//static THD_FUNCTION(rx_thread, arg);
static THD_FUNCTION(tx_thread, arg);

static int rf_tx_wrapper(char *data, int len, PACKET_STATE_t * phandle);


void nrf_driver_init_ext_nrf(PACKET_STATE_t * phandle) {
	ext_nrf = true;

	if (!tx_running) {
		tx_stop = false;
		xTaskCreate(tx_thread, "NRF_TX", 256, phandle, PRIO_BELOW_NORMAL, NULL);
	}
}

void nrf_driver_stop(void) {
	if (from_nrf || ext_nrf) {
		return;
	}

//	if (rx_running) {
//		rfhelp_stop();
//	}

	tx_stop = true;
	rx_stop = true;

	while (rx_running || tx_running) {
		chThdSleepMilliseconds(1);
	}
}

void nrf_driver_start_pairing(int ms, PACKET_STATE_t * phandle) {
	if (ext_nrf) {
		pairing_time_end = chVTGetSystemTimeX() + MS2ST(ms);

		if (!pairing_active) {
			pairing_active = true;

			uint8_t send_buffer[PACKET_SIZE(5)];
			uint8_t * data = send_buffer + PACKET_HEADER;

			data[0] = COMM_EXT_NRF_ESB_SET_CH_ADDR;
			data[1] = 67;
			data[2] = 0xC6;
			data[3] = 0xC5;
			data[4] = 0x0;
			packet_send_packet(send_buffer, 5, phandle);
		}
	}
}

static int rf_tx_wrapper(char *data, int len, PACKET_STATE_t * phandle) {
	int res = 0;

	if (ext_nrf) {
		uint8_t send_buffer[PACKET_SIZE(len + 1)];
		uint8_t * buffer = send_buffer + PACKET_HEADER;
		buffer[0] = COMM_EXT_NRF_ESB_SEND_DATA;
		memcpy(buffer + 1, data, len);
		packet_send_packet(send_buffer, len + 1, phandle);
	}

	return res;
}

static THD_FUNCTION(tx_thread, arg){

	PACKET_STATE_t * phandle = arg;

	chRegSetThreadName("Nrf TX");
	tx_running = true;

	for(;;) {
		if (tx_stop) {
			tx_running = false;
			vTaskDelete(NULL);
			vTaskDelay(portMAX_DELAY);
		}

		nosend_cnt++;

		if (nosend_cnt >= ALIVE_INTERVAL && !pairing_active) {
			uint8_t pl[18];
			int32_t index = 0;
			static uint8_t seq_cnt = 0;
			seq_cnt++;

			setup_values val = mc_interface_get_setup_values();
			float wh_left = 0;

			pl[index++] = MOTE_PACKET_ALIVE;
			buffer_append_float16(pl, mc_interface_get_battery_level(&wh_left), 1e3, &index);
			buffer_append_float32(pl, mc_interface_get_speed(), 1e3, &index);
			buffer_append_float32(pl, mc_interface_get_distance_abs(), 1e3, &index);
			buffer_append_float16(pl, mc_interface_temp_fet_filtered(), 1e1, &index);
			buffer_append_float16(pl, mc_interface_temp_motor_filtered(), 1e1, &index);
			pl[index++] = seq_cnt;
			buffer_append_float32(pl, wh_left, 1e3, &index);
			buffer_append_float32(pl, val.wh_tot, 1e4, &index);
			buffer_append_float32(pl, val.wh_charge_tot, 1e4, &index);
			pl[index++] = (uint8_t)((int8_t)(mc_interface_get_tot_current_directional_filtered() /
					(mc_interface_get_configuration()->l_current_max *
							mc_interface_get_configuration()->l_current_max_scale) * 100.0));

			if (driver_paused == 0) {
				rf_tx_wrapper((char*)pl, index, phandle);
			}

			nosend_cnt = 0;
		}

		if (driver_paused > 0) {
			driver_paused--;
		}

		if (chVTGetSystemTimeX() > pairing_time_end && pairing_active) {
			pairing_active = false;

			if (ext_nrf) {
				const app_configuration *appconf_ptr = app_get_configuration();
				uint8_t send_buffer[PACKET_SIZE(5)];
				uint8_t * data = send_buffer + PACKET_HEADER;
				data[0] = COMM_EXT_NRF_ESB_SET_CH_ADDR;
				data[1] = appconf_ptr->app_nrf_conf.channel;
				data[2] = appconf_ptr->app_nrf_conf.address[0];
				data[3] = appconf_ptr->app_nrf_conf.address[1];
				data[4] = appconf_ptr->app_nrf_conf.address[2];
				packet_send_packet(send_buffer, 5, phandle);
			}

			uint8_t send_buffer[PACKET_SIZE(5)];
			uint8_t * data = send_buffer + PACKET_HEADER;
			data[0] = COMM_NRF_START_PAIRING;
			data[1] = NRF_PAIR_FAIL;
			packet_send_packet(send_buffer, 2, phandle);
		}

		chThdSleepMilliseconds(1);
	}

}

void nrf_driver_process_packet(unsigned char *buf, unsigned char len, PACKET_STATE_t * phandle) {
	MOTE_PACKET packet = buf[0];
	chuck_data cdata;
	int32_t ind = 0;
	int buttons;

	nrf_restart_rx_time = NRF_RESTART_TIMEOUT;

	switch (packet) {
	case MOTE_PACKET_BATT_LEVEL:
		// TODO!
		break;

	case MOTE_PACKET_BUTTONS:
		ind = 1;
		mstate.js_x = buf[ind++];
		mstate.js_y = buf[ind++];
		buttons = buf[ind++];
		mstate.bt_c = buttons & (1 << 0);
		mstate.bt_z = buttons & (1 << 1);
		mstate.bt_push = buttons & (1 << 2);
		mstate.rev_has_state = buttons & (1 << 3);
		mstate.is_rev = buttons & (1 << 4);
		mstate.vbat = (float)buffer_get_int16(buf, &ind) / 1000.0;

		cdata.js_x = 255 - mstate.js_x;
		cdata.js_y = mstate.js_y;
		cdata.bt_c = mstate.bt_c;
		cdata.bt_z = mstate.bt_z;
		cdata.rev_has_state = mstate.rev_has_state;
		cdata.is_rev = mstate.is_rev;

		app_nunchuk_update_output(&cdata);
		break;

	case MOTE_PACKET_FILL_RX_BUFFER:
		memcpy(rx_buffer + buf[1], buf + 2, len - 2);
		break;

	case MOTE_PACKET_FILL_RX_BUFFER_LONG: {
		int rxbuf_ind = (unsigned int)buf[1] << 8;
		rxbuf_ind |= buf[2];
		if (rxbuf_ind < RX_BUFFER_SIZE) {
			memcpy(rx_buffer + rxbuf_ind, buf + 3, len - 3);
		}
	}
	break;

	case MOTE_PACKET_PAIRING_INFO: {
		ind = 1;

		if (!pairing_active) {
			break;
		}

		pairing_active = false;

		app_configuration appconf = *app_get_configuration();
		appconf.app_nrf_conf.address[0] = buf[ind++];
		appconf.app_nrf_conf.address[1] = buf[ind++];
		appconf.app_nrf_conf.address[2] = buf[ind++];
		appconf.app_nrf_conf.channel = buf[ind++];
		appconf.app_nrf_conf.crc_type = NRF_CRC_1B;
		appconf.app_nrf_conf.retries = 3;
		appconf.app_nrf_conf.retry_delay = NRF_RETR_DELAY_1000US;
		appconf.app_nrf_conf.send_crc_ack = true;
		appconf.app_nrf_conf.speed = NRF_SPEED_1M;

		if (ext_nrf) {
			uint8_t send_buffer[PACKET_SIZE(5)];
			uint8_t * data = send_buffer + PACKET_HEADER;
			data[0] = COMM_EXT_NRF_ESB_SET_CH_ADDR;
			data[1] = appconf.app_nrf_conf.channel;
			data[2] = appconf.app_nrf_conf.address[0];
			data[3] = appconf.app_nrf_conf.address[1];
			data[4] = appconf.app_nrf_conf.address[2];
			packet_send_packet(send_buffer, 5, phandle);
		}

		appconf.app_chuk_conf.ctrl_type = CHUK_CTRL_TYPE_CURRENT;

		from_nrf = true;
		conf_general_store_app_configuration(&appconf);
		app_set_configuration(&appconf);

		commands_send_appconf(COMM_GET_APPCONF, &appconf, phandle);

		uint8_t send_buffer[PACKET_SIZE(5)];
		uint8_t * data = send_buffer + PACKET_HEADER;
		data[0] = COMM_NRF_START_PAIRING;
		data[1] = NRF_PAIR_OK;
		packet_send_packet(send_buffer, 2, phandle);

		from_nrf = false;
	} break;

	default:
		break;
	}
}

bool nrf_driver_is_pairing(void) {
	return pairing_active;
}

bool nrf_driver_ext_nrf_running(void) {
	return ext_nrf;
}

void nrf_driver_pause(int ms) {
	driver_paused = ms;
}
