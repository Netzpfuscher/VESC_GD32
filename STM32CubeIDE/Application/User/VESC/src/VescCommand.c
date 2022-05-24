/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

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

    Ported to GD32F303 and FreeRTOS 2022 Jens Kerrinnes
 */

#include "VescCommand.h"
#include "defines.h"
#include "buffer.h"
#include "packet.h"
#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "confgenerator.h"
#include <math.h>
#include <mc_interface.h>
#include "utils.h"
#include "system.h"
#include "conf_general.h"
#include "stdarg.h"
#include <printf.h>
#include "terminal.h"
#include "product.h"
#include "app.h"
#include "ninebot.h"
#include "mcpwm_foc.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "mempools.h"


static void(* volatile send_func)(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle) = 0;
static volatile int fw_version_sent_cnt = 0;
static disp_pos_mode display_position_mode;

void command_block_call( void * pvParameter1, uint32_t arg );


#define PRINTF_STACK_SIZE 128u
void commands_printf(PACKET_STATE_t * phandle, const char* format, ...) {
	if(phandle==NULL) return;
	va_list arg;
	va_start (arg, format);
	int len;
	uint8_t send_buffer[PACKET_SIZE(PRINTF_STACK_SIZE)];
	uint8_t * print_buffer = send_buffer + PACKET_HEADER;

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf((char*)print_buffer + 1, PRINTF_STACK_SIZE - 1, format, arg);
	va_end (arg);

	if(len > 0) {
		packet_send_packet((unsigned char*)send_buffer, (len < 254) ? len + 1 : 255, phandle);
	}
}


void commands_send_mcconf(COMM_PACKET_ID packet_id, mc_configuration *mcconf, PACKET_STATE_t * phandle) {
	uint8_t * send_buffer = pvPortMalloc(512 + PACKET_HEADER);
	if(send_buffer == NULL){
		commands_printf(phandle, "Malloc failed send mcconf)");
		return;
	}
	memset(send_buffer,0,512 + PACKET_HEADER);
	uint8_t * buffer = send_buffer + PACKET_HEADER;
	buffer[0] = packet_id;
	int32_t len = confgenerator_serialize_mcconf(&buffer[1], mcconf);
	packet_send_packet(send_buffer, len + 1, phandle);
	vPortFree(send_buffer);
}

void commands_send_appconf(COMM_PACKET_ID packet_id, app_configuration *appconf, PACKET_STATE_t * phandle) {
	uint8_t * send_buffer = pvPortMalloc(512 + PACKET_HEADER);
	if(send_buffer == NULL){
		commands_printf(phandle, "Malloc failed send appconf)");
		return;
	}
	memset(send_buffer,0,512 + PACKET_HEADER);
	uint8_t * buffer = send_buffer + PACKET_HEADER;
	buffer[0] = packet_id;
	int32_t len = confgenerator_serialize_appconf(&buffer[1], appconf);
	packet_send_packet(send_buffer, len + 1, phandle);
	vPortFree(send_buffer);
}


void commands_send_rotor_pos(PACKET_STATE_t * phandle, float rotor_pos) {
	uint8_t send_buffer[PACKET_SIZE(10)];
	uint8_t * buffer = send_buffer + PACKET_HEADER;
	int32_t index = 0;
	buffer[index++] = COMM_ROTOR_POSITION;
	buffer_append_int32(buffer, (int32_t)(rotor_pos * 100000.0), &index);
	packet_send_packet(send_buffer, index, phandle);
}

void send_position(PACKET_STATE_t * phandle){
	switch (display_position_mode) {
	case DISP_POS_MODE_ENCODER:
		if(phandle->port->half_duplex==true && (xTaskGetTickCount() % 100)) break;
		commands_send_rotor_pos(phandle, mc_interface_get_pid_pos_now());

		break;
//	case DISP_POS_MODE_OBSERVER:
//
//		break;
//
//	case DISP_POS_MODE_PID_POS_ERROR:
//		//commands_send_rotor_pos(utils_angle_difference(mc_interface_get_pid_pos_set(), mc_interface_get_pid_pos_now()));
//		break;
//
	default:
		break;
	}
}


const uint8_t test[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
uint8_t VescToSTM_get_uid(uint8_t * ptr, uint8_t size){
	if(size>12) size=12;
	memcpy(ptr, test, size);
	return size;
}

typedef struct __PACKET_COPY_t__ PACKET_COPY_t;

struct __PACKET_COPY_t__{
	PACKET_STATE_t * phandle;
	uint8_t * data;
	uint32_t len;
};

PACKET_COPY_t * commands_alloc_packet_copy(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle){
	PACKET_COPY_t * ret = pvPortMalloc(sizeof(PACKET_COPY_t));
	if(ret == NULL) return NULL;
	ret->len = len;
	if(len){
		ret->data = pvPortMalloc(ret->len);
		if(ret->data == NULL){
			vPortFree(ret);
			return NULL;
		}
		memcpy(ret->data, data, ret->len);
	}else{
		ret->data=NULL;
	}
	ret->phandle = phandle;
	return ret;
}

void commands_free_packet_copy(PACKET_COPY_t * packet){
	vPortFree(packet->data);
	vPortFree(packet);
}

void commands_process_packet(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle) {

	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;


	//send_func = reply_func;

	// Avoid calling invalid function pointer if it is null.
	// commands_send_packet will make the check.
//	if (!reply_func) {
//		reply_func = commands_send_packet;
//	}

	switch (packet_id) {
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[PACKET_SIZE(60)];
		uint8_t * buffer = send_buffer + PACKET_HEADER;
		buffer[ind++] = COMM_FW_VERSION;
		buffer[ind++] = FW_VERSION_MAJOR;
		buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char*)(buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

		ind += VescToSTM_get_uid(buffer + ind, 12);

		buffer[ind++] = appconf.pairing_done;
		buffer[ind++] = FW_TEST_VERSION_NUMBER;

		buffer[ind++] = HW_TYPE_VESC;

		buffer[ind++] = 0; // No custom config
		buffer[ind++] = 0; //No phase filters
		buffer[ind++] = 0;
		buffer[ind++] = 0;

		fw_version_sent_cnt++;

		packet_send_packet(send_buffer, ind, phandle);
		packet_send_packet(send_buffer, ind, phandle);
		} break;

		case COMM_JUMP_TO_BOOTLOADER_ALL_CAN:
		case COMM_JUMP_TO_BOOTLOADER:
			break;
		case COMM_ERASE_NEW_APP_ALL_CAN:
		case COMM_ERASE_NEW_APP: {
			int32_t ind = 0;
			uint8_t send_buffer[PACKET_SIZE(10)];
			uint8_t * buffer = send_buffer + PACKET_HEADER;
			buffer[ind++] = COMM_ERASE_NEW_APP;
			buffer[ind++] = 1;
			packet_send_packet(send_buffer, ind, phandle);
		} break;
		case COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO:
		case COMM_WRITE_NEW_APP_DATA_ALL_CAN:
		case COMM_WRITE_NEW_APP_DATA_LZO:
		case COMM_WRITE_NEW_APP_DATA: {
			int32_t ind = 0;
			uint8_t send_buffer[PACKET_SIZE(16)];
			uint8_t * buffer = send_buffer + PACKET_HEADER;
			buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
			buffer[ind++] = 1;
			//buffer_append_uint32(send_buffer, new_app_offset, &ind);
			buffer_append_uint32(buffer, 0, &ind);
			packet_send_packet(send_buffer, ind, phandle);
		} break;

		case COMM_GET_VALUES:
		case COMM_GET_VALUES_SELECTIVE: {
			int32_t ind = 0;
			uint8_t send_buffer[PACKET_SIZE(80)];
			uint8_t * buffer = send_buffer + PACKET_HEADER;
			buffer[ind++] = packet_id;

			uint32_t mask = 0xFFFFFFFF;
			if (packet_id == COMM_GET_VALUES_SELECTIVE) {
				int32_t ind2 = 0;
				mask = buffer_get_uint32(data, &ind2);
				buffer_append_uint32(buffer, mask, &ind);
			}

			if (mask & ((uint32_t)1 << 0)) {
				buffer_append_float16(buffer, mc_interface_temp_fet_filtered() , 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 1)) {
				buffer_append_float16(buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 2)) {
				buffer_append_float32(buffer, mc_interface_read_reset_avg_motor_current(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 3)) {
				buffer_append_float32(buffer, mc_interface_read_reset_avg_input_current(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 4)) {
				buffer_append_float32(buffer, mc_interface_read_reset_avg_id(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 5)) {
				buffer_append_float32(buffer, mc_interface_read_reset_avg_iq(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 6)) {
				buffer_append_float16(buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind);
			}
			if (mask & ((uint32_t)1 << 7)) {
				buffer_append_float32(buffer, mc_interface_get_rpm(), 1e0, &ind);
			}
			if (mask & ((uint32_t)1 << 8)) {
				buffer_append_float16(buffer, mc_interface_get_input_voltage_filtered(), 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 9)) {
				buffer_append_float32(buffer, mc_interface_get_amp_hours(false), 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 10)) {
				buffer_append_float32(buffer, mc_interface_get_amp_hours_charged(false), 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 11)) {
				buffer_append_float32(buffer, mc_interface_get_watt_hours(false), 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 12)) {
				buffer_append_float32(buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 13)) {
				buffer_append_int32(buffer, mc_interface_get_tachometer_value(false), &ind);
			}
			if (mask & ((uint32_t)1 << 14)) {
				buffer_append_int32(buffer, mc_interface_get_tachometer_abs_value(false), &ind);
			}
			if (mask & ((uint32_t)1 << 15)) {
				buffer[ind++] = mc_interface_get_fault();
			}
			if (mask & ((uint32_t)1 << 16)) {
				buffer_append_float32(buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);
			}
			if (mask & ((uint32_t)1 << 17)) {
				uint8_t current_controller_id = app_get_configuration()->controller_id;
				buffer[ind++] = current_controller_id;
			}
			if (mask & ((uint32_t)1 << 18)) {
				buffer_append_float16(buffer, 0, 1e1, &ind);
				buffer_append_float16(buffer, 0, 1e1, &ind);
				buffer_append_float16(buffer, 0, 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 19)) {
				buffer_append_float32(buffer, mc_interface_read_reset_avg_vd(), 1e3, &ind);
			}
			if (mask & ((uint32_t)1 << 20)) {
				buffer_append_float32(buffer, mc_interface_read_reset_avg_vq(), 1e3, &ind);
			}
			if (mask & ((uint32_t)1 << 21)) {
				uint8_t status = 0;
				status |= timeout_has_timeout();
				status |= timeout_kill_sw_active() << 1;
				buffer[ind++] = status;
			}

			packet_send_packet(send_buffer, ind, phandle);
		} break;

			case COMM_SET_DUTY: {
				int32_t ind = 0;
				mc_interface_set_duty((float)buffer_get_int32(data, &ind) / 100000.0);
				timeout_reset();
			} break;

			case COMM_SET_CURRENT: {
				int32_t ind = 0;
				mc_interface_set_current((float)buffer_get_int32(data, &ind) / 1000.0);
				timeout_reset();
			} break;

			case COMM_SET_CURRENT_BRAKE: {
				int32_t ind = 0;
				mc_interface_set_brake_current((float)buffer_get_int32(data, &ind) / 1000.0);
				timeout_reset();
			} break;

			case COMM_SET_RPM: {
				int32_t ind = 0;
				mc_interface_set_pid_speed((float)buffer_get_int32(data, &ind));
				timeout_reset();
			} break;

			case COMM_SET_POS: {
				int32_t ind = 0;
				mc_interface_set_pid_pos((float)buffer_get_int32(data, &ind) / 1000000.0);
				timeout_reset();
			} break;

			case COMM_SET_HANDBRAKE: {
				int32_t ind = 0;
				mc_interface_set_handbrake(buffer_get_float32(data, 1e3, &ind));
				timeout_reset();
			} break;

			case COMM_SET_DETECT: {
				int32_t ind = 0;
				display_position_mode = data[ind++];
				/*
				if (mc_interface_get_configuration()->motor_type == MOTOR_TYPE_BLDC) {
					if (display_position_mode == DISP_POS_MODE_NONE) {
						mc_interface_release_motor();
					} else if (display_position_mode == DISP_POS_MODE_INDUCTANCE) {
						mcpwm_set_detect();
					}
				}*/

				//VescToSTM_timeout_reset();
			} break;

			case COMM_SET_SERVO_POS:
				break;

			case COMM_SET_MCCONF: {
				mc_configuration *mcconf = mempools_alloc_mcconf();
				*mcconf = *mc_interface_get_configuration();


				if (confgenerator_deserialize_mcconf(data, mcconf)) {

					conf_general_mcconf_hw_limits(mcconf);
					mc_interface_set_configuration(mcconf);
					conf_general_store_mc_configuration(mcconf, 0);

					vTaskDelay(100);

					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(10)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = packet_id;
					packet_send_packet(send_buffer, ind, phandle);
				} else {
					//commands_printf("Warning: Could not set mcconf due to wrong signature");
				}

				int32_t ind = 0;
				uint8_t send_buffer[PACKET_SIZE(10)];
				uint8_t * buffer = send_buffer + PACKET_HEADER;
				buffer[ind++] = packet_id;
				packet_send_packet(send_buffer, ind, phandle);

				mempools_free_mcconf(mcconf);
				} break;

				case COMM_GET_MCCONF:
				case COMM_GET_MCCONF_DEFAULT: {
					mc_configuration *mcconf = mempools_alloc_mcconf();
					if(mcconf == NULL){
						commands_printf(phandle, "Malloc failed get mcconf");
						return;
					}

					if (packet_id == COMM_GET_MCCONF) {
						*mcconf = *mc_interface_get_configuration();
					} else {
						confgenerator_set_defaults_mcconf(mcconf);
					}
					commands_send_mcconf(packet_id, mcconf, phandle);
					mempools_free_mcconf(mcconf);
				} break;

				case COMM_SET_APPCONF:{
					app_configuration *appconf = app_get_configuration();

					if (confgenerator_deserialize_appconf(data, appconf)) {
						conf_general_store_app_configuration(appconf);
						app_set_configuration(appconf);
						//timeout_configure(appconf->timeout_msec, appconf->timeout_brake_current);
						vTaskDelay(pdMS_TO_TICKS(200));

						int32_t ind = 0;
						uint8_t send_buffer[50];
						send_buffer[ind++] = packet_id;
						packet_send_packet(send_buffer, ind, phandle);
					} else {
						commands_printf(phandle, "Warning: Could not set appconf due to wrong signature");
					}
				}
					break;
				case COMM_GET_APPCONF:
				case COMM_GET_APPCONF_DEFAULT: {

					app_configuration *appconf = mempools_alloc_appconf();

					if (packet_id == COMM_GET_APPCONF) {
						*appconf = *app_get_configuration();
					} else {
						confgenerator_set_defaults_appconf(appconf);
					}

					commands_send_appconf(packet_id, appconf, phandle);

					mempools_free_appconf(appconf);
				}
				break;

				case COMM_SAMPLE_PRINT: {
					uint16_t sample_len;
					uint8_t decimation;
					debug_sampling_mode mode;

					int32_t ind = 0;
					mode = data[ind++];
					sample_len = buffer_get_uint16(data, &ind);
					decimation = data[ind++];

					bool raw = false;
					if (len > (uint32_t)ind) {
						raw = data[ind++];
					}

					mc_interface_sample_print_data(mode, sample_len, decimation, raw, phandle);
				} break;

				case COMM_REBOOT:
					NVIC_SystemReset();
					break;

				case COMM_ALIVE:
					//SHUTDOWN_RESET();
					timeout_reset();
					break;

				case COMM_GET_DECODED_PPM: {
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = COMM_GET_DECODED_PPM;
					//buffer_append_int32(send_buffer, (int32_t)(app_ppm_get_decoded_level() * 1000000.0), &ind);
					buffer_append_int32(buffer, 0, &ind);
					//buffer_append_int32(send_buffer, (int32_t)(servodec_get_last_pulse_len(0) * 1000000.0), &ind);
					buffer_append_int32(buffer, 0, &ind);
					packet_send_packet(send_buffer, ind, phandle);

				} break;

				case COMM_GET_DECODED_ADC: {

					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = COMM_GET_DECODED_ADC;
					buffer_append_int32(buffer, app_adc_get_decoded_level() * 1000000.0, &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage() * 1000000.0), &ind);
					buffer_append_int32(buffer, 0 * 1000000.0, &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_decoded_level2() * 1000000.0), &ind);
					buffer_append_int32(buffer, app_adc_get_decoded_level2() * 1000000.0, &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage2() * 1000000.0), &ind);
					buffer_append_int32(buffer, 0 * 1000000.0, &ind);
					packet_send_packet(send_buffer, ind, phandle);
				} break;

				case COMM_GET_DECODED_CHUK: {
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = COMM_GET_DECODED_CHUK;
					buffer_append_int32(buffer, (int32_t)(0 * 1000000.0), &ind);
					packet_send_packet(send_buffer, ind, phandle);
				} break;

				case COMM_GET_DECODED_BALANCE: {
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(50)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = COMM_GET_DECODED_BALANCE;
					memset(buffer,0,36);
					ind+=36;
					packet_send_packet(send_buffer, ind, phandle);
				} break;

				case COMM_FORWARD_CAN:
					break;

				case COMM_SET_CHUCK_DATA:{
					chuck_data chuck_d_tmp;

					int32_t ind = 0;
					chuck_d_tmp.js_x = data[ind++];
					chuck_d_tmp.js_y = data[ind++];
					chuck_d_tmp.bt_c = data[ind++];
					chuck_d_tmp.bt_z = data[ind++];
					chuck_d_tmp.acc_x = buffer_get_int16(data, &ind);
					chuck_d_tmp.acc_y = buffer_get_int16(data, &ind);
					chuck_d_tmp.acc_z = buffer_get_int16(data, &ind);

					if (len >= (unsigned int)ind + 2) {
						chuck_d_tmp.rev_has_state = data[ind++];
						chuck_d_tmp.is_rev = data[ind++];
					} else {
						chuck_d_tmp.rev_has_state = false;
						chuck_d_tmp.is_rev = false;
					}
					//VescToStm_nunchuk_update_output(&chuck_d_tmp);
				} break;

				case COMM_CUSTOM_APP_DATA:
					break;

				case COMM_NRF_START_PAIRING: {
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = packet_id;
					buffer[ind++] = NRF_PAIR_STARTED;
					packet_send_packet(send_buffer, ind, phandle);
				} break;

				case COMM_GPD_BUFFER_SIZE_LEFT: {
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = COMM_GPD_BUFFER_SIZE_LEFT;
					//buffer_append_int32(send_buffer, gpdrive_buffer_size_left(), &ind);
					buffer_append_int32(buffer, 128, &ind);
					packet_send_packet(buffer, ind, phandle);
				} break;
				case COMM_GPD_SET_FSW:
				case COMM_GPD_FILL_BUFFER:
				case COMM_GPD_OUTPUT_SAMPLE:
				case COMM_GPD_SET_MODE:
				case COMM_GPD_FILL_BUFFER_INT8:
				case COMM_GPD_FILL_BUFFER_INT16:
				case COMM_GPD_SET_BUFFER_INT_SCALE:
				break;

				case COMM_GET_VALUES_SETUP:
				case COMM_GET_VALUES_SETUP_SELECTIVE: {
					setup_values val = mc_interface_get_setup_values();
					val.ah_charge_tot = 0;
					val.ah_tot = 0;
					val.current_in_tot = 0;
					val.current_tot = 0;
					val.num_vescs = 1;
					val.wh_charge_tot = 0;
					val.wh_tot = 0;

					float wh_batt_left = 1.0;
					float battery_level = 0.0;//TODO VescToSTM_get_battery_level(&wh_batt_left);

					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(80)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = packet_id;

					uint32_t mask = 0xFFFFFFFF;
					if (packet_id == COMM_GET_VALUES_SETUP_SELECTIVE) {
						int32_t ind2 = 0;
						mask = buffer_get_uint32(data, &ind2);
						buffer_append_uint32(buffer, mask, &ind);
					}

					if (mask & ((uint32_t)1 << 0)) {
						buffer_append_float16(buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
					}
					if (mask & ((uint32_t)1 << 1)) {
						buffer_append_float16(buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
					}
					if (mask & ((uint32_t)1 << 2)) {
						buffer_append_float32(buffer, val.current_tot, 1e2, &ind);
					}
					if (mask & ((uint32_t)1 << 3)) {
						buffer_append_float32(buffer, val.current_in_tot, 1e2, &ind);
					}
					if (mask & ((uint32_t)1 << 4)) {
						buffer_append_float16(buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 5)) {
						buffer_append_float32(buffer, mc_interface_get_rpm(), 1e0, &ind);
					}
					if (mask & ((uint32_t)1 << 6)) {
						buffer_append_float32(buffer, mc_interface_get_speed(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 7)) {
						buffer_append_float16(buffer, mc_interface_get_input_voltage_filtered(), 1e1, &ind);
					}
					if (mask & ((uint32_t)1 << 8)) {
						buffer_append_float16(buffer, battery_level, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 9)) {
						buffer_append_float32(buffer, val.ah_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 10)) {
						buffer_append_float32(buffer, val.ah_charge_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 11)) {
						buffer_append_float32(buffer, val.wh_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 12)) {
						buffer_append_float32(buffer, val.wh_charge_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 13)) {
						buffer_append_float32(buffer, mc_interface_get_distance(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 14)) {
						buffer_append_float32(buffer, mc_interface_get_distance_abs(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 15)) {
						buffer_append_float32(buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);
					}
					if (mask & ((uint32_t)1 << 16)) {
						buffer[ind++] = mc_interface_get_fault();
					}
					if (mask & ((uint32_t)1 << 17)) {
						uint8_t current_controller_id = app_get_configuration()->controller_id;
						buffer[ind++] = current_controller_id;
					}
					if (mask & ((uint32_t)1 << 18)) {
						buffer[ind++] = val.num_vescs;
					}
					if (mask & ((uint32_t)1 << 19)) {
						buffer_append_float32(buffer, wh_batt_left, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 20)) {
						buffer_append_uint32(buffer, mc_interface_get_odometer(), &ind);
					}
					if (mask & ((uint32_t)1 << 21)) {
						//buffer_append_uint32(send_buffer, chVTGetSystemTimeX() / (CH_CFG_ST_FREQUENCY / 1000), &ind);
						buffer_append_uint32(buffer, xTaskGetTickCount()/2, &ind);
					}

					packet_send_packet(send_buffer, ind, phandle);
				    } break;

				case COMM_SET_ODOMETER: {
					int32_t ind = 0;
					mc_interface_set_odometer(buffer_get_uint32(data, &ind));
					//TODO timeout_reset();
				} break;

				case COMM_SET_MCCONF_TEMP:
				case COMM_SET_MCCONF_TEMP_SETUP: {
					mc_configuration *mcconf = mempools_alloc_mcconf();
					*mcconf = *mc_interface_get_configuration();

					int32_t ind = 0;
					bool store = data[ind++];
					bool forward_can = data[ind++];
					bool ack = data[ind++];
					bool divide_by_controllers = data[ind++];
					UNUSED(store);UNUSED(forward_can);UNUSED(divide_by_controllers);
					float controller_num = 1.0;

					mcconf->l_current_min_scale = buffer_get_float32_auto(data, &ind);
					mcconf->l_current_max_scale = buffer_get_float32_auto(data, &ind);

					if (packet_id == COMM_SET_MCCONF_TEMP_SETUP) {
						const float fact = ((mcconf->si_motor_poles / 2.0) * 60.0 *
								mcconf->si_gear_ratio) / (mcconf->si_wheel_diameter * M_PI);

						mcconf->l_min_erpm = buffer_get_float32_auto(data, &ind) * fact;
						mcconf->l_max_erpm = buffer_get_float32_auto(data, &ind) * fact;

						// Write computed RPM back and change forwarded packet id to
						// COMM_SET_MCCONF_TEMP. This way only the master has to be
						// aware of the setup information.
						ind -= 8;
						buffer_append_float32_auto(data, mcconf->l_min_erpm, &ind);
						buffer_append_float32_auto(data, mcconf->l_max_erpm, &ind);
					} else {
						mcconf->l_min_erpm = buffer_get_float32_auto(data, &ind);
						mcconf->l_max_erpm = buffer_get_float32_auto(data, &ind);
					}


					mcconf->l_min_duty = buffer_get_float32_auto(data, &ind);
					mcconf->l_max_duty = buffer_get_float32_auto(data, &ind);
					mcconf->l_watt_min = buffer_get_float32_auto(data, &ind) / controller_num;
					mcconf->l_watt_max = buffer_get_float32_auto(data, &ind) / controller_num;

					// Write divided data back to the buffer, as the other controllers have no way to tell
					// how many controllers are on the bus and thus need pre-divided data.
					// We set divide by controllers to false before forwarding.
					ind -= 8;
					buffer_append_float32_auto(data, mcconf->l_watt_min, &ind);
					buffer_append_float32_auto(data, mcconf->l_watt_max, &ind);

					// Battery limits can be set optionally in a backwards-compatible way.
					if ((int32_t)len >= (ind + 8)) {
						mcconf->l_in_current_min = buffer_get_float32_auto(data, &ind);
						mcconf->l_in_current_max = buffer_get_float32_auto(data, &ind);
					}

					mcconf->lo_current_min = mcconf->l_current_min * mcconf->l_current_min_scale;
					mcconf->lo_current_max = mcconf->l_current_max * mcconf->l_current_max_scale;
					mcconf->lo_current_motor_min_now = mcconf->lo_current_min;
					mcconf->lo_current_motor_max_now = mcconf->lo_current_max;
					mcconf->lo_in_current_min = mcconf->l_in_current_min;
					mcconf->lo_in_current_max = mcconf->l_in_current_max;

					if (ack) {
						ind = 0;
						uint8_t send_buffer[PACKET_SIZE(20)];
						uint8_t * buffer = send_buffer + PACKET_HEADER;
						buffer[ind++] = packet_id;
						packet_send_packet(send_buffer, ind, phandle);
					}
					mempools_free_mcconf(mcconf);

				} break;

				case COMM_GET_MCCONF_TEMP: {
					mc_configuration *mcconf =mempools_alloc_mcconf();
					*mcconf = *mc_interface_get_configuration();
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;

					buffer[ind++] = packet_id;
					buffer_append_float32_auto(buffer, mcconf->l_current_min_scale, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_current_max_scale, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_min_erpm, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_max_erpm, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_min_duty, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_max_duty, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_watt_min, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_watt_max, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_in_current_min, &ind);
					buffer_append_float32_auto(buffer, mcconf->l_in_current_max, &ind);
					// Setup config needed for speed calculation
					buffer[ind++] = (uint8_t)mcconf->si_motor_poles;
					buffer_append_float32_auto(buffer, mcconf->si_gear_ratio, &ind);
					buffer_append_float32_auto(buffer, mcconf->si_wheel_diameter, &ind);

					packet_send_packet(send_buffer, ind, phandle);
					mempools_free_mcconf(mcconf);
				} break;

				case COMM_EXT_NRF_PRESENT:
				case COMM_EXT_NRF_ESB_RX_DATA:
				case COMM_APP_DISABLE_OUTPUT:{
					int32_t ind = 0;
					bool fwd_can = data[ind++];
					int time = buffer_get_int32(data, &ind);
					app_disable_output(time);

					if (fwd_can) {
						data[0] = 0; // Don't continue forwarding
						uint8_t send_buffer[PACKET_SIZE(20)];
						uint8_t * buffer = send_buffer + PACKET_HEADER;
						buffer[ind++] = packet_id;
						packet_send_packet(send_buffer, ind, phandle);
						//comm_can_send_buffer(255, data - 1, len + 1, 0);
					}
				}break;

				case COMM_TERMINAL_CMD_SYNC:
					data[len] = '\0';
					terminal_process_string((char*)data, phandle);
					break;

				case COMM_GET_IMU_DATA: {
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(70)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = packet_id;

					int32_t ind2 = 0;
					uint32_t mask = buffer_get_uint16(data, &ind2);

					buffer_append_uint16(buffer, mask, &ind);

					for(uint8_t i=0;i<16;i++){
						if (mask & ((uint32_t)1 << i)) {
							buffer_append_float32_auto(buffer, 0, &ind);
						}
					}

					packet_send_packet(send_buffer, ind, phandle);
				} break;

				case COMM_ERASE_BOOTLOADER_ALL_CAN:
				case COMM_ERASE_BOOTLOADER: {
					int32_t ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = COMM_ERASE_BOOTLOADER;
					buffer[ind++] = 1;
					packet_send_packet(send_buffer, ind, phandle);
				} break;

				case COMM_SET_CURRENT_REL: {
					int32_t ind = 0;
					mc_interface_set_current_rel(buffer_get_float32(data, 1e5, &ind));
					//TODO timeout_reset();
				} break;

				case COMM_CAN_FWD_FRAME:
					break;
				case COMM_SET_BATTERY_CUT: {
					int32_t ind = 0;
					float start = buffer_get_float32(data, 1e3, &ind);
					float end = buffer_get_float32(data, 1e3, &ind);
					bool store = data[ind++];
					bool fwd_can = data[ind++];
					UNUSED(store);UNUSED(fwd_can);

					/*if (fwd_can) {
						comm_can_conf_battery_cut(255, store, start, end);
					}*/

					mc_configuration *mcconf = mempools_alloc_mcconf();
					*mcconf = *mc_interface_get_configuration();

					if (mcconf->l_battery_cut_start != start || mcconf->l_battery_cut_end != end) {
						mcconf->l_battery_cut_start = start;
						mcconf->l_battery_cut_end = end;

						/*if (store) {
							conf_general_store_mc_configuration(mcconf,
									mc_interface_get_motor_thread() == 2);
						}*/

						//mc_interface_set_configuration(mcconf);
					}
					mempools_free_mcconf(mcconf);

					// Send ack
					ind = 0;
					uint8_t send_buffer[PACKET_SIZE(20)];
					uint8_t * buffer = send_buffer + PACKET_HEADER;
					buffer[ind++] = packet_id;
					packet_send_packet(send_buffer, ind, phandle);
				} break;

				case COMM_SET_CAN_MODE: {
					int32_t ind = 0;
					bool ack = data[ind++];
					if (ack) {
						ind = 0;
						uint8_t send_buffer[PACKET_SIZE(20)];
						uint8_t * buffer = send_buffer + PACKET_HEADER;
						buffer[ind++] = packet_id;
						packet_send_packet(send_buffer, ind, phandle);
					}
				} break;

				case COMM_BMS_GET_VALUES:
				case COMM_BMS_SET_CHARGE_ALLOWED:
				case COMM_BMS_SET_BALANCE_OVERRIDE:
				case COMM_BMS_RESET_COUNTERS:
				case COMM_BMS_FORCE_BALANCE:
				case COMM_BMS_ZERO_CURRENT_OFFSET: {
					//bms_process_cmd(data - 1, len + 1);
					break;
				}

				// Blocking commands. Only one of them runs at any given time, in their
				// own thread. If other blocking commands come before the previous one has
				// finished, they are discarded.
				case COMM_TERMINAL_CMD:
				case COMM_DETECT_MOTOR_PARAM:
				case COMM_DETECT_MOTOR_R_L:
				case COMM_DETECT_MOTOR_FLUX_LINKAGE:
				case COMM_DETECT_ENCODER:
				case COMM_DETECT_HALL_FOC:
				case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
				case COMM_PING_CAN:
				case COMM_DETECT_APPLY_ALL_FOC:
				case COMM_BM_CONNECT:
				case COMM_BM_ERASE_FLASH_ALL:
				case COMM_BM_WRITE_FLASH_LZO:
				case COMM_BM_WRITE_FLASH:
				case COMM_BM_REBOOT:
				case COMM_BM_DISCONNECT:
				case COMM_BM_MAP_PINS_DEFAULT:
				case COMM_BM_MAP_PINS_NRF5X:
				case COMM_BM_MEM_READ:
				case COMM_GET_IMU_CALIBRATION: {
					PACKET_COPY_t * temp = commands_alloc_packet_copy(data, len, phandle);
					if(temp == NULL){
						commands_printf(phandle, "Malloc failed blocking call)");
					}else{
						xTimerPendFunctionCall(command_block_call, temp, packet_id, MS_TO_TICKS(200));
					}
					/*
					if (!is_blocking) {
						memcpy(blocking_thread_cmd_buffer, data - 1, len + 1);
						blocking_thread_cmd_len = len + 1;
						is_blocking = true;
						blocking_thread_motor = mc_interface_get_motor_thread();
						send_func_blocking = reply_func;
						chEvtSignal(blocking_tp, (eventmask_t)1);
					}*/
					break;
				}
				default:
					break;
				}
}

void command_block_call( void * pvParameter1, uint32_t arg ){
	PACKET_COPY_t * packet = (PACKET_COPY_t*)pvParameter1;
	uint8_t * data = packet->data;
	uint32_t len = packet->len;
	PACKET_STATE_t * phandle = packet->phandle;

//	// Wait for main to finish
//	while(!main_init_done()) {
//		chThdSleepMilliseconds(10);
//	}

	COMM_PACKET_ID packet_id = arg;



	switch (packet_id) {
	case COMM_DETECT_MOTOR_PARAM: {
		uint8_t buffer[PACKET_SIZE(20)];
		uint8_t * send_buffer = buffer + PACKET_HEADER;
		int32_t ind = 0;
		float detect_current = buffer_get_float32(data, 1e3, &ind);
		float detect_min_rpm = buffer_get_float32(data, 1e3, &ind);
		float detect_low_duty = buffer_get_float32(data, 1e3, &ind);
		float detect_cycle_int_limit;
		float detect_coupling_k;
		int8_t detect_hall_table[8];
		int detect_hall_res;

//		if (!conf_general_detect_motor_param(detect_current, detect_min_rpm,
//				detect_low_duty, &detect_cycle_int_limit, &detect_coupling_k,
//				detect_hall_table, &detect_hall_res)) {
//			detect_cycle_int_limit = 0.0;
//			detect_coupling_k = 0.0;
//		}

		ind = 0;
		send_buffer[ind++] = COMM_DETECT_MOTOR_PARAM;
		buffer_append_int32(send_buffer, (int32_t)(detect_cycle_int_limit * 1000.0), &ind);
		buffer_append_int32(send_buffer, (int32_t)(detect_coupling_k * 1000.0), &ind);
		memcpy(send_buffer + ind, detect_hall_table, 8);
		ind += 8;
		send_buffer[ind++] = detect_hall_res;
		packet_send_packet(buffer, ind, phandle);

	} break;

	case COMM_DETECT_MOTOR_R_L: {
		mc_configuration *mcconf = mempools_alloc_mcconf();
		*mcconf = *mc_interface_get_configuration();
		mc_configuration *mcconf_old = mempools_alloc_mcconf();
		*mcconf_old = *mcconf;

		uint8_t buffer[PACKET_SIZE(20)];
		uint8_t * send_buffer = buffer + PACKET_HEADER;

		mcconf->motor_type = MOTOR_TYPE_FOC;
		mc_interface_set_configuration(mcconf);

		float r = 0.0;
		float l = 0.0;
		float ld_lq_diff = 0.0;
		bool res = mcpwm_foc_measure_res_ind(&r, &l, &ld_lq_diff);
		mc_interface_set_configuration(mcconf_old);

		if (!res) {
			r = 0.0;
			l = 0.0;
		}

		int32_t ind = 0;
		send_buffer[ind++] = COMM_DETECT_MOTOR_R_L;
		buffer_append_float32(send_buffer, r, 1e6, &ind);
		buffer_append_float32(send_buffer, l, 1e3, &ind);
		buffer_append_float32(send_buffer, ld_lq_diff, 1e3, &ind);

		packet_send_packet(buffer, ind, phandle);

		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);
	} break;

	case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
	} break;

	case COMM_DETECT_ENCODER: {
	} break;

	case COMM_DETECT_HALL_FOC: {
		mc_configuration *mcconf = mempools_alloc_mcconf();
		*mcconf = *mc_interface_get_configuration();
		uint8_t buffer[PACKET_SIZE(50)];
		uint8_t * send_buffer = buffer + PACKET_HEADER;

		if (mcconf->m_sensor_port_mode == SENSOR_PORT_MODE_HALL) {
			mc_configuration *mcconf_old = mempools_alloc_mcconf();
			*mcconf_old = *mcconf;

			int32_t ind = 0;
			float current = buffer_get_float32(data, 1e3, &ind);

			mcconf->motor_type = MOTOR_TYPE_FOC;
			mcconf->foc_f_zv = 10000.0;
			mcconf->foc_current_kp = 0.01;
			mcconf->foc_current_ki = 10.0;
			mc_interface_set_configuration(mcconf);

			uint8_t hall_tab[8];
			bool res = mcpwm_foc_hall_detect(current, hall_tab);
			mc_interface_set_configuration(mcconf_old);

			ind = 0;
			send_buffer[ind++] = COMM_DETECT_HALL_FOC;
			memcpy(send_buffer + ind, hall_tab, 8);
			ind += 8;
			send_buffer[ind++] = res ? 0 : 1;

			packet_send_packet(buffer, ind, phandle);

			mempools_free_mcconf(mcconf_old);
		} else {
			int32_t ind = 0;
			send_buffer[ind++] = COMM_DETECT_HALL_FOC;
			memset(send_buffer, 255, 8);
			ind += 8;
			send_buffer[ind++] = 0;
			packet_send_packet(buffer, ind, phandle);
		}

		mempools_free_mcconf(mcconf);
	} break;

	case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP: {
		int32_t ind = 0;
		uint8_t buffer[PACKET_SIZE(20)];
		uint8_t * send_buffer = buffer + PACKET_HEADER;
		float current = buffer_get_float32(data, 1e3, &ind);
		float erpm_per_sec = buffer_get_float32(data, 1e3, &ind);
		float duty = buffer_get_float32(data, 1e3, &ind);
		float resistance = buffer_get_float32(data, 1e6, &ind);
		float inductance = 0.0;

		if (len >= (uint32_t)ind + 4) {
			inductance = buffer_get_float32(data, 1e8, &ind);
		}

		float linkage, linkage_undriven, undriven_samples;
		bool res = conf_general_measure_flux_linkage_openloop(current, duty,
				erpm_per_sec, resistance, inductance,
				&linkage, &linkage_undriven, &undriven_samples);

		if (undriven_samples > 60) {
			linkage = linkage_undriven;
		}

		if (!res) {
			linkage = 0.0;
		}

		ind = 0;
		send_buffer[ind++] = COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP;
		buffer_append_float32(send_buffer, linkage, 1e7, &ind);
		packet_send_packet(buffer, ind, phandle);
	} break;

	case COMM_DETECT_APPLY_ALL_FOC: {
		int32_t ind = 0;
		bool detect_can = data[ind++];
		uint8_t buffer[PACKET_SIZE(50)];
		uint8_t * send_buffer = buffer + PACKET_HEADER;
		float max_power_loss = buffer_get_float32(data, 1e3, &ind);
		float min_current_in = buffer_get_float32(data, 1e3, &ind);
		float max_current_in = buffer_get_float32(data, 1e3, &ind);
		float openloop_rpm = buffer_get_float32(data, 1e3, &ind);
		float sl_erpm = buffer_get_float32(data, 1e3, &ind);

		int res = conf_general_detect_apply_all_foc_can(detect_can, max_power_loss, min_current_in, max_current_in, openloop_rpm, sl_erpm, phandle);

		ind = 0;
		send_buffer[ind++] = COMM_DETECT_APPLY_ALL_FOC;
		buffer_append_int16(send_buffer, res, &ind);
		packet_send_packet(buffer, ind, phandle);
	} break;

	case COMM_TERMINAL_CMD:
		data[len] = '\0';
//			chMtxLock(&terminal_mutex);
		terminal_process_string((char*)data, phandle);
//			chMtxUnlock(&terminal_mutex);
		break;

	case COMM_PING_CAN: {
		uint8_t buffer[PACKET_SIZE(28)];
		uint8_t * send_buffer = buffer + PACKET_HEADER;
		int32_t ind = 0;
		send_buffer[ind++] = COMM_PING_CAN;
		//buffer[ind++] = 0; Add one byte for each detected
		packet_send_packet(buffer, ind, phandle);
	} break;

#if HAS_BLACKMAGIC
	case COMM_BM_CONNECT: {
		int32_t ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, bm_connect(), &ind);
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_ERASE_FLASH_ALL: {
		int32_t ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, bm_erase_flash_all(), &ind);
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_WRITE_FLASH_LZO:
	case COMM_BM_WRITE_FLASH: {
		if (packet_id == COMM_BM_WRITE_FLASH_LZO) {
			memcpy(send_buffer, data + 6, len - 6);
			int32_t ind = 4;
			lzo_uint decompressed_len = buffer_get_uint16(data, &ind);
			lzo1x_decompress_safe(send_buffer, len - 6, data + 4, &decompressed_len, NULL);
			len = decompressed_len + 4;
		}

		int32_t ind = 0;
		uint32_t addr = buffer_get_uint32(data, &ind);

		int res = bm_write_flash(addr, data + ind, len - ind);

		ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, res, &ind);
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_REBOOT: {
		int32_t ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, bm_reboot(), &ind);
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_HALT_REQ: {
		bm_halt_req();

		int32_t ind = 0;
		send_buffer[ind++] = packet_id;
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_DISCONNECT: {
		bm_disconnect();
		bm_leave_nrf_debug_mode();

		int32_t ind = 0;
		send_buffer[ind++] = packet_id;
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_MAP_PINS_DEFAULT: {
		bm_default_swd_pins();
		int32_t ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, 1, &ind);
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_MAP_PINS_NRF5X: {
		int32_t ind = 0;
		send_buffer[ind++] = packet_id;

#ifdef NRF5x_SWDIO_GPIO
		buffer_append_int16(send_buffer, 1, &ind);
		bm_change_swd_pins(NRF5x_SWDIO_GPIO, NRF5x_SWDIO_PIN,
				NRF5x_SWCLK_GPIO, NRF5x_SWCLK_PIN);
#else
		buffer_append_int16(send_buffer, 0, &ind);
#endif
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;

	case COMM_BM_MEM_READ: {
		int32_t ind = 0;
		uint32_t addr = buffer_get_uint32(data, &ind);
		uint16_t read_len = buffer_get_uint16(data, &ind);

		if (read_len > sizeof(send_buffer) - 3) {
			read_len = sizeof(send_buffer) - 3;
		}

		int res = bm_mem_read(addr, send_buffer + 3, read_len);

		ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, res, &ind);
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind + read_len);
		}
	} break;

	case COMM_BM_MEM_WRITE: {
		int32_t ind = 0;
		uint32_t addr = buffer_get_uint32(data, &ind);

		int res = bm_mem_write(addr, data + ind, len - ind);

		ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, res, &ind);
		if (send_func_blocking) {
			send_func_blocking(send_buffer, ind);
		}
	} break;
#endif
//		case COMM_GET_IMU_CALIBRATION: {
//			int32_t ind = 0;
//			float yaw = buffer_get_float32(data, 1e3, &ind);
//			float imu_cal[9];
//			imu_get_calibration(yaw, imu_cal);
//
//			ind = 0;
//			send_buffer[ind++] = COMM_GET_IMU_CALIBRATION;
//			buffer_append_float32(send_buffer, imu_cal[0], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[1], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[2], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[3], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[4], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[5], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[6], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[7], 1e6, &ind);
//			buffer_append_float32(send_buffer, imu_cal[8], 1e6, &ind);
//
//			if (send_func_blocking) {
//				send_func_blocking(send_buffer, ind);
//			}
//		} break;

	default:
		break;
	}
	commands_free_packet_copy(packet);
}

