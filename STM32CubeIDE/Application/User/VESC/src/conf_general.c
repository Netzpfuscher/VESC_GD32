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

#include <commands.h>
#include <stdlib.h>
#include "conf_general.h"
#include "utils_math.h"
#include "confgenerator.h"
#include "stm32f1xx_hal.h"
#include "crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include <math.h>
#include <mc_interface.h>
#include "mcconf_default.h"
#include <string.h>
#include "main.h"
#include "mcpwm_foc.h"
#include "mempools.h"
#include "timeout.h"

app_configuration appconf;

int conf_general_autodetect_apply_sensors_foc(float current,
		bool store_mcconf_on_success, bool send_mcconf_on_success) {
	int result = -1;

	mc_configuration *mcconf = mempools_alloc_mcconf();
	mc_configuration *mcconf_old = mempools_alloc_mcconf();

	*mcconf = *mc_interface_get_configuration();
	*mcconf_old = *mcconf;

	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
	mcconf->foc_current_kp = 0.0005;
	mcconf->foc_current_ki = 1.0;
	mc_interface_set_configuration(mcconf);

	// Wait maximum 5s for fault code to disappear
	for (int i = 0;i < 500;i++) {
		if (mc_interface_get_fault() == FAULT_CODE_NONE) {
			break;
		}
		vTaskDelay(MS_TO_TICKS(10));
	}

	if (mc_interface_get_fault() != FAULT_CODE_NONE) {
		mc_interface_set_configuration(mcconf_old);
		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);
		return -1;
	}

	// Wait one second for things to get ready after
	// the fault disappears.
	vTaskDelay(MS_TO_TICKS(1000));

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	mc_interface_lock();

	// Hall sensors
	mcconf->m_sensor_port_mode = SENSOR_PORT_MODE_HALL;
	mc_interface_set_configuration(mcconf);

	uint8_t hall_table[8];
	bool res = mcpwm_foc_hall_detect(current, hall_table);

	// Lock again, as hall detection will undo the lock
	mc_interface_lock();

	if (res) {
		mcconf_old->m_sensor_port_mode = SENSOR_PORT_MODE_HALL;
		mcconf_old->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
		for (int i = 0;i < 8;i++) {
			mcconf_old->foc_hall_table[i] = hall_table[i];
		}

		result = 1;
	}

	// Sensorless
	if (!res) {
		mcconf_old->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
		result = 0;
		res = true;
	}

	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_wait_for_motor_release(1.0);
	mc_interface_set_configuration(mcconf_old);

	// On success store the mc configuration, also send it to VESC Tool.
	if (res) {
		if (store_mcconf_on_success) {
			conf_general_store_mc_configuration(mcconf_old,
					mc_interface_get_motor_thread() == 2);
		}

		if (send_mcconf_on_success) {
			commands_send_mcconf(COMM_GET_MCCONF, mcconf_old, 0);
		}
	}

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_old);

	return result;
}

void conf_general_calc_apply_foc_cc_kp_ki_gain(mc_configuration *mcconf, float tc) {
	float r = mcconf->foc_motor_r;
	float l = mcconf->foc_motor_l;
	float lambda = mcconf->foc_motor_flux_linkage;

	float bw = 1.0 / (tc * 1e-6);
	float kp = l * bw;
	float ki = r * bw;
	float gain = 1.0e-3 / SQ(lambda);
//	float gain = (0.00001 / r) / SQ(lambda); // Old method

	mcconf->foc_current_kp = kp;
	mcconf->foc_current_ki = ki;
	mcconf->foc_observer_gain = gain * 1e6;
}

static bool measure_r_l_imax(float current_min, float current_max,
		float max_power_loss, float *r, float *l, float *ld_lq_diff, float *i_max) {
	float current_start = current_max / 50;
	if (current_start < (current_min * 1.1)) {
		current_start = current_min * 1.1;
	}

	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();

	const float res_old = mcconf->foc_motor_r;

	float i_last = 0.0;
	for (float i = current_start;i < current_max;i *= 1.5) {
		float res_tmp = mcpwm_foc_measure_resistance(i, 5, false);
		i_last = i;

		if (res_tmp == 0.0) {
			mempools_free_mcconf(mcconf);
			return false;
		}

		if ((i * i * res_tmp * 1.5) >= (max_power_loss / 5.0)) {
			break;
		}
	}

	*r = mcpwm_foc_measure_resistance(i_last, 100, true);
	if (*r == 0.0) {
		mempools_free_mcconf(mcconf);
		return false;
	}

	mcconf->foc_motor_r = *r;
	mc_interface_set_configuration(mcconf);

	bool result = true;
	*l = mcpwm_foc_measure_inductance_current(i_last, 100, 0, ld_lq_diff) * 1e-6;
	if (*l == 0.0) {
		result = false;
	}

	*ld_lq_diff *= 1e-6;
	*i_max = sqrtf(max_power_loss / *r / 1.5);
	utils_truncate_number(i_max, HW_LIM_CURRENT);

	mcconf->foc_motor_r = res_old;
	mc_interface_set_configuration(mcconf);
	mempools_free_mcconf(mcconf);

	return result;
}

static bool wait_fault(int timeout_ms) {
	int motor_last = mc_interface_get_motor_thread();

	for (int i = 0;i < (timeout_ms / 10);i++) {
		mc_interface_select_motor_thread(1);
		mc_fault_code fault1 = mc_interface_get_fault();
		mc_interface_select_motor_thread(2);
		mc_fault_code fault2 = mc_interface_get_fault();

		if (fault1 == FAULT_CODE_NONE && fault2 == FAULT_CODE_NONE) {
			break;
		}

		vTaskDelay(MS_TO_TICKS(10));
	}

	mc_interface_select_motor_thread(1);
	mc_fault_code fault1 = mc_interface_get_fault();
	mc_interface_select_motor_thread(2);
	mc_fault_code fault2 = mc_interface_get_fault();

	mc_interface_select_motor_thread(motor_last);

	return fault1 == FAULT_CODE_NONE && fault2 == FAULT_CODE_NONE;
}

static bool wait_motor_stop(int timeout_ms) {
	int motor_last = mc_interface_get_motor_thread();

	for (int i = 0;i < (timeout_ms / 10);i++) {
		mc_interface_select_motor_thread(1);
		float rpm1 = mc_interface_get_rpm();
		mc_interface_select_motor_thread(2);
		float rpm2 = mc_interface_get_rpm();

		if (fabsf(rpm1) < 100.0 && fabsf(rpm2) < 100.0) {
			break;
		}
		vTaskDelay(MS_TO_TICKS(10));
	}

	mc_interface_select_motor_thread(1);
	float rpm1 = mc_interface_get_rpm();
	mc_interface_select_motor_thread(2);
	float rpm2 = mc_interface_get_rpm();

	mc_interface_select_motor_thread(motor_last);

	return fabsf(rpm1) < 100.0 && fabsf(rpm2) < 100.0;
}

void conf_general_init(void) {
	conf_general_read_app_configuration(&appconf);
	app_set_configuration(&appconf);


	//enable cycle counter
	DEMCR |= DEMCR_TRCENA;
	DWT_CTRL |= CYCCNTENA;

}

/**
 * Get mc_configuration CRC (motor 1 or 2)
 *
 * @param conf
 * Pointer to mc_configuration or NULL for current config
 *
 * @param is_motor_2
 * true if motor2, false if motor1
 *
 * @return
 * CRC16 (with crc field in struct temporarily set to zero).
 */
unsigned conf_calc_crc(mc_configuration* conf_in) {
	volatile mc_configuration* conf = conf_in;

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(mc_configuration));
	conf->crc = crc_old;
	return crc_new;
}

uint8_t Flash_ReadByte_MC(uint32_t x){
	uint8_t data[4];
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_127+((x/4)*4)));
	return data[x%4];
}

uint8_t Flash_ReadByte_APP(uint32_t x){
	uint8_t data[4];
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_126+((x/4)*4)));
	return data[x%4];
}

/**
 * Read app_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a app_configuration struct to write the read configuration to.
 */
void conf_general_read_app_configuration(app_configuration *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;

	for (unsigned int i = 0;i < (sizeof(app_configuration));i++) {
		conf_addr[i] = Flash_ReadByte_APP(i);
	}

	// check CRC
#ifdef TEST_BAD_APP_CRC
	conf->crc++;
#endif
	if(conf->crc != app_calc_crc(conf)) {
		is_ok = false;
//		mc_interface_fault_stop(FAULT_CODE_FLASH_CORRUPTION_APP_CFG, false, false);
		//fault_data f;
		//f.fault = FAULT_CODE_FLASH_CORRUPTION_APP_CFG;
		//terminal_add_fault_data(&f);
	}

	// Set the default configuration
	if (!is_ok) {
		confgenerator_set_defaults_appconf(conf);
	}
}

/**
 * Read mc_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a mc_configuration struct to write the read configuration to.
 */
void conf_general_read_mc_configuration(mc_configuration *conf, bool is_motor_2) {
	uint8_t *conf_addr = (uint8_t*)conf;
	//unsigned int base = is_motor_2 ? EEPROM_BASE_MCCONF_2 : EEPROM_BASE_MCCONF;

	for (unsigned int i = 0;i < (sizeof(mc_configuration));i++) {
		conf_addr[i] = Flash_ReadByte_MC(i);
	}

	if(conf->crc != conf_calc_crc(conf)) {
//		mc_interface_fault_stop(FAULT_CODE_FLASH_CORRUPTION_MC_CFG, is_motor_2, false);
		//fault_data f;
		//f.fault = FAULT_CODE_FLASH_CORRUPTION_MC_CFG;
		//terminal_add_fault_data(&f);
	}


}

bool conf_general_write_flash(uint8_t page, uint8_t * data, uint16_t size){
	uint32_t word;
	uint8_t byte=0;
	uint8_t * word_ptr = (uint8_t*)&word;
	uint32_t flash_incr=0;

	HAL_FLASH_Unlock();

	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef s_eraseinit;
	s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
	s_eraseinit.PageAddress = 0x08000000 + ((uint32_t)page*PAGE_SIZE);
	s_eraseinit.NbPages     = 1;
	HAL_FLASHEx_Erase(&s_eraseinit, &page_error);

	for (unsigned int i = 0;i < size;i++) {

		word_ptr[byte] = data[i];
		byte++;
		if(byte==4){
			byte=0;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, s_eraseinit.PageAddress+(flash_incr*4), *((uint32_t*)word_ptr));
			word=0;
			flash_incr++;
		}
	}
	if(byte!=0){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, s_eraseinit.PageAddress+(flash_incr*4), *((uint32_t*)word_ptr));
	}
	HAL_FLASH_Lock();
	return true;
}

/**
 * Write app_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_app_configuration(app_configuration *conf) {
	bool is_ok = true;


	conf->crc = app_calc_crc(conf);


	is_ok = conf_general_write_flash(APP_PAGE, (uint8_t*)conf, sizeof(app_configuration));

	vTaskDelay(500);

	return is_ok;
}

/**
 * Write mc_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_mc_configuration(mc_configuration *conf, bool is_motor_2) {
	bool is_ok = true;

	conf->crc = conf_calc_crc(conf);

	HAL_FLASH_Unlock();

	is_ok = conf_general_write_flash(CONF_PAGE, (uint8_t*)conf, sizeof(mc_configuration));

	vTaskDelay(500);

	return is_ok;
}

/*mc_configuration* mc_interface_get_configuration(void){
	return &mc_conf;
}*/

void conf_general_mcconf_hw_limits(mc_configuration *mcconf) {
	utils_truncate_number(&mcconf->l_current_max_scale, 0.0, 1.0);
	utils_truncate_number(&mcconf->l_current_min_scale, 0.0, 1.0);
	// This limit should always be active, as starving the threads never
	// makes sense.

#ifndef DISABLE_HW_LIMITS
#ifdef HW_LIM_CURRENT
	utils_truncate_number(&mcconf->l_current_max, HW_LIM_CURRENT);
	utils_truncate_number(&mcconf->l_current_min, HW_LIM_CURRENT);
#endif
#ifdef HW_LIM_CURRENT_IN
	utils_truncate_number(&mcconf->l_in_current_max, HW_LIM_CURRENT_IN);
	utils_truncate_number(&mcconf->l_in_current_min, HW_LIM_CURRENT);
#endif
#ifdef HW_LIM_CURRENT_ABS
	utils_truncate_number(&mcconf->l_abs_current_max, HW_LIM_CURRENT_ABS);
#endif
#ifdef HW_LIM_VIN
	utils_truncate_number(&mcconf->l_max_vin, HW_LIM_VIN);
	utils_truncate_number(&mcconf->l_min_vin, HW_LIM_VIN);
#endif
#ifdef HW_LIM_ERPM
	utils_truncate_number(&mcconf->l_max_erpm, HW_LIM_ERPM);
	utils_truncate_number(&mcconf->l_min_erpm, HW_LIM_ERPM);
#endif
#ifdef HW_LIM_DUTY_MIN
	utils_truncate_number(&mcconf->l_min_duty, HW_LIM_DUTY_MIN);
#endif
#ifdef HW_LIM_DUTY_MAX
	utils_truncate_number(&mcconf->l_max_duty, HW_LIM_DUTY_MAX);
#endif
#ifdef HW_LIM_TEMP_FET
	utils_truncate_number(&mcconf->l_temp_fet_start, HW_LIM_TEMP_FET);
	utils_truncate_number(&mcconf->l_temp_fet_end, HW_LIM_TEMP_FET);
#endif

#endif
}


/**
 * Try to measure the motor flux linkage using open loop FOC control.
 *
 * @param current
 * The Q-axis current to spin up the motor.
 *
 * @param duty
 * Duty cycle % to measure at
 *
 * @param erpm_per_sec
 * Acceleration rate
 *
 * @param res
 * The motor phase resistance.
 *
 * @param linkage
 * The calculated flux linkage.
 *
 * @param linkage_undriven
 * Flux linkage measured while the motor was undriven.
 *
 * @param undriven_samples
 * Number of flux linkage samples while the motor was undriven.
 *
 * @return
 * True for success, false otherwise.
 */
bool conf_general_measure_flux_linkage_openloop(float current, float duty,
		float erpm_per_sec, float res, float ind, float *linkage,
		float *linkage_undriven, float *undriven_samples) {
	bool result = false;

	mc_configuration *mcconf = pvPortMalloc(sizeof(mc_configuration));
	mc_configuration *mcconf_old = pvPortMalloc(sizeof(mc_configuration));

	*mcconf = *mc_interface_get_configuration();
	*mcconf_old = *mcconf;

	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
	mcconf->foc_current_kp = 0.0005;
	mcconf->foc_current_ki = 1.0;
	mcconf->foc_cc_decoupling = FOC_CC_DECOUPLING_DISABLED;
	mc_interface_set_configuration(mcconf);

	// Wait maximum 5s for fault code to disappear
	for (int i = 0;i < 500;i++) {
		if (mc_interface_get_fault() == FAULT_CODE_NONE) {
			break;
		}
		vTaskDelay(MS_TO_TICKS(10));
	}

	if (mc_interface_get_fault() != FAULT_CODE_NONE) {
		mc_interface_set_configuration(mcconf_old);
		vPortFree(mcconf);
		vPortFree(mcconf_old);
		return false;
	}

	// Wait one second for things to get ready after
	// the fault disapears.
	vTaskDelay(MS_TO_TICKS(1000));

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	mc_interface_lock();

	int cnt = 0;
	float rpm_now = 0;

	// Start by locking the motor
	for (int i = 0;i < 200;i++) {
		mcpwm_foc_set_openloop((float)i * current / 200.0, rpm_now);
		vTaskDelay(MS_TO_TICKS(1));
	}

	float duty_still = 0;
	float samples = 0;
	for (int i = 0;i < 1000;i++) {
		duty_still += fabsf(mc_interface_get_duty_cycle_now());
		samples += 1.0;
		vTaskDelay(MS_TO_TICKS(1));
	}

	if (mc_interface_get_fault() != FAULT_CODE_NONE) {
		timeout_configure(tout, tout_c, tout_ksw);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_wait_for_motor_release(1.0);
		mc_interface_set_configuration(mcconf_old);
		vPortFree(mcconf);
		vPortFree(mcconf_old);
		return false;
	}

	duty_still /= samples;
	float duty_max = 0.0;
	const int max_time = 15000;

	while (fabsf(mc_interface_get_duty_cycle_now()) < duty) {
		rpm_now += erpm_per_sec / 1000.0;
		mcpwm_foc_set_openloop(current, mcconf->m_invert_direction ? -rpm_now : rpm_now);

		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			timeout_configure(tout, tout_c, tout_ksw);
			mc_interface_unlock();
			mc_interface_release_motor();
			mc_interface_wait_for_motor_release(1.0f);
			mc_interface_set_configuration(mcconf_old);
			vPortFree(mcconf);
			vPortFree(mcconf_old);
			return false;
		}

		vTaskDelay(MS_TO_TICKS(1));
		cnt++;
		float duty_now = fabsf(mc_interface_get_duty_cycle_now());

		if (duty_now > duty_max) {
			duty_max = duty_now;
		}

		if (cnt >= max_time) {
			*linkage = -1.0;
			break;
		}

		if (cnt > 4000 && duty_now < (duty_max * 0.7)) {
			cnt = max_time;
			*linkage = -2.0;
			break;
		}

		if (cnt > 4000 && duty < duty_still * 1.1) {
			cnt = max_time;
			*linkage = -3.0;
			break;
		}

		if (rpm_now >= 12000) {
			break;
		}
	}

	vTaskDelay(MS_TO_TICKS(1000));

	if (cnt < max_time) {
		float vq_avg = 0.0;
		float vd_avg = 0.0;
		float iq_avg = 0.0;
		float id_avg = 0.0;
		float samples2 = 0.0;

		for (int i = 0;i < 10000;i++) {
			vq_avg += mcpwm_foc_get_vq();
			vd_avg += mcpwm_foc_get_vd();
			iq_avg += mcpwm_foc_get_iq();
			id_avg += mcpwm_foc_get_id();
			samples2 += 1.0;
			vTaskDelay(1);
		}

		vq_avg /= samples2;
		vd_avg /= samples2;
		iq_avg /= samples2;
		id_avg /= samples2;

		float rad_s = RPM2RADPS_f(rpm_now);
		float v_mag = NORM2_f(vq_avg, vd_avg);
		float i_mag = NORM2_f(iq_avg, id_avg);
		*linkage = (v_mag - res * i_mag) / rad_s - i_mag * ind;

		mcconf->foc_motor_r = res;
		mcconf->foc_motor_l = ind;
		mcconf->foc_motor_flux_linkage = *linkage;
		mcconf->foc_observer_gain = 0.5e3 / SQ(*linkage);
		mc_interface_set_configuration(mcconf);

		// Give the observer time to settle
		vTaskDelay(MS_TO_TICKS(500));

		// Turn off the FETs
		mcpwm_foc_stop_pwm(false);

		// Clear any lingering current set points
		mcpwm_foc_set_current(0.0);

		// Let the H-bridges settle
		vTaskDelay(MS_TO_TICKS(5));

		float linkage_sum = 0.0;
		float linkage_samples = 0.0;
		for (int i = 0;i < 2000;i++) {
			float rad_s_now = RPM2RADPS_f(mcpwm_foc_get_rpm_faster());
			if (fabsf(mcpwm_foc_get_duty_cycle_now()) < 0.02) {
				break;
			}

			linkage_sum += mcpwm_foc_get_vq() / rad_s_now;

			// Optionally use magnitude
//			linkage_sum += sqrtf(SQ(mcpwm_foc_get_vq()) + SQ(mcpwm_foc_get_vd())) / rad_s_now;

			// Optionally use magnitude of observer state
//			float x1, x2;
//			mcpwm_foc_get_observer_state(&x1, &x2);
//			linkage_sum += sqrtf(SQ(x1) + SQ(x2));

			linkage_samples += 1.0;
			for (uint32_t i = 0; i < 1000; i++) {
			    __asm__ __volatile__ ("" : "+g" (i) : :);
			}
			//vTaskDelay(1); //TODO Was 1 tick @ 10kHz
		}

		*undriven_samples = linkage_samples;

		if (linkage_samples > 0) {
			*linkage_undriven = linkage_sum / linkage_samples;
		} else {
			*linkage_undriven = 0.0;
		}

		result = true;
	}

	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_wait_for_motor_release(1.0);
	mc_interface_set_configuration(mcconf_old);
	vPortFree(mcconf);
	vPortFree(mcconf_old);
	return result;
}

/**
 * Detect and apply all parameters, current limits and sensors. This is done for
 * both motors on dual controllers.
 *
 * @param max_power_loss
 * The maximum power loss to derive current limits, as well as detection currents, from.
 *
 * @param store_mcconf_on_success
 * Store motor configuration in emulated EEPROM if the detection succeeds.
 *
 * @param send_mcconf_on_success
 * Send motor configuration if the detection succeeds.
 *
 * @return
 * >=0: Success, see conf_general_autodetect_apply_sensors_foc codes
 * -10: Flux linkage detection failed
 *  -x: see conf_general_autodetect_apply_sensors_foc faults
 */
int conf_general_detect_apply_all_foc(float max_power_loss,
		bool store_mcconf_on_success, bool send_mcconf_on_success) {
	int result = -1;

	int motor_last = mc_interface_get_motor_thread();
	mc_interface_select_motor_thread(1);

	mc_configuration *mcconf = mempools_alloc_mcconf();
	mc_configuration *mcconf_old = mempools_alloc_mcconf();

	*mcconf = *mc_interface_get_configuration();
	*mcconf_old = *mcconf;


	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
	mcconf->foc_f_zv = 10000.0; // Lower f_zv => less dead-time distortion
	mcconf->foc_current_kp = 0.0005;
	mcconf->foc_current_ki = 1.0;
	mcconf->l_current_max = MCCONF_L_CURRENT_MAX;
	mcconf->l_current_min = MCCONF_L_CURRENT_MIN;
	mcconf->l_current_max_scale = MCCONF_L_CURRENT_MAX_SCALE;
	mcconf->l_current_min_scale = MCCONF_L_CURRENT_MIN_SCALE;
	mcconf->l_watt_max = MCCONF_L_WATT_MAX;
	mcconf->l_watt_min = MCCONF_L_WATT_MIN;
	mcconf->l_max_erpm = MCCONF_L_RPM_MAX;
	mcconf->l_min_erpm = MCCONF_L_RPM_MIN;
	mc_interface_set_configuration(mcconf);


	// Wait maximum 5s for fault code to disappear
	if (!wait_fault(5000)) {
		mc_interface_set_configuration(mcconf_old);
		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);
		mc_interface_select_motor_thread(motor_last);
		return -1;
	}

	// Wait one second for things to get ready after
	// the fault disappears.
	vTaskDelay(MS_TO_TICKS(1000));

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	mc_interface_lock();


	float r = 0.0;
	float l = 0.0;
	float ld_lq_diff;
	float i_max = 0.0;
	bool res_r_l_imax_m1 = measure_r_l_imax(mcconf->cc_min_current, mcconf->l_current_max, max_power_loss, &r, &l, &ld_lq_diff, &i_max);

	bool res_r_l_imax_m2 = true;

	if (!res_r_l_imax_m1 || !res_r_l_imax_m2) {
		timeout_configure(tout, tout_c, tout_ksw);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_wait_for_motor_release(1.0);
		mc_interface_set_configuration(mcconf_old);
		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);
		mc_interface_select_motor_thread(motor_last);
		return -11;
	}

	// Increase switching frequency for flux linkage measurement
	// as dead-time distortion has less effect at higher modulation.
	// Having a smooth rotation is more important.
	mcconf->foc_f_zv = 20000.0;
	mc_interface_set_configuration(mcconf);


	float lambda = 0.0;
	float lambda_undriven = 0.0;
	float lambda_undriven_samples = 0.0;
	int res = conf_general_measure_flux_linkage_openloop(i_max / 2.5, 0.3, 1800, r, l,
			&lambda, &lambda_undriven, &lambda_undriven_samples);

	if (lambda_undriven_samples > 60) {
		lambda = lambda_undriven;
	}

	bool res_linkage_m2 = true;

	if (res && res_linkage_m2) {
		mcconf_old->l_current_max = i_max;
		mcconf_old->l_current_min = -i_max;
		mcconf_old->motor_type = MOTOR_TYPE_FOC;
		mcconf_old->foc_motor_r = r;
		mcconf_old->foc_motor_l = l;
		mcconf_old->foc_motor_ld_lq_diff = ld_lq_diff;
		mcconf_old->foc_motor_flux_linkage = lambda;

		if (mc_interface_temp_motor_filtered() > -10) {
			mcconf_old->foc_temp_comp_base_temp = mc_interface_temp_motor_filtered();
#ifdef HW_HAS_PHASE_FILTERS
			mcconf_old->foc_temp_comp = true;
#endif
		}

		conf_general_calc_apply_foc_cc_kp_ki_gain(mcconf_old, 1000);
		mc_interface_set_configuration(mcconf_old);

		wait_motor_stop(10000);


		// This will also store the settings to emulated eeprom and send them to vesc tool
		result = conf_general_autodetect_apply_sensors_foc(i_max / 3.0,
				store_mcconf_on_success, send_mcconf_on_success);

		int res_sensors_m2 = 0;

		if (res_sensors_m2 < 0) {
			result = res_sensors_m2;
		}
	} else {
		result = -10;
	}

	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_lock_override_once();
	mc_interface_release_motor();
	mc_interface_wait_for_motor_release(1.0);
	mc_interface_unlock();

	if (result < 0) {
		mc_interface_set_configuration(mcconf_old);
	}


	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_old);

	mc_interface_select_motor_thread(motor_last);

	return result;
}

/**
 * Same as conf_general_detect_apply_all_foc, but also start detection on VESCs found on the CAN-bus.
 *
 * @param detect_can
 * Run detection on VESCs found on the CAN-bus as well. Setting this to false makes
 * this function behave like conf_general_detect_apply_all_foc, with the convenience
 * of also applying the settings.
 *
 * @param max_power_loss
 * The maximum power loss to derive current limits, as well as detection currents, from.
 *
 * @param min_current_in
 * Minimum input current (negative value). 0 means leave it unchanged.
 *
 * @param max_current_in
 * MAximum input current. 0 means leave it unchanged.
 *
 * @param openloop_rpm
 * FOC openloop ERPM in sensorless mode. 0 means leave it unchanged.
 *
 * @param sl_erpm
 * FOC ERPM above which sensorless should be used in sensored modes. 0 means leave it unchanged.
 *
 * @return
 * Same as conf_general_detect_apply_all_foc, and
 * -50: CAN detection timed out
 * -51: CAN detection failed
 */
int conf_general_detect_apply_all_foc_can(bool detect_can, float max_power_loss,
		float min_current_in, float max_current_in, float openloop_rpm, float sl_erpm, PACKET_STATE_t * phandle) {

	int motor_last = mc_interface_get_motor_thread();
	mc_interface_select_motor_thread(1);

	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();


	if (fabsf(min_current_in) > 0.001) {
		mcconf->l_in_current_min = min_current_in;
	} else {
		mcconf->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
	}

	if (fabsf(max_current_in) > 0.001) {
		mcconf->l_in_current_max = max_current_in;
	} else {
		mcconf->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
	}

	if (fabsf(openloop_rpm) > 0.001) {
		mcconf->foc_openloop_rpm = openloop_rpm;
	} else {
		mcconf->foc_openloop_rpm = MCCONF_FOC_OPENLOOP_RPM;
	}

	if (fabsf(sl_erpm) > 0.001) {
		mcconf->foc_sl_erpm = sl_erpm;
	} else {
		mcconf->foc_sl_erpm = MCCONF_FOC_SL_ERPM;
	}

	mc_interface_set_configuration(mcconf);


	int res = conf_general_detect_apply_all_foc(max_power_loss, false, false);

	// Store and send settings
	if (res >= 0) {

		*mcconf = *mc_interface_get_configuration();
		conf_general_store_mc_configuration(mcconf, mc_interface_get_motor_thread() == 2);

		commands_send_mcconf(COMM_GET_MCCONF, mcconf, phandle);
		vTaskDelay(MS_TO_TICKS(1000));
	}

	mempools_free_mcconf(mcconf);

	mc_interface_select_motor_thread(motor_last);

	return res;
}
