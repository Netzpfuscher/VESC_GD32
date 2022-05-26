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
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "mcpwm_foc.h"
#include <mc_interface.h>
//#include "hal.h"
//#include "stm32f4xx_conf.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_adc.h"
#include "digital_filter.h"
#include "utils_math.h"
#include "utils_sys.h"
//#include "ledpwm.h"
#include "terminal.h"
//#include "encoder/encoder.h"
#include "timeout.h"
//#include "timer.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "foc_math.h"
#include "FreeRTOS.h"
#include <commands.h>
#include "task_init.h"
#include "main.h"
#include "mcconf_default.h"

// Private variables
static volatile bool m_dccal_done = false;
static volatile float m_last_adc_isr_duration;
static volatile bool m_init_done = false;
static volatile motor_all_state_t m_motor_1;
static volatile int m_isr_motor = 0;

// Private functions
static void control_current(motor_all_state_t *motor, float dt);
static void update_valpha_vbeta(motor_all_state_t *motor, float mod_alpha, float mod_beta);
static void stop_pwm_hw(motor_all_state_t *motor);
static void start_pwm_hw(motor_all_state_t *motor);
static void terminal_tmp(int argc, const char **argv);
static void terminal_plot_hfi(int argc, const char **argv);
static void timer_update(motor_all_state_t *motor, float dt);
static void input_current_offset_measurement( void );
static void hfi_update(volatile motor_all_state_t *motor, float dt);

#define TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

// Threads
static volatile bool timer_thd_stop;
void timer_thread(void * argument);
static volatile bool hfi_thd_stop;
void hfi_thread(void * argument);
static volatile bool pid_thd_stop;
void pid_thread(void * argument);

// #define M_MOTOR: For single motor compilation, expands to &m_motor_1.
// For dual motors, expands to &m_motor_1 or _2, depending on is_second_motor.
#ifdef HW_HAS_DUAL_MOTORS
#define M_MOTOR(is_second_motor) (is_second_motor ? &m_motor_2 : &m_motor_1)
#else
#define M_MOTOR(is_second_motor)  (((void)is_second_motor), &m_motor_1)
#endif

static void update_hfi_samples(foc_hfi_samples samples, volatile motor_all_state_t *motor) {
	utils_sys_lock_cnt();

	memset((void*)&motor->m_hfi, 0, sizeof(motor->m_hfi));
	switch (samples) {
	case HFI_SAMPLES_8:
		motor->m_hfi.samples = 8;
		motor->m_hfi.table_fact = 4;
		motor->m_hfi.fft_bin0_func = utils_fft8_bin0;
		motor->m_hfi.fft_bin1_func = utils_fft8_bin1;
		motor->m_hfi.fft_bin2_func = utils_fft8_bin2;
		break;

	case HFI_SAMPLES_16:
		motor->m_hfi.samples = 16;
		motor->m_hfi.table_fact = 2;
		motor->m_hfi.fft_bin0_func = utils_fft16_bin0;
		motor->m_hfi.fft_bin1_func = utils_fft16_bin1;
		motor->m_hfi.fft_bin2_func = utils_fft16_bin2;
		break;

	case HFI_SAMPLES_32:
		motor->m_hfi.samples = 32;
		motor->m_hfi.table_fact = 1;
		motor->m_hfi.fft_bin0_func = utils_fft32_bin0;
		motor->m_hfi.fft_bin1_func = utils_fft32_bin1;
		motor->m_hfi.fft_bin2_func = utils_fft32_bin2;
		break;
	}

	utils_sys_unlock_cnt();
}

//static void timer_reinit(int f_zv) {
void timer_reinit(int f_zv) {

	TIM1->CNT = 0;
	TIM2->CNT = 0;

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = CPU_MHZ / f_zv;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; //TODO: depends on gate driver!
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = (CPU_MHZ / f_zv);
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 32;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);


	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

	uint32_t top = TIM1->ARR;

	TIM1->CCR4 = top-100;



	TIM1->CCR1 = top/2; //set initial PWM values
	TIM1->CCR2 = top/2;
	TIM1->CCR3 = top/2;

	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);

}
#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))
#define TIMxCCER_MASK_CH1          ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N))
#define TIMxCCER_MASK_CH2          ((uint16_t)  (LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N))
#define TIMxCCER_MASK_CH3          ((uint16_t)  (LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2) {
	//TODO utils_sys_lock_cnt();

#ifndef HW_HAS_DUAL_MOTORS
	(void)conf_m2;
#endif

	m_init_done = false;

	memset((void*)&m_motor_1, 0, sizeof(motor_all_state_t));
	m_isr_motor = 0;

	m_motor_1.m_conf = conf_m1;
	m_motor_1.m_state = MC_STATE_OFF;
	m_motor_1.m_control_mode = CONTROL_MODE_NONE;
	m_motor_1.m_hall_dt_diff_last = 1.0;
	foc_precalc_values((motor_all_state_t*)&m_motor_1);
	update_hfi_samples(m_motor_1.m_conf->foc_hfi_samples, &m_motor_1);

	//virtual_motor_init(conf_m1);


	timer_reinit((int)m_motor_1.m_conf->foc_f_zv);

	stop_pwm_hw((motor_all_state_t*)&m_motor_1);

	SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG); //external trigger enable
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	SET_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG); //external trigger enable
	SET_BIT(ADC3->CR2, ADC_CR2_JEXTTRIG); //external trigger enable
	//__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
	LL_ADC_Enable( ADC1 );
	LL_ADC_Enable( ADC2 );
	LL_ADC_Enable( ADC3 );

	//LL_ADC_ClearFlag_JEOS( ADC1);
	//LL_ADC_EnableIT_JEOS( ADC1 );

	if (m_motor_1.m_conf->foc_offsets_cal_on_boot) {
			systime_t cal_start_time = xTaskGetTickCount();
			float cal_start_timeout = 10.0;

			// Wait for input voltage to rise above minimum voltage
			while (mc_interface_get_input_voltage_filtered() < m_motor_1.m_conf->l_min_vin) {
				vTaskDelay(MS_TO_TICKS(1));
				if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
					m_dccal_done = true;
					break;
				}
			}

			// Wait for input voltage to settle
			if (!m_dccal_done) {
				float v_in_last = mc_interface_get_input_voltage_filtered();
				systime_t v_in_stable_time = xTaskGetTickCount();
				while (UTILS_AGE_S(v_in_stable_time) < 2.0) {
					vTaskDelay(MS_TO_TICKS(1));

					float v_in_now = mc_interface_get_input_voltage_filtered();
					if (fabsf(v_in_now - v_in_last) > 1.5) {
						v_in_last = v_in_now;
						v_in_stable_time = xTaskGetTickCount();
					}

					if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
						m_dccal_done = true;
						break;
					}
				}
			}

			// Wait for fault codes to go away
			if (!m_dccal_done) {
				while (mc_interface_get_fault() != FAULT_CODE_NONE) {
					vTaskDelay(MS_TO_TICKS(1));
					if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
						m_dccal_done = true;
						break;
					}
				}
			}
		vTaskDelay(MS_TO_TICKS(1000));

			if (!m_dccal_done) {
				m_motor_1.m_conf->foc_offsets_voltage[0] = MCCONF_FOC_OFFSETS_VOLTAGE_0;
				m_motor_1.m_conf->foc_offsets_voltage[1] = MCCONF_FOC_OFFSETS_VOLTAGE_1;
				m_motor_1.m_conf->foc_offsets_voltage[2] = MCCONF_FOC_OFFSETS_VOLTAGE_2;

				m_motor_1.m_conf->foc_offsets_voltage_undriven[0] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0;
				m_motor_1.m_conf->foc_offsets_voltage_undriven[1] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1;
				m_motor_1.m_conf->foc_offsets_voltage_undriven[2] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2;

				m_motor_1.m_conf->foc_offsets_current[0] = MCCONF_FOC_OFFSETS_CURRENT_0;
				m_motor_1.m_conf->foc_offsets_current[1] = MCCONF_FOC_OFFSETS_CURRENT_1;
				m_motor_1.m_conf->foc_offsets_current[2] = MCCONF_FOC_OFFSETS_CURRENT_2;

				mcpwm_foc_dc_cal(false);
			}
		} else {
			m_dccal_done = true;
		}


	// Start threads
	timer_thd_stop = false;
	static uint8_t task = 0;

	if(task==0){
		xTaskCreate(timer_thread, "tskTIMER", 256, NULL, PRIO_NORMAL, NULL);

		hfi_thd_stop = false;
		xTaskCreate(hfi_thread, "tskHFI", 256, NULL, PRIO_NORMAL, NULL);

		pid_thd_stop = false;
		xTaskCreate(pid_thread, "tskPID", 256, NULL, PRIO_NORMAL, NULL);
		task = 1;
	}

	m_init_done = true;
}

void mcpwm_foc_deinit(void) {
	if (!m_init_done) {
		return;
	}

	m_init_done = false;

	timer_thd_stop = true;
	while (timer_thd_stop) {
		vTaskDelay(MS_TO_TICKS(1));
	}

	hfi_thd_stop = true;
	while (hfi_thd_stop) {
		vTaskDelay(MS_TO_TICKS(1));
	}

	pid_thd_stop = true;
	while (pid_thd_stop) {
		vTaskDelay(MS_TO_TICKS(1));
	}

}

static volatile motor_all_state_t *get_motor_now(void) {
#ifdef HW_HAS_DUAL_MOTORS
	return mc_interface_motor_now() == 1 ? &m_motor_1 : &m_motor_2;
#else
	return &m_motor_1;
#endif
}

bool mcpwm_foc_init_done(void) {
	return m_init_done;
}

void mcpwm_foc_set_configuration(mc_configuration *configuration) {
	get_motor_now()->m_conf = configuration;
	foc_precalc_values((motor_all_state_t*)get_motor_now());

	// Below we check if anything in the configuration changed that requires stopping the motor.

	uint32_t top = CPU_MHZ / (int)configuration->foc_f_zv;
	if (TIM1->ARR != top) {

		get_motor_now()->m_control_mode = CONTROL_MODE_NONE;
		get_motor_now()->m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)get_motor_now());
		timer_reinit(configuration->foc_f_zv);
		//TIMER_UPDATE_SAMP_TOP_M1(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);

	}

	if (((1 << get_motor_now()->m_conf->foc_hfi_samples) * 8) != get_motor_now()->m_hfi.samples) {
		get_motor_now()->m_control_mode = CONTROL_MODE_NONE;
		get_motor_now()->m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)get_motor_now());
		update_hfi_samples(get_motor_now()->m_conf->foc_hfi_samples, get_motor_now());
	}

	//virtual_motor_set_configuration(configuration);
}

mc_state mcpwm_foc_get_state(void) {
	return get_motor_now()->m_state;
}

mc_control_mode mcpwm_foc_control_mode(void) {
	return get_motor_now()->m_control_mode;
}

bool mcpwm_foc_is_dccal_done(void) {
	return m_dccal_done;
}

/**
 * Get the current motor used in the mcpwm ISR
 *
 * @return
 * 0: Not in ISR
 * 1: Motor 1
 * 2: Motor 2
 */
int mcpwm_foc_isr_motor(void) {
	return m_isr_motor;
}

/**
 * Switch off all FETs.
 */
void mcpwm_foc_stop_pwm(bool is_second_motor) {
	motor_all_state_t *motor = (motor_all_state_t*)M_MOTOR(is_second_motor);

	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw(motor);
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_foc_set_duty(float dutyCycle) {
	get_motor_now()->m_control_mode = CONTROL_MODE_DUTY;
	get_motor_now()->m_duty_cycle_set = dutyCycle;

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * WARNING: This function does not use ramping. A too large step with a large motor
 * can destroy hardware.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_foc_set_duty_noramp(float dutyCycle) {
	// TODO: Actually do this without ramping
	mcpwm_foc_set_duty(dutyCycle);
}

/**
 * Use PID rpm control. Note that this value has to be multiplied by half of
 * the number of motor poles.
 *
 * @param rpm
 * The electrical RPM goal value to use.
 */
void mcpwm_foc_set_pid_speed(float rpm) {
	if (get_motor_now()->m_conf->s_pid_ramp_erpms_s > 0.0 ) {
		if (get_motor_now()->m_control_mode != CONTROL_MODE_SPEED ||
				get_motor_now()->m_state != MC_STATE_RUNNING) {
			get_motor_now()->m_speed_pid_set_rpm = mcpwm_foc_get_rpm();
		}

		get_motor_now()->m_speed_command_rpm = rpm;
	} else {
		get_motor_now()->m_speed_pid_set_rpm = rpm;
	}

	get_motor_now()->m_control_mode = CONTROL_MODE_SPEED;

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use PID position control. Note that this only works when encoder support
 * is enabled.
 *
 * @param pos
 * The desired position of the motor in degrees.
 */
void mcpwm_foc_set_pid_pos(float pos) {
	get_motor_now()->m_control_mode = CONTROL_MODE_POS;
	get_motor_now()->m_pos_pid_set = pos;

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use current control and specify a goal current to use. The sign determines
 * the direction of the torque. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use.
 */
void mcpwm_foc_set_current(float current) {
	get_motor_now()->m_control_mode = CONTROL_MODE_CURRENT;
	get_motor_now()->m_iq_set = current;
	get_motor_now()->m_id_set = 0;
	
	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

void mcpwm_foc_release_motor(void) {
	get_motor_now()->m_control_mode = CONTROL_MODE_CURRENT;
	get_motor_now()->m_iq_set = 0.0;
	get_motor_now()->m_id_set = 0.0;
	get_motor_now()->m_motor_released = true;
}

/**
 * Brake the motor with a desired current. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use. Positive and negative values give the same effect.
 */
void mcpwm_foc_set_brake_current(float current) {
	get_motor_now()->m_control_mode = CONTROL_MODE_CURRENT_BRAKE;
	get_motor_now()->m_iq_set = current;

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Apply a fixed static current vector in open loop to emulate an electric
 * handbrake.
 *
 * @param current
 * The brake current to use.
 */
void mcpwm_foc_set_handbrake(float current) {
	get_motor_now()->m_control_mode = CONTROL_MODE_HANDBRAKE;
	get_motor_now()->m_iq_set = current;

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Produce an openloop rotating current.
 *
 * @param current
 * The current to use.
 *
 * @param rpm
 * The RPM to use.
 */
void mcpwm_foc_set_openloop(float current, float rpm) {
	utils_truncate_number(&current, -get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale,
						  get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale);

	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP;
	get_motor_now()->m_iq_set = current;
	get_motor_now()->m_openloop_speed = RPM2RADPS_f(rpm);

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Produce an openloop current at a fixed phase.
 *
 * @param current
 * The current to use.
 *
 * @param phase
 * The phase to use in degrees, range [0.0 360.0]
 */
void mcpwm_foc_set_openloop_phase(float current, float phase) {
	utils_truncate_number(&current, -get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale,
						  get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale);

	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP_PHASE;
	get_motor_now()->m_id_set = current;
	get_motor_now()->m_iq_set = 0;

	get_motor_now()->m_openloop_phase = DEG2RAD_f(phase);
	utils_norm_angle_rad((float*)&get_motor_now()->m_openloop_phase);

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Get current offsets,
 * this is used by the virtual motor to save the current offsets,
 * when it is connected
 */
void mcpwm_foc_get_current_offsets(
		volatile float *curr0_offset,
		volatile float *curr1_offset,
		volatile float *curr2_offset,
		bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	*curr0_offset = motor->m_conf->foc_offsets_current[0];
	*curr1_offset = motor->m_conf->foc_offsets_current[1];
	*curr2_offset = motor->m_conf->foc_offsets_current[2];
}

/**
 * Set current offsets values,
 * this is used by the virtual motor to set the previously saved offsets back,
 * when it is disconnected
 */
void mcpwm_foc_set_current_offsets(volatile float curr0_offset,
								   volatile float curr1_offset,
								   volatile float curr2_offset) {
	get_motor_now()->m_conf->foc_offsets_current[0] = curr0_offset;
	get_motor_now()->m_conf->foc_offsets_current[1] = curr1_offset;
	get_motor_now()->m_conf->foc_offsets_current[2] = curr2_offset;
}

void mcpwm_foc_get_voltage_offsets(
		float *v0_offset,
		float *v1_offset,
		float *v2_offset,
		bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	*v0_offset = motor->m_conf->foc_offsets_voltage[0];
	*v1_offset = motor->m_conf->foc_offsets_voltage[1];
	*v2_offset = motor->m_conf->foc_offsets_voltage[2];
}

void mcpwm_foc_get_voltage_offsets_undriven(
		float *v0_offset,
		float *v1_offset,
		float *v2_offset,
		bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	*v0_offset = motor->m_conf->foc_offsets_voltage_undriven[0];
	*v1_offset = motor->m_conf->foc_offsets_voltage_undriven[1];
	*v2_offset = motor->m_conf->foc_offsets_voltage_undriven[2];
}

/**
 * Produce an openloop rotating voltage.
 *
 * @param dutyCycle
 * The duty cycle to use.
 *
 * @param rpm
 * The RPM to use.
 */
void mcpwm_foc_set_openloop_duty(float dutyCycle, float rpm) {
	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP_DUTY;
	get_motor_now()->m_duty_cycle_set = dutyCycle;
	get_motor_now()->m_openloop_speed = RPM2RADPS_f(rpm);

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Produce an openloop voltage at a fixed phase.
 *
 * @param dutyCycle
 * The duty cycle to use.
 *
 * @param phase
 * The phase to use in degrees, range [0.0 360.0]
 */
void mcpwm_foc_set_openloop_duty_phase(float dutyCycle, float phase) {
	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
	get_motor_now()->m_duty_cycle_set = dutyCycle;
	get_motor_now()->m_openloop_phase = DEG2RAD_f(phase);
	utils_norm_angle_rad((float*)&get_motor_now()->m_openloop_phase);

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

float mcpwm_foc_get_duty_cycle_set(void) {
	return get_motor_now()->m_duty_cycle_set;
}

float mcpwm_foc_get_duty_cycle_now(void) {
	return get_motor_now()->m_motor_state.duty_now;
}

float mcpwm_foc_get_pid_pos_set(void) {
	return get_motor_now()->m_pos_pid_set;
}

float mcpwm_foc_get_pid_pos_now(void) {
	return get_motor_now()->m_pos_pid_now;
}

/**
 * Get the current switching frequency.
 *
 * @return
 * The switching frequency in Hz.
 */
float mcpwm_foc_get_switching_frequency_now(void) {
	return get_motor_now()->m_conf->foc_f_zv;
}

/**
 * Get the current sampling frequency.
 *
 * @return
 * The sampling frequency in Hz.
 */
float mcpwm_foc_get_sampling_frequency_now(void) {
#ifdef HW_HAS_PHASE_SHUNTS
	if (get_motor_now()->m_conf->foc_sample_v0_v7) {
		return get_motor_now()->m_conf->foc_f_zv;
	} else {
		return get_motor_now()->m_conf->foc_f_zv / 2.0;
	}
#else
	return get_motor_now()->m_conf->foc_f_zv / 2.0;
#endif
}

/**
 * Returns Ts used for virtual motor sync
 */
float mcpwm_foc_get_ts(void) {
#ifdef HW_HAS_PHASE_SHUNTS
	if (get_motor_now()->m_conf->foc_sample_v0_v7) {
		return (1.0 / get_motor_now()->m_conf->foc_f_zv) ;
	} else {
		return (1.0 / (get_motor_now()->m_conf->foc_f_zv / 2.0));
	}
#else
	return (1.0 / get_motor_now()->m_conf->foc_f_zv) ;
#endif
}

bool mcpwm_foc_is_using_encoder(void) {
	return get_motor_now()->m_using_encoder;
}

void mcpwm_foc_get_observer_state(float *x1, float *x2) {
	volatile motor_all_state_t *motor = get_motor_now();
	*x1 = motor->m_observer_state.x1;
	*x2 = motor->m_observer_state.x2;
}

/**
 * Set current off delay. Prevent the current controller from switching off modulation
 * for target currents < cc_min_current for this amount of time.
 */
void mcpwm_foc_set_current_off_delay(float delay_sec) {
	if (get_motor_now()->m_current_off_delay < delay_sec) {
		get_motor_now()->m_current_off_delay = delay_sec;
	}
}

float mcpwm_foc_get_tot_current_motor(bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq) * motor->m_motor_state.i_abs;
}

float mcpwm_foc_get_tot_current_filtered_motor(bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq_filter) * motor->m_motor_state.i_abs_filter;
}

float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_motor_state.i_bus;
}

float mcpwm_foc_get_tot_current_in_filtered_motor(bool is_second_motor) {
	// TODO: Filter current?
	return M_MOTOR(is_second_motor)->m_motor_state.i_bus;
}

float mcpwm_foc_get_abs_motor_current_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_motor_state.i_abs;
}

float mcpwm_foc_get_abs_motor_current_filtered_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_motor_state.i_abs_filter;
}

mc_state mcpwm_foc_get_state_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_state;
}

/**
 * Calculate the current RPM of the motor. This is a signed value and the sign
 * depends on the direction the motor is rotating in. Note that this value has
 * to be divided by half the number of motor poles.
 *
 * @return
 * The RPM value.
 */
float mcpwm_foc_get_rpm(void) {
	return RADPS2RPM_f(get_motor_now()->m_motor_state.speed_rad_s);
	//	return get_motor_now()->m_speed_est_fast * RADPS2RPM_f;
}

/**
 * Same as above, but uses the fast and noisier estimator.
 */
float mcpwm_foc_get_rpm_fast(void) {
	return RADPS2RPM_f(get_motor_now()->m_speed_est_fast);
}

/**
 * Same as above, but uses the faster and noisier estimator.
 */
float mcpwm_foc_get_rpm_faster(void) {
	return RADPS2RPM_f(get_motor_now()->m_speed_est_faster);
}

/**
 * Get the motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current(void) {
	volatile motor_all_state_t *motor = get_motor_now();
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq) * motor->m_motor_state.i_abs;
}

/**
 * Get the filtered motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_filtered(void) {
	volatile motor_all_state_t *motor = get_motor_now();
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq_filter) * motor->m_motor_state.i_abs_filter;
}

/**
 * Get the magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current(void) {
	return get_motor_now()->m_motor_state.i_abs;
}

/**
 * Get the magnitude of the motor current unbalance
 *
 * @return
 * The magnitude of the phase currents unbalance.
 */
float mcpwm_foc_get_abs_motor_current_unbalance(void) {
	return get_motor_now()->m_curr_unbalance * FAC_CURRENT;
}

/**
 * Get the magnitude of the motor voltage.
 *
 * @return
 * The magnitude of the motor voltage.
 */
float mcpwm_foc_get_abs_motor_voltage(void) {
	const float vd_tmp = get_motor_now()->m_motor_state.vd;
	const float vq_tmp = get_motor_now()->m_motor_state.vq;
	return NORM2_f(vd_tmp, vq_tmp);
}

/**
 * Get the filtered magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current_filtered(void) {
	return get_motor_now()->m_motor_state.i_abs_filter;
}

/**
 * Get the motor current. The sign of this value represents the direction
 * in which the motor generates torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current_directional(void) {
	return get_motor_now()->m_motor_state.iq;
}

/**
 * Get the filtered motor current. The sign of this value represents the
 * direction in which the motor generates torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_directional_filtered(void) {
	return get_motor_now()->m_motor_state.iq_filter;
}

/**
 * Get the direct axis motor current.
 *
 * @return
 * The D axis current.
 */
float mcpwm_foc_get_id(void) {
	return get_motor_now()->m_motor_state.id;
}

/**
 * Get the quadrature axis motor current.
 *
 * @return
 * The Q axis current.
 */
float mcpwm_foc_get_iq(void) {
	return get_motor_now()->m_motor_state.iq;
}

/**
 * Get the input current to the motor controller.
 *
 * @return
 * The input current.
 */
float mcpwm_foc_get_tot_current_in(void) {
	return get_motor_now()->m_motor_state.i_bus;
}

/**
 * Get the filtered input current to the motor controller.
 *
 * @return
 * The filtered input current.
 */
float mcpwm_foc_get_tot_current_in_filtered(void) {
	return get_motor_now()->m_motor_state.i_bus; // TODO: Calculate filtered current?
}

/**
 * Set the number of steps the motor has rotated. This number is signed and
 * becomes a negative when the motor is rotating backwards.
 *
 * @param steps
 * New number of steps will be set after this call.
 *
 * @return
 * The previous tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_set_tachometer_value(int steps) {
	int val = get_motor_now()->m_tachometer;
	get_motor_now()->m_tachometer = steps;
	return val;
}

/**
 * Read the number of steps the motor has rotated. This number is signed and
 * will return a negative number when the motor is rotating backwards.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_get_tachometer_value(bool reset) {
	int val = get_motor_now()->m_tachometer;

	if (reset) {
		get_motor_now()->m_tachometer = 0;
	}

	return val;
}

/**
 * Read the absolute number of steps the motor has rotated.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_get_tachometer_abs_value(bool reset) {
	int val = get_motor_now()->m_tachometer_abs;

	if (reset) {
		get_motor_now()->m_tachometer_abs = 0;
	}

	return val;
}

/**
 * Read the motor phase.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase(void) {
	float angle = RAD2DEG_f(get_motor_now()->m_motor_state.phase);
	utils_norm_angle(&angle);
	return angle;
}

/**
 * Read the phase that the observer has calculated.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase_observer(void) {
	float angle = RAD2DEG_f(get_motor_now()->m_phase_now_observer);
	utils_norm_angle(&angle);
	return angle;
}

/**
 * Read the phase from based on the encoder.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase_encoder(void) {
	float angle = RAD2DEG_f(get_motor_now()->m_phase_now_encoder);
	utils_norm_angle(&angle);
	return angle;
}

float mcpwm_foc_get_vd(void) {
	return get_motor_now()->m_motor_state.vd;
}

float mcpwm_foc_get_vq(void) {
	return get_motor_now()->m_motor_state.vq;
}

float mcpwm_foc_get_mod_alpha_raw(void) {
	return get_motor_now()->m_motor_state.mod_alpha_raw;
}

float mcpwm_foc_get_mod_beta_raw(void) {
	return get_motor_now()->m_motor_state.mod_beta_raw;
}

float mcpwm_foc_get_mod_alpha_measured(void) {
	return get_motor_now()->m_motor_state.mod_alpha_measured;
}

float mcpwm_foc_get_mod_beta_measured(void) {
	return get_motor_now()->m_motor_state.mod_beta_measured;
}


/**
 * Lock the motor with a current and sample the voltage and current to
 * calculate the motor resistance.
 *
 * @param current
 * The locking current.
 *
 * @param samples
 * The number of samples to take.
 *
 * @param stop_after
 * Stop motor after finishing the measurement. Otherwise, the current will
 * still be applied after returning. Setting this to false is useful if you want
 * to run this function again right away, without stopping the motor in between.
 *
 * @return
 * The calculated motor resistance.
 */
float mcpwm_foc_measure_resistance(float current, int samples, bool stop_after) {
	mc_interface_lock();

	volatile motor_all_state_t *motor = get_motor_now();

	motor->m_phase_override = true;
	motor->m_phase_now_override = 0.0;
	motor->m_id_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Ramp up the current slowly
	while (fabsf(motor->m_iq_set - current) > 0.001) {
		utils_step_towards((float*)&motor->m_iq_set, current, fabsf(current) / 500.0);		
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_phase_override = false;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw((motor_all_state_t*)motor);

			timeout_configure(tout, tout_c, tout_ksw);
			mc_interface_unlock();

			return 0.0;
		}
		vTaskDelay(MS_TO_TICKS(1));
	}

	// Wait for the current to rise and the motor to lock.
	vTaskDelay(MS_TO_TICKS(100));

	// Sample
	motor->m_samples.avg_current_tot = 0.0;
	motor->m_samples.avg_voltage_tot = 0.0;
	motor->m_samples.sample_num = 0;

	int cnt = 0;
	while (motor->m_samples.sample_num < samples) {
		vTaskDelay(MS_TO_TICKS(1));
		cnt++;
		// Timeout
		if (cnt > 10000) {
			break;
		}

		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_phase_override = false;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw((motor_all_state_t*)motor);

			timeout_configure(tout, tout_c, tout_ksw);
			mc_interface_unlock();

			return 0.0;
		}
	}

	const float current_avg = motor->m_samples.avg_current_tot / (float)motor->m_samples.sample_num;
	const float voltage_avg = motor->m_samples.avg_voltage_tot / (float)motor->m_samples.sample_num;

	// Stop
	if (stop_after) {
		motor->m_id_set = 0.0;
		motor->m_iq_set = 0.0;
		motor->m_phase_override = false;
		motor->m_control_mode = CONTROL_MODE_NONE;
		motor->m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)motor);
	}

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	return voltage_avg / current_avg;
}

/**
 * Measure the motor inductance with short voltage pulses.
 *
 * @param duty
 * The duty cycle to use in the pulses.
 *
 * @param samples
 * The number of samples to average over.
 *
 * @param
 * The current that was used for this measurement.
 *
 * @return
 * The average d and q axis inductance in uH.
 */
float mcpwm_foc_measure_inductance(float duty, int samples, float *curr, float *ld_lq_diff) {
	volatile motor_all_state_t *motor = get_motor_now();

	mc_foc_sensor_mode sensor_mode_old = motor->m_conf->foc_sensor_mode;
	float f_zv_old = motor->m_conf->foc_f_zv;
	float hfi_voltage_start_old = motor->m_conf->foc_hfi_voltage_start;
	float hfi_voltage_run_old = motor->m_conf->foc_hfi_voltage_run;
	float hfi_voltage_max_old = motor->m_conf->foc_hfi_voltage_max;
	float sl_erpm_hfi_old = motor->m_conf->foc_sl_erpm_hfi;
	bool sample_v0_v7_old = motor->m_conf->foc_sample_v0_v7;
	foc_hfi_samples samples_old = motor->m_conf->foc_hfi_samples;
	bool sample_high_current_old = motor->m_conf->foc_sample_high_current;

	mc_interface_lock();
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw((motor_all_state_t*)motor);

	motor->m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HFI;
	motor->m_conf->foc_hfi_voltage_start = duty * mc_interface_get_input_voltage_filtered() * (2.0 / 3.0) * SQRT3_BY_2;
	motor->m_conf->foc_hfi_voltage_run = duty * mc_interface_get_input_voltage_filtered() * (2.0 / 3.0) * SQRT3_BY_2;
	motor->m_conf->foc_hfi_voltage_max = duty * mc_interface_get_input_voltage_filtered() * (2.0 / 3.0) * SQRT3_BY_2;
	motor->m_conf->foc_sl_erpm_hfi = 20000.0;
	motor->m_conf->foc_sample_v0_v7 = false;
	motor->m_conf->foc_hfi_samples = HFI_SAMPLES_32;
	motor->m_conf->foc_sample_high_current = false;

	if (motor->m_conf->foc_f_zv > 30.0e3) {
		motor->m_conf->foc_f_zv = 30.0e3;
	}

	mcpwm_foc_set_configuration(motor->m_conf);

	vTaskDelay(MS_TO_TICKS(1));

	timeout_reset();
	mcpwm_foc_set_duty(0.0);
	vTaskDelay(MS_TO_TICKS(1));

	int ready_cnt = 0;
	while (!motor->m_hfi.ready) {
		vTaskDelay(MS_TO_TICKS(1));
		ready_cnt++;
		if (ready_cnt > 100) {
			break;
		}
	}

	if (samples < 10) {
		samples = 10;
	}

	float l_sum = 0.0;
	float ld_lq_diff_sum = 0.0;
	float i_sum = 0.0;
	float iterations = 0.0;

	for (int i = 0;i < (samples / 10);i++) {
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw((motor_all_state_t*)motor);

			motor->m_conf->foc_sensor_mode = sensor_mode_old;
			motor->m_conf->foc_f_zv = f_zv_old;
			motor->m_conf->foc_hfi_voltage_start = hfi_voltage_start_old;
			motor->m_conf->foc_hfi_voltage_run = hfi_voltage_run_old;
			motor->m_conf->foc_hfi_voltage_max = hfi_voltage_max_old;
			motor->m_conf->foc_sl_erpm_hfi = sl_erpm_hfi_old;
			motor->m_conf->foc_sample_v0_v7 = sample_v0_v7_old;
			motor->m_conf->foc_hfi_samples = samples_old;
			motor->m_conf->foc_sample_high_current = sample_high_current_old;

			mcpwm_foc_set_configuration(motor->m_conf);

			mc_interface_unlock();

			return 0.0;
		}

		timeout_reset();
		mcpwm_foc_set_duty(0.0);
		vTaskDelay(MS_TO_TICKS(10));

		float real_bin0, imag_bin0;
		float real_bin2, imag_bin2;
		float real_bin0_i, imag_bin0_i;

		motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer, &real_bin0, &imag_bin0);
		motor->m_hfi.fft_bin2_func((float*)motor->m_hfi.buffer, &real_bin2, &imag_bin2);
		motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer_current, &real_bin0_i, &imag_bin0_i);

		l_sum += real_bin0;
		i_sum += real_bin0_i;

		// See https://vesc-project.com/comment/8338#comment-8338
		ld_lq_diff_sum += 4.0 * NORM2_f(real_bin2, imag_bin2);

		iterations++;
	}

	mcpwm_foc_set_current(0.0);

	motor->m_conf->foc_sensor_mode = sensor_mode_old;
	motor->m_conf->foc_f_zv = f_zv_old;
	motor->m_conf->foc_hfi_voltage_start = hfi_voltage_start_old;
	motor->m_conf->foc_hfi_voltage_run = hfi_voltage_run_old;
	motor->m_conf->foc_hfi_voltage_max = hfi_voltage_max_old;
	motor->m_conf->foc_sl_erpm_hfi = sl_erpm_hfi_old;
	motor->m_conf->foc_sample_v0_v7 = sample_v0_v7_old;
	motor->m_conf->foc_hfi_samples = samples_old;
	motor->m_conf->foc_sample_high_current = sample_high_current_old;

	mcpwm_foc_set_configuration(motor->m_conf);

	mc_interface_unlock();

	// The observer is more stable when the inductance is underestimated compared to overestimated,
	// so scale it by 0.8. This helps motors that start to saturate at higher currents and when
	// the hardware has problems measuring the inductance correctly. Another reason for decreasing the
	// measured value is that delays in the hardware and/or a high resistance compared to inductance
	// will cause the value to be overestimated.
	// NOTE: This used to be 0.8, but was changed to 0.9 as that works better with HFIv2 on most motors.
	float ind_scale_factor = 0.9;

	if (curr) {
		*curr = i_sum / iterations;
	}

	if (ld_lq_diff) {
		*ld_lq_diff = (ld_lq_diff_sum / iterations) * 1e6 * ind_scale_factor;
	}

	return (l_sum / iterations) * 1e6 * ind_scale_factor;
}

/**
 * Measure the motor inductance with short voltage pulses. The difference from the
 * other function is that this one will aim for a specific measurement current. It
 * will also use an appropriate switching frequency.
 *
 * @param curr_goal
 * The measurement current to aim for.
 *
 * @param samples
 * The number of samples to average over.
 *
 * @param *curr
 * The current that was used for this measurement.
 *
 * @return
 * The average d and q axis inductance in uH.
 */
float mcpwm_foc_measure_inductance_current(float curr_goal, int samples, float *curr, float *ld_lq_diff) {
	float duty_last = 0.0;
	for (float i = 0.02;i < 0.5;i *= 1.5) {
		float i_tmp;
		if (mcpwm_foc_measure_inductance(i, 10, &i_tmp, 0) == 0.0) {
			return 0.0;
		}

		duty_last = i;
		if (i_tmp >= curr_goal) {
			break;
		}
	}

	float ind = mcpwm_foc_measure_inductance(duty_last, samples, curr, ld_lq_diff);
	return ind;
}

/**
 * Automatically measure the resistance and inductance of the motor with small steps.
 *
 * @param res
 * The measured resistance in ohm.
 *
 * @param ind
 * The measured inductance in microhenry.
 *
 * @param ld_lq_diff
 * The measured difference in D axis and Q axis inductance.
 *
 * @return
 * True if the measurement succeeded, false otherwise.
 */
bool mcpwm_foc_measure_res_ind(float *res, float *ind, float *ld_lq_diff) {
	volatile motor_all_state_t *motor = get_motor_now();
	bool result = false;

	const float kp_old = motor->m_conf->foc_current_kp;
	const float ki_old = motor->m_conf->foc_current_ki;
	const float res_old = motor->m_conf->foc_motor_r;

	motor->m_conf->foc_current_kp = 0.001;
	motor->m_conf->foc_current_ki = 1.0;

	float i_last = 0.0;
	for (float i = 2.0;i < (motor->m_conf->l_current_max / 2.0);i *= 1.5) {
		float r_tmp = mcpwm_foc_measure_resistance(i, 20, false);
		if (r_tmp == 0.0) {
			motor->m_conf->foc_current_kp = kp_old;
			motor->m_conf->foc_current_ki = ki_old;
			return false;
		}
		if (i > (1.0 / r_tmp)) {
			i_last = i;
			break;
		}
	}

	if (i_last < 0.01) {
		i_last = (motor->m_conf->l_current_max / 2.0);
	}

#ifdef HW_AXIOM_FORCE_HIGH_CURRENT_MEASUREMENTS
	i_last = (motor->m_conf->l_current_max / 2.0);
#endif

	*res = mcpwm_foc_measure_resistance(i_last, 200, true);
	if (*res != 0.0) {
		motor->m_conf->foc_motor_r = *res;
		*ind = mcpwm_foc_measure_inductance_current(i_last, 200, 0, ld_lq_diff);
		if (*ind != 0.0) {
			result = true;
		}
	}

	motor->m_conf->foc_current_kp = kp_old;
	motor->m_conf->foc_current_ki = ki_old;
	motor->m_conf->foc_motor_r = res_old;

	return result;
}

/**
 * Run the motor in open loop and figure out at which angles the hall sensors are.
 *
 * @param current
 * Current to use.
 *
 * @param hall_table
 * Table to store the result to.
 *
 * @return
 * true: Success
 * false: Something went wrong
 */
bool mcpwm_foc_hall_detect(float current, uint8_t *hall_table) {
	volatile motor_all_state_t *motor = get_motor_now();

	mc_interface_lock();

	motor->m_phase_override = true;
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;

	// MTPA overrides id target
	MTPA_MODE mtpa_old = motor->m_conf->foc_mtpa_mode;
	motor->m_conf->foc_mtpa_mode = MTPA_MODE_OFF;

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Lock the motor
	motor->m_phase_now_override = 0;

	for (int i = 0;i < 1000;i++) {
		motor->m_id_set = (float)i * current / 1000.0;
		vTaskDelay(MS_TO_TICKS(1));
	}

	float sin_hall[8];
	float cos_hall[8];
	int hall_iterations[8];
	memset(sin_hall, 0, sizeof(sin_hall));
	memset(cos_hall, 0, sizeof(cos_hall));
	memset(hall_iterations, 0, sizeof(hall_iterations));

	// Forwards
	for (int i = 0;i < 3;i++) {
		for (int j = 0;j < 360;j++) {
			motor->m_phase_now_override = DEG2RAD_f(j);
			vTaskDelay(MS_TO_TICKS(5));

			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			float s, c;
			sincosf(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	// Reverse
	for (int i = 0;i < 3;i++) {
		for (int j = 360;j >= 0;j--) {
			motor->m_phase_now_override = DEG2RAD_f(j);
			vTaskDelay(MS_TO_TICKS(5));

			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			float s, c;
			sincosf(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_phase_override = false;
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw((motor_all_state_t*)motor);

	motor->m_conf->foc_mtpa_mode = mtpa_old;

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);

	int fails = 0;
	for(int i = 0;i < 8;i++) {
		if (hall_iterations[i] > 30) {
			float ang = RAD2DEG_f(atan2f(sin_hall[i], cos_hall[i]));
			utils_norm_angle(&ang);
			hall_table[i] = (uint8_t)(ang * 200.0 / 360.0);
		} else {
			hall_table[i] = 255;
			fails++;
		}
	}

	mc_interface_unlock();

	return fails == 2;
}

/**
 * Calibrate voltage and current offsets. For the observer to work at low modulation it
 * is very important to get all current and voltage offsets right. Therefore we store
 * the offsets for when the motor is undriven and when it is driven separately. The
 * motor is driven at 50% modulation on all phases when measuring the driven offset, which
 * corresponds to space-vector modulation with 0 amplitude.
 *
 * cal_undriven:
 * Calibrate undriven voltages too. This requires the motor to stand still.
 *
 * return:
 * -1: Timed out while waiting for fault code to go away.
 * 1: Success
 *
 */
int mcpwm_foc_dc_cal(bool cal_undriven) {
	// Wait max 5 seconds for DRV-fault to go away
	int cnt = 0;
	while(IS_DRV_FAULT()){
		vTaskDelay(MS_TO_TICKS(1));
		cnt++;
		if (cnt > 5000) {
			return -1;
		}
	};

	vTaskDelay(MS_TO_TICKS(1000));

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Measure driven offsets

	const float samples = 1000.0;
	float current_sum[3] = {0.0, 0.0, 0.0};
	float voltage_sum[3] = {0.0, 0.0, 0.0};

	TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);

	// Start PWM on phase 1
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM1->CCER |= TIMxCCER_MASK_CH1;

	vTaskDelay(1);

	for (float i = 0;i < samples;i++) {
		current_sum[0] += m_motor_1.m_currents_adc[0];
		voltage_sum[0] += GET_VOLT1();
		vTaskDelay(1);
	}

	// Start PWM on phase 2
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM1->CCER |= TIMxCCER_MASK_CH2;


	vTaskDelay(1);

	for (float i = 0;i < samples;i++) {
		current_sum[1] += m_motor_1.m_currents_adc[1];
		voltage_sum[1] += GET_VOLT2();
		vTaskDelay(1);
	}

	// Start PWM on phase 3
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM1->CCER |= TIMxCCER_MASK_CH3;

	vTaskDelay(1);

	for (float i = 0;i < samples;i++) {
		current_sum[2] += m_motor_1.m_currents_adc[2];
		voltage_sum[2] += GET_VOLT3();
		vTaskDelay(1);
	}

	stop_pwm_hw((motor_all_state_t*)&m_motor_1);

	m_motor_1.m_conf->foc_offsets_current[0] = current_sum[0] / samples;
	m_motor_1.m_conf->foc_offsets_current[1] = current_sum[1] / samples;
	m_motor_1.m_conf->foc_offsets_current[2] = current_sum[2] / samples;

	voltage_sum[0] /= samples;
	voltage_sum[1] /= samples;
	voltage_sum[2] /= samples;
	float v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0;

	m_motor_1.m_conf->foc_offsets_voltage[0] = voltage_sum[0] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[1] = voltage_sum[1] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[2] = voltage_sum[2] - v_avg;

	// Measure undriven offsets

	if (cal_undriven) {
		vTaskDelay(MS_TO_TICKS(10));

		voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;

		for (float i = 0;i < samples;i++) {
			v_avg = (GET_VOLT1() + GET_VOLT2() + GET_VOLT3()) / 3.0;
			voltage_sum[0] += GET_VOLT1() - v_avg;
			voltage_sum[1] += GET_VOLT2() - v_avg;
			voltage_sum[2] += GET_VOLT3() - v_avg;

			vTaskDelay(1);
		}

		stop_pwm_hw((motor_all_state_t*)&m_motor_1);

		voltage_sum[0] /= samples;
		voltage_sum[1] /= samples;
		voltage_sum[2] /= samples;

		m_motor_1.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum[0];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum[1];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum[2];

	}

	// TODO: Make sure that offsets are no more than e.g. 5%, as larger values indicate hardware problems.

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	m_dccal_done = true;

	return 1;
}

void mcpwm_foc_print_state(void) {
	commands_printf(main_uart.phandle, "Mod d:     %.2f", (double)get_motor_now()->m_motor_state.mod_d);
	commands_printf(main_uart.phandle, "Mod q:     %.2f", (double)get_motor_now()->m_motor_state.mod_q);
	commands_printf(main_uart.phandle, "Mod q flt: %.2f", (double)get_motor_now()->m_motor_state.mod_q_filter);
	commands_printf(main_uart.phandle, "Duty:      %.2f", (double)get_motor_now()->m_motor_state.duty_now);
	commands_printf(main_uart.phandle, "Vd:        %.2f", (double)get_motor_now()->m_motor_state.vd);
	commands_printf(main_uart.phandle, "Vq:        %.2f", (double)get_motor_now()->m_motor_state.vq);
	commands_printf(main_uart.phandle, "Phase:     %.2f", (double)get_motor_now()->m_motor_state.phase);
	commands_printf(main_uart.phandle, "V_alpha:   %.2f", (double)get_motor_now()->m_motor_state.v_alpha);
	commands_printf(main_uart.phandle, "V_beta:    %.2f", (double)get_motor_now()->m_motor_state.v_beta);
	commands_printf(main_uart.phandle, "id:        %.2f", (double)get_motor_now()->m_motor_state.id);
	commands_printf(main_uart.phandle, "iq:        %.2f", (double)get_motor_now()->m_motor_state.iq);
	commands_printf(main_uart.phandle, "id_filter: %.2f", (double)get_motor_now()->m_motor_state.id_filter);
	commands_printf(main_uart.phandle, "iq_filter: %.2f", (double)get_motor_now()->m_motor_state.iq_filter);
	commands_printf(main_uart.phandle, "id_target: %.2f", (double)get_motor_now()->m_motor_state.id_target);
	commands_printf(main_uart.phandle, "iq_target: %.2f", (double)get_motor_now()->m_motor_state.iq_target);
	commands_printf(main_uart.phandle, "i_abs:     %.2f", (double)get_motor_now()->m_motor_state.i_abs);
	commands_printf(main_uart.phandle, "i_abs_flt: %.2f", (double)get_motor_now()->m_motor_state.i_abs_filter);
	commands_printf(main_uart.phandle, "Obs_x1:    %.2f", (double)get_motor_now()->m_observer_state.x1);
	commands_printf(main_uart.phandle, "Obs_x2:    %.2f", (double)get_motor_now()->m_observer_state.x2);
	commands_printf(main_uart.phandle, "lambda_est:%.2f", (double)get_motor_now()->m_observer_state.lambda_est);
	commands_printf(main_uart.phandle, "vd_int:    %.2f", (double)get_motor_now()->m_motor_state.vd_int);
	commands_printf(main_uart.phandle, "vq_int:    %.2f", (double)get_motor_now()->m_motor_state.vq_int);
	commands_printf(main_uart.phandle, "off_delay: %.2f", (double)get_motor_now()->m_current_off_delay);
}

float mcpwm_foc_get_last_adc_isr_duration(void) {
	return m_last_adc_isr_duration;
}

void mcpwm_foc_tim_sample_int_handler(void) {
	if (m_init_done) {
		// Generate COM event here for synchronization
//		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
//		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);

//		virtual_motor_int_handler(
//				m_motor_1.m_motor_state.v_alpha,
//				m_motor_1.m_motor_state.v_beta);
	}
}

void mcpwm_foc_adc_int_handler(void *p, uint32_t flags) {
	(void)p;
	(void)flags;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	static int skip = 0;
	if (++skip == FOC_CONTROL_LOOP_FREQ_DIVIDER) {
		skip = 0;
	} else {
		return;
	}

	//uint32_t t_start = timer_time_now();

	uint32_t cycles = *DWT_CYCCNT;

	bool is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);
	int norm_curr_ofs = 0;

#ifdef HW_HAS_DUAL_MOTORS
	bool is_second_motor = is_v7;
	norm_curr_ofs = is_second_motor ? 3 : 0;
	motor_all_state_t *motor_now = is_second_motor ? (motor_all_state_t*)&m_motor_2 : (motor_all_state_t*)&m_motor_1;
	motor_all_state_t *motor_other = is_second_motor ? (motor_all_state_t*)&m_motor_1 : (motor_all_state_t*)&m_motor_2;
	m_isr_motor = is_second_motor ? 2 : 1;
#ifdef HW_HAS_3_SHUNTS
	volatile TIM_TypeDef *tim = is_second_motor ? TIM8 : TIM1;
#endif
#else
	motor_all_state_t *motor_other = (motor_all_state_t*)&m_motor_1;
	motor_all_state_t *motor_now = (motor_all_state_t*)&m_motor_1;;
	m_isr_motor = 1;
#ifdef HW_HAS_3_SHUNTS
	volatile TIM_TypeDef *tim = TIM1;
#endif
#endif

	mc_configuration *conf_now = motor_now->m_conf;
	mc_configuration *conf_other = motor_other->m_conf;

	// Update modulation for V7 and collect current samples. This is used by the HFI.
	if (motor_other->m_duty_next_set) {
		motor_other->m_duty_next_set = false;

		float curr0 = ((float)GET_CURRENT1() - conf_other->foc_offsets_current[0]) * FAC_CURRENT;
		float curr1 = ((float)GET_CURRENT2() - conf_other->foc_offsets_current[1]) * FAC_CURRENT;

		TIMER_UPDATE_DUTY_M1(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);

		motor_other->m_i_alpha_sample_next = curr0;
		motor_other->m_i_beta_sample_next = ONE_BY_SQRT3 * curr0 + TWO_BY_SQRT3 * curr1;
	}

#ifndef HW_HAS_DUAL_MOTORS
#ifdef HW_HAS_PHASE_SHUNTS
	if (!conf_now->foc_sample_v0_v7 && is_v7) {
		return;
	}
#else
	if (is_v7) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		return;
	}
#endif
#endif

	// Reset the watchdog
	timeout_feed_WDT(THREAD_MCPWM);


#ifdef HW_HAS_DUAL_MOTORS
	float curr0 = 0;
	float curr1 = 0;

	if (is_second_motor) {
		curr0 = GET_CURRENT1_M2();
		curr1 = GET_CURRENT2_M2();
	} else {
		curr0 = GET_CURRENT1();
		curr1 = GET_CURRENT2();
	}
#else
	float curr0 = GET_CURRENT1();
	float curr1 = GET_CURRENT2();
#ifdef HW_HAS_DUAL_PARALLEL
	curr0 += GET_CURRENT1_M2();
	curr1 += GET_CURRENT2_M2();
#endif
#endif

#ifdef HW_HAS_3_SHUNTS
#ifdef HW_HAS_DUAL_MOTORS
	float curr2 = is_second_motor ? GET_CURRENT3_M2() : GET_CURRENT3();
#else
	float curr2 = GET_CURRENT3();
#ifdef HW_HAS_DUAL_PARALLEL
	curr2 += GET_CURRENT3_M2();
#endif
#endif
#endif

	motor_now->m_currents_adc[0] = curr0;
	motor_now->m_currents_adc[1] = curr1;
#ifdef HW_HAS_3_SHUNTS
	motor_now->m_currents_adc[2] = curr2;
#else
	motor_now->m_currents_adc[2] = 0.0;
#endif

	curr0 -= conf_now->foc_offsets_current[0];
	curr1 -= conf_now->foc_offsets_current[1];
#ifdef HW_HAS_3_SHUNTS
	curr2 -= conf_now->foc_offsets_current[2];
	motor_now->m_curr_unbalance = curr0 + curr1 + curr2;
#endif

	ADC_curr_norm_value[0 + norm_curr_ofs] = curr0;
	ADC_curr_norm_value[1 + norm_curr_ofs] = curr1;
#ifdef HW_HAS_3_SHUNTS
	ADC_curr_norm_value[2 + norm_curr_ofs] = curr2;
#else
	ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
#endif

	// Use the best current samples depending on the modulation state.
#ifdef HW_HAS_3_SHUNTS
	if (conf_now->foc_sample_high_current) {
		// High current sampling mode. Choose the lower currents to derive the highest one
		// in order to be able to measure higher currents.
		const float i0_abs = fabsf(ADC_curr_norm_value[0 + norm_curr_ofs]);
		const float i1_abs = fabsf(ADC_curr_norm_value[1 + norm_curr_ofs]);
		const float i2_abs = fabsf(ADC_curr_norm_value[2 + norm_curr_ofs]);

		if (i0_abs > i1_abs && i0_abs > i2_abs) {
			ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
		} else if (i1_abs > i0_abs && i1_abs > i2_abs) {
			ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
		} else if (i2_abs > i0_abs && i2_abs > i1_abs) {
			ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
		}
	} else {
#ifdef HW_HAS_PHASE_SHUNTS
		if (is_v7) {
			if (tim->CCR1 > 500 && tim->CCR2 > 500) {
				// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
				// This is especially important when using HFI.
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			} else {
				if (tim->CCR1 < tim->CCR2 && tim->CCR1 < tim->CCR3) {
					ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR2 < tim->CCR1 && tim->CCR2 < tim->CCR3) {
					ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR3 < tim->CCR1 && tim->CCR3 < tim->CCR2) {
					ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
				}
			}
		} else {
			if (tim->CCR1 < (tim->ARR - 500) && tim->CCR2 < (tim->ARR - 500)) {
				// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
				// This is especially important when using HFI.
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			} else {
				if (tim->CCR1 > tim->CCR2 && tim->CCR1 > tim->CCR3) {
					ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR2 > tim->CCR1 && tim->CCR2 > tim->CCR3) {
					ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR3 > tim->CCR1 && tim->CCR3 > tim->CCR2) {
					ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
				}
			}
		}
#else
		if (tim->CCR1 < (tim->ARR - 500) && tim->CCR2 < (tim->ARR - 500)) {
			// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
			// This is especially important when using HFI.
			ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
		} else {
			if (tim->CCR1 > tim->CCR2 && tim->CCR1 > tim->CCR3) {
				ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
			} else if (tim->CCR2 > tim->CCR1 && tim->CCR2 > tim->CCR3) {
				ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
			} else if (tim->CCR3 > tim->CCR1 && tim->CCR3 > tim->CCR2) {
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			}
		}
#endif
	}
#endif

	float ia = ADC_curr_norm_value[0 + norm_curr_ofs] * FAC_CURRENT;
	float ib = ADC_curr_norm_value[1 + norm_curr_ofs] * FAC_CURRENT;
//	float ic = -(ia + ib);

#ifdef HW_HAS_PHASE_SHUNTS
	float dt;
	if (conf_now->foc_sample_v0_v7) {
		dt = 1.0 / conf_now->foc_f_zv;
	} else {
		dt = 1.0 / (conf_now->foc_f_zv / 2.0);
	}
#else
	float dt = 1.0 / (conf_now->foc_f_zv / 2.0);
#endif

	// This has to be done for the skip function to have any chance at working with the
	// observer and control loops.
	// TODO: Test this.
	dt *= (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;

	UTILS_LP_FAST(motor_now->m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1);


	if (motor_now->m_state == MC_STATE_RUNNING) {
		// Clarke transform assuming balanced currents
		motor_now->m_motor_state.i_alpha = ia;
		motor_now->m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

		motor_now->m_i_alpha_sample_with_offset = motor_now->m_motor_state.i_alpha;
		motor_now->m_i_beta_sample_with_offset = motor_now->m_motor_state.i_beta;

		if (motor_now->m_i_alpha_beta_has_offset) {
			motor_now->m_motor_state.i_alpha = 0.5 * (motor_now->m_motor_state.i_alpha + motor_now->m_i_alpha_sample_next);
			motor_now->m_motor_state.i_beta = 0.5 * (motor_now->m_motor_state.i_beta + motor_now->m_i_beta_sample_next);
			motor_now->m_i_alpha_beta_has_offset = false;
		}

		const float duty_now = motor_now->m_motor_state.duty_now;
		const float duty_abs = fabsf(duty_now);
		const float vq_now = motor_now->m_motor_state.vq;
		const float speed_fast_now = motor_now->m_pll_speed;

		float id_set_tmp = motor_now->m_id_set;
		float iq_set_tmp = motor_now->m_iq_set;
		motor_now->m_motor_state.max_duty = conf_now->l_max_duty;

		if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			utils_truncate_number_abs(&iq_set_tmp, -conf_now->lo_current_min);
		}

		UTILS_LP_FAST(motor_now->m_duty_abs_filtered, duty_abs, 0.01);
		utils_truncate_number_abs((float*)&motor_now->m_duty_abs_filtered, 1.0);

		UTILS_LP_FAST(motor_now->m_duty_filtered, duty_now, 0.01);
		utils_truncate_number_abs((float*)&motor_now->m_duty_filtered, 1.0);

		float duty_set = motor_now->m_duty_cycle_set;
		bool control_duty = motor_now->m_control_mode == CONTROL_MODE_DUTY ||
				motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY ||
				motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE;

		// Short all phases (duty=0) the moment the direction or modulation changes sign. That will avoid
		// active braking or changing direction. Keep all phases shorted (duty == 0) until the
		// braking current reaches the set or maximum value, then go back to current control
		// mode. Stay in duty=0 for at least 10 cycles to avoid jumping in and out of that mode rapidly
		// around the threshold.
		if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			if ((SIGN(speed_fast_now) != SIGN(motor_now->m_br_speed_before) ||
					SIGN(vq_now) != SIGN(motor_now->m_br_vq_before) ||
					fabsf(motor_now->m_duty_filtered) < 0.001 || motor_now->m_br_no_duty_samples < 10) &&
					motor_now->m_motor_state.i_abs_filter < fabsf(iq_set_tmp)) {
				control_duty = true;
				duty_set = 0.0;
				motor_now->m_br_no_duty_samples = 0;
			} else if (motor_now->m_br_no_duty_samples < 10) {
				control_duty = true;
				duty_set = 0.0;
				motor_now->m_br_no_duty_samples++;
			}
		} else {
			motor_now->m_br_no_duty_samples = 0;
		}

		motor_now->m_br_speed_before = speed_fast_now;
		motor_now->m_br_vq_before = vq_now;

		// Brake when set ERPM is below min ERPM
		if (motor_now->m_control_mode == CONTROL_MODE_SPEED &&
				fabsf(motor_now->m_speed_pid_set_rpm) < conf_now->s_pid_min_erpm) {
			control_duty = true;
			duty_set = 0.0;
		}

		// Reset integrator when leaving duty cycle mode, as the windup protection is not too fast. Making
		// better windup protection is probably better, but not easy.
		if (!control_duty && motor_now->m_was_control_duty) {
			motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;
			if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
					conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
				motor_now->m_motor_state.vq_int -= motor_now->m_motor_state.speed_rad_s * conf_now->foc_motor_flux_linkage;
			}
		}
		motor_now->m_was_control_duty = control_duty;

		if (!control_duty) {
			motor_now->m_duty_i_term = motor_now->m_motor_state.iq / conf_now->lo_current_max;
		}

		if (control_duty) {
			// Duty cycle control
			if (fabsf(duty_set) < (duty_abs - 0.05) ||
					(SIGN(motor_now->m_motor_state.vq) * motor_now->m_motor_state.iq) < conf_now->lo_current_min) {
				// Truncating the duty cycle here would be dangerous, so run a PID controller.

				// Reset the integrator in duty mode to not increase the duty if the load suddenly changes. In braking
				// mode this would cause a discontinuity, so there we want to keep the value of the integrator.
				if (motor_now->m_control_mode == CONTROL_MODE_DUTY) {
					if (duty_now > 0.0) {
						if (motor_now->m_duty_i_term > 0.0) {
							motor_now->m_duty_i_term = 0.0;
						}
					} else {
						if (motor_now->m_duty_i_term < 0.0) {
							motor_now->m_duty_i_term = 0.0;
						}
					}
				}

				// Compensation for supply voltage variations
				float scale = 1.0 / motor_now->m_motor_state.v_bus;

				// Compute error
				float error = duty_set - motor_now->m_motor_state.duty_now;

				// Compute parameters
				float p_term = error * conf_now->foc_duty_dowmramp_kp * scale;
				motor_now->m_duty_i_term += error * (conf_now->foc_duty_dowmramp_ki * dt) * scale;

				// I-term wind-up protection
				utils_truncate_number((float*)&motor_now->m_duty_i_term, -1.0, 1.0);

				// Calculate output
				float output = p_term + motor_now->m_duty_i_term;
				utils_truncate_number(&output, -1.0, 1.0);
				iq_set_tmp = output * conf_now->lo_current_max;
			} else {
				// If the duty cycle is less than or equal to the set duty cycle just limit
				// the modulation and use the maximum allowed current.
				motor_now->m_duty_i_term = motor_now->m_motor_state.iq / conf_now->lo_current_max;
				motor_now->m_motor_state.max_duty = duty_set;
				if (duty_set > 0.0) {
					iq_set_tmp = conf_now->lo_current_max;
				} else {
					iq_set_tmp = -conf_now->lo_current_max;
				}
			}
		} else if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			// Braking
			iq_set_tmp = -SIGN(speed_fast_now) * fabsf(iq_set_tmp);
		}

		// Set motor phase
		{
			if (!motor_now->m_phase_override) {
				foc_observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
						motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta,
						dt, &(motor_now->m_observer_state), &motor_now->m_phase_now_observer, motor_now);

				// Compensate from the phase lag caused by the switching frequency. This is important for motors
				// that run on high ERPM compared to the switching frequency.
				motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * (0.5 + conf_now->foc_observer_offset);
				utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);
			}

			switch (conf_now->foc_sensor_mode) {
			case FOC_SENSOR_MODE_ENCODER:

				break;
			case FOC_SENSOR_MODE_HALL:
				motor_now->m_phase_now_observer = foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now,
						utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			case FOC_SENSOR_MODE_SENSORLESS:
				if (motor_now->m_phase_observer_override) {
					motor_now->m_motor_state.phase = motor_now->m_phase_now_observer_override;
					motor_now->m_observer_state.x1 = motor_now->m_observer_x1_override;
					motor_now->m_observer_state.x2 = motor_now->m_observer_x2_override;
					iq_set_tmp += conf_now->foc_sl_openloop_boost_q * SIGN(iq_set_tmp);
					if (conf_now->foc_sl_openloop_max_q > conf_now->cc_min_current) {
						utils_truncate_number_abs(&iq_set_tmp, conf_now->foc_sl_openloop_max_q);
					}
				} else {
					motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;

			case FOC_SENSOR_MODE_HFI_START:
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;

				if (motor_now->m_phase_observer_override) {
					motor_now->m_hfi.est_done_cnt = 0;
					motor_now->m_hfi.flip_cnt = 0;

					motor_now->m_min_rpm_hyst_timer = 0.0;
					motor_now->m_min_rpm_timer = 0.0;
					motor_now->m_phase_observer_override = false;
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;

			case FOC_SENSOR_MODE_HFI:
			case FOC_SENSOR_MODE_HFI_V2:
			case FOC_SENSOR_MODE_HFI_V3:
			case FOC_SENSOR_MODE_HFI_V4:
			case FOC_SENSOR_MODE_HFI_V5:
				if (fabsf(RADPS2RPM_f(motor_now->m_speed_est_fast)) > conf_now->foc_sl_erpm_hfi) {
					motor_now->m_hfi.observer_zero_time = 0;
				} else {
					motor_now->m_hfi.observer_zero_time += dt;
				}

				if (motor_now->m_hfi.observer_zero_time < conf_now->foc_hfi_obs_ovr_sec) {
					motor_now->m_hfi.angle = motor_now->m_phase_now_observer;
					motor_now->m_hfi.double_integrator = -motor_now->m_speed_est_fast;
				}

				motor_now->m_motor_state.phase = foc_correct_encoder(
						motor_now->m_phase_now_observer,
						motor_now->m_hfi.angle,
						motor_now->m_speed_est_fast,
						conf_now->foc_sl_erpm_hfi,
						motor_now);

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			}

			if (motor_now->m_control_mode == CONTROL_MODE_HANDBRAKE) {
				// Force the phase to 0 in handbrake mode so that the current simply locks the rotor.
				motor_now->m_motor_state.phase = 0.0;
			} else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP ||
					motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY) {
				motor_now->m_openloop_angle += dt * motor_now->m_openloop_speed;
				utils_norm_angle_rad((float*)&motor_now->m_openloop_angle);
				motor_now->m_motor_state.phase = motor_now->m_openloop_angle;
			} else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE ||
					motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE) {
				motor_now->m_motor_state.phase = motor_now->m_openloop_phase;
			}

			if (motor_now->m_phase_override) {
				motor_now->m_motor_state.phase = motor_now->m_phase_now_override;
			}

			utils_fast_sincos_better(motor_now->m_motor_state.phase,
					(float*)&motor_now->m_motor_state.phase_sin,
					(float*)&motor_now->m_motor_state.phase_cos);
		}

		// Apply MTPA. See: https://github.com/vedderb/bldc/pull/179
		const float ld_lq_diff = conf_now->foc_motor_ld_lq_diff;
		if (conf_now->foc_mtpa_mode != MTPA_MODE_OFF && ld_lq_diff != 0.0) {
			const float lambda = conf_now->foc_motor_flux_linkage;

			float iq_ref = iq_set_tmp;
			if (conf_now->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED) {
				iq_ref = utils_min_abs(iq_set_tmp, motor_now->m_motor_state.iq_filter);
			}

			id_set_tmp = (lambda - sqrtf(SQ(lambda) + 8.0 * SQ(ld_lq_diff * iq_ref))) / (4.0 * ld_lq_diff);
			iq_set_tmp = SIGN(iq_set_tmp) * sqrtf(SQ(iq_set_tmp) - SQ(id_set_tmp));
		}

		const float mod_q = motor_now->m_motor_state.mod_q_filter;

		// Running FW from the 1 khz timer seems fast enough.
//		run_fw(motor_now, dt);
		id_set_tmp -= motor_now->m_i_fw_set;
		iq_set_tmp -= SIGN(mod_q) * motor_now->m_i_fw_set * conf_now->foc_fw_q_current_factor;

		// Apply current limits
		// TODO: Consider D axis current for the input current as well.
		if (mod_q > 0.001) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_min / mod_q, conf_now->lo_in_current_max / mod_q);
		} else if (mod_q < -0.001) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_max / mod_q, conf_now->lo_in_current_min / mod_q);
		}

		if (mod_q > 0.0) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_current_min, conf_now->lo_current_max);
		} else {
			utils_truncate_number(&iq_set_tmp, -conf_now->lo_current_max, -conf_now->lo_current_min);
		}

		float current_max_abs = fabsf(utils_max_abs(conf_now->lo_current_max, conf_now->lo_current_min));
		utils_truncate_number_abs(&id_set_tmp, current_max_abs);
		utils_truncate_number_abs(&iq_set_tmp, sqrtf(SQ(current_max_abs) - SQ(id_set_tmp)));

		motor_now->m_motor_state.id_target = id_set_tmp;
		motor_now->m_motor_state.iq_target = iq_set_tmp;

		control_current(motor_now, dt);
	} else {
		// Motor is not running

		// The current is 0 when the motor is undriven
		motor_now->m_motor_state.i_alpha = 0.0;
		motor_now->m_motor_state.i_beta = 0.0;
		motor_now->m_motor_state.id = 0.0;
		motor_now->m_motor_state.iq = 0.0;
		motor_now->m_motor_state.id_filter = 0.0;
		motor_now->m_motor_state.iq_filter = 0.0;
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
		GET_INPUT_CURRENT_OFFSET(); // TODO: should this be done here?
#endif
		motor_now->m_motor_state.i_bus = 0.0;
		motor_now->m_motor_state.i_abs = 0.0;
		motor_now->m_motor_state.i_abs_filter = 0.0;

		// Track back emf
		update_valpha_vbeta(motor_now, 0.0, 0.0);

		// Run observer
		foc_observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
						motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta,
						dt, &(motor_now->m_observer_state), 0, motor_now);
		motor_now->m_phase_now_observer = utils_fast_atan2(motor_now->m_x2_prev + motor_now->m_observer_state.x2,
														   motor_now->m_x1_prev + motor_now->m_observer_state.x1);

		// The observer phase offset has to be added here as well, with 0.5 switching cycles offset
		// compared to when running. Otherwise going from undriven to driven causes a current
		// spike.
		motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * conf_now->foc_observer_offset;
		utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);

		motor_now->m_x1_prev = motor_now->m_observer_state.x1;
		motor_now->m_x2_prev = motor_now->m_observer_state.x2;

		// Set motor phase
		{
			switch (conf_now->foc_sensor_mode) {
			case FOC_SENSOR_MODE_ENCODER:
				motor_now->m_motor_state.phase = foc_correct_encoder(
						motor_now->m_phase_now_observer,
						motor_now->m_phase_now_encoder,
						motor_now->m_speed_est_fast,
						conf_now->foc_sl_erpm,
						motor_now);
				break;
			case FOC_SENSOR_MODE_HALL:
				motor_now->m_phase_now_observer = foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now,
						utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				break;
			case FOC_SENSOR_MODE_SENSORLESS:
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				break;
			case FOC_SENSOR_MODE_HFI:
			case FOC_SENSOR_MODE_HFI_V2:
			case FOC_SENSOR_MODE_HFI_V3:
			case FOC_SENSOR_MODE_HFI_V4:
			case FOC_SENSOR_MODE_HFI_V5:
			case FOC_SENSOR_MODE_HFI_START:{
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				if (fabsf(RADPS2RPM_f(motor_now->m_pll_speed)) < (conf_now->foc_sl_erpm_hfi * 1.1)) {
					motor_now->m_hfi.est_done_cnt = 0;
					motor_now->m_hfi.flip_cnt = 0;
				}
			} break;

			}

			utils_fast_sincos_better(motor_now->m_motor_state.phase,
					(float*)&motor_now->m_motor_state.phase_sin,
					(float*)&motor_now->m_motor_state.phase_cos);
		}

		// HFI Restore
		CURRENT_FILTER_ON();
		motor_now->m_hfi.ind = 0;
		motor_now->m_hfi.ready = false;
		motor_now->m_hfi.double_integrator = 0.0;
		motor_now->m_hfi.is_samp_n = false;
		motor_now->m_hfi.prev_sample = 0.0;
		motor_now->m_hfi.angle = motor_now->m_motor_state.phase;

		float s = motor_now->m_motor_state.phase_sin;
		float c = motor_now->m_motor_state.phase_cos;

		// Park transform
		float vd_tmp = c * motor_now->m_motor_state.v_alpha + s * motor_now->m_motor_state.v_beta;
		float vq_tmp = c * motor_now->m_motor_state.v_beta  - s * motor_now->m_motor_state.v_alpha;

		UTILS_NAN_ZERO(motor_now->m_motor_state.vd);
		UTILS_NAN_ZERO(motor_now->m_motor_state.vq);

		UTILS_LP_FAST(motor_now->m_motor_state.vd, vd_tmp, 0.2);
		UTILS_LP_FAST(motor_now->m_motor_state.vq, vq_tmp, 0.2);

		// Set the current controller integrator to the BEMF voltage to avoid
		// a current spike when the motor is driven again. Notice that we have
		// to take decoupling into account.
		motor_now->m_motor_state.vd_int = motor_now->m_motor_state.vd;
		motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;

		if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
				conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
			motor_now->m_motor_state.vq_int -= motor_now->m_motor_state.speed_rad_s * conf_now->foc_motor_flux_linkage;
		}

		// Update corresponding modulation
		/* voltage_normalize = 1/(2/3*V_bus) */
		const float voltage_normalize = 1.5 / motor_now->m_motor_state.v_bus;

		motor_now->m_motor_state.mod_d = motor_now->m_motor_state.vd * voltage_normalize;
		motor_now->m_motor_state.mod_q = motor_now->m_motor_state.vq * voltage_normalize;
		UTILS_NAN_ZERO(motor_now->m_motor_state.mod_q_filter);
		UTILS_LP_FAST(motor_now->m_motor_state.mod_q_filter, motor_now->m_motor_state.mod_q, 0.2);
		utils_truncate_number_abs((float*)&motor_now->m_motor_state.mod_q_filter, 1.0);
	}

	// Calculate duty cycle
	motor_now->m_motor_state.duty_now = SIGN(motor_now->m_motor_state.vq) *
			NORM2_f(motor_now->m_motor_state.mod_d, motor_now->m_motor_state.mod_q) * TWO_BY_SQRT3;

	float phase_for_speed_est = 0.0;
	switch (conf_now->foc_speed_soure) {
	case SPEED_SRC_CORRECTED:
		phase_for_speed_est = motor_now->m_motor_state.phase;
		break;
	case SPEED_SRC_OBSERVER:
		phase_for_speed_est = motor_now->m_phase_now_observer;
		break;
	};

	// Run PLL for speed estimation
	foc_pll_run(phase_for_speed_est, dt, &motor_now->m_pll_phase, &motor_now->m_pll_speed, conf_now);
	motor_now->m_motor_state.speed_rad_s = motor_now->m_pll_speed;

	// Low latency speed estimation, for e.g. HFI and speed control.
	{
		float diff = utils_angle_difference_rad(phase_for_speed_est, motor_now->m_phase_before_speed_est);
		utils_truncate_number(&diff, -M_PI / 3.0, M_PI / 3.0);

		UTILS_LP_FAST(motor_now->m_speed_est_fast, diff / dt, 0.01);
		UTILS_NAN_ZERO(motor_now->m_speed_est_fast);

		UTILS_LP_FAST(motor_now->m_speed_est_faster, diff / dt, 0.2);
		UTILS_NAN_ZERO(motor_now->m_speed_est_faster);

		// pll wind-up protection
		utils_truncate_number_abs((float*)&motor_now->m_pll_speed, fabsf(motor_now->m_speed_est_fast) * 3.0);

		motor_now->m_phase_before_speed_est = phase_for_speed_est;
	}

	// Update tachometer (resolution = 60 deg as for BLDC)
	float ph_tmp = motor_now->m_motor_state.phase;
	utils_norm_angle_rad(&ph_tmp);
	int step = (int)floorf((ph_tmp + M_PI) / (2.0 * M_PI) * 6.0);
	utils_truncate_number_int(&step, 0, 5);
	int diff = step - motor_now->m_tacho_step_last;
	motor_now->m_tacho_step_last = step;

	if (diff > 3) {
		diff -= 6;
	} else if (diff < -2) {
		diff += 6;
	}

	motor_now->m_tachometer += diff;
	motor_now->m_tachometer_abs += abs(diff);

	// Track position control angle
	float angle_now = 0.0;

	angle_now = RAD2DEG_f(motor_now->m_motor_state.phase);

	utils_norm_angle(&angle_now);

	if (conf_now->p_pid_ang_div > 0.98 && conf_now->p_pid_ang_div < 1.02) {
		motor_now->m_pos_pid_now = angle_now;
	} else {
		if (angle_now < 90.0 && motor_now->m_pid_div_angle_last > 270.0) {
			motor_now->m_pid_div_angle_accumulator += 360.0 / conf_now->p_pid_ang_div;
			utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
		} else if (angle_now > 270.0 && motor_now->m_pid_div_angle_last < 90.0) {
			motor_now->m_pid_div_angle_accumulator -= 360.0 / conf_now->p_pid_ang_div;
			utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
		}

		motor_now->m_pid_div_angle_last = angle_now;

		motor_now->m_pos_pid_now = motor_now->m_pid_div_angle_accumulator + angle_now / conf_now->p_pid_ang_div;
		utils_norm_angle((float*)&motor_now->m_pos_pid_now);
	}



	mc_interface_mc_timer_isr(false);

	m_isr_motor = 0;
	m_last_adc_isr_duration = (*DWT_CYCCNT - cycles);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	//m_last_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}

// Private functions

static void timer_update(motor_all_state_t *motor, float dt) {
	foc_run_fw(motor, dt);

	const mc_configuration *conf_now = motor->m_conf;

	// Calculate temperature-compensated parameters here
	if (mc_interface_temp_motor_filtered() > -30.0) {
		float comp_fact = 1.0 + 0.00386 * (mc_interface_temp_motor_filtered() - conf_now->foc_temp_comp_base_temp);
		motor->m_res_temp_comp = conf_now->foc_motor_r * comp_fact;
		motor->m_current_ki_temp_comp = conf_now->foc_current_ki * comp_fact;
	} else {
		motor->m_res_temp_comp = conf_now->foc_motor_r;
		motor->m_current_ki_temp_comp = conf_now->foc_current_ki;
	}

	// Check if it is time to stop the modulation. Notice that modulation is kept on as long as there is
	// field weakening current.
	utils_sys_lock_cnt();
	utils_step_towards((float*)&motor->m_current_off_delay, 0.0, dt);
	if (!motor->m_phase_override && motor->m_state == MC_STATE_RUNNING &&
			(motor->m_control_mode == CONTROL_MODE_CURRENT ||
					motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
					motor->m_control_mode == CONTROL_MODE_HANDBRAKE ||
					motor->m_control_mode == CONTROL_MODE_OPENLOOP ||
					motor->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE)) {

		// This is required to allow releasing the motor when cc_min_current is 0
		float min_current = conf_now->cc_min_current;
		if (min_current < 0.001 && get_motor_now()->m_motor_released) {
			min_current = 0.001;
		}

		if (fabsf(motor->m_iq_set) < min_current &&
				fabsf(motor->m_id_set) < min_current &&
				motor->m_i_fw_set < min_current &&
				motor->m_current_off_delay < dt) {
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw(motor);
		}
	}
	utils_sys_unlock_cnt();

	// Use this to study the openloop timers under experiment plot
#if 0
	{
		static bool plot_started = false;
		static int plot_div = 0;
		static float plot_int = 0.0;
		static int get_fw_version_cnt = 0;

		if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
			get_fw_version_cnt = commands_get_fw_version_sent_cnt();
			plot_started = false;
		}

		plot_div++;
		if (plot_div >= 10) {
			plot_div = 0;
			if (!plot_started) {
				plot_started = true;
				commands_init_plot("Time", "Val");
				commands_plot_add_graph("m_min_rpm_timer");
				commands_plot_add_graph("m_min_rpm_hyst_timer");
			}

			commands_plot_set_graph(0);
			commands_send_plot_points(plot_int, motor->m_min_rpm_timer);
			commands_plot_set_graph(1);
			commands_send_plot_points(plot_int, motor->m_min_rpm_hyst_timer);
			plot_int++;
		}
	}
#endif

	// Use this to study the observer state in a XY-plot
#if 0
	{
		static bool plot_started = false;
		static int plot_div = 0;
		static int get_fw_version_cnt = 0;

		if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
			get_fw_version_cnt = commands_get_fw_version_sent_cnt();
			plot_started = false;
		}

		plot_div++;
		if (plot_div >= 10) {
			plot_div = 0;
			if (!plot_started) {
				plot_started = true;
				commands_init_plot("X1", "X2");
				commands_plot_add_graph("Observer");
				commands_plot_add_graph("Observer Mag");
			}

			commands_plot_set_graph(0);
			commands_send_plot_points(m_motor_1.m_observer_x1, m_motor_1.m_observer_x2);
			float mag = NORM2_f(m_motor_1.m_observer_x1, m_motor_1.m_observer_x2);
			commands_plot_set_graph(1);
			commands_send_plot_points(0.0, mag);
		}
	}
#endif

	float t_lock = conf_now->foc_sl_openloop_time_lock;
	float t_ramp = conf_now->foc_sl_openloop_time_ramp;
	float t_const = conf_now->foc_sl_openloop_time;

	float openloop_current = fabsf(motor->m_motor_state.iq_filter);
	openloop_current += conf_now->foc_sl_openloop_boost_q;
	if (conf_now->foc_sl_openloop_max_q > 0.0) {
		utils_truncate_number(&openloop_current, 0.0, conf_now->foc_sl_openloop_boost_q);
	}

	float openloop_rpm_max = utils_map(openloop_current,
			0.0, conf_now->l_current_max,
			conf_now->foc_openloop_rpm_low * conf_now->foc_openloop_rpm,
			conf_now->foc_openloop_rpm);

	utils_truncate_number_abs(&openloop_rpm_max, conf_now->foc_openloop_rpm);

	float openloop_rpm = openloop_rpm_max;
	if (conf_now->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER) {
		float time_fwd = t_lock + t_ramp + t_const - motor->m_min_rpm_timer;
		if (time_fwd < t_lock) {
			openloop_rpm = 0.0;
		} else if (time_fwd < (t_lock + t_ramp)) {
			openloop_rpm = utils_map(time_fwd, t_lock,
					t_lock + t_ramp, 0.0, openloop_rpm);
		}
	}

	utils_truncate_number_abs(&openloop_rpm, openloop_rpm_max);

	float add_min_speed = 0.0;
	if (motor->m_motor_state.duty_now > 0.0) {
		add_min_speed = RPM2RADPS_f(openloop_rpm) * dt;
	} else {
		add_min_speed = -RPM2RADPS_f(openloop_rpm) * dt;
	}

	// Open loop encoder angle for when the index is not found
	motor->m_phase_now_encoder_no_index += add_min_speed;
	utils_norm_angle_rad((float*)&motor->m_phase_now_encoder_no_index);

	if (fabsf(motor->m_pll_speed) < RPM2RADPS_f(openloop_rpm_max) &&
			motor->m_min_rpm_hyst_timer < conf_now->foc_sl_openloop_hyst) {
		motor->m_min_rpm_hyst_timer += dt;
	} else if (motor->m_min_rpm_hyst_timer > 0.0) {
		motor->m_min_rpm_hyst_timer -= dt;
	}

	// Don't use this in brake mode.
	if (motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
			(motor->m_state == MC_STATE_RUNNING && fabsf(motor->m_motor_state.duty_now) < 0.001)) {
		motor->m_min_rpm_hyst_timer = 0.0;
		motor->m_min_rpm_timer = 0.0;
		motor->m_phase_observer_override = false;
	}

	bool started_now = false;
	if (motor->m_min_rpm_hyst_timer >= conf_now->foc_sl_openloop_hyst &&
			motor->m_min_rpm_timer <= 0.0001) {
		motor->m_min_rpm_timer = t_lock + t_ramp + t_const;
		started_now = true;
	}

	if (motor->m_state != MC_STATE_RUNNING) {
		motor->m_min_rpm_timer = 0.0;
	}

	if (motor->m_min_rpm_timer > 0.0) {
		motor->m_phase_now_observer_override += add_min_speed;

		// When the motor gets stuck it tends to be 90 degrees off, so start the open loop
		// sequence by correcting with 60 degrees.
		if (started_now) {
			if (motor->m_motor_state.duty_now > 0.0) {
				motor->m_phase_now_observer_override += M_PI / 3.0;
			} else {
				motor->m_phase_now_observer_override -= M_PI / 3.0;
			}
		}

		utils_norm_angle_rad((float*)&motor->m_phase_now_observer_override);
		motor->m_phase_observer_override = true;
		motor->m_min_rpm_timer -= dt;
		motor->m_min_rpm_hyst_timer = 0.0;

		// Set observer state to help it start tracking when leaving open loop.
		float s, c;
		utils_fast_sincos_better(motor->m_phase_now_observer_override + SIGN(motor->m_motor_state.duty_now) * M_PI / 4.0, &s, &c);
		motor->m_observer_x1_override = c * conf_now->foc_motor_flux_linkage;
		motor->m_observer_x2_override = s * conf_now->foc_motor_flux_linkage;
	} else {
		motor->m_phase_now_observer_override = motor->m_phase_now_observer;
		motor->m_phase_observer_override = false;
	}

	// Samples
	if (motor->m_state == MC_STATE_RUNNING) {
		const volatile float vd_tmp = motor->m_motor_state.vd;
		const volatile float vq_tmp = motor->m_motor_state.vq;
		const volatile float id_tmp = motor->m_motor_state.id;
		const volatile float iq_tmp = motor->m_motor_state.iq;

		motor->m_samples.avg_current_tot += NORM2_f(id_tmp, iq_tmp);
		motor->m_samples.avg_voltage_tot += NORM2_f(vd_tmp, vq_tmp);
		motor->m_samples.sample_num++;
	}

	// Observer gain scaling, based on bus voltage and duty cycle
	float gamma_tmp = utils_map(fabsf(motor->m_motor_state.duty_now),
								0.0, 40.0 / motor->m_motor_state.v_bus,
								0, conf_now->foc_observer_gain);
	if (gamma_tmp < (conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain)) {
		gamma_tmp = conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain;
	}

	// 4.0 scaling is kind of arbitrary, but it should make configs from old VESC Tools more likely to work.
	motor->m_gamma_now = gamma_tmp * 4.0;

	// Run resistance observer
	// See "An adaptive flux observer for the permanent magnet synchronous motor"
	// https://doi.org/10.1002/acs.2587
	{
		float res_est_gain = 0.00002;
		float i_abs_sq = SQ(motor->m_motor_state.i_abs);
		motor->m_r_est = motor->m_r_est_state - 0.5 * res_est_gain * conf_now->foc_motor_l * i_abs_sq;
		float res_dot = -res_est_gain * (motor->m_r_est * i_abs_sq + motor->m_speed_est_fast *
				(motor->m_motor_state.i_beta * motor->m_observer_state.x1 - motor->m_motor_state.i_alpha * motor->m_observer_state.x2) -
				(motor->m_motor_state.i_alpha * motor->m_motor_state.v_alpha + motor->m_motor_state.i_beta * motor->m_motor_state.v_beta));
		motor->m_r_est_state += res_dot * dt;

		utils_truncate_number((float*)&motor->m_r_est_state, conf_now->foc_motor_r * 0.25, conf_now->foc_motor_r * 3.0);
	}
}

//static void terminal_tmp(int argc, const char **argv) {
//	(void)argc;
//	(void)argv;
//
//	int top = 1;
//	if (argc == 2) {
//		float seconds = -1.0;
//		sscanf(argv[1], "%f", &seconds);
//
//		if (seconds > 0.0) {
//			top = seconds * 2;
//		}
//	}
//
//	if (top > 1) {
//		commands_init_plot("Time", "Temperature");
//		commands_plot_add_graph("Temp Measured");
//		commands_plot_add_graph("Temp Estimated");
//		commands_plot_add_graph("lambda_est");
//	}
//
//	for (int i = 0;i < top;i++) {
//		float res_est = m_motor_1.m_r_est;
//		float t_base = m_motor_1.m_conf->foc_temp_comp_base_temp;
//		float res_base = m_motor_1.m_conf->foc_motor_r;
//		float t_est = (res_est / res_base - 1) / 0.00386 + t_base;
//		float t_meas = mc_interface_temp_motor_filtered();
//
//		if (top > 1) {
//			commands_plot_set_graph(0);
//			commands_send_plot_points((float)i / 2.0, t_meas);
//			commands_plot_set_graph(1);
//			commands_send_plot_points((float)i / 2.0, t_est);
//			commands_plot_set_graph(2);
//			commands_send_plot_points((float)i / 2.0, m_motor_1.m_observer_state.lambda_est);
//			commands_printf("Sample %d of %d", i, top);
//		}
//
//		commands_printf("R: %.2f, EST: %.2f",
//				(double)(res_base * 1000.0), (double)(res_est * 1000.0));
//		commands_printf("T: %.2f, T_EST: %.2f\n",
//				(double)t_meas, (double)t_est);
//
//		chThdSleepMilliseconds(500);
//	}
//}

// TODO: This won't work for dual motors
static void input_current_offset_measurement(void) {
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
	static uint16_t delay_current_offset_measurement = 0;

	if (delay_current_offset_measurement < 1000) {
		delay_current_offset_measurement++;
	} else {
		if (delay_current_offset_measurement == 1000) {
			delay_current_offset_measurement++;
			MEASURE_INPUT_CURRENT_OFFSET();
		}
	}
#endif
}

void timer_thread(void * arg){
	(void)arg;

	for(;;) {
		const float dt = 0.001;

		if (timer_thd_stop) {
			timer_thd_stop = false;
			//return;
		}

		timer_update((motor_all_state_t*)&m_motor_1, dt);

		input_current_offset_measurement();

		vTaskDelay(MS_TO_TICKS(1));
	}
}

static void hfi_update(volatile motor_all_state_t *motor, float dt) {
	(void)dt;
	float rpm_abs = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

	if (rpm_abs > motor->m_conf->foc_sl_erpm_hfi) {
		motor->m_hfi.angle = motor->m_phase_now_observer;
		motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
	}

	if (motor->m_hfi.ready) {
		if ((motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 ||
				motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) &&
				motor->m_hfi.est_done_cnt >= motor->m_conf->foc_hfi_start_samples) {
			// Nothing done here, the update is done in the interrupt.
		} else if ((motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 ||
				motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) &&
				motor->m_hfi.est_done_cnt >= motor->m_conf->foc_hfi_start_samples) {
			// Nothing done here, the update is done in the interrupt.

			// Enable to set the observer position to the HFI angle for plotting the error in the position plot RT page in VESC Tool. Just
			// remember that enabling this will make the transition to using the observer bad.
#if 0
			float s, c;
			utils_fast_sincos_better(motor->m_hfi.angle, &s, &c);
			motor->m_observer_x1 = c * motor->m_conf->foc_motor_flux_linkage;
			motor->m_observer_x2 = s * motor->m_conf->foc_motor_flux_linkage;
#endif

			// Enable to plot the sample together with encoder position
#if 0

			commands_plot_set_graph(0);
			commands_send_plot_points(motor->m_hfi_plot_sample, ind_a);
			commands_plot_set_graph(1);
			commands_send_plot_points(motor->m_hfi_plot_sample, RAD2DEG_f(motor->m_phase_now_encoder) / 4e6);
			motor->m_hfi_plot_sample++;
#endif
		} else {
			float real_bin1, imag_bin1, real_bin2, imag_bin2;
			motor->m_hfi.fft_bin1_func((float*)motor->m_hfi.buffer, &real_bin1, &imag_bin1);
			motor->m_hfi.fft_bin2_func((float*)motor->m_hfi.buffer, &real_bin2, &imag_bin2);

			float mag_bin_1 = NORM2_f(imag_bin1, real_bin1);
			float angle_bin_1 = -utils_fast_atan2(imag_bin1, real_bin1);

			angle_bin_1 += M_PI / 1.7; // Why 1.7??
			utils_norm_angle_rad(&angle_bin_1);

			float mag_bin_2 = NORM2_f(imag_bin2, real_bin2);
			float angle_bin_2 = -utils_fast_atan2(imag_bin2, real_bin2) / 2.0;

			// Assuming this thread is much faster than it takes to fill the HFI buffer completely,
			// we should lag 1/2 HFI buffer behind in phase. Compensate for that here.
			float dt_sw;
			if (motor->m_conf->foc_sample_v0_v7) {
				dt_sw = 1.0 / motor->m_conf->foc_f_zv;
			} else {
				dt_sw = 1.0 / (motor->m_conf->foc_f_zv / 2.0);
			}
			angle_bin_2 += motor->m_motor_state.speed_rad_s * ((float)motor->m_hfi.samples / 2.0) * dt_sw;

			if (fabsf(utils_angle_difference_rad(angle_bin_2 + M_PI, motor->m_hfi.angle)) <
					fabsf(utils_angle_difference_rad(angle_bin_2, motor->m_hfi.angle))) {
				angle_bin_2 += M_PI;
			}

			if (motor->m_hfi.est_done_cnt < motor->m_conf->foc_hfi_start_samples) {
				motor->m_hfi.est_done_cnt++;

				if (fabsf(utils_angle_difference_rad(angle_bin_2, angle_bin_1)) > (M_PI / 2.0)) {
					motor->m_hfi.flip_cnt++;
				}
			}

			if (motor->m_hfi.est_done_cnt >= motor->m_conf->foc_hfi_start_samples) {
				if (motor->m_hfi.flip_cnt >= (motor->m_conf->foc_hfi_start_samples / 2)) {
					angle_bin_2 += M_PI;
				}
				motor->m_hfi.flip_cnt = 0;

				if (motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START) {
					float s, c;
					utils_fast_sincos_better(angle_bin_2, &s, &c);
					motor->m_observer_state.x1 = c * motor->m_conf->foc_motor_flux_linkage;
					motor->m_observer_state.x2 = s * motor->m_conf->foc_motor_flux_linkage;
				}
			}

			motor->m_hfi.angle = angle_bin_2;
			utils_norm_angle_rad((float*)&motor->m_hfi.angle);

			// As angle_bin_1 is based on saturation, it is only accurate when the motor current is low. It
			// might be possible to compensate for that, which would allow HFI on non-salient motors.
			//			m_hfi.angle = angle_bin_1;

			if (motor->m_hfi_plot_en == 1) {
				static float hfi_plot_div = 0;
				hfi_plot_div++;

				if (hfi_plot_div >= 8) {
					hfi_plot_div = 0;

					float real_bin0, imag_bin0;
					motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer, &real_bin0, &imag_bin0);

//					commands_plot_set_graph(0);
//					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_hfi.angle);
//
//					commands_plot_set_graph(1);
//					commands_send_plot_points(motor->m_hfi_plot_sample, angle_bin_1);
//
//					commands_plot_set_graph(2);
//					commands_send_plot_points(motor->m_hfi_plot_sample, 2.0 * mag_bin_2 * 1e6);
//
//					commands_plot_set_graph(3);
//					commands_send_plot_points(motor->m_hfi_plot_sample, 2.0 * mag_bin_1 * 1e6);
//
//					commands_plot_set_graph(4);
//					commands_send_plot_points(motor->m_hfi_plot_sample, real_bin0 * 1e6);

//					commands_plot_set_graph(0);
//					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_motor_state.speed_rad_s);
//
//					commands_plot_set_graph(1);
//					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_speed_est_fast);

					motor->m_hfi_plot_sample++;
				}
			} else if (motor->m_hfi_plot_en == 2) {
				static float hfi_plot_div = 0;
				hfi_plot_div++;

				if (hfi_plot_div >= 8) {
					hfi_plot_div = 0;

					if (motor->m_hfi_plot_sample >= motor->m_hfi.samples) {
						motor->m_hfi_plot_sample = 0;
					}

//					commands_plot_set_graph(0);
//					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_hfi.buffer_current[(int)motor->m_hfi_plot_sample]);
//
//					commands_plot_set_graph(1);
//					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_hfi.buffer[(int)motor->m_hfi_plot_sample] * 1e6);

					motor->m_hfi_plot_sample++;
				}
			}
		}
	} else {
		motor->m_hfi.angle = motor->m_phase_now_observer;
		motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
	}
}

void hfi_thread(void * arg){
	(void)arg;

	//uint32_t t_last = timer_time_now();

	for(;;) {
		if (hfi_thd_stop) {
			hfi_thd_stop = false;
			//return;
		}

		//float dt = timer_seconds_elapsed_since(t_last);
		float dt = 0.0005;
		//t_last = timer_time_now();

		hfi_update(&m_motor_1, dt);

		vTaskDelay(1);
	}
}

void pid_thread(void * arg){
	(void)arg;

	//uint32_t last_time = timer_time_now();

	for(;;) {
		if (pid_thd_stop) {
			pid_thd_stop = false;
			//return;
		}
		uint32_t ticks=1;
		switch (m_motor_1.m_conf->sp_pid_loop_rate) {
		case PID_RATE_25_HZ: ticks = MS_TO_TICKS(2000/25); break;
		case PID_RATE_50_HZ: ticks = MS_TO_TICKS(2000/50); break;
		case PID_RATE_100_HZ: ticks = MS_TO_TICKS(2000/100); break;
		case PID_RATE_250_HZ: ticks = MS_TO_TICKS(2000/250);; break;
		case PID_RATE_500_HZ: ticks = MS_TO_TICKS(2000/500); break;
		case PID_RATE_1000_HZ: ticks = MS_TO_TICKS(2000/1000); break;
		case PID_RATE_2500_HZ: ticks = 1; break;
		case PID_RATE_5000_HZ: ticks = 1; break;
		case PID_RATE_10000_HZ: ticks = 1; break;
		}
		vTaskDelay(ticks);

		//float dt = timer_seconds_elapsed_since(last_time);
		//last_time = timer_time_now();
		float dt = ((float)ticks/2.0/1000.0);

		//foc_run_pid_control_pos(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_1);
		foc_run_pid_control_speed(dt, (motor_all_state_t*)&m_motor_1);

	}
}

/**
 * Run the current control loop.
 *
 * @param state_m
 * The motor state.
 *
 * Parameters that shall be set before calling this function:
 * id_target
 * iq_target
 * max_duty
 * phase
 * i_alpha
 * i_beta
 * v_bus
 * speed_rad_s
 *
 * Parameters that will be updated in this function:
 * i_bus
 * i_abs
 * i_abs_filter
 * v_alpha
 * v_beta
 * mod_d
 * mod_q
 * id
 * iq
 * id_filter
 * iq_filter
 * vd
 * vq
 * vd_int
 * vq_int
 * svm_sector
 *
 * @param dt
 * The time step in seconds.
 */
static void control_current(motor_all_state_t *motor, float dt) {
	volatile motor_state_t *state_m = &motor->m_motor_state;
	volatile mc_configuration *conf_now = motor->m_conf;

	float s = state_m->phase_sin;
	float c = state_m->phase_cos;

	float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

	bool do_hfi = (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5 ||
			(conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START &&
					motor->m_control_mode != CONTROL_MODE_CURRENT_BRAKE &&
					fabsf(state_m->iq_target) > conf_now->cc_min_current)) &&
							!motor->m_phase_override &&
							abs_rpm < (conf_now->foc_sl_erpm_hfi * (motor->m_cc_was_hfi ? 1.8 : 1.5));

	bool hfi_est_done = motor->m_hfi.est_done_cnt >= conf_now->foc_hfi_start_samples;

	// Only allow Q axis current after the HFI ambiguity is resolved. This causes
	// a short delay when starting.
	if (do_hfi && !hfi_est_done) {
		state_m->iq_target = 0;
	} else if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START) {
		do_hfi = false;
	}

	motor->m_cc_was_hfi = do_hfi;

	float max_duty = fabsf(state_m->max_duty);
	utils_truncate_number(&max_duty, 0.0, conf_now->l_max_duty);

    // Park transform: transforms the currents from stator to the rotor reference frame
	state_m->id = c * state_m->i_alpha + s * state_m->i_beta;
	state_m->iq = c * state_m->i_beta  - s * state_m->i_alpha;

	// Low passed currents are used for less time critical parts, not for the feedback
	UTILS_LP_FAST(state_m->id_filter, state_m->id, conf_now->foc_current_filter_const);
	UTILS_LP_FAST(state_m->iq_filter, state_m->iq, conf_now->foc_current_filter_const);

	float d_gain_scale = 1.0;
	if (conf_now->foc_d_gain_scale_start < 0.99) {
		float max_mod_norm = fabsf(state_m->duty_now / max_duty);
		if (max_duty < 0.01) {
			max_mod_norm = 1.0;
		}
		if (max_mod_norm > conf_now->foc_d_gain_scale_start) {
			d_gain_scale = utils_map(max_mod_norm, conf_now->foc_d_gain_scale_start, 1.0,
					1.0, conf_now->foc_d_gain_scale_max_mod);
			if (d_gain_scale < conf_now->foc_d_gain_scale_max_mod) {
				d_gain_scale = conf_now->foc_d_gain_scale_max_mod;
			}
		}
	}

	float Ierr_d = state_m->id_target - state_m->id;
	float Ierr_q = state_m->iq_target - state_m->iq;

	float ki = conf_now->foc_current_ki;
	if (conf_now->foc_temp_comp) {
		ki = motor->m_current_ki_temp_comp;
	}

	state_m->vd_int += Ierr_d * (ki * d_gain_scale * dt);
	state_m->vq_int += Ierr_q * (ki * dt);

	state_m->vd = state_m->vd_int + Ierr_d * conf_now->foc_current_kp * d_gain_scale; //Feedback (PI controller). No D action needed because the plant is a first order system (tf = 1/(Ls+R))
	state_m->vq = state_m->vq_int + Ierr_q * conf_now->foc_current_kp;

	// Decoupling. Using feedforward this compensates for the fact that the equations of a PMSM
	// are not really decoupled (the d axis current has impact on q axis voltage and visa-versa):
    //      Resistance  Inductance   Cross terms   Back-EMF   (see www.mathworks.com/help/physmod/sps/ref/pmsm.html)
    // vd = Rs*id   +   Ld*did/dt −  ωe*iq*Lq
    // vq = Rs*iq   +   Lq*diq/dt +  ωe*id*Ld     + ωe*ψm
	float dec_vd = 0.0;
	float dec_vq = 0.0;
	float dec_bemf = 0.0;

	if (motor->m_control_mode < CONTROL_MODE_HANDBRAKE && conf_now->foc_cc_decoupling != FOC_CC_DECOUPLING_DISABLED) {
		switch (conf_now->foc_cc_decoupling) {
		case FOC_CC_DECOUPLING_CROSS:
			dec_vd = state_m->iq_filter * motor->m_speed_est_fast * motor->p_lq; // m_speed_est_fast is ωe in [rad/s]
			dec_vq = state_m->id_filter * motor->m_speed_est_fast * motor->p_ld;
			break;

		case FOC_CC_DECOUPLING_BEMF:
			dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
			break;

		case FOC_CC_DECOUPLING_CROSS_BEMF:
			dec_vd = state_m->iq_filter * motor->m_speed_est_fast * motor->p_lq;
			dec_vq = state_m->id_filter * motor->m_speed_est_fast * motor->p_ld;
			dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
			break;

		default:
			break;
		}
	}

	state_m->vd -= dec_vd; //Negative sign as in the PMSM equations
	state_m->vq += dec_vq + dec_bemf;

	// Calculate the max length of the voltage space vector without overmodulation.
	// Is simply 1/sqrt(3) * v_bus. See https://microchipdeveloper.com/mct5001:start. Adds margin with max_duty.
	float max_v_mag = ONE_BY_SQRT3 * max_duty * state_m->v_bus;

	// Saturation and anti-windup. Notice that the d-axis has priority as it controls field
	// weakening and the efficiency.
	float vd_presat = state_m->vd;
	utils_truncate_number_abs((float*)&state_m->vd, max_v_mag);
	state_m->vd_int += (state_m->vd - vd_presat);

	float max_vq = sqrtf(SQ(max_v_mag) - SQ(state_m->vd));
	float vq_presat = state_m->vq;
	utils_truncate_number_abs((float*)&state_m->vq, max_vq);
	state_m->vq_int += (state_m->vq - vq_presat);

	utils_saturate_vector_2d((float*)&state_m->vd, (float*)&state_m->vq, max_v_mag);

	// mod_d and mod_q are normalized such that 1 corresponds to the max possible voltage:
	//    voltage_normalize = 1/(2/3*V_bus)
	// This includes overmodulation and therefore cannot be made in any direction.
	// Note that this scaling is different from max_v_mag, which is without over modulation.
	const float voltage_normalize = 1.5 / state_m->v_bus;
	state_m->mod_d = state_m->vd * voltage_normalize;
	state_m->mod_q = state_m->vq * voltage_normalize;
	UTILS_NAN_ZERO(state_m->mod_q_filter);
	UTILS_LP_FAST(state_m->mod_q_filter, state_m->mod_q, 0.2);

	// TODO: Have a look at this?
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
	state_m->i_bus = GET_INPUT_CURRENT();
#else
	state_m->i_bus = state_m->mod_alpha_measured * state_m->i_alpha + state_m->mod_beta_measured * state_m->i_beta;
	// TODO: Also calculate motor power based on v_alpha, v_beta, i_alpha and i_beta. This is much more accurate
	// with phase filters than using the modulation and bus current.
#endif
	state_m->i_abs = NORM2_f(state_m->id, state_m->iq);
	state_m->i_abs_filter = NORM2_f(state_m->id_filter, state_m->iq_filter);

    // Inverse Park transform: transforms the (normalized) voltages from the rotor reference frame to the stator frame
	state_m->mod_alpha_raw = c * state_m->mod_d - s * state_m->mod_q;
	state_m->mod_beta_raw  = c * state_m->mod_q + s * state_m->mod_d;

	update_valpha_vbeta(motor, state_m->mod_alpha_raw, state_m->mod_beta_raw);

    // Dead time compensated values for vd and vq. Note that these are not used to control the switching times.
	state_m->vd = c * motor->m_motor_state.v_alpha + s * motor->m_motor_state.v_beta;
	state_m->vq = c * motor->m_motor_state.v_beta  - s * motor->m_motor_state.v_alpha;

	// HFI
	if (do_hfi) {
		CURRENT_FILTER_OFF();

		float mod_alpha_v7 = state_m->mod_alpha_raw;
		float mod_beta_v7 = state_m->mod_beta_raw;

#ifdef HW_HAS_PHASE_SHUNTS
		float mod_alpha_v0 = state_m->mod_alpha_raw;
		float mod_beta_v0 = state_m->mod_beta_raw;
#endif

		float hfi_voltage;
		if (motor->m_hfi.est_done_cnt < conf_now->foc_hfi_start_samples) {
			hfi_voltage = conf_now->foc_hfi_voltage_start;
		} else {
			hfi_voltage = utils_map(fabsf(state_m->iq), -0.01, conf_now->l_current_max,
					conf_now->foc_hfi_voltage_run, conf_now->foc_hfi_voltage_max);
		}

		utils_truncate_number_abs(&hfi_voltage, state_m->v_bus * (1.0 - fabsf(state_m->duty_now)) * SQRT3_BY_2 * (2.0 / 3.0) * 0.95);

		if ((conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) && hfi_est_done) {
			if (motor->m_hfi.is_samp_n) {
				float sample_now = c * motor->m_i_beta_sample_with_offset - s * motor->m_i_alpha_sample_with_offset;
				float di = (motor->m_hfi.prev_sample - sample_now);

				if (!motor->m_using_encoder) {
					motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
					motor->m_hfi.angle = motor->m_phase_now_observer;
				} else {
					float hfi_dt = dt * 2.0;
#ifdef HW_HAS_PHASE_SHUNTS
					if (!conf_now->foc_sample_v0_v7 && conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4) {
						hfi_dt = dt;
					}
#endif
					foc_hfi_adjust_angle(
							(di * conf_now->foc_f_zv) / (hfi_voltage * motor->p_inv_ld_lq),
							motor, hfi_dt
					);
				}

#ifdef HW_HAS_PHASE_SHUNTS
				if (conf_now->foc_sample_v0_v7 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) {
					mod_alpha_v7 -= hfi_voltage * c * voltage_normalize;
					mod_beta_v7 -= hfi_voltage * s * voltage_normalize;
				} else {
					motor->m_hfi.prev_sample = c * motor->m_i_beta_sample_next - s * motor->m_i_alpha_sample_next;

					mod_alpha_v0 -= hfi_voltage * c * voltage_normalize;
					mod_beta_v0 -= hfi_voltage * s * voltage_normalize;

					mod_alpha_v7 += hfi_voltage * c * voltage_normalize;
					mod_beta_v7 += hfi_voltage * s * voltage_normalize;

					motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;
					motor->m_i_alpha_beta_has_offset = true;
				}
#else
				mod_alpha_v7 -= hfi_voltage * c * voltage_normalize;
				mod_beta_v7 -= hfi_voltage * s * voltage_normalize;
#endif
			} else {
				motor->m_hfi.prev_sample = c * state_m->i_beta - s * state_m->i_alpha;
				mod_alpha_v7 += hfi_voltage * c * voltage_normalize;
				mod_beta_v7  += hfi_voltage * s * voltage_normalize;
			}
		} else if ((conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) && hfi_est_done) {
			if (motor->m_hfi.is_samp_n) {
				if (fabsf(state_m->iq_target) > conf_now->foc_hfi_hyst) {
					motor->m_hfi.sign_last_sample = SIGN(state_m->iq_target);
				}

				float sample_now = motor->m_hfi.cos_last * motor->m_i_alpha_sample_with_offset +
						motor->m_hfi.sin_last * motor->m_i_beta_sample_with_offset;
				float di = (sample_now - motor->m_hfi.prev_sample);

				if (!motor->m_using_encoder) {
					motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
					motor->m_hfi.angle = motor->m_phase_now_observer;
				} else {
					float hfi_dt = dt * 2.0;
#ifdef HW_HAS_PHASE_SHUNTS
					if (!conf_now->foc_sample_v0_v7 && conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2) {
						hfi_dt = dt;
					}
#endif
					foc_hfi_adjust_angle(
							motor->m_hfi.sign_last_sample * ((conf_now->foc_f_zv * di) /
									hfi_voltage - motor->p_v2_v3_inv_avg_half) / motor->p_inv_ld_lq,
							motor, hfi_dt
					);
				}

				// Use precomputed rotation matrix
				if (motor->m_hfi.sign_last_sample > 0) {
					// +45 Degrees
					motor->m_hfi.sin_last = ONE_BY_SQRT2 * (c + s);
					motor->m_hfi.cos_last = ONE_BY_SQRT2 * (c - s);
				} else {
					// -45 Degrees
					motor->m_hfi.sin_last = ONE_BY_SQRT2 * (-c + s);
					motor->m_hfi.cos_last = ONE_BY_SQRT2 * (c + s);
				}

#ifdef HW_HAS_PHASE_SHUNTS
				if (conf_now->foc_sample_v0_v7 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) {
					mod_alpha_v7 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v7 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
				} else {
					motor->m_hfi.prev_sample = motor->m_hfi.cos_last * motor->m_i_alpha_sample_next +
							motor->m_hfi.sin_last * motor->m_i_beta_sample_next;

					mod_alpha_v0 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v0 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;

					mod_alpha_v7 -= hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v7 -= hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;

					motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;
					motor->m_i_alpha_beta_has_offset = true;
				}
#else
				mod_alpha_v7 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
				mod_beta_v7 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
#endif
			} else {
				motor->m_hfi.prev_sample = motor->m_hfi.cos_last * state_m->i_alpha + motor->m_hfi.sin_last * state_m->i_beta;
				mod_alpha_v7 -= hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
				mod_beta_v7  -= hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
			}
		} else {
			if (motor->m_hfi.is_samp_n) {
				float sample_now = (utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
						utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta);
				float di = (sample_now - motor->m_hfi.prev_sample);

				motor->m_hfi.buffer_current[motor->m_hfi.ind] = di;

				if (di > 0.01) {
					motor->m_hfi.buffer[motor->m_hfi.ind] = hfi_voltage / (conf_now->foc_f_zv * di);
				}

				motor->m_hfi.ind++;
				if (motor->m_hfi.ind == motor->m_hfi.samples) {
					motor->m_hfi.ind = 0;
					motor->m_hfi.ready = true;
				}

				mod_alpha_v7 += hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
				mod_beta_v7  -= hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
			} else {
				motor->m_hfi.prev_sample = utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
						utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta;

				mod_alpha_v7 -= hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
				mod_beta_v7  += hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
			}
		}

		utils_saturate_vector_2d(&mod_alpha_v7, &mod_beta_v7, SQRT3_BY_2 * 0.95);
		motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;

		if (conf_now->foc_sample_v0_v7) {
			state_m->mod_alpha_raw = mod_alpha_v7;
			state_m->mod_beta_raw = mod_beta_v7;
		} else {
#ifdef HW_HAS_PHASE_SHUNTS
			if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4) {
				utils_saturate_vector_2d(&mod_alpha_v0, &mod_beta_v0, SQRT3_BY_2 * 0.95);
				state_m->mod_alpha_raw = mod_alpha_v0;
				state_m->mod_beta_raw = mod_beta_v0;
			}
#endif

			// Delay adding the HFI voltage when not sampling in both 0 vectors, as it will cancel
			// itself with the opposite pulse from the previous HFI sample. This makes more sense
			// when drawing the SVM waveform.
			foc_svm(mod_alpha_v7, mod_beta_v7, TIM1->ARR,
				(uint32_t*)&motor->m_duty1_next,
				(uint32_t*)&motor->m_duty2_next,
				(uint32_t*)&motor->m_duty3_next,
				(uint32_t*)&state_m->svm_sector);
			motor->m_duty_next_set = true;
		}
	} else {
		CURRENT_FILTER_ON();
		motor->m_hfi.ind = 0;
		motor->m_hfi.ready = false;
		motor->m_hfi.is_samp_n = false;
		motor->m_hfi.prev_sample = 0.0;
		motor->m_hfi.double_integrator = 0.0;
	}

	// Set output (HW Dependent)
	uint32_t duty1, duty2, duty3, top;
	top = TIM1->ARR;

	// Calculate the duty cycles for all the phases. This also injects a zero modulation signal to
	// be able to fully utilize the bus voltage. See https://microchipdeveloper.com/mct5001:start
	foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw, top, &duty1, &duty2, &duty3, (uint32_t*)&state_m->svm_sector);

	if (motor == &m_motor_1) {
		TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);

#ifdef HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
	} else {
#ifndef HW_HAS_DUAL_PARALLEL
		//TODO TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
	}

	if (!motor->m_output_on) {
		start_pwm_hw(motor);
	}


}

static void update_valpha_vbeta(motor_all_state_t *motor, float mod_alpha, float mod_beta) {
	motor_state_t *state_m = &motor->m_motor_state;
	mc_configuration *conf_now = motor->m_conf;
	float Va, Vb, Vc;

	volatile float *ofs_volt = conf_now->foc_offsets_voltage_undriven;
	if (motor->m_state == MC_STATE_RUNNING) {
		ofs_volt = conf_now->foc_offsets_voltage;
	}

	Va = (GET_VOLT1() - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	Vb = (GET_VOLT2() - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	Vc = (GET_VOLT3() - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;


	// Deadtime compensation
	float s = state_m->phase_sin;
	float c = state_m->phase_cos;
	const float i_alpha_filter = c * state_m->id_filter - s * state_m->iq_filter;
	const float i_beta_filter = c * state_m->iq_filter + s * state_m->id_filter;
	const float ia_filter = i_alpha_filter;
	const float ib_filter = -0.5 * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
	const float ic_filter = -0.5 * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;

	// mod_alpha_sign = 2/3*sign(ia) - 1/3*sign(ib) - 1/3*sign(ic)
	// mod_beta_sign  = 1/sqrt(3)*sign(ib) - 1/sqrt(3)*sign(ic)
	const float mod_alpha_filter_sgn = (1.0 / 3.0) * (2.0 * SIGN(ia_filter) - SIGN(ib_filter) - SIGN(ic_filter));
	const float mod_beta_filter_sgn = ONE_BY_SQRT3 * (SIGN(ib_filter) - SIGN(ic_filter));

	const float mod_comp_fact = conf_now->foc_dt_us * 1e-6 * conf_now->foc_f_zv;
	const float mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
	const float mod_beta_comp = mod_beta_filter_sgn * mod_comp_fact;

	mod_alpha -= mod_alpha_comp;
	mod_beta -= mod_beta_comp;

	state_m->va = Va;
	state_m->vb = Vb;
	state_m->vc = Vc;
	state_m->mod_alpha_measured = mod_alpha;
	state_m->mod_beta_measured = mod_beta;

	// v_alpha = 2/3*Va - 1/3*Vb - 1/3*Vc
	// v_beta  = 1/sqrt(3)*Vb - 1/sqrt(3)*Vc
	float v_alpha = (1.0 / 3.0) * (2.0 * Va - Vb - Vc);
	float v_beta = ONE_BY_SQRT3 * (Vb - Vc);

	// Keep the modulation updated so that the filter stays updated
	// even when the motor is undriven.
	if (motor->m_state != MC_STATE_RUNNING) {
		/* voltage_normalize = 1/(2/3*V_bus) */
		const float voltage_normalize = 1.5 / state_m->v_bus;

		mod_alpha = v_alpha * voltage_normalize;
		mod_beta = v_beta * voltage_normalize;
	}

	float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

	float filter_const = 1.0;
	if (abs_rpm < 10000.0) {
		filter_const = utils_map(abs_rpm, 0.0, 10000.0, 0.01, 1.0);
	}

	float v_mag = NORM2_f(v_alpha, v_beta);
	// The 0.1 * v_mag term below compensates for the filter attenuation as the speed increases.
	// It is chosen by trial and error, so this can be improved.
	UTILS_LP_FAST(state_m->v_mag_filter, v_mag + 0.1 * v_mag * filter_const, filter_const);
	UTILS_LP_FAST(state_m->mod_alpha_filter, mod_alpha, filter_const);
	UTILS_LP_FAST(state_m->mod_beta_filter, mod_beta, filter_const);
	UTILS_NAN_ZERO(state_m->v_mag_filter);
	UTILS_NAN_ZERO(state_m->mod_alpha_filter);
	UTILS_NAN_ZERO(state_m->mod_beta_filter);

	mod_alpha = state_m->mod_alpha_filter;
	mod_beta = state_m->mod_beta_filter;

	if (motor->m_state == MC_STATE_RUNNING) {
#ifdef HW_HAS_PHASE_FILTERS
		if (conf_now->foc_phase_filter_enable && abs_rpm < conf_now->foc_phase_filter_max_erpm) {
			float mod_mag = NORM2_f(mod_alpha, mod_beta);
			float v_mag_mod = mod_mag * (2.0 / 3.0) * state_m->v_bus;

			if (fabsf(v_mag_mod - state_m->v_mag_filter) > (conf_now->l_max_vin * 0.05)) {
				mc_interface_set_fault_info("v_mag_mod: %.2f, v_mag_filter: %.2f", 2, v_mag_mod, state_m->v_mag_filter);
				mc_interface_fault_stop(FAULT_CODE_PHASE_FILTER, &m_motor_1 != motor, true);
			}

			// Compensate for the phase delay by using the direction of the modulation
			// together with the magnitude from the phase filters
			if (mod_mag > 0.04) {
				state_m->v_alpha = mod_alpha / mod_mag * state_m->v_mag_filter;
				state_m->v_beta = mod_beta / mod_mag * state_m->v_mag_filter;
			} else {
				state_m->v_alpha = v_alpha;
				state_m->v_beta = v_beta;
			}

			state_m->is_using_phase_filters = true;
		} else {
#endif
			state_m->v_alpha = mod_alpha * (2.0 / 3.0) * state_m->v_bus;
			state_m->v_beta = mod_beta * (2.0 / 3.0) * state_m->v_bus;
			state_m->is_using_phase_filters = false;
#ifdef HW_HAS_PHASE_FILTERS
		}
#endif
	} else {
		state_m->v_alpha = v_alpha;
		state_m->v_beta = v_beta;
		state_m->is_using_phase_filters = false;

#ifdef HW_USE_LINE_TO_LINE
		// rotate alpha-beta 30 degrees to compensate for line-to-line phase voltage sensing
		float x_tmp = state_m->v_alpha;
		float y_tmp = state_m->v_beta;

		state_m->v_alpha = x_tmp * COS_MINUS_30_DEG - y_tmp * SIN_MINUS_30_DEG;
		state_m->v_beta = x_tmp * SIN_MINUS_30_DEG + y_tmp * COS_MINUS_30_DEG;

		// compensate voltage amplitude
		state_m->v_alpha *= ONE_BY_SQRT3;
		state_m->v_beta *= ONE_BY_SQRT3;
#endif
	}
}

static void stop_pwm_hw(motor_all_state_t *motor) {
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;

	TIM1->CCER &= ~TIMxCCER_MASK_CH123;


	motor->m_output_on = false;
}

static void start_pwm_hw(motor_all_state_t *motor) {

	TIM1->CCER |= TIMxCCER_MASK_CH123;

	motor->m_output_on = true;

}

//static void terminal_plot_hfi(int argc, const char **argv) {
//	if (argc == 2) {
//		int d = -1;
//		sscanf(argv[1], "%d", &d);
//
//		if (d == 0 || d == 1 || d == 2 || d == 3) {
//			get_motor_now()->m_hfi_plot_en = d;
//			if (get_motor_now()->m_hfi_plot_en == 1) {
//				get_motor_now()->m_hfi_plot_sample = 0.0;
//				commands_init_plot("Sample", "Value");
//				commands_plot_add_graph("Phase");
//				commands_plot_add_graph("Phase bin2");
//				commands_plot_add_graph("Ld - Lq (uH");
//				commands_plot_add_graph("L Diff Sat (uH)");
//				commands_plot_add_graph("L Avg (uH)");
//			} else if (get_motor_now()->m_hfi_plot_en == 2) {
//				get_motor_now()->m_hfi_plot_sample = 0.0;
//				commands_init_plot("Sample Index", "Value");
//				commands_plot_add_graph("Current (A)");
//				commands_plot_add_graph("Inductance (uH)");
//			} else if (get_motor_now()->m_hfi_plot_en == 3) {
//				get_motor_now()->m_hfi_plot_sample = 0.0;
//				commands_init_plot("Sample Index", "Value");
//				commands_plot_add_graph("Inductance");;
//			}
//
//			commands_printf(main_uart.phandle, get_motor_now()->m_hfi_plot_en ?
//					"HFI plot enabled" :
//					"HFI plot disabled");
//		} else {
//			commands_printf(main_uart.phandle, "Invalid Argument. en has to be 0, 1, 2 or 3.\n");
//		}
//	} else {
//		commands_printf(main_uart.phandle, "This command requires one argument.\n");
//	}
//}
