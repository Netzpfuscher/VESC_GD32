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

//#include "ch.h"
//#include "hal.h"
#include <commands.h>
#include "terminal.h"
//#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "timeout.h"
//#include "encoder/encoder.h"
#include "app.h"
//#include "comm_usb.h"
//#include "comm_usb_serial.h"
#include "mempools.h"
#include "crc.h"
#include "firmware_metadata.h"

#include "FreeRTOS.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

// Settings
#define FAULT_VEC_LEN						25
#define CALLBACK_LEN						40

// Private types
typedef struct _terminal_callback_struct {
	const char *command;
	const char *help;
	const char *arg_names;
	void(*cbf)(int argc, const char **argv);
} terminal_callback_struct;

// Private variables
static volatile fault_data fault_vec[FAULT_VEC_LEN];
static volatile int fault_vec_write = 0;
static terminal_callback_struct callbacks[CALLBACK_LEN];
static int callback_write = 0;


void terminal_top(PACKET_STATE_t * phandle){
    TaskStatus_t * taskStats;
    uint32_t taskCount = uxTaskGetNumberOfTasks();
    uint32_t sysTime;

    taskStats = pvPortMalloc( taskCount * sizeof( TaskStatus_t ) );
    if(taskStats){
        taskCount = uxTaskGetSystemState(taskStats, taskCount, &sysTime);

        commands_printf(phandle, "Task info:");


        commands_printf(phandle, "FOC ms: %f ", mcpwm_foc_get_last_adc_isr_duration());

        commands_printf(phandle, "Tasks: %d",  taskCount);

        uint32_t heapRemaining = xPortGetFreeHeapSize();
        commands_printf(phandle, "Mem: %db Free: %db Used: %db (%d%%)", configTOTAL_HEAP_SIZE, heapRemaining, configTOTAL_HEAP_SIZE - heapRemaining, ((configTOTAL_HEAP_SIZE - heapRemaining) * 100) / configTOTAL_HEAP_SIZE);

        uint32_t currTask = 0;
        for(;currTask < taskCount; currTask++){
			char name[configMAX_TASK_NAME_LEN+1];
			strncpy(name, taskStats[currTask].pcTaskName, configMAX_TASK_NAME_LEN);
			commands_printf(phandle, "%d Name: %s State: %s Runtime: %d Stack free: %d", taskStats[currTask].xTaskNumber, name, SYS_getTaskStateString(taskStats[currTask].eCurrentState), taskStats[currTask].ulRunTimeCounter, taskStats[currTask].usStackHighWaterMark);
        }
        commands_printf(phandle, "EOL");
        vPortFree(taskStats);
    }
}

void terminal_process_string(char *str, PACKET_STATE_t * phandle) {
	enum { kMaxArgs = 64 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		commands_printf(phandle, "No command received\n");
		return;
	}

	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf != 0 && strcmp(argv[0], callbacks[i].command) == 0) {
			callbacks[i].cbf(argc, (const char**)argv);
			return;
		}
	}

	if (strcmp(argv[0], "ping") == 0) {
		commands_printf(phandle, "pong\n");
	} else if (strcmp(argv[0], "mem") == 0) {
		commands_printf(phandle, "minimum heap free  : %u bytes\n", xPortGetMinimumEverFreeHeapSize());
		commands_printf(phandle, "heap free total  : %u bytes\n", xPortGetFreeHeapSize());
	} else if (strcmp(argv[0], "threads") == 0) {
		terminal_top(phandle);
	} else if (strcmp(argv[0], "fault") == 0) {
		commands_printf(phandle, "%s\n", mc_interface_fault_to_string(mc_interface_get_fault()));
	} else if (strcmp(argv[0], "faults") == 0) {
		if (fault_vec_write == 0) {
			commands_printf(phandle, "No faults registered since startup\n");
		} else {
			commands_printf(phandle, "The following faults were registered since start:\n");
			for (int i = 0;i < fault_vec_write;i++) {
				commands_printf(phandle, "Fault            : %s", mc_interface_fault_to_string(fault_vec[i].fault));
				commands_printf(phandle, "Motor            : %d", fault_vec[i].motor);
				commands_printf(phandle, "Current          : %.1f", (double)fault_vec[i].current);
				commands_printf(phandle, "Current filtered : %.1f", (double)fault_vec[i].current_filtered);
				commands_printf(phandle, "Voltage          : %.2f", (double)fault_vec[i].voltage);
#ifdef HW_HAS_GATE_DRIVER_SUPPLY_MONITOR
				commands_printf(phandle, "Gate drv voltage : %.2f", (double)fault_vec[i].gate_driver_voltage);
#endif
				commands_printf(phandle, "Duty             : %.3f", (double)fault_vec[i].duty);
				commands_printf(phandle, "RPM              : %.1f", (double)fault_vec[i].rpm);
				commands_printf(phandle, "Tacho            : %d", fault_vec[i].tacho);
				commands_printf(phandle, "Cycles running   : %d", fault_vec[i].cycles_running);
				commands_printf(phandle, "TIM duty         : %d", (int)((float)fault_vec[i].tim_top * fault_vec[i].duty));
				commands_printf(phandle, "TIM val samp     : %d", fault_vec[i].tim_val_samp);
				commands_printf(phandle, "TIM current samp : %d", fault_vec[i].tim_current_samp);
				commands_printf(phandle, "TIM top          : %d", fault_vec[i].tim_top);
				commands_printf(phandle, "Comm step        : %d", fault_vec[i].comm_step);
				commands_printf(phandle, "Temperature      : %.2f", (double)fault_vec[i].temperature);
#ifdef HW_HAS_DRV8301
				if (fault_vec[i].fault == FAULT_CODE_DRV) {
					commands_printf(phandle, "DRV8301_FAULTS   : %s", drv8301_faults_to_string(fault_vec[i].drv8301_faults));
				}
#elif defined(HW_HAS_DRV8320S)
				if (fault_vec[i].fault == FAULT_CODE_DRV) {
					commands_printf(phandle, "DRV8320S_FAULTS  : %s", drv8320s_faults_to_string(fault_vec[i].drv8301_faults));
				}
#elif defined(HW_HAS_DRV8323S)
				if (fault_vec[i].fault == FAULT_CODE_DRV) {
					commands_printf(phandle, "DRV8323S_FAULTS  : %s", drv8323s_faults_to_string(fault_vec[i].drv8301_faults));
				}
#endif
				if (fault_vec[i].info_str != 0) {
					char f_str[100];
					strcpy(f_str, "Info             : ");
					strcpy(f_str + 19, fault_vec[i].info_str);
					if (fault_vec[i].info_argn == 0) {
						commands_printf(phandle, f_str);
					} else if (fault_vec[i].info_argn == 1) {
						commands_printf(phandle, f_str, (double)fault_vec[i].info_args[0]);
					} else if (fault_vec[i].info_argn == 2) {
						commands_printf(phandle, f_str, (double)fault_vec[i].info_args[0], (double)fault_vec[i].info_args[1]);
					}
				}
				commands_printf(phandle, " ");
			}
		}
	} else if (strcmp(argv[0], "tim") == 0) {
		//chSysLock();
		volatile int t1_cnt = TIM1->CNT;
		//volatile int t8_cnt = TIM8->CNT;
		volatile int t1_cnt2 = TIM1->CNT;
		volatile int t2_cnt = TIM2->CNT;
		volatile int dir1 = !!(TIM1->CR1 & (1 << 4));
		//volatile int dir8 = !!(TIM8->CR1 & (1 << 4));
		//chSysUnlock();

		int duty1 = TIM1->CCR1;
		int duty2 = TIM1->CCR2;
		int duty3 = TIM1->CCR3;
		int top = TIM1->ARR;
		//int voltage_samp = TIM8->CCR1;
		//int current1_samp = TIM1->CCR4;
		//int current2_samp = TIM8->CCR2;

		commands_printf(phandle, "Tim1 CNT: %i", t1_cnt);
		//commands_printf(phandle, "Tim8 CNT: %i", t8_cnt);
		commands_printf(phandle, "Tim2 CNT: %i", t2_cnt);
		//commands_printf(phandle, "Amount off CNT: %i",top - (2*t8_cnt + t1_cnt + t1_cnt2)/2);
		commands_printf(phandle, "Duty cycle1: %u", duty1);
		commands_printf(phandle, "Duty cycle2: %u", duty2);
		commands_printf(phandle, "Duty cycle3: %u", duty3);
		commands_printf(phandle, "Top: %u", top);
		commands_printf(phandle, "Dir1: %u", dir1);
		//commands_printf(phandle, "Dir8: %u", dir8);
		//commands_printf(phandle, "Voltage sample: %u", voltage_samp);
		//commands_printf(phandle, "Current 1 sample: %u", current1_samp);
		//commands_printf(phandle, "Current 2 sample: %u\n", current2_samp);
	} else if (strcmp(argv[0], "volt") == 0) {
		commands_printf(phandle, "Input voltage: %.2f\n", (double)mc_interface_get_input_voltage_filtered());
#ifdef HW_HAS_GATE_DRIVER_SUPPLY_MONITOR
		commands_printf(phandle, "Gate driver power supply output voltage: %.2f\n", (double)GET_GATE_DRIVER_SUPPLY_VOLTAGE());
#endif
	} else if (strcmp(argv[0], "measure_res") == 0) {
		if (argc == 2) {
			float current = -1.0;
			sscanf(argv[1], "%f", &current);

			mc_configuration *mcconf = mempools_alloc_mcconf();
			*mcconf = *mc_interface_get_configuration();
			mc_configuration *mcconf_old = mempools_alloc_mcconf();
			*mcconf_old = *mc_interface_get_configuration();

			if (current > 0.0 && current <= mcconf->l_current_max) {
				mcconf->motor_type = MOTOR_TYPE_FOC;
				mc_interface_set_configuration(mcconf);

				commands_printf(phandle, "Resistance: %.6f ohm\n", (double)mcpwm_foc_measure_resistance(current, 2000, true));

				mc_interface_set_configuration(mcconf_old);
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}

			mempools_free_mcconf(mcconf);
			mempools_free_mcconf(mcconf_old);
		} else {
			commands_printf(phandle, "This command requires one argument.\n");
		}
	} else if (strcmp(argv[0], "measure_ind") == 0) {
		if (argc == 2) {
			float duty = -1.0;
			sscanf(argv[1], "%f", &duty);

			if (duty > 0.0 && duty < 0.9) {
				mc_configuration *mcconf = mempools_alloc_mcconf();
				*mcconf = *mc_interface_get_configuration();
				mc_configuration *mcconf_old = mempools_alloc_mcconf();
				*mcconf_old = *mc_interface_get_configuration();

				mcconf->motor_type = MOTOR_TYPE_FOC;
				mc_interface_set_configuration(mcconf);

				float curr, ld_lq_diff;
				float ind = mcpwm_foc_measure_inductance(duty, 400, &curr, &ld_lq_diff);
				commands_printf(phandle, "Inductance: %.2f uH, ld_lq_diff: %.2f uH (%.2f A)\n",
						(double)ind, (double)ld_lq_diff, (double)curr);

				mc_interface_set_configuration(mcconf_old);

				mempools_free_mcconf(mcconf);
				mempools_free_mcconf(mcconf_old);
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires one argument.\n");
		}
	} else if (strcmp(argv[0], "measure_res_ind") == 0) {
		mc_configuration *mcconf = mempools_alloc_mcconf();
		*mcconf = *mc_interface_get_configuration();
		mc_configuration *mcconf_old = mempools_alloc_mcconf();
		*mcconf_old = *mc_interface_get_configuration();

		mcconf->motor_type = MOTOR_TYPE_FOC;
		mc_interface_set_configuration(mcconf);

		float res = 0.0;
		float ind = 0.0;
		float ld_lq_diff = 0.0;
		if (mcpwm_foc_measure_res_ind(&res, &ind, &ld_lq_diff)) {
			commands_printf(phandle, "Resistance: %.6f ohm", (double)res);
			commands_printf(phandle, "Inductance: %.2f uH (Lq-Ld: %.2f uH)\n", (double)ind, (double)ld_lq_diff);
		} else {
			commands_printf(phandle, "Error measuring resistance or inductance\n");
		}


		mc_interface_set_configuration(mcconf_old);

		mempools_free_mcconf(mcconf);
		mempools_free_mcconf(mcconf_old);
	} else if (strcmp(argv[0], "measure_linkage_foc") == 0) {
		if (argc == 2) {
			float duty = -1.0;
			sscanf(argv[1], "%f", &duty);

			if (duty > 0.0) {
				mc_configuration *mcconf = mempools_alloc_mcconf();
				*mcconf = *mc_interface_get_configuration();
				mc_configuration *mcconf_old = mempools_alloc_mcconf();
				*mcconf_old = *mc_interface_get_configuration();

				mcconf->motor_type = MOTOR_TYPE_FOC;
				mc_interface_set_configuration(mcconf);

				// Disable timeout
				systime_t tout = timeout_get_timeout_msec();
				float tout_c = timeout_get_brake_current();
				KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
				timeout_reset();
				timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

				for (int i = 0;i < 100;i++) {
					mc_interface_set_duty(((float)i / 100.0) * duty);
					vTaskDelay(MS_TO_TICKS(20));
				}

				float vq_avg = 0.0;
				float rpm_avg = 0.0;
				float samples = 0.0;
				float iq_avg = 0.0;
				for (int i = 0;i < 1000;i++) {
					vq_avg += mcpwm_foc_get_vq();
					rpm_avg += mc_interface_get_rpm();
					iq_avg += mc_interface_get_tot_current_directional();
					samples += 1.0;
					vTaskDelay(MS_TO_TICKS(1));
				}

				mc_interface_release_motor();
				mc_interface_wait_for_motor_release(1.0);
				mc_interface_set_configuration(mcconf_old);

				mempools_free_mcconf(mcconf);
				mempools_free_mcconf(mcconf_old);

				// Enable timeout
				timeout_configure(tout, tout_c, tout_ksw);

				vq_avg /= samples;
				rpm_avg /= samples;
				iq_avg /= samples;

				float linkage = (vq_avg - mcconf->foc_motor_r * iq_avg) / RPM2RADPS_f(rpm_avg);

				commands_printf(phandle, "Flux linkage: %.7f\n", (double)linkage);
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires one argument.\n");
		}
	} else if (strcmp(argv[0], "measure_linkage_openloop") == 0) {
		if (argc == 6) {
			float current = -1.0;
			float duty = -1.0;
			float erpm_per_sec = -1.0;
			float res = -1.0;
			float ind = -1.0;
			sscanf(argv[1], "%f", &current);
			sscanf(argv[2], "%f", &duty);
			sscanf(argv[3], "%f", &erpm_per_sec);
			sscanf(argv[4], "%f", &res);
			sscanf(argv[5], "%f", &ind);

			if (current > 0.0 && current <= mc_interface_get_configuration()->l_current_max &&
					erpm_per_sec > 0.0 && duty > 0.02 && res >= 0.0 && ind >= 0.0) {
				float linkage, linkage_undriven, undriven_samples;
				commands_printf(phandle, "Measuring flux linkage...");
				conf_general_measure_flux_linkage_openloop(current, duty, erpm_per_sec, res, ind,
						&linkage, &linkage_undriven, &undriven_samples);
				commands_printf(phandle,
						"Flux linkage            : %.7f\n"
						"Flux Linkage (undriven) : %.7f\n"
						"Undriven samples        : %.1f\n",
						(double)linkage, (double)linkage_undriven, (double)undriven_samples);
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires five arguments.\n");
		}
	} else if (strcmp(argv[0], "foc_state") == 0) {
		mcpwm_foc_print_state();
		commands_printf(phandle, " ");
	} else if (strcmp(argv[0], "foc_dc_cal") == 0) {
		commands_printf(phandle, "Performing DC offset calibration...");
		int res = mcpwm_foc_dc_cal(true);
		if (res >= 0) {
			conf_general_store_mc_configuration((mc_configuration*)mc_interface_get_configuration(),
					mc_interface_get_motor_thread() == 2);
			commands_printf(phandle, "Done!\n");
		} else {
			commands_printf(phandle, "DC Cal Failed: %d\n", res);
		}
	} else if (strcmp(argv[0], "hw_status") == 0) {
		commands_printf(phandle, "Firmware: %d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR);
#ifdef HW_NAME
		commands_printf(phandle, "Hardware: %s", HW_NAME);
#endif
		commands_printf(phandle, "UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
				STM32_UUID_8[0], STM32_UUID_8[1], STM32_UUID_8[2], STM32_UUID_8[3],
				STM32_UUID_8[4], STM32_UUID_8[5], STM32_UUID_8[6], STM32_UUID_8[7],
				STM32_UUID_8[8], STM32_UUID_8[9], STM32_UUID_8[10], STM32_UUID_8[11]);
		//commands_printf(phandle, "Permanent NRF found: %s", conf_general_permanent_nrf_found ? "Yes" : "No");

		//commands_printf(phandle, "Odometer : %llu m", mc_interface_get_odometer());
		//commands_printf(phandle, "Runtime  : %llu s", g_backup.runtime);

		float curr0_offset;
		float curr1_offset;
		float curr2_offset;

		mcpwm_foc_get_current_offsets(&curr0_offset, &curr1_offset, &curr2_offset,
				mc_interface_get_motor_thread() == 2);

		commands_printf(phandle, "FOC Current Offsets: %.2f %.2f %.2f",
				(double)curr0_offset, (double)curr1_offset, (double)curr2_offset);

		float v0_offset;
		float v1_offset;
		float v2_offset;

		mcpwm_foc_get_voltage_offsets(&v0_offset, &v1_offset, &v2_offset,
				mc_interface_get_motor_thread() == 2);

		commands_printf(phandle, "FOC Voltage Offsets: %.4f %.4f %.4f",
				(double)v0_offset, (double)v1_offset, (double)v2_offset);

		mcpwm_foc_get_voltage_offsets_undriven(&v0_offset, &v1_offset, &v2_offset,
				mc_interface_get_motor_thread() == 2);

		commands_printf(phandle, "FOC Voltage Offsets Undriven: %.4f %.4f %.4f",
				(double)v0_offset, (double)v1_offset, (double)v2_offset);

#ifdef COMM_USE_USB
		commands_printf(phandle, "USB config events: %d", comm_usb_serial_configured_cnt());
		commands_printf(phandle, "USB write timeouts: %u", comm_usb_get_write_timeout_cnt());
#else
		commands_printf(phandle, "USB not enabled on hardware.");
#endif

		commands_printf(phandle, "Mempool mcconf now: %d highest: %d (max %d)",
				mempools_mcconf_allocated_num(), mempools_mcconf_highest(), MEMPOOLS_MCCONF_NUM - 1);
		commands_printf(phandle, "Mempool appconf now: %d highest: %d (max %d)",
				mempools_appconf_allocated_num(), mempools_appconf_highest(), MEMPOOLS_APPCONF_NUM - 1);

		commands_printf(phandle, " ");
	} else if (strcmp(argv[0], "foc_openloop") == 0) {
		if (argc == 3) {
			float current = -1.0;
			float erpm = -1.0;
			sscanf(argv[1], "%f", &current);
			sscanf(argv[2], "%f", &erpm);

			if (current >= 0.0 && erpm >= 0.0) {
				timeout_reset();
				mcpwm_foc_set_openloop(current, erpm);
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires two arguments.\n");
		}
	} else if (strcmp(argv[0], "foc_openloop_duty") == 0) {
		if (argc == 3) {
			float duty = -1.0;
			float erpm = -1.0;
			sscanf(argv[1], "%f", &duty);
			sscanf(argv[2], "%f", &erpm);

			if (duty >= 0.0 && erpm >= 0.0) {
				timeout_reset();
				mcpwm_foc_set_openloop_duty(duty, erpm);
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires two arguments.\n");
		}
	} else if (strcmp(argv[0], "foc_sensors_detect_apply") == 0) {
		if (argc == 2) {
			float current = -1.0;
			sscanf(argv[1], "%f", &current);

			if (current > 0.0 && current <= mc_interface_get_configuration()->l_current_max) {
				int res = conf_general_autodetect_apply_sensors_foc(current, true, true);

				if (res == 0) {
					commands_printf(phandle, "No sensors found, using sensorless mode.\n");
				} else if (res == 1) {
					commands_printf(phandle, "Found hall sensors, using them.\n");
				} else if (res == 2) {
					commands_printf(phandle, "Found AS5047 encoder, using it.\n");
				} else {
					commands_printf(phandle, "Detection error: %d\n", res);
				}
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires one argument.\n");
		}
	} else if (strcmp(argv[0], "rotor_lock_openloop") == 0) {
		if (argc == 4) {
			float current = -1.0;
			float time = -1.0;
			float angle = -1.0;
			sscanf(argv[1], "%f", &current);
			sscanf(argv[2], "%f", &time);
			sscanf(argv[3], "%f", &angle);

			if (fabsf(current) <= mc_interface_get_configuration()->l_current_max &&
					angle >= 0.0 && angle <= 360.0) {
				if (time <= 1e-6) {
					timeout_reset();
					mcpwm_foc_set_openloop_phase(current, angle);
					commands_printf(phandle, "OK\n");
				} else {
					int print_div = 0;
					for (float t = 0.0;t < time;t += 0.002) {
						timeout_reset();
						mcpwm_foc_set_openloop_phase(current, angle);
						vTaskDelay(MS_TO_TICKS(2));

						print_div++;
						if (print_div >= 200) {
							print_div = 0;
							commands_printf(phandle, "T left: %.2f s", (double)(time - t));
						}
					}

					mc_interface_set_current(0.0);

					commands_printf(phandle, "Done\n");
				}
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires three arguments.\n");
		}
	} else if (strcmp(argv[0], "foc_detect_apply_all") == 0) {
		if (argc == 2) {
			float max_power_loss = -1.0;
			sscanf(argv[1], "%f", &max_power_loss);

			if (max_power_loss > 0.0) {
				int motor_thread_old = mc_interface_get_motor_thread();

				commands_printf(phandle, "Running detection...");
				int res = conf_general_detect_apply_all_foc(max_power_loss, true, true);

				commands_printf(phandle, "Res: %d", res);
				mc_interface_select_motor_thread(1);

				if (res >= 0) {
					commands_printf(phandle, "Detection finished and applied. Results:");
					const volatile mc_configuration *mcconf = mc_interface_get_configuration();
#ifdef HW_HAS_DUAL_MOTORS
					commands_printf(phandle, "\nMOTOR 1\n");
#endif
					commands_printf(phandle, "Motor Current       : %.1f A", (double)(mcconf->l_current_max));
					commands_printf(phandle, "Motor R             : %.2f mOhm", (double)(mcconf->foc_motor_r * 1e3));
					commands_printf(phandle, "Motor L             : %.2f uH", (double)(mcconf->foc_motor_l * 1e6));
					commands_printf(phandle, "Motor Flux Linkage  : %.3f mWb", (double)(mcconf->foc_motor_flux_linkage * 1e3));
					commands_printf(phandle, "Temp Comp           : %s", mcconf->foc_temp_comp ? "true" : "false");
					if (mcconf->foc_temp_comp) {
						commands_printf(phandle, "Temp Comp Base Temp : %.1f degC", (double)mcconf->foc_temp_comp_base_temp);
					}

					if (mcconf->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS) {
						commands_printf(phandle, "No sensors found, using sensorless mode.\n");
					} else if (mcconf->foc_sensor_mode == FOC_SENSOR_MODE_HALL) {
						commands_printf(phandle, "Found hall sensors, using them.\n");
					} else if (mcconf->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) {
						commands_printf(phandle, "Found AS5047 encoder, using it.\n");
					} else {
						commands_printf(phandle, "Detection error: %d\n", res);
					}
				} else {
					if (res == -10) {
						commands_printf(phandle, "Could not measure flux linkage.");
					} else if (res == -11) {
						commands_printf(phandle, "Fault code occurred during detection.");
					}

					commands_printf(phandle, "Detection failed.\n");
				}

				mc_interface_select_motor_thread(motor_thread_old);
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires one argument.\n");
		}
	} else if (strcmp(argv[0], "foc_detect_apply_all_can") == 0) {
		if (argc == 2) {
			float max_power_loss = -1.0;
			sscanf(argv[1], "%f", &max_power_loss);

			if (max_power_loss > 0.0) {
				commands_printf(phandle, "Running detection...");
				int res = conf_general_detect_apply_all_foc_can(true, max_power_loss, 0.0, 0.0, 0.0, 0.0, phandle);

				commands_printf(phandle, "Res: %d", res);

				if (res >= 0) {
					commands_printf(phandle, "Detection finished and applied. Results:");
					const volatile mc_configuration *mcconf = mc_interface_get_configuration();
					commands_printf(phandle, "Motor Current       : %.1f A", (double)(mcconf->l_current_max));
					commands_printf(phandle, "Motor R             : %.2f mOhm", (double)(mcconf->foc_motor_r * 1e3));
					commands_printf(phandle, "Motor L             : %.2f microH", (double)(mcconf->foc_motor_l * 1e6));
					commands_printf(phandle, "Motor Flux Linkage  : %.3f mWb", (double)(mcconf->foc_motor_flux_linkage * 1e3));
					commands_printf(phandle, "Temp Comp           : %s", mcconf->foc_temp_comp ? "true" : "false");
					if (mcconf->foc_temp_comp) {
						commands_printf(phandle, "Temp Comp Base Temp : %.1f degC", (double)mcconf->foc_temp_comp_base_temp);
					}

					if (mcconf->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS) {
						commands_printf(phandle, "No sensors found, using sensorless mode.\n");
					} else if (mcconf->foc_sensor_mode == FOC_SENSOR_MODE_HALL) {
						commands_printf(phandle, "Found hall sensors, using them.\n");
					} else if (mcconf->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) {
						commands_printf(phandle, "Found AS5047 encoder, using it.\n");
					} else {
						commands_printf(phandle, "Detection error: %d\n", res);
					}
				} else {
					if (res == -10) {
						commands_printf(phandle, "Could not measure flux linkage.");
					} else if (res == -11) {
						commands_printf(phandle, "Fault code occurred during detection.");
					}

					commands_printf(phandle, "Detection failed.\n");
				}
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires one argument.\n");
		}
	} else if (strcmp(argv[0], "uptime") == 0) {
		commands_printf(phandle, "Uptime: %.2f s\n", (double)xTaskGetTickCount() / (double)configTICK_RATE_HZ);
	} else if (strcmp(argv[0], "hall_analyze") == 0) {
		if (argc == 2) {
			float current = -1.0;
			sscanf(argv[1], "%f", &current);

			if (current > 0.0 && current <= mc_interface_get_configuration()->l_current_max) {
				commands_printf(phandle, "Starting hall sensor analysis...\n");

				mc_interface_lock();
				mc_configuration *mcconf = mempools_alloc_mcconf();
				*mcconf = *mc_interface_get_configuration();
				mc_motor_type motor_type_old = mcconf->motor_type;
				mcconf->motor_type = MOTOR_TYPE_FOC;
				mc_interface_set_configuration(mcconf);

				commands_init_plot("Angle", "Hall Sensor State", phandle);
				commands_plot_add_graph("Hall 1", phandle);
				commands_plot_add_graph("Hall 2", phandle);
				commands_plot_add_graph("Hall 3", phandle);
				commands_plot_add_graph("Combined", phandle);

				float phase = 0.0;

				for (int i = 0;i < 1000;i++) {
					timeout_reset();
					mcpwm_foc_set_openloop_phase((float)i * current / 1000.0, phase);
					vTaskDelay(MS_TO_TICKS(1));
				}

				bool is_second_motor = mc_interface_get_motor_thread() == 2;
				int hall_last = utils_read_hall(is_second_motor, mcconf->m_hall_extra_samples);
				float transitions[7] = {0.0};
				int states[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
				int transition_index = 0;

				for (int i = 0;i < 720;i++) {
					int hall = utils_read_hall(is_second_motor, mcconf->m_hall_extra_samples);
					if (hall_last != hall) {
						if (transition_index < 7) {
							transitions[transition_index++] = phase;
						}

						for (int j = 0;j < 8;j++) {
							if (states[j] == hall || states[j] == -1) {
								states[j] = hall;
								break;
							}
						}
					}
					hall_last = hall;

					// Notice that the plots are offset slightly in Y, to make it easier to see them.
					commands_plot_set_graph(0, phandle);
					commands_send_plot_points(phase, (float)(hall & 1) * 1.02, phandle);
					commands_plot_set_graph(1, phandle);
					commands_send_plot_points(phase, (float)((hall >> 1) & 1) * 1.04, phandle);
					commands_plot_set_graph(2, phandle);
					commands_send_plot_points(phase, (float)((hall >> 2) & 1) * 1.06, phandle);
					commands_plot_set_graph(3, phandle);
					commands_send_plot_points(phase, (float)hall, phandle);

					phase += 1.0;
					timeout_reset();
					mcpwm_foc_set_openloop_phase(current, phase);
					vTaskDelay(MS_TO_TICKS(20));
				}

				mc_interface_lock_override_once();
				mc_interface_release_motor();
				mc_interface_wait_for_motor_release(1.0);
				mcconf->motor_type = motor_type_old;
				mc_interface_set_configuration(mcconf);
				mempools_free_mcconf(mcconf);
				mc_interface_unlock();

				int state_num = 0;
				for (int i = 0;i < 8;i++) {
					if (states[i] != -1) {
						state_num++;
					}
				}

				if (state_num == 6) {
					commands_printf(phandle, "Found 6 different states. This seems correct.\n");
				} else {
					commands_printf(phandle, "Found %d different states. Something is most likely wrong...\n", state_num);
				}

				float min = 900.0;
				float max = 0.0;
				for (int i = 0;i < 6;i++) {
					float diff = fabsf(utils_angle_difference(transitions[i], transitions[i + 1]));
					commands_printf(phandle, "Hall diff %d: %.1f degrees", i + 1, (double)diff);
					if (diff < min) {
						min = diff;
					}
					if (diff > max) {
						max = diff;
					}
				}

				float deviation = (max - min) / 2.0;
				if (deviation < 5) {
					commands_printf(phandle, "Maximum deviation: %.2f degrees. This is good alignment.\n", (double)deviation);
				} else if ((max - min) < 10) {
					commands_printf(phandle, "Maximum deviation: %.2f degrees. This is OK, but not great alignment.\n", (double)deviation);
				} else if ((max - min) < 15) {
					commands_printf(phandle, "Maximum deviation: %.2f degrees. This is bad, but probably usable alignment.\n", (double)deviation);
				} else {
					commands_printf(phandle, "Maximum deviation: %.2f degrees. The hall sensors are significantly misaligned. This has "
							"to be fixed for proper operation.\n", (double)(max - min));
				}

				commands_printf(phandle, "Done. Go to the Realtime Data > Experiment page to see the plot.\n");
			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires one argument.\n");
		}
	} else if (strcmp(argv[0], "crc") == 0) {
		unsigned mc_crc0 = mc_interface_get_configuration()->crc;
		unsigned mc_crc1 = mc_interface_calc_crc(NULL, false);
		unsigned app_crc0 = app_get_configuration()->crc;
		unsigned app_crc1 = app_calc_crc(NULL);
		commands_printf(phandle, "MC CFG crc: 0x%04X (stored)  0x%04X (recalc)", mc_crc0, mc_crc1);
		commands_printf(phandle, "APP CFG crc: 0x%04X (stored)  0x%04X (recalc)", app_crc0, app_crc1);
		commands_printf(phandle, "Discrepancy is expected due to run-time recalculation of config params.\n");
	} else if (strcmp(argv[0], "drv_reset_faults") == 0) {
		HW_RESET_DRV_FAULTS();
	} else if (strcmp(argv[0], "update_pid_pos_offset") == 0) {
		if (argc == 3) {
			float angle_now = -500.0;
			int store = false;

			sscanf(argv[1], "%f", &angle_now);
			sscanf(argv[2], "%d", &store);

			if (angle_now > -360.0 && angle_now < 360.0) {
				mc_interface_update_pid_pos_offset(angle_now, store);
				commands_printf(phandle, "OK\n");
			} else {
				commands_printf(phandle, "Invalid arguments\n");
			}
		}
	} else if (strcmp(argv[0], "fwinfo") == 0) {
		commands_printf(phandle, "GIT Branch: %s", GIT_BRANCH_NAME);
		commands_printf(phandle, "GIT Hash  : %s", GIT_COMMIT_HASH);
		commands_printf(phandle, "Compiler  : %s\n", ARM_GCC_VERSION);
	}

	// The help command
	else if (strcmp(argv[0], "help") == 0) {
		commands_printf(phandle, "Valid commands are:");
		commands_printf(phandle, "help");
		commands_printf(phandle, "  Show this help");

		commands_printf(phandle, "ping");
		commands_printf(phandle, "  Print pong here to see if the reply works");

		commands_printf(phandle, "mem");
		commands_printf(phandle, "  Show memory usage");

		commands_printf(phandle, "threads");
		commands_printf(phandle, "  List all threads");

		commands_printf(phandle, "fault");
		commands_printf(phandle, "  Prints the current fault code");

		commands_printf(phandle, "faults");
		commands_printf(phandle, "  Prints all stored fault codes and conditions when they arrived");

		commands_printf(phandle, "tim");
		commands_printf(phandle, "  Prints tim1 and tim8 settings");

		commands_printf(phandle, "volt");
		commands_printf(phandle, "  Prints different voltages");

		commands_printf(phandle, "measure_res [current]");
		commands_printf(phandle, "  Lock the motor with a current and calculate its resistance");

		commands_printf(phandle, "measure_ind [duty]");
		commands_printf(phandle, "  Send short voltage pulses, measure the current and calculate the motor inductance");

		commands_printf(phandle, "measure_linkage [current] [duty] [min_erpm] [motor_res]");
		commands_printf(phandle, "  Run the motor in BLDC delay mode and measure the flux linkage");
		commands_printf(phandle, "  example measure_linkage 5 0.5 700 0.076");
		commands_printf(phandle, "  tip: measure the resistance with measure_res first");

		commands_printf(phandle, "measure_res_ind");
		commands_printf(phandle, "  Measure the motor resistance and inductance with an incremental adaptive algorithm.");

		commands_printf(phandle, "measure_linkage_foc [duty]");
		commands_printf(phandle, "  Run the motor with FOC and measure the flux linkage.");

		commands_printf(phandle, "measure_linkage_openloop [current] [duty] [erpm_per_sec] [motor_res] [motor_ind]");
		commands_printf(phandle, "  Run the motor in openloop FOC and measure the flux linkage");
		commands_printf(phandle, "  example measure_linkage_openloop 5 0.5 1000 0.076 0.000015");
		commands_printf(phandle, "  tip: measure the resistance with measure_res first");

		commands_printf(phandle, "foc_state");
		commands_printf(phandle, "  Print some FOC state variables.");

		commands_printf(phandle, "foc_dc_cal");
		commands_printf(phandle, "  Calibrate current and voltage DC offsets.");

		commands_printf(phandle, "hw_status");
		commands_printf(phandle, "  Print some hardware status information.");

		commands_printf(phandle, "foc_openloop [current] [erpm]");
		commands_printf(phandle, "  Create an open loop rotating current vector.");

		commands_printf(phandle, "foc_openloop_duty [duty] [erpm]");
		commands_printf(phandle, "  Create an open loop rotating voltage vector.");

		commands_printf(phandle, "foc_sensors_detect_apply [current]");
		commands_printf(phandle, "  Automatically detect FOC sensors, and apply settings on success.");

		commands_printf(phandle, "rotor_lock_openloop [current_A] [time_S] [angle_DEG]");
		commands_printf(phandle, "  Lock the motor with a current for a given time. Time 0 means forever, or");
		commands_printf(phandle, "  or until the heartbeat packets stop.");

		commands_printf(phandle, "foc_detect_apply_all [max_power_loss_W]");
		commands_printf(phandle, "  Detect and apply all motor settings, based on maximum resistive motor power losses.");

		commands_printf(phandle, "foc_detect_apply_all_can [max_power_loss_W]");
		commands_printf(phandle, "  Detect and apply all motor settings, based on maximum resistive motor power losses. Also");
		commands_printf(phandle, "  initiates detection in all VESCs found on the CAN-bus.");

		commands_printf(phandle, "uptime");
		commands_printf(phandle, "  Prints how many seconds have passed since boot.");

		commands_printf(phandle, "hall_analyze [current]");
		commands_printf(phandle, "  Rotate motor in open loop and analyze hall sensors.");

		commands_printf(phandle, "crc");
		commands_printf(phandle, "  Print CRC values.");

		commands_printf(phandle, "update_pid_pos_offset [angle_now] [store]");
		commands_printf(phandle, "  Update position PID offset.");

		commands_printf(phandle, "fwinfo");
		commands_printf(phandle, "  Print detailed firmware info.");

		for (int i = 0;i < callback_write;i++) {
			if (callbacks[i].cbf == 0) {
				continue;
			}

			if (callbacks[i].arg_names) {
				commands_printf(phandle, "%s %s", callbacks[i].command, callbacks[i].arg_names);
			} else {
				commands_printf(phandle, callbacks[i].command);
			}

			if (callbacks[i].help) {
				commands_printf(phandle, "  %s", callbacks[i].help);
			} else {
				commands_printf(phandle, "  There is no help available for this command.");
			}
		}

		commands_printf(phandle, " ");
	} else {
		commands_printf(phandle, "Invalid command: %s\n"
				"type help to list all available commands\n", argv[0]);
	}
}

void terminal_add_fault_data(fault_data *data) {
	fault_vec[fault_vec_write++] = *data;
	if (fault_vec_write >= FAULT_VEC_LEN) {
		fault_vec_write = 0;
	}
}

mc_fault_code terminal_get_first_fault(void) {
	if (fault_vec_write == 0) {
		return FAULT_CODE_NONE;
	} else {
		return fault_vec[0].fault;
	}
}

/**
 * Register a custom command  callback to the terminal. If the command
 * is already registered the old command callback will be replaced.
 *
 * @param command
 * The command name.
 *
 * @param help
 * A help text for the command. Can be NULL.
 *
 * @param arg_names
 * The argument names for the command, e.g. [arg_a] [arg_b]
 * Can be NULL.
 *
 * @param cbf
 * The callback function for the command.
 */
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv)) {

	int callback_num = callback_write;

	for (int i = 0;i < callback_write;i++) {
		// First check the address in case the same callback is registered more than once.
		if (callbacks[i].command == command) {
			callback_num = i;
			break;
		}

		// Check by string comparison.
		if (strcmp(callbacks[i].command, command) == 0) {
			callback_num = i;
			break;
		}

		// Check if the callback is empty (unregistered)
		if (callbacks[i].cbf == 0) {
			callback_num = i;
			break;
		}
	}

	callbacks[callback_num].command = command;
	callbacks[callback_num].help = help;
	callbacks[callback_num].arg_names = arg_names;
	callbacks[callback_num].cbf = cbf;

	if (callback_num == callback_write) {
		callback_write++;
		if (callback_write >= CALLBACK_LEN) {
			callback_write = 0;
		}
	}
}

void terminal_unregister_callback(void(*cbf)(int argc, const char **argv)) {
	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf == cbf) {
			callbacks[i].cbf = 0;
		}
	}
}

