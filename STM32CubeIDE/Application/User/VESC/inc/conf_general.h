/*
	Copyright 2017 - 2021 Benjamin Vedder	benjamin@vedder.se

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

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include "datatypes.h"
#include "mcconf_default.h"
#include "appconf_default.h"
#include "packet.h"

#define CODE_IND_QML	0
#define CODE_IND_LISP	1
#define FLASH_COMPLETE  1

#define ADDR_FLASH_PAGE_126    ((uint32_t)0x08000000+(APP_PAGE*PAGE_SIZE))
#define ADDR_FLASH_PAGE_127    ((uint32_t)0x08000000+(CONF_PAGE*PAGE_SIZE))

//extern mc_configuration mc_conf;
extern app_configuration appconf;
extern bool conf_general_permanent_nrf_found;

// Functions
void conf_general_init(void);
void conf_general_read_app_configuration(app_configuration *conf);
void conf_general_read_mc_configuration(mc_configuration *conf, bool is_motor_2);
bool conf_general_store_mc_configuration(mc_configuration *conf, bool is_motor_2);
void conf_update_override_current(mc_configuration *mcconf);
void conf_general_setup_mc(mc_configuration *mcconf);
void conf_general_update_current(mc_configuration *mcconf);
//mc_configuration* mc_interface_get_configuration(void);
bool conf_general_store_app_configuration(app_configuration *conf);
void conf_general_mcconf_hw_limits(mc_configuration *mcconf);
int conf_general_detect_apply_all_foc_can(bool detect_can, float max_power_loss, float min_current_in, float max_current_in, float openloop_rpm, float sl_erpm, PACKET_STATE_t * phandle);
bool conf_general_measure_flux_linkage_openloop(float current, float duty,float erpm_per_sec, float res, float ind, float *linkage,float *linkage_undriven, float *undriven_samples);

uint16_t conf_general_write_code(int ind, uint32_t offset, uint8_t *data, uint32_t len);
uint16_t conf_general_erase_code(int ind);
uint8_t* conf_general_code_data(int ind);
uint32_t conf_general_code_size(int ind);

bool conf_general_erase_flash(uint32_t addr, uint32_t pages);
bool conf_general_write_fl(uint32_t base, uint8_t * data, uint16_t size);

#endif /* CONF_GENERAL_H_ */

