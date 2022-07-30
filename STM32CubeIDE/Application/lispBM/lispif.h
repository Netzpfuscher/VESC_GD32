/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Joel Svensson    svenssonjoel@yahoo.se

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

#ifndef LISPBM_LISPIF_H_
#define LISPBM_LISPIF_H_

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

// Functions
void lispif_init(void);
bool lispif_restart(bool print, bool load_code);
void lispif_disable_all_events(void);
void lispif_stop_lib(void);
void lispif_process_cmd(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle);
void lispif_process_can(uint32_t can_id, uint8_t *data8, int len, bool is_ext);
void lispif_process_custom_app_data(unsigned char *data, unsigned int len);
void lispif_set_ext_load_callback(void (*p_func)(void));

void lispif_load_vesc_extensions(void);
bool lispif_vesc_dynamic_loader(const char *str, const char **code);

#endif /* LISPBM_LISPIF_H_ */
