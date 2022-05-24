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

#ifndef VESCCOMMAND_H_
#define VESCCOMMAND_H_

#include "datatypes.h"
#include "product.h"
#include "packet.h"

void commands_send_mcconf(COMM_PACKET_ID packet_id, mc_configuration *mcconf, PACKET_STATE_t * phandle);
void commands_process_packet(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle);
void commands_send_packet(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle);
void send_sample();
void send_position(PACKET_STATE_t * phandle);
void commands_printf(PACKET_STATE_t * phandle, const char* format, ...);
void commands_send_appconf(COMM_PACKET_ID packet_id, app_configuration *appconf, PACKET_STATE_t * phandle);

void commands_init_plot(char *namex, char *namey, PACKET_STATE_t * phandle);
void commands_plot_add_graph(char *name, PACKET_STATE_t * phandle);
void commands_plot_set_graph(int graph, PACKET_STATE_t * phandle);
void commands_send_plot_points(float x, float y, PACKET_STATE_t * phandle);

typedef enum {
	SAMP_IDLE,
	SAMP_START,
	SAMP_SAMPLING,
	SAMP_FINISHED,
	SAMP_SENDING
}SAMP_STATES;



typedef struct samp_struct samp_str;
struct samp_struct{
	PACKET_STATE_t * phandle;
	SAMP_STATES state;
	uint8_t dec;
	uint8_t dec_state;
	uint16_t index;
	uint16_t n_samp;
	uint16_t vesc_tool_samples;
	debug_sampling_mode mode;
	uint16_t * m_curr0_samples; //4 bits LSB Hall Pos 12bit curr0
	uint16_t * m_curr1_samples; //4 bits LSB Hall Pos 12bit curr1
#if SCOPE_UVW == 1
	uint8_t * m_v0_samples;
	uint8_t * m_v1_samples;
	uint8_t * m_v2_samples;
#endif
};

extern volatile samp_str samples;

#endif
