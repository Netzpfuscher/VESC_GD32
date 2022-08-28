/*
 * command_cust.c
 *
 *  Created on: 25.08.2022
 *      Author: jensk
 */

#include "command_cust.h"
#include "ninebot.h"
#include "mc_interface.h"
#include "math.h"
#include "app.h"
#include "shutdown.h"

void camilo_btn_callback(eButtonEvent btn);

uint8_t command_cust_mode=4;
uint8_t	command_cust_light=0;
uint8_t	command_cust_beep=0;


void command_cust_callback(PACKET_STATE_t * phandle, uint8_t * data, int packet_id){

	switch(packet_id){
		case 200: {  //From Dash
			int32_t ind = 0;
			float ad1 = data[ind++];
			float ad2 = data[ind++];
			app_adc_detach_adc(true);
			shutdown_set_callback(camilo_btn_callback);
			app_adc_adc1_override(1.0/255.0*ad1);
			app_adc_adc2_override(1.0/255.0*ad2);
		} break;
		case 201: {	//To Dash
			int32_t ind = 0;
			uint8_t send_buffer[PACKET_SIZE(20)];
			uint8_t * buffer = send_buffer + PACKET_HEADER;
			buffer[ind++] = packet_id;
			buffer[ind++] = command_cust_mode;
			buffer[ind++] = floor(mc_interface_get_battery_level(NULL)*100.0);
			buffer[ind++] = command_cust_light;	//Light
			buffer[ind++] = command_cust_beep;	//Beep
			buffer[ind++] = ceilf(mc_interface_get_speed() * 3.6);
			buffer[ind++] = mc_interface_get_fault();
			packet_send_packet(send_buffer, ind, phandle);
			command_cust_beep=0;
		} break;
	}

}

void recalc_speed(float speed){
	mc_configuration *mcconf = (mc_configuration*)mc_interface_get_configuration();
	const float speed_fact = ((mcconf->si_motor_poles / 2.0) * 60.0 * mcconf->si_gear_ratio) / (mcconf->si_wheel_diameter * M_PI);
	mcconf->l_max_erpm = speed * speed_fact / 3.6;

	conf_general_mcconf_hw_limits(mcconf);

}


void camilo_btn_callback(eButtonEvent btn){
	command_cust_beep=1;
	if(btn==SINGLE_PRESS){
		command_cust_light = command_cust_light ? 0 : 1;
	}
	if(btn==DOUBLE_PRESS){
		switch(command_cust_mode){
			case M365_MODE_DRIVE:
				recalc_speed(255);
				command_cust_mode = M365_MODE_SPORT;
				break;
			case M365_MODE_SPORT:
				recalc_speed(6);
				command_cust_mode = M365_MODE_SLOW;
				break;
			case M365_MODE_SLOW:
				recalc_speed(25);
				command_cust_mode = M365_MODE_DRIVE;
				break;
		  }
	}
}
