/*
 * command_cust.h
 *
 *  Created on: 25.08.2022
 *      Author: jensk
 */

#ifndef APPLICATION_USER_VESC_INC_COMMAND_CUST_H_
#define APPLICATION_USER_VESC_INC_COMMAND_CUST_H_

#include "packet.h"

void command_cust_callback(PACKET_STATE_t * phandle, uint8_t * data, int packet_id);



#endif /* APPLICATION_USER_VESC_INC_COMMAND_CUST_H_ */
