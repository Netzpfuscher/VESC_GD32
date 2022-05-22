/****************************************************************************/
//  Function: C file ninebot communication
//  Author:   Camilo Ruiz
//  Date:    october 10 2017
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
// I am not responsible of any damage caused by the misuse of this library
// use at your own risk
//
// If you modify this or use this, please don't delete my name and give me the credits
// Greetings from Colombia :) 
// Hardware port by ub4raf
/****************************************************************************/
#include "ninebot.h"
#include "m365_register_map.h"

enum{
	NIN_IDLE, NIN_HEAD, NIN_LEN, NIN_ADDR, NIN_CMD, NIN_ARG, NIN_DATA, NIN_CRC
};


uint16_t ninebot_parse(uint8_t data , NinebotPack *message){
	static uint8_t state = NIN_IDLE;
	static uint8_t cnt = 0;
	static uint16_t checksum=0;
	switch (state){
	case NIN_IDLE:
		if(data==NinebotHeader0) state = NIN_HEAD;
		break;
	case NIN_HEAD:
		if(data==NinebotHeader1) state = NIN_LEN;
		break;
	case NIN_LEN:
		message->len = data-1;
		checksum=0;
		checksum=checksum + data;
		state = NIN_ADDR;
		break;
	case NIN_ADDR:
		cnt++;
		state = NIN_CMD;
		checksum=checksum + data;
		message->addr = data;
		break;
	case NIN_CMD:
		cnt=0;
		state = NIN_ARG;
		checksum=checksum + data;
		message->cmd = data;
		break;
	case NIN_ARG:
		cnt++;
		state = NIN_DATA;
		checksum=checksum + data;
		message->arg = data;
		break;
	case NIN_DATA:
		cnt++;
		checksum=checksum + data;
		if((cnt-2) < NinebotMaxPayload){
			message->payload[cnt-2] = data;
		}
		if(cnt == (message->len)){
			state = NIN_CRC;
			cnt=0;
		}
		break;
	case NIN_CRC:
		message->CheckSum[cnt] = data;
		if(cnt==1){
			checksum ^= 0xFFFF;
			state = NIN_IDLE;
			return checksum - ((message->CheckSum[1]<<8) + message->CheckSum[0]);
		}
		cnt++;
		break;
	}

return 1;
}

void addCRC(uint8_t * message, uint8_t size){
    unsigned long cksm = 0;
    for(int i = 2; i < size - 2; i++) cksm += message[i];
    cksm ^= 0xFFFF;
    message[size - 2] = (uint8_t)(cksm&0xFF);
    message[size - 1] = (uint8_t)((cksm&0xFF00) >> 8);
}
