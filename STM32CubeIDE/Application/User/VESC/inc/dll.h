/*
	Copyright 2022 Jens Kerrinnes

	This file is part of the VESC_GD32 firmware.

	The VESC_GD32 firmware is free software: you can redistribute it and/or modify
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

#ifndef DLL_H
#define DLL_H

#include <stdint.h>

#define dll_malloc(x) 	pvPortMalloc(x)
#define dll_free(x) 	vPortFree(x)

typedef struct __Node_t__ Node_t;

struct __Node_t__{
	void * data;
	Node_t* next;
	Node_t* prev;
};

typedef struct{
	Node_t * head;
	Node_t * tail;
	Node_t * last;
}DLL_t;

void dll_init(DLL_t * dll);

//Inserts a Node at head of doubly linked list
Node_t* dll_insert_head(DLL_t * dll, void * data);

//Inserts a Node at tail of Doubly linked list
Node_t* dll_inseert_tail(DLL_t * dll, void * data);

Node_t* dll_get_next(DLL_t * dll);

uint32_t dll_get_count(DLL_t * dll);

void dll_to_start(DLL_t * dll);

void dll_remove_item(DLL_t * dll, Node_t * item);

#endif /* APPLICATION_USER_VESC_INC_DLL_H_ */
