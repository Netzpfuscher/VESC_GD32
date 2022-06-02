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

#include "dll.h"
#include "FreeRTOS.h"

//Creates a new Node and returns pointer to it.
void dll_init(DLL_t * dll) {
	dll->head = NULL;
	dll->tail = NULL;
	dll->last = NULL;
}

//Inserts a Node at head of doubly linked list
Node_t* dll_insert_head(DLL_t * dll, void * data) {
	Node_t* newNode = dll_malloc(sizeof(Node_t));
	newNode->prev = NULL;
	newNode->next = NULL;
	newNode->data = data;
	if(dll->head == NULL){
		dll->head = newNode;
		dll->tail = newNode;
		dll->last = newNode;
		return newNode;
	}
	newNode->next = dll->head;
	dll->head->prev = newNode;
	dll->head = newNode;
	return newNode;
}

//Inserts a Node at tail of Doubly linked list
Node_t* dll_insert_tail(DLL_t * dll, void * data) {
	Node_t * newNode = dll_malloc(sizeof(Node_t));
	newNode->prev = NULL;
	newNode->next = NULL;
	newNode->data = data;
	if(dll->head == NULL) {
		dll->head = newNode;
		dll->tail = newNode;
		dll->last = newNode;
		return newNode;
	}
	newNode->prev = dll->tail;
	dll->tail->next = newNode;
	dll->tail = newNode;
	return newNode;
}

Node_t* dll_get_next(DLL_t * dll){
	if(dll->last == NULL) return NULL;
	Node_t * last = dll->last;
	if(dll->last == dll->tail){
		dll->last = dll->head;
	}else{
		dll->last = dll->last->next;
	}
	return last;
}

Node_t* dll_get_prev(DLL_t * dll){
	if(dll->last == NULL) return NULL;
	Node_t * last = dll->last;
	if(dll->last == dll->head){
		dll->last = dll->tail;
	}else{
		dll->last = dll->last->prev;
	}
	return last;
}

void dll_remove_item(DLL_t * dll, Node_t * item){
	if(item == dll->tail){
		Node_t * prev = item->prev;
		prev->next = NULL;
		dll->tail = prev;
		if(dll->last == item) dll->last = dll->tail;
		dll_free(item);
		return;
	}else if(item == dll->head){
		Node_t * next = item->next;
		next->prev = NULL;
		dll->head = next;
		if(dll->last == item) dll->last = dll->head;
		dll_free(item);
		return;
	}
	Node_t * next = item->next;
	Node_t * prev = item->prev;
	prev->next = next;
	next->prev = prev;
	dll_free(item);
}

void dll_pop(DLL_t * dll){
	dll_remove_item(dll, dll->tail);
}

void dll_to_start(DLL_t * dll){
	dll->last = dll->head;
}

uint32_t dll_get_count(DLL_t * dll){
	if(dll->last == NULL) return 0;
	Node_t * last = dll->head;
	uint32_t cnt=1;
	while(last->next){
		cnt++;
		last = last->next;
	}
	return cnt;
}
