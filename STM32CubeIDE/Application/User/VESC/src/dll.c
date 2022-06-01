/*
 * dll.c
 *
 *  Created on: 01.06.2022
 *      Author: jensk
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
	Node_t* newNode = pvPortMalloc(sizeof(Node_t));
	if(dll->head == NULL){
		dll->head = newNode;
		dll->tail = newNode;
		dll->last = newNode;
	}
	newNode->data = data;
	newNode->next = dll->head;
	newNode->prev = dll->tail;
	dll->head->prev = newNode;
	dll->head = newNode;
	dll->tail->next = newNode;
	return newNode;
}

//Inserts a Node at tail of Doubly linked list
Node_t* dll_insert_tail(DLL_t * dll, void * data) {
	Node_t * newNode = pvPortMalloc(sizeof(Node_t));
	newNode->data = data;
	if(dll->head == NULL) {
		dll->head = newNode;
		dll->tail = newNode;
		dll->last = newNode;
	}
	newNode->prev = dll->tail;
	dll->tail->next = newNode;
	dll->tail = newNode;
	dll->tail->next = dll->head;
	dll->head->prev = dll->tail;
	return newNode;
}

Node_t* dll_get_next(DLL_t * dll){
	if(dll->last == NULL) return NULL;
	Node_t * next = dll->last->next;
	dll->last = next;
	return next;
}
