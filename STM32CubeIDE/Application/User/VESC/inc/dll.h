/*
 * dll.h
 *
 *  Created on: 01.06.2022
 *      Author: jensk
 */

#ifndef DLL_H
#define DLL_H

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

#endif /* APPLICATION_USER_VESC_INC_DLL_H_ */
