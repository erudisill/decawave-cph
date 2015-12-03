/*
 * cph_queue.h
 *
 *  Created on: Dec 3, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_CPH_QUEUE_H_
#define SRC_CPH_CPH_QUEUE_H_


enum {
	CPH_QUEUE_OK = 0,
	CPH_QUEUE_FULL,
	CPH_QUEUE_EMPTY
};

typedef struct {
	int head;
	int tail;
	int count;
	int item_size;
	int max_items;
	void * mem;
} cph_queue_info_t;


void cph_queue_init(cph_queue_info_t * info, int item_size, int max_items, void * mem);
void cph_queue_clear(cph_queue_info_t * q);
int cph_queue_peek(cph_queue_info_t *q, void ** item);
int cph_queue_push(cph_queue_info_t * q, void * item);
int cph_queue_pop(cph_queue_info_t * q, void * item);

#endif /* SRC_CPH_CPH_QUEUE_H_ */
