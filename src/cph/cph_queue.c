/*
 * cph_queue.c
 *
 *  Created on: Dec 3, 2015
 *      Author: ericrudisill
 */

#include <cph_queue.h>
#include <stdint.h>
#include <string.h>

void cph_queue_init(cph_queue_info_t * info, int item_size, int max_items, void * mem) {
	info->head = 0;
	info->tail = 0;
	info->count = 0;
	info->item_size = item_size;
	info->max_items = max_items;
	info->mem = mem;
}

void cph_queue_clear(cph_queue_info_t * q) {
	q->head = 0;
	q->tail = 0;
	q->count = 0;
}

int cph_queue_peek(cph_queue_info_t *q, void ** item) {

	// Bail if the queue is empty
	if (q->count == 0)
		return CPH_QUEUE_EMPTY;

	*item = q->mem + (q->item_size * q->head);

	return CPH_QUEUE_OK;
}

int cph_queue_push(cph_queue_info_t * q, void * item) {

	// Bail if the queue is full
	if (q->count == q->max_items)
		return CPH_QUEUE_FULL;

	// Copy the item into the queue
	void * to = q->mem + (q->item_size * q->tail);
	memcpy(to, item, q->item_size);

	// Bump the count
	q->count++;

	// Wrap the tail .. If we became full, tail will == head
	q->tail++;
	if (q->tail == q->max_items)
		q->tail = (q->tail % q->max_items);

	return CPH_QUEUE_OK;
}

int cph_queue_pop(cph_queue_info_t * q, void * item) {

	// Bail if queue is empty
	if (q->count == 0)
		return CPH_QUEUE_EMPTY;

	// Copy the item from the queue to the destination
	void * from = q->mem + (q->item_size * q->head);
	memcpy(item, from, q->item_size);

	// Dec the count
	q->count--;

	// Wrap the head .. If nothing left, head will == tail
	q->head++;
	if (q->head == q->max_items)
		q->head = (q->head % q->max_items);

	return CPH_QUEUE_OK;
}


/*
void cph_queue_test(void) {
	uint8_t item[] = "TEST0\0";
	uint8_t	data[5*5];
	int result;
	cph_queue_info_t q;

	cph_queue_init(&q, 5, 5, data);

	for (int i=0;i<5;i++) {
		item[4] = '0' + i;
		printf("push: %s\r\n", item);
		cph_queue_push(&q, item);
	}

	printf("data: %s\r\n", data);

	for (int i=0;i<3;i++) {
		cph_queue_pop(&q, item);
		printf("pop : %s\r\n", item);
	}

	printf("data: %s\r\n", data);

	item[4] = 'A';
	cph_queue_push(&q, item);
	item[4] = 'B';
	cph_queue_push(&q, item);
	item[4] = 'C';
	result = cph_queue_push(&q, item);
	item[4] = 'D';
	result = cph_queue_push(&q, item);

	printf("result: %d\r\n", result);
	printf("data: %s\r\n", data);

	for (int i=0;i<5;i++) {
		cph_queue_pop(&q, item);
		printf("pop : %s\r\n", item);
	}

	result = cph_queue_pop(&q, item);
	printf("result: %d\r\n", result);
	printf("count: %d\r\n", q.count);
}
*/

