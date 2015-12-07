/*
 * tag.h
 *
 *  Created on: Dec 7, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_TAG_H_
#define SRC_TAG_H_


#define TAG_BURST_COUNT 	3
#define TAG_EVENTS_MAX		10
#define TAG_BURST_DELAY		250
// 100ms 32-bit
#define TAG_SEND_DELAY		0x17CDC00

void tag_init(void);


#endif /* SRC_TAG_H_ */
