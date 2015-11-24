/*
 * cph_millis.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_CPH_MILLIS_H_
#define SRC_CPH_CPH_MILLIS_H_

#include <cph.h>

extern volatile uint32_t g_cph_millis;

#define cph_get_millis()	g_cph_millis

void cph_millis_init(void);
void cph_millis_delay(uint32_t millis);


#endif /* SRC_CPH_CPH_MILLIS_H_ */
