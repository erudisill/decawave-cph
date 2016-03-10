/*
 * cph_stdio.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_CPH_STDIO_H_
#define SRC_CPH_CPH_STDIO_H_

#include <cph.h>

struct usart_module * cph_stdio_init(void);
void cph_stdio_set_rx_callback(usart_callback_t callback_func);


#endif /* SRC_CPH_CPH_STDIO_H_ */
