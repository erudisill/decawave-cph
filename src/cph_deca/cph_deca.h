/*
 * cph_deca.h
 *
 *  Created on: Dec 2, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_DECA_CPH_DECA_H_
#define SRC_CPH_DECA_CPH_DECA_H_

#include <deca_device_api.h>

#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // MP counter is 40 bits


void cph_deca_config_dw(void);
void cph_deca_init_dw(void (*txcallback)(const dwt_callback_data_t *), void (*rxcallback)(const dwt_callback_data_t *));
void cph_deca_print_status(uint32_t count);

void cph_deca_init(void);
void cph_deca_run(void);


#endif /* SRC_CPH_DECA_CPH_DECA_H_ */
