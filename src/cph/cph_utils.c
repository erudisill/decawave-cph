/*
 * cph_utils.c
 *
 *  Created on: Dec 14, 2015
 *      Author: ericrudisill
 */

#include <cph.h>

uint16_t cph_utils_get_shortid_candidate(void) {
    uint32_t seed = 0;
    seed ^= *((volatile uint32_t*)0x0080A00C);
    seed ^= *((volatile uint32_t*)0x0080A040);
    seed ^= *((volatile uint32_t*)0x0080A044);
    seed ^= *((volatile uint32_t*)0x0080A048);
//    TRACE("seed: %08X\r\n", seed);
    uint16_t shortid = 0;
	srand(seed);
	uint8_t select = (uint8_t)(seed & 0xFF);
	for (int i=0;i<select;i++) {
		shortid += rand() % 0xFFFF;
	}
//    TRACE("shortid: %04X\r\n", shortid);

    return shortid;
}
