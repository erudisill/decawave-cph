/*
 * cph_config.c
 *
 *  Created on: Dec 14, 2015
 *      Author: ericrudisill
 */

#include <cph.h>

static uint8_t config_buffer[NVMCTRL_PAGE_SIZE];
static struct nvm_config config_nvm;

void * cph_config_init(void) {
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
	return cph_config_read();
}

void * cph_config_read(void) {
	enum status_code error_code;
	uint32_t address;
	address = ((NVMCTRL_PAGES * NVMCTRL_ROW_PAGES) - 1) * NVMCTRL_PAGE_SIZE;
	do {
		error_code = nvm_read_buffer(address, config_buffer, NVMCTRL_PAGE_SIZE);
	} while(error_code == STATUS_BUSY);
	return (void*)config_buffer;
}

void cph_config_write(void) {
	enum status_code error_code;
	uint32_t address;
	// ERASE
	address = ((NVMCTRL_PAGES * NVMCTRL_ROW_PAGES) - NVMCTRL_ROW_PAGES) * NVMCTRL_PAGE_SIZE;
	do {
		error_code = nvm_erase_row(address);
	} while(error_code == STATUS_BUSY);
	address = ((NVMCTRL_PAGES * NVMCTRL_ROW_PAGES) - 1) * NVMCTRL_PAGE_SIZE;
	do {
		error_code = nvm_write_buffer(address, config_buffer, NVMCTRL_PAGE_SIZE);
	} while(error_code == STATUS_BUSY);
}

//static void read_config(void) {
//	struct nvm_config config_nvm;
//	struct nvm_parameters parameters_nvm;
//	enum status_code error_code;
//
//	nvm_get_config_defaults(&config_nvm);
//	config_nvm.manual_page_write = false;
//	nvm_set_config(&config_nvm);
//
//	nvm_get_parameters(&parameters_nvm);
//	printf("flash: %d pages, %d bytes per page\r\n", parameters_nvm.nvm_number_of_pages, parameters_nvm.page_size);
//
//	uint32_t address = (parameters_nvm.nvm_number_of_pages - 1) * NVMCTRL_PAGE_SIZE;
//
//	// READ
//	do {
//		error_code = nvm_read_buffer(address, flash_buffer, NVMCTRL_PAGE_SIZE);
//	} while(error_code == STATUS_BUSY);
//	printf("flash @%08X: ", address);
//	for (int i=0;i<NVMCTRL_PAGE_SIZE;i++) {
//		printf("%02X ", flash_buffer[i]);
//	}
//	printf("\r\n");
//
//	// ERASE
//	address = (parameters_nvm.nvm_number_of_pages - NVMCTRL_ROW_PAGES) * NVMCTRL_PAGE_SIZE;
//	do {
//		error_code = nvm_erase_row(address);
//	} while(error_code == STATUS_BUSY);
//
//
//	// READ
//	address = (parameters_nvm.nvm_number_of_pages - 1) * NVMCTRL_PAGE_SIZE;
//	do {
//		error_code = nvm_read_buffer(address, flash_buffer, NVMCTRL_PAGE_SIZE);
//	} while(error_code == STATUS_BUSY);
//	printf("flash @%08X: ", address);
//	for (int i=0;i<NVMCTRL_PAGE_SIZE;i++) {
//		printf("%02X ", flash_buffer[i]);
//	}
//	printf("\r\n");
//
//	// WRITE
//	address = (parameters_nvm.nvm_number_of_pages - 1) * NVMCTRL_PAGE_SIZE;
//	flash_buffer[0] = 'C';
//	flash_buffer[1] = 'P';
//	flash_buffer[2] = 'H';
//	flash_buffer[3] = 'T';
//	do {
//		error_code = nvm_write_buffer(address, flash_buffer, NVMCTRL_PAGE_SIZE);
//	} while(error_code == STATUS_BUSY);
//
//	// READ
//	address = (parameters_nvm.nvm_number_of_pages - 1) * NVMCTRL_PAGE_SIZE;
//	do {
//		error_code = nvm_read_buffer(address, flash_buffer, NVMCTRL_PAGE_SIZE);
//	} while(error_code == STATUS_BUSY);
//	printf("flash @%08X: ", address);
//	for (int i=0;i<NVMCTRL_PAGE_SIZE;i++) {
//		printf("%02X ", flash_buffer[i]);
//	}
//	printf("\r\n");
//
//}
