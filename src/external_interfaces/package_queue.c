﻿
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "package_queue.h"
#include "bt_uart_interrupt.h"
#include "compiler.h"
#include <string.h>

typedef struct package_info {
	int pos;
	uint16_t number;
	int size;
	uint8_t flag;
} package_info;

#define LOG2_PACKAGE_BUFFER_SIZE 9
#define PKG_BUFFER_SIZE (1<<LOG2_PACKAGE_BUFFER_SIZE)
#define BUFFER_SIZE_MASK (PKG_BUFFER_SIZE-1)
#define MAX_NR_PACKAGES (PKG_BUFFER_SIZE>>2)
#define PACKAGE_INDEX_MASK (MAX_NR_PACKAGES-1)
// TODO: Below should have a relation to min_log_divider and the sampling frequency
#define LOG2_RESEND_DELAY 7
#define RESEND_DELAY (1<<LOG2_RESEND_DELAY)
#define RESEND_DELAY_MASK (RESEND_DELAY-1)
const package_info NO_PACKAGE = {0};

uint8_t pkg_buffer[PKG_BUFFER_SIZE*2];
package_info pkg_info[MAX_NR_PACKAGES];
uint8_t oldest_pkg = 0;
uint8_t newest_pkg = 0;

uint32_t pkg_delay_counter = 0;

static inline void remove_oldest_package(void){
	pkg_info[oldest_pkg] = NO_PACKAGE;
	if(oldest_pkg!=newest_pkg)
	oldest_pkg=(oldest_pkg+1) & PACKAGE_INDEX_MASK;
	pkg_delay_counter = 0;
}

static inline bool is_buffer_nonempty(void){
	return pkg_info[oldest_pkg].size>0;
}

void add_package_to_queue(uint8_t* package,int package_size,uint16_t package_number,uint8_t flag) {
	uint32_t start = ( pkg_info[newest_pkg].pos + pkg_info[newest_pkg].size) & BUFFER_SIZE_MASK;
	uint32_t space_in_buffer = (pkg_info[oldest_pkg].pos - start) & BUFFER_SIZE_MASK;
	if((pkg_info[oldest_pkg].size==0 || space_in_buffer >= package_size) && package_size) {
		if(is_buffer_nonempty())
		newest_pkg=(newest_pkg+1) & PACKAGE_INDEX_MASK;
		memcpy(pkg_buffer+start,package,package_size);
		memcpy(pkg_buffer+start+PKG_BUFFER_SIZE,package,min(package_size,PKG_BUFFER_SIZE-start));
		memcpy(pkg_buffer,package+PKG_BUFFER_SIZE-start,package_size- min(package_size,PKG_BUFFER_SIZE-start) );
		pkg_info[newest_pkg].pos = start;
		pkg_info[newest_pkg].number = package_number;
		pkg_info[newest_pkg].size = package_size;
		pkg_info[newest_pkg].flag = flag;
	}
}

void send_and_remove_data_from_queue(void){
	if (is_buffer_nonempty()) {
		// Try to send everything in the buffer
		int nob = ((pkg_info[newest_pkg].pos - pkg_info[oldest_pkg].pos) & BUFFER_SIZE_MASK) + pkg_info[newest_pkg].size;
		nob -= bt_send_buf(pkg_buffer+pkg_info[oldest_pkg].pos, nob);
		// Circle through packages and remove those which have been sent
		while (nob && pkg_info[oldest_pkg].size<=nob) {
			nob -= pkg_info[oldest_pkg].size;
			remove_oldest_package();
		}
		// Decrease size of last one
		if (nob) {
			pkg_info[oldest_pkg].size -=nob;
			pkg_info[oldest_pkg].pos = (pkg_info[oldest_pkg].pos+nob) & BUFFER_SIZE_MASK;
			pkg_info[oldest_pkg].flag |= MODIFIED_PACKAGE;
		}
	}
}

void send_and_remove_package_from_queue(void) {
	if (is_buffer_nonempty())
		if(!bt_send_buf_allornothing(pkg_buffer+pkg_info[oldest_pkg].pos,pkg_info[oldest_pkg].size))
			remove_oldest_package();
}

void send_package_from_queue(void) {
	if (is_buffer_nonempty() && (pkg_delay_counter==0 || pkg_info[oldest_pkg].flag & SINGLE_TRANSMIT))
		if(!bt_send_buf_allornothing(pkg_buffer+pkg_info[oldest_pkg].pos,pkg_info[oldest_pkg].size)){
			pkg_delay_counter = (pkg_delay_counter+1) & RESEND_DELAY_MASK;
			if (pkg_info[oldest_pkg].flag & SINGLE_TRANSMIT)
				remove_oldest_package();
	}
}

void remove_package_from_queue(uint16_t package_nr) {
	if (pkg_info[oldest_pkg].number==package_nr)
		remove_oldest_package();
}

void empty_package_queue(void){
	while(is_buffer_nonempty())
		remove_oldest_package();
	oldest_pkg = newest_pkg = 0;
}
