
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "package_queue.h"
#include "bt_uart_interrupt.h"
#include "compiler.h"
#include <string.h>

#define BUFFER_SIZE_MASK (PKG_BUFFER_SIZE-1)
#define PACKAGE_INDEX_MASK (MAX_NR_PACKAGES-1)
// TODO: Below should have a relation to min_log_divider and the sampling frequency
#define LOG2_RESEND_DELAY 7
#define RESEND_DELAY (1<<LOG2_RESEND_DELAY)
#define RESEND_DELAY_MASK (RESEND_DELAY-1)
static const package_info NO_PACKAGE = {0};

static inline void remove_oldest_package(pkg_queue* queue){
	queue->pkg_info[queue->oldest_pkg] = NO_PACKAGE;
	if(queue->oldest_pkg!=queue->newest_pkg)
	queue->oldest_pkg=(queue->oldest_pkg+1) & PACKAGE_INDEX_MASK;
	queue->pkg_delay_counter = 0;
}

static inline bool is_buffer_nonempty(pkg_queue* queue){
	return queue->pkg_info[queue->oldest_pkg].size>0;
}

void add_package_to_queue(const uint8_t* package,int package_size,uint16_t package_number,uint8_t flag,pkg_queue* queue) {
	uint32_t start = ( queue->pkg_info[queue->newest_pkg].pos + queue->pkg_info[queue->newest_pkg].size) & BUFFER_SIZE_MASK;
	uint32_t space_in_buffer = (queue->pkg_info[queue->oldest_pkg].pos - start) & BUFFER_SIZE_MASK;
	if((queue->pkg_info[queue->oldest_pkg].size==0 || space_in_buffer >= package_size) && package_size) {
		if(is_buffer_nonempty(queue))
		queue->newest_pkg=(queue->newest_pkg+1) & PACKAGE_INDEX_MASK;
		memcpy(queue->pkg_buffer+start,package,package_size);
		memcpy(queue->pkg_buffer+start+PKG_BUFFER_SIZE,package,min(package_size,PKG_BUFFER_SIZE-start));
		memcpy(queue->pkg_buffer,package+PKG_BUFFER_SIZE-start,package_size- min(package_size,PKG_BUFFER_SIZE-start) );
		queue->pkg_info[queue->newest_pkg].pos = start;
		queue->pkg_info[queue->newest_pkg].number = package_number;
		queue->pkg_info[queue->newest_pkg].size = package_size;
		queue->pkg_info[queue->newest_pkg].flag = flag;
	}
}

void send_and_remove_data_from_queue(pkg_queue* queue){
	if (is_buffer_nonempty(queue)) {
		// Try to send everything in the buffer
		int nob = ((queue->pkg_info[queue->newest_pkg].pos - queue->pkg_info[queue->oldest_pkg].pos) & BUFFER_SIZE_MASK) + queue->pkg_info[queue->newest_pkg].size;
		nob -= queue->send_buf(queue->pkg_buffer+queue->pkg_info[queue->oldest_pkg].pos, nob);
		// Circle through packages and remove those which have been sent
		while (nob && queue->pkg_info[queue->oldest_pkg].size<=nob) {
			nob -= queue->pkg_info[queue->oldest_pkg].size;
			remove_oldest_package(queue);
		}
		// Decrease size of last one
		if (nob) {
			queue->pkg_info[queue->oldest_pkg].size -=nob;
			queue->pkg_info[queue->oldest_pkg].pos = (queue->pkg_info[queue->oldest_pkg].pos+nob) & BUFFER_SIZE_MASK;
			queue->pkg_info[queue->oldest_pkg].flag |= MODIFIED_PACKAGE;
		}
	}
}

void send_and_remove_package_from_queue(pkg_queue* queue) {
	if (is_buffer_nonempty(queue))
		if(!queue->send_buf_allornothing(queue->pkg_buffer+queue->pkg_info[queue->oldest_pkg].pos,queue->pkg_info[queue->oldest_pkg].size))
			remove_oldest_package(queue);
}

void send_package_from_queue(pkg_queue* queue) {
	if (is_buffer_nonempty(queue)){
		if (queue->pkg_delay_counter==0 || queue->pkg_info[queue->oldest_pkg].flag & SINGLE_TRANSMIT){ // TODO: is latter requirement really necessary. In this case, delay counter is 0 anyway?
			if(!queue->send_buf_allornothing(queue->pkg_buffer+queue->pkg_info[queue->oldest_pkg].pos,queue->pkg_info[queue->oldest_pkg].size)){
				queue->pkg_delay_counter = (queue->pkg_delay_counter+1) & RESEND_DELAY_MASK;
				if (queue->pkg_info[queue->oldest_pkg].flag & SINGLE_TRANSMIT)
					remove_oldest_package(queue);
			}
		} else {
			queue->pkg_delay_counter = (queue->pkg_delay_counter+1) & RESEND_DELAY_MASK;
		}
	}
}

void remove_pkg_from_queue(uint16_t package_nr,pkg_queue* queue) {
	if (queue->pkg_info[queue->oldest_pkg].number==package_nr)
		remove_oldest_package(queue);
}

void empty_pkg_queue(pkg_queue* queue){
	while(is_buffer_nonempty(queue))
		remove_oldest_package(queue);
	queue->oldest_pkg = queue->newest_pkg = 0;
}
