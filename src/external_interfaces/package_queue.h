
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef PACKAGE_QUEUE_H_
#define PACKAGE_QUEUE_H_

#include <stdint.h>

#define SINGLE_TRANSMIT 1
#define MODIFIED_PACKAGE 2

typedef struct package_info {
	uint16_t pos;
	uint16_t number;
	uint16_t size;
	uint8_t flag;
} package_info;

#define LOG2_PACKAGE_BUFFER_SIZE 11
#define PKG_BUFFER_SIZE (1<<LOG2_PACKAGE_BUFFER_SIZE)
#define MAX_NR_PACKAGES (PKG_BUFFER_SIZE>>2)

typedef struct pkg_queue {
	uint8_t pkg_buffer[PKG_BUFFER_SIZE*2];
	package_info pkg_info[MAX_NR_PACKAGES];
	uint8_t oldest_pkg;
	uint8_t newest_pkg;
	uint32_t pkg_delay_counter;
	uint32_t (*send_buf)(uint8_t*,uint32_t);
	uint32_t (*send_buf_allornothing)(uint8_t*,uint32_t);
} pkg_queue;

void add_package_to_queue(const uint8_t* package,int package_size,uint16_t package_number,uint8_t flag,pkg_queue* queue);
void send_and_remove_data_from_queue(pkg_queue* queue);
void send_and_remove_package_from_queue(pkg_queue* queue);
void send_package_from_queue(pkg_queue* queue);
void remove_pkg_from_queue(uint16_t package_nr,pkg_queue* queue);
void empty_pkg_queue(pkg_queue* queue);

#endif /* PACKAGE_QUEUE_H_ */