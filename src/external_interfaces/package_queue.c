
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "package_queue.h"
#include "bt_uart_interrupt.h"

typedef struct package_info {
	uint16_t pkg_pos;
	uint16_t pkg_number;
	uint8_t pkg_size;
} package_info;

#define LOG2_PACKAGE_BUFFER_SIZE 9
#define PACKAGE_BUFFER_SIZE (1<<LOG2_PACKAGE_BUFFER_SIZE)
#define BUFFER_SIZE_MASK (PACKAGE_BUFFER_SIZE-1)
#define MAX_NR_PACKAGES (PACKAGE_BUFFER_SIZE>>2)
#define PACKAGE_INDEX_MASK (MAX_NR_PACKAGES-1)
// TODO: Below should have a relation to min_log_divider and the sampling frequency
#define LOG2_RESEND_DELAY 7
#define RESEND_DELAY (1<<LOG2_RESEND_DELAY)
const package_info NO_PACKAGE = {0};

uint8_t pkg_buffer[PACKAGE_BUFFER_SIZE];
package_info pkg_info[MAX_NR_PACKAGES];
uint8_t oldest_pkg = 0;
uint8_t newest_pkg = 0;

uint32_t pkg_delay_counter = 0;

void add_package_to_queue(uint8_t* package,uint8_t package_size,uint16_t package_number) {
	uint32_t start = ( pkg_info[newest_pkg].pkg_pos + pkg_info[newest_pkg].pkg_size) & BUFFER_SIZE_MASK;
	volatile uint32_t space_in_buffer = (pkg_info[oldest_pkg].pkg_pos - start) & BUFFER_SIZE_MASK;
	if(pkg_info[oldest_pkg].pkg_pos==pkg_info[newest_pkg].pkg_pos || space_in_buffer >= package_size){
		newest_pkg=(newest_pkg+1) & PACKAGE_INDEX_MASK;
		int i;
		for (i=0;i<package_size;i++) pkg_buffer[(start + i) & BUFFER_SIZE_MASK] = package[i];
		pkg_info[newest_pkg].pkg_pos = start;
		pkg_info[newest_pkg].pkg_number = package_number;
		pkg_info[newest_pkg].pkg_size = package_size;
		if (pkg_info[oldest_pkg].pkg_size==0) {
			oldest_pkg=newest_pkg;
			pkg_delay_counter = 0;
		}
	}
}

void send_package_from_queue(void) {
	if (pkg_info[oldest_pkg].pkg_size && pkg_delay_counter==0) {
		if ( pkg_info[oldest_pkg].pkg_size <= space_in_bt_uart_buf() ) {
			bt_send_buf(pkg_buffer+pkg_info[oldest_pkg].pkg_pos,pkg_info[oldest_pkg].pkg_size);
		}
	}
	pkg_delay_counter = (pkg_delay_counter+1) & (RESEND_DELAY-1);
}

void remove_package_from_queue(uint16_t package_nr) {
	if (pkg_info[oldest_pkg].pkg_number==package_nr) {
		pkg_info[oldest_pkg] = NO_PACKAGE;
		oldest_pkg=(oldest_pkg+1) & PACKAGE_INDEX_MASK;
		if (oldest_pkg==newest_pkg) {
			newest_pkg = oldest_pkg;
		}
		pkg_delay_counter = 0;
	}
}

void empty_package_queue(void){
	int i;
	for (i=0;i<MAX_NR_PACKAGES;i++) {
		pkg_info[i] = NO_PACKAGE;
	}
	oldest_pkg = newest_pkg = 0;
}