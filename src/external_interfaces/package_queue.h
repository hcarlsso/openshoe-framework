/*
 * package_queue.h
 *
 * Created: 2014-03-25 20:18:59
 *  Author: jnil02
 */ 


#ifndef PACKAGE_QUEUE_H_
#define PACKAGE_QUEUE_H_

#include <stdint.h>

void add_package_to_queue(uint8_t* package,uint8_t package_size,uint16_t package_number);
void send_package_from_queue(void);
void remove_package_from_queue(uint16_t package_nr);


#endif /* PACKAGE_QUEUE_H_ */