
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef USB_PACKAGE_QUEUE_H_
#define USB_PACKAGE_QUEUE_H_

#include <stdint.h>

#define SINGLE_TRANSMIT 1
#define MODIFIED_PACKAGE 2

void usb_add_package_to_queue(uint8_t* package,int package_size,uint16_t package_number,uint8_t flag);
void usb_send_and_remove_data_from_queue(void);
void usb_send_and_remove_package_from_queue(void);
void usb_send_package_from_queue(void);
void usb_remove_package_from_queue(uint16_t package_nr);
void usb_empty_package_queue(void);


#endif /* USB_PACKAGE_QUEUE_H_ */