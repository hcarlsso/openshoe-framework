
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef USB_INTERFACE_H_
#define USB_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

void usb_interface_init(void);

bool is_usb_attached(void);
bool is_data_available(void);
uint8_t get_byte_from_usb(uint8_t* write_position);
uint32_t usb_write_buf_nonblocking(const uint8_t* buf, uint32_t size);
uint32_t usb_write_buf_nonblocking_allornothing(const uint8_t* buf, uint32_t size);

#include <stdbool.h>
// USB vbus callback function
///\cond
void vbus_event_callback(bool b_high);
///\endcond

#endif /* USB_INTERFACE_H_ */