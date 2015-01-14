
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef USB_INTERFACE_H_
#define USB_INTERFACE_H_

#include <stdint.h>

#define COMMAND_FROM_USB 1


void usb_interface_init(void);
void usb_transmit_data(void);
void usb_receive_command(void);

void usb_set_state_output(uint8_t state_id, uint8_t divider);
void usb_set_conditional_output(uint8_t state_id);


#include <stdbool.h>
// USB vbus callback function
///\cond
void vbus_event_callback(bool b_high);
///\endcond

#endif /* USB_INTERFACE_H_ */