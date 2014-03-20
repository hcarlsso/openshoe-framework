/*
 * usb_interface.h
 *
 * Created: 2014-03-18 10:10:47
 *  Author: jnil02
 */ 


#ifndef USB_INTERFACE_H_
#define USB_INTERFACE_H_

#include <stdint.h>

#define COMMAND_FROM_USB 1


void usb_interface_init(void);
void usb_transmit_data(void);
void usb_receive_command(void);

void usb_set_state_output(uint8_t state_id, uint8_t divider);
void usb_reset_output_counters(void);
void usb_set_conditional_output(uint8_t state_id);


#include <stdbool.h>
// USB vbus callback function
///\cond
void vbus_event_callback(bool b_high);
///\endcond

#endif /* USB_INTERFACE_H_ */