
/** \file
	\brief High level external user interface (USB).
	
	\details This file contains the functions for 1) receiving and parsing
	commands and executing commands responses 2) determining what to transmit
	and to transmit data to the system user via USB.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

///	\addtogroup user_interface
///	@{

// Needed for memcpy
#include <string.h>
#include "usb_interface.h"
#include "parsing_util.h"

//#include "conf_usb.h"
#include "udd.h"
#include "udc.h"
#include "udi_cdc.h"
#include "usbc_device.h"

/// Initialization function for communication interface
void usb_interface_init(void){
	// Start usb controller	
	udc_start();
}

bool is_usb_attached(void) {
	return !Is_udd_detached();
}

bool is_data_available(void) {
	return udi_cdc_is_rx_ready();
}

void get_byte_from_usb(uint8_t* write_position) {
	*write_position = udi_cdc_getc();
}

uint32_t usb_write_buf_nonblocking(const uint8_t* buf, uint32_t size) {
	return udi_cdc_write_buf_nonblocking(buf,size);
}

uint32_t usb_write_buf_nonblocking_allornothing(const uint8_t* buf, uint32_t size) {
	return udi_cdc_write_buf_nonblocking_allornothing(buf,size);
}

// Callback function for usb-vbus event (see conf_usb.h)
///\cond
void vbus_event_callback(bool b_high){
	b_high ? udc_attach() : udc_detach();}
///\endcond
	
// @}