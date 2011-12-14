
/** \file
	\brief High level external user interface (USB) header file.
	
	\details 
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/ 

#ifndef EXTERNAL_INTERFACE_H_
#define EXTERNAL_INTERFACE_H_

#include "conf_usb.h"
#include "udd.h"
#include "udc.h"
#include "udi_cdc.h"
#include "usbc_device.h"

#include "process_sequence.h"
#include "nav_types.h"

void com_interface_init(void);

void transmit_data(void);
void receive_command(void);


void set_state_output(uint8_t state_id, uint8_t divider);

/// USB vbus callback function
void vbus_event_callback(bool b_high);

#endif /* EXTERNAL_INTERFACE_H_ */