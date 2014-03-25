
/** \file
	\brief High level external user interface (USB) header file.
	
	\details 
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

/**
	\ingroup openshoe_runtime_framework
	
	\defgroup user_interface User (USB) interface	
	\brief This group contains high level functions for communication between user and system.
	@{
*/

#ifndef EXTERNAL_INTERFACE_H_
#define EXTERNAL_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

void external_interface_init(void);
void transmit_data(void);
void receive_command(void);

void set_state_output(uint8_t state_id, uint8_t divider,uint8_t from);
void reset_output_counters(uint8_t from);
void set_conditional_output(uint8_t state_id,uint8_t from);
void set_lossy_transmission(bool onoff,uint8_t from);

#endif /* EXTERNAL_INTERFACE_H_ */

//@}