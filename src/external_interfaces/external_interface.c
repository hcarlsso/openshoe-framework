
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "external_interface.h"
#include "usb_interface.h"
#include "bluetooth_interface.h"
#include "package_queue.h"
#include "usb_package_queue.h"

#include <board.h>

#include "control_tables.h"


void external_interface_init(void){
	
	// TODO: These initialization functions should be replaced by a code generating script/preprocessors
	// They only initialize constant arrays.
	commands_init();	
	system_states_init();
	processing_functions_init();
	
	usb_interface_init();
	
	#ifdef BT_MODULE
	bt_interface_init();
	#endif
}

void transmit_data(void){
	#ifdef BT_MODULE
	bt_transmit_data();
	#endif
	
	usb_transmit_data();
}

void receive_command(void){
	usb_receive_command();
	
	#ifdef BT_MODULE
	bt_receive_command();
	#endif
}

void set_state_output(uint8_t state_id, uint8_t divider,uint8_t from){
	if (from & COMMAND_FROM_USB) usb_set_state_output(state_id,divider);
	
	#ifdef BT_MODULE
	if (from & COMMAND_FROM_BT) bt_set_state_output(state_id,divider);
	#endif
	
}
void set_conditional_output(uint8_t state_id,uint8_t from){
	if (from & COMMAND_FROM_USB) usb_set_conditional_output(state_id);
	
	#ifdef BT_MODULE
	if (from & COMMAND_FROM_BT) bt_set_conditional_output(state_id);
	#endif
}
void set_lossless_transmission(bool onoff,uint8_t from){
	if (from & COMMAND_FROM_USB)
		usb_set_lossless_transmission(onoff);
	
	#ifdef BT_MODULE
	if (from & COMMAND_FROM_BT)
		bt_set_lossless_transmission(onoff);
	#endif
}

void receive_package_ack(uint16_t package_number, uint8_t from){
	if (from & COMMAND_FROM_USB)
		usb_remove_package_from_queue(package_number);
	
	#ifdef BT_MODULE
	if (from & COMMAND_FROM_BT)
		remove_package_from_queue(package_number);
	#endif	
}
