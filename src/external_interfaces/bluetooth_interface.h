/*
 * bluetooth_interface.h
 *
 * Created: 2014-03-18 10:11:01
 *  Author: jnil02
 */ 


#ifndef BLUETOOTH_INTERFACE_H_
#define BLUETOOTH_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

#define COMMAND_FROM_BT 2


void bt_interface_init(void);
void bt_receive_command(void);
void bt_transmit_data(void);

void bt_set_state_output(uint8_t state_id, uint8_t divider);
void bt_reset_output_counters(void);
void bt_set_conditional_output(uint8_t state_id);
void bt_set_lossy_transmission(bool onoff);

#endif /* BLUETOOTH_INTERFACE_H_ */