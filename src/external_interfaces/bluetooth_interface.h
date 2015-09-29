
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef BLUETOOTH_INTERFACE_H_
#define BLUETOOTH_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

void bt_interface_init(void);
bool is_bluetooth_paired(void);

uint32_t bt_send_buf(const uint8_t* buf,uint32_t nob);
uint32_t bt_send_buf_allornothing(const uint8_t* buf,uint32_t nob);
bool bt_is_data_available(void);
uint8_t bt_get_byte(uint8_t* dest);


#endif /* BLUETOOTH_INTERFACE_H_ */